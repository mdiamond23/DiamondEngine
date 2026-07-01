# Vulkan port — Particles (step 4) + main.cpp `--vulkan` wiring (step 5)

A step-by-step implementation guide. Written against the code as of the Renderer2D
port completion (quads + textures + text all working). Part 1 = particle billboard
renderer. Part 2 = wiring the Vulkan backend into the editor via a `--vulkan` flag,
including the deferred UISystem pass (3b-ii).

The golden rule from the whole port: **one small, independently-verifiable chunk at
a time.** Build + smoke-run (zero validation errors) after each numbered step.

---

## PART 1 — Particle system port (step 4)

`VulkanParticleRenderer` is almost a line-for-line sibling of `VulkanRenderer2D`.
Same multi-batch machinery (accumulate all verts once, one dynamic buffer, per-texture
descriptor set, draw per texture run with `firstVertex`). Differences: 3D world-space
verts, a view/proj instead of an ortho, two blend modes, and depth-test against the
scene. Reuse `VulkanRenderer2D.cpp` as your template.

### Step 1.1 — Add `RHIBlendMode::Additive` to the RHI (small, foundational)

Particles need additive blending; the enum only has `Opaque`/`Alpha`.

- `Engine/include/Renderer/RHI/RHIEnums.h`: add `Additive` to `enum class RHIBlendMode`.
- `Engine/src/Platform/Vulkan/RHI/VulkanRHIResources.cpp` (~line 275, the blend
  attachment if/else): add a branch for `Additive`:
  ```cpp
  } else if (desc.blendMode == RHIBlendMode::Additive) {
      blendAttachment.blendEnable         = VK_TRUE;
      blendAttachment.srcColorBlendFactor = VK_BLEND_FACTOR_SRC_ALPHA;
      blendAttachment.dstColorBlendFactor = VK_BLEND_FACTOR_ONE;
      blendAttachment.colorBlendOp        = VK_BLEND_OP_ADD;
      blendAttachment.srcAlphaBlendFactor = VK_BLEND_FACTOR_ONE;
      blendAttachment.dstAlphaBlendFactor = VK_BLEND_FACTOR_ONE;
      blendAttachment.alphaBlendOp        = VK_BLEND_OP_ADD;
  } else { /* Opaque — existing blendEnable = FALSE */ }
  ```
- **Verify:** build the engine. No behavior change yet (nothing uses Additive).

### Step 1.2 — The two shaders

Create `Assets/Shaders/Vulkan/particle.vert` and `particle.frag` (ports of the inline
GL shaders in `OpenGLParticleRenderer.cpp`). Add both to the glslang `foreach` list in
`Engine/CMakeLists.txt` (same place `renderer2d.vert renderer2d.frag` were added).

`particle.vert` — verts are already world-space (billboards expanded on the CPU), so the
vertex shader just applies viewProj (push-constant, mirroring renderer2d.vert):
```glsl
#version 450
layout(location = 0) in vec3 aPos;
layout(location = 1) in vec2 aUV;
layout(location = 2) in vec4 aColor;
layout(push_constant) uniform Push { mat4 uViewProj; } pc;
layout(location = 0) out vec2 vUV;
layout(location = 1) out vec4 vColor;
void main() {
    vUV = aUV; vColor = aColor;
    gl_Position = pc.uViewProj * vec4(aPos, 1.0);
}
```
`particle.frag`:
```glsl
#version 450
layout(location = 0) in vec2 vUV;
layout(location = 1) in vec4 vColor;
layout(set = 0, binding = 0) uniform sampler2D uTex;
layout(location = 0) out vec4 FragColor;
void main() { FragColor = vColor * texture(uTex, vUV); }
```

### Step 1.3 — `VulkanParticleRenderer` (`Engine/src/Platform/Vulkan/Resources/`)

Copy `VulkanRenderer2D.{h,cpp}` → `VulkanParticleRenderer.{h,cpp}` and adapt:

- Implements `ParticleRenderer` (`Begin(view,proj)/Draw(particles,texture,blend)/End`)
  plus the Vulkan-only `SetCommandList(RHICommandList*)`.
- **Vertex** = `{ glm::vec3 pos; glm::vec2 uv; glm::vec4 color; }` (stride 36, attributes
  Float3/Float2/Float4). No `mode` field.
- **Two pipelines** — build one with `blendMode = Alpha`, one with `Additive`. Both:
  `depthFormat = RHIFormat::Depth32F` (matches the graph's `gDepth`), `depthTest = true`,
  `depthWrite = false`, `depthCompare = Less`, `cullMode = None`, push-constant
  `{ Vertex, sizeof(mat4) }`, one `CombinedImageSampler@0 Fragment`.
- **Begin(view, proj):** store `m_ViewProj = proj * view`; pull the camera basis exactly
  like the GL version — `m_CamRight = vec3(view[0][0],view[1][0],view[2][0])`,
  `m_CamUp = vec3(view[0][1],view[1][1],view[2][1])`. Clear verts/batches.
- **Draw(particles, texture, blend):** `dynamic_cast<const VulkanTexture2D*>(&texture)`
  → `Rhi()`; expand each particle into 6 world-space verts (copy the billboard math from
  `OpenGLParticleRenderer::Draw` verbatim — rotation via `cos/sin`, the `bl/br/tr/tl`
  corners, the same top-left-origin UVs). Push into `m_Verts`. **Blend is part of the
  batch key**, so a `Batch` here is `{ RHITexture*, RHIBlendMode(blend), first, count }` —
  extend `SetTexture` to also break the run when the blend mode changes, and pick the
  matching pipeline per batch in `End()`.
- **End():** identical to Renderer2D — close the final run, one `m_VertexBuffer->Update`,
  then per batch: `BindPipeline(alpha or additive)`, `PushConstants(Vertex,0,
  sizeof(mat4), &m_ViewProj)`, `BindResourceSet(0, GetSet(tex))`, `BindVertexBuffer`,
  `Draw(count,1,first)`.
- Per-texture `GetSet` cache — identical to Renderer2D.
- Add `VulkanParticleRenderer.cpp` to the gated Vulkan sources in `Engine/CMakeLists.txt`.

### Step 1.4 — Wire the particle pass into `SceneRenderer` (the one tricky bit)

In `Engine/src/Renderer/SceneRenderer.cpp`:

1. **Members:** `std::unique_ptr<VulkanParticleRenderer> m_Particles;` a default particle
   texture `std::unique_ptr<RHITexture>` (a soft radial dot — or reuse a 1x1 white for
   now), and a per-frame scratch list of batches:
   `struct PBatch { RHITexture* tex; ParticleBlend blend; std::vector<RenderParticle> parts; };`
   `std::vector<PBatch> m_ParticleBatches;`. Also need the particle **texture** as a
   `VulkanTexture2D` so `Draw` can `dynamic_cast` it — keep a `std::shared_ptr<Texture>`.
2. **BuildResources():** construct `m_Particles` with the shader dir. Create the default
   particle texture (via `Texture::CreateFromPixels` once `SetResourceDevice` is set, or a
   small generated `VulkanTexture2D`).
3. **BuildGraph():** insert the pass **between DeferredLighting and Tonemap** so insertion
   order (the Kahn tiebreaker) places it there:
   ```cpp
   graph.AddPass("Particles")
       .Write(hdrLit)     // composite over the lit scene (color)
       .Write(gDepth)     // depth attachment for read-only depth-test
       .Load()            // LOAD both — don't clear the scene / its depth
       .SetExecute([this](RHICommandList* cmd) {
           m_Particles->SetCommandList(cmd);
           m_Particles->Begin(m_FrameView, m_FrameProj);
           for (auto& b : m_ParticleBatches)
               m_Particles->Draw(b.parts, *b.texAsset, b.blend);
           m_Particles->End();
       });
   ```
   Store `m_FrameView/m_FrameProj` as members set in `RenderToSwapchain` (the pass runs
   inside `graph.Execute`, after you've computed them).
4. **RenderToSwapchain():** before `BeginFrame`, gather emitters from the scene into
   `m_ParticleBatches` (mirror `main.cpp`'s Particles pass: iterate
   `view<ParticleEmitterComponent>()`, copy `em.pool[0..liveCount]` → `RenderParticle`,
   pick `em.texture` or the default). Then set `m_FrameView/m_FrameProj` = the `view`/`proj`
   already computed there.

**⚠ The risk to verify (depth + write-ordering):**
- The Vulkan graph opens `BeginRendering` from a pass's **writes**; you can't hand-bind a
  target like the GL pass does. So particles must `.Write(hdrLit).Write(gDepth).Load()`.
- Ordering: two passes now write `hdrLit` (DeferredLighting then Particles) with no reader
  between them. The graph orders readers-after-writers; pure write-after-write relies on
  the Kahn **insertion-order tiebreak**. Because you add Particles after Lighting and before
  Tonemap (which *reads* hdrLit → pins Tonemap last), it should slot in correctly. **If a
  frame shows particles missing or the scene cleared**, the graph isn't ordering the two
  writers — fallback: give the pass a real read edge on a lighting output it doesn't sample
  (harmless barrier) or teach the graph a write-after-write dependency.
- `gDepth` is `Depth32F`, already `DepthAttachment|Sampled`. `.Write(gDepth)` + pipeline
  `depthWrite=false` = read-only depth test. `.Load()` keeps the G-buffer depth.

### Step 1.5 — Verify particles
- Add a `ParticleEmitterComponent` to the demo scene in `VulkanSceneDemo.cpp` (or a couple),
  or hand-fill one `PBatch` with a few `RenderParticle`s to prove the pipeline first.
- Build + smoke-run: **zero validation errors**, particles depth-occlude behind the cube,
  additive sparks brighten. Play-verify.

---

## PART 2 — main.cpp `--vulkan` wiring + UISystem pass (step 5 / 3b-ii)

`Sandbox/src/main.cpp` (1173 lines) is deeply GL-coupled: it builds the GL `RenderGraph`
inline (~14 passes), uses a GL FBO for the editor viewport, GL thumbnails, GL
Renderer2D/particles. Goal: a `--vulkan` flag that runs the editor through `SceneRenderer`
instead, GL staying the default. Do this in slices; each builds + runs.

### Step 2.1 — Backend flag + window creation (small)
- Parse `--vulkan` from `argv` early (before window creation). Store a `bool useVulkan`.
- GLFW window hint: Vulkan needs `glfwWindowHint(GLFW_CLIENT_API, GLFW_NO_API)`; GL keeps
  its current hints. macOS: force `useVulkan = false` (GL-only there).
- **Verify:** GL path still launches unchanged when the flag is absent.

### Step 2.2 — Branch the renderer construction (medium)
The cleanest structure: keep the entire existing GL path in an `if (!useVulkan) { … }` and
add a parallel Vulkan path. The Vulkan path:
- `auto device = RHIDevice::Create(window, RHIBackend::Vulkan);`
- `Texture::SetResourceDevice(device.get());` (fonts/UI/particle textures become Vulkan).
- `auto sceneRenderer = SceneRenderer::Create(device.get(), fbW, fbH);`
- `VulkanImGuiLayer imgui; imgui.Init(window, static_cast<VulkanRHIDevice*>(device.get()));`
- Register meshes with `sceneRenderer->RegisterMesh(...)` as scene meshes load. **Watch the
  StubMesh problem**: `Mesh::Create` issues GL calls; under Vulkan the editor must not build
  GL meshes. This is the biggest refactor — the asset/mesh path assumes GL. For a first
  bring-up, load a fixed scene through the same StubMesh + `RegisterMesh` bridge the demo
  uses, and defer the full asset-pipeline-under-Vulkan.
- **Verify:** Vulkan editor window opens, ImGui draws, scene renders to the swapchain
  (no viewport-in-panel yet — full-window is fine for this slice).

### Step 2.3 — Viewport-as-texture (medium)
The editor draws the scene into a docked ImGui panel, not the whole window. So
`SceneRenderer` needs to render into an **offscreen color target** you can sample in ImGui:
- Add to `SceneRenderer`: render the deferred chain with Tonemap writing to an `hdrLit`-style
  LDR **output texture** (declare an `outputColor` graph texture; Tonemap's `AddToGraph`
  already supports writing to a texture instead of the swapchain — pass the handle).
- Expose `RHITexture* OutputColor()` + a `Resize(w,h)` (rebuild the graph textures at the new
  panel size).
- In main.cpp: `ImTextureID id = ImGui_ImplVulkan_AddTexture(sampler, view, layout)` for the
  output image, `ImGui::Image(id, panelSize)`. (This needs the raw `VkImageView` + a sampler
  from the RHI texture — add a small accessor on the Vulkan RHI texture, mirroring how
  `VulkanImGuiLayer` reaches into the device.)
- Handle panel resize → `SceneRenderer::Resize`.
- **Verify:** scene shows inside the viewport panel; docking works; resize doesn't crash.

### Step 2.4 — UISystem as its own LDR graph pass (3b-ii)
Now the deferred Renderer2D factory + UI wiring:
- `Engine/src/Renderer/Renderer2D.cpp`: add the Vulkan case to `Renderer2D::Create()`. Same
  device problem as Texture — either register a device (like `Texture::SetResourceDevice`)
  or construct `VulkanRenderer2D` directly in main.cpp and pass it to `UIRenderSystem::Render`.
  Direct construction is simplest and matches the demo.
- Add a UI pass to `SceneRenderer`'s graph **after Tonemap** (LDR), writing the swapchain (or
  the output texture) with `.Load()`. Its execute callback: `r2d->SetCommandList(cmd);` then
  `UIRenderSystem::Render(scene.GetRegistry(), *r2d, {fbW,fbH})` (which internally does
  `Begin/DrawWidget…/End`). This is the "UI = LDR pass after tonemap" decision realized.
- **Verify:** in-game UI widgets (Image/Text/ProgressBar/Button) render over the scene under
  Vulkan.

### Step 2.5 — Particles in the real scene
Particles are already a `SceneRenderer` pass (Part 1). Confirm the editor's play-mode
emitters show up under Vulkan; the simulation (`ParticleSystem`) is renderer-agnostic and
needs no changes.

---

## What is still NOT ported after Parts 1 & 2 (the parity tail — later, together)
- **Per-material PBR textures / MaterialCache / TBN / full 6-tex gbuffer** (Slice 3) — the
  biggest *visual* gap; currently one shared checkerboard albedo. **Do this next, together.**
- Skybox / background environment render (GL had `DrawSkybox`).
- Point/spot shadows (cube maps) + spot lights in the deferred shader.
- Frustum culling under Vulkan `[0,1]` depth (`Frustum::Extract` is GL-NDC).
- Forward/transparency passes (SceneRenderer is deferred-only).
- **Editor thumbnails** (`MeshThumbnailRenderer` is pure GL → needs an RHI offscreen preview
  renderer + `ImGui_ImplVulkan_AddTexture`). Content-browser polish; defer.

## Build/verify reflexes (every step)
- `cmake --build build --target VulkanScene` (or `Sandbox`) — expect a clean link.
- Smoke-run 6s, grep for `error|validation|VUID|not freed` — expect **zero**.
- Destruction order: every RHI resource (buffers/textures/sets, i.e. anything VMA-backed)
  must be reset **before** the `RHIDevice`. New owners (particle renderer, r2d, output
  texture) reset before `device.reset()`; `Texture::SetResourceDevice(nullptr)` at shutdown.
