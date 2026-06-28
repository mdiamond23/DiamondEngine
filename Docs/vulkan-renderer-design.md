# Vulkan Renderer Design

Status: **DESIGN** — no code yet. This doc precedes any porting work.

## Goal

Add a Vulkan rendering backend alongside the existing OpenGL one, eventually
reaching feature parity with the current deferred PBR pipeline (G-buffer, SSAO,
shadows/CSM, IBL, transparency, particles, bloom, tonemap, FXAA, 2D/UI/text,
ImGui editor). OpenGL is retained as a still-buildable backend but is **frozen**
for new features once the RHI lands. Multithreaded command recording is
explicitly **out of scope for v1** — we reach single-threaded parity first, then
parallelize.

## Locked decisions

| Decision | Choice | Rationale |
|----------|--------|-----------|
| Function loading | **volk** | Per-instance/device function pointers, mirrors how GLAD loads GL. Avoids the loader trampoline overhead. |
| Memory allocation | **Vulkan Memory Allocator (VMA)** | Sub-allocation, defrag, budget tracking. We still own *lifetime* (deletion queue); VMA only owns *allocation*. |
| Windowing | **GLFW** (unchanged) | `glfwCreateWindowSurface`, `glfwGetRequiredInstanceExtensions`. No `GLFW_NO_API` for the GL window; Vulkan window is created with `GLFW_NO_API`. |
| Abstraction seam | **Thin RHI, shared passes** | Passes are written **once** against a Render Hardware Interface (RHI). No per-backend pass duplication. |
| Shader pipeline | **GLSL → SPIR-V now, Slang later** | Reach PBR parity compiling existing GLSL to SPIR-V (shaderc/glslang). Migrate to Slang as a *separate* phase so Vulkan bugs and shader-language bugs never entangle. |
| macOS | **Stays on OpenGL** | No MoltenVK. Lets us target a clean **Vulkan 1.3** baseline (dynamic rendering, synchronization2, timeline semaphores) on Windows/Linux without portability-subset caveats. |
| Texture/model loading | **Unchanged** | stb_image, cgltf/assimp, skinning all stay. They produce CPU-side data; only the GPU upload path changes (becomes an RHI buffer/texture create + staging copy). |
| First milestone | **Device up + clear screen** | volk + VMA + surface + swapchain + frames-in-flight + validation, clearing to a color. Proves the foundation before geometry. |

## Why the current code does *not* "just port"

Three things in the existing renderer are OpenGL-shaped at the *implementation*
level, not just the orchestration level. These are the real work:

1. **`RenderGraph` is pure GL.** [Engine/include/Renderer/RenderGraph.h](../Engine/include/Renderer/RenderGraph.h)
   `#include <glad/gl.h>` and is built on `GLenum internalFormat`, `glTexture`,
   `glFBO`, `glDepthRBO`. The graph is also *constructed* in
   [Sandbox/src/main.cpp](../Sandbox/src/main.cpp) (~L598-811) using `GL_RGBA16F`
   literals and concrete `OpenGL*Pass` objects.
   **Good news:** `RGPass` already declares per-pass `reads`/`writes` — exactly
   the dependency data a Vulkan graph needs to insert image-layout transitions
   and pipeline barriers. The *design* survives; the *code* is rewritten over the
   RHI, and graph construction moves out of `main.cpp` into the engine.

2. **Shader uniforms have no Vulkan equivalent.**
   [Engine/include/Renderer/Shader.h](../Engine/include/Renderer/Shader.h) is
   `SetMat4("name", ...)` — i.e. `glUniform`-by-name. Vulkan has no such call.
   This becomes descriptor sets + uniform/storage buffers + push constants. This
   is a redesign of how materials/passes feed data to shaders, not a port. See
   "Shader & binding model" below.

3. **Transient resources are freed every `Execute()`** — illegal in Vulkan,
   where a resource referenced by an in-flight command buffer cannot be freed.
   Requires frames-in-flight + a deletion queue. VMA does not solve this.

## Surface area (don't forget these)

Parity is **not** just the PBR passes. All of the following have OpenGL impls
that need an RHI/Vulkan path before the editor and game render correctly:

- `OpenGLRenderer2D`, `Renderer2D` (sprites/lines)
- `UISystem` / `UIRenderSystem` / `Font` (text + widgets)
- `DebugDraw` (gizmos, audio debug viz)
- `OpenGLParticleRenderer`
- **ImGui backend**: `imgui_impl_opengl3` → `imgui_impl_vulkan`. Viewport-as-
  texture changes from a GL texture id passed to `ImGui::Image` to a
  `VkDescriptorSet` (via `ImGui_ImplVulkan_AddTexture`).

## Architecture: the RHI

A new backend-neutral layer under `Engine/include/Renderer/RHI/` and
`Engine/src/Platform/Vulkan/`. Passes and the render graph talk only to RHI
interfaces; they never see `Gl*` or `Vk*` types.

```
Renderer/RHI/                      (public, backend-neutral)
  RHIDevice.h        — create/destroy resources, begin/end frame, submit
  RHICommandList.h   — record: bind pipeline, bind resources, draw, dispatch, barriers
  RHIBuffer.h        — vertex/index/uniform/storage buffers (+ Map/Unmap)
  RHITexture.h       — images + views + samplers; RHIFormat enum (replaces GLenum)
  RHIPipeline.h      — graphics/compute pipeline + layout (built from shader reflection)
  RHIShader.h        — SPIR-V module + reflected binding layout
  RHIResourceSet.h   — descriptor-set analogue (a "binding group")
  RHIEnums.h         — RHIFormat, RHILoadOp, RHIStoreOp, RHITextureUsage, ...

Platform/Vulkan/                   (private impl)
  VulkanDevice / VulkanSwapchain / VulkanCommandList / VulkanBuffer /
  VulkanTexture / VulkanPipeline / VulkanShader / VulkanDescriptorAllocator /
  VulkanDeletionQueue / VulkanContext (instance, device, queues, VMA, volk)

Platform/OpenGL/                   (existing — wrapped to implement the same RHI,
                                    or kept as legacy path; see migration)
```

### RHIFormat

Replace the raw `GLenum internalFormat` in `RGTextureDesc` with a neutral
`RHIFormat` enum (`RGBA16F`, `RGBA8`, `R16F`, `Depth32F`, ...). Each backend maps
it: OpenGL → `GL_RGBA16F`; Vulkan → `VK_FORMAT_R16G16B16A16_SFLOAT`. This single
change is what lets the render graph stop including `glad/gl.h`.

### Render graph over the RHI

`RenderGraph` keeps its current shape (declare textures, add passes with
reads/writes, compile = topo-sort + cull, execute in order) but:

- `TextureEntry` holds an `RHITexture` instead of `glTexture/glFBO/glDepthRBO`.
- MRT "borrow primary's FBO" logic becomes attachment grouping for a single
  dynamic-rendering pass (`vkCmdBeginRendering` with N color attachments).
- **`reads`/`writes` drive automatic barriers.** Before a pass, the graph
  transitions each read texture to `SHADER_READ_ONLY` and each write texture to
  `COLOR_ATTACHMENT`/`DEPTH_ATTACHMENT`, emitting the needed `VkImageMemoryBarrier2`.
  On OpenGL these are no-ops (or `glMemoryBarrier`).
- Transient textures are pooled/aliased, not freed mid-frame; actual destruction
  goes through the deletion queue.

## Shader & binding model

This is the highest-risk redesign. Target model (works for both backends):

- **Per-frame UBO** (camera/view/proj/time/light arrays) — one descriptor set,
  bound once per frame (set 0).
- **Per-material UBO + textures** — descriptor set bound per material (set 1).
  Replaces the per-draw `SetVec3`/`SetMat4` calls.
- **Per-draw push constants** — model matrix, bone-palette offset, small data
  (set/binding-free, fast). Replaces the per-draw `glUniform` storm.
- **Skinning palette** — storage buffer indexed by push-constant offset.
- **Descriptor layouts built from shader reflection** (SPIR-V reflection via
  `spirv-reflect`, later Slang's reflection API). This is what makes Slang pay
  off and removes hand-maintained binding tables.

`Shader.h`'s by-name setters are replaced by writing into mapped UBO/push-constant
structs. The OpenGL backend implements the same RHI by backing UBOs with
`glBindBufferRange` and push constants with a small uniform block — so passes
written once still run on GL.

### GLSL → SPIR-V build step

- Compile `Assets/Shaders/*.{vert,frag,comp}` to `.spv` at build time via
  `glslangValidator`/`shaderc` (CMake custom command), or at runtime via
  libshaderc for hot-reload (keeps the existing `ShaderLibrary` watch feature).
- Existing GLSL needs minor edits for Vulkan: explicit `layout(set=, binding=)`,
  `gl_Position.y` flip or negative viewport height, no default uniforms.
- **Slang migration (later phase):** Slang compiles to SPIR-V *and* GLSL, has a
  module system and first-class reflection. Swap the compile step; the RHI
  binding model above is already Slang-friendly.

## Frames, sync, lifetime

- **Frames in flight = 2.** Per-frame: command pool/buffer, in-flight fence,
  image-available + render-finished semaphores, and a deletion queue.
- **Deletion queue**: resources are queued for destruction and only freed once
  the frame that last used them has retired (fence signalled). Fixes concern #3.
- **Vulkan 1.3 features**: `dynamicRendering` (no `VkRenderPass`/`VkFramebuffer`
  objects), `synchronization2` (`Vk*Barrier2`), `timelineSemaphore`. Simplifies
  the swapchain and barrier code substantially.

## Dependencies (CMake / FetchContent)

| Lib | How | Notes |
|-----|-----|-------|
| Vulkan-Headers | FetchContent | Headers only; we load via volk, not the system loader. |
| volk | FetchContent | `VK_NO_PROTOTYPES` + `volkInitialize`/`volkLoadDevice`. |
| VMA | FetchContent (header) | Compiled into one TU like miniaudio's `*_impl.c`. |
| shaderc *or* glslang | FetchContent / SDK | Offline (`.spv` at build) and/or runtime hot-reload. |
| spirv-reflect | FetchContent | Build descriptor layouts from SPIR-V. |
| Vulkan SDK | system install | Needed for **validation layers** (dev) and tooling (RenderDoc, `glslangValidator`). Not a link dependency at runtime. |

Vulkan is gated behind a `DIAMOND_ENABLE_VULKAN` CMake option (default ON for
Windows/Linux, OFF/`APPLE`). The macOS build path is unchanged.

## Backend selection

`RendererAPI::API` already has `Vulkan`. Selection at startup (config/CLI flag),
defaulting to OpenGL until Vulkan reaches parity, then flipping the default on
Windows/Linux. macOS forces OpenGL.

## Migration / OpenGL coexistence

The existing `OpenGL*Pass` classes are **not** rewritten on day one. Plan:

1. Build the RHI interfaces + Vulkan impl to the clear-screen milestone.
2. Stand up one shared pass (PBR surface) over the RHI and an OpenGL RHI impl,
   proving "write once, run on both."
3. Port passes to the RHI one at a time; the old `OpenGL*Pass` stays as the
   reference/fallback until its RHI replacement is verified.
4. Once all passes are on the RHI, the old GL passes are deleted; OpenGL lives
   on only as an RHI backend (frozen feature set).

## Milestones

- **M0 — Foundation (first milestone):** volk + VMA + GLFW Vulkan surface +
  swapchain + 2 frames-in-flight + validation layers + deletion queue. Clears
  the screen to a color. No geometry.
- **M1 — First triangle:** one graphics pipeline, vertex/index buffer, SPIR-V
  shader, per-frame UBO + push constants. Validates the binding model.
- **M2 — RHI extraction:** formalize RHIDevice/CommandList/Buffer/Texture/
  Pipeline; implement the OpenGL RHI backend; one shared PBR pass runs on both.
- **M3 — Render graph over RHI:** RHIFormat, automatic barriers from reads/
  writes, transient pooling.
- **M4 — PBR parity:** G-buffer, SSAO, shadows/CSM, IBL, transparency,
  particles, bloom, tonemap, FXAA on Vulkan.
- **M5 — Editor parity:** Renderer2D, UI/Font, DebugDraw, ImGui-Vulkan, viewport
  texture display.
- **Later:** Slang migration; multithreaded command recording.

## Open questions (revisit during M2)

- Descriptor management strategy: per-frame descriptor pool reset vs. a caching
  allocator (start with per-frame reset, optimize later).
- Pipeline cache + pipeline derivation from material/vertex-layout permutations.
- Whether the OpenGL RHI backend is worth the abstraction tax vs. freezing GL as
  a legacy non-RHI path. Decide once one shared pass exists on both.
