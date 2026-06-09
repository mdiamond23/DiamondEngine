# DiamondEngine Roadmap

The goal of this roadmap is to reach a point where a complete game can be built
using only the editor — no hardcoded scene setup, no recompiling to add objects.

---

## Current State

The renderer is already ahead of schedule for a learning engine.

| System | Status |
|--------|--------|
| OpenGL deferred renderer | Done |
| PBR materials (Albedo, Normal, Metallic, Roughness, AO, Emissive) | Done |
| Cascade shadow maps (CSM, 4 cascades) | Done |
| Point light shadow maps | Done |
| Image-based lighting (IBL) | Done |
| SSAO | Done |
| Frustum culling | Done |
| ECS (EnTT) | Done |
| Editor — Hierarchy panel | Done |
| Editor — Inspector panel (Transform + Mesh Renderer flags) | Done |
| Editor — Inspector: drag-drop mesh/material from Content Browser, asset path display, per-texture slot editing | Done |
| Editor — Content browser with thumbnails | Done |
| Editor — Viewport click-to-select (ray-AABB picking) | Done |
| Editor — Transform gizmos (Translate W / Rotate E / Scale R) | Done |
| MeshComponent flags (visible, castsShadow, receivesShadows) | Done |

---

## Milestone 1 — Editor Foundation
> Goal: build and save a scene without touching code

- [x] Scene serialization — save/load entities + components to JSON
- [x] Drag-drop mesh + material assignment from Content Browser to Inspector ([#20](https://github.com/mdiamond23/DiamondEngine/issues/20))
- [x] Show asset paths in Inspector for assigned mesh/material ([#20](https://github.com/mdiamond23/DiamondEngine/issues/20))
- [x] Light components in ECS (move hardcoded lights into entities with Inspector UI)
- [x] Parent-child transforms (hierarchy grouping, inherited transform)
- [x] Undo / redo (at minimum for transform edits)

---

## Milestone 2 — Gameplay Foundation
> Goal: something can actually happen at runtime

- [x] Input system — named actions mapped to keys, not raw GLFW calls
- [x] Scripting — native C++ components with `OnStart()` / `OnUpdate(dt)` / `OnDestroy()`
- [ ] Collision + physics — [Jolt Physics](https://github.com/jrouwe/JoltPhysics) recommended
- [x] Play mode — start/stop the game loop inside the editor without relaunching

---

## Milestone 3 — Asset Pipeline
> Goal: load real game assets reliably and efficiently

- [ ] Asset registry — track every loaded asset by path, prevent double-loading
- [ ] Hot reload — detect file changes, reload textures and shaders without restart
- [ ] Prefabs — save an entity + its components as a reusable scene template
- [ ] Texture compression — BCn/DXT via a pre-process step (e.g. `texconv`)

---

## Milestone 4 — Shipping
> Goal: export a standalone game without the editor

- [ ] Separate engine runtime from editor (editor code stays in Sandbox)
- [ ] Packager / build step that bundles assets + executable
- [ ] Audio — [miniaudio](https://miniaud.io) (single header, no dependencies)
- [ ] Basic in-game UI — ImGui in game mode or a dedicated UI layer

---

## Milestone 5 — Vulkan Backend
> Goal: replace the OpenGL renderer with Vulkan while keeping the rest of the engine unchanged

The engine's renderer is already abstracted behind interfaces (`Mesh`, `Shader`, `Texture`,
`RendererAPI`). The strategy is to implement Vulkan versions of those interfaces alongside
the existing OpenGL ones, then switch the backend via a compile-time or runtime flag.

- [ ] Abstract `RendererAPI` fully — ensure no raw `gl*` calls exist outside `Platform/OpenGL/`
- [ ] Vulkan device + instance setup (physical device selection, logical device, queues)
- [ ] Swapchain + render pass + framebuffers
- [ ] Vulkan shader pipeline — compile GLSL to SPIR-V via `glslc` as a CMake step
- [ ] Vertex/index buffer abstraction (`VulkanMesh`)
- [ ] Uniform buffers + descriptor sets (replaces `glUniform*` calls)
- [ ] Texture + sampler abstraction (`VulkanTexture`)
- [ ] Deferred rendering G-buffer in Vulkan (multiple render passes / subpasses)
- [ ] Port shadow passes (CSM + point lights)
- [ ] Port IBL, SSAO, and PBR lighting pass
- [ ] ImGui Vulkan backend (`imgui_impl_vulkan`)
- [ ] Remove OpenGL backend once Vulkan is feature-complete (or keep both behind a flag)

### Notes
- Add [Vulkan SDK](https://vulkan.lunarg.com) + `Vulkan::Vulkan` CMake target via `find_package(Vulkan REQUIRED)`
- [Vulkan Memory Allocator (VMA)](https://github.com/GPUOpen-LibrariesAndSDKs/VulkanMemoryAllocator) handles GPU allocations — strongly recommended over manual memory management
- Work through the G-buffer and shadow passes incrementally; don't attempt a full port in one go
- Good reference: [Vulkan Tutorial](https://vulkan-tutorial.com) for device/swapchain setup, [vkguide.dev](https://vkguide.dev) for a modern engine-oriented approach

---

## Open Issues

| # | Title | Priority |
|---|-------|----------|
| [#18](https://github.com/mdiamond23/DiamondEngine/issues/18) | Improve mesh thumbnail camera angle using PCA of vertices | Low |
| [#19](https://github.com/mdiamond23/DiamondEngine/issues/19) | Wire up receivesShadows flag in deferred renderer | Low |
