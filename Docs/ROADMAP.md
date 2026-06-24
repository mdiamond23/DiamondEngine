# DiamondEngine Roadmap

The goal of this roadmap is to reach a point where a complete game can be built
using only the editor — no hardcoded scene setup, no recompiling to add objects.

The long-term game target is a casual physics-based fighting game (Gang Beasts-style):
ragdoll characters, active physics control, grabbing, local multiplayer.

---

## Current State

| System | Status |
|--------|--------|
| OpenGL deferred renderer | Done |
| PBR materials (Albedo, Normal, Metallic, Roughness, AO, Emissive) | Done |
| Cascade shadow maps (CSM, 4 cascades) | Done |
| Point light shadow maps | Done |
| Image-based lighting (IBL) | Done |
| SSAO | Done |
| Bloom + FXAA | Done |
| Frustum culling | Done |
| ECS (EnTT) | Done |
| Physics — Jolt rigid bodies, triggers, raycasting, spherecasting | Done |
| Physics — Box, Sphere, Capsule collider shapes | Done |
| Editor — Hierarchy, Inspector, Content Browser, Console panels | Done |
| Editor — Viewport click-to-select (ray-AABB picking) | Done |
| Editor — Transform gizmos (Translate W / Rotate E / Scale R) | Done |
| Editor — Play / Pause / Stop with scene snapshot restore | Done |
| Editor — F to frame selected object in viewport | Done |
| Input — named actions + axes mapped to keyboard / mouse | Done |
| Input — gamepad support (up to 4 controllers, PS4/PS5/Xbox via GLFW) | Done |
| Scripting — native C++ `OnStart` / `OnUpdate` / `OnDestroy` systems | Done |
| Scene serialization — save/load to JSON | Done |
| Parent-child transforms | Done |
| Undo / redo for transform edits | Done |

---

## Milestone 1 — Editor Foundation ✓
> Goal: build and save a scene without touching code

- [x] Scene serialization — save/load entities + components to JSON
- [x] Drag-drop mesh + material assignment from Content Browser to Inspector
- [x] Light components in ECS (move hardcoded lights into entities with Inspector UI)
- [x] Parent-child transforms (hierarchy grouping, inherited transform)
- [x] Undo / redo (at minimum for transform edits)

---

## Milestone 2 — Gameplay Foundation ✓
> Goal: something can actually happen at runtime

- [x] Input system — named actions mapped to keys, not raw GLFW calls
- [x] Gamepad support — up to 4 controllers; PS4/PS5/Xbox/Switch Pro via GLFW gamepad database; `BindAction("Jump", GamepadButton::South)`
- [x] Scripting — native C++ components with `OnStart()` / `OnUpdate(dt)` / `OnDestroy()`
- [x] Collision + physics — Jolt Physics; rigid bodies (Static/Dynamic/Kinematic), Box/Sphere/Capsule shapes, triggers, callbacks, raycasting, spherecasting
- [x] Play / Pause / Stop — full editor game loop with scene snapshot and restore on Stop
- [x] Editor — F to frame selected object (AABB-aware camera focus)

---

## Milestone 3 — Physics Completion & Editor Polish
> Goal: finish the physics surface before building on top of it

- [x] Convex hull collider shape — exposed in Inspector and scripting API
- [x] Physics constraints / joint system — hinge + swing-twist (ball-socket) with angular limits and motors (velocity/position, runtime-drivable via `Physics::SetMotorTarget` / `SetMotorTargetOrientation`); entity-to-entity targeting by UUID; collision groups so connected bones don't self-collide; deferred creation + cross-entity destruction safety; engine-level API wrapping Jolt. (Slider/fixed/point/distance types not built yet — trivial additions to the same `BuildConstraint` framework.)
- [x] Physics materials — restitution and friction per-collider, exposed in Inspector
- [x] Editor — camera smooth orbit / pan (middle mouse) to complement fly mode
- [x] Editor — duplicate entity (Ctrl+D)
- [x] Editor — multi-select transform gizmo

---

## Milestone 4 — Character & Animation Systems
> Goal: lay the foundation for physics-based characters (prerequisite for the game)
> Dependency order matters: GLTF importer → Skeletal animation → Constraints → Ragdoll → IK

- [x] GLTF / GLB importer — skinned meshes + embedded animation clips (via [cgltf](https://github.com/jkuhlmann/cgltf) or [fastgltf](https://github.com/spnda/fastgltf))
- [x] Skeletal animation system — bone hierarchy, skinning weights, GPU skinning shader
- [x] Animation clips + blending — keyframe playback, pose-blend pipeline, cross-fade between clips, and a reusable `.animsm` state machine (states/transitions/parameters) with an in-editor Animator window (timeline scrubber + node-graph canvas). (Any-state transitions, exit-time, and blend trees deferred.)
- [x] Material assets — reusable `.mat` files: create in the content browser (rendered material-ball preview), drag onto meshes, shared instances (edit one → all update), `MeshComponent` references by path. Inline materials still supported.
- [x] Shader system — formal registration and hot-swap instead of ad-hoc shader handles; materials reference a shader/variant by name
- [x] Ragdoll system — skeleton mapped to rigid body chains; passive (limp) and active (joint motors fight toward an animation pose) modes
- [x] Procedural animation / IK — foot placement, hand reach, secondary motion on loose parts
- [x] Grab / constraint system — runtime point constraints for grabbing objects and other characters; destroy on release

---

## Milestone 5 — Game Layer
> Goal: enough systems to build a local-multiplayer physics fighting game

- [x] Dynamic camera system — multi-target tracking, zoom-out when players spread apart, collision avoidance. 
- [ ] In-game UI system — HUD, menus, health bars separate from ImGui (ImGui is editor-only)
  - [x] Text rendering — `Font` bakes a TTF/OTF (stb_truetype) into an R8 glyph atlas; backend-agnostic (rides the abstract `Texture`)
  - [x] 2D batched quad renderer — `Renderer2D` interface (`Begin`/`End`/`DrawQuad`/`DrawTexturedQuad`/`DrawText` + ortho helper) with `OpenGLRenderer2D` batching by texture; quads/sprites/text share one shader
  - [x] Canvas + anchoring — `CanvasComponent` (scale mode + reference resolution) + `RectTransformComponent` (min/max anchors, pivot, position/size, zOrder); `UISystem` resolves rects top-down (point + stretch) into screen pixels for `Renderer2D`. Inspector authoring + scene serialization wired up (undo-tracked).
  - [ ] ECS widget components + UISystem (Image/Text/Button/ProgressBar)
  - [ ] Input + gamepad-navigable focus/events
  - [ ] Data flow: imperative API from scripts now; event bus later ([#27](https://github.com/mdiamond23/DiamondEngine/issues/27))
  - [ ] Editor authoring + serialization
- [ ] Particle system — hit sparks, dust, environmental effects
- [ ] Audio — [miniaudio](https://miniaud.io) (single header, no dependencies); 3D positional audio
- [ ] Local multiplayer — shared-screen with multiple gamepads; player management, respawn
- [ ] Game camera entity — runtime-controllable camera component usable from scripts

---

## Milestone 6 — Asset Pipeline
> Goal: load real game assets reliably and efficiently

- [ ] Asset registry — track every loaded asset by path, prevent double-loading
- [ ] Hot reload — detect file changes, reload textures and shaders without restart
- [ ] Prefabs — save an entity + its components as a reusable scene template
- [ ] Texture compression — BCn/DXT via a pre-process step (e.g. `texconv`)

---

## Milestone 7 — Shipping
> Goal: export a standalone game without the editor

- [ ] Separate engine runtime from editor (editor code stays in Sandbox)
- [ ] Packager / build step that bundles assets + executable
- [ ] Basic in-game UI — ImGui in game mode or a dedicated UI layer

---

## Milestone 8 — Vulkan Backend
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
- Can be worked on in parallel with other milestones since the renderer is already abstracted

---

## Reach — Networking
> Not planned for the near term; extremely hard to get right for physics games

- [ ] P2P architecture — rollback netcode (GGPO-style) for a fighting game; lockstep won't work with Jolt's determinism requirements
- [ ] Lobby / matchmaking — would require a relay server or NAT traversal (e.g. [GameNetworkingSockets](https://github.com/ValveSoftware/GameNetworkingSockets))

---

## Open Issues

| # | Title | Priority |
|---|-------|----------|
| [#18](https://github.com/mdiamond23/DiamondEngine/issues/18) | Improve mesh thumbnail camera angle using PCA of vertices | Low |
| [#19](https://github.com/mdiamond23/DiamondEngine/issues/19) | Wire up receivesShadows flag in deferred renderer | Low |
