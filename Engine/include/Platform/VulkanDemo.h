#pragma once

namespace Diamond {

// M3 smoke test: opens a GLFW window, brings up the Vulkan context + swapchain,
// and draws a textured, lit, depth-tested unit cube (real MeshData geometry, an
// orbiting perspective camera fed through the per-frame UBO, a checkerboard
// combined image-sampler) until the window is closed. Returns 0 on clean exit.
// The only public surface of the Vulkan backend — all Vk* types stay private to
// MyEngine's Platform/Vulkan TUs.
int RunVulkanMeshDemo();

// Wiring Slice 1 smoke test: builds a real Scene (floor + sphere + cubes + point
// lights + sun) and an orbit camera, then drives it through SceneRenderer's
// deferred chain into a Vulkan window — proving the Scene/ECS → RHI bridge end to
// end (no editor, no ImGui yet). Returns 0 on clean exit.
int RunVulkanSceneDemo();

// The Sandbox editor's Vulkan mode (`--vulkan` flag): a docked ImGui editor shell
// whose viewport panel shows SceneRenderer's offscreen output as a sampled image,
// with the in-game UI (canvas widgets) and particles rendered as graph passes.
// Fixed test scene for now — the GL editor's asset pipeline (real Mesh/Texture
// assets, thumbnails) under Vulkan is a later slice. Unlike the demos this is
// declared in every build: without DIAMOND_ENABLE_VULKAN it logs an error and
// returns nonzero, so the app can always link against it.
int RunVulkanEditor();

} // namespace Diamond
