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

} // namespace Diamond
