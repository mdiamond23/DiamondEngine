#pragma once

namespace Diamond {

// M3 smoke test: opens a GLFW window, brings up the Vulkan context + swapchain,
// and draws a textured, lit, depth-tested unit cube (real MeshData geometry, an
// orbiting perspective camera fed through the per-frame UBO, a checkerboard
// combined image-sampler) until the window is closed. Returns 0 on clean exit.
// The only public surface of the Vulkan backend — all Vk* types stay private to
// MyEngine's Platform/Vulkan TUs.
int RunVulkanMeshDemo();

} // namespace Diamond
