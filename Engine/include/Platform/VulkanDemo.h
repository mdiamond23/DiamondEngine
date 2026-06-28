#pragma once

namespace Diamond {

// M1 smoke test: opens a GLFW window, brings up the Vulkan context + swapchain,
// and draws one spinning, pulsing indexed triangle (exercising the graphics
// pipeline + per-frame UBO descriptor set + push constants) until the window is
// closed. Returns 0 on clean exit. The only public surface of the Vulkan
// backend — all Vk* types stay private to MyEngine's Platform/Vulkan TUs.
int RunVulkanTriangleDemo();

} // namespace Diamond
