#pragma once

namespace Diamond {

// M0 smoke test: opens a GLFW window, brings up the Vulkan context + swapchain,
// and clears the screen to an animated color until the window is closed.
// Returns 0 on clean exit. The only public surface of the Vulkan backend — all
// Vk* types stay private to MyEngine's Platform/Vulkan TUs.
int RunVulkanClearDemo();

} // namespace Diamond
