// M0 Vulkan smoke-test harness. Brings up the Vulkan backend and clears the
// screen — proves volk/VMA/instance/device/swapchain/frame-loop all work end to
// end without touching the OpenGL editor in Sandbox/main.cpp.
#include "Platform/VulkanDemo.h"

int main() {
    return Diamond::RunVulkanClearDemo();
}
