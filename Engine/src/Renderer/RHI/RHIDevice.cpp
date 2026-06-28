#include "Renderer/RHI/RHIDevice.h"

#include <spdlog/spdlog.h>

// Backend dispatch for RHIDevice::Create. Always compiled; the concrete backend
// factories live in their (conditionally-compiled) platform TUs so this file
// never pulls in volk/GL headers.
namespace Diamond {

#ifdef DIAMOND_ENABLE_VULKAN
std::unique_ptr<RHIDevice> CreateVulkanRHIDevice(GLFWwindow* window);
#endif

std::unique_ptr<RHIDevice> RHIDevice::Create(GLFWwindow* window, RHIBackend backend) {
    switch (backend) {
        case RHIBackend::Vulkan:
#ifdef DIAMOND_ENABLE_VULKAN
            return CreateVulkanRHIDevice(window);
#else
            spdlog::error("[RHI] Vulkan backend requested but not compiled in (DIAMOND_ENABLE_VULKAN off)");
            return nullptr;
#endif
        case RHIBackend::OpenGL:
            // OpenGL RHI backend lands in M2.2.
            spdlog::error("[RHI] OpenGL backend not yet implemented");
            return nullptr;
    }
    return nullptr;
}

} // namespace Diamond
