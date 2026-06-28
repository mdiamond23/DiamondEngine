#pragma once

// Central include for the Vulkan backend. volk MUST be included before any other
// Vulkan header (it defines VK_NO_PROTOTYPES and re-includes vulkan.h itself), and
// before vk_mem_alloc.h so VMA sees the volk-loaded entry points.
#include <volk.h>
#include <vk_mem_alloc.h>

#include <spdlog/spdlog.h>
#include <cstdlib>

namespace Diamond {

// Human-readable VkResult for diagnostics. Only the results we realistically hit
// during bring-up; everything else falls through to the numeric code.
inline const char* VkResultToString(VkResult r) {
    switch (r) {
        case VK_SUCCESS:                        return "VK_SUCCESS";
        case VK_NOT_READY:                      return "VK_NOT_READY";
        case VK_TIMEOUT:                        return "VK_TIMEOUT";
        case VK_SUBOPTIMAL_KHR:                 return "VK_SUBOPTIMAL_KHR";
        case VK_ERROR_OUT_OF_HOST_MEMORY:       return "VK_ERROR_OUT_OF_HOST_MEMORY";
        case VK_ERROR_OUT_OF_DEVICE_MEMORY:     return "VK_ERROR_OUT_OF_DEVICE_MEMORY";
        case VK_ERROR_INITIALIZATION_FAILED:    return "VK_ERROR_INITIALIZATION_FAILED";
        case VK_ERROR_DEVICE_LOST:              return "VK_ERROR_DEVICE_LOST";
        case VK_ERROR_LAYER_NOT_PRESENT:        return "VK_ERROR_LAYER_NOT_PRESENT";
        case VK_ERROR_EXTENSION_NOT_PRESENT:    return "VK_ERROR_EXTENSION_NOT_PRESENT";
        case VK_ERROR_FEATURE_NOT_PRESENT:      return "VK_ERROR_FEATURE_NOT_PRESENT";
        case VK_ERROR_INCOMPATIBLE_DRIVER:      return "VK_ERROR_INCOMPATIBLE_DRIVER";
        case VK_ERROR_OUT_OF_DATE_KHR:          return "VK_ERROR_OUT_OF_DATE_KHR";
        case VK_ERROR_SURFACE_LOST_KHR:         return "VK_ERROR_SURFACE_LOST_KHR";
        default:                                return "VK_ERROR_<unknown>";
    }
}

} // namespace Diamond

// Aborts on any non-success result. Vulkan bring-up errors are almost always
// programmer/driver faults that can't be recovered from mid-frame, so failing
// loud-and-early beats threading VkResult through every call site. Swapchain
// out-of-date results are handled explicitly at the call site, not via this macro.
#define VK_CHECK(expr)                                                              \
    do {                                                                            \
        VkResult _vk_res = (expr);                                                  \
        if (_vk_res != VK_SUCCESS) {                                                \
            spdlog::critical("[Vulkan] {} failed: {} ({}:{})",                      \
                             #expr, ::Diamond::VkResultToString(_vk_res),           \
                             __FILE__, __LINE__);                                   \
            std::abort();                                                           \
        }                                                                           \
    } while (0)
