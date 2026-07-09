#pragma once

#include "Platform/Vulkan/VulkanCommon.h"
#include <cstdint>
#include <functional>

struct GLFWwindow;

namespace Diamond {

// Resolved queue family indices. graphics/present are required; transfer is a
// dedicated DMA family when the GPU exposes one, otherwise it aliases graphics.
struct QueueFamilyIndices {
    uint32_t graphics = UINT32_MAX;
    uint32_t present  = UINT32_MAX;
    uint32_t transfer = UINT32_MAX;

    bool IsComplete() const { return graphics != UINT32_MAX && present != UINT32_MAX; }
};

// Owns everything with device lifetime: instance, debug messenger, surface,
// physical/logical device, queues, and the VMA allocator. Created once at
// startup, destroyed once at shutdown. Swapchain and per-frame state live
// elsewhere so their (shorter) lifetimes stay obvious.
//
// Vulkan types never escape the Vulkan backend — this header is compiled only
// into MyEngine's private TUs, never exposed in public engine headers (same
// discipline as Jolt).
class VulkanContext {
public:
    // 'window' must have been created with the GLFW_NO_API client-API hint.
    void Init(GLFWwindow* window);
    void Shutdown();

    VkInstance       Instance()       const { return m_Instance; }
    VkPhysicalDevice PhysicalDevice() const { return m_PhysicalDevice; }
    VkDevice         Device()         const { return m_Device; }
    VkSurfaceKHR     Surface()        const { return m_Surface; }
    VmaAllocator     Allocator()      const { return m_Allocator; }

    const QueueFamilyIndices& Queues() const { return m_Queues; }
    VkQueue GraphicsQueue() const { return m_GraphicsQueue; }
    VkQueue PresentQueue()  const { return m_PresentQueue; }
    VkQueue TransferQueue() const { return m_TransferQueue; }

    const VkPhysicalDeviceProperties& DeviceProperties() const { return m_DeviceProps; }

    // Whether VK_EXT_debug_utils is enabled — gates command-buffer labels and
    // object naming (RenderDoc groups/names). Independent of validation.
    bool DebugUtilsEnabled() const { return m_DebugUtilsEnabled; }

    // Names a Vulkan object for capture/debug tools. No-op when debug utils is
    // unavailable. 'handle' is the raw object handle cast to uint64_t.
    void SetObjectName(VkObjectType type, uint64_t handle, const char* name) const;

    // Records 'record' into a transient command buffer, submits it to the
    // graphics queue, and blocks until it retires. For one-off GPU work outside
    // the frame loop — staging copies, layout transitions during resource
    // creation. Synchronous and not for per-frame use.
    void ImmediateSubmit(const std::function<void(VkCommandBuffer)>& record);

private:
    void CreateInstance();
    void CreateDebugMessenger();
    void CreateSurface(GLFWwindow* window);
    void SelectPhysicalDevice();
    void CreateLogicalDevice();
    void CreateAllocator();
    void CreateImmediateContext();

    VkInstance               m_Instance       = VK_NULL_HANDLE;
    VkDebugUtilsMessengerEXT m_DebugMessenger = VK_NULL_HANDLE;
    VkSurfaceKHR             m_Surface        = VK_NULL_HANDLE;
    VkPhysicalDevice         m_PhysicalDevice = VK_NULL_HANDLE;
    VkDevice                 m_Device         = VK_NULL_HANDLE;
    VmaAllocator             m_Allocator      = VK_NULL_HANDLE;

    QueueFamilyIndices m_Queues;
    VkQueue            m_GraphicsQueue = VK_NULL_HANDLE;
    VkQueue            m_PresentQueue  = VK_NULL_HANDLE;
    VkQueue            m_TransferQueue = VK_NULL_HANDLE;

    // Transient pool + fence backing ImmediateSubmit (one-off out-of-frame work).
    VkCommandPool      m_ImmediatePool  = VK_NULL_HANDLE;
    VkFence            m_ImmediateFence = VK_NULL_HANDLE;

    VkPhysicalDeviceProperties m_DeviceProps{};
    bool                       m_ValidationEnabled = false;
    bool                       m_DebugUtilsEnabled = false;
};

} // namespace Diamond
