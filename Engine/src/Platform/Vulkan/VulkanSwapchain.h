#pragma once

#include "Platform/Vulkan/VulkanCommon.h"
#include <vector>
#include <cstdint>

struct GLFWwindow;

namespace Diamond {

class VulkanContext;

// Owns the swapchain and its per-image views. Window-lifetime: rebuilt whenever
// the surface changes (resize / minimize-restore). Deliberately separate from
// VulkanContext so the "recreate on resize" boundary is a single object.
//
// No framebuffers or render passes are stored — we render straight into the
// image views via VK_KHR_dynamic_rendering.
class VulkanSwapchain {
public:
    void Init(VulkanContext* ctx, GLFWwindow* window);
    void Recreate();   // tears down and rebuilds at the window's current size
    void Destroy();

    VkSwapchainKHR Handle()      const { return m_Swapchain; }
    VkFormat       ImageFormat() const { return m_ImageFormat; }
    VkExtent2D     Extent()      const { return m_Extent; }
    uint32_t       ImageCount()  const { return static_cast<uint32_t>(m_Images.size()); }

    VkImage     Image(uint32_t i)     const { return m_Images[i]; }
    VkImageView ImageView(uint32_t i) const { return m_ImageViews[i]; }

    // True when the window is minimized (zero-area framebuffer); the caller
    // should skip rendering until it returns false.
    bool IsZeroSize() const { return m_Extent.width == 0 || m_Extent.height == 0; }

private:
    void Build();
    void DestroyViews();

    VulkanContext* m_Ctx    = nullptr;
    GLFWwindow*    m_Window = nullptr;

    VkSwapchainKHR           m_Swapchain  = VK_NULL_HANDLE;
    VkFormat                 m_ImageFormat = VK_FORMAT_UNDEFINED;
    VkExtent2D               m_Extent{ 0, 0 };
    std::vector<VkImage>     m_Images;       // owned by the swapchain, not destroyed by us
    std::vector<VkImageView> m_ImageViews;   // we create and destroy these
};

} // namespace Diamond
