#include "Platform/Vulkan/VulkanSwapchain.h"
#include "Platform/Vulkan/VulkanContext.h"

#define GLFW_INCLUDE_NONE
#include <GLFW/glfw3.h>

#include <algorithm>
#include <limits>

namespace Diamond {

namespace {

// Prefer 8-bit BGRA sRGB (the de-facto desktop default); otherwise take whatever
// the surface offers first.
VkSurfaceFormatKHR ChooseSurfaceFormat(const std::vector<VkSurfaceFormatKHR>& formats) {
    for (const auto& f : formats)
        if (f.format == VK_FORMAT_B8G8R8A8_SRGB &&
            f.colorSpace == VK_COLOR_SPACE_SRGB_NONLINEAR_KHR)
            return f;
    return formats[0];
}

// MAILBOX = low-latency triple buffering when available; FIFO is the guaranteed
// (vsync) fallback.
VkPresentModeKHR ChoosePresentMode(const std::vector<VkPresentModeKHR>& modes) {
    for (auto m : modes)
        if (m == VK_PRESENT_MODE_MAILBOX_KHR) return m;
    return VK_PRESENT_MODE_FIFO_KHR;
}

VkExtent2D ChooseExtent(const VkSurfaceCapabilitiesKHR& caps, GLFWwindow* window) {
    // A non-special currentExtent means the surface dictates the size.
    if (caps.currentExtent.width != std::numeric_limits<uint32_t>::max())
        return caps.currentExtent;

    int w = 0, h = 0;
    glfwGetFramebufferSize(window, &w, &h);
    VkExtent2D extent{ static_cast<uint32_t>(w), static_cast<uint32_t>(h) };
    extent.width  = std::clamp(extent.width,  caps.minImageExtent.width,  caps.maxImageExtent.width);
    extent.height = std::clamp(extent.height, caps.minImageExtent.height, caps.maxImageExtent.height);
    return extent;
}

} // namespace

void VulkanSwapchain::Init(VulkanContext* ctx, GLFWwindow* window) {
    m_Ctx    = ctx;
    m_Window = window;
    Build();
}

void VulkanSwapchain::Build() {
    VkPhysicalDevice phys    = m_Ctx->PhysicalDevice();
    VkSurfaceKHR     surface = m_Ctx->Surface();

    VkSurfaceCapabilitiesKHR caps;
    vkGetPhysicalDeviceSurfaceCapabilitiesKHR(phys, surface, &caps);

    uint32_t formatCount = 0;
    vkGetPhysicalDeviceSurfaceFormatsKHR(phys, surface, &formatCount, nullptr);
    std::vector<VkSurfaceFormatKHR> formats(formatCount);
    vkGetPhysicalDeviceSurfaceFormatsKHR(phys, surface, &formatCount, formats.data());

    uint32_t modeCount = 0;
    vkGetPhysicalDeviceSurfacePresentModesKHR(phys, surface, &modeCount, nullptr);
    std::vector<VkPresentModeKHR> modes(modeCount);
    vkGetPhysicalDeviceSurfacePresentModesKHR(phys, surface, &modeCount, modes.data());

    const VkSurfaceFormatKHR surfaceFormat = ChooseSurfaceFormat(formats);
    const VkPresentModeKHR   presentMode   = ChoosePresentMode(modes);
    m_Extent      = ChooseExtent(caps, m_Window);
    m_ImageFormat = surfaceFormat.format;

    if (IsZeroSize()) return;   // minimized — nothing to build yet

    // One more than the minimum avoids stalling on the driver, clamped to max
    // (0 == "no maximum").
    uint32_t imageCount = caps.minImageCount + 1;
    if (caps.maxImageCount > 0 && imageCount > caps.maxImageCount)
        imageCount = caps.maxImageCount;

    VkSwapchainCreateInfoKHR ci{ VK_STRUCTURE_TYPE_SWAPCHAIN_CREATE_INFO_KHR };
    ci.surface          = surface;
    ci.minImageCount    = imageCount;
    ci.imageFormat      = surfaceFormat.format;
    ci.imageColorSpace  = surfaceFormat.colorSpace;
    ci.imageExtent      = m_Extent;
    ci.imageArrayLayers = 1;
    ci.imageUsage       = VK_IMAGE_USAGE_COLOR_ATTACHMENT_BIT;
    ci.preTransform     = caps.currentTransform;
    ci.compositeAlpha   = VK_COMPOSITE_ALPHA_OPAQUE_BIT_KHR;
    ci.presentMode      = presentMode;
    ci.clipped          = VK_TRUE;

    // If graphics and present are different families, allow both to touch the
    // images concurrently; otherwise exclusive ownership is faster.
    const QueueFamilyIndices& q = m_Ctx->Queues();
    const uint32_t families[] = { q.graphics, q.present };
    if (q.graphics != q.present) {
        ci.imageSharingMode      = VK_SHARING_MODE_CONCURRENT;
        ci.queueFamilyIndexCount = 2;
        ci.pQueueFamilyIndices   = families;
    } else {
        ci.imageSharingMode = VK_SHARING_MODE_EXCLUSIVE;
    }

    VK_CHECK(vkCreateSwapchainKHR(m_Ctx->Device(), &ci, nullptr, &m_Swapchain));

    uint32_t actualCount = 0;
    vkGetSwapchainImagesKHR(m_Ctx->Device(), m_Swapchain, &actualCount, nullptr);
    m_Images.resize(actualCount);
    vkGetSwapchainImagesKHR(m_Ctx->Device(), m_Swapchain, &actualCount, m_Images.data());

    m_ImageViews.resize(actualCount);
    for (uint32_t i = 0; i < actualCount; ++i) {
        VkImageViewCreateInfo vi{ VK_STRUCTURE_TYPE_IMAGE_VIEW_CREATE_INFO };
        vi.image        = m_Images[i];
        vi.viewType     = VK_IMAGE_VIEW_TYPE_2D;
        vi.format       = m_ImageFormat;
        vi.subresourceRange.aspectMask = VK_IMAGE_ASPECT_COLOR_BIT;
        vi.subresourceRange.levelCount = 1;
        vi.subresourceRange.layerCount = 1;
        VK_CHECK(vkCreateImageView(m_Ctx->Device(), &vi, nullptr, &m_ImageViews[i]));
    }
}

void VulkanSwapchain::Recreate() {
    // Block until the GPU is idle before tearing down resources it may still use.
    vkDeviceWaitIdle(m_Ctx->Device());
    DestroyViews();
    if (m_Swapchain) {
        vkDestroySwapchainKHR(m_Ctx->Device(), m_Swapchain, nullptr);
        m_Swapchain = VK_NULL_HANDLE;
    }
    Build();
}

void VulkanSwapchain::DestroyViews() {
    for (VkImageView view : m_ImageViews)
        vkDestroyImageView(m_Ctx->Device(), view, nullptr);
    m_ImageViews.clear();
    m_Images.clear();
}

void VulkanSwapchain::Destroy() {
    DestroyViews();
    if (m_Swapchain) {
        vkDestroySwapchainKHR(m_Ctx->Device(), m_Swapchain, nullptr);
        m_Swapchain = VK_NULL_HANDLE;
    }
}

} // namespace Diamond
