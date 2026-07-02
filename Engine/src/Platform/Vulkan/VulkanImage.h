#pragma once

#include "Platform/Vulkan/VulkanCommon.h"
#include <cstdint>

namespace Diamond {

class VulkanContext;

// A GPU image paired with its VMA allocation and a full-subresource view. Like
// VulkanBuffer, lifetime is manual for now (create once, destroy at shutdown /
// on swapchain recreate); the deletion queue arrives with the resource-heavy
// milestones. Used for depth attachments and sampled textures.
struct VulkanImage {
    VkImage       image      = VK_NULL_HANDLE;
    VmaAllocation allocation = VK_NULL_HANDLE;
    VkImageView   view       = VK_NULL_HANDLE;
    VkFormat      format     = VK_FORMAT_UNDEFINED;
    VkExtent2D    extent     { 0, 0 };
};

// Creates a device-local 2D image plus a matching view over all its mips/single
// layer. 'aspect' selects the view aspect (COLOR for textures, DEPTH for depth
// buffers); 'mipLevels' > 1 allocates a mip chain (filled by the caller, e.g. the
// RHI texture upload's blit-downsample).
VulkanImage CreateImage(VulkanContext& ctx, uint32_t width, uint32_t height,
                        VkFormat format, VkImageUsageFlags usage,
                        VkImageAspectFlags aspect, uint32_t mipLevels = 1);

void DestroyImage(VulkanContext& ctx, VulkanImage& image);

// synchronization2 image-layout transition over a full-subresource range (all
// mips, single layer). Generic across aspects (color/depth) so depth attachments
// and texture uploads share one barrier path.
void TransitionImageLayout(VkCommandBuffer cmd, VkImage image, VkImageAspectFlags aspect,
                           VkImageLayout oldLayout, VkImageLayout newLayout,
                           VkPipelineStageFlags2 srcStage, VkAccessFlags2 srcAccess,
                           VkPipelineStageFlags2 dstStage, VkAccessFlags2 dstAccess);

} // namespace Diamond
