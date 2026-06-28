#pragma once

#include "Platform/Vulkan/VulkanCommon.h"

namespace Diamond {

class VulkanContext;

// A GPU buffer paired with its VMA allocation. Lifetime is manual for now — the
// resource-heavy milestones add a deletion queue; at M1 every buffer is created
// once and destroyed at shutdown, so direct vmaDestroyBuffer is sufficient.
struct VulkanBuffer {
    VkBuffer      handle     = VK_NULL_HANDLE;
    VmaAllocation allocation = VK_NULL_HANDLE;
    VkDeviceSize  size       = 0;
};

// Raw buffer create. 'memUsage' + 'flags' (e.g. HOST_ACCESS_SEQUENTIAL_WRITE |
// MAPPED for a persistently-mapped upload buffer) pick the memory type and
// mapping. 'outInfo' optionally returns the allocation info, whose pMappedData
// is the persistent mapping when MAPPED was requested.
VulkanBuffer CreateBuffer(VmaAllocator allocator, VkDeviceSize size,
                          VkBufferUsageFlags usage, VmaMemoryUsage memUsage,
                          VmaAllocationCreateFlags flags = 0,
                          VmaAllocationInfo* outInfo = nullptr);

// Creates a device-local buffer and fills it from CPU data via a host-visible
// staging buffer + a one-time copy on the graphics queue (VulkanContext::
// ImmediateSubmit). The staging buffer is freed before returning.
VulkanBuffer CreateDeviceLocalBuffer(VulkanContext& ctx, const void* data,
                                     VkDeviceSize size, VkBufferUsageFlags usage);

void DestroyBuffer(VmaAllocator allocator, VulkanBuffer& buffer);

} // namespace Diamond
