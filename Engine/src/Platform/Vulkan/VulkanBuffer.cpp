#include "Platform/Vulkan/VulkanBuffer.h"
#include "Platform/Vulkan/VulkanContext.h"

#include <cstring>

namespace Diamond {

VulkanBuffer CreateBuffer(VmaAllocator allocator, VkDeviceSize size,
                          VkBufferUsageFlags usage, VmaMemoryUsage memUsage,
                          VmaAllocationCreateFlags flags,
                          VmaAllocationInfo* outInfo) {
    VkBufferCreateInfo bufInfo{ VK_STRUCTURE_TYPE_BUFFER_CREATE_INFO };
    bufInfo.size        = size;
    bufInfo.usage       = usage;
    bufInfo.sharingMode = VK_SHARING_MODE_EXCLUSIVE;

    VmaAllocationCreateInfo allocInfo{};
    allocInfo.usage = memUsage;
    allocInfo.flags = flags;

    VulkanBuffer buffer;
    buffer.size = size;
    VK_CHECK(vmaCreateBuffer(allocator, &bufInfo, &allocInfo,
                             &buffer.handle, &buffer.allocation, outInfo));
    return buffer;
}

VulkanBuffer CreateDeviceLocalBuffer(VulkanContext& ctx, const void* data,
                                     VkDeviceSize size, VkBufferUsageFlags usage) {
    VmaAllocator allocator = ctx.Allocator();

    // Host-visible, persistently-mapped staging source.
    VmaAllocationInfo stagingInfo{};
    VulkanBuffer staging = CreateBuffer(
        allocator, size, VK_BUFFER_USAGE_TRANSFER_SRC_BIT,
        VMA_MEMORY_USAGE_AUTO,
        VMA_ALLOCATION_CREATE_HOST_ACCESS_SEQUENTIAL_WRITE_BIT |
        VMA_ALLOCATION_CREATE_MAPPED_BIT,
        &stagingInfo);
    std::memcpy(stagingInfo.pMappedData, data, static_cast<size_t>(size));

    // Device-local destination — the GPU reads from here every frame.
    VulkanBuffer dst = CreateBuffer(
        allocator, size, usage | VK_BUFFER_USAGE_TRANSFER_DST_BIT,
        VMA_MEMORY_USAGE_AUTO, 0);

    ctx.ImmediateSubmit([&](VkCommandBuffer cmd) {
        VkBufferCopy copy{};
        copy.size = size;
        vkCmdCopyBuffer(cmd, staging.handle, dst.handle, 1, &copy);
    });

    DestroyBuffer(allocator, staging);
    return dst;
}

void DestroyBuffer(VmaAllocator allocator, VulkanBuffer& buffer) {
    if (buffer.handle) {
        vmaDestroyBuffer(allocator, buffer.handle, buffer.allocation);
        buffer = {};
    }
}

} // namespace Diamond
