// Single translation unit that compiles the Vulkan Memory Allocator implementation.
// Mirrors the miniaudio_impl.c pattern — keeps VMA's ~20k-line header out of every
// other TU and confines its build cost to one file.
//
// We load Vulkan entry points through volk (VK_NO_PROTOTYPES), so VMA must not
// reference any statically-linked Vulkan symbols. We hand it function pointers at
// runtime via VmaVulkanFunctions when the allocator is created (see VulkanDevice).

#define VMA_STATIC_VULKAN_FUNCTIONS  0   // don't reference vkGetX prototypes directly
#define VMA_DYNAMIC_VULKAN_FUNCTIONS 0   // we supply the function pointers ourselves

#include <volk.h>                        // pulls in <vulkan/vulkan.h> with VK_NO_PROTOTYPES

#define VMA_IMPLEMENTATION
#include <vk_mem_alloc.h>
