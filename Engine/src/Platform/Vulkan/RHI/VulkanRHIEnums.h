#pragma once

#include "Renderer/RHI/RHIEnums.h"
#include "Platform/Vulkan/VulkanCommon.h"

// Inline RHI-enum → Vulkan-enum mappings, shared by the Vulkan backend TUs.
// Kept in one place so a new format/stage is added exactly once.
namespace Diamond {

inline VkFormat ToVkFormat(RHIFormat f) {
    switch (f) {
        case RHIFormat::RGBA8:      return VK_FORMAT_R8G8B8A8_UNORM;
        case RHIFormat::BGRA8:      return VK_FORMAT_B8G8R8A8_UNORM;
        case RHIFormat::BGRA8_SRGB: return VK_FORMAT_B8G8R8A8_SRGB;
        case RHIFormat::RGBA16F:    return VK_FORMAT_R16G16B16A16_SFLOAT;
        case RHIFormat::R16F:       return VK_FORMAT_R16_SFLOAT;
        case RHIFormat::Depth32F:   return VK_FORMAT_D32_SFLOAT;
        case RHIFormat::Undefined:  return VK_FORMAT_UNDEFINED;
    }
    return VK_FORMAT_UNDEFINED;
}

inline RHIFormat FromVkFormat(VkFormat f) {
    switch (f) {
        case VK_FORMAT_R8G8B8A8_UNORM:      return RHIFormat::RGBA8;
        case VK_FORMAT_B8G8R8A8_UNORM:      return RHIFormat::BGRA8;
        case VK_FORMAT_B8G8R8A8_SRGB:       return RHIFormat::BGRA8_SRGB;
        case VK_FORMAT_R16G16B16A16_SFLOAT: return RHIFormat::RGBA16F;
        case VK_FORMAT_R16_SFLOAT:          return RHIFormat::R16F;
        case VK_FORMAT_D32_SFLOAT:          return RHIFormat::Depth32F;
        default:                            return RHIFormat::Undefined;
    }
}

inline VkFormat ToVkFormat(RHIVertexFormat f) {
    switch (f) {
        case RHIVertexFormat::Float:  return VK_FORMAT_R32_SFLOAT;
        case RHIVertexFormat::Float2: return VK_FORMAT_R32G32_SFLOAT;
        case RHIVertexFormat::Float3: return VK_FORMAT_R32G32B32_SFLOAT;
        case RHIVertexFormat::Float4: return VK_FORMAT_R32G32B32A32_SFLOAT;
    }
    return VK_FORMAT_UNDEFINED;
}

inline VkShaderStageFlags ToVkShaderStages(RHIShaderStage s) {
    VkShaderStageFlags flags = 0;
    if (HasFlag(s, RHIShaderStage::Vertex))   flags |= VK_SHADER_STAGE_VERTEX_BIT;
    if (HasFlag(s, RHIShaderStage::Fragment)) flags |= VK_SHADER_STAGE_FRAGMENT_BIT;
    return flags;
}

inline VkPrimitiveTopology ToVkTopology(RHIPrimitiveTopology t) {
    switch (t) {
        case RHIPrimitiveTopology::TriangleList:  return VK_PRIMITIVE_TOPOLOGY_TRIANGLE_LIST;
        case RHIPrimitiveTopology::TriangleStrip: return VK_PRIMITIVE_TOPOLOGY_TRIANGLE_STRIP;
        case RHIPrimitiveTopology::LineList:      return VK_PRIMITIVE_TOPOLOGY_LINE_LIST;
    }
    return VK_PRIMITIVE_TOPOLOGY_TRIANGLE_LIST;
}

inline VkCullModeFlags ToVkCullMode(RHICullMode c) {
    switch (c) {
        case RHICullMode::None: return VK_CULL_MODE_NONE;
        case RHICullMode::Back: return VK_CULL_MODE_BACK_BIT;
        case RHICullMode::Front:return VK_CULL_MODE_FRONT_BIT;
    }
    return VK_CULL_MODE_NONE;
}

inline VkIndexType ToVkIndexType(RHIIndexType t) {
    return t == RHIIndexType::U16 ? VK_INDEX_TYPE_UINT16 : VK_INDEX_TYPE_UINT32;
}

inline VkDescriptorType ToVkDescriptorType(RHIResourceType t) {
    switch (t) {
        case RHIResourceType::UniformBuffer:        return VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER;
        case RHIResourceType::CombinedImageSampler: return VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER;
    }
    return VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER;
}

inline VkFilter ToVkFilter(RHIFilter f) {
    return f == RHIFilter::Nearest ? VK_FILTER_NEAREST : VK_FILTER_LINEAR;
}

} // namespace Diamond
