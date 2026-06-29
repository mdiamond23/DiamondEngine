#pragma once

#include "Renderer/RHI/RHIResources.h"
#include "Platform/Vulkan/VulkanBuffer.h"
#include "Platform/Vulkan/VulkanImage.h"
#include "Platform/Vulkan/RHI/VulkanRHIDevice.h"

#include <array>

namespace Diamond {

// GPU buffer behind RHIBuffer. Static buffers are a single device-local
// allocation; dynamic buffers are a per-frame ring of host-visible, persistently
// mapped copies so the CPU can rewrite next frame's data without waiting on the
// GPU. Update()/binding always target the device's current frame slot.
class VulkanRHIBuffer : public RHIBuffer {
public:
    VulkanRHIBuffer(VulkanRHIDevice* device, const RHIBufferDesc& desc);
    ~VulkanRHIBuffer() override;

    void Update(const void* data, uint64_t size) override;

    bool     IsDynamic() const { return m_Dynamic; }
    // The buffer handle to bind/read for a given frame slot. Static buffers
    // ignore 'frame' and always return the single allocation.
    VkBuffer Handle(uint32_t frame) const;

private:
    VulkanRHIDevice* m_Device;
    bool             m_Dynamic;

    VulkanBuffer m_Static;   // dynamic == false
    std::array<VulkanBuffer, VulkanRHIDevice::kFramesInFlight> m_Ring{};   // dynamic == true
    std::array<void*,        VulkanRHIDevice::kFramesInFlight> m_Mapped{};
};

// Sampled texture behind RHITexture: a device-local image uploaded once from CPU
// pixels (staging copy + layout transitions) plus its sampler. Static — read-only
// on the GPU after creation, so a single image/view/sampler is shared by every
// frame's descriptor set.
class VulkanRHITexture : public RHITexture {
public:
    VulkanRHITexture(VulkanRHIDevice* device, const RHITextureDesc& desc);
    ~VulkanRHITexture() override;

    VkImageView View()    const { return m_Image.view; }
    VkSampler   Sampler() const { return m_Sampler; }

private:
    VulkanRHIDevice* m_Device;
    VulkanImage      m_Image;
    VkSampler        m_Sampler = VK_NULL_HANDLE;
};

// A compiled SPIR-V module + the stage it feeds.
class VulkanRHIShader : public RHIShader {
public:
    VulkanRHIShader(VulkanRHIDevice* device, const RHIShaderDesc& desc);
    ~VulkanRHIShader() override;

    VkShaderModule Module() const { return m_Module; }
    RHIShaderStage Stage()  const { return m_Stage; }

private:
    VulkanRHIDevice* m_Device;
    VkShaderModule   m_Module = VK_NULL_HANDLE;
    RHIShaderStage   m_Stage;
};

// Graphics pipeline + its layout. Owns the descriptor-set-0 layout so resource
// sets allocated against this pipeline match its bindings.
class VulkanRHIPipeline : public RHIPipeline {
public:
    VulkanRHIPipeline(VulkanRHIDevice* device, const RHIPipelineDesc& desc);
    ~VulkanRHIPipeline() override;

    VkPipeline            Handle()    const { return m_Pipeline; }
    VkPipelineLayout      Layout()    const { return m_Layout; }
    VkDescriptorSetLayout SetLayout() const { return m_SetLayout; }

private:
    VulkanRHIDevice*      m_Device;
    VkDescriptorSetLayout m_SetLayout = VK_NULL_HANDLE;
    VkPipelineLayout      m_Layout    = VK_NULL_HANDLE;
    VkPipeline            m_Pipeline  = VK_NULL_HANDLE;
};

// One descriptor set per frame slot, each pointing at that frame's copy of the
// bound (dynamic) buffers, so BindResourceSet always picks a set that is safe to
// read while the CPU writes the next frame's buffer copy.
class VulkanRHIResourceSet : public RHIResourceSet {
public:
    VulkanRHIResourceSet(VulkanRHIDevice* device, VulkanRHIPipeline* pipeline,
                         uint32_t setIndex, const std::vector<RHIBufferBinding>& buffers,
                         const std::vector<RHITextureBinding>& textures);
    ~VulkanRHIResourceSet() override = default;   // sets are freed with the device pool

    VkDescriptorSet Handle(uint32_t frame) const { return m_Sets[frame]; }

private:
    VulkanRHIDevice* m_Device;
    std::array<VkDescriptorSet, VulkanRHIDevice::kFramesInFlight> m_Sets{};
};

} // namespace Diamond
