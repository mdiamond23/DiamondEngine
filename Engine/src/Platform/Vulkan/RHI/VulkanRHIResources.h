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

// Texture behind RHITexture. Two flavors share this class:
//   * Static (initialData set): a single device-local image uploaded once from
//     CPU pixels, left in SHADER_READ_ONLY. 'frame' is ignored on access.
//   * Render target (Color/DepthAttachment usage): one image per frame-in-flight
//     so two concurrent frames never write the same image. Layout is tracked per
//     slot and transitioned by the command list between passes.
class VulkanRHITexture : public RHITexture {
public:
    VulkanRHITexture(VulkanRHIDevice* device, const RHITextureDesc& desc);
    ~VulkanRHITexture() override;

    bool               IsRenderTarget() const { return m_RenderTarget; }
    VkSampler          Sampler() const { return m_Sampler; }
    VkFormat           Format()  const { return m_Format; }
    VkExtent2D         Extent()  const { return m_Extent; }
    VkImageAspectFlags Aspect()  const { return m_Aspect; }

    // Per-frame image/view/layout for render targets; static textures ignore
    // 'frame' and always resolve to their single image.
    VkImage        Image(uint32_t frame) const { return m_Images[Slot(frame)].image; }
    VkImageView    View(uint32_t frame)  const { return m_Images[Slot(frame)].view; }
    VkImageLayout& LayoutRef(uint32_t frame)   { return m_Layouts[Slot(frame)]; }

private:
    uint32_t Slot(uint32_t frame) const { return m_RenderTarget ? frame : 0; }

    VulkanRHIDevice*   m_Device;
    bool               m_RenderTarget = false;
    VkSampler          m_Sampler = VK_NULL_HANDLE;
    VkFormat           m_Format  = VK_FORMAT_UNDEFINED;
    VkExtent2D         m_Extent  { 0, 0 };
    VkImageAspectFlags m_Aspect  = VK_IMAGE_ASPECT_COLOR_BIT;

    std::array<VulkanImage,   VulkanRHIDevice::kFramesInFlight> m_Images{};
    std::array<VkImageLayout, VulkanRHIDevice::kFramesInFlight> m_Layouts{};
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
    // Descriptor-set layout for set 'index' (0, or 1 when the pipeline declares a
    // second set). A resource set is allocated against the layout of the set it
    // targets so it binds at the matching index.
    VkDescriptorSetLayout SetLayout(uint32_t index = 0) const { return m_SetLayouts[index]; }

private:
    VulkanRHIDevice*      m_Device;
    // [0] always present; [1] valid only when resourceBindings1 was non-empty.
    std::array<VkDescriptorSetLayout, 2> m_SetLayouts{ VK_NULL_HANDLE, VK_NULL_HANDLE };
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
    // Returns the sets to the device pool. Same contract as the other RHI
    // resources: caller must WaitIdle() before destroying a set still in flight.
    ~VulkanRHIResourceSet() override;

    VkDescriptorSet Handle(uint32_t frame) const { return m_Sets[frame]; }

private:
    VulkanRHIDevice* m_Device;
    std::array<VkDescriptorSet, VulkanRHIDevice::kFramesInFlight> m_Sets{};
};

} // namespace Diamond
