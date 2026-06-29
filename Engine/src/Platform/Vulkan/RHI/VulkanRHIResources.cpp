#include "Platform/Vulkan/RHI/VulkanRHIResources.h"
#include "Platform/Vulkan/RHI/VulkanRHIEnums.h"

#include <cstring>

namespace Diamond {

// ── Buffer ───────────────────────────────────────────────────────────────────

namespace {
VkBufferUsageFlags ToVkBufferUsage(RHIBufferUsage usage) {
    VkBufferUsageFlags flags = 0;
    if (HasFlag(usage, RHIBufferUsage::Vertex))  flags |= VK_BUFFER_USAGE_VERTEX_BUFFER_BIT;
    if (HasFlag(usage, RHIBufferUsage::Index))   flags |= VK_BUFFER_USAGE_INDEX_BUFFER_BIT;
    if (HasFlag(usage, RHIBufferUsage::Uniform)) flags |= VK_BUFFER_USAGE_UNIFORM_BUFFER_BIT;
    return flags;
}
} // namespace

VulkanRHIBuffer::VulkanRHIBuffer(VulkanRHIDevice* device, const RHIBufferDesc& desc)
    : m_Device(device), m_Dynamic(desc.dynamic) {
    VmaAllocator allocator = m_Device->Ctx().Allocator();

    if (!m_Dynamic) {
        // Static: device-local, uploaded once from initialData.
        m_Static = CreateDeviceLocalBuffer(m_Device->Ctx(), desc.initialData,
                                           desc.size, ToVkBufferUsage(desc.usage));
        return;
    }

    // Dynamic: one persistently-mapped, host-visible copy per frame in flight.
    for (uint32_t i = 0; i < VulkanRHIDevice::kFramesInFlight; ++i) {
        VmaAllocationInfo info{};
        m_Ring[i] = CreateBuffer(
            allocator, desc.size, ToVkBufferUsage(desc.usage), VMA_MEMORY_USAGE_AUTO,
            VMA_ALLOCATION_CREATE_HOST_ACCESS_SEQUENTIAL_WRITE_BIT |
            VMA_ALLOCATION_CREATE_MAPPED_BIT,
            &info);
        m_Mapped[i] = info.pMappedData;
    }
}

VulkanRHIBuffer::~VulkanRHIBuffer() {
    VmaAllocator allocator = m_Device->Ctx().Allocator();
    DestroyBuffer(allocator, m_Static);
    for (VulkanBuffer& b : m_Ring) DestroyBuffer(allocator, b);
}

void VulkanRHIBuffer::Update(const void* data, uint64_t size) {
    // Writes the current frame's copy; the device guaranteed that slot idle in
    // BeginFrame, so this never races the GPU.
    std::memcpy(m_Mapped[m_Device->CurrentFrame()], data, static_cast<size_t>(size));
}

VkBuffer VulkanRHIBuffer::Handle(uint32_t frame) const {
    return m_Dynamic ? m_Ring[frame].handle : m_Static.handle;
}

// ── Texture ──────────────────────────────────────────────────────────────────

namespace {
// Bytes per pixel for the formats M3 can upload from CPU pixels.
uint32_t BytesPerPixel(RHIFormat f) {
    switch (f) {
        case RHIFormat::RGBA8: return 4;
        default:               return 4;   // only RGBA8 textures uploaded so far
    }
}
} // namespace

VulkanRHITexture::VulkanRHITexture(VulkanRHIDevice* device, const RHITextureDesc& desc)
    : m_Device(device) {
    VulkanContext& ctx = m_Device->Ctx();
    m_Format = ToVkFormat(desc.format);
    m_Extent = { desc.width, desc.height };

    const bool isColorTarget = HasFlag(desc.usage, RHITextureUsage::ColorAttachment);
    const bool isDepthTarget = HasFlag(desc.usage, RHITextureUsage::DepthAttachment);
    m_RenderTarget = isColorTarget || isDepthTarget;
    m_Aspect       = isDepthTarget ? VK_IMAGE_ASPECT_DEPTH_BIT : VK_IMAGE_ASPECT_COLOR_BIT;

    VkImageUsageFlags usage = 0;
    if (HasFlag(desc.usage, RHITextureUsage::Sampled)) usage |= VK_IMAGE_USAGE_SAMPLED_BIT;
    if (isColorTarget)        usage |= VK_IMAGE_USAGE_COLOR_ATTACHMENT_BIT;
    if (isDepthTarget)        usage |= VK_IMAGE_USAGE_DEPTH_STENCIL_ATTACHMENT_BIT;
    if (desc.initialData)     usage |= VK_IMAGE_USAGE_TRANSFER_DST_BIT;

    // Render targets keep one image per frame-in-flight; static textures use one.
    const uint32_t count = m_RenderTarget ? VulkanRHIDevice::kFramesInFlight : 1;
    for (uint32_t i = 0; i < count; ++i) {
        m_Images[i]  = CreateImage(ctx, desc.width, desc.height, m_Format, usage, m_Aspect);
        m_Layouts[i] = VK_IMAGE_LAYOUT_UNDEFINED;
    }

    // Static textures upload their pixels once and settle in SHADER_READ_ONLY.
    if (desc.initialData) {
        VmaAllocator allocator = ctx.Allocator();
        const VkDeviceSize bytes = static_cast<VkDeviceSize>(desc.width) * desc.height
                                 * BytesPerPixel(desc.format);
        VmaAllocationInfo stagingInfo{};
        VulkanBuffer staging = CreateBuffer(
            allocator, bytes, VK_BUFFER_USAGE_TRANSFER_SRC_BIT, VMA_MEMORY_USAGE_AUTO,
            VMA_ALLOCATION_CREATE_HOST_ACCESS_SEQUENTIAL_WRITE_BIT | VMA_ALLOCATION_CREATE_MAPPED_BIT,
            &stagingInfo);
        std::memcpy(stagingInfo.pMappedData, desc.initialData, static_cast<size_t>(bytes));

        ctx.ImmediateSubmit([&](VkCommandBuffer cmd) {
            TransitionImageLayout(cmd, m_Images[0].image, VK_IMAGE_ASPECT_COLOR_BIT,
                                  VK_IMAGE_LAYOUT_UNDEFINED, VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL,
                                  VK_PIPELINE_STAGE_2_TOP_OF_PIPE_BIT, 0,
                                  VK_PIPELINE_STAGE_2_COPY_BIT, VK_ACCESS_2_TRANSFER_WRITE_BIT);

            VkBufferImageCopy copy{};
            copy.imageSubresource.aspectMask = VK_IMAGE_ASPECT_COLOR_BIT;
            copy.imageSubresource.layerCount = 1;
            copy.imageExtent = { desc.width, desc.height, 1 };
            vkCmdCopyBufferToImage(cmd, staging.handle, m_Images[0].image,
                                   VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL, 1, &copy);

            TransitionImageLayout(cmd, m_Images[0].image, VK_IMAGE_ASPECT_COLOR_BIT,
                                  VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL,
                                  VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL,
                                  VK_PIPELINE_STAGE_2_COPY_BIT, VK_ACCESS_2_TRANSFER_WRITE_BIT,
                                  VK_PIPELINE_STAGE_2_FRAGMENT_SHADER_BIT, VK_ACCESS_2_SHADER_READ_BIT);
        });
        DestroyBuffer(allocator, staging);
        m_Layouts[0] = VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL;
    }

    // Sampled textures (static or sampled render targets) need a sampler.
    if (HasFlag(desc.usage, RHITextureUsage::Sampled)) {
        VkSamplerCreateInfo samplerInfo{ VK_STRUCTURE_TYPE_SAMPLER_CREATE_INFO };
        samplerInfo.magFilter    = ToVkFilter(desc.filter);
        samplerInfo.minFilter    = ToVkFilter(desc.filter);
        samplerInfo.mipmapMode   = VK_SAMPLER_MIPMAP_MODE_LINEAR;
        // Render targets clamp (no wrap artifacts on a full-image sample); uploaded
        // textures repeat so tiled UVs work.
        const VkSamplerAddressMode addr = m_RenderTarget
            ? VK_SAMPLER_ADDRESS_MODE_CLAMP_TO_EDGE : VK_SAMPLER_ADDRESS_MODE_REPEAT;
        samplerInfo.addressModeU = addr;
        samplerInfo.addressModeV = addr;
        samplerInfo.addressModeW = addr;
        samplerInfo.maxLod       = VK_LOD_CLAMP_NONE;
        VK_CHECK(vkCreateSampler(ctx.Device(), &samplerInfo, nullptr, &m_Sampler));
    }
}

VulkanRHITexture::~VulkanRHITexture() {
    if (m_Sampler) vkDestroySampler(m_Device->Ctx().Device(), m_Sampler, nullptr);
    for (VulkanImage& img : m_Images)
        if (img.image) DestroyImage(m_Device->Ctx(), img);
}

// ── Shader ───────────────────────────────────────────────────────────────────

VulkanRHIShader::VulkanRHIShader(VulkanRHIDevice* device, const RHIShaderDesc& desc)
    : m_Device(device), m_Stage(desc.stage) {
    VkShaderModuleCreateInfo ci{ VK_STRUCTURE_TYPE_SHADER_MODULE_CREATE_INFO };
    ci.codeSize = desc.wordCount * sizeof(uint32_t);
    ci.pCode    = desc.spirv;
    VK_CHECK(vkCreateShaderModule(m_Device->Ctx().Device(), &ci, nullptr, &m_Module));
}

VulkanRHIShader::~VulkanRHIShader() {
    vkDestroyShaderModule(m_Device->Ctx().Device(), m_Module, nullptr);
}

// ── Pipeline ─────────────────────────────────────────────────────────────────

VulkanRHIPipeline::VulkanRHIPipeline(VulkanRHIDevice* device, const RHIPipelineDesc& desc)
    : m_Device(device) {
    VkDevice dev = m_Device->Ctx().Device();

    // Descriptor set 0 layout from the requested bindings.
    std::vector<VkDescriptorSetLayoutBinding> bindings;
    bindings.reserve(desc.resourceBindings.size());
    for (const RHIResourceBinding& b : desc.resourceBindings) {
        VkDescriptorSetLayoutBinding vb{};
        vb.binding         = b.binding;
        vb.descriptorType  = ToVkDescriptorType(b.type);
        vb.descriptorCount = 1;
        vb.stageFlags      = ToVkShaderStages(b.stages);
        bindings.push_back(vb);
    }

    VkDescriptorSetLayoutCreateInfo setLayoutInfo{ VK_STRUCTURE_TYPE_DESCRIPTOR_SET_LAYOUT_CREATE_INFO };
    setLayoutInfo.bindingCount = static_cast<uint32_t>(bindings.size());
    setLayoutInfo.pBindings    = bindings.data();
    VK_CHECK(vkCreateDescriptorSetLayout(dev, &setLayoutInfo, nullptr, &m_SetLayout));

    VkPushConstantRange pushRange{};
    const bool hasPush = desc.pushConstants.size > 0;
    if (hasPush) {
        pushRange.stageFlags = ToVkShaderStages(desc.pushConstants.stages);
        pushRange.offset     = 0;
        pushRange.size       = desc.pushConstants.size;
    }

    VkPipelineLayoutCreateInfo layoutInfo{ VK_STRUCTURE_TYPE_PIPELINE_LAYOUT_CREATE_INFO };
    layoutInfo.setLayoutCount         = 1;
    layoutInfo.pSetLayouts            = &m_SetLayout;
    layoutInfo.pushConstantRangeCount = hasPush ? 1 : 0;
    layoutInfo.pPushConstantRanges    = hasPush ? &pushRange : nullptr;
    VK_CHECK(vkCreatePipelineLayout(dev, &layoutInfo, nullptr, &m_Layout));

    auto* vert = static_cast<VulkanRHIShader*>(desc.vertexShader);
    auto* frag = static_cast<VulkanRHIShader*>(desc.fragmentShader);

    VkPipelineShaderStageCreateInfo stages[2]{};
    stages[0].sType  = VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO;
    stages[0].stage  = VK_SHADER_STAGE_VERTEX_BIT;
    stages[0].module = vert->Module();
    stages[0].pName  = "main";
    stages[1].sType  = VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO;
    stages[1].stage  = VK_SHADER_STAGE_FRAGMENT_BIT;
    stages[1].module = frag->Module();
    stages[1].pName  = "main";

    VkVertexInputBindingDescription binding{};
    binding.binding   = 0;
    binding.stride    = desc.vertexLayout.stride;
    binding.inputRate = VK_VERTEX_INPUT_RATE_VERTEX;

    std::vector<VkVertexInputAttributeDescription> attrs;
    attrs.reserve(desc.vertexLayout.attributes.size());
    for (const RHIVertexAttribute& a : desc.vertexLayout.attributes) {
        VkVertexInputAttributeDescription va{};
        va.location = a.location;
        va.binding  = 0;
        va.format   = ToVkFormat(a.format);
        va.offset   = a.offset;
        attrs.push_back(va);
    }

    const bool hasVertexInput = desc.vertexLayout.stride > 0;
    VkPipelineVertexInputStateCreateInfo vertexInput{ VK_STRUCTURE_TYPE_PIPELINE_VERTEX_INPUT_STATE_CREATE_INFO };
    vertexInput.vertexBindingDescriptionCount   = hasVertexInput ? 1 : 0;
    vertexInput.pVertexBindingDescriptions      = hasVertexInput ? &binding : nullptr;
    vertexInput.vertexAttributeDescriptionCount = static_cast<uint32_t>(attrs.size());
    vertexInput.pVertexAttributeDescriptions    = attrs.data();

    VkPipelineInputAssemblyStateCreateInfo inputAssembly{ VK_STRUCTURE_TYPE_PIPELINE_INPUT_ASSEMBLY_STATE_CREATE_INFO };
    inputAssembly.topology = ToVkTopology(desc.topology);

    VkPipelineViewportStateCreateInfo viewportState{ VK_STRUCTURE_TYPE_PIPELINE_VIEWPORT_STATE_CREATE_INFO };
    viewportState.viewportCount = 1;   // dynamic — set per frame by the device
    viewportState.scissorCount  = 1;

    VkPipelineRasterizationStateCreateInfo raster{ VK_STRUCTURE_TYPE_PIPELINE_RASTERIZATION_STATE_CREATE_INFO };
    raster.polygonMode = VK_POLYGON_MODE_FILL;
    raster.cullMode    = ToVkCullMode(desc.cullMode);
    raster.frontFace   = VK_FRONT_FACE_COUNTER_CLOCKWISE;
    raster.lineWidth   = 1.0f;

    VkPipelineMultisampleStateCreateInfo multisample{ VK_STRUCTURE_TYPE_PIPELINE_MULTISAMPLE_STATE_CREATE_INFO };
    multisample.rasterizationSamples = VK_SAMPLE_COUNT_1_BIT;

    // Color-attachment formats: the MRT list when given (deferred G-buffer), else a
    // single-element list from colorFormat. The render pass's color attachments,
    // the blend state, and the dynamic-rendering formats must all agree in count.
    // A depth-only pipeline (shadow/CSM depth pass) declares neither — it leaves the
    // list empty so the render scope opens with zero color attachments.
    std::vector<VkFormat> colorFormats;
    if (!desc.colorFormats.empty())
        for (RHIFormat f : desc.colorFormats) colorFormats.push_back(ToVkFormat(f));
    else if (desc.colorFormat != RHIFormat::Undefined)
        colorFormats.push_back(ToVkFormat(desc.colorFormat));

    // One opaque (blend-disabled) attachment state per color target.
    VkPipelineColorBlendAttachmentState blendAttachment{};
    blendAttachment.colorWriteMask = VK_COLOR_COMPONENT_R_BIT | VK_COLOR_COMPONENT_G_BIT
                                   | VK_COLOR_COMPONENT_B_BIT | VK_COLOR_COMPONENT_A_BIT;
    blendAttachment.blendEnable = VK_FALSE;
    std::vector<VkPipelineColorBlendAttachmentState> blendAttachments(
        colorFormats.size(), blendAttachment);

    VkPipelineColorBlendStateCreateInfo colorBlend{ VK_STRUCTURE_TYPE_PIPELINE_COLOR_BLEND_STATE_CREATE_INFO };
    colorBlend.attachmentCount = static_cast<uint32_t>(blendAttachments.size());
    colorBlend.pAttachments    = blendAttachments.data();

    const VkDynamicState dynamics[] = { VK_DYNAMIC_STATE_VIEWPORT, VK_DYNAMIC_STATE_SCISSOR };
    VkPipelineDynamicStateCreateInfo dynamicState{ VK_STRUCTURE_TYPE_PIPELINE_DYNAMIC_STATE_CREATE_INFO };
    dynamicState.dynamicStateCount = 2;
    dynamicState.pDynamicStates    = dynamics;

    // Depth test/write. Undefined depthFormat → no depth attachment for this
    // pipeline (state disabled, format left UNDEFINED in the rendering info).
    const bool hasDepth = desc.depthFormat != RHIFormat::Undefined;
    VkPipelineDepthStencilStateCreateInfo depthStencil{ VK_STRUCTURE_TYPE_PIPELINE_DEPTH_STENCIL_STATE_CREATE_INFO };
    depthStencil.depthTestEnable  = (hasDepth && desc.depthTest)  ? VK_TRUE : VK_FALSE;
    depthStencil.depthWriteEnable = (hasDepth && desc.depthWrite) ? VK_TRUE : VK_FALSE;
    depthStencil.depthCompareOp   = VK_COMPARE_OP_LESS;   // reversed-Z is a later optimization
    depthStencil.minDepthBounds   = 0.0f;
    depthStencil.maxDepthBounds   = 1.0f;

    VkPipelineRenderingCreateInfo renderingInfo{ VK_STRUCTURE_TYPE_PIPELINE_RENDERING_CREATE_INFO };
    renderingInfo.colorAttachmentCount    = static_cast<uint32_t>(colorFormats.size());
    renderingInfo.pColorAttachmentFormats = colorFormats.data();
    renderingInfo.depthAttachmentFormat   = ToVkFormat(desc.depthFormat);

    VkGraphicsPipelineCreateInfo pipelineInfo{ VK_STRUCTURE_TYPE_GRAPHICS_PIPELINE_CREATE_INFO };
    pipelineInfo.pNext               = &renderingInfo;
    pipelineInfo.stageCount          = 2;
    pipelineInfo.pStages             = stages;
    pipelineInfo.pVertexInputState   = &vertexInput;
    pipelineInfo.pInputAssemblyState = &inputAssembly;
    pipelineInfo.pViewportState      = &viewportState;
    pipelineInfo.pRasterizationState = &raster;
    pipelineInfo.pMultisampleState   = &multisample;
    pipelineInfo.pDepthStencilState  = &depthStencil;
    pipelineInfo.pColorBlendState    = &colorBlend;
    pipelineInfo.pDynamicState       = &dynamicState;
    pipelineInfo.layout              = m_Layout;
    VK_CHECK(vkCreateGraphicsPipelines(dev, VK_NULL_HANDLE, 1, &pipelineInfo, nullptr, &m_Pipeline));
}

VulkanRHIPipeline::~VulkanRHIPipeline() {
    VkDevice dev = m_Device->Ctx().Device();
    vkDestroyPipeline(dev, m_Pipeline, nullptr);
    vkDestroyPipelineLayout(dev, m_Layout, nullptr);
    vkDestroyDescriptorSetLayout(dev, m_SetLayout, nullptr);
}

// ── Resource set ─────────────────────────────────────────────────────────────

VulkanRHIResourceSet::VulkanRHIResourceSet(VulkanRHIDevice* device, VulkanRHIPipeline* pipeline,
                                           uint32_t /*setIndex*/,
                                           const std::vector<RHIBufferBinding>& buffers,
                                           const std::vector<RHITextureBinding>& textures)
    : m_Device(device) {
    VkDevice dev = m_Device->Ctx().Device();

    // One descriptor set per frame slot, all with the pipeline's set-0 layout.
    std::array<VkDescriptorSetLayout, VulkanRHIDevice::kFramesInFlight> layouts;
    layouts.fill(pipeline->SetLayout());

    VkDescriptorSetAllocateInfo alloc{ VK_STRUCTURE_TYPE_DESCRIPTOR_SET_ALLOCATE_INFO };
    alloc.descriptorPool     = m_Device->DescriptorPool();
    alloc.descriptorSetCount = VulkanRHIDevice::kFramesInFlight;
    alloc.pSetLayouts        = layouts.data();
    VK_CHECK(vkAllocateDescriptorSets(dev, &alloc, m_Sets.data()));

    // Each frame's set points at that frame's copy of each bound buffer; textures
    // are static, so the same image/sampler is written into every frame's set.
    for (uint32_t frame = 0; frame < VulkanRHIDevice::kFramesInFlight; ++frame) {
        std::vector<VkDescriptorBufferInfo> bufferInfos(buffers.size());
        std::vector<VkDescriptorImageInfo>  imageInfos(textures.size());
        std::vector<VkWriteDescriptorSet>   writes;
        writes.reserve(buffers.size() + textures.size());

        for (size_t i = 0; i < buffers.size(); ++i) {
            auto* buf = static_cast<VulkanRHIBuffer*>(buffers[i].buffer);
            bufferInfos[i] = {};
            bufferInfos[i].buffer = buf->Handle(frame);
            bufferInfos[i].offset = 0;
            bufferInfos[i].range  = VK_WHOLE_SIZE;

            VkWriteDescriptorSet w{ VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET };
            w.dstSet          = m_Sets[frame];
            w.dstBinding      = buffers[i].binding;
            w.descriptorCount = 1;
            w.descriptorType  = VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER;
            w.pBufferInfo     = &bufferInfos[i];
            writes.push_back(w);
        }

        for (size_t i = 0; i < textures.size(); ++i) {
            auto* tex = static_cast<VulkanRHITexture*>(textures[i].texture);
            imageInfos[i] = {};
            imageInfos[i].sampler     = tex->Sampler();
            imageInfos[i].imageView   = tex->View(frame);   // per-frame for render targets
            imageInfos[i].imageLayout = VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL;

            VkWriteDescriptorSet w{ VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET };
            w.dstSet          = m_Sets[frame];
            w.dstBinding      = textures[i].binding;
            w.descriptorCount = 1;
            w.descriptorType  = VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER;
            w.pImageInfo      = &imageInfos[i];
            writes.push_back(w);
        }

        vkUpdateDescriptorSets(dev, static_cast<uint32_t>(writes.size()), writes.data(), 0, nullptr);
    }
}

} // namespace Diamond
