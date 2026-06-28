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
        vb.descriptorType  = VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER;   // only kind so far
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

    VkPipelineColorBlendAttachmentState blendAttachment{};
    blendAttachment.colorWriteMask = VK_COLOR_COMPONENT_R_BIT | VK_COLOR_COMPONENT_G_BIT
                                   | VK_COLOR_COMPONENT_B_BIT | VK_COLOR_COMPONENT_A_BIT;
    blendAttachment.blendEnable = VK_FALSE;

    VkPipelineColorBlendStateCreateInfo colorBlend{ VK_STRUCTURE_TYPE_PIPELINE_COLOR_BLEND_STATE_CREATE_INFO };
    colorBlend.attachmentCount = 1;
    colorBlend.pAttachments    = &blendAttachment;

    const VkDynamicState dynamics[] = { VK_DYNAMIC_STATE_VIEWPORT, VK_DYNAMIC_STATE_SCISSOR };
    VkPipelineDynamicStateCreateInfo dynamicState{ VK_STRUCTURE_TYPE_PIPELINE_DYNAMIC_STATE_CREATE_INFO };
    dynamicState.dynamicStateCount = 2;
    dynamicState.pDynamicStates    = dynamics;

    VkFormat colorFormat = ToVkFormat(desc.colorFormat);
    VkPipelineRenderingCreateInfo renderingInfo{ VK_STRUCTURE_TYPE_PIPELINE_RENDERING_CREATE_INFO };
    renderingInfo.colorAttachmentCount    = 1;
    renderingInfo.pColorAttachmentFormats = &colorFormat;

    VkGraphicsPipelineCreateInfo pipelineInfo{ VK_STRUCTURE_TYPE_GRAPHICS_PIPELINE_CREATE_INFO };
    pipelineInfo.pNext               = &renderingInfo;
    pipelineInfo.stageCount          = 2;
    pipelineInfo.pStages             = stages;
    pipelineInfo.pVertexInputState   = &vertexInput;
    pipelineInfo.pInputAssemblyState = &inputAssembly;
    pipelineInfo.pViewportState      = &viewportState;
    pipelineInfo.pRasterizationState = &raster;
    pipelineInfo.pMultisampleState   = &multisample;
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
                                           const std::vector<RHIBufferBinding>& buffers)
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

    // Point each frame's set at that frame's copy of each bound buffer.
    for (uint32_t frame = 0; frame < VulkanRHIDevice::kFramesInFlight; ++frame) {
        std::vector<VkDescriptorBufferInfo> bufferInfos(buffers.size());
        std::vector<VkWriteDescriptorSet>   writes(buffers.size());
        for (size_t i = 0; i < buffers.size(); ++i) {
            auto* buf = static_cast<VulkanRHIBuffer*>(buffers[i].buffer);
            bufferInfos[i] = {};
            bufferInfos[i].buffer = buf->Handle(frame);
            bufferInfos[i].offset = 0;
            bufferInfos[i].range  = VK_WHOLE_SIZE;

            writes[i] = { VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET };
            writes[i].dstSet          = m_Sets[frame];
            writes[i].dstBinding      = buffers[i].binding;
            writes[i].descriptorCount = 1;
            writes[i].descriptorType  = VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER;
            writes[i].pBufferInfo     = &bufferInfos[i];
        }
        vkUpdateDescriptorSets(dev, static_cast<uint32_t>(writes.size()), writes.data(), 0, nullptr);
    }
}

} // namespace Diamond
