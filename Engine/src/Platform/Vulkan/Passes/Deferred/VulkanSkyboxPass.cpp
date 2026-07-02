#include "Platform/Vulkan/Passes/Deferred/VulkanSkyboxPass.h"
#include "Platform/Vulkan/Passes/VulkanPassCommon.h"
#include "Platform/Vulkan/Passes/IBL/VulkanIBLPass.h"
#include "Platform/Vulkan/RHI/VulkanRHIDevice.h"
#include "Platform/Vulkan/RHI/VulkanRHIResources.h"
#include "Renderer/RHI/RHIDevice.h"
#include "Renderer/RHI/RHICommandList.h"
#include "Renderer/MeshData.h"

#include <vector>

namespace Diamond {

VulkanSkyboxPass::VulkanSkyboxPass(RHIDevice* device, const std::string& shaderDir)
    : m_Device(device)
{
    const std::vector<uint32_t> vs = LoadSpirv(shaderDir, "skybox.vert.spv");
    const std::vector<uint32_t> fs = LoadSpirv(shaderDir, "skybox.frag.spv");
    RHIShaderDesc vsDesc{ RHIShaderStage::Vertex,   vs.data(), vs.size() };
    RHIShaderDesc fsDesc{ RHIShaderStage::Fragment, fs.data(), fs.size() };
    m_Vert = device->CreateShader(vsDesc);
    m_Frag = device->CreateShader(fsDesc);

    RHIBufferDesc ub;
    ub.size    = sizeof(CameraUBO);
    ub.usage   = RHIBufferUsage::Uniform;
    ub.dynamic = true;
    m_UBO = device->CreateBuffer(ub);

    RHIPipelineDesc desc;
    desc.vertexShader        = m_Vert.get();
    desc.fragmentShader      = m_Frag.get();
    desc.vertexLayout.stride = sizeof(glm::vec3);   // the pass's own position-only cube
    desc.vertexLayout.attributes = { { 0, RHIVertexFormat::Float3, 0 } };
    desc.resourceBindings = {
        { 0, RHIResourceType::UniformBuffer,        RHIShaderStage::Vertex },
        { 1, RHIResourceType::CombinedImageSampler, RHIShaderStage::Fragment },
    };
    desc.colorFormat  = RHIFormat::RGBA16F;
    desc.cullMode     = RHICullMode::None;          // viewed from inside the cube
    desc.depthFormat  = RHIFormat::Depth32F;
    desc.depthTest    = true;
    desc.depthWrite   = false;
    desc.depthCompare = RHICompareOp::LessEqual;    // depth == 1 at the far plane
    m_Pipeline = device->CreatePipeline(desc);

    // Position-only unit cube, repacked from the engine's mesh factory.
    const MeshData cube = MeshData::UnitCube();
    std::vector<glm::vec3> positions;
    positions.reserve(cube.Vertices.size());
    for (const Vertex& v : cube.Vertices)
        positions.push_back(v.Position);

    RHIBufferDesc vb;
    vb.size        = positions.size() * sizeof(glm::vec3);
    vb.usage       = RHIBufferUsage::Vertex;
    vb.initialData = positions.data();
    m_CubeVB = device->CreateBuffer(vb);

    RHIBufferDesc ib;
    ib.size        = cube.Indices.size() * sizeof(uint32_t);
    ib.usage       = RHIBufferUsage::Index;
    ib.initialData = cube.Indices.data();
    m_CubeIB = device->CreateBuffer(ib);
    m_CubeIndexCount = static_cast<uint32_t>(cube.Indices.size());
}

VulkanSkyboxPass::~VulkanSkyboxPass() = default;

void VulkanSkyboxPass::AddToGraph(RHIRenderGraph& graph,
                                  RGTextureHandle color, RGTextureHandle depth)
{
    if (!m_Set)
        m_Set = m_Device->CreateResourceSet(
            m_Pipeline.get(), 0, { { 0, m_UBO.get() } });   // binding 1 (env) bound raw in BindIBL

    graph.AddPass("Skybox")
        .Write(color)   // composite into the lit HDR scene
        .Write(depth)   // read-only far-plane test (depthWrite = false)
        .Load()         // LOAD both — keep the lit scene + its depth
        .SetExecute([this](RHICommandList* cmd) {
            cmd->BindPipeline(m_Pipeline.get());
            cmd->BindResourceSet(0, m_Set.get());
            cmd->BindVertexBuffer(m_CubeVB.get());
            cmd->BindIndexBuffer(m_CubeIB.get(), RHIIndexType::U32);
            cmd->DrawIndexed(m_CubeIndexCount);
        });
}

void VulkanSkyboxPass::BindIBL(const VulkanIBLPass& ibl)
{
    // The env cubemap isn't an RHITexture, so write its view into every frame
    // slot of the set by hand (mirrors the lighting/forward passes' BindIBL).
    auto* device = static_cast<VulkanRHIDevice*>(m_Device);
    auto* set    = static_cast<VulkanRHIResourceSet*>(m_Set.get());
    VkDevice dev = device->Ctx().Device();

    for (uint32_t frame = 0; frame < VulkanRHIDevice::kFramesInFlight; ++frame) {
        VkDescriptorImageInfo info{};
        info.sampler     = ibl.Sampler();
        info.imageView   = ibl.EnvView();
        info.imageLayout = VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL;

        VkWriteDescriptorSet w{ VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET };
        w.dstSet          = set->Handle(frame);
        w.dstBinding      = 1;
        w.descriptorCount = 1;
        w.descriptorType  = VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER;
        w.pImageInfo      = &info;
        vkUpdateDescriptorSets(dev, 1, &w, 0, nullptr);
    }
}

void VulkanSkyboxPass::SetFrameData(const glm::mat4& view, const glm::mat4& projection)
{
    CameraUBO cam{};
    cam.view       = view;
    cam.projection = projection;
    m_UBO->Update(&cam, sizeof(cam));
}

} // namespace Diamond
