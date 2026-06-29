#include "Platform/Vulkan/Passes/Deferred/VulkanDeferredLightingPass.h"
#include "Platform/Vulkan/Passes/VulkanPassCommon.h"
#include "Renderer/RHI/RHIDevice.h"
#include "Renderer/RHI/RHICommandList.h"

#include <algorithm>

namespace Diamond {

namespace {
constexpr RHIFormat kHDRFormat = RHIFormat::RGBA16F;   // HDR scene color
} // namespace

VulkanDeferredLightingPass::VulkanDeferredLightingPass(RHIDevice* device,
                                                       const std::string& shaderDir)
    : m_Device(device)
{
    const std::vector<uint32_t> vs = LoadSpirv(shaderDir, "fullscreen.vert.spv");
    const std::vector<uint32_t> fs = LoadSpirv(shaderDir, "deferred_lighting.frag.spv");
    RHIShaderDesc vsDesc{ RHIShaderStage::Vertex,   vs.data(), vs.size() };
    RHIShaderDesc fsDesc{ RHIShaderStage::Fragment, fs.data(), fs.size() };
    m_Vert = device->CreateShader(vsDesc);
    m_Frag = device->CreateShader(fsDesc);

    RHIBufferDesc uboDesc;
    uboDesc.size    = sizeof(LightingUBO);
    uboDesc.usage   = RHIBufferUsage::Uniform;
    uboDesc.dynamic = true;
    m_UBO = device->CreateBuffer(uboDesc);

    RHIPipelineDesc desc;
    desc.vertexShader   = m_Vert.get();
    desc.fragmentShader = m_Frag.get();
    desc.resourceBindings = {
        { 0,  RHIResourceType::CombinedImageSampler, RHIShaderStage::Fragment }, // gViewPos
        { 1,  RHIResourceType::CombinedImageSampler, RHIShaderStage::Fragment }, // gViewNormal
        { 2,  RHIResourceType::CombinedImageSampler, RHIShaderStage::Fragment }, // gAlbedo
        { 3,  RHIResourceType::CombinedImageSampler, RHIShaderStage::Fragment }, // gMaterial
        { 4,  RHIResourceType::CombinedImageSampler, RHIShaderStage::Fragment }, // ssao
        { 5,  RHIResourceType::CombinedImageSampler, RHIShaderStage::Fragment }, // gEmissive
        { 6,  RHIResourceType::CombinedImageSampler, RHIShaderStage::Fragment }, // shadowCascade0
        { 7,  RHIResourceType::CombinedImageSampler, RHIShaderStage::Fragment }, // shadowCascade1
        { 8,  RHIResourceType::CombinedImageSampler, RHIShaderStage::Fragment }, // shadowCascade2
        { 9,  RHIResourceType::CombinedImageSampler, RHIShaderStage::Fragment }, // shadowCascade3
        { 10, RHIResourceType::UniformBuffer,        RHIShaderStage::Fragment }, // LightingUBO
    };
    desc.colorFormat = kHDRFormat;
    m_Pipeline = device->CreatePipeline(desc);
}

VulkanDeferredLightingPass::~VulkanDeferredLightingPass() = default;

void VulkanDeferredLightingPass::AddToGraph(
        RHIRenderGraph& graph,
        RGTextureHandle viewPos, RGTextureHandle viewNormal,
        RGTextureHandle albedo,  RGTextureHandle material,
        RGTextureHandle ssao,    RGTextureHandle emissive,
        const std::array<RGTextureHandle, NUM_CASCADES>& cascades,
        RGTextureHandle output)
{
    if (!m_Set)
        m_Set = m_Device->CreateResourceSet(
            m_Pipeline.get(), 0, { { 10, m_UBO.get() } },
            { { 0, graph.GetTexture(viewPos) },     { 1, graph.GetTexture(viewNormal) },
              { 2, graph.GetTexture(albedo) },      { 3, graph.GetTexture(material) },
              { 4, graph.GetTexture(ssao) },        { 5, graph.GetTexture(emissive) },
              { 6, graph.GetTexture(cascades[0]) }, { 7, graph.GetTexture(cascades[1]) },
              { 8, graph.GetTexture(cascades[2]) }, { 9, graph.GetTexture(cascades[3]) } });

    // Read every input so the graph barriers each to SampledRead and orders this
    // pass after the G-buffer/SSAO/CSM producers. Reading all four cascades also
    // keeps cascades 1-3 alive through dead-pass culling.
    graph.AddPass("DeferredLighting")
        .Read(viewPos).Read(viewNormal).Read(albedo).Read(material)
        .Read(ssao).Read(emissive)
        .Read(cascades[0]).Read(cascades[1]).Read(cascades[2]).Read(cascades[3])
        .Write(output)
        .SetExecute([this](RHICommandList* cmd) {
            cmd->BindPipeline(m_Pipeline.get());
            cmd->BindResourceSet(0, m_Set.get());
            cmd->Draw(3);
        });
}

void VulkanDeferredLightingPass::SetFrameData(
        const glm::mat4& view,
        const glm::vec3& sunDirWorld, const glm::vec3& sunColor,
        const glm::vec3& ambient,
        const std::array<glm::mat4, NUM_CASCADES>& lightMatrices,
        const std::array<float, NUM_CASCADES>& splits,
        const std::vector<glm::vec3>& pointPosWorld,
        const std::vector<glm::vec3>& pointColor)
{
    // CSM: fold inverse(view) into each cascade matrix so the shader can go from a
    // view-space position straight to light clip (matches the debug shader).
    const glm::mat4 invView = glm::inverse(view);
    const glm::mat3 viewRot(view);   // rigid view → direction transform
    for (int i = 0; i < NUM_CASCADES; ++i) {
        m_UBOData.lightFromView[i] = lightMatrices[i] * invView;
        m_UBOData.cascadeSplits[i] = splits[i];
    }

    m_UBOData.sunDirView = glm::vec4(glm::normalize(viewRot * sunDirWorld), 0.0f);
    m_UBOData.sunColor   = glm::vec4(sunColor, 0.0f);
    m_UBOData.ambient    = glm::vec4(ambient, 0.0f);

    const int np = std::min<int>(static_cast<int>(pointPosWorld.size()), 4);
    for (int i = 0; i < np; ++i) {
        // Point positions live in world space → transform to view space.
        m_UBOData.pointPos[i]   = view * glm::vec4(pointPosWorld[i], 1.0f);
        m_UBOData.pointColor[i] = glm::vec4(pointColor[i], 0.0f);
    }
    m_UBOData.counts = glm::vec4(static_cast<float>(np), 0.0f, 0.0f, 0.0f);

    m_UBO->Update(&m_UBOData, sizeof(LightingUBO));
}

} // namespace Diamond
