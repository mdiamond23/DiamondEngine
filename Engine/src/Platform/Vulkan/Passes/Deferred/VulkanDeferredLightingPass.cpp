#include "Platform/Vulkan/Passes/Deferred/VulkanDeferredLightingPass.h"
#include "Platform/Vulkan/Passes/VulkanPassCommon.h"
#include "Platform/Vulkan/Passes/IBL/VulkanIBLPass.h"
#include "Platform/Vulkan/Passes/Shadows/VulkanPointShadowPass.h"
#include "Platform/Vulkan/RHI/VulkanRHIDevice.h"
#include "Platform/Vulkan/RHI/VulkanRHIResources.h"
#include "Renderer/RHI/RHIDevice.h"
#include "Renderer/RHI/RHICommandList.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <utility>

namespace Diamond {

namespace {
constexpr RHIFormat kHDRFormat = RHIFormat::RGBA16F;   // HDR scene color
} // namespace

VulkanDeferredLightingPass::VulkanDeferredLightingPass(RHIDevice* device,
                                                       const std::string& shaderDir)
    : m_Device(device), m_ShaderDir(shaderDir)
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
        { 11, RHIResourceType::CombinedImageSampler, RHIShaderStage::Fragment }, // irradianceMap (cube)
        { 12, RHIResourceType::CombinedImageSampler, RHIShaderStage::Fragment }, // prefilterMap (cube)
        { 13, RHIResourceType::CombinedImageSampler, RHIShaderStage::Fragment }, // brdfLUT (2D)
        { 14, RHIResourceType::CombinedImageSampler, RHIShaderStage::Fragment }, // spotShadow0
        { 15, RHIResourceType::CombinedImageSampler, RHIShaderStage::Fragment }, // spotShadow1
        { 16, RHIResourceType::CombinedImageSampler, RHIShaderStage::Fragment }, // spotShadow2
        { 17, RHIResourceType::CombinedImageSampler, RHIShaderStage::Fragment }, // spotShadow3
        { 18, RHIResourceType::CombinedImageSampler, RHIShaderStage::Fragment }, // pointShadow0 (cube)
        { 19, RHIResourceType::CombinedImageSampler, RHIShaderStage::Fragment }, // pointShadow1 (cube)
        { 20, RHIResourceType::CombinedImageSampler, RHIShaderStage::Fragment }, // pointShadow2 (cube)
        { 21, RHIResourceType::CombinedImageSampler, RHIShaderStage::Fragment }, // pointShadow3 (cube)
    };
    // MRT: location 0 = the lit scene, location 1 = the far-field diffuse
    // irradiance this pass used, which ssgi_composite subtracts to replace the
    // term rather than stack on it.
    desc.colorFormats = { kHDRFormat, kHDRFormat };
    m_Pipeline = device->CreatePipeline(desc);
}

VulkanDeferredLightingPass::~VulkanDeferredLightingPass() = default;

RGPass& VulkanDeferredLightingPass::AddToGraph(
        RHIRenderGraph& graph,
        RGTextureHandle viewPos, RGTextureHandle viewNormal,
        RGTextureHandle albedo,  RGTextureHandle material,
        RGTextureHandle ssao,    RGTextureHandle emissive,
        const std::array<RGTextureHandle, NUM_CASCADES>& cascades,
        const std::array<RHITexture*, NUM_SPOTS>& spotShadows,
        RGTextureHandle output, RGTextureHandle indirect)
{
    if (!m_Set) {
        m_TexBindings = {
            { 0,  graph.GetTexture(viewPos) },     { 1,  graph.GetTexture(viewNormal) },
            { 2,  graph.GetTexture(albedo) },      { 3,  graph.GetTexture(material) },
            { 4,  graph.GetTexture(ssao) },        { 5,  graph.GetTexture(emissive) },
            { 6,  graph.GetTexture(cascades[0]) }, { 7,  graph.GetTexture(cascades[1]) },
            { 8,  graph.GetTexture(cascades[2]) }, { 9,  graph.GetTexture(cascades[3]) },
            { 14, spotShadows[0] },                { 15, spotShadows[1] },
            { 16, spotShadows[2] },                { 17, spotShadows[3] },
        };
        m_Set = m_Device->CreateResourceSet(
            m_Pipeline.get(), 0, { { 10, m_UBO.get() } }, m_TexBindings);
    }

    // Read every graph input so the graph barriers each to SampledRead and orders
    // this pass after the G-buffer/SSAO/CSM producers. Reading all cascades also
    // keeps them alive through dead-pass culling. The spot maps are NOT graph
    // textures — their owner records + transitions them before the graph runs.
    return graph.AddPass("DeferredLighting")
        .Read(viewPos).Read(viewNormal).Read(albedo).Read(material)
        .Read(ssao).Read(emissive)
        .Read(cascades[0]).Read(cascades[1]).Read(cascades[2]).Read(cascades[3])
        // Write order is attachment order: output = location 0, indirect = 1.
        .Write(output).Write(indirect)
        .SetExecute([this](RHICommandList* cmd) {
            cmd->BindPipeline(m_Pipeline.get());
            cmd->BindResourceSet(0, m_Set.get());
            cmd->Draw(3);
        });
}

void VulkanDeferredLightingPass::AddToGraph(
        RHIRenderGraph& graph,
        RGTextureHandle viewPos, RGTextureHandle viewNormal,
        RGTextureHandle albedo,  RGTextureHandle material,
        RGTextureHandle ssao,    RGTextureHandle emissive,
        const std::array<RGTextureHandle, NUM_CASCADES>& cascades,
        const std::array<RGTextureHandle, NUM_SPOTS>& spotShadows,
        RGTextureHandle output, RGTextureHandle indirect)
{
    std::array<RHITexture*, NUM_SPOTS> resolved{};
    for (int i = 0; i < NUM_SPOTS; ++i)
        resolved[i] = graph.GetTexture(spotShadows[i]);

    RGPass& pass = AddToGraph(graph, viewPos, viewNormal, albedo, material,
                              ssao, emissive, cascades, resolved, output, indirect);
    // Graph-owned spot maps rely on this pass's reads for their SampledRead
    // transition (and, when written, producer ordering).
    for (const RGTextureHandle& h : spotShadows)
        pass.Read(h);
}

void VulkanDeferredLightingPass::BindIBL(const VulkanIBLPass& ibl)
{
    m_PrefilterMaxLod = static_cast<float>(ibl.NumPrefilterMips() - 1);

    // The baked maps are world-space cubemaps (not RHITextures), so write bindings
    // 11-13 into every frame slot of the resource set by hand. The maps are static,
    // so the same view/sampler goes into both frames' sets.
    auto* device = static_cast<VulkanRHIDevice*>(m_Device);
    auto* set    = static_cast<VulkanRHIResourceSet*>(m_Set.get());
    VkDevice  dev     = device->Ctx().Device();
    VkSampler sampler = ibl.Sampler();

    const std::array<std::pair<uint32_t, VkImageView>, 3> binds = { {
        { 11, ibl.IrradianceView() }, { 12, ibl.PrefilterView() }, { 13, ibl.BrdfView() },
    } };

    for (uint32_t frame = 0; frame < VulkanRHIDevice::kFramesInFlight; ++frame) {
        std::array<VkDescriptorImageInfo, 3> infos{};
        std::array<VkWriteDescriptorSet, 3>  writes{};
        for (size_t i = 0; i < binds.size(); ++i) {
            infos[i].sampler     = sampler;
            infos[i].imageView   = binds[i].second;
            infos[i].imageLayout = VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL;

            writes[i] = VkWriteDescriptorSet{ VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET };
            writes[i].dstSet          = set->Handle(frame);
            writes[i].dstBinding      = binds[i].first;
            writes[i].descriptorCount = 1;
            writes[i].descriptorType  = VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER;
            writes[i].pImageInfo      = &infos[i];
        }
        vkUpdateDescriptorSets(dev, static_cast<uint32_t>(writes.size()), writes.data(), 0, nullptr);
    }
}

void VulkanDeferredLightingPass::BindPointShadows(const VulkanPointShadowPass& shadows)
{
    // Same raw-write pattern as BindIBL, but the cubes are per-frame-in-flight, so
    // each frame slot's set gets that slot's views. The views never change (only
    // the cube contents do), so this is a one-time write per set build.
    auto* device = static_cast<VulkanRHIDevice*>(m_Device);
    auto* set    = static_cast<VulkanRHIResourceSet*>(m_Set.get());
    VkDevice  dev     = device->Ctx().Device();
    VkSampler sampler = shadows.Sampler();

    constexpr int kLights = VulkanPointShadowPass::MAX_LIGHTS;
    for (uint32_t frame = 0; frame < VulkanRHIDevice::kFramesInFlight; ++frame) {
        std::array<VkDescriptorImageInfo, kLights> infos{};
        std::array<VkWriteDescriptorSet, kLights>  writes{};
        for (int i = 0; i < kLights; ++i) {
            infos[i].sampler     = sampler;
            infos[i].imageView   = shadows.CubeView(frame, i);
            infos[i].imageLayout = VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL;

            writes[i] = VkWriteDescriptorSet{ VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET };
            writes[i].dstSet          = set->Handle(frame);
            writes[i].dstBinding      = 18 + static_cast<uint32_t>(i);
            writes[i].descriptorCount = 1;
            writes[i].descriptorType  = VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER;
            writes[i].pImageInfo      = &infos[i];
        }
        vkUpdateDescriptorSets(dev, static_cast<uint32_t>(writes.size()), writes.data(), 0, nullptr);
    }
}

void VulkanDeferredLightingPass::SetFrameData(
        const glm::mat4& view,
        const glm::vec3& sunDirWorld, const glm::vec3& sunColor,
        const std::array<glm::mat4, NUM_CASCADES>& lightMatrices,
        const std::array<float, NUM_CASCADES>& splits,
        const std::vector<glm::vec3>& pointPosWorld,
        const std::vector<glm::vec3>& pointColor,
        float pointShadowFar,
        const std::vector<SpotLightInfo>& spots,
        const std::array<glm::mat4, NUM_SPOTS>& spotMatrices,
        const std::vector<float>& pointRadius)
{
    // CSM: fold inverse(view) into each cascade matrix so the shader can go from a
    // view-space position straight to light clip (matches the debug shader). The
    // same inverse(view) also rotates view-space normals/reflections to world space
    // for the IBL cubemap lookups and reconstructs the world position the point
    // cube-shadow lookups need.
    const glm::mat4 invView = glm::inverse(view);
    const glm::mat3 viewRot(view);   // rigid view → direction transform
    for (int i = 0; i < NUM_CASCADES; ++i) {
        m_UBOData.lightFromView[i] = lightMatrices[i] * invView;
        m_UBOData.cascadeSplits[i] = splits[i];
    }
    m_UBOData.invView = invView;

    m_UBOData.sunDirView = glm::vec4(glm::normalize(viewRot * sunDirWorld), 0.0f);
    m_UBOData.sunColor   = glm::vec4(sunColor, 0.0f);

    const int np = std::min<int>(static_cast<int>(pointPosWorld.size()), 4);
    for (int i = 0; i < np; ++i) {
        // Point positions live in world space → transform to view space (the world
        // position is kept too — the shadow cubes sample by world direction).
        m_UBOData.pointPos[i]      = view * glm::vec4(pointPosWorld[i], 1.0f);
        // Radius rides .w (the view transform left w = 1). Callers that don't
        // supply radii (the mesh demo) get an effectively unbounded falloff.
        m_UBOData.pointPos[i].w    = i < static_cast<int>(pointRadius.size())
                                       ? pointRadius[i] : 1e6f;
        m_UBOData.pointColor[i]    = glm::vec4(pointColor[i], 0.0f);
        m_UBOData.pointPosWorld[i] = glm::vec4(pointPosWorld[i], 0.0f);
    }

    const int ns = std::min<int>(static_cast<int>(spots.size()), NUM_SPOTS);
    for (int i = 0; i < ns; ++i) {
        const SpotLightInfo& s = spots[i];
        // Fold inverse(view) like the cascades so the shader projects view-space
        // positions straight into the spot's shadow clip.
        m_UBOData.spotFromView[i] = spotMatrices[i] * invView;
        m_UBOData.spotPosView[i]  = glm::vec4(glm::vec3(view * glm::vec4(s.position, 1.0f)),
                                              std::cos(glm::radians(s.outerDeg)));
        m_UBOData.spotDirView[i]  = glm::vec4(glm::normalize(viewRot * s.direction),
                                              std::cos(glm::radians(s.innerDeg)));
        m_UBOData.spotColor[i]    = glm::vec4(s.color, s.range);
    }

    m_UBOData.counts = glm::vec4(static_cast<float>(np), m_PrefilterMaxLod,
                                 static_cast<float>(ns), pointShadowFar);
    m_UBOData.ambient = glm::vec4(m_AmbientIntensity, 0.0f, 0.0f, 0.0f);

    m_UBO->Update(&m_UBOData, sizeof(LightingUBO));
}

void VulkanDeferredLightingPass::Reload()
{
    const std::vector<uint32_t> vs = TryLoadSpirv(m_ShaderDir, "fullscreen.vert.spv");
    const std::vector<uint32_t> fs = TryLoadSpirv(m_ShaderDir, "deferred_lighting.frag.spv");
    if (vs.empty() || fs.empty()) {
        spdlog::warn("[VulkanDeferredLightingPass] reload skipped — missing/corrupt SPIR-V");
        return;
    }

    RHIShaderDesc vsDesc{ RHIShaderStage::Vertex,   vs.data(), vs.size() };
    RHIShaderDesc fsDesc{ RHIShaderStage::Fragment, fs.data(), fs.size() };
    m_Vert = m_Device->CreateShader(vsDesc);
    m_Frag = m_Device->CreateShader(fsDesc);

    RHIPipelineDesc desc;
    desc.vertexShader   = m_Vert.get();
    desc.fragmentShader = m_Frag.get();
    desc.resourceBindings = {
        { 0,  RHIResourceType::CombinedImageSampler, RHIShaderStage::Fragment },
        { 1,  RHIResourceType::CombinedImageSampler, RHIShaderStage::Fragment },
        { 2,  RHIResourceType::CombinedImageSampler, RHIShaderStage::Fragment },
        { 3,  RHIResourceType::CombinedImageSampler, RHIShaderStage::Fragment },
        { 4,  RHIResourceType::CombinedImageSampler, RHIShaderStage::Fragment },
        { 5,  RHIResourceType::CombinedImageSampler, RHIShaderStage::Fragment },
        { 6,  RHIResourceType::CombinedImageSampler, RHIShaderStage::Fragment },
        { 7,  RHIResourceType::CombinedImageSampler, RHIShaderStage::Fragment },
        { 8,  RHIResourceType::CombinedImageSampler, RHIShaderStage::Fragment },
        { 9,  RHIResourceType::CombinedImageSampler, RHIShaderStage::Fragment },
        { 10, RHIResourceType::UniformBuffer,        RHIShaderStage::Fragment },
        { 11, RHIResourceType::CombinedImageSampler, RHIShaderStage::Fragment },
        { 12, RHIResourceType::CombinedImageSampler, RHIShaderStage::Fragment },
        { 13, RHIResourceType::CombinedImageSampler, RHIShaderStage::Fragment },
        { 14, RHIResourceType::CombinedImageSampler, RHIShaderStage::Fragment },
        { 15, RHIResourceType::CombinedImageSampler, RHIShaderStage::Fragment },
        { 16, RHIResourceType::CombinedImageSampler, RHIShaderStage::Fragment },
        { 17, RHIResourceType::CombinedImageSampler, RHIShaderStage::Fragment },
        { 18, RHIResourceType::CombinedImageSampler, RHIShaderStage::Fragment },
        { 19, RHIResourceType::CombinedImageSampler, RHIShaderStage::Fragment },
        { 20, RHIResourceType::CombinedImageSampler, RHIShaderStage::Fragment },
        { 21, RHIResourceType::CombinedImageSampler, RHIShaderStage::Fragment },
    };
    desc.colorFormats = { kHDRFormat, kHDRFormat };
    m_Pipeline = m_Device->CreatePipeline(desc);

    // AddToGraph only runs once at graph-build time, so rebuild the set eagerly
    // from the bindings it captured — the caller re-calls BindIBL/
    // BindPointShadows right after to refill bindings 11-13/18-21.
    m_Set = m_Device->CreateResourceSet(
        m_Pipeline.get(), 0, { { 10, m_UBO.get() } }, m_TexBindings);
}

} // namespace Diamond
