#include "Platform/Vulkan/Passes/GI/VulkanRTReflectionPass.h"
#include "Platform/Vulkan/Passes/VulkanPassCommon.h"
#include "Platform/Vulkan/Passes/IBL/VulkanIBLPass.h"
#include "Platform/Vulkan/RHI/VulkanRHIDevice.h"
#include "Platform/Vulkan/RHI/VulkanRHIResources.h"
#include "Renderer/RHI/RHIDevice.h"
#include "Renderer/RHI/RHICommandList.h"

#include <glm/gtc/matrix_inverse.hpp>

#include <algorithm>
#include <vector>

namespace Diamond {

namespace {
constexpr uint32_t kLocalSize = 8;   // matches rt_reflection_passthrough.comp
} // namespace

VulkanRTReflectionPass::VulkanRTReflectionPass(RHIDevice* device, const std::string& shaderDir,
                                               uint32_t width, uint32_t height)
    : m_Device(device), m_Width(width), m_Height(height)
{
    if (!device->SupportsRayTracing()) return;   // stays inert; Available() false

    RHIBufferDesc uboDesc;
    uboDesc.size    = sizeof(ReflectUBO);
    uboDesc.usage   = RHIBufferUsage::Uniform;
    uboDesc.dynamic = true;
    m_UBO = device->CreateBuffer(uboDesc);

    const std::vector<uint32_t> rgen = LoadSpirv(shaderDir, "rt_reflection.rgen.spv");
    const std::vector<uint32_t> miss = LoadSpirv(shaderDir, "rt_reflection.rmiss.spv");
    const std::vector<uint32_t> chit = LoadSpirv(shaderDir, "rt_reflection.rchit.spv");
    const std::vector<uint32_t> pass = LoadSpirv(shaderDir, "rt_reflection_passthrough.comp.spv");

    RHIShaderDesc rgenDesc{ RHIShaderStage::RayGen,     rgen.data(), rgen.size() };
    RHIShaderDesc missDesc{ RHIShaderStage::Miss,       miss.data(), miss.size() };
    RHIShaderDesc chitDesc{ RHIShaderStage::ClosestHit, chit.data(), chit.size() };
    RHIShaderDesc passDesc{ RHIShaderStage::Compute,    pass.data(), pass.size() };

    m_Raygen          = device->CreateShader(rgenDesc);
    m_Miss            = device->CreateShader(missDesc);
    m_ClosestHit      = device->CreateShader(chitDesc);
    m_PassthroughComp = device->CreateShader(passDesc);

    // ── Trace (ray tracing) ──────────────────────────────────────────────────
    {
        RHIRayTracingPipelineDesc desc{};
        desc.raygenShader     = m_Raygen.get();
        desc.missShader       = m_Miss.get();
        desc.closestHitShader = m_ClosestHit.get();
        desc.resourceBindings = {
            { 0,  RHIResourceType::StorageImage,          RHIShaderStage::RayGen },
            // ClosestHit too: it casts inline ray-query shadow rays at the same
            // TLAS the raygen traces.
            { 1,  RHIResourceType::AccelerationStructure, RHIShaderStage::RayGen
                                                        | RHIShaderStage::ClosestHit },
            { 2,  RHIResourceType::CombinedImageSampler,  RHIShaderStage::RayGen },  // ssrColor
            { 3,  RHIResourceType::CombinedImageSampler,  RHIShaderStage::RayGen },  // gViewPos
            { 4,  RHIResourceType::CombinedImageSampler,  RHIShaderStage::RayGen },  // gViewNormal
            { 5,  RHIResourceType::CombinedImageSampler,  RHIShaderStage::RayGen },  // gMaterial
            { 6,  RHIResourceType::UniformBuffer,         RHIShaderStage::RayGen
                                                        | RHIShaderStage::ClosestHit
                                                        | RHIShaderStage::Miss },
            { 7,  RHIResourceType::UniformBuffer,         RHIShaderStage::ClosestHit },
            { 8,  RHIResourceType::StorageBuffer,         RHIShaderStage::ClosestHit },
            { 9,  RHIResourceType::CombinedImageSampler,  RHIShaderStage::ClosestHit },
            { 10, RHIResourceType::CombinedImageSampler,  RHIShaderStage::ClosestHit },
            { 11, RHIResourceType::CombinedImageSampler,  RHIShaderStage::ClosestHit },
            { 12, RHIResourceType::CombinedImageSampler,  RHIShaderStage::Miss },
            // Bindless albedo maps, indexed per hit by the material's slot.
            { 13, RHIResourceType::CombinedImageSampler,  RHIShaderStage::ClosestHit,
                  kMaxTextures },
        };
        desc.pushConstants = { RHIShaderStage::RayGen, sizeof(TracePush) };
        // Shadows are ray QUERIES, which traverse inline — no recursion.
        desc.maxRecursionDepth = 1;
        m_Pipeline = device->CreateRayTracingPipeline(desc);
    }

    // ── Passthrough (compute) ────────────────────────────────────────────────
    if (m_Pipeline) {
        RHIComputePipelineDesc desc{};
        desc.computeShader    = m_PassthroughComp.get();
        desc.resourceBindings = {
            { 0, RHIResourceType::StorageImage,         RHIShaderStage::Compute },
            { 1, RHIResourceType::CombinedImageSampler, RHIShaderStage::Compute },
        };
        m_PassthroughPipeline = device->CreateComputePipeline(desc);
    }
}

VulkanRTReflectionPass::~VulkanRTReflectionPass() = default;

void VulkanRTReflectionPass::RebuildTraceSet()
{
    // Any of these missing means the scene cannot be traced yet — or at all: an
    // empty scene never builds a TLAS. The execute path falls back to the
    // passthrough, which is why this returns quietly rather than complaining.
    if (!m_TLAS || !m_Geometry || !m_LightUBO || m_AlbedoTextures.empty()) return;
    if (!m_IrradianceTex || !m_VisibilityTex || !m_ProbeDataTex) return;

    std::vector<RHITextureBinding> textures = {
        { 0,  m_ReflectionTex }, { 2,  m_SSRTex },
        { 3,  m_ViewPosTex },    { 4,  m_ViewNormalTex },
        { 5,  m_MaterialTex },
        { 9,  m_IrradianceTex }, { 10, m_VisibilityTex },
        { 11, m_ProbeDataTex },
    };
    // Every slot of the albedo array must be written — accessing an unwritten
    // descriptor is invalid even if no ray ever reaches it. The tail pads with
    // slot 0, which SceneRenderer guarantees is 1x1 white.
    textures.reserve(textures.size() + kMaxTextures);
    for (uint32_t i = 0; i < kMaxTextures; ++i) {
        RHITexture* tex = i < m_AlbedoTextures.size() ? m_AlbedoTextures[i]
                                                      : m_AlbedoTextures[0];
        textures.push_back({ 13, tex, i });
    }

    // The set being replaced may still be referenced by an in-flight frame.
    m_Device->WaitIdle();
    m_TraceSet = m_Device->CreateResourceSet(
        m_Pipeline.get(), 0,
        { { 6, m_UBO.get() }, { 7, m_LightUBO }, { 8, m_Geometry } },
        textures,
        { { 1, m_TLAS } });
    m_SetDirty = false;

    // Binding 12 is the baked env cubemap, which isn't an RHITexture — it goes
    // in raw, and only after the set exists, so it has to be re-applied here.
    if (m_IBL) BindEnvironment(*m_IBL);
}

void VulkanRTReflectionPass::BindEnvironment(const VulkanIBLPass& ibl)
{
    m_IBL = &ibl;
    if (!m_TraceSet) return;   // RebuildTraceSet re-applies this once it exists

    auto* device = static_cast<VulkanRHIDevice*>(m_Device);
    auto* set    = static_cast<VulkanRHIResourceSet*>(m_TraceSet.get());

    VkDescriptorImageInfo info{};
    info.sampler     = ibl.Sampler();
    info.imageView   = ibl.EnvView();
    info.imageLayout = VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL;

    for (uint32_t frame = 0; frame < VulkanRHIDevice::kFramesInFlight; ++frame) {
        VkWriteDescriptorSet write{ VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET };
        write.dstSet          = set->Handle(frame);
        write.dstBinding      = 12;
        write.descriptorCount = 1;
        write.descriptorType  = VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER;
        write.pImageInfo      = &info;
        vkUpdateDescriptorSets(device->Ctx().Device(), 1, &write, 0, nullptr);
    }
}

void VulkanRTReflectionPass::AddToGraph(RHIRenderGraph& graph,
                                        RGTextureHandle ssrColor,
                                        RGTextureHandle rtReflection,
                                        RGTextureHandle viewPos,
                                        RGTextureHandle viewNormal,
                                        RGTextureHandle material,
                                        const VulkanDeferredLightingPass::DDGIAtlases& ddgi)
{
    if (!Available()) return;

    m_ReflectionTex = graph.GetTexture(rtReflection);
    m_SSRTex        = graph.GetTexture(ssrColor);
    m_ViewPosTex    = graph.GetTexture(viewPos);
    m_ViewNormalTex = graph.GetTexture(viewNormal);
    m_MaterialTex   = graph.GetTexture(material);

    // The atlases resolve to the shared textures when they exist and to the
    // caller's 1x1 fallback otherwise — the set must be complete either way,
    // and whether the contents mean anything is giMode's business.
    m_IrradianceTex = ddgi.Valid() ? graph.GetTexture(ddgi.irradiance) : ddgi.fallback;
    m_VisibilityTex = ddgi.Valid() ? graph.GetTexture(ddgi.visibility) : ddgi.fallback;
    m_ProbeDataTex  = ddgi.Valid() ? graph.GetTexture(ddgi.probeData)  : ddgi.fallback;

    // No TLAS dependency, so this one can be built now.
    if (!m_PassthroughSet)
        m_PassthroughSet = m_Device->CreateResourceSet(
            m_PassthroughPipeline.get(), 0, {},
            { { 0, m_ReflectionTex }, { 1, m_SSRTex } });

    const uint32_t groupsX = (m_Width  + kLocalSize - 1) / kLocalSize;
    const uint32_t groupsY = (m_Height + kLocalSize - 1) / kLocalSize;

    RGPass& pass = graph.AddPass("RTReflect").AsCompute()
        .Read(ssrColor).Read(viewPos).Read(viewNormal).Read(material)
        .Write(rtReflection)
        .SetExecute([this, groupsX, groupsY](RHICommandList* cmd) {
            if (m_Enabled) {
                // Deferred until execute: the TLAS and geometry table may not
                // exist when the graph is built, and either can be replaced.
                if (m_SetDirty) RebuildTraceSet();
                if (m_TraceSet) {
                    TracePush push{ m_ConfidenceThreshold, m_RoughnessCutoff,
                                    m_MaxRayDistance };
                    cmd->BindPipeline(m_Pipeline.get());
                    cmd->BindResourceSet(0, m_TraceSet.get());
                    cmd->PushConstants(RHIShaderStage::RayGen, 0, sizeof(push), &push);
                    cmd->TraceRays(m_Width, m_Height);
                    return;
                }
            }

            // Disabled, or nothing to trace against (a scene with no static
            // opaque meshes never builds a TLAS). The composite samples this
            // target now, so falling through without a write would show
            // whatever the pool last held — copy SSR instead.
            cmd->BindPipeline(m_PassthroughPipeline.get());
            cmd->BindResourceSet(0, m_PassthroughSet.get());
            cmd->Dispatch(groupsX, groupsY, 1);
        });

    // Ordering only — the hit shader samples these through the trace set, not
    // through the graph. Costs nothing new: it makes this pass depend on
    // DDGIBlendIrradiance, already upstream of lighting -> SSGI -> SSR -> here.
    if (ddgi.Valid())
        pass.Read(ddgi.irradiance).Read(ddgi.visibility).Read(ddgi.probeData);
}

void VulkanRTReflectionPass::SetTLAS(RHIAccelStruct* tlas) {
    if (m_TLAS == tlas) return;
    m_TLAS     = tlas;
    m_SetDirty = true;
}

void VulkanRTReflectionPass::SetGeometryBuffer(RHIBuffer* geometry) {
    if (m_Geometry == geometry) return;
    m_Geometry = geometry;
    m_SetDirty = true;
}

void VulkanRTReflectionPass::SetLightingUBO(RHIBuffer* ubo) {
    if (m_LightUBO == ubo) return;
    m_LightUBO = ubo;
    m_SetDirty = true;
}

void VulkanRTReflectionPass::SetAlbedoTextures(const std::vector<RHITexture*>& textures) {
    if (textures.empty()) return;
    m_AlbedoTextures = textures;
    m_SetDirty       = true;
}

void VulkanRTReflectionPass::SetFrameData(
        const glm::mat4& view, int giMode,
        const VulkanDeferredLightingPass::DDGISampleParams& volume,
        float skyIntensity)
{
    if (!Available()) return;

    m_UBOData.view        = view;
    m_UBOData.invView     = glm::inverse(view);
    m_UBOData.ddgiOrigin  = glm::vec4(volume.origin,  0.0f);
    m_UBOData.ddgiSpacing = glm::vec4(volume.spacing, 0.0f);
    m_UBOData.ddgiCounts  = glm::ivec4(volume.counts, 0);
    m_UBOData.ddgiParams  = glm::vec4(volume.normalBias, volume.energy,
                                      volume.viewBias, static_cast<float>(giMode));
    m_UBOData.params      = glm::vec4(m_MaxRayDistance, skyIntensity, 0.0f, 0.0f);
    m_UBO->Update(&m_UBOData, sizeof(ReflectUBO));
}

void VulkanRTReflectionPass::SetConfidenceThreshold(float threshold) {
    m_ConfidenceThreshold = std::clamp(threshold, 0.0f, 1.0f);
}

void VulkanRTReflectionPass::SetRoughnessCutoff(float cutoff) {
    m_RoughnessCutoff = std::clamp(cutoff, 0.0f, 1.0f);
}

void VulkanRTReflectionPass::SetMaxRayDistance(float distance) {
    m_MaxRayDistance = std::max(0.1f, distance);
}

} // namespace Diamond
