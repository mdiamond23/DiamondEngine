#include "Platform/Vulkan/Passes/GI/VulkanDDGIPass.h"
#include "Platform/Vulkan/Passes/VulkanPassCommon.h"
#include "Platform/Vulkan/Passes/IBL/VulkanIBLPass.h"
#include "Platform/Vulkan/RHI/VulkanRHIDevice.h"
#include "Platform/Vulkan/RHI/VulkanRHIResources.h"
#include "Renderer/RHI/RHIDevice.h"
#include "Renderer/RHI/RHICommandList.h"

#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/quaternion.hpp>

#include <algorithm>
#include <cmath>
#include <random>

namespace Diamond {

namespace {

constexpr uint32_t kBlendLocalSize    = 8;    // matches both blend .comp
constexpr uint32_t kRelocateLocalSize = 64;   // matches ddgi_relocate.comp

// How sharply the visibility blend's cosine weight falls off. Higher keeps a
// wall's silhouette crisp in the depth moments; the paper's default is 50.
constexpr float kDepthSharpness = 50.0f;

// A uniformly-distributed random rotation (Shoemake). Rotating the Fibonacci ray
// set every frame is what turns a fixed 64-direction pattern into an unbiased
// estimate once the hysteresis blend averages across frames.
glm::mat4 RandomRotation(std::mt19937& rng) {
    std::uniform_real_distribution<float> u(0.0f, 1.0f);
    const float u1 = u(rng), u2 = u(rng), u3 = u(rng);
    const float s1 = std::sqrt(1.0f - u1), s2 = std::sqrt(u1);
    const float t1 = 6.28318530718f * u2, t2 = 6.28318530718f * u3;
    glm::quat q(std::cos(t2) * s2,                       // w
                std::sin(t1) * s1, std::cos(t1) * s1,    // x, y
                std::sin(t2) * s2);                      // z
    return glm::mat4_cast(q);
}

} // namespace

VulkanDDGIPass::VulkanDDGIPass(RHIDevice* device, const std::string& shaderDir,
                               RHIFormat debugTargetFormat)
    : m_Device(device), m_ShaderDir(shaderDir)
{
    if (!device->SupportsRayTracing()) return;   // inert; Available() stays false

    RHIBufferDesc uboDesc;
    uboDesc.size    = sizeof(DDGIUBO);
    uboDesc.usage   = RHIBufferUsage::Uniform;
    uboDesc.dynamic = true;
    m_UBO = device->CreateBuffer(uboDesc);

    const std::vector<uint32_t> rgen  = LoadSpirv(shaderDir, "ddgi_probe_trace.rgen.spv");
    const std::vector<uint32_t> miss  = LoadSpirv(shaderDir, "ddgi_probe_trace.rmiss.spv");
    const std::vector<uint32_t> chit  = LoadSpirv(shaderDir, "ddgi_probe_trace.rchit.spv");
    const std::vector<uint32_t> bIrr  = LoadSpirv(shaderDir, "ddgi_blend_irradiance.comp.spv");
    const std::vector<uint32_t> bVis  = LoadSpirv(shaderDir, "ddgi_blend_visibility.comp.spv");
    const std::vector<uint32_t> reloc = LoadSpirv(shaderDir, "ddgi_relocate.comp.spv");
    const std::vector<uint32_t> dVert = LoadSpirv(shaderDir, "ddgi_probe_debug.vert.spv");
    const std::vector<uint32_t> dFrag = LoadSpirv(shaderDir, "ddgi_probe_debug.frag.spv");

    RHIShaderDesc rgenDesc { RHIShaderStage::RayGen,     rgen.data(),  rgen.size()  };
    RHIShaderDesc missDesc { RHIShaderStage::Miss,       miss.data(),  miss.size()  };
    RHIShaderDesc chitDesc { RHIShaderStage::ClosestHit, chit.data(),  chit.size()  };
    RHIShaderDesc bIrrDesc { RHIShaderStage::Compute,    bIrr.data(),  bIrr.size()  };
    RHIShaderDesc bVisDesc { RHIShaderStage::Compute,    bVis.data(),  bVis.size()  };
    RHIShaderDesc relDesc  { RHIShaderStage::Compute,    reloc.data(), reloc.size() };
    RHIShaderDesc dvDesc   { RHIShaderStage::Vertex,     dVert.data(), dVert.size() };
    RHIShaderDesc dfDesc   { RHIShaderStage::Fragment,   dFrag.data(), dFrag.size() };

    m_Raygen              = device->CreateShader(rgenDesc);
    m_Miss                = device->CreateShader(missDesc);
    m_ClosestHit          = device->CreateShader(chitDesc);
    m_BlendIrradianceComp = device->CreateShader(bIrrDesc);
    m_BlendVisibilityComp = device->CreateShader(bVisDesc);
    m_RelocateComp        = device->CreateShader(relDesc);
    m_DebugVert           = device->CreateShader(dvDesc);
    m_DebugFrag           = device->CreateShader(dfDesc);

    // ── Probe trace (ray tracing) ────────────────────────────────────────────
    // Stage masks are exact: naming a stage that never touches a binding costs
    // nothing at runtime but hides which shader actually reads what.
    {
        RHIRayTracingPipelineDesc desc{};
        desc.raygenShader     = m_Raygen.get();
        desc.missShader       = m_Miss.get();
        desc.closestHitShader = m_ClosestHit.get();
        desc.resourceBindings = {
            // ClosestHit too: it casts inline ray-query shadow rays at the same
            // TLAS the raygen traces.
            { 0,  RHIResourceType::AccelerationStructure, RHIShaderStage::RayGen
                                                        | RHIShaderStage::ClosestHit },
            { 1,  RHIResourceType::StorageImage,          RHIShaderStage::RayGen },
            { 2,  RHIResourceType::UniformBuffer,         RHIShaderStage::RayGen
                                                        | RHIShaderStage::ClosestHit
                                                        | RHIShaderStage::Miss },
            { 3,  RHIResourceType::StorageBuffer,         RHIShaderStage::ClosestHit },
            { 4,  RHIResourceType::CombinedImageSampler,  RHIShaderStage::ClosestHit },
            { 5,  RHIResourceType::CombinedImageSampler,  RHIShaderStage::ClosestHit },
            { 6,  RHIResourceType::CombinedImageSampler,  RHIShaderStage::RayGen
                                                        | RHIShaderStage::ClosestHit },
            { 7,  RHIResourceType::UniformBuffer,         RHIShaderStage::ClosestHit },
            { 8,  RHIResourceType::CombinedImageSampler,  RHIShaderStage::ClosestHit },
            { 9,  RHIResourceType::CombinedImageSampler,  RHIShaderStage::ClosestHit },
            { 10, RHIResourceType::CombinedImageSampler,  RHIShaderStage::ClosestHit },
            { 11, RHIResourceType::CombinedImageSampler,  RHIShaderStage::ClosestHit },
            { 12, RHIResourceType::CombinedImageSampler,  RHIShaderStage::Miss },
            // Bindless albedo maps, indexed per hit by the material's slot.
            { 13, RHIResourceType::CombinedImageSampler,  RHIShaderStage::ClosestHit,
                  kMaxTextures },
        };
        // The recursive bounce comes from last frame's atlas, not from a ray
        // traced inside the hit shader — depth 1 is genuinely all this needs.
        desc.maxRecursionDepth = 1;
        m_TracePipeline = device->CreateRayTracingPipeline(desc);
    }
    if (!m_TracePipeline) return;   // Available() false — nothing else is usable

    // ── Blend irradiance / visibility (compute) ──────────────────────────────
    {
        RHIComputePipelineDesc desc{};
        desc.computeShader    = m_BlendIrradianceComp.get();
        desc.resourceBindings = {
            { 0, RHIResourceType::StorageImage,         RHIShaderStage::Compute },
            { 1, RHIResourceType::CombinedImageSampler, RHIShaderStage::Compute },
            { 2, RHIResourceType::UniformBuffer,        RHIShaderStage::Compute },
            { 3, RHIResourceType::CombinedImageSampler, RHIShaderStage::Compute },
        };
        m_BlendIrradiancePipeline = device->CreateComputePipeline(desc);

        desc.computeShader        = m_BlendVisibilityComp.get();
        m_BlendVisibilityPipeline = device->CreateComputePipeline(desc);
    }

    // ── Relocation + classification (compute) ────────────────────────────────
    {
        RHIComputePipelineDesc desc{};
        desc.computeShader    = m_RelocateComp.get();
        desc.resourceBindings = {
            { 0, RHIResourceType::StorageImage,         RHIShaderStage::Compute },
            { 1, RHIResourceType::CombinedImageSampler, RHIShaderStage::Compute },
            { 2, RHIResourceType::UniformBuffer,        RHIShaderStage::Compute },
        };
        m_RelocatePipeline = device->CreateComputePipeline(desc);
    }

    // ── Probe debug viz (raster) ─────────────────────────────────────────────
    // Depth-tested against the G-buffer depth but not depth-WRITING, matching
    // VulkanDebugDrawPass: probes are occluded by the scene, and overlapping
    // probes resolve by draw order (they rarely overlap at the default radius).
    {
        RHIPipelineDesc desc;
        desc.vertexShader     = m_DebugVert.get();
        desc.fragmentShader   = m_DebugFrag.get();
        desc.resourceBindings = {
            { 0, RHIResourceType::UniformBuffer,        RHIShaderStage::Vertex
                                                      | RHIShaderStage::Fragment },
            { 1, RHIResourceType::CombinedImageSampler, RHIShaderStage::Vertex
                                                      | RHIShaderStage::Fragment },
            { 2, RHIResourceType::CombinedImageSampler, RHIShaderStage::Fragment },
        };
        desc.cullMode     = RHICullMode::None;
        desc.colorFormat  = debugTargetFormat;
        desc.depthFormat  = RHIFormat::Depth32F;
        desc.depthTest    = true;
        desc.depthWrite   = false;
        desc.depthCompare = RHICompareOp::Less;
        m_DebugPipeline   = device->CreatePipeline(desc);
    }
}

VulkanDDGIPass::~VulkanDDGIPass() = default;

// ── Descriptor sets ──────────────────────────────────────────────────────────

void VulkanDDGIPass::RebuildTraceSet()
{
    if (!m_TLAS || !m_Geometry || !m_LightUBO || m_AlbedoTextures.empty()) return;

    std::vector<RHITextureBinding> textures = {
        { 1,  m_RayDataTex },
        { 4,  m_IrradianceTex },
        { 5,  m_VisibilityTex },
        { 6,  m_ProbeDataTex },
        { 8,  m_CascadeTex[0] },
        { 9,  m_CascadeTex[1] },
        { 10, m_CascadeTex[2] },
        { 11, m_CascadeTex[3] },
    };
    // Every slot of the albedo array must be written — accessing an unwritten
    // descriptor is invalid even if no ray ever reaches it. The tail pads with
    // slot 0, which SceneRenderer guarantees is 1×1 white.
    textures.reserve(textures.size() + kMaxTextures);
    for (uint32_t i = 0; i < kMaxTextures; ++i) {
        RHITexture* tex = i < m_AlbedoTextures.size() ? m_AlbedoTextures[i]
                                                      : m_AlbedoTextures[0];
        textures.push_back({ 13, tex, i });
    }

    // The set being replaced may still be referenced by an in-flight frame.
    m_Device->WaitIdle();
    m_TraceSet = m_Device->CreateResourceSet(
        m_TracePipeline.get(), 0,
        { { 2, m_UBO.get() }, { 3, m_Geometry }, { 7, m_LightUBO } },
        textures,
        { { 0, m_TLAS } });
    m_SetDirty = false;

    // Binding 12 is the baked env cubemap, which isn't an RHITexture — it goes
    // in raw, and only after the set exists, so it has to be re-applied here
    // rather than once at setup.
    if (m_IBL) BindEnvironment(*m_IBL);
}

void VulkanDDGIPass::BindEnvironment(const VulkanIBLPass& ibl)
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

// ── Graph ────────────────────────────────────────────────────────────────────

void VulkanDDGIPass::AddToGraph(RHIRenderGraph& graph,
                                RGTextureHandle rayData,
                                RGTextureHandle irradiance,
                                RGTextureHandle visibility,
                                RGTextureHandle probeData,
                                const std::array<RGTextureHandle, NUM_CASCADES>& cascades,
                                RGTextureHandle debugTarget, RGTextureHandle depth,
                                bool toSwapchain)
{
    if (!Available()) return;

    m_RayDataTex    = graph.GetTexture(rayData);
    m_IrradianceTex = graph.GetTexture(irradiance);
    m_VisibilityTex = graph.GetTexture(visibility);
    m_ProbeDataTex  = graph.GetTexture(probeData);
    for (int i = 0; i < NUM_CASCADES; ++i)
        m_CascadeTex[i] = graph.GetTexture(cascades[i]);

    // The compute/raster sets bind only graph textures, all of which exist now.
    // Note each atlas appears as a STORAGE image in the pass that writes it and
    // as a SAMPLER everywhere else — never both in one set, which would need two
    // image layouts simultaneously.
    if (!m_BlendIrradianceSet)
        m_BlendIrradianceSet = m_Device->CreateResourceSet(
            m_BlendIrradiancePipeline.get(), 0, { { 2, m_UBO.get() } },
            { { 0, m_IrradianceTex }, { 1, m_RayDataTex }, { 3, m_ProbeDataTex } });

    if (!m_BlendVisibilitySet)
        m_BlendVisibilitySet = m_Device->CreateResourceSet(
            m_BlendVisibilityPipeline.get(), 0, { { 2, m_UBO.get() } },
            { { 0, m_VisibilityTex }, { 1, m_RayDataTex }, { 3, m_ProbeDataTex } });

    if (!m_RelocateSet)
        m_RelocateSet = m_Device->CreateResourceSet(
            m_RelocatePipeline.get(), 0, { { 2, m_UBO.get() } },
            { { 0, m_ProbeDataTex }, { 1, m_RayDataTex } });

    if (!m_DebugSet)
        m_DebugSet = m_Device->CreateResourceSet(
            m_DebugPipeline.get(), 0, { { 0, m_UBO.get() } },
            { { 1, m_ProbeDataTex }, { 2, m_IrradianceTex } });

    // ── Trace ────────────────────────────────────────────────────────────────
    // Reading the cascades is what orders this after the CSM pass; the three
    // ReadHistory declarations take last frame's atlases without depending on
    // this frame's blend, which would be a cycle.
    {
        RGPass& pass = graph.AddPass("DDGIProbeTrace");
        pass.AsCompute();
        for (int i = 0; i < NUM_CASCADES; ++i) pass.Read(cascades[i]);
        pass.ReadHistory(irradiance).ReadHistory(visibility).ReadHistory(probeData)
            .Write(rayData)
            .SetExecute([this](RHICommandList* cmd) {
                if (!m_Enabled || !m_HasVolume || !m_TLAS || !m_Geometry) return;
                if (m_SetDirty) RebuildTraceSet();
                if (!m_TraceSet) return;
                cmd->BindPipeline(m_TracePipeline.get());
                cmd->BindResourceSet(0, m_TraceSet.get());
                cmd->TraceRays(static_cast<uint32_t>(m_RayCount),
                               static_cast<uint32_t>(m_ProbeCount));
            });
    }

    // ── Blend ────────────────────────────────────────────────────────────────
    graph.AddPass("DDGIBlendIrradiance").AsCompute()
        .Read(rayData).ReadHistory(probeData)
        .Write(irradiance)
        .SetExecute([this](RHICommandList* cmd) {
            if (!m_Enabled || !m_HasVolume || !m_TraceSet) return;
            const uint32_t tile = kIrradianceRes + 2;
            const uint32_t w    = static_cast<uint32_t>(m_Counts.x * m_Counts.y) * tile;
            const uint32_t h    = static_cast<uint32_t>(m_Counts.z) * tile;
            cmd->BindPipeline(m_BlendIrradiancePipeline.get());
            cmd->BindResourceSet(0, m_BlendIrradianceSet.get());
            cmd->Dispatch((w + kBlendLocalSize - 1) / kBlendLocalSize,
                          (h + kBlendLocalSize - 1) / kBlendLocalSize, 1);
        });

    graph.AddPass("DDGIBlendVisibility").AsCompute()
        .Read(rayData).ReadHistory(probeData)
        .Write(visibility)
        .SetExecute([this](RHICommandList* cmd) {
            if (!m_Enabled || !m_HasVolume || !m_TraceSet) return;
            const uint32_t tile = kVisibilityRes + 2;
            const uint32_t w    = static_cast<uint32_t>(m_Counts.x * m_Counts.y) * tile;
            const uint32_t h    = static_cast<uint32_t>(m_Counts.z) * tile;
            cmd->BindPipeline(m_BlendVisibilityPipeline.get());
            cmd->BindResourceSet(0, m_BlendVisibilitySet.get());
            cmd->Dispatch((w + kBlendLocalSize - 1) / kBlendLocalSize,
                          (h + kBlendLocalSize - 1) / kBlendLocalSize, 1);
        });

    // ── Relocate + classify ──────────────────────────────────────────────────
    // Last, so the offsets it writes are what the NEXT frame's trace reads back.
    graph.AddPass("DDGIRelocate").AsCompute()
        .Read(rayData)
        .Write(probeData)
        .SetExecute([this](RHICommandList* cmd) {
            if (!m_Enabled || !m_HasVolume || !m_TraceSet) return;
            const uint32_t groups =
                (static_cast<uint32_t>(m_ProbeCount) + kRelocateLocalSize - 1)
                / kRelocateLocalSize;
            cmd->BindPipeline(m_RelocatePipeline.get());
            cmd->BindResourceSet(0, m_RelocateSet.get());
            cmd->Dispatch(groups, 1, 1);
            // Everything downstream may now treat the atlases as real history.
            m_HistoryValid = true;
            ++m_FramesSinceReset;
        });

    // ── Debug viz ────────────────────────────────────────────────────────────
    // Over the tonemapped LDR image, LOADing it so a disabled viz costs nothing
    // but two layout transitions — the same shape as the RT debug blit.
    {
        RGPass& pass = graph.AddPass("DDGIProbeDebug");
        if (toSwapchain) pass.WriteSwapchain();
        else             pass.Write(debugTarget);
        pass.Write(depth)               // read-only depth test (depthWrite = false)
            .Read(irradiance).Read(probeData)
            .Load()
            .SetExecute([this](RHICommandList* cmd) {
                if (!m_ShowProbes || !m_HasVolume || !m_Enabled) return;
                cmd->BindPipeline(m_DebugPipeline.get());
                cmd->BindResourceSet(0, m_DebugSet.get());
                cmd->Draw(6, static_cast<uint32_t>(m_ProbeCount));
            });
    }
}

// ── Per-frame state ──────────────────────────────────────────────────────────

void VulkanDDGIPass::SetTLAS(RHIAccelStruct* tlas)
{
    if (m_TLAS == tlas) return;
    m_TLAS     = tlas;
    m_SetDirty = true;
}

void VulkanDDGIPass::SetGeometryBuffer(RHIBuffer* geometry)
{
    if (m_Geometry == geometry) return;
    m_Geometry = geometry;
    m_SetDirty = true;
}

void VulkanDDGIPass::SetLightingUBO(RHIBuffer* ubo)
{
    if (m_LightUBO == ubo) return;
    m_LightUBO = ubo;
    m_SetDirty = true;
}

void VulkanDDGIPass::SetAlbedoTextures(const std::vector<RHITexture*>& textures)
{
    if (textures.empty() || textures == m_AlbedoTextures) return;
    m_AlbedoTextures = textures;
    m_SetDirty       = true;
}

VulkanDDGIPass::GridLayout VulkanDDGIPass::ComputeGrid(const DDGIVolumeComponent& volume,
                                                       const glm::vec3& center)
{
    GridLayout g;
    g.counts = glm::clamp(volume.probeCounts, glm::ivec3(1), glm::ivec3(kMaxProbesPerAxis));

    // Probes sit on a regular grid spanning the volume box, corner probes
    // exactly on the boundary. A single-probe axis degenerates to the centre.
    const glm::vec3 extent = glm::max(volume.extent, glm::vec3(0.01f));
    g.spacing = extent;
    for (int i = 0; i < 3; ++i)
        if (g.counts[i] > 1) g.spacing[i] = extent[i] / static_cast<float>(g.counts[i] - 1);

    g.origin = center - extent * 0.5f;
    return g;
}

void VulkanDDGIPass::SetFrameData(const DDGIVolumeComponent* volume,
                                  const glm::vec3& center,
                                  const glm::mat4& view, const glm::mat4& projection,
                                  float skyIntensity, float exposure)
{
    if (!Available()) return;

    m_HasVolume = volume != nullptr;
    if (!m_HasVolume) return;

    static thread_local std::mt19937 rng{ 0x5EED };

    const GridLayout grid   = ComputeGrid(*volume, center);
    const glm::ivec3 counts = grid.counts;
    const int rays = std::clamp(volume->raysPerProbe, 1, kMaxRays);

    // A different grid means the atlases describe probes that no longer exist —
    // blending the old contents in would drag stale irradiance across the scene.
    if (counts != m_Counts || rays != m_RayCount) InvalidateHistory();

    m_Counts     = counts;
    m_RayCount   = rays;
    m_ProbeCount = counts.x * counts.y * counts.z;
    m_ShowProbes = volume->showProbes;

    m_UBOData.view        = view;
    m_UBOData.viewProj    = projection * view;
    m_UBOData.rayRotation = RandomRotation(rng);
    m_UBOData.origin      = glm::vec4(grid.origin, 0.0f);
    m_UBOData.spacing     = glm::vec4(grid.spacing,
                                      std::min({ grid.spacing.x, grid.spacing.y,
                                                 grid.spacing.z }));
    m_UBOData.counts      = glm::ivec4(counts, m_ProbeCount);
    // Progressive average over the opening frames: blending frame N into the
    // running mean wants hysteresis N/(N+1), which is 0 on the first frame and
    // climbs toward the user's value. That converges as fast as the ray budget
    // allows, instead of taking ~1/(1-hysteresis) frames to crawl out from
    // whatever the atlas held. Once it reaches the user's value it stops
    // mattering, which is why there is no magic window length.
    const float progressive =
        1.0f - 1.0f / static_cast<float>(m_FramesSinceReset + 1);
    const float hysteresis =
        std::min(std::clamp(volume->hysteresis, 0.0f, 0.99f), progressive);

    m_UBOData.params      = glm::vec4(hysteresis,
                                      volume->normalBias,
                                      std::max(volume->energy, 0.0f),
                                      std::max(volume->maxRayDistance, 0.1f));
    m_UBOData.params2     = glm::vec4(skyIntensity, volume->viewBias, kDepthSharpness,
                                      static_cast<float>(rays));
    m_UBOData.params3     = glm::vec4(m_HistoryValid ? 1.0f : 0.0f,
                                      std::clamp(volume->backfaceThreshold, 0.01f, 1.0f),
                                      std::max(volume->probeRadius, 0.01f),
                                      exposure);

    m_UBO->Update(&m_UBOData, sizeof(DDGIUBO));
    ++m_FrameIndex;
}

void VulkanDDGIPass::SetEnabled(bool enabled)
{
    if (m_Enabled == enabled) return;
    m_Enabled = enabled;
    // The atlases went stale while the passes were skipped.
    InvalidateHistory();
}

} // namespace Diamond
