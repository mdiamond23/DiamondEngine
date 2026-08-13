#include "Platform/Vulkan/Passes/Deferred/VulkanSSGIPass.h"
#include "Platform/Vulkan/Passes/VulkanPassCommon.h"
#include "Renderer/RHI/RHIDevice.h"
#include "Renderer/RHI/RHICommandList.h"

#include <algorithm>
#include <cstdint>
#include <vector>

namespace Diamond {

namespace {
constexpr RHIFormat kSSGIFormat = RHIFormat::RGBA16F;  // rgb = irradiance, a = confidence
constexpr uint32_t  kLocalSize  = 8;                   // matches both .comp local_size
} // namespace

VulkanSSGIPass::VulkanSSGIPass(RHIDevice* device, const std::string& shaderDir,
                               uint32_t width, uint32_t height)
    : m_Device(device),
      m_HalfW(std::max(1u, width  / 2)),
      m_HalfH(std::max(1u, height / 2))
{
    m_UBOData.halfSize = glm::vec2(m_HalfW, m_HalfH);

    RHIBufferDesc uboDesc;
    uboDesc.size    = sizeof(SSGIUBO);
    uboDesc.usage   = RHIBufferUsage::Uniform;
    uboDesc.dynamic = true;
    m_UBO = device->CreateBuffer(uboDesc);

    const std::vector<uint32_t> vs        = LoadSpirv(shaderDir, "fullscreen.vert.spv");
    const std::vector<uint32_t> trace     = LoadSpirv(shaderDir, "ssgi.comp.spv");
    const std::vector<uint32_t> temporal  = LoadSpirv(shaderDir, "ssgi_temporal.comp.spv");
    const std::vector<uint32_t> composite = LoadSpirv(shaderDir, "ssgi_composite.frag.spv");
    const std::vector<uint32_t> copy      = LoadSpirv(shaderDir, "copy.frag.spv");

    RHIShaderDesc vsDesc{ RHIShaderStage::Vertex,   vs.data(),        vs.size() };
    RHIShaderDesc trDesc{ RHIShaderStage::Compute,  trace.data(),     trace.size() };
    RHIShaderDesc tmDesc{ RHIShaderStage::Compute,  temporal.data(),  temporal.size() };
    RHIShaderDesc cpDesc{ RHIShaderStage::Fragment, composite.data(), composite.size() };
    RHIShaderDesc cyDesc{ RHIShaderStage::Fragment, copy.data(),      copy.size() };

    m_FullscreenVert = device->CreateShader(vsDesc);
    m_TraceComp      = device->CreateShader(trDesc);
    m_TemporalComp   = device->CreateShader(tmDesc);
    m_CompositeFrag  = device->CreateShader(cpDesc);
    m_CopyFrag       = device->CreateShader(cyDesc);

    // ── Trace (compute) ──────────────────────────────────────────────────────
    {
        RHIComputePipelineDesc desc{};
        desc.computeShader    = m_TraceComp.get();
        desc.resourceBindings = {
            { 0, RHIResourceType::StorageImage,         RHIShaderStage::Compute }, // ssgiRaw
            { 1, RHIResourceType::CombinedImageSampler, RHIShaderStage::Compute }, // gViewPos
            { 2, RHIResourceType::CombinedImageSampler, RHIShaderStage::Compute }, // gViewNormal
            { 3, RHIResourceType::CombinedImageSampler, RHIShaderStage::Compute }, // sceneColor
            { 4, RHIResourceType::UniformBuffer,        RHIShaderStage::Compute }, // SSGIUBO
        };
        desc.pushConstants = { RHIShaderStage::Compute, sizeof(TracePush) };
        m_TracePipeline = device->CreateComputePipeline(desc);
    }

    // ── Temporal (compute) ───────────────────────────────────────────────────
    // Dimensions come from textureSize, so no UBO here.
    {
        RHIComputePipelineDesc desc{};
        desc.computeShader    = m_TemporalComp.get();
        desc.resourceBindings = {
            { 0, RHIResourceType::StorageImage,         RHIShaderStage::Compute }, // ssgiResolved
            { 1, RHIResourceType::CombinedImageSampler, RHIShaderStage::Compute }, // ssgiRaw
            { 2, RHIResourceType::CombinedImageSampler, RHIShaderStage::Compute }, // ssgiHistory
            { 3, RHIResourceType::CombinedImageSampler, RHIShaderStage::Compute }, // gVelocity
        };
        desc.pushConstants = { RHIShaderStage::Compute, sizeof(TemporalPush) };
        m_TemporalPipeline = device->CreateComputePipeline(desc);
    }

    // ── Composite (raster, full res) ─────────────────────────────────────────
    {
        RHIPipelineDesc desc;
        desc.vertexShader     = m_FullscreenVert.get();
        desc.fragmentShader   = m_CompositeFrag.get();
        desc.resourceBindings = {
            { 0, RHIResourceType::CombinedImageSampler, RHIShaderStage::Fragment }, // sceneColor
            { 1, RHIResourceType::CombinedImageSampler, RHIShaderStage::Fragment }, // ssgiResolved
            { 2, RHIResourceType::CombinedImageSampler, RHIShaderStage::Fragment }, // gViewPos
            { 3, RHIResourceType::CombinedImageSampler, RHIShaderStage::Fragment }, // gViewNormal
            { 4, RHIResourceType::CombinedImageSampler, RHIShaderStage::Fragment }, // gAlbedo
            { 5, RHIResourceType::CombinedImageSampler, RHIShaderStage::Fragment }, // gMaterial
            { 6, RHIResourceType::CombinedImageSampler, RHIShaderStage::Fragment }, // ao (unused)
            { 7, RHIResourceType::CombinedImageSampler, RHIShaderStage::Fragment }, // gIndirect
        };
        desc.pushConstants = { RHIShaderStage::Fragment, sizeof(CompositePush) };
        desc.colorFormat   = kSSGIFormat;
        m_CompositePipeline = device->CreatePipeline(desc);
    }

    // ── History copy (raster, half res) ──────────────────────────────────────
    {
        RHIPipelineDesc desc;
        desc.vertexShader     = m_FullscreenVert.get();
        desc.fragmentShader   = m_CopyFrag.get();
        desc.resourceBindings = {
            { 0, RHIResourceType::CombinedImageSampler, RHIShaderStage::Fragment }, // uSource
        };
        desc.colorFormat = kSSGIFormat;
        m_CopyPipeline = device->CreatePipeline(desc);
    }
}

VulkanSSGIPass::~VulkanSSGIPass() = default;

void VulkanSSGIPass::AddToGraph(RHIRenderGraph& graph,
                                RGTextureHandle viewPos, RGTextureHandle viewNormal,
                                RGTextureHandle sceneColor, RGTextureHandle velocity,
                                RGTextureHandle albedo, RGTextureHandle material,
                                RGTextureHandle ao,   RGTextureHandle indirect,
                                RGTextureHandle ssgiRaw, RGTextureHandle ssgiResolved,
                                RGTextureHandle ssgiHistory, RGTextureHandle outColor)
{
    // Sets are built once — graph textures keep their identity across frames.
    // Note ssgiRaw/ssgiResolved bind as STORAGE images in the pass that writes
    // them and as samplers in the pass that reads them; never both in one set,
    // which would need two image layouts at once.
    if (!m_TraceSet)
        m_TraceSet = m_Device->CreateResourceSet(
            m_TracePipeline.get(), 0, { { 4, m_UBO.get() } },
            { { 0, graph.GetTexture(ssgiRaw) },
              { 1, graph.GetTexture(viewPos) },
              { 2, graph.GetTexture(viewNormal) },
              { 3, graph.GetTexture(sceneColor) } });

    if (!m_TemporalSet)
        m_TemporalSet = m_Device->CreateResourceSet(
            m_TemporalPipeline.get(), 0, {},
            { { 0, graph.GetTexture(ssgiResolved) },
              { 1, graph.GetTexture(ssgiRaw) },
              { 2, graph.GetTexture(ssgiHistory) },
              { 3, graph.GetTexture(velocity) } });

    if (!m_CopySet)
        m_CopySet = m_Device->CreateResourceSet(
            m_CopyPipeline.get(), 0, {},
            { { 0, graph.GetTexture(ssgiResolved) } });

    if (!m_CompositeSet)
        m_CompositeSet = m_Device->CreateResourceSet(
            m_CompositePipeline.get(), 0, {},
            { { 0, graph.GetTexture(sceneColor) },
              { 1, graph.GetTexture(ssgiResolved) },
              { 2, graph.GetTexture(viewPos) },
              { 3, graph.GetTexture(viewNormal) },
              { 4, graph.GetTexture(albedo) },
              { 5, graph.GetTexture(material) },
              { 6, graph.GetTexture(ao) },
              { 7, graph.GetTexture(indirect) } });

    const uint32_t groupsX = (m_HalfW + kLocalSize - 1) / kLocalSize;
    const uint32_t groupsY = (m_HalfH + kLocalSize - 1) / kLocalSize;

    graph.AddPass("SSGITrace").AsCompute()
        .Read(viewPos).Read(viewNormal).Read(sceneColor)
        .Write(ssgiRaw)
        .SetExecute([this, groupsX, groupsY](RHICommandList* cmd) {
            if (!m_Enabled) return;   // skip the expensive dispatch outright
            TracePush push{};
            push.frameIndex  = m_FrameIndex++;
            push.rayCount    = m_RayCount;
            push.maxDistance = m_MaxDistance;
            push.intensity   = m_Intensity;
            cmd->BindPipeline(m_TracePipeline.get());
            cmd->BindResourceSet(0, m_TraceSet.get());
            cmd->PushConstants(RHIShaderStage::Compute, 0, sizeof(push), &push);
            cmd->Dispatch(groupsX, groupsY, 1);
        });

    // ReadHistory, not Read: it wants LAST frame's contents, so it must not
    // depend on this frame's copy pass — that would be a cycle.
    graph.AddPass("SSGITemporal").AsCompute()
        .Read(ssgiRaw).Read(velocity)
        .ReadHistory(ssgiHistory)
        .Write(ssgiResolved)
        .SetExecute([this, groupsX, groupsY](RHICommandList* cmd) {
            if (!m_Enabled) return;
            TemporalPush push{};
            push.alpha = m_HistoryValid ? m_Alpha : 1.0f;
            cmd->BindPipeline(m_TemporalPipeline.get());
            cmd->BindResourceSet(0, m_TemporalSet.get());
            cmd->PushConstants(RHIShaderStage::Compute, 0, sizeof(push), &push);
            cmd->Dispatch(groupsX, groupsY, 1);
        });

    graph.AddPass("SSGIHistoryCopy")
        .Read(ssgiResolved).Write(ssgiHistory)
        .SetExecute([this](RHICommandList* cmd) {
            if (!m_Enabled) return;
            cmd->BindPipeline(m_CopyPipeline.get());
            cmd->BindResourceSet(0, m_CopySet.get());
            cmd->Draw(3);
            m_HistoryValid = true;   // next frame's temporal may blend
        });

    graph.AddPass("SSGIComposite")
        .Read(sceneColor).Read(ssgiResolved).Read(viewPos).Read(viewNormal)
        .Read(albedo).Read(material).Read(ao).Read(indirect)
        .Write(outColor)
        .SetExecute([this](RHICommandList* cmd) {
            CompositePush push{};
            push.strength = m_Enabled ? 1.0f : 0.0f;
            cmd->BindPipeline(m_CompositePipeline.get());
            cmd->BindResourceSet(0, m_CompositeSet.get());
            cmd->PushConstants(RHIShaderStage::Fragment, 0, sizeof(push), &push);
            cmd->Draw(3);
        });
}

void VulkanSSGIPass::SetProjection(const glm::mat4& projection)
{
    m_UBOData.projection = projection;
    m_UBO->Update(&m_UBOData, sizeof(SSGIUBO));
}

void VulkanSSGIPass::SetEnabled(bool enabled)
{
    if (m_Enabled == enabled) return;
    m_Enabled = enabled;
    // Whatever is in the history predates the gap — reseed on re-enable.
    m_HistoryValid = false;
}

void VulkanSSGIPass::SetRayCount(int rays)          { m_RayCount = std::max(1, rays); }
void VulkanSSGIPass::SetIntensity(float intensity)  { m_Intensity = std::max(0.0f, intensity); }
void VulkanSSGIPass::SetMaxDistance(float distance) { m_MaxDistance = std::max(0.01f, distance); }
void VulkanSSGIPass::SetBlendFactor(float alpha)    { m_Alpha = std::clamp(alpha, 0.01f, 1.0f); }

} // namespace Diamond
