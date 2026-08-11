#include "Platform/Vulkan/Passes/PostProcess/VulkanAutoExposurePass.h"
#include "Platform/Vulkan/Passes/VulkanPassCommon.h"
#include "Renderer/RHI/RHIDevice.h"
#include "Renderer/RHI/RHICommandList.h"

#include <cmath>
#include <string>

namespace Diamond {

namespace {
// Single channel is all the chain carries: log2 luminance, then the multiplier.
constexpr RHIFormat kLumaFormat = RHIFormat::R16F;
} // namespace

VulkanAutoExposurePass::VulkanAutoExposurePass(RHIDevice* device, const std::string& shaderDir)
    : m_Device(device)
{
    // Retained adaptation value — single-buffered for the same reason as the TAA
    // history: frame N's write must be exactly what frame N+1 samples, which the
    // per-frame-in-flight duplication of a normal render target would stagger.
    RHITextureDesc adapted;
    adapted.width          = 1;
    adapted.height         = 1;
    adapted.format         = kLumaFormat;
    adapted.usage          = RHITextureUsage::Sampled | RHITextureUsage::ColorAttachment;
    adapted.singleBuffered = true;
    adapted.debugName      = "autoExposureAdapted";
    m_Adapted = device->CreateTexture(adapted);

    const std::vector<uint32_t> vs      = LoadSpirv(shaderDir, "fullscreen.vert.spv");
    m_FullscreenVert = device->CreateShader(
        RHIShaderDesc{ RHIShaderStage::Vertex, vs.data(), vs.size() });

    auto makeFrag = [&](const char* name) {
        const std::vector<uint32_t> fs = LoadSpirv(shaderDir, name);
        RHIShaderDesc d{ RHIShaderStage::Fragment, fs.data(), fs.size() };
        return device->CreateShader(d);
    };
    m_ExtractFrag = makeFrag("luma_extract.frag.spv");
    m_ReduceFrag  = makeFrag("luma_reduce.frag.spv");
    m_AdaptFrag   = makeFrag("exposure_adapt.frag.spv");
    m_CopyFrag    = makeFrag("copy.frag.spv");

    // Extract — HDR scene in, log-luminance out.
    {
        RHIPipelineDesc desc;
        desc.vertexShader     = m_FullscreenVert.get();
        desc.fragmentShader   = m_ExtractFrag.get();
        desc.resourceBindings = {
            { 0, RHIResourceType::CombinedImageSampler, RHIShaderStage::Fragment },
        };
        desc.colorFormat  = kLumaFormat;
        m_ExtractPipeline = device->CreatePipeline(desc);
    }
    // Reduce — one pipeline reused by every step; the step's source is the only
    // thing that differs, and that lives in the descriptor set.
    {
        RHIPipelineDesc desc;
        desc.vertexShader     = m_FullscreenVert.get();
        desc.fragmentShader   = m_ReduceFrag.get();
        desc.resourceBindings = {
            { 0, RHIResourceType::CombinedImageSampler, RHIShaderStage::Fragment },
        };
        desc.colorFormat = kLumaFormat;
        m_ReducePipeline = device->CreatePipeline(desc);
    }
    // Adapt — 1x1 scene luma + retained value in, MRT multiplier + new value out.
    {
        RHIPipelineDesc desc;
        desc.vertexShader     = m_FullscreenVert.get();
        desc.fragmentShader   = m_AdaptFrag.get();
        desc.resourceBindings = {
            { 0, RHIResourceType::CombinedImageSampler, RHIShaderStage::Fragment }, // scene luma
            { 1, RHIResourceType::CombinedImageSampler, RHIShaderStage::Fragment }, // retained
        };
        desc.pushConstants = { RHIShaderStage::Fragment, sizeof(AdaptPush) };
        // location 0 = exposure multiplier, 1 = adapted log-luminance.
        desc.colorFormats  = { kLumaFormat, kLumaFormat };
        m_AdaptPipeline    = device->CreatePipeline(desc);
    }
    // Retained-value copy — the shared passthrough shader.
    {
        RHIPipelineDesc desc;
        desc.vertexShader     = m_FullscreenVert.get();
        desc.fragmentShader   = m_CopyFrag.get();
        desc.resourceBindings = {
            { 0, RHIResourceType::CombinedImageSampler, RHIShaderStage::Fragment },
        };
        desc.colorFormat = kLumaFormat;
        m_CopyPipeline   = device->CreatePipeline(desc);
    }
}

VulkanAutoExposurePass::~VulkanAutoExposurePass() = default;

void VulkanAutoExposurePass::AddToGraph(RHIRenderGraph& graph, RGTextureHandle hdrScene,
                                        RGTextureHandle exposureOut)
{
    // ── Extract ─────────────────────────────────────────────────────────────────
    const RGTextureHandle extracted = graph.DeclareTexture(
        "autoExpLuma" + std::to_string(kExtractSize),
        { kExtractSize, kExtractSize, kLumaFormat });

    if (!m_ExtractSet)
        m_ExtractSet = m_Device->CreateResourceSet(
            m_ExtractPipeline.get(), 0, {}, { { 0, graph.GetTexture(hdrScene) } });

    graph.AddPass("AutoExposureExtract")
        .Read(hdrScene)
        .Write(extracted)
        .SetExecute([this](RHICommandList* cmd) {
            cmd->BindPipeline(m_ExtractPipeline.get());
            cmd->BindResourceSet(0, m_ExtractSet.get());
            cmd->Draw(3);
        });

    // ── Reduce to 1x1 ───────────────────────────────────────────────────────────
    RGTextureHandle src  = extracted;
    uint32_t        size = kExtractSize;
    for (int i = 0; i < kReduceSteps; ++i) {
        size /= 4;
        const RGTextureHandle dst = graph.DeclareTexture(
            "autoExpLuma" + std::to_string(size), { size, size, kLumaFormat });

        if (!m_ReduceSets[static_cast<size_t>(i)])
            m_ReduceSets[static_cast<size_t>(i)] = m_Device->CreateResourceSet(
                m_ReducePipeline.get(), 0, {}, { { 0, graph.GetTexture(src) } });
        RHIResourceSet* set = m_ReduceSets[static_cast<size_t>(i)].get();

        graph.AddPass("AutoExposureReduce")
            .Read(src)
            .Write(dst)
            .SetExecute([this, set](RHICommandList* cmd) {
                cmd->BindPipeline(m_ReducePipeline.get());
                cmd->BindResourceSet(0, set);
                cmd->Draw(3);
            });

        src = dst;
    }

    // ── Adapt ───────────────────────────────────────────────────────────────────
    // The new adapted value needs its own target so the post-graph copy sees it;
    // exposureOut is what downstream samples. Neither may be the retained image.
    const RGTextureHandle adaptedSrc =
        graph.DeclareTexture("autoExpAdaptedSrc", { 1, 1, kLumaFormat });

    if (!m_AdaptSet)
        m_AdaptSet = m_Device->CreateResourceSet(
            m_AdaptPipeline.get(), 0, {},
            { { 0, graph.GetTexture(src) },
              { 1, m_Adapted.get() } });

    m_AdaptedSrc = graph.GetTexture(adaptedSrc);
    if (!m_CopySet)
        m_CopySet = m_Device->CreateResourceSet(
            m_CopyPipeline.get(), 0, {}, { { 0, m_AdaptedSrc } });

    // adaptedSrc's only consumer is RecordAdaptedCopy, outside the graph — mark
    // it so culling can never drop the adapt pass and leave the copy reading a
    // stale image (the same reason TAA marks its historySrc).
    graph.MarkOutput(adaptedSrc);

    // Writes in frag `location` order: 0 = exposureOut, 1 = adaptedSrc.
    graph.AddPass("AutoExposureAdapt")
        .Read(src)
        .Write(exposureOut)
        .Write(adaptedSrc)
        .SetExecute([this](RHICommandList* cmd) {
            AdaptPush push{};
            // Frame-rate independent exponential ease. Without valid history
            // (first frame, just re-enabled, post-resize rebuild) snap instead,
            // so the retained value seeds from the scene rather than easing out
            // of whatever the image happened to hold.
            push.alpha = m_HistoryValid
                ? 1.0f - std::exp(-m_DeltaTime * m_Speed)
                : 1.0f;
            push.keyValue    = m_KeyValue;
            push.minLogLuma  = m_MinLogLuma;
            push.maxLogLuma  = m_MaxLogLuma;
            push.autoEnabled = m_Enabled ? 1 : 0;

            cmd->BindPipeline(m_AdaptPipeline.get());
            cmd->BindResourceSet(0, m_AdaptSet.get());
            cmd->PushConstants(RHIShaderStage::Fragment, 0, sizeof(push), &push);
            cmd->Draw(3);
        });
}

void VulkanAutoExposurePass::Prepare(RHICommandList* cmd)
{
    // The adapt pass's set binds the retained image from frame one, and Vulkan
    // requires a sampled image in a valid layout even where a uniform branch
    // never reads it — seed Undefined -> SampledRead once.
    if (m_Prepared) return;
    cmd->TransitionTexture(m_Adapted.get(), RHITextureState::SampledRead);
    m_Prepared = true;
}

void VulkanAutoExposurePass::RecordAdaptedCopy(RHICommandList* cmd)
{
    // Outside any render scope, after graph.Execute. adaptedSrc has no in-graph
    // reader, so it is still in ColorTarget layout from the adapt pass — move it
    // to SampledRead for the copy's sampler, and the retained image to
    // ColorTarget so it can be written.
    cmd->TransitionTexture(m_AdaptedSrc, RHITextureState::SampledRead);
    cmd->TransitionTexture(m_Adapted.get(), RHITextureState::ColorTarget);

    RHIRenderPass pass;
    RHIAttachment color;
    color.texture = m_Adapted.get();
    color.clear   = false;   // the fullscreen copy overwrites the single texel
    pass.colorAttachments.push_back(color);
    cmd->BeginRendering(pass);
    cmd->BindPipeline(m_CopyPipeline.get());
    cmd->BindResourceSet(0, m_CopySet.get());
    cmd->Draw(3);
    cmd->EndRendering();

    cmd->TransitionTexture(m_Adapted.get(), RHITextureState::SampledRead);
    m_HistoryValid = true;
}

} // namespace Diamond
