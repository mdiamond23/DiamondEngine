#include "Platform/Vulkan/Passes/PostProcess/VulkanTAAPass.h"
#include "Platform/Vulkan/Passes/VulkanPassCommon.h"
#include "Renderer/RHI/RHIDevice.h"
#include "Renderer/RHI/RHICommandList.h"

namespace Diamond {

namespace {
constexpr RHIFormat kHDRFormat = RHIFormat::RGBA16F;   // scene chain + history
} // namespace

VulkanTAAPass::VulkanTAAPass(RHIDevice* device, const std::string& shaderDir,
                             uint32_t width, uint32_t height)
    : m_Device(device)
{
    // History — single-buffered on purpose (see the class comment): frame N's
    // copy must be exactly what frame N+1 samples, and the per-frame-in-flight
    // duplication of a normal render target would stagger that to two frames.
    RHITextureDesc hist;
    hist.width          = width;
    hist.height         = height;
    hist.format         = kHDRFormat;
    hist.usage          = RHITextureUsage::Sampled | RHITextureUsage::ColorAttachment;
    hist.singleBuffered = true;
    hist.debugName      = "taaHistory";
    m_History = device->CreateTexture(hist);

    const std::vector<uint32_t> vs   = LoadSpirv(shaderDir, "fullscreen.vert.spv");
    const std::vector<uint32_t> fs   = LoadSpirv(shaderDir, "taa_resolve.frag.spv");
    const std::vector<uint32_t> copy = LoadSpirv(shaderDir, "copy.frag.spv");

    RHIShaderDesc vsDesc{ RHIShaderStage::Vertex,   vs.data(),   vs.size() };
    RHIShaderDesc fsDesc{ RHIShaderStage::Fragment, fs.data(),   fs.size() };
    RHIShaderDesc cpDesc{ RHIShaderStage::Fragment, copy.data(), copy.size() };
    m_FullscreenVert = device->CreateShader(vsDesc);
    m_ResolveFrag    = device->CreateShader(fsDesc);
    m_CopyFrag       = device->CreateShader(cpDesc);

    // Resolve: fullscreen, samples scene + velocity (graph) and history (raw).
    {
        RHIPipelineDesc desc;
        desc.vertexShader     = m_FullscreenVert.get();
        desc.fragmentShader   = m_ResolveFrag.get();
        desc.resourceBindings = {
            { 0, RHIResourceType::CombinedImageSampler, RHIShaderStage::Fragment }, // uScene
            { 1, RHIResourceType::CombinedImageSampler, RHIShaderStage::Fragment }, // uVelocity
            { 2, RHIResourceType::CombinedImageSampler, RHIShaderStage::Fragment }, // uHistory
        };
        desc.pushConstants = { RHIShaderStage::Fragment, sizeof(float) };   // alpha
        // MRT: location 0 = outColor (frame chain), 1 = historySrc (copy source).
        desc.colorFormats  = { kHDRFormat, kHDRFormat };
        m_ResolvePipeline  = device->CreatePipeline(desc);
    }
    // History copy: the shared passthrough copy.frag, resolved output -> history.
    {
        RHIPipelineDesc desc;
        desc.vertexShader     = m_FullscreenVert.get();
        desc.fragmentShader   = m_CopyFrag.get();
        desc.resourceBindings = {
            { 0, RHIResourceType::CombinedImageSampler, RHIShaderStage::Fragment }, // uSource
        };
        desc.colorFormat = kHDRFormat;
        m_CopyPipeline   = device->CreatePipeline(desc);
    }
}

VulkanTAAPass::~VulkanTAAPass() = default;

void VulkanTAAPass::AddToGraph(RHIRenderGraph& graph,
                               RGTextureHandle sceneColor, RGTextureHandle velocity,
                               RGTextureHandle outColor, RGTextureHandle historySrc) {
    // Sets are built once — graph textures keep their identity across frames,
    // and the history is pass-owned. The history is NOT a declared graph read:
    // its transitions are recorded manually in Prepare/RecordHistoryCopy.
    if (!m_ResolveSet)
        m_ResolveSet = m_Device->CreateResourceSet(
            m_ResolvePipeline.get(), 0, {},
            { { 0, graph.GetTexture(sceneColor) },
              { 1, graph.GetTexture(velocity) },
              { 2, m_History.get() } });

    m_HistorySrc = graph.GetTexture(historySrc);
    if (!m_CopySet)
        m_CopySet = m_Device->CreateResourceSet(
            m_CopyPipeline.get(), 0, {},
            { { 0, m_HistorySrc } });

    // Writes in frag `location` order: 0 = outColor, 1 = historySrc.
    graph.AddPass("TAAResolve")
        .Read(sceneColor).Read(velocity)
        .Write(outColor)
        .Write(historySrc)
        .SetExecute([this](RHICommandList* cmd) {
            // Invalid history (first frame, TAA just re-enabled, post-resize
            // pass rebuild) -> alpha 1.0 -> the shader passes the current
            // frame straight through and never dereferences the history.
            const float alpha = m_HistoryValid ? m_Alpha : 1.0f;
            cmd->BindPipeline(m_ResolvePipeline.get());
            cmd->BindResourceSet(0, m_ResolveSet.get());
            cmd->PushConstants(RHIShaderStage::Fragment, 0, sizeof(float), &alpha);
            cmd->Draw(3);
        });
}

void VulkanTAAPass::Prepare(RHICommandList* cmd) {
    // The resolve's descriptor set binds the history from frame one, and Vulkan
    // requires a sampled image in a valid layout even when the shader's uniform
    // branch never reads it — seed Undefined -> SampledRead once.
    if (m_Prepared) return;
    cmd->TransitionTexture(m_History.get(), RHITextureState::SampledRead);
    m_Prepared = true;
}

void VulkanTAAPass::RecordHistoryCopy(RHICommandList* cmd) {
    // Outside any render scope, after graph.Execute. historySrc has no in-graph
    // reader, so it's still in ColorTarget layout from the resolve — move it to
    // SampledRead for the copy's sampler, and the history to ColorTarget.
    cmd->TransitionTexture(m_HistorySrc, RHITextureState::SampledRead);
    cmd->TransitionTexture(m_History.get(), RHITextureState::ColorTarget);

    RHIRenderPass pass;
    RHIAttachment color;
    color.texture = m_History.get();
    color.clear   = false;   // every pixel is overwritten by the fullscreen copy
    pass.colorAttachments.push_back(color);
    cmd->BeginRendering(pass);
    cmd->BindPipeline(m_CopyPipeline.get());
    cmd->BindResourceSet(0, m_CopySet.get());
    cmd->Draw(3);
    cmd->EndRendering();

    cmd->TransitionTexture(m_History.get(), RHITextureState::SampledRead);
    m_HistoryValid = true;
}

} // namespace Diamond
