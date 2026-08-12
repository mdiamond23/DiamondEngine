#include "Platform/Vulkan/Passes/PostProcess/VulkanTAAPass.h"
#include "Platform/Vulkan/Passes/VulkanPassCommon.h"
#include "Renderer/RHI/RHIDevice.h"
#include "Renderer/RHI/RHICommandList.h"

namespace Diamond {

namespace {
constexpr RHIFormat kHDRFormat = RHIFormat::RGBA16F;   // scene chain + history
} // namespace

VulkanTAAPass::VulkanTAAPass(RHIDevice* device, const std::string& shaderDir)
    : m_Device(device)
{
    const std::vector<uint32_t> vs   = LoadSpirv(shaderDir, "fullscreen.vert.spv");
    const std::vector<uint32_t> fs   = LoadSpirv(shaderDir, "taa_resolve.frag.spv");
    const std::vector<uint32_t> copy = LoadSpirv(shaderDir, "copy.frag.spv");

    RHIShaderDesc vsDesc{ RHIShaderStage::Vertex,   vs.data(),   vs.size() };
    RHIShaderDesc fsDesc{ RHIShaderStage::Fragment, fs.data(),   fs.size() };
    RHIShaderDesc cpDesc{ RHIShaderStage::Fragment, copy.data(), copy.size() };
    m_FullscreenVert = device->CreateShader(vsDesc);
    m_ResolveFrag    = device->CreateShader(fsDesc);
    m_CopyFrag       = device->CreateShader(cpDesc);

    // Resolve: fullscreen, samples scene + velocity + history.
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
    // History copy: the shared passthrough copy.frag, historySrc -> history.
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
                               RGTextureHandle outColor, RGTextureHandle historySrc,
                               RGTextureHandle history) {
    // Sets are built once — graph textures keep their identity across frames.
    // (A resize recreates the whole pass, so these rebuild against the new
    // targets rather than going stale.)
    if (!m_ResolveSet)
        m_ResolveSet = m_Device->CreateResourceSet(
            m_ResolvePipeline.get(), 0, {},
            { { 0, graph.GetTexture(sceneColor) },
              { 1, graph.GetTexture(velocity) },
              { 2, graph.GetTexture(history) } });

    if (!m_CopySet)
        m_CopySet = m_Device->CreateResourceSet(
            m_CopyPipeline.get(), 0, {},
            { { 0, graph.GetTexture(historySrc) } });

    // Writes in frag `location` order: 0 = outColor, 1 = historySrc. The history
    // is a ReadHistory, not a Read: its value is last frame's, so depending on
    // this frame's copy pass (which reads historySrc, written here) would close
    // a cycle. The graph transitions it for sampling either way.
    graph.AddPass("TAAResolve")
        .Read(sceneColor).Read(velocity)
        .ReadHistory(history)
        .Write(outColor)
        .Write(historySrc)
        .SetExecute([this](RHICommandList* cmd) {
            // Invalid history (first frame, TAA just re-enabled, post-resize
            // pass rebuild) -> alpha 1.0 -> the shader passes the current
            // frame straight through and never dereferences the history.
            const float alpha = (m_Enabled && m_HistoryValid) ? m_Alpha : 1.0f;
            cmd->BindPipeline(m_ResolvePipeline.get());
            cmd->BindResourceSet(0, m_ResolveSet.get());
            cmd->PushConstants(RHIShaderStage::Fragment, 0, sizeof(float), &alpha);
            cmd->Draw(3);
        });

    // The pass writes a persistent target, so the graph keeps it (and the
    // resolve upstream of it) alive even though nothing reads the history with
    // a real dependency edge — next frame's resolve is the consumer.
    graph.AddPass("TAAHistoryCopy")
        .Read(historySrc)
        .Write(history)
        .Load()   // the fullscreen copy overwrites every pixel
        .SetExecute([this](RHICommandList* cmd) {
            // Disabled: skip the draw so the accumulation stops and re-enabling
            // self-seeds from a passthrough frame instead of blending against
            // stale history. The empty render scope is all that remains of the
            // cost while off.
            if (!m_Enabled) {
                m_HistoryValid = false;
                return;
            }
            cmd->BindPipeline(m_CopyPipeline.get());
            cmd->BindResourceSet(0, m_CopySet.get());
            cmd->Draw(3);
            m_HistoryValid = true;
        });
}

} // namespace Diamond
