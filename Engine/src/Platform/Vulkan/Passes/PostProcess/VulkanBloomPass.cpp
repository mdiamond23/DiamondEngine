#include "Platform/Vulkan/Passes/PostProcess/VulkanBloomPass.h"
#include "Platform/Vulkan/Passes/VulkanPassCommon.h"
#include "Renderer/RHI/RHIDevice.h"
#include "Renderer/RHI/RHICommandList.h"

#include <array>

namespace Diamond {

namespace {
// Scratch targets are HDR so bright values survive the blur before tonemapping.
constexpr RHIFormat kScratchFormat = RHIFormat::RGBA16F;
// Number of separable-blur half-steps (alternating H/V). Even, so the final
// result lands in the second ping-pong target. Matches the GL renderer's 6.
constexpr int       kBlurPasses    = 6;
} // namespace

VulkanBloomPass::VulkanBloomPass(RHIDevice* device, const std::string& shaderDir,
                                 uint32_t width, uint32_t height, RHIFormat outputFormat)
    : m_Device(device), m_Width(width), m_Height(height)
{
    const std::vector<uint32_t> vs = LoadSpirv(shaderDir, "fullscreen.vert.spv");
    RHIShaderDesc vsDesc{ RHIShaderStage::Vertex, vs.data(), vs.size() };
    m_Vert = device->CreateShader(vsDesc);

    auto makeFrag = [&](const char* name) {
        const std::vector<uint32_t> fs = LoadSpirv(shaderDir, name);
        RHIShaderDesc d{ RHIShaderStage::Fragment, fs.data(), fs.size() };
        return device->CreateShader(d);
    };
    m_ExtractFrag   = makeFrag("bloom_extract.frag.spv");
    m_BlurFrag      = makeFrag("bloom_blur.frag.spv");
    m_CompositeFrag = makeFrag("bloom_composite.frag.spv");

    // Bright-pass extract — one sampler (the HDR scene), HDR scratch output.
    {
        RHIPipelineDesc desc;
        desc.vertexShader     = m_Vert.get();
        desc.fragmentShader   = m_ExtractFrag.get();
        desc.resourceBindings = {
            { 0, RHIResourceType::CombinedImageSampler, RHIShaderStage::Fragment }, // scene
            { 1, RHIResourceType::CombinedImageSampler, RHIShaderStage::Fragment }, // exposure
        };
        desc.pushConstants = { RHIShaderStage::Fragment, sizeof(ExtractPush) };
        desc.colorFormat   = kScratchFormat;
        m_ExtractPipeline  = device->CreatePipeline(desc);
    }

    // Blur — one sampler + a push constant selecting the axis. Reused every step.
    {
        RHIPipelineDesc desc;
        desc.vertexShader     = m_Vert.get();
        desc.fragmentShader   = m_BlurFrag.get();
        desc.resourceBindings = {
            { 0, RHIResourceType::CombinedImageSampler, RHIShaderStage::Fragment },
        };
        desc.pushConstants = { RHIShaderStage::Fragment, sizeof(int) };
        desc.colorFormat   = kScratchFormat;
        m_BlurPipeline = device->CreatePipeline(desc);
    }

    // Composite — two samplers (scene + bloom), writing the final output target.
    {
        RHIPipelineDesc desc;
        desc.vertexShader     = m_Vert.get();
        desc.fragmentShader   = m_CompositeFrag.get();
        desc.resourceBindings = {
            { 0, RHIResourceType::CombinedImageSampler, RHIShaderStage::Fragment },
            { 1, RHIResourceType::CombinedImageSampler, RHIShaderStage::Fragment },
        };
        desc.pushConstants  = { RHIShaderStage::Fragment, sizeof(float) };   // intensity
        desc.colorFormat    = outputFormat;
        m_CompositePipeline = device->CreatePipeline(desc);
    }
}

VulkanBloomPass::~VulkanBloomPass() = default;

void VulkanBloomPass::AddToGraph(RHIRenderGraph& graph, RGTextureHandle hdrInput,
                                 RGTextureHandle exposureTex, RGTextureHandle output)
{
    const RGTextureDesc scratch{ m_Width, m_Height, kScratchFormat };
    const RGTextureHandle bright = graph.DeclareTexture("BloomBright", scratch);

    // One target per blur step, not a two-target ping-pong. The graph points every
    // reader at a texture's LAST writer, so alternating two targets would make step
    // 1 (reading ping[0]) depend on step 4 (ping[0]'s last writer) while step 4
    // depends on step 5 — a cycle Compile() rejects outright. Unique targets keep
    // each texture single-writer and the chain strictly linear.
    std::array<RGTextureHandle, kBlurPasses> steps{};
    for (int i = 0; i < kBlurPasses; ++i)
        steps[i] = graph.DeclareTexture("BloomBlur" + std::to_string(i), scratch);

    // ── Bright-pass extract ─────────────────────────────────────────────────────
    if (!m_ExtractSet)
        m_ExtractSet = m_Device->CreateResourceSet(
            m_ExtractPipeline.get(), 0, {},
            { { 0, graph.GetTexture(hdrInput) },
              { 1, graph.GetTexture(exposureTex) } });

    graph.AddPass("BloomExtract")
        .Read(hdrInput)
        .Read(exposureTex)
        .Write(bright)
        .SetExecute([this](RHICommandList* cmd) {
            const ExtractPush push{ m_Threshold, m_Exposure };
            cmd->BindPipeline(m_ExtractPipeline.get());
            cmd->BindResourceSet(0, m_ExtractSet.get());
            cmd->PushConstants(RHIShaderStage::Fragment, 0, sizeof(push), &push);
            cmd->Draw(3);
        });

    // ── Separable blur chain ────────────────────────────────────────────────────
    // Step i reads the previous step's output (step 0 reads the bright pass) and
    // writes its own target. The axis alternates, horizontal first. Each step gets
    // its own set bound to its source.
    const bool buildSets = m_BlurSets.empty();
    for (int i = 0; i < kBlurPasses; ++i) {
        const RGTextureHandle dst = steps[i];
        const RGTextureHandle src = (i == 0) ? bright : steps[i - 1];
        const int horizontal = (i % 2 == 0) ? 1 : 0;

        if (buildSets)
            m_BlurSets.push_back(m_Device->CreateResourceSet(
                m_BlurPipeline.get(), 0, {}, { { 0, graph.GetTexture(src) } }));
        RHIResourceSet* set = m_BlurSets[static_cast<size_t>(i)].get();

        graph.AddPass("BloomBlur")
            .Read(src)
            .Write(dst)
            .SetExecute([this, set, horizontal](RHICommandList* cmd) {
                cmd->BindPipeline(m_BlurPipeline.get());
                cmd->BindResourceSet(0, set);
                cmd->PushConstants(RHIShaderStage::Fragment, 0, sizeof(int), &horizontal);
                cmd->Draw(3);
            });
    }

    // Final blurred bloom is the last step's target. kBlurPasses is even, so the
    // chain ends on a vertical pass and the Gaussian is separable-complete.
    const RGTextureHandle blurred = steps[kBlurPasses - 1];

    // ── Additive composite (scene + bloom) → output ─────────────────────────────
    if (!m_CompositeSet)
        m_CompositeSet = m_Device->CreateResourceSet(
            m_CompositePipeline.get(), 0, {},
            { { 0, graph.GetTexture(hdrInput) }, { 1, graph.GetTexture(blurred) } });

    RGPass& pass = graph.AddPass("BloomComposite").Read(hdrInput).Read(blurred);
    if (output.IsValid()) pass.Write(output);
    else                  pass.WriteSwapchain();
    pass.SetExecute([this](RHICommandList* cmd) {
        cmd->BindPipeline(m_CompositePipeline.get());
        cmd->BindResourceSet(0, m_CompositeSet.get());
        cmd->PushConstants(RHIShaderStage::Fragment, 0, sizeof(float), &m_Intensity);
        cmd->Draw(3);
    });
}

} // namespace Diamond
