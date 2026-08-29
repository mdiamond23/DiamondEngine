#include "Platform/Vulkan/Passes/Deferred/VulkanSSRPass.h"
#include "Platform/Vulkan/Passes/VulkanPassCommon.h"
#include "Renderer/RHI/RHIDevice.h"
#include "Renderer/RHI/RHICommandList.h"

#include <cstdint>
#include <vector>

namespace Diamond {

namespace {
// ssrHit: rg = the hit's screen UV RELATIVE to the pixel that traced it,
// b = confidence. Relative is what makes RGBA16F viable — half floats carry
// relative precision, so an absolute UV would quantise to ~2 full-res texels at
// 4K and the reflected image would snap rather than slide as the camera moved.
// ssrColor: rgb = reflection colour, a = hit confidence.
constexpr RHIFormat kSSRFormat = RHIFormat::RGBA16F;
} // namespace

VulkanSSRPass::VulkanSSRPass(RHIDevice* device, const std::string& shaderDir, uint32_t width, uint32_t height)
    : m_Device(device) {

    m_UBOData.fullSize  = glm::vec2(width, height);
    m_UBOData.traceSize = glm::vec2(TraceDim(width), TraceDim(height));

    // Dynamic UBO — the sizes are filled now, projection per frame.
    RHIBufferDesc uboDesc;
    uboDesc.size    = sizeof(SSRUBO);
    uboDesc.usage   = RHIBufferUsage::Uniform;
    uboDesc.dynamic = true;
    m_UBO = device->CreateBuffer(uboDesc);

    // ── Pipelines ────────────────────────────────────────────────────────────
    const std::vector<uint32_t> vs           = LoadSpirv(shaderDir, "fullscreen.vert.spv");
    const std::vector<uint32_t> ssr          = LoadSpirv(shaderDir, "ssr.frag.spv");
    const std::vector<uint32_t> ssrResolve   = LoadSpirv(shaderDir, "ssr_resolve.frag.spv");
    const std::vector<uint32_t> ssrComposite = LoadSpirv(shaderDir, "ssr_composite.frag.spv");

    RHIShaderDesc vsDesc{ RHIShaderStage::Vertex,   vs.data(),  vs.size() };
    RHIShaderDesc ssrDesc{ RHIShaderStage::Fragment, ssr.data(), ssr.size() };
    RHIShaderDesc ssrResolveDesc{ RHIShaderStage::Fragment, ssrResolve.data(), ssrResolve.size() };
    RHIShaderDesc ssrCompositeDesc{ RHIShaderStage::Fragment, ssrComposite.data(), ssrComposite.size() };
    m_FullscreenVert    = device->CreateShader(vsDesc);
    m_SSRFrag           = device->CreateShader(ssrDesc);
    m_SSRResolveFrag    = device->CreateShader(ssrResolveDesc);
    m_SSRCompositeFrag  = device->CreateShader(ssrCompositeDesc);

    // Trace, at TraceDim resolution. No sceneColor binding — this pass produces
    // hit UVs, and the reflected image is only ever fetched at full res by the
    // resolve below.
    {
        RHIPipelineDesc desc;
        desc.vertexShader     = m_FullscreenVert.get();
        desc.fragmentShader   = m_SSRFrag.get();
        desc.resourceBindings = {
            { 0, RHIResourceType::CombinedImageSampler, RHIShaderStage::Fragment }, // gViewPos
            { 1, RHIResourceType::CombinedImageSampler, RHIShaderStage::Fragment }, // gViewNormal
            { 2, RHIResourceType::CombinedImageSampler, RHIShaderStage::Fragment }, // gMaterial
            { 3, RHIResourceType::UniformBuffer,        RHIShaderStage::Fragment }, // SSRUBO
            { 4, RHIResourceType::CombinedImageSampler, RHIShaderStage::Fragment }, // linearDepth
            { 5, RHIResourceType::CombinedImageSampler, RHIShaderStage::Fragment }, // coarseDepth
        };
        desc.colorFormat = kSSRFormat;
        m_SSRPipeline = device->CreatePipeline(desc);
    }
    // Resolve, full res. Needs the projection too: near-mirror pixels re-trace
    // their own ray here rather than reuse a low-res one.
    {
        RHIPipelineDesc desc;
        desc.vertexShader     = m_FullscreenVert.get();
        desc.fragmentShader   = m_SSRResolveFrag.get();
        desc.resourceBindings = {
            { 0, RHIResourceType::CombinedImageSampler, RHIShaderStage::Fragment }, // ssrHit
            { 1, RHIResourceType::CombinedImageSampler, RHIShaderStage::Fragment }, // gViewPos
            { 2, RHIResourceType::CombinedImageSampler, RHIShaderStage::Fragment }, // gViewNormal
            { 3, RHIResourceType::CombinedImageSampler, RHIShaderStage::Fragment }, // gMaterial
            { 4, RHIResourceType::CombinedImageSampler, RHIShaderStage::Fragment }, // sceneColor
            { 5, RHIResourceType::UniformBuffer,        RHIShaderStage::Fragment }, // SSRUBO
            { 6, RHIResourceType::CombinedImageSampler, RHIShaderStage::Fragment }, // linearDepth
            { 7, RHIResourceType::CombinedImageSampler, RHIShaderStage::Fragment }, // coarseDepth
        };
        desc.colorFormat = kSSRFormat;   // ssrColor — .a is load-bearing confidence
        m_SSRResolvePipeline = device->CreatePipeline(desc);
    }
    // Composite: resolves mix(scene, reflection, material weight) into a fresh
    // target — it can't blend into hdrLit in place, since reading and writing
    // the same texture across the two passes would cycle the graph (readers
    // depend on a texture's LAST writer). Bindings mirror ssr_composite.frag.
    {
        RHIPipelineDesc desc;
        desc.vertexShader     = m_FullscreenVert.get();
        desc.fragmentShader   = m_SSRCompositeFrag.get();
        desc.resourceBindings = {
            { 0, RHIResourceType::CombinedImageSampler, RHIShaderStage::Fragment }, // ssrColor
            { 1, RHIResourceType::CombinedImageSampler, RHIShaderStage::Fragment }, // gViewPos
            { 2, RHIResourceType::CombinedImageSampler, RHIShaderStage::Fragment }, // gViewNormal
            { 3, RHIResourceType::CombinedImageSampler, RHIShaderStage::Fragment }, // gMaterial
            { 4, RHIResourceType::CombinedImageSampler, RHIShaderStage::Fragment }, // sceneColor
        };
        desc.colorFormat = RHIFormat::RGBA16F;   // outColor, hdrLit's format
        m_SSRCompositePipeline = device->CreatePipeline(desc);
    }
}

VulkanSSRPass::~VulkanSSRPass() = default;

void VulkanSSRPass::AddToGraph(RHIRenderGraph& graph,
                               RGTextureHandle viewPos, RGTextureHandle viewNormal,
                               RGTextureHandle linearDepth, RGTextureHandle coarseDepth,
                               RGTextureHandle sceneColor, RGTextureHandle ssrHit,
                               RGTextureHandle ssrColor,
                               RGTextureHandle gMaterial, RGTextureHandle outColor,
                               RGTextureHandle compositeSource) {
    // Where the composite reads reflections from. Defaults to what the resolve
    // wrote; RT reflections substitute their own merged target.
    const RGTextureHandle reflection =
        compositeSource.IsValid() ? compositeSource : ssrColor;

    // Sets are built once — graph textures keep their identity across frames.
    if (!m_SSRSet)
        m_SSRSet = m_Device->CreateResourceSet(
            m_SSRPipeline.get(), 0, { { 3, m_UBO.get() } },
            { { 0, graph.GetTexture(viewPos) },
              { 1, graph.GetTexture(viewNormal) },
              { 2, graph.GetTexture(gMaterial) },
              { 4, graph.GetTexture(linearDepth) },
              { 5, graph.GetTexture(coarseDepth) } });

    if (!m_SSRResolveSet)
        m_SSRResolveSet = m_Device->CreateResourceSet(
            m_SSRResolvePipeline.get(), 0, { { 5, m_UBO.get() } },
            { { 0, graph.GetTexture(ssrHit) },
              { 1, graph.GetTexture(viewPos) },
              { 2, graph.GetTexture(viewNormal) },
              { 3, graph.GetTexture(gMaterial) },
              { 4, graph.GetTexture(sceneColor) },
              { 6, graph.GetTexture(linearDepth) },
              { 7, graph.GetTexture(coarseDepth) } });

    if (!m_SSRCompositeSet)
        m_SSRCompositeSet = m_Device->CreateResourceSet(
            m_SSRCompositePipeline.get(), 0, {},
            { { 0, graph.GetTexture(reflection) },
              { 1, graph.GetTexture(viewPos) },
              { 2, graph.GetTexture(viewNormal) },
              { 3, graph.GetTexture(gMaterial) },
              { 4, graph.GetTexture(sceneColor) } });

    // Cleared to zero so misses stay at confidence 0 (the resolve treats a zero
    // b channel as "that ray found nothing").
    graph.AddPass("SSRTrace")
        .Read(viewPos).Read(viewNormal).Read(gMaterial)
        .Read(linearDepth).Read(coarseDepth)
        .Write(ssrHit)
        .SetClearColor({ 0.0f, 0.0f, 0.0f, 0.0f })
        .SetExecute([this](RHICommandList* cmd) {
            cmd->BindPipeline(m_SSRPipeline.get());
            cmd->BindResourceSet(0, m_SSRSet.get());
            cmd->Draw(3);
        });
    // Full-res resolve into ssrColor. Every pixel is written (the shader's
    // early-outs all store a value), so no clear is needed.
    graph.AddPass("SSRResolve")
        .Read(ssrHit).Read(viewPos).Read(viewNormal).Read(gMaterial)
        .Read(sceneColor).Read(linearDepth).Read(coarseDepth)
        .Write(ssrColor)
        .SetExecute([this](RHICommandList* cmd) {
            cmd->BindPipeline(m_SSRResolvePipeline.get());
            cmd->BindResourceSet(0, m_SSRResolveSet.get());
            cmd->Draw(3);
        });
    // Fullscreen composite into outColor: every pixel outputs either its scene
    // color (weight 0) or the lerp toward the reflection.
    graph.AddPass("SSRComposite")
        .Read(reflection).Read(viewPos).Read(viewNormal).Read(gMaterial).Read(sceneColor)
        .Write(outColor)
        .SetExecute([this](RHICommandList* cmd) {
            cmd->BindPipeline(m_SSRCompositePipeline.get());
            cmd->BindResourceSet(0, m_SSRCompositeSet.get());
            cmd->Draw(3);
        });
}

void VulkanSSRPass::SetProjection(const glm::mat4& projection) {
    m_UBOData.projection = projection;
    m_UBO->Update(&m_UBOData, sizeof(SSRUBO));
}

} // namespace Diamond
