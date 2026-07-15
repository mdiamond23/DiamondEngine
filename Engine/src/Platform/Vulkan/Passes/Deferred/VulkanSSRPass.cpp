#include "Platform/Vulkan/Passes/Deferred/VulkanSSRPass.h"
#include "Platform/Vulkan/Passes/VulkanPassCommon.h"
#include "Renderer/RHI/RHIDevice.h"
#include "Renderer/RHI/RHICommandList.h"

#include <cstdint>
#include <vector>

namespace Diamond {

namespace {
constexpr RHIFormat kSSRFormat = RHIFormat::RGBA16F;   // rgb = reflection color, a = hit confidence
} // namespace

VulkanSSRPass::VulkanSSRPass(RHIDevice* device, const std::string& shaderDir, uint32_t width, uint32_t height)
    : m_Device(device) {

    m_UBOData.screenSize = glm::vec2(width, height);

    // Dynamic UBO — screenSize is filled now, projection per frame.
    RHIBufferDesc uboDesc;
    uboDesc.size    = sizeof(SSRUBO);
    uboDesc.usage   = RHIBufferUsage::Uniform;
    uboDesc.dynamic = true;
    m_UBO = device->CreateBuffer(uboDesc);

    // ── Pipeline ─────────────────────────────────────────────────────────────────
    const std::vector<uint32_t> vs              = LoadSpirv(shaderDir, "fullscreen.vert.spv");
    const std::vector<uint32_t> ssr             = LoadSpirv(shaderDir, "ssr.frag.spv");
    const std::vector<uint32_t> ssrComposite    = LoadSpirv(shaderDir, "ssr_composite.frag.spv");

    RHIShaderDesc vsDesc{ RHIShaderStage::Vertex,   vs.data(),  vs.size() };
    RHIShaderDesc ssrDesc{ RHIShaderStage::Fragment, ssr.data(), ssr.size() };
    RHIShaderDesc ssrCompositeDesc{ RHIShaderStage::Fragment, ssrComposite.data(), ssrComposite.size() };
    m_FullscreenVert = device->CreateShader(vsDesc);
    m_SSRFrag           = device->CreateShader(ssrDesc);
    m_SSRCompositeFrag  = device->CreateShader(ssrCompositeDesc);

    {
        RHIPipelineDesc desc;
        desc.vertexShader     = m_FullscreenVert.get();
        desc.fragmentShader   = m_SSRFrag.get();
        desc.resourceBindings = {
            { 0, RHIResourceType::CombinedImageSampler, RHIShaderStage::Fragment }, // gViewPos
            { 1, RHIResourceType::CombinedImageSampler, RHIShaderStage::Fragment }, // gViewNormal
            { 2, RHIResourceType::CombinedImageSampler, RHIShaderStage::Fragment }, // sceneColor
            { 3, RHIResourceType::UniformBuffer,        RHIShaderStage::Fragment }, // SSRUBO
        };
        desc.colorFormat = kSSRFormat;
        m_SSRPipeline = device->CreatePipeline(desc);
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
                               RGTextureHandle sceneColor, RGTextureHandle ssrColor,
                               RGTextureHandle gMaterial, RGTextureHandle outColor) {
    // Set is built once — graph textures keep their identity across frames.
    if (!m_SSRSet)
        m_SSRSet = m_Device->CreateResourceSet(
            m_SSRPipeline.get(), 0, { { 3, m_UBO.get() } },
            { { 0, graph.GetTexture(viewPos) },
              { 1, graph.GetTexture(viewNormal) },
              { 2, graph.GetTexture(sceneColor) } });

    if (!m_SSRCompositeSet)
        m_SSRCompositeSet = m_Device->CreateResourceSet(
            m_SSRCompositePipeline.get(), 0, {},
            { { 0, graph.GetTexture(ssrColor) },
              { 1, graph.GetTexture(viewPos) },
              { 2, graph.GetTexture(viewNormal) },
              { 3, graph.GetTexture(gMaterial) },
              { 4, graph.GetTexture(sceneColor) } });

    // Cleared to zero so misses stay at confidence 0 (composite falls back to IBL).
    graph.AddPass("SSR")
        .Read(viewPos).Read(viewNormal).Read(sceneColor)
        .Write(ssrColor)
        .SetClearColor({ 0.0f, 0.0f, 0.0f, 0.0f })
        .SetExecute([this](RHICommandList* cmd) {
            cmd->BindPipeline(m_SSRPipeline.get());
            cmd->BindResourceSet(0, m_SSRSet.get());
            cmd->Draw(3);
        });
    // Fullscreen resolve into outColor: every pixel outputs either its scene
    // color (weight 0) or the lerp toward the reflection.
    graph.AddPass("SSRComposite")
        .Read(ssrColor).Read(viewPos).Read(viewNormal).Read(gMaterial).Read(sceneColor)
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
