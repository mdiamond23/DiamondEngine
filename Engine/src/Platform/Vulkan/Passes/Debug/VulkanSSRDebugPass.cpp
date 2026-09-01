#include "Platform/Vulkan/Passes/Debug/VulkanSSRDebugPass.h"
#include "Platform/Vulkan/Passes/VulkanPassCommon.h"
#include "Renderer/RHI/RHIDevice.h"
#include "Renderer/RHI/RHICommandList.h"

#include <cstdint>
#include <vector>

namespace Diamond {

VulkanSSRDebugPass::VulkanSSRDebugPass(RHIDevice* device, const std::string& shaderDir,
                                       uint32_t width, uint32_t height,
                                       RHIFormat targetFormat)
    : m_Device(device) {

    m_UBOData.fullSize = glm::vec2(width, height);

    RHIBufferDesc uboDesc;
    uboDesc.size    = sizeof(DebugUBO);
    uboDesc.usage   = RHIBufferUsage::Uniform;
    uboDesc.dynamic = true;
    m_UBO = device->CreateBuffer(uboDesc);

    const std::vector<uint32_t> vs = LoadSpirv(shaderDir, "fullscreen.vert.spv");
    const std::vector<uint32_t> fs = LoadSpirv(shaderDir, "ssr_debug.frag.spv");

    RHIShaderDesc vsDesc{ RHIShaderStage::Vertex,   vs.data(), vs.size() };
    RHIShaderDesc fsDesc{ RHIShaderStage::Fragment, fs.data(), fs.size() };
    m_FullscreenVert = device->CreateShader(vsDesc);
    m_Frag           = device->CreateShader(fsDesc);

    // Bindings mirror ssr_debug.frag, which in turn mirrors the resolve's own
    // inputs minus sceneColor — this pass never looks at the reflected image,
    // only at what the march did to find it.
    RHIPipelineDesc desc;
    desc.vertexShader     = m_FullscreenVert.get();
    desc.fragmentShader   = m_Frag.get();
    desc.resourceBindings = {
        { 0, RHIResourceType::CombinedImageSampler, RHIShaderStage::Fragment }, // gViewPos
        { 1, RHIResourceType::CombinedImageSampler, RHIShaderStage::Fragment }, // gViewNormal
        { 2, RHIResourceType::CombinedImageSampler, RHIShaderStage::Fragment }, // gMaterial
        { 3, RHIResourceType::UniformBuffer,        RHIShaderStage::Fragment }, // SSRDebugUBO
        { 4, RHIResourceType::CombinedImageSampler, RHIShaderStage::Fragment }, // linearDepth
        { 5, RHIResourceType::CombinedImageSampler, RHIShaderStage::Fragment }, // coarseDepth
    };
    desc.colorFormat = targetFormat;
    m_Pipeline = device->CreatePipeline(desc);
}

VulkanSSRDebugPass::~VulkanSSRDebugPass() = default;

void VulkanSSRDebugPass::AddToGraph(RHIRenderGraph& graph,
                                    RGTextureHandle viewPos, RGTextureHandle viewNormal,
                                    RGTextureHandle gMaterial, RGTextureHandle linearDepth,
                                    RGTextureHandle coarseDepth,
                                    RGTextureHandle target, bool toSwapchain) {
    if (!Available()) return;

    if (!m_Set)
        m_Set = m_Device->CreateResourceSet(
            m_Pipeline.get(), 0, { { 3, m_UBO.get() } },
            { { 0, graph.GetTexture(viewPos) },
              { 1, graph.GetTexture(viewNormal) },
              { 2, graph.GetTexture(gMaterial) },
              { 4, graph.GetTexture(linearDepth) },
              { 5, graph.GetTexture(coarseDepth) } });

    // Registered every frame, drawing only when a mode is selected: Load() keeps
    // the tonemapped frame, and the skipped draw leaves it exactly as it was.
    RGPass& pass = graph.AddPass("SSRDebug");
    pass.Read(viewPos).Read(viewNormal).Read(gMaterial)
        .Read(linearDepth).Read(coarseDepth);
    if (toSwapchain) pass.WriteSwapchain();
    else             pass.Write(target);
    pass.Load().SetExecute([this](RHICommandList* cmd) {
        if (!IsEnabled()) return;
        cmd->BindPipeline(m_Pipeline.get());
        cmd->BindResourceSet(0, m_Set.get());
        cmd->Draw(3);
    });
}

void VulkanSSRDebugPass::SetProjection(const glm::mat4& projection) {
    m_UBOData.projection = projection;
    m_UBOData.mode       = static_cast<int>(m_Mode);
    m_UBO->Update(&m_UBOData, sizeof(DebugUBO));
}

} // namespace Diamond
