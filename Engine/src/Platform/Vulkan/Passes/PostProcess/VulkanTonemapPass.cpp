#include "Platform/Vulkan/Passes/PostProcess/VulkanTonemapPass.h"
#include "Platform/Vulkan/Passes/VulkanPassCommon.h"
#include "Renderer/RHI/RHIDevice.h"
#include "Renderer/RHI/RHICommandList.h"

namespace Diamond {

VulkanTonemapPass::VulkanTonemapPass(RHIDevice* device, const std::string& shaderDir,
                                     RHIFormat outputFormat)
    : m_Device(device)
{
    const std::vector<uint32_t> vs = LoadSpirv(shaderDir, "fullscreen.vert.spv");
    const std::vector<uint32_t> fs = LoadSpirv(shaderDir, "tonemap.frag.spv");
    RHIShaderDesc vsDesc{ RHIShaderStage::Vertex,   vs.data(), vs.size() };
    RHIShaderDesc fsDesc{ RHIShaderStage::Fragment, fs.data(), fs.size() };
    m_Vert = device->CreateShader(vsDesc);
    m_Frag = device->CreateShader(fsDesc);

    RHIPipelineDesc desc;
    desc.vertexShader   = m_Vert.get();
    desc.fragmentShader = m_Frag.get();
    // No vertex layout — the fullscreen triangle is generated from gl_VertexIndex.
    desc.resourceBindings = {
        { 0, RHIResourceType::CombinedImageSampler, RHIShaderStage::Fragment },
    };
    desc.pushConstants = { RHIShaderStage::Fragment, sizeof(float) };   // exposure
    desc.colorFormat = outputFormat;   // backbuffer, or an LDR texture for FXAA to read
    m_Pipeline = device->CreatePipeline(desc);
}

VulkanTonemapPass::~VulkanTonemapPass() = default;

void VulkanTonemapPass::AddToGraph(RHIRenderGraph& graph, RGTextureHandle hdrInput,
                                   RGTextureHandle output)
{
    // The HDR input is graph-owned but pooled (stable for the graph's lifetime), so
    // the sampler set is built once. A resize that recreates the pool texture would
    // require rebuilding this; the demo's offscreen targets are fixed-size.
    if (!m_Set)
        m_Set = m_Device->CreateResourceSet(
            m_Pipeline.get(), 0, {}, { { 0, graph.GetTexture(hdrInput) } });

    RGPass& pass = graph.AddPass("Tonemap").Read(hdrInput);
    if (output.IsValid()) pass.Write(output);   // LDR texture for a later FXAA pass
    else                  pass.WriteSwapchain();
    pass.SetExecute([this](RHICommandList* cmd) {
        cmd->BindPipeline(m_Pipeline.get());
        cmd->BindResourceSet(0, m_Set.get());
        cmd->PushConstants(RHIShaderStage::Fragment, 0, sizeof(float), &m_Exposure);
        cmd->Draw(3);
    });
}

} // namespace Diamond
