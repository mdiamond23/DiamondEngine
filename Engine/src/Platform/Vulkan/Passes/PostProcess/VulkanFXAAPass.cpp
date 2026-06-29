#include "Platform/Vulkan/Passes/PostProcess/VulkanFXAAPass.h"
#include "Platform/Vulkan/Passes/VulkanPassCommon.h"
#include "Renderer/RHI/RHIDevice.h"
#include "Renderer/RHI/RHICommandList.h"

namespace Diamond {

VulkanFXAAPass::VulkanFXAAPass(RHIDevice* device, const std::string& shaderDir,
                               RHIFormat outputFormat)
    : m_Device(device)
{
    const std::vector<uint32_t> vs = LoadSpirv(shaderDir, "fullscreen.vert.spv");
    const std::vector<uint32_t> fs = LoadSpirv(shaderDir, "fxaa.frag.spv");
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
    desc.colorFormat = outputFormat;
    m_Pipeline = device->CreatePipeline(desc);
}

VulkanFXAAPass::~VulkanFXAAPass() = default;

void VulkanFXAAPass::AddToGraph(RHIRenderGraph& graph, RGTextureHandle ldrInput,
                                RGTextureHandle output)
{
    if (!m_Set)
        m_Set = m_Device->CreateResourceSet(
            m_Pipeline.get(), 0, {}, { { 0, graph.GetTexture(ldrInput) } });

    RGPass& pass = graph.AddPass("FXAA").Read(ldrInput);
    if (output.IsValid()) pass.Write(output);
    else                  pass.WriteSwapchain();
    pass.SetExecute([this](RHICommandList* cmd) {
        cmd->BindPipeline(m_Pipeline.get());
        cmd->BindResourceSet(0, m_Set.get());
        cmd->Draw(3);
    });
}

} // namespace Diamond
