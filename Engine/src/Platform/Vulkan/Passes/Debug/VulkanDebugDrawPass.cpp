#include "Platform/Vulkan/Passes/Debug/VulkanDebugDrawPass.h"
#include "Platform/Vulkan/Passes/VulkanPassCommon.h"   // LoadSpirv
#include "Renderer/RHI/RHIDevice.h"
#include "Renderer/RHI/RHICommandList.h"

#include <glm/gtc/type_ptr.hpp>
#include <spdlog/spdlog.h>

#include <cstddef>

namespace Diamond {

VulkanDebugDrawPass::VulkanDebugDrawPass(RHIDevice* device, const std::string& shaderDir,
                                         RHIFormat colorFormat)
    : m_Device(device)
{
    const std::vector<uint32_t> vs = LoadSpirv(shaderDir, "debugdraw.vert.spv");
    const std::vector<uint32_t> fs = LoadSpirv(shaderDir, "debugdraw.frag.spv");
    RHIShaderDesc vsDesc{ RHIShaderStage::Vertex,   vs.data(), vs.size() };
    RHIShaderDesc fsDesc{ RHIShaderStage::Fragment, fs.data(), fs.size() };
    m_Vert = device->CreateShader(vsDesc);
    m_Frag = device->CreateShader(fsDesc);

    RHIPipelineDesc desc;
    desc.vertexShader   = m_Vert.get();
    desc.fragmentShader = m_Frag.get();
    desc.vertexLayout.stride     = sizeof(DebugDraw::Vertex);
    desc.vertexLayout.attributes = {
        { 0, RHIVertexFormat::Float3, static_cast<uint32_t>(offsetof(DebugDraw::Vertex, pos))   },
        { 1, RHIVertexFormat::Float3, static_cast<uint32_t>(offsetof(DebugDraw::Vertex, color)) },
    };
    desc.pushConstants = { RHIShaderStage::Vertex, sizeof(glm::mat4) };
    desc.topology     = RHIPrimitiveTopology::LineList;
    desc.cullMode     = RHICullMode::None;
    desc.colorFormat  = colorFormat;
    desc.depthFormat  = RHIFormat::Depth32F;
    desc.depthTest    = true;
    desc.depthWrite   = false;   // draw on top without corrupting depth for later passes
    desc.depthCompare = RHICompareOp::Less;
    m_Pipeline = device->CreatePipeline(desc);

    RHIBufferDesc vb;
    vb.size    = static_cast<uint64_t>(kMaxVertices) * sizeof(DebugDraw::Vertex);
    vb.usage   = RHIBufferUsage::Vertex;
    vb.dynamic = true;
    m_VertexBuffer = device->CreateBuffer(vb);
}

VulkanDebugDrawPass::~VulkanDebugDrawPass() = default;

void VulkanDebugDrawPass::AddToGraph(RHIRenderGraph& graph, RGTextureHandle color,
                                     bool toSwapchain, RGTextureHandle depth)
{
    RGPass& pass = graph.AddPass("DebugDraw");
    if (toSwapchain) pass.WriteSwapchain();
    else             pass.Write(color);
    pass.Write(depth)   // read-only depth test (depthWrite = false)
        .Load()         // LOAD both — keep the tonemapped scene + its depth
        .SetExecute([this](RHICommandList* cmd) {
            if (m_VertexCount == 0) return;
            cmd->BindPipeline(m_Pipeline.get());
            cmd->PushConstants(RHIShaderStage::Vertex, 0, sizeof(glm::mat4),
                               glm::value_ptr(m_ViewProj));
            cmd->BindVertexBuffer(m_VertexBuffer.get());
            cmd->Draw(m_VertexCount);
        });
}

void VulkanDebugDrawPass::SetFrameData(const glm::mat4& viewProj)
{
    m_ViewProj = viewProj;

    std::vector<DebugDraw::Vertex> verts = DebugDraw::TakeVertices();
    if (verts.size() > kMaxVertices) {
        spdlog::warn("[VulkanDebugDrawPass] {} verts exceeds cap {} — clamping",
                     verts.size(), kMaxVertices);
        verts.resize(kMaxVertices);
    }
    m_VertexCount = static_cast<uint32_t>(verts.size());
    if (m_VertexCount > 0)
        m_VertexBuffer->Update(verts.data(), verts.size() * sizeof(DebugDraw::Vertex));
}

} // namespace Diamond
