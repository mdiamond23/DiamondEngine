#include "Platform/Vulkan/Resources/VulkanParticleRenderer.h"
#include "Platform/Vulkan/Resources/VulkanTexture2D.h"
#include "Platform/Vulkan/Passes/VulkanPassCommon.h"   // LoadSpirv
#include "Renderer/RHI/RHICommandList.h"
#include "Renderer/RHI/RHIDevice.h"

#include <glm/gtc/type_ptr.hpp>
#include <spdlog/spdlog.h>

#include <cmath>
#include <cstddef>

namespace Diamond {

namespace {

RHIBlendMode ToRHIBlendMode(ParticleBlend blend)
{
    return blend == ParticleBlend::Additive ? RHIBlendMode::Additive : RHIBlendMode::Alpha;
}

} // namespace

VulkanParticleRenderer::VulkanParticleRenderer(RHIDevice* device, const std::string& shaderDir,
                                               RHIFormat colorFormat)
    : m_Device(device)
{
    const std::vector<uint32_t> vs = LoadSpirv(shaderDir, "particle.vert.spv");
    const std::vector<uint32_t> fs = LoadSpirv(shaderDir, "particle.frag.spv");
    RHIShaderDesc vsDesc{ RHIShaderStage::Vertex,   vs.data(), vs.size() };
    RHIShaderDesc fsDesc{ RHIShaderStage::Fragment, fs.data(), fs.size() };
    m_Vert = device->CreateShader(vsDesc);
    m_Frag = device->CreateShader(fsDesc);

    RHIPipelineDesc desc;
    desc.vertexShader   = m_Vert.get();
    desc.fragmentShader = m_Frag.get();
    desc.vertexLayout.stride     = sizeof(Vertex);
    desc.vertexLayout.attributes = {
        { 0, RHIVertexFormat::Float3, static_cast<uint32_t>(offsetof(Vertex, pos))   },
        { 1, RHIVertexFormat::Float2, static_cast<uint32_t>(offsetof(Vertex, uv))    },
        { 2, RHIVertexFormat::Float4, static_cast<uint32_t>(offsetof(Vertex, color)) },
    };
    desc.resourceBindings = {
        { 0, RHIResourceType::CombinedImageSampler, RHIShaderStage::Fragment },
    };
    desc.pushConstants = { RHIShaderStage::Vertex, sizeof(glm::mat4) };
    desc.colorFormat   = colorFormat;
    desc.depthFormat   = RHIFormat::Depth32F;
    desc.depthTest     = true;
    desc.depthWrite    = false;
    desc.depthCompare  = RHICompareOp::Less;
    desc.cullMode      = RHICullMode::None;

    desc.blendMode = RHIBlendMode::Alpha;
    m_AlphaPipeline = device->CreatePipeline(desc);

    desc.blendMode = RHIBlendMode::Additive;
    m_AdditivePipeline = device->CreatePipeline(desc);

    RHIBufferDesc vb;
    vb.size    = static_cast<uint64_t>(kMaxVertices) * sizeof(Vertex);
    vb.usage   = RHIBufferUsage::Vertex;
    vb.dynamic = true;
    m_VertexBuffer = device->CreateBuffer(vb);
}

VulkanParticleRenderer::~VulkanParticleRenderer() = default;

void VulkanParticleRenderer::AddToGraph(RHIRenderGraph& graph, RGTextureHandle color,
                                        RGTextureHandle depth,
                                        std::function<void(VulkanParticleRenderer&)> emit)
{
    graph.AddPass("Particles")
        .Write(color)   // composite over the lit HDR scene
        .Write(depth)   // read-only depth test (depthWrite = false)
        .Load()         // LOAD both — keep the scene + its depth
        .SetExecute([this, emit = std::move(emit)](RHICommandList* cmd) {
            SetCommandList(cmd);
            emit(*this);
        });
}

void VulkanParticleRenderer::Begin(const glm::mat4& view, const glm::mat4& proj)
{
    m_ViewProj = proj * view;

    // World-space camera basis = rows of the view rotation. In glm's column-major
    // storage that is view[col][row], so row 0 (right) and row 1 (up) read across.
    m_CamRight = glm::vec3(view[0][0], view[1][0], view[2][0]);
    m_CamUp    = glm::vec3(view[0][1], view[1][1], view[2][1]);

    m_Verts.clear();
    m_Batches.clear();
    m_CurrentTex   = nullptr;
    m_CurrentBlend = RHIBlendMode::Alpha;
    m_BatchStart   = 0;
}

void VulkanParticleRenderer::SetTexture(RHITexture* tex, RHIBlendMode blendMode)
{
    if (tex == m_CurrentTex && blendMode == m_CurrentBlend) return;

    const uint32_t end = static_cast<uint32_t>(m_Verts.size());
    if (m_CurrentTex && end > m_BatchStart)
        m_Batches.push_back({ m_CurrentTex, m_CurrentBlend, m_BatchStart, end - m_BatchStart });

    m_CurrentTex   = tex;
    m_CurrentBlend = blendMode;
    m_BatchStart   = end;
}

RHIResourceSet* VulkanParticleRenderer::GetSet(RHITexture* tex)
{
    auto it = m_Sets.find(tex);
    if (it != m_Sets.end()) return it->second.get();

    auto set = m_Device->CreateResourceSet(m_AlphaPipeline.get(), 0, {}, { { 0, tex } });
    RHIResourceSet* raw = set.get();
    m_Sets.emplace(tex, std::move(set));
    return raw;
}

void VulkanParticleRenderer::Draw(const std::vector<RenderParticle>& particles,
                                  const Texture& texture, ParticleBlend blend)
{
    if (particles.empty()) return;

    const auto* vkTexture = dynamic_cast<const VulkanTexture2D*>(&texture);
    if (!vkTexture || !vkTexture->Rhi()) {
        spdlog::warn("[VulkanParticleRenderer] Draw() called with a non-Vulkan texture");
        return;
    }

    SetTexture(vkTexture->Rhi(), ToRHIBlendMode(blend));
    m_Verts.reserve(m_Verts.size() + particles.size() * 6);

    for (const RenderParticle& p : particles) {
        const float c = std::cos(p.rotation);
        const float s = std::sin(p.rotation);
        // Spin the camera basis in the view plane for screen-facing rotation.
        const glm::vec3 r = (m_CamRight * c + m_CamUp * s) * (p.size * 0.5f);
        const glm::vec3 u = (m_CamUp * c - m_CamRight * s) * (p.size * 0.5f);

        const glm::vec3 bl = p.position - r - u;
        const glm::vec3 br = p.position + r - u;
        const glm::vec3 tr = p.position + r + u;
        const glm::vec3 tl = p.position - r + u;

        // UVs: top-left origin (matches the engine's texture convention).
        m_Verts.push_back({ bl, {0.0f, 1.0f}, p.color });
        m_Verts.push_back({ br, {1.0f, 1.0f}, p.color });
        m_Verts.push_back({ tr, {1.0f, 0.0f}, p.color });
        m_Verts.push_back({ bl, {0.0f, 1.0f}, p.color });
        m_Verts.push_back({ tr, {1.0f, 0.0f}, p.color });
        m_Verts.push_back({ tl, {0.0f, 0.0f}, p.color });
    }
}

void VulkanParticleRenderer::End()
{
    const uint32_t end = static_cast<uint32_t>(m_Verts.size());
    if (m_CurrentTex && end > m_BatchStart)
        m_Batches.push_back({ m_CurrentTex, m_CurrentBlend, m_BatchStart, end - m_BatchStart });
    m_BatchStart = end;

    if (!m_Cmd || m_Verts.empty() || m_Batches.empty()) {
        m_Verts.clear();
        m_Batches.clear();
        m_CurrentTex = nullptr;
        m_BatchStart = 0;
        return;
    }

    if (m_Verts.size() > kMaxVertices) {
        spdlog::warn("[VulkanParticleRenderer] batch of {} verts exceeds cap {} - clamping",
                     m_Verts.size(), kMaxVertices);
        m_Verts.resize(kMaxVertices);
    }

    m_VertexBuffer->Update(m_Verts.data(), m_Verts.size() * sizeof(Vertex));

    const uint32_t total = static_cast<uint32_t>(m_Verts.size());
    for (const Batch& b : m_Batches) {
        if (b.first >= total) continue;
        uint32_t count = b.count;
        if (b.first + count > total) count = total - b.first;
        if (count == 0) continue;

        RHIPipeline* pipeline = (b.blendMode == RHIBlendMode::Additive)
            ? m_AdditivePipeline.get()
            : m_AlphaPipeline.get();
        m_Cmd->BindPipeline(pipeline);
        m_Cmd->PushConstants(RHIShaderStage::Vertex, 0, sizeof(glm::mat4),
                             glm::value_ptr(m_ViewProj));
        m_Cmd->BindResourceSet(0, GetSet(b.texture));
        m_Cmd->BindVertexBuffer(m_VertexBuffer.get());
        m_Cmd->Draw(count, 1, b.first);
    }

    m_Verts.clear();
    m_Batches.clear();
    m_CurrentTex = nullptr;
    m_BatchStart = 0;
}

} // namespace Diamond
