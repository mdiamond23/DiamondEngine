#include "Platform/Vulkan/Resources/VulkanRenderer2D.h"
#include "Platform/Vulkan/Resources/VulkanTexture2D.h"
#include "Platform/Vulkan/Passes/VulkanPassCommon.h"   // LoadSpirv
#include "Renderer/RHI/RHIDevice.h"
#include "Renderer/RHI/RHICommandList.h"
#include "Renderer/Font.h"

#include <glm/gtc/type_ptr.hpp>
#include <spdlog/spdlog.h>

#include <cstddef>

namespace Diamond {

VulkanRenderer2D::VulkanRenderer2D(RHIDevice* device, const std::string& shaderDir,
                                   RHIFormat colorFormat)
    : m_Device(device)
{
    const std::vector<uint32_t> vs = LoadSpirv(shaderDir, "renderer2d.vert.spv");
    const std::vector<uint32_t> fs = LoadSpirv(shaderDir, "renderer2d.frag.spv");
    RHIShaderDesc vsDesc{ RHIShaderStage::Vertex,   vs.data(), vs.size() };
    RHIShaderDesc fsDesc{ RHIShaderStage::Fragment, fs.data(), fs.size() };
    m_Vert = device->CreateShader(vsDesc);
    m_Frag = device->CreateShader(fsDesc);

    RHIPipelineDesc desc;
    desc.vertexShader   = m_Vert.get();
    desc.fragmentShader = m_Frag.get();
    desc.vertexLayout.stride     = sizeof(Vertex);
    desc.vertexLayout.attributes = {
        { 0, RHIVertexFormat::Float2, static_cast<uint32_t>(offsetof(Vertex, pos))   },
        { 1, RHIVertexFormat::Float2, static_cast<uint32_t>(offsetof(Vertex, uv))    },
        { 2, RHIVertexFormat::Float4, static_cast<uint32_t>(offsetof(Vertex, color)) },
        { 3, RHIVertexFormat::Float,  static_cast<uint32_t>(offsetof(Vertex, mode))  },
    };
    desc.resourceBindings = {
        { 0, RHIResourceType::CombinedImageSampler, RHIShaderStage::Fragment },
    };
    desc.pushConstants = { RHIShaderStage::Vertex, sizeof(glm::mat4) };
    desc.blendMode     = RHIBlendMode::Alpha;   // straight-alpha src-over overlay
    desc.colorFormat   = colorFormat;
    m_Pipeline = device->CreatePipeline(desc);

    // Dynamic per-frame vertex ring (rewritten each End()).
    RHIBufferDesc vb;
    vb.size    = static_cast<uint64_t>(kMaxVertices) * sizeof(Vertex);
    vb.usage   = RHIBufferUsage::Vertex;
    vb.dynamic = true;
    m_VertexBuffer = device->CreateBuffer(vb);

    // 1x1 white texture so solid-color quads share the textured shader path.
    const uint8_t white[4] = { 255, 255, 255, 255 };
    RHITextureDesc tex;
    tex.width       = 1;
    tex.height      = 1;
    tex.format      = RHIFormat::RGBA8;
    tex.usage       = RHITextureUsage::Sampled;
    tex.initialData = white;
    tex.filter      = RHIFilter::Nearest;
    m_WhiteTex = device->CreateTexture(tex);
}

VulkanRenderer2D::~VulkanRenderer2D() = default;

void VulkanRenderer2D::Begin(const glm::mat4& projection)
{
    m_Projection = projection;
    m_Verts.clear();
    m_Batches.clear();
    m_CurrentTex = nullptr;
    m_BatchStart = 0;
}

void VulkanRenderer2D::SetTexture(RHITexture* tex)
{
    if (tex == m_CurrentTex) return;
    // Close the run accumulated under the previous texture.
    const uint32_t end = static_cast<uint32_t>(m_Verts.size());
    if (m_CurrentTex && end > m_BatchStart)
        m_Batches.push_back({ m_CurrentTex, m_BatchStart, end - m_BatchStart });
    m_CurrentTex = tex;
    m_BatchStart = end;
}

RHIResourceSet* VulkanRenderer2D::GetSet(RHITexture* tex)
{
    auto it = m_Sets.find(tex);
    if (it != m_Sets.end()) return it->second.get();
    auto set = m_Device->CreateResourceSet(m_Pipeline.get(), 0, {}, { { 0, tex } });
    RHIResourceSet* raw = set.get();
    m_Sets.emplace(tex, std::move(set));
    return raw;
}

void VulkanRenderer2D::PushQuad(const glm::vec2& pos, const glm::vec2& size,
                                const glm::vec2& uvMin, const glm::vec2& uvMax,
                                const glm::vec4& color, float mode)
{
    const glm::vec2 tl = pos;
    const glm::vec2 tr = { pos.x + size.x, pos.y };
    const glm::vec2 br = { pos.x + size.x, pos.y + size.y };
    const glm::vec2 bl = { pos.x,          pos.y + size.y };

    const glm::vec2 uvTL = uvMin;
    const glm::vec2 uvTR = { uvMax.x, uvMin.y };
    const glm::vec2 uvBR = uvMax;
    const glm::vec2 uvBL = { uvMin.x, uvMax.y };

    // Two triangles: (TL, BR, TR) and (TL, BL, BR).
    m_Verts.push_back({ tl, uvTL, color, mode });
    m_Verts.push_back({ br, uvBR, color, mode });
    m_Verts.push_back({ tr, uvTR, color, mode });
    m_Verts.push_back({ tl, uvTL, color, mode });
    m_Verts.push_back({ bl, uvBL, color, mode });
    m_Verts.push_back({ br, uvBR, color, mode });
}

void VulkanRenderer2D::DrawQuad(const glm::vec2& pos, const glm::vec2& size,
                                const glm::vec4& color)
{
    SetTexture(m_WhiteTex.get());
    PushQuad(pos, size, glm::vec2(0.0f), glm::vec2(1.0f), color, 0.0f);
}

void VulkanRenderer2D::DrawTexturedQuad(const glm::vec2& pos, const glm::vec2& size,
                                        const Texture& texture, const glm::vec4& tint,
                                        const glm::vec2& uvMin, const glm::vec2& uvMax)
{
    const auto* vk = dynamic_cast<const VulkanTexture2D*>(&texture);
    if (!vk || !vk->Rhi()) return;
    SetTexture(vk->Rhi());
    PushQuad(pos, size, uvMin, uvMax, tint, 0.0f);
}

float VulkanRenderer2D::DrawText(const Font& font, const std::string& text,
                                 const glm::vec2& pos, const glm::vec4& color, float scale)
{
    const auto& atlas = font.Atlas();
    if (!atlas) return pos.x;
    const auto* vk = dynamic_cast<const VulkanTexture2D*>(atlas.get());
    if (!vk || !vk->Rhi()) return pos.x;
    SetTexture(vk->Rhi());

    float penX      = pos.x;
    float baselineY = pos.y + font.Ascent() * scale;

    for (char ch : text) {
        if (ch == '\n') {
            penX = pos.x;
            baselineY += font.LineHeight() * scale;
            continue;
        }
        const Glyph* g = font.GetGlyph(static_cast<uint32_t>(static_cast<unsigned char>(ch)));
        if (!g) continue;
        if (g->size.x > 0.0f && g->size.y > 0.0f) {
            const glm::vec2 qpos  = { penX + g->offset.x * scale, baselineY + g->offset.y * scale };
            const glm::vec2 qsize = g->size * scale;
            PushQuad(qpos, qsize, g->uvMin, g->uvMax, color, 1.0f);
        }
        penX += g->advance * scale;
    }
    return penX - pos.x;
}

void VulkanRenderer2D::End()
{
    // Close the final run.
    const uint32_t end = static_cast<uint32_t>(m_Verts.size());
    if (m_CurrentTex && end > m_BatchStart)
        m_Batches.push_back({ m_CurrentTex, m_BatchStart, end - m_BatchStart });
    m_BatchStart = end;

    if (!m_Cmd || m_Verts.empty() || m_Batches.empty()) {
        m_Verts.clear();
        m_Batches.clear();
        m_CurrentTex = nullptr;
        m_BatchStart = 0;
        return;
    }
    if (m_Verts.size() > kMaxVertices) {
        spdlog::warn("[VulkanRenderer2D] batch of {} verts exceeds cap {} — clamping",
                     m_Verts.size(), kMaxVertices);
        m_Verts.resize(kMaxVertices);
    }

    m_VertexBuffer->Update(m_Verts.data(), m_Verts.size() * sizeof(Vertex));

    m_Cmd->BindPipeline(m_Pipeline.get());
    m_Cmd->PushConstants(RHIShaderStage::Vertex, 0, sizeof(glm::mat4),
                         glm::value_ptr(m_Projection));
    m_Cmd->BindVertexBuffer(m_VertexBuffer.get());

    // One draw per texture run. firstVertex offsets into the single upload, so no
    // dynamic-buffer copy is ever overwritten mid-frame.
    const uint32_t total = static_cast<uint32_t>(m_Verts.size());
    for (const auto& b : m_Batches) {
        if (b.first >= total) continue;                 // dropped by the clamp above
        uint32_t count = b.count;
        if (b.first + count > total) count = total - b.first;
        if (count == 0) continue;
        m_Cmd->BindResourceSet(0, GetSet(b.texture));
        m_Cmd->Draw(count, 1, b.first);
    }

    m_Verts.clear();
    m_Batches.clear();
    m_CurrentTex = nullptr;
    m_BatchStart = 0;
}

} // namespace Diamond
