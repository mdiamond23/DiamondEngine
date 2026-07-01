#pragma once

#include "Renderer/Renderer2D.h"
#include "Renderer/RHI/RHIResources.h"

#include <glm/glm.hpp>
#include <cstdint>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

namespace Diamond {

class RHIDevice;
class RHICommandList;

// Vulkan 2D overlay batcher. Mirrors OpenGLRenderer2D: accumulates screen-space
// quads (and, later, text) into a CPU vertex buffer and records them into a
// command list.
//
// The one structural difference from the GL backend: it has no implicit global
// pipeline state to toggle. It records into an RHICommandList supplied per frame
// via SetCommandList(), so it must run inside a render scope that is already open
// (a render-graph pass, or the SceneRenderer swapchain overlay).
//
// Batching: unlike the GL backend it cannot re-upload its dynamic vertex buffer
// on every texture switch (one host-visible copy per frame — a second Update()
// would clobber the first before the deferred draws run). Instead it accumulates
// every vertex for the frame into one buffer and records a run of contiguous
// draws, one per texture, using firstVertex offsets. A descriptor set is built
// and cached per RHITexture the first time it's drawn.
//
// Solid quads use a 1x1 white texture (mode 0); textured quads sample a
// VulkanTexture2D (mode 0); text samples a font atlas as coverage (mode 1). All
// three share one pipeline and interleave within a frame.
class VulkanRenderer2D : public Renderer2D {
public:
    // 'colorFormat' is the format of the render target this records into (the
    // swapchain for an overlay). shaderDir is DIAMOND_VULKAN_SHADER_DIR.
    VulkanRenderer2D(RHIDevice* device, const std::string& shaderDir, RHIFormat colorFormat);
    ~VulkanRenderer2D() override;

    // Vulkan-only: the (already in-scope) command list the next Begin/End batch
    // records into. Set this before Begin().
    void SetCommandList(RHICommandList* cmd) { m_Cmd = cmd; }

    void Begin(const glm::mat4& projection) override;
    void End() override;

    void  DrawQuad(const glm::vec2& pos, const glm::vec2& size,
                   const glm::vec4& color) override;
    void  DrawTexturedQuad(const glm::vec2& pos, const glm::vec2& size,
                           const Texture& texture, const glm::vec4& tint,
                           const glm::vec2& uvMin, const glm::vec2& uvMax) override;
    float DrawText(const Font& font, const std::string& text,
                   const glm::vec2& pos, const glm::vec4& color, float scale) override;

private:
    struct Vertex {
        glm::vec2 pos;
        glm::vec2 uv;
        glm::vec4 color;
        float     mode;   // 0 = modulate RGBA, 1 = R channel as coverage (text)
    };

    void PushQuad(const glm::vec2& pos, const glm::vec2& size,
                  const glm::vec2& uvMin, const glm::vec2& uvMax,
                  const glm::vec4& color, float mode);

    // Switch the texture the following quads sample, closing the pending run into
    // a batch when it changes. A no-op if 'tex' is already current.
    void SetTexture(RHITexture* tex);
    // The (lazily built, cached) descriptor set that samples 'tex' at binding 0.
    RHIResourceSet* GetSet(RHITexture* tex);

    // A contiguous run of vertices that all sample one texture.
    struct Batch {
        RHITexture* texture = nullptr;
        uint32_t    first   = 0;   // firstVertex into m_Verts
        uint32_t    count   = 0;   // vertex count
    };

    static constexpr uint32_t kMaxVertices = 24000;   // 4000 quads per frame

    RHIDevice*      m_Device = nullptr;
    RHICommandList* m_Cmd    = nullptr;
    glm::mat4       m_Projection{ 1.0f };
    std::vector<Vertex> m_Verts;

    // Current run being accumulated (closed into m_Batches on texture change / End).
    std::vector<Batch> m_Batches;
    RHITexture*        m_CurrentTex = nullptr;
    uint32_t           m_BatchStart = 0;

    std::unique_ptr<RHIShader>   m_Vert;
    std::unique_ptr<RHIShader>   m_Frag;
    std::unique_ptr<RHIPipeline> m_Pipeline;
    std::unique_ptr<RHIBuffer>   m_VertexBuffer;   // dynamic per-frame ring
    std::unique_ptr<RHITexture>  m_WhiteTex;       // 1x1 white — solid quads

    // One descriptor set per texture ever drawn (keyed by the RHITexture). The
    // sampled textures must outlive this renderer.
    std::unordered_map<RHITexture*, std::unique_ptr<RHIResourceSet>> m_Sets;
};

} // namespace Diamond
