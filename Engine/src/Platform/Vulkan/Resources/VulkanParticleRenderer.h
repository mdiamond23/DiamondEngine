#pragma once

#include "Renderer/ParticleRenderer.h"
#include "Renderer/RHI/RHIResources.h"
#include "Renderer/RHI/RHIRenderGraph.h"

#include <glm/glm.hpp>
#include <cstdint>
#include <functional>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

namespace Diamond {

class RHIDevice;
class RHICommandList;

// Vulkan particle renderer. Expands particles into world-space billboard
// triangles, uploads one dynamic vertex buffer per frame, and records
// texture/blend runs into an already-open render scope.
class VulkanParticleRenderer : public ParticleRenderer {
public:
    // 'colorFormat' is the format of the HDR color target this records into.
    // shaderDir is DIAMOND_VULKAN_SHADER_DIR.
    VulkanParticleRenderer(RHIDevice* device, const std::string& shaderDir, RHIFormat colorFormat);
    ~VulkanParticleRenderer() override;

    // Vulkan-only: the command list the next Begin/End batch records into.
    void SetCommandList(RHICommandList* cmd) { m_Cmd = cmd; }

    // Register the particle pass. Composites into 'color' (HDR) with a read-only
    // depth test against 'depth' — both LOADed over the lit scene (the pipeline's
    // depthWrite is false). The graph orders this after whatever produced 'color'
    // and inserts the write-after-write barrier. 'emit' issues the frame's
    // Begin(view,proj) / Draw(...) / End() against this renderer inside the pass
    // scope; the command list is already bound when it runs.
    void AddToGraph(RHIRenderGraph& graph, RGTextureHandle color, RGTextureHandle depth,
                    std::function<void(VulkanParticleRenderer&)> emit);

    void Begin(const glm::mat4& view, const glm::mat4& proj) override;
    void Draw(const std::vector<RenderParticle>& particles,
              const Texture& texture, ParticleBlend blend) override;
    void End() override;

    // Hot-reload (editor only): recompiles particle.vert/frag from shaderDir's
    // current .spv and rebuilds both pipelines IN PLACE (same 'this' — the
    // graph pass captured in AddToGraph reads m_AlphaPipeline/m_AdditivePipeline
    // at Execute time, no graph rebuild). Drops the per-texture set cache (built
    // against the old pipeline layout); GetSet rebuilds lazily on the next Draw.
    // Caller must WaitIdle() first. A no-op (logs) if the new SPIR-V is
    // missing/corrupt.
    void Reload();

private:
    struct Vertex {
        glm::vec3 pos;
        glm::vec2 uv;
        glm::vec4 color;
    };

    struct Batch {
        RHITexture*  texture   = nullptr;
        RHIBlendMode blendMode = RHIBlendMode::Alpha;
        uint32_t     first     = 0;
        uint32_t     count     = 0;
    };

    void SetTexture(RHITexture* tex, RHIBlendMode blendMode);
    RHIResourceSet* GetSet(RHITexture* tex);

    static constexpr uint32_t kMaxVertices = 24000;   // 4000 quads per frame

    RHIDevice*      m_Device = nullptr;
    RHICommandList* m_Cmd    = nullptr;
    std::string     m_ShaderDir;
    RHIFormat       m_ColorFormat;

    glm::mat4 m_ViewProj {1.0f};
    glm::vec3 m_CamRight {1.0f, 0.0f, 0.0f};
    glm::vec3 m_CamUp    {0.0f, 1.0f, 0.0f};
    std::vector<Vertex> m_Verts;

    std::vector<Batch> m_Batches;
    RHITexture*        m_CurrentTex   = nullptr;
    RHIBlendMode       m_CurrentBlend = RHIBlendMode::Alpha;
    uint32_t           m_BatchStart   = 0;

    std::unique_ptr<RHIShader>   m_Vert;
    std::unique_ptr<RHIShader>   m_Frag;
    std::unique_ptr<RHIPipeline> m_AlphaPipeline;
    std::unique_ptr<RHIPipeline> m_AdditivePipeline;
    std::unique_ptr<RHIBuffer>   m_VertexBuffer;

    // One descriptor set per texture ever drawn. The sampled textures must
    // outlive this renderer.
    std::unordered_map<RHITexture*, std::unique_ptr<RHIResourceSet>> m_Sets;
};

} // namespace Diamond
