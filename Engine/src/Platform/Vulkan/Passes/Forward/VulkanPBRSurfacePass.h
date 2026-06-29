#pragma once

#include "Renderer/RHI/RHIRenderGraph.h"

#include <glm/glm.hpp>

#include <functional>
#include <memory>
#include <string>

namespace Diamond {

class RHIDevice;
class RHIBuffer;
class RHIShader;
class RHIPipeline;
class RHIResourceSet;
class RHICommandList;
class VulkanIBLPass;

// Forward PBR surface — Vulkan port of OpenGLPBRSurfacePass. The forward shading
// path (an alternative to the deferred chain): it draws the skybox background and
// opaque geometry lit with full Cook-Torrance + IBL in world space.
//
// It composes over the deferred result rather than replacing it: a pass can't blend
// over the attachment it also samples, so the pass first fullscreen-copies the
// previous HDR ('input') into its own 'output' target, then renders the skybox (far
// plane, LessEqual depth) and the forward-lit objects over it, depth-testing against
// the shared scene depth. This keeps the render graph's single-writer invariant
// (each pass writes a fresh HDR target) while still layering onto the lit scene.
//
// Follows the ported-pass template: the pass owns its three pipelines (copy, skybox,
// forward) + shaders, the per-frame UBOs, and the descriptor sets; the graph owns the
// transient textures. The IBL cubemaps/LUT (not RHITextures) are bound raw via BindIBL.
class VulkanPBRSurfacePass {
public:
    VulkanPBRSurfacePass(RHIDevice* device, const std::string& shaderDir);
    ~VulkanPBRSurfacePass();

    // Register the pass. 'input' is the previous HDR result (copied in first), 'output'
    // the fresh HDR target this pass writes, 'depth' the shared scene depth (loaded,
    // tested against). 'drawSkybox' records the skybox cube; 'drawForward' records the
    // forward-lit geometry (bind buffers, push model, draw) — both after the pass binds
    // the matching pipeline + set.
    void AddToGraph(RHIRenderGraph& graph,
                    RGTextureHandle input, RGTextureHandle output, RGTextureHandle depth,
                    std::function<void(RHICommandList*)> drawSkybox,
                    std::function<void(RHICommandList*)> drawForward);

    // Bind the baked IBL maps (skybox env cube + irradiance/prefilter/BRDF) into the
    // skybox + forward descriptor sets. Call once after AddToGraph; maps are static.
    void BindIBL(const VulkanIBLPass& ibl);

    // Upload this frame's camera + lights. Call after BeginFrame, before graph.Execute.
    void SetFrameData(const glm::mat4& view, const glm::mat4& projection,
                      const glm::vec3& camPos,
                      const glm::vec3 lightPositions[4], const glm::vec3 lightColors[4]);

private:
    // std140 layouts matching the shaders.
    struct CameraUBO {        // skybox.vert
        glm::mat4 view;
        glm::mat4 projection;
    };
    struct ForwardUBO {       // pbr_surface.vert/frag
        glm::mat4 view;
        glm::mat4 projection;
        glm::vec4 camPos;            // .xyz
        glm::vec4 albedo;            // .rgb albedo, .w metallic
        glm::vec4 material;          // .x roughness, .y ao
        glm::vec4 lightPositions[4]; // .xyz
        glm::vec4 lightColors[4];    // .rgb
    };

    RHIDevice* m_Device;

    // Constant material for the forward demo object (slightly metallic so the IBL
    // reflection is visible). Mirrors the GL pass's PBRMaterial constants.
    glm::vec3 m_Albedo    { 0.95f, 0.93f, 0.88f };
    float     m_Metallic  = 0.85f;
    float     m_Roughness = 0.2f;
    float     m_AO        = 1.0f;

    CameraUBO  m_CamData{};
    ForwardUBO m_FwdData{};

    // Copy: fullscreen.vert + copy.frag.
    std::unique_ptr<RHIShader>      m_CopyVert;
    std::unique_ptr<RHIShader>      m_CopyFrag;
    std::unique_ptr<RHIPipeline>    m_CopyPipeline;
    std::unique_ptr<RHIResourceSet> m_CopySet;

    // Skybox: skybox.vert/frag.
    std::unique_ptr<RHIShader>      m_SkyVert;
    std::unique_ptr<RHIShader>      m_SkyFrag;
    std::unique_ptr<RHIPipeline>    m_SkyPipeline;
    std::unique_ptr<RHIBuffer>      m_SkyUBO;
    std::unique_ptr<RHIResourceSet> m_SkySet;

    // Forward PBR: pbr_surface.vert/frag.
    std::unique_ptr<RHIShader>      m_FwdVert;
    std::unique_ptr<RHIShader>      m_FwdFrag;
    std::unique_ptr<RHIPipeline>    m_FwdPipeline;
    std::unique_ptr<RHIBuffer>      m_FwdUBO;
    std::unique_ptr<RHIResourceSet> m_FwdSet;
};

} // namespace Diamond
