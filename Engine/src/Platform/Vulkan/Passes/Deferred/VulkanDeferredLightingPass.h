#pragma once

#include "Renderer/RHI/RHIRenderGraph.h"
#include "Renderer/RHI/RHIResources.h"   // RHITextureBinding
#include "Platform/Vulkan/Passes/Shadows/VulkanCSMPass.h"        // NUM_CASCADES
#include "Platform/Vulkan/Passes/Shadows/VulkanSpotShadowPass.h" // MAX_SPOTS + SpotLightInfo

#include <glm/glm.hpp>

#include <array>
#include <memory>
#include <string>
#include <vector>

namespace Diamond {

class RHIDevice;
class RHIBuffer;
class RHIShader;
class RHIPipeline;
class RHIResourceSet;
class VulkanIBLPass;
class VulkanPointShadowPass;

// Deferred lighting — Vulkan port of OpenGLDeferredLightingPass. A fullscreen
// pass that resolves the G-buffer + SSAO + cascaded shadow maps into one HDR
// color target with Cook-Torrance PBR. This is the pass that ties the deferred
// chain together (G-buffer + SSAO + CSM); IBL and forward rendering build on it.
//
// All lighting is done in VIEW space (the G-buffer stores view-space pos/normal),
// so the pass needs no world reconstruction: the camera is the origin and CSM
// reuses lightMatrix * inverse(view). Scope: directional sun with CSM shadows,
// point lights with distance-cubemap shadows, spot lights with perspective shadow
// maps, and IBL ambient (invView reconstructs the world position/direction the
// cube-shadow and IBL lookups need).
//
// Follows the ported-pass template: the pass owns its pipeline + shaders, the
// per-frame lighting UBO, and the descriptor set binding the G-buffer/SSAO/
// cascade reads; the graph owns the transient textures. The set is built once;
// SetFrameData refreshes the UBO each frame (after BeginFrame, before Execute).
class VulkanDeferredLightingPass {
public:
    static constexpr int NUM_CASCADES = VulkanCSMPass::NUM_CASCADES;
    static constexpr int NUM_SPOTS    = VulkanSpotShadowPass::MAX_SPOTS;

    VulkanDeferredLightingPass(RHIDevice* device, const std::string& shaderDir);
    ~VulkanDeferredLightingPass();

    // Register the lighting pass: reads the G-buffer (view pos/normal, albedo,
    // material, emissive) + blurred SSAO + the cascade depth maps, and writes the
    // HDR 'output' target (RGBA16F). Sets are built once from the graph's pooled
    // textures. 'spotShadows' are the spot pass's OWN depth maps (rendered +
    // transitioned by its Record before this graph executes, so they take no
    // graph reads); returns the pass so a caller can append reads.
    RGPass& AddToGraph(RHIRenderGraph& graph,
                       RGTextureHandle viewPos, RGTextureHandle viewNormal,
                       RGTextureHandle albedo,  RGTextureHandle material,
                       RGTextureHandle ssao,    RGTextureHandle emissive,
                       const std::array<RGTextureHandle, NUM_CASCADES>& cascades,
                       const std::array<RHITexture*, NUM_SPOTS>& spotShadows,
                       RGTextureHandle output);

    // Graph-handle spot maps (demo scaffolds): resolves the handles and forwards,
    // then reads them so the graph orders after their writers and transitions
    // never-written dummies to a sampleable layout.
    void AddToGraph(RHIRenderGraph& graph,
                    RGTextureHandle viewPos, RGTextureHandle viewNormal,
                    RGTextureHandle albedo,  RGTextureHandle material,
                    RGTextureHandle ssao,    RGTextureHandle emissive,
                    const std::array<RGTextureHandle, NUM_CASCADES>& cascades,
                    const std::array<RGTextureHandle, NUM_SPOTS>& spotShadows,
                    RGTextureHandle output);

    // Bind the baked IBL maps (irradiance/prefilter cubemaps + BRDF LUT) into the
    // resource set's IBL slots (bindings 11-13). The maps are world-space cubemaps,
    // not RHITextures, so they're written raw. Call once after AddToGraph (which
    // creates the set) and before the frame loop; the baked maps are static.
    void BindIBL(const VulkanIBLPass& ibl);

    // Bind the point-shadow cubes into slots 18-21 — same raw-write pattern as
    // BindIBL, except the cubes are per-frame-in-flight so each frame slot gets its
    // own views. Call once after AddToGraph; the views are stable for the pass's
    // lifetime (only their contents are re-rendered).
    void BindPointShadows(const VulkanPointShadowPass& shadows);

    // Upload this frame's lighting data into the UBO. The sun direction + point/
    // spot lights are given in WORLD space and transformed to view space here (the
    // shader works in view space). 'lightMatrices'/'splits' come from the CSM pass,
    // 'spotMatrices' from the spot-shadow pass. Call after RHIDevice::BeginFrame
    // and before graph.Execute.
    void SetFrameData(const glm::mat4& view,
                      const glm::vec3& sunDirWorld, const glm::vec3& sunColor,
                      const std::array<glm::mat4, NUM_CASCADES>& lightMatrices,
                      const std::array<float, NUM_CASCADES>& splits,
                      const std::vector<glm::vec3>& pointPosWorld,
                      const std::vector<glm::vec3>& pointColor,
                      float pointShadowFar = 25.0f,
                      const std::vector<SpotLightInfo>& spots = {},
                      const std::array<glm::mat4, NUM_SPOTS>& spotMatrices = {});

    // Hot-reload (editor only): recompiles fullscreen.vert + deferred_lighting.frag
    // from shaderDir's current .spv and rebuilds the pipeline IN PLACE (same
    // 'this' — the graph pass reads m_Pipeline at Execute time, no graph
    // rebuild). Also rebuilds the descriptor set eagerly from the texture
    // bindings AddToGraph captured (its graph pass only runs once at
    // graph-build time, so nothing else would ever repopulate it) — the caller
    // must re-call BindIBL and BindPointShadows right after, since those write
    // the IBL/point-shadow slots separately. Caller must WaitIdle() first. A
    // no-op (logs) if the new SPIR-V is missing/corrupt.
    void Reload();

private:
    // std140 layout matching deferred_lighting.frag's LightingUBO.
    struct LightingUBO {
        glm::mat4 lightFromView[NUM_CASCADES];
        glm::mat4 invView;
        glm::vec4 cascadeSplits;
        glm::vec4 sunDirView;
        glm::vec4 sunColor;
        glm::vec4 pointPos[4];
        glm::vec4 pointColor[4];
        glm::vec4 pointPosWorld[4];
        glm::mat4 spotFromView[NUM_SPOTS];
        glm::vec4 spotPosView[NUM_SPOTS];   // .xyz view-space position, .w cos(outer)
        glm::vec4 spotDirView[NUM_SPOTS];   // .xyz view-space direction, .w cos(inner)
        glm::vec4 spotColor[NUM_SPOTS];
        glm::vec4 counts;   // x = numPointLights, y = prefilter max LOD,
                            // z = numSpotLights, w = point-shadow far plane
    };

    RHIDevice*                      m_Device;
    std::string                     m_ShaderDir;
    LightingUBO                     m_UBOData{};
    float                           m_PrefilterMaxLod = 0.0f;

    std::unique_ptr<RHIShader>      m_Vert;
    std::unique_ptr<RHIShader>      m_Frag;
    std::unique_ptr<RHIPipeline>    m_Pipeline;
    std::unique_ptr<RHIBuffer>      m_UBO;
    std::unique_ptr<RHIResourceSet> m_Set;
    // The texture bindings AddToGraph built the set from — kept so Reload can
    // rebuild the set without the graph/handles (the underlying pooled
    // RHITexture* addresses are stable for the pass's lifetime).
    std::vector<RHITextureBinding>  m_TexBindings;
};

} // namespace Diamond
