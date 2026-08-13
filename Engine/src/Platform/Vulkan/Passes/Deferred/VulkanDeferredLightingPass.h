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

    // The shared DDGI probe atlases (SceneRenderer owns them; every view graph
    // imports them). Handles are invalid on a device without ray tracing, in
    // which case 'fallback' fills the three bindings so the descriptor set stays
    // complete — giMode never selects the probe path there, so the contents are
    // never read.
    struct DDGIAtlases {
        RGTextureHandle irradiance;
        RGTextureHandle visibility;
        RGTextureHandle probeData;
        RHITexture*     fallback = nullptr;
        bool Valid() const { return irradiance.IsValid(); }
    };

    // Everything DDGISampleIrradiance needs from a volume. View-independent, so
    // one set of values serves every view. Mirrors the fields VulkanDDGIPass
    // derives in its own SetFrameData — see SetGIParams.
    struct DDGISampleParams {
        glm::vec3  origin{ 0.0f };   // world position of probe (0,0,0)
        glm::vec3  spacing{ 1.0f };
        glm::ivec3 counts{ 0 };
        float      normalBias = 0.1f;
        float      viewBias   = 0.1f;
        float      energy     = 1.0f;
    };

    VulkanDeferredLightingPass(RHIDevice* device, const std::string& shaderDir);
    ~VulkanDeferredLightingPass();

    // Register the lighting pass: reads the G-buffer (view pos/normal, albedo,
    // material, emissive) + blurred SSAO + the cascade depth maps, and MRT-writes
    // the HDR 'output' target and 'indirect' (both RGBA16F). 'indirect' carries
    // the far-field diffuse irradiance this pass used, before the kD/albedo/AO
    // multiplier — ssgi_composite subtracts it so SSGI replaces that term
    // instead of stacking on it, and it carries DDGI probe irradiance once
    // probes exist. Sets are built once from the graph's pooled textures.
    // 'spotShadows' are the spot pass's OWN depth maps (rendered + transitioned
    // by its Record before this graph executes, so they take no graph reads);
    // returns the pass so a caller can append reads.
    RGPass& AddToGraph(RHIRenderGraph& graph,
                       RGTextureHandle viewPos, RGTextureHandle viewNormal,
                       RGTextureHandle albedo,  RGTextureHandle material,
                       RGTextureHandle ao,      RGTextureHandle emissive,
                       const std::array<RGTextureHandle, NUM_CASCADES>& cascades,
                       const std::array<RHITexture*, NUM_SPOTS>& spotShadows,
                       const DDGIAtlases& ddgi,
                       RGTextureHandle output, RGTextureHandle indirect);

    // Graph-handle spot maps (demo scaffolds): resolves the handles and forwards,
    // then reads them so the graph orders after their writers and transitions
    // never-written dummies to a sampleable layout.
    void AddToGraph(RHIRenderGraph& graph,
                    RGTextureHandle viewPos, RGTextureHandle viewNormal,
                    RGTextureHandle albedo,  RGTextureHandle material,
                    RGTextureHandle ao,      RGTextureHandle emissive,
                    const std::array<RGTextureHandle, NUM_CASCADES>& cascades,
                    const std::array<RGTextureHandle, NUM_SPOTS>& spotShadows,
                    const DDGIAtlases& ddgi,
                    RGTextureHandle output, RGTextureHandle indirect);

    // Sky/IBL ambient multiplier (SkyLightComponent::intensity, default 1). Read
    // by the next SetFrameData, so call it before that. Scales the irradiance and
    // prefiltered specular together — and therefore the 'indirect' MRT output, so
    // the SSGI composite's subtraction stays exact.
    void SetAmbientIntensity(float intensity) { m_AmbientIntensity = intensity; }

    // GI tier + the volume the probe path samples. 'giMode' is 2 for DDGI
    // probes, 0/1 for the IBL irradiance cubemap — one uniform branch in the
    // shader, no permutations. 'aoIndirectStrength' fades GTAO visibility on the
    // indirect diffuse term; since slice 4.5 that irradiance is fetched along
    // the bent normal, so the multiply is a solid-angle fraction rather than a
    // second occlusion test and BOTH tiers pass the user's value straight
    // through. 'volume' is ignored unless giMode is 2. Read by the next
    // SetFrameData, so call it before that.
    void SetGIParams(int giMode, float aoIndirectStrength,
                     const DDGISampleParams& volume) {
        m_GIMode               = giMode;
        m_SSAOIndirectStrength = aoIndirectStrength;
        m_DDGIVolume           = volume;
    }

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
    // 'spotMatrices' from the spot-shadow pass. 'pointRadius' is each point
    // light's influence range — the shader's falloff window reaches zero there,
    // which must match the sphere GatherLights culls with or lights pop at the
    // frustum edge; missing entries mean "unbounded" (plain inverse-square).
    // Call after RHIDevice::BeginFrame and before graph.Execute.
    void SetFrameData(const glm::mat4& view,
                      const glm::vec3& sunDirWorld, const glm::vec3& sunColor,
                      const std::array<glm::mat4, NUM_CASCADES>& lightMatrices,
                      const std::array<float, NUM_CASCADES>& splits,
                      const std::vector<glm::vec3>& pointPosWorld,
                      const std::vector<glm::vec3>& pointColor,
                      float pointShadowFar = 25.0f,
                      const std::vector<SpotLightInfo>& spots = {},
                      const std::array<glm::mat4, NUM_SPOTS>& spotMatrices = {},
                      const std::vector<float>& pointRadius = {});

    // The per-frame lighting UBO, so another pass can shade with the same lights
    // without duplicating SetFrameData's world→view transform work. DDGI's
    // closest-hit shader binds it and declares the identical std140 block.
    RHIBuffer* UBO() const { return m_UBO.get(); }

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
        glm::vec4 pointPos[4];              // .xyz view-space position, .w radius
        glm::vec4 pointColor[4];
        glm::vec4 pointPosWorld[4];
        glm::mat4 spotFromView[NUM_SPOTS];
        glm::vec4 spotPosView[NUM_SPOTS];   // .xyz view-space position, .w cos(outer)
        glm::vec4 spotDirView[NUM_SPOTS];   // .xyz view-space direction, .w cos(inner)
        glm::vec4 spotColor[NUM_SPOTS];     // .xyz radiant intensity, .w range
        glm::vec4 counts;   // x = numPointLights, y = prefilter max LOD,
                            // z = numSpotLights, w = point-shadow far plane
        glm::vec4 ambient;  // x = sky/IBL intensity, y = giMode,
                            // z = SSAO strength on the indirect diffuse term
        // Only the volume fields DDGISampleIrradiance reads — no matrices, the
        // lookup is view-independent. Junk unless ambient.y selects the probe path.
        glm::vec4  ddgiOrigin;
        glm::vec4  ddgiSpacing;
        glm::ivec4 ddgiCounts;
        glm::vec4  ddgiParams;   // x = normalBias, y = energy, z = viewBias
    };

    RHIDevice*                      m_Device;
    std::string                     m_ShaderDir;
    LightingUBO                     m_UBOData{};
    float                           m_PrefilterMaxLod = 0.0f;
    float                           m_AmbientIntensity = 1.0f;
    int                             m_GIMode = 0;
    float                           m_SSAOIndirectStrength = 1.0f;
    DDGISampleParams                m_DDGIVolume{};

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
