#pragma once

#include "Renderer/RHI/RHIRenderGraph.h"
#include "Platform/Vulkan/Passes/Deferred/VulkanDeferredLightingPass.h"   // DDGIAtlases, DDGISampleParams
#include "Platform/Vulkan/Passes/GI/VulkanDDGIPass.h"                     // kMaxTextures

#include <glm/glm.hpp>

#include <cstdint>
#include <memory>
#include <string>
#include <vector>

namespace Diamond {

class RHIDevice;
class RHIBuffer;
class RHIShader;
class RHIPipeline;
class RHIResourceSet;
class RHIAccelStruct;
class RHITexture;
class VulkanIBLPass;

// Ray-traced reflections (Docs/rt-reflections-design.md), slices 5.1-5.2.
//
// A FALLBACK under SSR, not a replacement. SSR runs first and keeps every pixel
// its screen-space march was confident about; this traces only where that
// failed — off-screen geometry, backfaces, rays that left the frame. The pass
// merges both into one target, so ssr_composite.frag samples a single texture
// and never learns RT exists.
//
// Hit shading reuses the DDGI stack wholesale: the same per-instance geometry
// table, the same bindless albedo array, the same lighting UBO, the same
// ray-query shadows. The one substitution is that the hit's indirect term is a
// DDGI lookup AT THE HIT POINT rather than a probe's own previous-frame value.
// Without that term a reflection of anything in shadow would be black.
//
// TWO PIPELINES, ONE GRAPH PASS. Once the composite is re-pointed here, this
// target must be written every frame — but tracing needs a TLAS, a geometry
// table, a lighting UBO and the albedo array, any of which may be missing (an
// empty scene, or before the first TLAS build), and reflections are toggleable.
// So the pass owns a passthrough compute that copies ssrColor through, and
// picks between the two at execute time. The graph still sees one writer.
// Choosing in C++ rather than branching in the raygen is deliberate: the raygen
// calls traceRayEXT, which makes the TLAS descriptor STATICALLY USED and
// therefore required to be valid — an in-shader "no TLAS" branch is not legal.
//
// Inert on a device without ray tracing: the constructor builds nothing,
// Available() is false, and AddToGraph registers nothing — the caller then
// leaves the composite pointed at ssrColor, so tier 1 is bit-for-bit unchanged.
class VulkanRTReflectionPass {
public:
    // Must match rt_reflection.rchit's array and, more importantly, the DDGI
    // pass's — both index the same SceneRenderer-owned texture table.
    static constexpr uint32_t kMaxTextures = VulkanDDGIPass::kMaxTextures;

    // 'width'/'height' are the offscreen G-buffer resolution — the launch grid
    // and the passthrough dispatch grid.
    VulkanRTReflectionPass(RHIDevice* device, const std::string& shaderDir,
                           uint32_t width, uint32_t height);
    ~VulkanRTReflectionPass();

    bool Available() const { return m_Pipeline != nullptr; }

    // Registers one pass: reads ssrColor + the G-buffer + the probe atlases,
    // writes rtReflection (full-res RGBA16F, storage = true — both paths write
    // it via imageStore). Caller must then point the SSR composite at
    // rtReflection instead of ssrColor.
    //
    // Reading the atlases costs no new ordering constraint: it makes this pass
    // depend on DDGIBlendIrradiance, which is already upstream of deferred
    // lighting -> SSGI -> SSR -> here.
    void AddToGraph(RHIRenderGraph& graph,
                    RGTextureHandle ssrColor, RGTextureHandle rtReflection,
                    RGTextureHandle viewPos, RGTextureHandle viewNormal,
                    RGTextureHandle material,
                    const VulkanDeferredLightingPass::DDGIAtlases& ddgi);

    // The scene TLAS. A different pointer than last time rebuilds the trace
    // descriptor set, so a TLAS recreated (not just refit) is picked up. Null
    // drops the pass to its passthrough — correct behaviour, not a failure.
    void SetTLAS(RHIAccelStruct* tlas);

    // Per-instance {vertexAddress, indexAddress, albedo, emissive, slot} table,
    // indexed by gl_InstanceCustomIndexEXT. SceneRenderer owns it and rebuilds
    // it alongside the TLAS, so this follows the same pointer-changed rule.
    void SetGeometryBuffer(RHIBuffer* geometry);

    // The deferred pass's lighting UBO — reflection hits shade with exactly the
    // same lights, in the same view space. Call once after construction, and
    // again whenever the lighting pass is recreated (a resize does that).
    void SetLightingUBO(RHIBuffer* ubo);

    // The bindless albedo table each geometry record indexes. Slot 0 must be a
    // neutral (white) texture — it is what untextured materials resolve to and
    // what pads the unused tail. Only call when the table actually grew; this
    // rebuilds the trace set, which blocks on WaitIdle.
    void SetAlbedoTextures(const std::vector<RHITexture*>& textures);

    // The baked radiance cubemap the miss shader samples as sky. Written raw
    // into the trace set (it isn't an RHITexture), so it must be re-applied
    // whenever that set is rebuilt — handled internally; callers just call this
    // again after an environment re-bake.
    void BindEnvironment(const VulkanIBLPass& ibl);

    // This frame's camera + probe volume. After RHIDevice::BeginFrame, before
    // Execute. 'giMode' 2 means the atlases hold real probe data; anything else
    // drops the hit's indirect term (the scene has no DDGI volume).
    // 'skyIntensity' matches what the deferred pass folds into its ambient.
    void SetFrameData(const glm::mat4& view, int giMode,
                      const VulkanDeferredLightingPass::DDGISampleParams& volume,
                      float skyIntensity);

    // Off routes through the passthrough, so the composite still gets SSR's
    // result. This is the A/B for what RT is contributing.
    void SetEnabled(bool enabled) { m_Enabled = enabled; }
    bool IsEnabled() const { return m_Enabled; }

    // SSR alpha above which SSR keeps the pixel; roughness above which the
    // composite's IBL fade owns it; how far a reflection ray may travel.
    void SetConfidenceThreshold(float threshold);
    void SetRoughnessCutoff(float cutoff);
    void SetMaxRayDistance(float distance);

private:
    // std140, matching rt_reflection.{rgen,rmiss,rchit}'s ReflectBlock.
    struct ReflectUBO {
        glm::mat4  view;
        glm::mat4  invView;
        glm::vec4  ddgiOrigin;
        glm::vec4  ddgiSpacing;
        glm::ivec4 ddgiCounts;
        glm::vec4  ddgiParams;   // x normalBias, y energy, z viewBias, w giMode
        glm::vec4  params;       // x maxRayDistance, y skyIntensity
    };

    // std430 push block matching rt_reflection.rgen's Push.
    struct TracePush {
        float confidenceThreshold;
        float roughnessCutoff;
        float maxRayDistance;
    };

    // Rebuilds the trace descriptor set (and re-applies the raw env-cube write).
    // Deferred to execute time: the TLAS and geometry table may not exist when
    // the graph is built, and either can be REPLACED rather than updated.
    void RebuildTraceSet();

    RHIDevice* m_Device;
    uint32_t   m_Width;
    uint32_t   m_Height;
    ReflectUBO m_UBOData{};

    bool  m_Enabled             = true;
    bool  m_SetDirty            = true;
    float m_ConfidenceThreshold = 0.05f;
    // Sharp reflections only. The composite fades to prefiltered IBL from
    // roughness 0.30, so tracing past that is both wasted and wrong-looking.
    float m_RoughnessCutoff = 0.30f;
    float m_MaxRayDistance  = 100.0f;

    RHIAccelStruct* m_TLAS     = nullptr;   // not owned (SceneRenderer)
    RHIBuffer*      m_Geometry = nullptr;   // not owned (SceneRenderer)
    RHIBuffer*      m_LightUBO = nullptr;   // not owned (deferred lighting pass)
    const VulkanIBLPass* m_IBL = nullptr;   // not owned
    std::vector<RHITexture*> m_AlbedoTextures;   // not owned (SceneRenderer pins them)

    std::unique_ptr<RHIShader>   m_Raygen;
    std::unique_ptr<RHIShader>   m_Miss;
    std::unique_ptr<RHIShader>   m_ClosestHit;
    std::unique_ptr<RHIShader>   m_PassthroughComp;

    std::unique_ptr<RHIPipeline> m_Pipeline;              // ray tracing
    std::unique_ptr<RHIPipeline> m_PassthroughPipeline;   // compute

    std::unique_ptr<RHIBuffer>      m_UBO;
    std::unique_ptr<RHIResourceSet> m_TraceSet;         // deferred — needs the TLAS
    std::unique_ptr<RHIResourceSet> m_PassthroughSet;   // built in AddToGraph

    // Graph textures resolved once in AddToGraph — the trace set is rebuilt
    // later, outside the graph, and needs them again.
    RHITexture* m_ReflectionTex = nullptr;
    RHITexture* m_SSRTex        = nullptr;
    RHITexture* m_ViewPosTex    = nullptr;
    RHITexture* m_ViewNormalTex = nullptr;
    RHITexture* m_MaterialTex   = nullptr;
    RHITexture* m_IrradianceTex = nullptr;
    RHITexture* m_VisibilityTex = nullptr;
    RHITexture* m_ProbeDataTex  = nullptr;
};

} // namespace Diamond
