#pragma once

#include "Renderer/RHI/RHIRenderGraph.h"
#include "Renderer/RHI/RHIResources.h"
#include "Platform/Vulkan/Passes/Shadows/VulkanCSMPass.h"   // NUM_CASCADES
#include "Scene/Components.h"                                // DDGIVolumeComponent

#include <glm/glm.hpp>

#include <array>
#include <cstdint>
#include <memory>
#include <string>

namespace Diamond {

class RHIDevice;
class RHIBuffer;
class RHIShader;
class RHIPipeline;
class RHIResourceSet;
class RHIAccelStruct;
class VulkanIBLPass;

// Dynamic Diffuse Global Illumination — probe update (Docs/gi-design.md slice 3).
//
// Five graph passes:
//   DDGIProbeTrace     (RT)      raysPerProbe x probeCount rays through the TLAS,
//                                shaded by the closest-hit shader into a ray-data
//                                image. Consumes LAST frame's irradiance atlas
//                                for the recursive multi-bounce term.
//   DDGIBlendIrradiance(compute) folds those rays into the octahedral irradiance
//                                atlas with hysteresis, borders included.
//   DDGIBlendVisibility(compute) same for the depth/depth^2 pair the Chebyshev
//                                occlusion test needs.
//   DDGIRelocate       (compute) one thread per probe: nudge probes out of walls
//                                and deactivate the ones stuck inside geometry.
//   DDGIProbeDebug     (raster)  optional sphere impostors shaded by their own
//                                irradiance, drawn over the tonemapped image.
//
// SLICE 3 SCOPE: nothing here lights the scene. The atlases exist and converge,
// and only the debug viz reads them — deferred_lighting.frag starts sampling
// them in slice 4, at which point gIndirect carries probe irradiance instead of
// the IBL sample and the SSGI composite needs no change at all.
//
// Per view, like every other pass in this renderer: probes are view-independent,
// so a second open view doubles the probe cost. Sharing one probe set across
// graphs is a slice-5 optimisation, not a correctness issue.
//
// Inert on a device without ray tracing: the constructor builds nothing and
// AddToGraph registers nothing, so tier 1 pays zero.
class VulkanDDGIPass {
public:
    static constexpr int NUM_CASCADES = VulkanCSMPass::NUM_CASCADES;

    // Must match ddgi_common.glsl. The atlases are allocated for the MAXIMUM
    // grid because graph textures are declared once while probe counts are
    // live-editable; a smaller grid uses the top-left sub-rect.
    static constexpr int kIrradianceRes    = 8;
    static constexpr int kVisibilityRes    = 16;
    static constexpr int kMaxProbesPerAxis = 16;
    static constexpr int kMaxProbes        = kMaxProbesPerAxis * kMaxProbesPerAxis
                                           * kMaxProbesPerAxis;
    static constexpr int kMaxRays          = 128;
    // Size of the bindless albedo array the closest-hit shader indexes. Fixed at
    // pipeline creation and matched by ddgi_probe_trace.rchit; every slot is
    // written at set creation (padded with 1×1 white), so no partially-bound
    // descriptor-indexing feature is involved.
    static constexpr uint32_t kMaxTextures = 256;

    // Atlas dimensions in texels: (probesX * probesY) tiles across by probesZ
    // down, each tile res + 2 square.
    static constexpr uint32_t IrradianceWidth() {
        return kMaxProbesPerAxis * kMaxProbesPerAxis * (kIrradianceRes + 2);
    }
    static constexpr uint32_t IrradianceHeight() {
        return kMaxProbesPerAxis * (kIrradianceRes + 2);
    }
    static constexpr uint32_t VisibilityWidth() {
        return kMaxProbesPerAxis * kMaxProbesPerAxis * (kVisibilityRes + 2);
    }
    static constexpr uint32_t VisibilityHeight() {
        return kMaxProbesPerAxis * (kVisibilityRes + 2);
    }

    // Where the probes actually sit, derived from the component + the volume's
    // world centre. Static because the lighting pass has to agree with it
    // exactly — a probe grid the sampler and the blend disagree about reads
    // irradiance from the wrong probes — and because the view that samples the
    // atlases may have no DDGI pass instance of its own.
    struct GridLayout {
        glm::vec3  origin;    // world position of probe (0,0,0)
        glm::vec3  spacing;
        glm::ivec3 counts;
    };
    static GridLayout ComputeGrid(const DDGIVolumeComponent& volume,
                                  const glm::vec3& center);

    // 'debugTargetFormat' is the format of the LDR image the probe viz draws
    // over — dynamic rendering rejects a mismatched pipeline.
    VulkanDDGIPass(RHIDevice* device, const std::string& shaderDir,
                   RHIFormat debugTargetFormat);
    ~VulkanDDGIPass();

    bool Available() const { return m_TracePipeline != nullptr; }

    // 'rayData' is (kMaxRays x kMaxProbes) RGBA16F storage. The three atlases are
    // persistent AND storage: written by compute, sampled by the trace through
    // ReadHistory. 'probeData' is (kMaxProbes x 1) RGBA16F — rgb = relocation
    // offset, a = active flag. 'depth' is the G-buffer depth the probe viz tests
    // against; 'debugTarget' the tonemapped LDR image it draws over.
    void AddToGraph(RHIRenderGraph& graph,
                    RGTextureHandle rayData,
                    RGTextureHandle irradiance,
                    RGTextureHandle visibility,
                    RGTextureHandle probeData,
                    const std::array<RGTextureHandle, NUM_CASCADES>& cascades,
                    RGTextureHandle debugTarget, RGTextureHandle depth,
                    bool toSwapchain);

    // The scene TLAS. A different pointer than last time rebuilds the trace
    // descriptor set; null leaves the whole pass dormant for the frame.
    void SetTLAS(RHIAccelStruct* tlas);

    // Per-instance {vertexAddress, indexAddress, albedo, emissive} table, indexed
    // in the closest-hit shader by gl_InstanceCustomIndexEXT. SceneRenderer owns
    // it and rebuilds it alongside the TLAS, so this follows the same
    // pointer-changed-means-rebuild-the-set rule as SetTLAS.
    void SetGeometryBuffer(RHIBuffer* geometry);

    // The deferred pass's lighting UBO — probe rays shade with exactly the same
    // lights, in the same view space. Call once after construction.
    void SetLightingUBO(RHIBuffer* ubo);

    // The bindless albedo table each RTGeometry record indexes. Slot 0 must be a
    // neutral (white) texture: it's what untextured materials resolve to, and
    // it's what pads the unused tail of the array. Only call when the table
    // actually grew — this rebuilds the trace set, which blocks on WaitIdle.
    void SetAlbedoTextures(const std::vector<RHITexture*>& textures);

    // The baked radiance cubemap the miss shader samples as sky. Written raw into
    // the trace set (it isn't an RHITexture), so it must be re-applied whenever
    // that set is rebuilt — the pass handles that internally; callers just call
    // this again after an environment re-bake.
    void BindEnvironment(const VulkanIBLPass& ibl);

    // This frame's volume + camera. After RHIDevice::BeginFrame, before Execute.
    // 'skyIntensity' is SkyLightComponent::intensity, matching what the deferred
    // pass folds into its ambient. Passing a null volume disables the pass for
    // the frame without touching the atlases.
    void SetFrameData(const DDGIVolumeComponent* volume, const glm::vec3& center,
                      const glm::mat4& view, const glm::mat4& projection,
                      float skyIntensity, float exposure);

    void SetEnabled(bool enabled);
    bool IsEnabled() const { return m_Enabled; }

    // Forget the accumulated atlases — the next frame blends with hysteresis 0
    // and relocation restarts from zero offsets. For scene loads and any
    // discontinuity the temporal blend would otherwise smear across.
    void InvalidateHistory() { m_HistoryValid = false; m_FramesSinceReset = 0; }

private:
    // std140, matching ddgi_common.glsl's DDGIVolume. 288 bytes.
    struct DDGIUBO {
        glm::mat4  view;
        glm::mat4  viewProj;
        glm::mat4  rayRotation;
        glm::vec4  origin;
        glm::vec4  spacing;
        glm::ivec4 counts;
        glm::vec4  params;    // hysteresis, normalBias, energy, maxRayDistance
        glm::vec4  params2;   // skyIntensity, viewBias, depthSharpness, rayCount
        glm::vec4  params3;   // historyValid, backfaceThreshold, debugRadius, debugExposure
    };

    // Rebuilds the trace descriptor set (and re-applies the raw env-cube write).
    // Deferred to execute time: the TLAS and geometry table may not exist when
    // the graph is built, and either can be REPLACED rather than updated.
    void RebuildTraceSet();

    RHIDevice*  m_Device;
    std::string m_ShaderDir;
    DDGIUBO     m_UBOData{};

    bool m_Enabled      = true;
    bool m_HistoryValid = false;   // false = blend ignores the atlases' contents
    bool m_HasVolume    = false;   // no DDGIVolumeComponent in the scene
    bool m_ShowProbes   = false;
    bool m_SetDirty     = true;

    // Live grid, cached from SetFrameData so the execute lambdas can size their
    // dispatches without holding a component reference.
    glm::ivec3 m_Counts     { 0 };
    int        m_ProbeCount = 0;
    int        m_RayCount   = 0;
    uint32_t   m_FrameIndex = 0;
    // Frames of probe update since the last history reset. Drives a progressive
    // average over the opening frames instead of the user's (very high)
    // hysteresis, so a scene load or a moved sun converges immediately rather
    // than ramping on screen for a second.
    uint32_t   m_FramesSinceReset = 0;

    RHIAccelStruct* m_TLAS     = nullptr;   // not owned
    RHIBuffer*      m_Geometry = nullptr;   // not owned (SceneRenderer)
    RHIBuffer*      m_LightUBO = nullptr;   // not owned (deferred lighting pass)
    const VulkanIBLPass* m_IBL = nullptr;   // not owned
    std::vector<RHITexture*> m_AlbedoTextures;   // not owned (SceneRenderer pins them)

    std::unique_ptr<RHIShader> m_Raygen;
    std::unique_ptr<RHIShader> m_Miss;
    std::unique_ptr<RHIShader> m_ClosestHit;
    std::unique_ptr<RHIShader> m_BlendIrradianceComp;
    std::unique_ptr<RHIShader> m_BlendVisibilityComp;
    std::unique_ptr<RHIShader> m_RelocateComp;
    std::unique_ptr<RHIShader> m_DebugVert;
    std::unique_ptr<RHIShader> m_DebugFrag;

    std::unique_ptr<RHIPipeline> m_TracePipeline;
    std::unique_ptr<RHIPipeline> m_BlendIrradiancePipeline;
    std::unique_ptr<RHIPipeline> m_BlendVisibilityPipeline;
    std::unique_ptr<RHIPipeline> m_RelocatePipeline;
    std::unique_ptr<RHIPipeline> m_DebugPipeline;

    std::unique_ptr<RHIBuffer> m_UBO;

    std::unique_ptr<RHIResourceSet> m_TraceSet;
    std::unique_ptr<RHIResourceSet> m_BlendIrradianceSet;
    std::unique_ptr<RHIResourceSet> m_BlendVisibilitySet;
    std::unique_ptr<RHIResourceSet> m_RelocateSet;
    std::unique_ptr<RHIResourceSet> m_DebugSet;

    // Graph textures resolved once in AddToGraph — the trace set is rebuilt
    // later, outside the graph, and needs them again.
    RHITexture* m_RayDataTex    = nullptr;
    RHITexture* m_IrradianceTex = nullptr;
    RHITexture* m_VisibilityTex = nullptr;
    RHITexture* m_ProbeDataTex  = nullptr;
    std::array<RHITexture*, NUM_CASCADES> m_CascadeTex{};
};

} // namespace Diamond
