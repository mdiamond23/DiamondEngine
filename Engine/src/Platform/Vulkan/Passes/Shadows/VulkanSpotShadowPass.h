#pragma once

#include "Renderer/RHI/RHIResources.h"

#include <glm/glm.hpp>

#include <array>
#include <functional>
#include <memory>
#include <string>
#include <vector>

namespace Diamond {

class RHIDevice;
class RHICommandList;
class RHIShader;
class RHIPipeline;

// One gathered spot light, world space. Cone angles are in degrees (the lighting
// pass computes the cosines it needs); 'range' bounds the light's influence and
// the shadow projection's far plane.
struct SpotLightInfo {
    glm::vec3 position { 0.0f };
    glm::vec3 direction{ 0.0f, -1.0f, 0.0f };
    glm::vec3 color    { 0.0f };              // radiant intensity (color * intensity)
    float     innerDeg = 15.0f;
    float     outerDeg = 30.0f;
    float     range    = 10.0f;
};

// Spot-light shadow maps — one perspective depth target per spot light, following
// the CSM pass shape: a depth-only pipeline (csm_depth shaders reused — they are
// a generic "push lightSpace * model, write depth" pass), the per-light matrix
// folded with each draw's model into a single push constant.
//
// Unlike the CSM cascades (whose fit is per-camera, so each view's graph renders
// its own), spot maps are view-independent — so the pass OWNS its depth targets
// (per-frame-in-flight internally, like any RHI render target) and Record()s all
// of them once per frame, before either view's graph executes. Both views'
// lighting sets then bind the same maps (the point-shadow pattern, but expressible
// in plain RHI since the targets are 2D). Inactive slots still clear their map
// (cheap, keeps the lighting set valid to sample) but record no draws.
class VulkanSpotShadowPass {
public:
    static constexpr int      MAX_SPOTS   = 4;
    static constexpr uint32_t kResolution = 1024;   // GL parity (was kSpotShadowRes)

    VulkanSpotShadowPass(RHIDevice* device, const std::string& shaderDir);
    ~VulkanSpotShadowPass();

    // Rebuild the per-spot light-space matrices from this frame's gathered spot
    // lights (at most MAX_SPOTS are used). Pure CPU work — callable any time.
    // A slot whose matrix changed (light moved / a new light took the slot) has its
    // static cache marked dirty here, so a moved light re-bakes its static map.
    void ComputeMatrices(const std::vector<SpotLightInfo>& spots);

    // Invalidate every active slot's static cache — call when the set of static
    // shadow casters (or any static caster's transform) changed this frame, since
    // the cache can't detect that itself. Re-bakes over the next frames-in-flight.
    void MarkStaticDirty();

    // Render every slot's map for this frame and leave each in SampledRead for the
    // lighting passes. Call between RHIDevice::BeginFrame and the first graph
    // Execute, after ComputeMatrices.
    //
    // Static shadow caching: each slot keeps a cached depth map of its STATIC
    // casters, re-baked only when dirty (drawStatic). Every frame the working map
    // is seeded from that cache (a fullscreen depth copy) and only this frame's
    // DYNAMIC casters are drawn over it (drawDynamic) — so unchanged level geometry
    // isn't re-drawn per frame. Both callbacks receive the slot's light-space
    // matrix and push lightSpace * model per draw (the CSM callback shape).
    void Record(RHICommandList* cmd,
                const std::function<void(RHICommandList*, const glm::mat4& lightSpace)>& drawStatic,
                const std::function<void(RHICommandList*, const glm::mat4& lightSpace)>& drawDynamic);

    // The pass-owned depth maps, for the lighting set (slots 14-17). Stable for
    // the pass's lifetime — only their contents are re-rendered.
    std::array<RHITexture*, MAX_SPOTS> Maps() const {
        std::array<RHITexture*, MAX_SPOTS> out{};
        for (int i = 0; i < MAX_SPOTS; ++i) out[i] = m_Maps[i].get();
        return out;
    }

    // Consumed by the deferred-lighting pass (folded with inverse(view) there).
    const std::array<glm::mat4, MAX_SPOTS>& GetLightMatrices() const { return m_LightMatrices; }

    // Skinned depth pipeline (csm_depth_skinned — same as the CSM pass's). Bound by
    // drawScene before recording skinned casters.
    RHIPipeline* SkinnedPipeline() const { return m_SkinnedPipeline.get(); }

private:
    RHIDevice*                   m_Device;
    std::unique_ptr<RHIShader>   m_Vert;
    std::unique_ptr<RHIShader>   m_Frag;
    std::unique_ptr<RHIShader>   m_SkinnedVert;
    std::unique_ptr<RHIPipeline> m_Pipeline;
    std::unique_ptr<RHIPipeline> m_SkinnedPipeline;

    // Working maps (sampled by the lighting pass) + per-slot cached static-caster
    // depth. The copy pipeline (fullscreen.vert + spot_depth_copy.frag) seeds a
    // working map from its static cache each frame; m_CopySets[i] binds m_Static[i].
    std::array<std::unique_ptr<RHITexture>, MAX_SPOTS>      m_Maps;
    std::array<std::unique_ptr<RHITexture>, MAX_SPOTS>      m_Static;
    std::unique_ptr<RHIShader>                             m_CopyVert;
    std::unique_ptr<RHIShader>                             m_CopyFrag;
    std::unique_ptr<RHIPipeline>                           m_CopyPipeline;
    std::array<std::unique_ptr<RHIResourceSet>, MAX_SPOTS> m_CopySets;

    // Per-slot static-cache dirty countdown (in frames). >0 means re-bake the
    // static map this frame; seeded to kFramesInFlight so a single dirty event
    // fills every frame-in-flight copy of the double-buffered target before the
    // cache is trusted. Init nonzero so slots bake on their first active frame.
    std::array<int, MAX_SPOTS>       m_StaticDirty{};

    std::array<glm::mat4, MAX_SPOTS> m_LightMatrices{};
    std::array<glm::mat4, MAX_SPOTS> m_PrevMatrices{};
    int                              m_Count     = 0;
    int                              m_PrevCount = 0;
};

} // namespace Diamond
