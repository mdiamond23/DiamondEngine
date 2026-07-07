#pragma once

#include "Renderer/RHI/RHIRenderGraph.h"

#include <glm/glm.hpp>

#include <array>
#include <functional>
#include <memory>
#include <string>
#include <vector>

namespace Diamond {

class RHIDevice;
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
// the CSM pass shape exactly: a depth-only pipeline (csm_depth shaders reused —
// they are a generic "push lightSpace * model, write depth" pass), the per-light
// matrix folded with each draw's model into a single push constant, the graph
// owning the depth targets. Inactive slots still clear their map (cheap, keeps
// the lighting set's reads valid) but record no draws.
class VulkanSpotShadowPass {
public:
    static constexpr int MAX_SPOTS = 4;

    VulkanSpotShadowPass(RHIDevice* device, const std::string& shaderDir);
    ~VulkanSpotShadowPass();

    // Rebuild the per-spot light-space matrices from this frame's gathered spot
    // lights (at most MAX_SPOTS are used). Pure CPU work — callable any time.
    void ComputeMatrices(const std::vector<SpotLightInfo>& spots);

    // Register one depth pass per spot slot. 'maps' are caller-declared Depth32F
    // targets; drawScene records the scene and receives the slot's light-space
    // matrix so it can push lightSpace * model per draw (same callback the CSM
    // pass uses).
    void AddToGraph(RHIRenderGraph& graph,
                    const std::array<RGTextureHandle, MAX_SPOTS>& maps,
                    std::function<void(RHICommandList*, const glm::mat4& lightSpace)> drawScene);

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

    std::array<glm::mat4, MAX_SPOTS> m_LightMatrices{};
    int                              m_Count = 0;
};

} // namespace Diamond
