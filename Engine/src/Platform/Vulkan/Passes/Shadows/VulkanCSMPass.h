#pragma once

#include "Renderer/RHI/RHIRenderGraph.h"

#include <glm/glm.hpp>

#include <array>
#include <functional>
#include <memory>
#include <string>

namespace Diamond {

class RHIDevice;
class RHIShader;
class RHIPipeline;

// Cascaded shadow maps — Vulkan port of OpenGLCSMPass. Splits the camera frustum
// into NUM_CASCADES depth slices and renders the scene from the sun for each, into
// one depth target per cascade. A depth-only pipeline (no color attachments, front-
// face culling to curb peter-panning); the per-cascade light matrix is folded with
// each draw's model into a single push-constant, so the pass needs no descriptors.
//
// Follows the ported-pass template: the pass owns its pipeline + shaders; the graph
// owns the cascade depth textures (declared Depth32F, which the graph now also makes
// Sampled so the lighting pass can read them). Call ComputeCascades each frame
// (after BeginFrame) to refresh the light matrices + split depths, then Execute.
class VulkanCSMPass {
public:
    static constexpr int NUM_CASCADES = 4;

    VulkanCSMPass(RHIDevice* device, const std::string& shaderDir);
    ~VulkanCSMPass();

    // Recompute the per-cascade light-space matrices + split depths from the camera
    // frustum and sun direction. 'near'/'far' bound the shadowed range (typically
    // tighter than the camera's far plane).
    void ComputeCascades(const glm::vec3& lightDir,
                         const glm::mat4& cameraView,
                         const glm::mat4& cameraProj,
                         float near, float far);

    // Register one depth pass per cascade. 'cascades' are caller-declared Depth32F
    // targets (one per cascade). drawScene records the scene; it receives the current
    // cascade's light-space matrix so it can push lightSpace * model per draw.
    void AddToGraph(RHIRenderGraph& graph,
                    const std::array<RGTextureHandle, NUM_CASCADES>& cascades,
                    std::function<void(RHICommandList*, const glm::mat4& lightSpace)> drawScene);

    // Consumed by the deferred-lighting pass once it lands.
    const std::array<glm::mat4, NUM_CASCADES>& GetLightMatrices() const { return m_LightMatrices; }
    const std::array<float,     NUM_CASCADES>& GetSplitDepths()   const { return m_SplitDepths; }

    // The skinned depth pipeline (csm_depth_skinned: empty set 0, set 1 = bone
    // palette, wider vertex layout). drawScene binds this before recording skinned
    // casters, pushing lightSpace * model and binding a set-1 bone set.
    RHIPipeline* SkinnedPipeline() const { return m_SkinnedPipeline.get(); }

private:
    RHIDevice*                   m_Device;
    std::unique_ptr<RHIShader>   m_Vert;
    std::unique_ptr<RHIShader>   m_Frag;
    std::unique_ptr<RHIShader>   m_SkinnedVert;
    std::unique_ptr<RHIPipeline> m_Pipeline;
    std::unique_ptr<RHIPipeline> m_SkinnedPipeline;

    std::array<glm::mat4, NUM_CASCADES> m_LightMatrices{};
    std::array<float,     NUM_CASCADES> m_SplitDepths{};
};

} // namespace Diamond
