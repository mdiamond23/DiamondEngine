#pragma once

#include "Renderer/RHI/RHIRenderGraph.h"
#include "Platform/Vulkan/Passes/Shadows/VulkanCSMPass.h"   // NUM_CASCADES

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

// Deferred lighting — Vulkan port of OpenGLDeferredLightingPass. A fullscreen
// pass that resolves the G-buffer + SSAO + cascaded shadow maps into one HDR
// color target with Cook-Torrance PBR. This is the pass that ties the deferred
// chain together (G-buffer + SSAO + CSM); IBL and forward rendering build on it.
//
// All lighting is done in VIEW space (the G-buffer stores view-space pos/normal),
// so the pass needs no world reconstruction: the camera is the origin and CSM
// reuses lightMatrix * inverse(view). Slice scope mirrors what's ported so far —
// directional sun with CSM shadows + point lights (unshadowed) + a constant
// ambient placeholder standing in for IBL until that pass lands.
//
// Follows the ported-pass template: the pass owns its pipeline + shaders, the
// per-frame lighting UBO, and the descriptor set binding the G-buffer/SSAO/
// cascade reads; the graph owns the transient textures. The set is built once;
// SetFrameData refreshes the UBO each frame (after BeginFrame, before Execute).
class VulkanDeferredLightingPass {
public:
    static constexpr int NUM_CASCADES = VulkanCSMPass::NUM_CASCADES;

    VulkanDeferredLightingPass(RHIDevice* device, const std::string& shaderDir);
    ~VulkanDeferredLightingPass();

    // Register the lighting pass: reads the G-buffer (view pos/normal, albedo,
    // material, emissive) + blurred SSAO + the cascade depth maps, and writes the
    // HDR 'output' target (RGBA16F). Sets are built once from the graph's pooled
    // textures.
    void AddToGraph(RHIRenderGraph& graph,
                    RGTextureHandle viewPos, RGTextureHandle viewNormal,
                    RGTextureHandle albedo,  RGTextureHandle material,
                    RGTextureHandle ssao,    RGTextureHandle emissive,
                    const std::array<RGTextureHandle, NUM_CASCADES>& cascades,
                    RGTextureHandle output);

    // Upload this frame's lighting data into the UBO. The sun direction + point
    // lights are given in WORLD space and transformed to view space here (the
    // shader works in view space). 'lightMatrices'/'splits' come from the CSM
    // pass. Call after RHIDevice::BeginFrame and before graph.Execute.
    void SetFrameData(const glm::mat4& view,
                      const glm::vec3& sunDirWorld, const glm::vec3& sunColor,
                      const glm::vec3& ambient,
                      const std::array<glm::mat4, NUM_CASCADES>& lightMatrices,
                      const std::array<float, NUM_CASCADES>& splits,
                      const std::vector<glm::vec3>& pointPosWorld,
                      const std::vector<glm::vec3>& pointColor);

private:
    // std140 layout matching deferred_lighting.frag's LightingUBO.
    struct LightingUBO {
        glm::mat4 lightFromView[NUM_CASCADES];
        glm::vec4 cascadeSplits;
        glm::vec4 sunDirView;
        glm::vec4 sunColor;
        glm::vec4 ambient;
        glm::vec4 pointPos[4];
        glm::vec4 pointColor[4];
        glm::vec4 counts;
    };

    RHIDevice*                      m_Device;
    LightingUBO                     m_UBOData{};

    std::unique_ptr<RHIShader>      m_Vert;
    std::unique_ptr<RHIShader>      m_Frag;
    std::unique_ptr<RHIPipeline>    m_Pipeline;
    std::unique_ptr<RHIBuffer>      m_UBO;
    std::unique_ptr<RHIResourceSet> m_Set;
};

} // namespace Diamond
