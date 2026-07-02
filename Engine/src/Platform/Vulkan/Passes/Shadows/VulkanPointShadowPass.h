#pragma once

#include "Platform/Vulkan/VulkanCommon.h"
#include "Platform/Vulkan/RHI/VulkanRHIDevice.h"

#include <glm/glm.hpp>

#include <array>
#include <functional>
#include <memory>
#include <string>
#include <vector>

namespace Diamond {

class RHIDevice;
class RHICommandList;
class RHIBuffer;
class RHIShader;
class RHIPipeline;
class RHIResourceSet;

// Point-light shadow maps — the Vulkan port of OpenGLShadowPass. One distance
// cubemap per point light (depth stores length(frag - light) / farPlane, written
// by point_shadow.frag), sampled by world-space direction in the deferred resolve
// with GL's 20-tap disk PCF.
//
// Like the IBL bake, this pass renders into individual cube faces, which the
// render graph can't express — so it records RAW Vulkan (per-face dynamic-
// rendering scopes + its own barriers) into the frame's command buffer via
// Record(), called between RHIDevice::BeginFrame and graph.Execute. The recorded
// work therefore always lands before the graph's deferred-lighting read.
//
// The cubes aren't RHITextures (those are 2D), so, exactly like render targets,
// the pass keeps one cube per light PER FRAME-IN-FLIGHT (two concurrent frames
// must never write the same image) and exposes per-slot views for the lighting
// pass to bind raw (the BindIBL pattern). Faces are rendered with the IBL bake's
// view matrices + the backend's negative-height viewport, so world-direction
// sampling matches GL. All cubes are cleared to depth 1 (no occluder) at
// creation, so slots without a live light are still valid to sample.
class VulkanPointShadowPass {
public:
    static constexpr int      MAX_LIGHTS  = 4;
    static constexpr uint32_t kResolution = 512;     // GL parity (SetupPointShadowMaps)

    VulkanPointShadowPass(RHIDevice* device, const std::string& shaderDir);
    ~VulkanPointShadowPass();

    VulkanPointShadowPass(const VulkanPointShadowPass&)            = delete;
    VulkanPointShadowPass& operator=(const VulkanPointShadowPass&) = delete;

    // Upload this frame's light positions (world space, at most MAX_LIGHTS) into
    // the face-matrix UBO. Call after RHIDevice::BeginFrame, before Record.
    void SetLights(const std::vector<glm::vec3>& positionsWorld);

    // Record the shadow renders for every set light: per light, 6 per-face depth
    // scopes + barriers, ending with the cube sampleable by the fragment stage.
    // drawScene records the casters; it must push each draw's model matrix at
    // push-constant offset 0 (the pass pushes the face index at offset 64).
    void Record(RHICommandList* cmd,
                const std::function<void(RHICommandList*)>& drawScene);

    // Raw handles for the deferred-lighting descriptor set (per frame slot, since
    // the cubes are per-frame-in-flight). Valid from construction.
    VkImageView CubeView(uint32_t frame, int light) const { return m_Cubes[frame][light].cubeView; }
    VkSampler   Sampler() const { return m_Sampler; }
    float       FarPlane() const { return kFarPlane; }

private:
    static constexpr float kFarPlane = 25.0f;   // GL parity

    // std140 layout matching point_shadow.vert's PointShadowUBO.
    struct PointShadowUBO {
        glm::mat4 faceVP[MAX_LIGHTS * 6];   // [light * 6 + face] — light clip from world
        glm::vec4 lightPosFar[MAX_LIGHTS];  // .xyz world position, .w farPlane
    };

    // A D32 cubemap: the cube view samples it, the six face views render into it.
    struct Cube {
        VkImage       image    = VK_NULL_HANDLE;
        VmaAllocation alloc    = VK_NULL_HANDLE;
        VkImageView   cubeView = VK_NULL_HANDLE;
        std::array<VkImageView, 6> faceViews{};
    };

    VulkanRHIDevice* m_Device;
    int              m_Count = 0;

    std::unique_ptr<RHIShader>      m_Vert;
    std::unique_ptr<RHIShader>      m_Frag;
    std::unique_ptr<RHIPipeline>    m_Pipeline;
    std::unique_ptr<RHIBuffer>      m_UBO;
    std::unique_ptr<RHIResourceSet> m_Set;

    VkSampler m_Sampler = VK_NULL_HANDLE;
    std::array<std::array<Cube, MAX_LIGHTS>, VulkanRHIDevice::kFramesInFlight> m_Cubes{};
};

} // namespace Diamond
