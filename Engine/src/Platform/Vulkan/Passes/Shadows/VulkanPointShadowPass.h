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
    // A slot whose position changed (light moved / a new light took the slot) has
    // its static cache marked dirty here, so a moved light re-bakes its cube.
    void SetLights(const std::vector<glm::vec3>& positionsWorld);

    // Invalidate every active slot's static cache — call when the set of static
    // shadow casters (or any static caster's transform) changed this frame, since
    // the cache can't detect that itself. Re-baked on the next Record.
    void MarkStaticDirty();

    // Record the shadow renders for every set light: per light, 6 per-face depth
    // scopes + barriers, ending with the cube sampleable by the fragment stage.
    // The callbacks record the casters; each must push its draw's model matrix at
    // push-constant offset 0 (the pass pushes the face index at offset 64). The
    // face index is also handed to the callback so a skinned pass that rebinds the
    // skinned pipeline can re-push it (offset 64) after the pipeline switch.
    //
    // Static shadow caching: each slot keeps a cached static-caster cube, re-baked
    // only when dirty (drawStatic). Every frame the working cube is seeded from
    // that cache — vkCmdCopyImage, all 6 faces at once (this pass is raw Vulkan,
    // so the transfer copy the RHI lacks is available; the spot pass needs a
    // fullscreen depth-copy pipeline instead) — and only this frame's DYNAMIC
    // casters are drawn over it (drawDynamic). Unlike the spot pass's RHI render
    // targets, the static cubes are single images (not per-frame-in-flight): they
    // are written only when dirty and read by transfer, with barriers ordering
    // both against prior frames, so one bake fills the cache.
    void Record(RHICommandList* cmd,
                const std::function<void(RHICommandList*, uint32_t faceIdx)>& drawStatic,
                const std::function<void(RHICommandList*, uint32_t faceIdx)>& drawDynamic);

    // The skinned distance-cube pipeline (point_shadow_skinned: set 0 = the shared
    // face-matrix UBO, set 1 = bone palette, wider vertex layout) and the shared
    // face-matrix set, so a skinned draw can rebind both under the new pipeline.
    RHIPipeline*    SkinnedPipeline() const { return m_SkinnedPipeline.get(); }
    RHIResourceSet* FaceSet()         const { return m_Set.get(); }

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
    // Static-cache cubes skip the cube view (they are only a transfer source).
    struct Cube {
        VkImage       image    = VK_NULL_HANDLE;
        VmaAllocation alloc    = VK_NULL_HANDLE;
        VkImageView   cubeView = VK_NULL_HANDLE;
        std::array<VkImageView, 6> faceViews{};
    };

    // Render 6 per-face depth scopes into 'cube' (the shared raw-record shape:
    // negative-height viewport, pipeline + face-matrix set bound per face, face
    // index pushed at offset 64). clear=false loads the copied-in static depth.
    void RenderFaces(RHICommandList* cmd, VkCommandBuffer raw, const Cube& cube,
                     int light, bool clear,
                     const std::function<void(RHICommandList*, uint32_t)>& draw);

    VulkanRHIDevice* m_Device;
    int              m_Count = 0;

    std::unique_ptr<RHIShader>      m_Vert;
    std::unique_ptr<RHIShader>      m_Frag;
    std::unique_ptr<RHIShader>      m_SkinnedVert;
    std::unique_ptr<RHIPipeline>    m_Pipeline;
    std::unique_ptr<RHIPipeline>    m_SkinnedPipeline;
    std::unique_ptr<RHIBuffer>      m_UBO;
    std::unique_ptr<RHIResourceSet> m_Set;

    VkSampler m_Sampler = VK_NULL_HANDLE;
    std::array<std::array<Cube, MAX_LIGHTS>, VulkanRHIDevice::kFramesInFlight> m_Cubes{};

    // Static shadow caching: per-slot cached static-caster cube (copy source for
    // the working cubes) + dirty flag (bake next Record) + previous positions for
    // moved-light detection in SetLights. Single-buffered by design — see Record.
    std::array<Cube, MAX_LIGHTS>      m_StaticCubes{};
    std::array<bool, MAX_LIGHTS>      m_StaticDirty{};
    std::array<glm::vec3, MAX_LIGHTS> m_PrevPos{};
    int                               m_PrevCount = 0;
};

} // namespace Diamond
