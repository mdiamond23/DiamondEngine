#pragma once

#include "Renderer/RHI/RHIRenderGraph.h"
#include "DebugDraw.h"

#include <glm/glm.hpp>

#include <memory>
#include <string>

namespace Diamond {

class RHIDevice;
class RHIBuffer;
class RHIShader;
class RHIPipeline;

// Vulkan counterpart of DebugDraw's GL Flush() — the deferred chain's collider/
// ragdoll/IK/audio wireframes (colliders, ragdolls, IK chains, audio gizmos all
// still just call DebugDraw::Line/Box/Sphere/Capsule; only the upload+render
// step differs per backend). One dynamic vertex buffer sized to a fixed cap,
// rewritten every frame; a line-list pipeline with depthWrite off (tested, not
// written, like GL's glDepthMask(GL_FALSE) during Flush) so wireframes draw on
// top of real geometry without corrupting the depth buffer for later passes.
// No resource set — the view-proj rides in a push constant, so AddToGraph needs
// no draw callback.
class VulkanDebugDrawPass {
public:
    // 'colorFormat' is the format of the LDR target this composites into
    // (RGBA8 offscreen, or the swapchain format when rendering direct).
    VulkanDebugDrawPass(RHIDevice* device, const std::string& shaderDir, RHIFormat colorFormat);
    ~VulkanDebugDrawPass();

    // Register the pass: LOAD + write 'color' (or the swapchain) over the
    // tonemapped scene, LOAD + write 'depth' read-only (depthWrite = false) so
    // wireframes occlude correctly against real geometry.
    void AddToGraph(RHIRenderGraph& graph, RGTextureHandle color, bool toSwapchain,
                    RGTextureHandle depth);

    // Snapshot this frame's DebugDraw accumulator (clearing it, mirroring GL's
    // Flush) and upload it to the dynamic vertex buffer. Call after
    // Physics::DrawColliders/DrawRagdolls/DrawIKDebug/DrawAudioDebug have run
    // for the frame, before the view's graph.Execute.
    void SetFrameData(const glm::mat4& viewProj);

private:
    static constexpr uint32_t kMaxVertices = 65536;

    RHIDevice*  m_Device;

    std::unique_ptr<RHIShader>   m_Vert;
    std::unique_ptr<RHIShader>   m_Frag;
    std::unique_ptr<RHIPipeline> m_Pipeline;
    std::unique_ptr<RHIBuffer>   m_VertexBuffer;

    glm::mat4 m_ViewProj {1.0f};
    uint32_t  m_VertexCount = 0;
};

} // namespace Diamond
