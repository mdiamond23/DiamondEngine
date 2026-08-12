#pragma once

#include "Renderer/RHI/RHIRenderGraph.h"

#include <glm/glm.hpp>

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

// Ray-tracing bring-up pass (Docs/gi-design.md slice 2). Traces one primary ray
// per pixel against the scene TLAS and blits the hit distance over the finished
// image. Deliberately throwaway: it proves the whole RT stack — extensions,
// device addresses, BLAS/TLAS, pipeline, SBT, barriers — end to end before DDGI
// is built on it.
//
// Two graph passes, mirroring how SSGI splits its compute trace from its raster
// resolve. The blit LOADS its target and skips its draw when disabled, so a
// disabled pass costs nothing but two layout transitions.
//
// Inert on a device without ray tracing: the constructor builds nothing and
// AddToGraph registers nothing.
class VulkanRTDebugPass {
public:
    // 'targetFormat' is the format of the image the blit writes — the offscreen
    // LDR texture, or the swapchain format in swapchain mode. Dynamic rendering
    // rejects a pipeline whose color format doesn't match its attachment.
    VulkanRTDebugPass(RHIDevice* device, const std::string& shaderDir,
                      uint32_t width, uint32_t height, RHIFormat targetFormat);
    ~VulkanRTDebugPass();

    bool Available() const { return m_Pipeline != nullptr; }

    // 'target' is the tonemapped LDR image (or the swapchain when toSwapchain).
    // 'rtDebug' must be declared at full res with storage = true. Insert after
    // tonemap so the visualization replaces the finished frame rather than
    // feeding into it.
    void AddToGraph(RHIRenderGraph& graph, RGTextureHandle rtDebug,
                    RGTextureHandle target, bool toSwapchain);

    // The scene TLAS. Passing a different pointer than last time rebuilds the
    // trace descriptor set, so a TLAS recreated (rather than rebuilt in place)
    // is picked up. Null disables the pass for the frame.
    void SetTLAS(RHIAccelStruct* tlas);

    // Camera inverses the raygen unprojects through. After BeginFrame, before
    // Execute — same contract as the other passes' SetProjection.
    void SetCamera(const glm::mat4& view, const glm::mat4& projection);

    void SetEnabled(bool enabled) { m_Enabled = enabled; }
    bool IsEnabled() const { return m_Enabled; }

    void  SetMaxDistance(float distance);
    float MaxDistance() const { return m_MaxDistance; }

private:
    // std140, matching rt_debug.rgen's Camera block.
    struct CameraUBO {
        glm::mat4 invView;
        glm::mat4 invProj;
    };

    struct TracePush {
        float maxDistance;
    };

    RHIDevice* m_Device;
    uint32_t   m_Width;
    uint32_t   m_Height;

    bool  m_Enabled     = false;   // off by default — a bring-up tool, not a feature
    float m_MaxDistance = 100.0f;

    RHIAccelStruct* m_TLAS    = nullptr;   // not owned (SceneRenderer keeps it)
    bool            m_SetDirty = true;

    std::unique_ptr<RHIShader>   m_Raygen;
    std::unique_ptr<RHIShader>   m_Miss;
    std::unique_ptr<RHIShader>   m_ClosestHit;
    std::unique_ptr<RHIShader>   m_FullscreenVert;
    std::unique_ptr<RHIShader>   m_CopyFrag;

    std::unique_ptr<RHIPipeline> m_Pipeline;       // ray tracing
    std::unique_ptr<RHIPipeline> m_BlitPipeline;   // raster

    std::unique_ptr<RHIBuffer>      m_CameraUBO;
    std::unique_ptr<RHIResourceSet> m_TraceSet;
    std::unique_ptr<RHIResourceSet> m_BlitSet;

    // Held so the trace set can be rebuilt when the TLAS pointer changes.
    RHITexture* m_RTDebugTexture = nullptr;
};

} // namespace Diamond
