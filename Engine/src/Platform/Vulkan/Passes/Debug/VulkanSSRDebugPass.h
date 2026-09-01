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

// SSR march diagnostic (ssr_debug.frag). ONE raster pass that re-runs the same
// ray ssr_resolve.frag's mirror path runs and false-colours what the march DID
// — hit or miss, where it landed, which step accepted it, how far past the
// surface it was, and how much of the tolerance came from surface slope.
//
// It draws over the finished, tonemapped frame, not into the SSR chain. A
// diagnostic fed through the composite, the RT merge, TAA and tonemap would be
// blended, reprojected and curve-remapped before anyone saw it, and the values
// on screen would no longer be the values the march computed.
//
// Cheaper than the pass it diagnoses: one full-res trace, no resolve, no
// low-res pass. Off by default and skips its draw entirely when off, so the
// frame underneath survives untouched — same Load()-and-skip shape as
// VulkanRTDebugPass.
class VulkanSSRDebugPass {
public:
    // Keep in step with ssr_debug.frag's MODE_* constants and with the editor's
    // combo labels.
    enum class Mode : int {
        Off         = 0,
        HitMiss     = 1,   // green = crossing found, red = none
        HitDistance = 2,   // screen distance origin -> hit; ~0 means self-hit
        Steps       = 3,   // which step accepted the crossing
        Overshoot   = 4,   // depth past the surface, relative to the tolerance
        Slope       = 5,   // the slope-scaled part of the tolerance
    };

    // 'targetFormat' is the format of the image drawn over — the offscreen LDR
    // texture, or the swapchain format in swapchain mode. Dynamic rendering
    // rejects a pipeline whose color format disagrees with its attachment.
    VulkanSSRDebugPass(RHIDevice* device, const std::string& shaderDir,
                       uint32_t width, uint32_t height, RHIFormat targetFormat);
    ~VulkanSSRDebugPass();

    bool Available() const { return m_Pipeline != nullptr; }

    // 'target' is the tonemapped LDR image (or the swapchain when toSwapchain).
    // Insert AFTER tonemap. The G-buffer/pyramid handles are the same ones
    // VulkanSSRPass reads, so the march sees identical inputs.
    void AddToGraph(RHIRenderGraph& graph,
                    RGTextureHandle viewPos, RGTextureHandle viewNormal,
                    RGTextureHandle gMaterial, RGTextureHandle linearDepth,
                    RGTextureHandle coarseDepth,
                    RGTextureHandle target, bool toSwapchain);

    void SetMode(Mode mode) { m_Mode = mode; }
    Mode GetMode() const    { return m_Mode; }
    bool IsEnabled() const  { return m_Mode != Mode::Off; }

    // Current frame's projection — the march projects view-space positions to
    // screen UVs with it. After BeginFrame, before Execute, same contract as
    // VulkanSSRPass::SetProjection.
    void SetProjection(const glm::mat4& projection);

private:
    // std140, matching ssr_debug.frag's SSRDebugUBO.
    struct DebugUBO {
        glm::mat4 projection;
        glm::vec2 fullSize;
        int       mode         = 0;
        float     roughnessMax = 0.9f;   // ssr_resolve.frag's ROUGHNESS_CUTOFF
    };

    RHIDevice* m_Device;
    DebugUBO   m_UBOData{};
    Mode       m_Mode = Mode::Off;

    std::unique_ptr<RHIShader>      m_FullscreenVert;
    std::unique_ptr<RHIShader>      m_Frag;
    std::unique_ptr<RHIPipeline>    m_Pipeline;
    std::unique_ptr<RHIBuffer>      m_UBO;
    std::unique_ptr<RHIResourceSet> m_Set;
};

} // namespace Diamond
