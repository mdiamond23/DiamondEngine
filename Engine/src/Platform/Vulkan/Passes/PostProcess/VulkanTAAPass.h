#pragma once

#include "Renderer/RHI/RHIRenderGraph.h"

#include <memory>
#include <string>

namespace Diamond {

class RHIDevice;
class RHITexture;
class RHIShader;
class RHIPipeline;
class RHIResourceSet;
class RHICommandList;

// Temporal anti-aliasing resolve. Consumes the jittered, SSR-composited HDR
// scene + the G-buffer's motion vectors and blends against last frame's
// resolved image (reprojected through the velocity, then clamped to the current
// 3x3 neighborhood's color AABB so stale history can't ghost). Insert after the
// SSR composite and before transparency/particles — those have no motion
// vectors, so they must draw over the resolved image, never into the history.
//
// Two graph passes. The resolve MRT-writes its result twice: outColor for the
// downstream chain (transparency/particles Load-blend into it in place, then
// tonemap) and historySrc, which nothing else touches. The copy then moves
// historySrc into the history, so translucents and particles — which have no
// motion vectors — never enter the accumulation.
//
// The history is a caller-declared PERSISTENT graph texture: one image rather
// than one per frame-in-flight, so what frame N wrote is exactly what frame N+1
// samples (per-frame-in-flight duplication would resolve 2-frame-old history
// against 1-frame velocity). The resolve consumes it with ReadHistory rather
// than Read — it wants last frame's contents, so it must NOT depend on this
// frame's copy pass, which would be a cycle.
//
// Toggling: both passes always run (the graph is compiled once), but while TAA
// is off the resolve pushes alpha = 1.0 (a pure passthrough) and the copy skips
// its draw and drops the history-valid flag. Re-enabling therefore self-seeds:
// passthrough frame -> copy -> blending.
class VulkanTAAPass {
public:
    VulkanTAAPass(RHIDevice* device, const std::string& shaderDir);
    ~VulkanTAAPass();

    // Register both passes. The resolve reads sceneColor (post-SSR HDR) +
    // velocity (G-buffer motion vectors), history-reads 'history', and writes
    // outColor + historySrc (both caller-declared RGBA16F, the same resolved
    // image). The copy reads historySrc and writes 'history'. Downstream passes
    // consume outColor; historySrc must have no other writer, and 'history' must
    // be declared with RGTextureDesc::persistent.
    void AddToGraph(RHIRenderGraph& graph,
                    RGTextureHandle sceneColor, RGTextureHandle velocity,
                    RGTextureHandle outColor, RGTextureHandle historySrc,
                    RGTextureHandle history);

    // Gates the accumulation for the coming frame — call before graph.Execute.
    // While false the resolve is a passthrough and the history stops updating.
    void SetEnabled(bool enabled) { m_Enabled = enabled; }

    // Forget the accumulated history — the next resolve passes the current
    // frame through (alpha 1.0) and the next copy re-seeds. For discontinuities
    // the velocity buffer can't express, like a camera teleport.
    void InvalidateHistory() { m_HistoryValid = false; }

    // Steady-state blend toward the current frame (default 0.1 — a frame's
    // contribution halves roughly every 7 frames).
    void SetBlendFactor(float alpha) { m_Alpha = alpha; }

private:
    RHIDevice*                      m_Device;
    float                           m_Alpha        = 0.1f;
    bool                            m_Enabled      = true;
    bool                            m_HistoryValid = false;  // alpha=1 passthrough until seeded

    std::unique_ptr<RHIShader>      m_FullscreenVert;
    std::unique_ptr<RHIShader>      m_ResolveFrag;
    std::unique_ptr<RHIShader>      m_CopyFrag;
    std::unique_ptr<RHIPipeline>    m_ResolvePipeline;
    std::unique_ptr<RHIPipeline>    m_CopyPipeline;   // historySrc -> history
    std::unique_ptr<RHIResourceSet> m_ResolveSet;
    std::unique_ptr<RHIResourceSet> m_CopySet;
};

} // namespace Diamond
