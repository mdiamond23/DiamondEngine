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
// Follows the ported-pass template with one extra wrinkle: the history. It is a
// pass-owned SINGLE-BUFFERED render target (RHITextureDesc::singleBuffered) —
// the per-frame-in-flight duplication normal render targets get would make the
// resolve sample 2-frame-old history against a 1-frame velocity. It can't be a
// graph texture either: the resolve both samples history and produces next
// frame's history, and a graph pass reading its own write target is a feedback
// loop. So the resolve samples m_History raw (the lighting pass's point-shadow
// cube precedent) and MRT-writes its result twice: outColor for the downstream
// chain (transparency/particles Load-blend into it in place, then tonemap) and
// historySrc, which nothing else touches. RecordHistoryCopy — recorded raw
// AFTER graph.Execute, outside any render scope — fullscreen-copies historySrc
// into m_History, so translucents/particles (no motion vectors) never enter
// the accumulation.
//
// Toggling: the pass always runs (the graph is compiled once), but with an
// invalid history it pushes alpha = 1.0 and the shader is a pure passthrough.
// The owner skips RecordHistoryCopy and calls InvalidateHistory while TAA is
// off, so re-enabling self-seeds: passthrough frame -> copy -> blending.
class VulkanTAAPass {
public:
    // 'width'/'height' is the view's offscreen resolution — sizes the history.
    VulkanTAAPass(RHIDevice* device, const std::string& shaderDir,
                  uint32_t width, uint32_t height);
    ~VulkanTAAPass();

    // Register the resolve: reads sceneColor (post-SSR HDR) + velocity
    // (G-buffer motion vectors), writes outColor + historySrc (both
    // caller-declared RGBA16F, same resolved image). Downstream passes consume
    // outColor; historySrc must have NO other writer or reader — it exists so
    // the post-graph history copy sees the resolve output, not the
    // transparency/particles composited over outColor.
    void AddToGraph(RHIRenderGraph& graph,
                    RGTextureHandle sceneColor, RGTextureHandle velocity,
                    RGTextureHandle outColor, RGTextureHandle historySrc);

    // Record once per frame BEFORE graph.Execute (first frame only, no-op
    // after): moves the never-written history into a sampleable layout so the
    // resolve's descriptor set is valid even while alpha = 1.0 ignores it.
    void Prepare(RHICommandList* cmd);

    // Record AFTER graph.Execute, outside any render scope: fullscreen-copies
    // the resolved outColor into the history and marks it valid. Skip this
    // (and call InvalidateHistory) while TAA is disabled.
    void RecordHistoryCopy(RHICommandList* cmd);

    // Forget the accumulated history — the next resolve passes the current
    // frame through (alpha 1.0) and the next RecordHistoryCopy re-seeds.
    void InvalidateHistory() { m_HistoryValid = false; }

    // Steady-state blend toward the current frame (default 0.1 — a frame's
    // contribution halves roughly every 7 frames).
    void SetBlendFactor(float alpha) { m_Alpha = alpha; }

private:
    RHIDevice*                      m_Device;
    float                           m_Alpha        = 0.1f;
    bool                            m_HistoryValid = false;  // alpha=1 passthrough until seeded
    bool                            m_Prepared     = false;  // history layout initialized

    std::unique_ptr<RHITexture>     m_History;    // single-buffered RGBA16F accumulation
    RHITexture*                     m_HistorySrc = nullptr;   // graph-owned; copy source

    std::unique_ptr<RHIShader>      m_FullscreenVert;
    std::unique_ptr<RHIShader>      m_ResolveFrag;
    std::unique_ptr<RHIShader>      m_CopyFrag;
    std::unique_ptr<RHIPipeline>    m_ResolvePipeline;
    std::unique_ptr<RHIPipeline>    m_CopyPipeline;   // outColor -> history
    std::unique_ptr<RHIResourceSet> m_ResolveSet;
    std::unique_ptr<RHIResourceSet> m_CopySet;
};

} // namespace Diamond
