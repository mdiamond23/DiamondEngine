#pragma once

#include "Renderer/RHI/RHIRenderGraph.h"

#include <array>
#include <memory>
#include <string>

namespace Diamond {

class RHIDevice;
class RHITexture;
class RHIShader;
class RHIPipeline;
class RHIResourceSet;
class RHICommandList;

// Auto-exposure (eye adaptation). Measures the scene's average log-luminance,
// smooths it over time, and publishes an exposure multiplier as a 1x1 texture
// that the bloom bright-pass and the tonemap both sample.
//
// Three stages, all raster — the RHI has no compute:
//   1. Extract   — HDR scene -> a fixed 256x256 square of log2 luminance.
//   2. Reduce x4 — 4x box reduction per step (256 -> 64 -> 16 -> 4 -> 1). Each
//                  output texel is four bilinear taps over its 4x4 source block.
//   3. Adapt     — 1x1: blend toward the retained value, emit the multiplier.
// The chain's size is FIXED, independent of view resolution: the extract samples
// with normalized UVs, so a resize changes nothing about stages 2-3 and the
// reduction cost is constant. Every stage writes its own target, never reusing
// one — the graph points readers at a texture's LAST writer, so a ping-pong
// would be a dependency cycle (see VulkanBloomPass for the same constraint).
//
// The retained value follows the TAA history pattern exactly: a pass-owned
// SINGLE-BUFFERED 1x1 image, sampled raw (not a declared graph read) because a
// graph pass reading its own write target is a feedback loop. The adapt pass
// MRT-writes the new value to a graph target, and RecordAdaptedCopy — recorded
// raw AFTER graph.Execute — copies it back into the retained image.
//
// Disabled, the chain still runs but the adapt stage emits 1.0, so the manual
// exposure the tonemap already applies is left in sole charge. That keeps the
// descriptor layouts identical either way, with the branch in one shader.
class VulkanAutoExposurePass {
public:
    VulkanAutoExposurePass(RHIDevice* device, const std::string& shaderDir);
    ~VulkanAutoExposurePass();

    // Wire the chain in: reads 'hdrScene', writes the 1x1 'exposureOut' that
    // downstream passes sample. Declares its own intermediates.
    void AddToGraph(RHIRenderGraph& graph, RGTextureHandle hdrScene,
                    RGTextureHandle exposureOut);

    // Record once per frame BEFORE graph.Execute (no-op after the first): puts
    // the never-written retained image into a sampleable layout, so the adapt
    // pass's descriptor set is valid even on the frame that ignores it.
    void Prepare(RHICommandList* cmd);

    // Record AFTER graph.Execute, outside any render scope: copies this frame's
    // adapted log-luminance into the retained image and marks it valid.
    void RecordAdaptedCopy(RHICommandList* cmd);

    // Forget the adaptation — the next frame snaps to the measured luminance
    // rather than easing from a stale value.
    void InvalidateHistory() { m_HistoryValid = false; }

    // Off = the pass emits an exposure of 1.0 (manual exposure only).
    void SetEnabled(bool enabled) { m_Enabled = enabled; }

    // Luminance the adapted average is mapped to — effectively "middle grey".
    // Higher = brighter overall image.
    void SetKeyValue(float key) { m_KeyValue = key; }

    // Metering range in log2 luminance (stops). The measured average is clamped
    // here before adaptation, which bounds how far exposure can swing.
    void SetRange(float minLogLuma, float maxLogLuma) {
        m_MinLogLuma = minLogLuma;
        m_MaxLogLuma = maxLogLuma;
    }

    // Adaptation rate in stops per second (higher = faster eye adjustment).
    void SetSpeed(float speed) { m_Speed = speed; }

    // Per-frame delta time, used to make adaptation frame-rate independent.
    void SetDeltaTime(float dt) { m_DeltaTime = dt; }

private:
    // Reduction chain sizes. 256 down to 1x1 by 4x steps — kReduceSteps of them.
    static constexpr uint32_t kExtractSize = 256;
    static constexpr int      kReduceSteps = 4;   // 256 -> 64 -> 16 -> 4 -> 1

    // Mirrors exposure_adapt.frag's push_constant block.
    struct AdaptPush {
        float alpha;
        float keyValue;
        float minLogLuma;
        float maxLogLuma;
        int   autoEnabled;
    };

    RHIDevice* m_Device;

    bool  m_Enabled      = false;   // opt-in; manual exposure is the default
    bool  m_HistoryValid = false;   // alpha=1 snap until seeded
    bool  m_Prepared     = false;   // retained-image layout initialized
    float m_KeyValue     = 0.18f;   // middle grey
    float m_MinLogLuma   = -8.0f;
    float m_MaxLogLuma   =  4.0f;
    float m_Speed        =  2.0f;   // stops/second
    float m_DeltaTime    =  1.0f / 60.0f;

    std::unique_ptr<RHITexture> m_Adapted;      // single-buffered 1x1 R16F
    RHITexture*                 m_AdaptedSrc = nullptr;   // graph-owned copy source

    std::unique_ptr<RHIShader> m_FullscreenVert;
    std::unique_ptr<RHIShader> m_ExtractFrag;
    std::unique_ptr<RHIShader> m_ReduceFrag;
    std::unique_ptr<RHIShader> m_AdaptFrag;
    std::unique_ptr<RHIShader> m_CopyFrag;

    std::unique_ptr<RHIPipeline> m_ExtractPipeline;
    std::unique_ptr<RHIPipeline> m_ReducePipeline;   // shared by every step
    std::unique_ptr<RHIPipeline> m_AdaptPipeline;
    std::unique_ptr<RHIPipeline> m_CopyPipeline;

    std::unique_ptr<RHIResourceSet>                            m_ExtractSet;
    std::array<std::unique_ptr<RHIResourceSet>, kReduceSteps>  m_ReduceSets;
    std::unique_ptr<RHIResourceSet>                            m_AdaptSet;
    std::unique_ptr<RHIResourceSet>                            m_CopySet;
};

} // namespace Diamond
