#pragma once
#include <cstdint>
#include <string>
#include <vector>

namespace Diamond
{
    // One profiled render pass (Docs/profiler-panel-design.md, per-pass phase).
    // Vulkan-only: the GL backend leaves RendererStats::passes empty. GPU times
    // are read back frames-in-flight late (like gpuFrameMs), so a pass row is a
    // couple of frames older than the top-level counters — fine for a profiler.
    struct PassStats
    {
        std::string scope;          // group: "Main View", "Game View", "Shadows"
        std::string name;           // pass name: "GBuffer", "SSAO", ...
        float    gpuMs     = 0.0f;  // EMA-smoothed timestamp span
        float    cpuMs     = 0.0f;  // EMA-smoothed command-recording time
        uint32_t drawCalls = 0;
        uint64_t triangles = 0;
        uint32_t width     = 0;     // render-target size; 0 = backbuffer/unknown
        uint32_t height    = 0;
    };

    struct RendererStats
    {
    float fps            = 0.0f;   // smoothed (EMA over ~0.5 s)
    float cpuFrameMs     = 0.0f;   // full main-loop iteration, measured by the app
    float gpuFrameMs     = 0.0f;   // GPU timestamp span (see Timing below)

    // Submission
    uint32_t drawCalls        = 0; // vkCmdDraw*/glDrawElements* issued
    uint32_t dispatchCalls    = 0; // compute dispatches (0 on GL today)
    uint64_t trianglesSubmitted = 0; // sum of indexCount/3 × instances

    // Scene / culling
    uint32_t visibleObjects   = 0; // draws surviving frustum culling (main view)
    uint32_t culledObjects    = 0; // draws rejected by frustum culling
    uint32_t shadowCasters    = 0; // draws submitted across all shadow passes

    // Bindings / resources
    uint32_t materialsBound   = 0; // unique materials referenced this frame
    uint32_t texturesUsed     = 0; // unique textures referenced this frame
    uint32_t descriptorWrites = 0; // vkUpdateDescriptorSets writes (0 on GL)
    uint32_t bufferUploads    = 0; // map/memcpy or staging copies this frame

    // Memory
    uint64_t vramBytes        = 0; // estimate — see VRAM below

    // Per-pass breakdown (Vulkan only; empty on GL). In execution order.
    std::vector<PassStats> passes;
    };
}
