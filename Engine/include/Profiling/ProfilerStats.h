#pragma once
#include <cstdint>

namespace Diamond 
{
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
    };
}