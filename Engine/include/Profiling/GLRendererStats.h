#pragma once
#include "Profiling/ProfilerStats.h"
#include <cstdint>

namespace Diamond {

// Frame-stats accumulator for the OpenGL backend (Docs/profiler-panel-design.md,
// Phase 1: CPU-side counters only). GL has no engine-side "device" object the
// way Vulkan's RHIDevice is — GLEditorBackend (in Sandbox) owns the render
// graph directly, while the draw/upload choke points (OpenGLMesh, the GL
// passes) live in Engine, which can't depend on Sandbox. This static facade —
// same shape as the Audio:: facade — is the shared home both sides reach.
//
// One GL context per process today, so process-wide state is fine; not
// thread-safe (GL calls aren't either).
namespace GLStats {

void Reset();                                  // clears the working accumulator
void RecordDraw(uint64_t triangleCount);
void RecordBufferUpload();
void RecordVisible(uint32_t count);
void RecordCulled(uint32_t count);
void RecordShadowCaster();
void RecordMaterialBound(const void* material); // deduped internally, per frame
void RecordTextureUsed(const void* texture);     // deduped internally, per frame
void SetGpuFrameMs(float ms);                    // Phase 2: GL_TIME_ELAPSED readback

// VRAM estimate: GL has no portable "how much VRAM am I using" query, so these
// maintain a running tally of live GPU allocations, updated at resource
// create/destroy time. Unlike the counters above, this tally is NOT cleared by
// Reset() — it reflects live allocations, not per-frame activity.
void RecordTextureAlloc(uint64_t bytes);
void RecordTextureFree(uint64_t bytes);
void RecordBufferAlloc(uint64_t bytes);
void RecordBufferFree(uint64_t bytes);

// Bytes-per-texel for the GL internal formats this codebase actually creates
// (mirrors RenderGraph.cpp's deriveFormatAndType). Defaults to 4 for anything
// unrecognized. Shared so every VRAM-tally call site uses the same table.
uint32_t BytesPerTexel(uint32_t glInternalFormat);

void Finalize();                                 // working -> snapshot
const RendererStats& GetStats();                 // last completed frame

} // namespace GLStats
} // namespace Diamond
