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
void Finalize();                                 // working -> snapshot
const RendererStats& GetStats();                 // last completed frame

} // namespace GLStats
} // namespace Diamond
