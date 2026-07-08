#include "Profiling/GLRendererStats.h"
#include <glad/gl.h>
#include <unordered_set>

namespace Diamond::GLStats {

namespace {
    RendererStats g_Working;
    RendererStats g_Snapshot;
    std::unordered_set<const void*> g_MaterialsSeen;
    std::unordered_set<const void*> g_TexturesSeen;

    // VRAM estimate: running tally of live GPU allocations. Persists across
    // Reset()/Finalize() — it's not a per-frame stat, see header comment.
    uint64_t g_VramBytes = 0;
}

void Reset() {
    g_Working = RendererStats{};
    g_MaterialsSeen.clear();
    g_TexturesSeen.clear();
}

void RecordDraw(uint64_t triangleCount) {
    g_Working.drawCalls++;
    g_Working.trianglesSubmitted += triangleCount;
}

void RecordBufferUpload() { g_Working.bufferUploads++; }
void RecordVisible(uint32_t count) { g_Working.visibleObjects += count; }
void RecordCulled(uint32_t count)  { g_Working.culledObjects  += count; }
void RecordShadowCaster() { g_Working.shadowCasters++; }
void RecordMaterialBound(const void* material) { g_MaterialsSeen.insert(material); }
void RecordTextureUsed(const void* texture)    { g_TexturesSeen.insert(texture); }
void SetGpuFrameMs(float ms) { g_Working.gpuFrameMs = ms; }

void RecordTextureAlloc(uint64_t bytes) { g_VramBytes += bytes; }
void RecordTextureFree(uint64_t bytes)  { g_VramBytes -= bytes; }
void RecordBufferAlloc(uint64_t bytes)  { g_VramBytes += bytes; }
void RecordBufferFree(uint64_t bytes)   { g_VramBytes -= bytes; }

uint32_t BytesPerTexel(uint32_t glInternalFormat) {
    switch (glInternalFormat) {
        case GL_R8:               return 1;
        case GL_R16F:             return 2;
        case GL_R32F:              return 4;
        case GL_RG8:              return 2;
        case GL_RG16F:            return 4;
        case GL_RGB8:             return 3;
        case GL_RGB16F:           return 6;
        case GL_RGBA8:            return 4;
        case GL_RGBA16F:          return 8;
        case GL_RGBA32F:          return 16;
        case GL_DEPTH_COMPONENT:
        case GL_DEPTH_COMPONENT24:
        case GL_DEPTH_COMPONENT32F: return 4;
        default:                  return 4;
    }
}

void Finalize() {
    g_Working.materialsBound = (uint32_t)g_MaterialsSeen.size();
    g_Working.texturesUsed   = (uint32_t)g_TexturesSeen.size();
    g_Working.vramBytes      = g_VramBytes;
    g_Snapshot = g_Working;
}

const RendererStats& GetStats() { return g_Snapshot; }

} // namespace Diamond::GLStats
