#include "Profiling/GLRendererStats.h"
#include <unordered_set>

namespace Diamond::GLStats {

namespace {
    RendererStats g_Working;
    RendererStats g_Snapshot;
    std::unordered_set<const void*> g_MaterialsSeen;
    std::unordered_set<const void*> g_TexturesSeen;
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

void Finalize() {
    g_Working.materialsBound = (uint32_t)g_MaterialsSeen.size();
    g_Working.texturesUsed   = (uint32_t)g_TexturesSeen.size();
    g_Snapshot = g_Working;
}

const RendererStats& GetStats() { return g_Snapshot; }

} // namespace Diamond::GLStats
