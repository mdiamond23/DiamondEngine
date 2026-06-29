#pragma once

#include "Renderer/RHI/RHIEnums.h"

#include <array>
#include <functional>
#include <memory>
#include <string>
#include <string_view>
#include <unordered_map>
#include <vector>

namespace Diamond {

class RHIDevice;
class RHICommandList;
class RHITexture;

// Handle into the graph's texture table. id is 1-based so 0 stays the "invalid"
// sentinel (matches the original GL render graph).
struct RGTextureHandle {
    uint32_t id = 0;
    bool IsValid() const { return id != 0; }
};

// What a graph-managed transient texture is. Usage is derived from the format
// (Depth32F → depth attachment; everything else → color attachment + sampled), so
// a pass only states size + format and the graph wires the rest.
struct RGTextureDesc {
    uint32_t  width  = 0;
    uint32_t  height = 0;
    RHIFormat format = RHIFormat::RGBA8;
};

// A node in the graph. reads/writes are the dependency data the graph topo-sorts
// on and — new for the RHI port — the source of the automatic image-layout
// barriers: every read is transitioned to SampledRead before the pass, and every
// write becomes a color/depth attachment (transitioned by BeginRendering).
struct RGPass {
    std::string                  name;
    std::vector<RGTextureHandle> reads;
    std::vector<RGTextureHandle> writes;        // color + depth; depth split out by format
    bool                         toSwapchain = false;  // write the backbuffer instead of textures
    bool                         clear       = true;   // CLEAR vs LOAD the write targets
    std::array<float, 4>         clearColor  { 0.0f, 0.0f, 0.0f, 1.0f };
    std::function<void(RHICommandList*)> execute;

    RGPass& Read(RGTextureHandle h)   { reads.push_back(h);  return *this; }
    RGPass& Write(RGTextureHandle h)  { writes.push_back(h); return *this; }
    RGPass& WriteSwapchain()          { toSwapchain = true;  return *this; }
    RGPass& SetClearColor(std::array<float, 4> c) { clear = true; clearColor = c; return *this; }
    RGPass& Load()                    { clear = false;       return *this; }
    RGPass& SetExecute(std::function<void(RHICommandList*)> fn) { execute = std::move(fn); return *this; }
};

// Backend-neutral render graph over the RHI. Same shape as the GL RenderGraph
// (declare textures, add passes with reads/writes, Compile = topo-sort + cull,
// Execute in order) but it records against an RHICommandList and never touches a
// backend type. Two differences from the GL version are forced by Vulkan:
//   * Transient textures are POOLED (kept alive across frames), not freed every
//     Execute — a resource referenced by an in-flight command buffer cannot be
//     freed. The render-target RHITexture is already per-frame-in-flight internally.
//   * reads/writes drive AUTOMATIC barriers instead of being pure ordering data.
// Typical use: declare + add passes + Compile once, then Execute every frame.
class RHIRenderGraph {
public:
    explicit RHIRenderGraph(RHIDevice* device);

    // Setup — declare resources and passes. Re-declaring a name with the same desc
    // reuses the pooled texture; a changed size/format recreates it.
    RGTextureHandle DeclareTexture(std::string_view name, const RGTextureDesc& desc);
    RGPass&         AddPass(std::string_view name);

    // Compile — topological sort (Kahn) + dead-pass culling. Ported verbatim from
    // the GL RenderGraph; the dependency model is unchanged.
    void Compile();

    // Execute — for each surviving pass in order: transition reads to SampledRead,
    // open a render scope on the writes (or the swapchain), run the pass, close it.
    void Execute(RHICommandList* cmd);

    // Drop the passes + declared-texture table (keeping the pooled textures) so the
    // graph can be rebuilt for a new frame. Not needed when Compiling once.
    void ResetPasses();

    // Resolve a declared handle to its (pool-owned) texture — e.g. to build a
    // resource set that samples a graph output.
    RHITexture* GetTexture(RGTextureHandle h) const;

private:
    struct TextureEntry {
        std::string   name;
        RGTextureDesc desc;
        RHITexture*   texture = nullptr;   // owned by m_Pool
        bool          isDepth = false;
    };
    struct PooledTexture {
        RGTextureDesc               desc;
        std::unique_ptr<RHITexture> texture;
    };

    RHIDevice*                                     m_Device;
    std::unordered_map<std::string, PooledTexture> m_Pool;   // survives ResetPasses
    std::vector<TextureEntry>                      m_Textures;
    std::vector<RGPass>                            m_Passes;
    std::vector<int>                               m_SortedIndices;
};

} // namespace Diamond
