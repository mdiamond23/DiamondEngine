#pragma once

#include "Renderer/RHI/RHIEnums.h"

#include <array>
#include <cstdint>
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
class RHIBuffer;

// Handle into the graph's texture table. id is 1-based so 0 stays the "invalid"
// sentinel (matches the original GL render graph).
struct RGTextureHandle {
    uint32_t id = 0;
    bool IsValid() const { return id != 0; }
};

// Handle into the graph's buffer table, same 1-based convention. Separate from
// RGTextureHandle so a buffer can never be passed where an image is meant.
struct RGBufferHandle {
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

    // Additionally bindable as a compute storage image (image2D). Required for
    // any texture a compute pass writes — the usage flag is fixed at creation,
    // so the graph can't infer it from the passes declared later.
    bool storage = false;

    // Survives across frames: one image rather than one per frame-in-flight, so
    // what frame N wrote is exactly what frame N+1 reads. This is what makes a
    // resource an accumulation buffer (TAA history, DDGI probe atlas) instead of
    // a transient. Pair with RGPass::ReadHistory to consume last frame's value
    // without creating a same-frame dependency.
    bool persistent = false;
};

// A graph-managed storage buffer. Created device-local and GPU-produced (no
// initial contents); 'dynamic' instead makes it host-visible and rewritten from
// the CPU each frame, like RHIBufferDesc::dynamic.
struct RGBufferDesc {
    uint64_t size    = 0;
    bool     dynamic = false;
};

// Whether a pass records draws inside a render scope or dispatches compute. The
// distinction is what the graph does around the pass body: a raster pass gets
// BeginRendering/EndRendering over its writes, a compute pass gets its writes
// transitioned to the storage layout and no render scope at all.
enum class RGPassType { Raster, Compute };

// A node in the graph. reads/writes are the dependency data the graph topo-sorts
// on and — new for the RHI port — the source of the automatic image-layout
// barriers: every read is transitioned to SampledRead before the pass, and every
// write becomes a color/depth attachment (raster) or a storage image (compute).
struct RGPass {
    std::string                  name;
    RGPassType                   type = RGPassType::Raster;
    std::vector<RGTextureHandle> reads;
    std::vector<RGTextureHandle> writes;        // color + depth; depth split out by format
    // Reads of a PERSISTENT texture's previous-frame contents. Deliberately not
    // a dependency: the value comes from the last frame, so a pass may read a
    // texture that a later pass in the same frame rewrites without that being
    // the cycle an ordinary Read would (correctly) be.
    std::vector<RGTextureHandle> historyReads;
    std::vector<RGBufferHandle>  bufferReads;
    std::vector<RGBufferHandle>  bufferWrites;
    bool                         toSwapchain = false;  // write the backbuffer instead of textures
    bool                         clear       = true;   // CLEAR vs LOAD the write targets
    // Depth's clear, separable from the color one above. A depth PREPASS needs
    // exactly this split: the G-buffer that follows must still CLEAR its color
    // targets (gViewPos = 0 is what marks a background pixel downstream) while
    // LOADING the depth the prepass just wrote, so early-Z can reject the
    // fragments it already knows are occluded.
    bool                         clearDepth  = true;
    std::array<float, 4>         clearColor  { 0.0f, 0.0f, 0.0f, 1.0f };
    std::function<void(RHICommandList*)> execute;

    RGPass& Read(RGTextureHandle h)         { reads.push_back(h);  return *this; }
    RGPass& Write(RGTextureHandle h)        { writes.push_back(h); return *this; }
    RGPass& ReadHistory(RGTextureHandle h)  { historyReads.push_back(h); return *this; }
    RGPass& ReadBuffer(RGBufferHandle h)    { bufferReads.push_back(h);  return *this; }
    RGPass& WriteBuffer(RGBufferHandle h)   { bufferWrites.push_back(h); return *this; }
    RGPass& AsCompute()                     { type = RGPassType::Compute; return *this; }
    RGPass& WriteSwapchain()          { toSwapchain = true;  return *this; }
    RGPass& SetClearColor(std::array<float, 4> c) { clear = true; clearColor = c; return *this; }
    RGPass& Load()                    { clear = false;       return *this; }
    // Keep the color clear, but LOAD depth (see clearDepth above).
    RGPass& LoadDepth()               { clearDepth = false;  return *this; }
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
    RGBufferHandle  DeclareBuffer(std::string_view name, const RGBufferDesc& desc);
    RGPass&         AddPass(std::string_view name);

    // Bring an externally-owned texture under the graph's dependency tracking and
    // automatic barriers, without the graph owning it. 'desc' describes what the
    // texture already is (the graph needs its size/format for attachment
    // classification and profiling); 'texture' must outlive the graph. Like a
    // persistent texture, an imported one is never culled — its writers are kept
    // because its consumer may live outside this graph.
    RGTextureHandle ImportTexture(std::string_view name, RHITexture* texture,
                                  const RGTextureDesc& desc);

    // Declare 'h' an output consumed OUTSIDE this graph (another graph's frame, an
    // ImGui image, ...). Its writers have no reader here, so culling would drop
    // them; marking keeps them (and their upstream chain) alive. The external
    // consumer is responsible for transitioning the texture to SampledRead after
    // Execute. Cleared by ResetPasses, like the pass list.
    void MarkOutput(RGTextureHandle h);

    // Compile — topological sort (Kahn) + dead-pass culling. Ported verbatim from
    // the GL RenderGraph; the dependency model is unchanged.
    void Compile();

    // Execute — for each surviving pass in order: transition reads to SampledRead,
    // open a render scope on the writes (or the swapchain), run the pass, close it.
    void Execute(RHICommandList* cmd);

    // Drop the passes + declared-texture table (keeping the pooled textures) so the
    // graph can be rebuilt for a new frame. Not needed when Compiling once.
    void ResetPasses();

    // Opt this graph into per-pass profiling (RHIDevice::BeginPassProfile around
    // every pass in Execute), grouped in the profiler under 'scope' ("Main View",
    // "Game View"). Default off, so sporadic graphs (thumbnails, particle
    // preview) don't spike the breakdown. Survives ResetPasses. Debug labels for
    // capture tools are emitted regardless of this setting.
    void SetProfileScope(std::string_view scope) { m_ProfileScope = scope; }

    // Resolve a declared handle to its (pool-owned) resource — e.g. to build a
    // resource set that samples a graph output.
    RHITexture* GetTexture(RGTextureHandle h) const;
    RHIBuffer*  GetBuffer(RGBufferHandle h) const;

private:
    struct TextureEntry {
        std::string   name;
        RGTextureDesc desc;
        RHITexture*   texture = nullptr;   // owned by m_Pool unless imported
        bool          isDepth = false;
        bool          imported = false;
    };
    struct BufferEntry {
        std::string  name;
        RGBufferDesc desc;
        RHIBuffer*   buffer = nullptr;     // owned by m_BufferPool
    };
    struct PooledTexture {
        RGTextureDesc               desc;
        std::unique_ptr<RHITexture> texture;
    };
    struct PooledBuffer {
        RGBufferDesc               desc;
        std::unique_ptr<RHIBuffer> buffer;
    };

    RHIDevice*                                     m_Device;
    std::string                                    m_ProfileScope;   // empty = don't profile
    std::unordered_map<std::string, PooledTexture> m_Pool;   // survives ResetPasses
    std::unordered_map<std::string, PooledBuffer>  m_BufferPool;
    std::vector<TextureEntry>                      m_Textures;
    std::vector<BufferEntry>                       m_Buffers;
    std::vector<RGPass>                            m_Passes;
    std::vector<uint32_t>                          m_OutputIds;   // externally-consumed textures
    std::vector<int>                               m_SortedIndices;
};

} // namespace Diamond
