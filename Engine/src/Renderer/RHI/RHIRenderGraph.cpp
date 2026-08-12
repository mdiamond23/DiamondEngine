#include "Renderer/RHI/RHIRenderGraph.h"
#include "Renderer/RHI/RHIDevice.h"
#include "Renderer/RHI/RHICommandList.h"

#include <spdlog/spdlog.h>
#include <queue>
#include <set>

namespace Diamond {

RHIRenderGraph::RHIRenderGraph(RHIDevice* device) : m_Device(device) {}

RGTextureHandle RHIRenderGraph::DeclareTexture(std::string_view name, const RGTextureDesc& desc)
{
    const bool isDepth = (desc.format == RHIFormat::Depth32F);

    // Reuse the pooled image unless its size/format changed — transient textures
    // are never freed mid-frame (they may be referenced by an in-flight frame), so
    // the pool persists across rebuilds and only recreates on a real description
    // change.
    std::string key(name);
    auto it = m_Pool.find(key);
    if (it == m_Pool.end() ||
        it->second.desc.width      != desc.width  ||
        it->second.desc.height     != desc.height ||
        it->second.desc.format     != desc.format ||
        it->second.desc.storage    != desc.storage ||
        it->second.desc.persistent != desc.persistent)
    {
        RHITextureDesc td;
        td.width     = desc.width;
        td.height    = desc.height;
        td.format    = desc.format;
        td.debugName = key.c_str();   // names the images in capture tools
        // Depth targets are also Sampled: shadow maps (CSM cascades) and depth
        // pre-passes are written as a depth attachment, then read in a later pass.
        td.usage  = isDepth ? (RHITextureUsage::DepthAttachment | RHITextureUsage::Sampled)
                            : (RHITextureUsage::ColorAttachment | RHITextureUsage::Sampled);
        if (desc.storage) td.usage = td.usage | RHITextureUsage::Storage;
        // Persistent == one image instead of one per frame-in-flight, which is
        // exactly what makes "what frame N wrote" readable in frame N+1.
        td.singleBuffered = desc.persistent;
        it = m_Pool.insert_or_assign(key, PooledTexture{ desc, m_Device->CreateTexture(td) }).first;
    }

    m_Textures.push_back(TextureEntry{ key, desc, it->second.texture.get(), isDepth, false });
    // id is 1-based so id=0 stays the "invalid" sentinel (matches IsValid())
    return RGTextureHandle{ static_cast<uint32_t>(m_Textures.size()) };
}

RGTextureHandle RHIRenderGraph::ImportTexture(std::string_view name, RHITexture* texture,
                                              const RGTextureDesc& desc)
{
    const bool isDepth = (desc.format == RHIFormat::Depth32F);
    m_Textures.push_back(TextureEntry{ std::string(name), desc, texture, isDepth, true });
    return RGTextureHandle{ static_cast<uint32_t>(m_Textures.size()) };
}

RGBufferHandle RHIRenderGraph::DeclareBuffer(std::string_view name, const RGBufferDesc& desc)
{
    std::string key(name);
    auto it = m_BufferPool.find(key);
    if (it == m_BufferPool.end() ||
        it->second.desc.size    != desc.size ||
        it->second.desc.dynamic != desc.dynamic)
    {
        RHIBufferDesc bd;
        bd.size    = desc.size;
        bd.usage   = RHIBufferUsage::Storage;
        bd.dynamic = desc.dynamic;
        // No initialData: a graph buffer is GPU-produced (or CPU-rewritten each
        // frame when dynamic), never uploaded once at creation.
        it = m_BufferPool.insert_or_assign(key, PooledBuffer{ desc, m_Device->CreateBuffer(bd) }).first;
    }

    m_Buffers.push_back(BufferEntry{ key, desc, it->second.buffer.get() });
    return RGBufferHandle{ static_cast<uint32_t>(m_Buffers.size()) };
}

RGPass& RHIRenderGraph::AddPass(std::string_view name)
{
    m_Passes.push_back(RGPass{ std::string(name) });
    return m_Passes.back();
}

RHITexture* RHIRenderGraph::GetTexture(RGTextureHandle h) const
{
    return m_Textures[h.id - 1].texture;
}

RHIBuffer* RHIRenderGraph::GetBuffer(RGBufferHandle h) const
{
    return m_Buffers[h.id - 1].buffer;
}

void RHIRenderGraph::MarkOutput(RGTextureHandle h)
{
    if (h.IsValid())
        m_OutputIds.push_back(h.id);
}

// Compile phase — topological sort + dead-pass culling. Based on the GL
// RenderGraph's model, generalized to support MULTIPLE writers per texture: a
// compositing pass (particles/forward/UI) writes an already-produced target with
// .Load(), so the graph must order such writers in insertion order (write-after-
// write), point readers at the LAST writer, and keep every writer of a live
// target alive. The single-writer GL version culled the earlier writer.
void RHIRenderGraph::Compile()
{
    const int nPasses  = (int)m_Passes.size();
    const int nTexSlot = (int)m_Textures.size() + 1;
    const int nBufSlot = (int)m_Buffers.size() + 1;

    // 1. Group the writers of each resource in pass-insertion order, and record
    //    the LAST writer (the "producer" a reader depends on).

    // writerMap[id] = -1 means no pass writes this resource
    std::vector<int>              writerMap(nTexSlot, -1);
    std::vector<std::vector<int>> writersOf(nTexSlot);
    std::vector<int>              bufWriterMap(nBufSlot, -1);
    std::vector<std::vector<int>> bufWritersOf(nBufSlot);

    for (int i = 0; i < nPasses; ++i)
    {
        for (const RGTextureHandle& h : m_Passes[i].writes)
        {
            writersOf[h.id].push_back(i);
            writerMap[h.id] = i; // last write wins
        }
        for (const RGBufferHandle& h : m_Passes[i].bufferWrites)
        {
            bufWritersOf[h.id].push_back(i);
            bufWriterMap[h.id] = i;
        }
    }

    // 2. Build the dependency edges (deduplicated) as a predecessor set per pass:
    //    - write-after-write: each writer of a resource depends on the previous one.
    //    - read-after-write: a reader depends on that resource's LAST writer.
    // historyReads are deliberately absent from both: they consume the previous
    // frame's contents of a persistent texture, so they impose no ordering on
    // this frame's writer — which is exactly what keeps a read-then-rewrite
    // accumulation (TAA history, DDGI probes) from compiling as a cycle.
    std::vector<std::set<int>> preds(nPasses);

    for (int id = 1; id < nTexSlot; ++id)
    {
        const std::vector<int>& w = writersOf[id];
        for (size_t k = 1; k < w.size(); ++k)
            if (w[k - 1] != w[k])
                preds[w[k]].insert(w[k - 1]);
    }
    for (int id = 1; id < nBufSlot; ++id)
    {
        const std::vector<int>& w = bufWritersOf[id];
        for (size_t k = 1; k < w.size(); ++k)
            if (w[k - 1] != w[k])
                preds[w[k]].insert(w[k - 1]);
    }
    for (int j = 0; j < nPasses; ++j)
    {
        for (const RGTextureHandle& r : m_Passes[j].reads)
        {
            const int w = writerMap[r.id];
            if (w != -1 && w != j) // ignore self read-modify-write of an own target
                preds[j].insert(w);
        }
        for (const RGBufferHandle& r : m_Passes[j].bufferReads)
        {
            const int w = bufWriterMap[r.id];
            if (w != -1 && w != j)
                preds[j].insert(w);
        }
        // Write-after-read on a persistent texture: this frame's writer must run
        // AFTER everyone who wanted last frame's value, or it would clobber the
        // history out from under them. This is the edge that makes a history read
        // ordered rather than merely acyclic — and it points writer-after-reader,
        // the opposite direction from the read-after-write above, so the pair
        // can't close a loop on its own.
        for (const RGTextureHandle& r : m_Passes[j].historyReads)
            for (int w : writersOf[r.id])
                if (w != j)
                    preds[w].insert(j);
    }

    // 3. Topological sort using Kahn's algorithm over the edge set.
    std::vector<int>              inDegree(nPasses, 0);
    std::vector<std::vector<int>> succ(nPasses);
    for (int j = 0; j < nPasses; ++j)
    {
        inDegree[j] = (int)preds[j].size();
        for (int p : preds[j])
            succ[p].push_back(j);
    }

    std::queue<int> ready;
    for (int i = 0; i < nPasses; ++i)
        if (inDegree[i] == 0)
            ready.push(i);

    std::vector<int> sorted;
    sorted.reserve(nPasses);

    while (!ready.empty())
    {
        int i = ready.front();
        ready.pop();
        sorted.push_back(i);

        for (int j : succ[i])
            if (--inDegree[j] == 0)
                ready.push(j); // j's last dependency is satisfied
    }

    if ((int)sorted.size() != nPasses)
        spdlog::error("RHIRenderGraph::Compile — cycle detected in pass dependencies!");

    // 4. Dead pass culling — walk sorted in reverse, mark alive passes.
    // Sink passes (no writes) are always alive — they write to the backbuffer.
    // Writers of a marked output are alive too: their reader lives outside this
    // graph. Marking a live pass's predecessors alive keeps EVERY writer of a
    // needed target (not just the last) and its whole upstream chain.
    std::set<uint32_t> outputs(m_OutputIds.begin(), m_OutputIds.end());
    auto writesOutput = [&](const RGPass& p) {
        for (const RGTextureHandle& h : p.writes)
        {
            // Persistent and imported targets are outputs by definition: the
            // consumer of a persistent one is the NEXT frame (reached by a
            // historyRead, which is edgeless), and an imported one's consumer is
            // outside the graph entirely. Culling either would silently stop the
            // accumulation.
            const TextureEntry& e = m_Textures[h.id - 1];
            if (outputs.count(h.id) || e.desc.persistent || e.imported)
                return true;
        }
        return false;
    };

    std::vector<int> alive(nPasses, 0); // indexed by pass index, not sorted position

    for (int i = (int)sorted.size() - 1; i >= 0; --i)
    {
        int passIdx = sorted[i];

        if (m_Passes[passIdx].writes.empty() || writesOutput(m_Passes[passIdx]))
            alive[passIdx] = 1; // sink pass or external output — always alive

        if (alive[passIdx])
        {
            // Any pass this one depends on is also alive.
            for (int p : preds[passIdx])
                alive[p] = 1;
        }
    }

    // 5. Store final execution order — only alive passes, in sorted order
    m_SortedIndices.clear();
    for (int passIdx : sorted)
    {
        if (alive[passIdx])
            m_SortedIndices.push_back(passIdx);
    }
}

// Execute phase — drive the command list. No GL-style transient allocation/free
// here: textures are pooled (created at declare, owned for the graph's lifetime).
void RHIRenderGraph::Execute(RHICommandList* cmd)
{
    const bool profile = !m_ProfileScope.empty();
    // Buffers carry no layout, so their write→read hazard needs an explicit
    // barrier where images get one for free from the layout transition. Tracked
    // so the first buffer-touching pass of a frame doesn't emit a pointless one.
    bool anyBufferAccess = false;

    for (int passIdx : m_SortedIndices)
    {
        RGPass& pass = m_Passes[passIdx];

        // Capture-tool group for the whole pass. The profile bracket sits just
        // inside it and also opens before the read barriers, so barrier cost is
        // attributed to the pass that required it. Target size: first write's
        // declared size, or 0x0 = backbuffer (the device substitutes its extent).
        cmd->BeginDebugLabel(pass.name.c_str());
        if (profile)
        {
            uint32_t w = 0, h = 0;
            if (!pass.toSwapchain && !pass.writes.empty())
            {
                const RGTextureDesc& d = m_Textures[pass.writes[0].id - 1].desc;
                w = d.width;
                h = d.height;
            }
            m_Device->BeginPassProfile(m_ProfileScope.c_str(), pass.name.c_str(), w, h);
        }

        // Auto-barrier: each read must be shader-readable before the pass runs.
        // History reads transition identically — only their *ordering* differs.
        for (const RGTextureHandle& h : pass.reads)
            cmd->TransitionTexture(m_Textures[h.id - 1].texture, RHITextureState::SampledRead);
        for (const RGTextureHandle& h : pass.historyReads)
            cmd->TransitionTexture(m_Textures[h.id - 1].texture, RHITextureState::SampledRead);

        const bool touchesBuffers = !pass.bufferReads.empty() || !pass.bufferWrites.empty();
        if (touchesBuffers && anyBufferAccess)
            cmd->StorageBarrier();

        if (pass.type == RGPassType::Compute)
        {
            // No render scope: a compute pass's writes are storage images, moved
            // to the General layout the same way a raster pass's writes are moved
            // to their attachment layout by BeginRendering.
            for (const RGTextureHandle& h : pass.writes)
                cmd->TransitionTexture(m_Textures[h.id - 1].texture, RHITextureState::Storage);

            if (pass.execute)
                pass.execute(cmd);
        }
        else
        {
            // Build the render scope from the writes (or the swapchain backbuffer).
            RHIRenderPass rp;
            if (pass.toSwapchain)
            {
                rp.toSwapchain      = true;
                rp.colorAttachments = { RHIAttachment{ nullptr, pass.clear, pass.clearColor } };
            }
            else
            {
                for (const RGTextureHandle& h : pass.writes)
                {
                    TextureEntry& e = m_Textures[h.id - 1];
                    if (e.isDepth)
                    {
                        rp.depthTexture = e.texture;
                        rp.clearDepth   = pass.clear;
                    }
                    else
                    {
                        rp.colorAttachments.push_back(
                            RHIAttachment{ e.texture, pass.clear, pass.clearColor });
                    }
                }
            }

            cmd->BeginRendering(rp);
            if (pass.execute)
                pass.execute(cmd);
            cmd->EndRendering();
        }

        if (touchesBuffers)
            anyBufferAccess = true;

        if (profile)
            m_Device->EndPassProfile();
        cmd->EndDebugLabel();
    }
}

void RHIRenderGraph::ResetPasses()
{
    // Keep the pools — pooled resources outlive the per-frame declaration tables.
    m_Passes.clear();
    m_Textures.clear();
    m_Buffers.clear();
    m_OutputIds.clear();
    m_SortedIndices.clear();
}

} // namespace Diamond
