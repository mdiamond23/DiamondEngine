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
        it->second.desc.width  != desc.width  ||
        it->second.desc.height != desc.height ||
        it->second.desc.format != desc.format)
    {
        RHITextureDesc td;
        td.width  = desc.width;
        td.height = desc.height;
        td.format = desc.format;
        // Depth targets are also Sampled: shadow maps (CSM cascades) and depth
        // pre-passes are written as a depth attachment, then read in a later pass.
        td.usage  = isDepth ? (RHITextureUsage::DepthAttachment | RHITextureUsage::Sampled)
                            : (RHITextureUsage::ColorAttachment | RHITextureUsage::Sampled);
        it = m_Pool.insert_or_assign(key, PooledTexture{ desc, m_Device->CreateTexture(td) }).first;
    }

    m_Textures.push_back(TextureEntry{ key, desc, it->second.texture.get(), isDepth });
    // id is 1-based so id=0 stays the "invalid" sentinel (matches IsValid())
    return RGTextureHandle{ static_cast<uint32_t>(m_Textures.size()) };
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

    // 1. Group the writers of each texture in pass-insertion order, and record the
    //    LAST writer (the "producer" a reader depends on).

    // writerMap[id] = -1 means no pass writes this texture
    std::vector<int>              writerMap(nTexSlot, -1);
    std::vector<std::vector<int>> writersOf(nTexSlot);

    for (int i = 0; i < nPasses; ++i)
    {
        for (const RGTextureHandle& h : m_Passes[i].writes)
        {
            writersOf[h.id].push_back(i);
            writerMap[h.id] = i; // last write wins
        }
    }

    // 2. Build the dependency edges (deduplicated) as a predecessor set per pass:
    //    - write-after-write: each writer of a texture depends on the previous one.
    //    - read-after-write: a reader depends on that texture's LAST writer.
    std::vector<std::set<int>> preds(nPasses);

    for (int id = 1; id < nTexSlot; ++id)
    {
        const std::vector<int>& w = writersOf[id];
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
            if (outputs.count(h.id))
                return true;
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
    for (int passIdx : m_SortedIndices)
    {
        RGPass& pass = m_Passes[passIdx];

        // Auto-barrier: each read must be shader-readable before the pass runs.
        // Writes are transitioned to their attachment layout by BeginRendering.
        for (const RGTextureHandle& h : pass.reads)
            cmd->TransitionTexture(m_Textures[h.id - 1].texture, RHITextureState::SampledRead);

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
}

void RHIRenderGraph::ResetPasses()
{
    // Keep m_Pool — pooled textures outlive the per-frame pass/declaration tables.
    m_Passes.clear();
    m_Textures.clear();
    m_OutputIds.clear();
    m_SortedIndices.clear();
}

} // namespace Diamond
