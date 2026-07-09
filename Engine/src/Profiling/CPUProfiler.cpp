#include "Profiling/CPUProfiler.h"

#include <algorithm>
#include <chrono>
#include <cstring>
#include <memory>
#include <mutex>
#include <string>

// Thread-local scope trees, merged into one snapshot at EndFrame (see the
// header for the contract). Locking model:
//   - node creation and per-frame accumulation lock the owning ThreadState's
//     mutex (uncontended except during the merge itself);
//   - the owner thread's read-only child lookup in BeginScope is lock-free —
//     only the owner mutates the tree, and it does so under the lock the
//     merge also takes, so the merge never observes a half-built node;
//   - the open-scope stack is touched only by the owner thread, never locked.

namespace Diamond
{
namespace
{
    using Clock = std::chrono::steady_clock;

    constexpr float kEmaAlpha   = 0.15f; // matches PassStats smoothing
    constexpr int   kMaxWindow  = 60;    // frames per max-tracking window (~1 s @ 60)
    constexpr int   kEvictAfter = 120;   // frames unseen before a row disappears

    struct Node
    {
        const char* name   = nullptr;
        int         parent = -1;
        int         depth  = 0;
        std::vector<int> children;

        // Per-frame accumulation — owner thread writes, merge reads+resets,
        // both under the ThreadState mutex.
        long long frameNs    = 0;
        uint32_t  frameCalls = 0;

        // Display state — merge (main) thread only.
        float avgMs         = 0.0f;
        float lastMs        = 0.0f;
        float windowMax     = 0.0f;
        float prevWindowMax = 0.0f;
        int   framesUnseen  = 0;
    };

    struct OpenScope
    {
        int               node;
        Clock::time_point start;
    };

    struct ThreadState
    {
        std::mutex             mutex;
        std::string            label;
        std::vector<Node>      nodes;
        std::vector<int>       roots;
        std::vector<OpenScope> stack;   // owner thread only

        int FindOrAddChild(int parent, const char* name)
        {
            const std::vector<int>& list = (parent < 0) ? roots : nodes[parent].children;
            for (int idx : list) {
                const Node& n = nodes[idx];
                if (n.name == name || std::strcmp(n.name, name) == 0)
                    return idx;
            }

            std::lock_guard lock(mutex);
            Node n;
            n.name   = name;
            n.parent = parent;
            n.depth  = (parent < 0) ? 0 : nodes[parent].depth + 1;
            nodes.push_back(std::move(n));
            const int idx = static_cast<int>(nodes.size()) - 1;
            ((parent < 0) ? roots : nodes[parent].children).push_back(idx);
            return idx;
        }
    };

    struct Registry
    {
        std::mutex mutex;   // guards `threads`; EndFrame holds it for the whole merge
        std::vector<std::unique_ptr<ThreadState>> threads;  // never removed — TLS points in
        std::vector<CPUScopeStats> snapshot;                // main thread only
        int windowCounter = 0;
    };

    Registry& GetRegistry()
    {
        static Registry r;
        return r;
    }

    ThreadState& GetThreadState()
    {
        thread_local ThreadState* tls = nullptr;
        if (!tls) {
            Registry& reg = GetRegistry();
            std::lock_guard lock(reg.mutex);
            auto state   = std::make_unique<ThreadState>();
            state->label = "Thread " + std::to_string(reg.threads.size());
            tls          = state.get();
            reg.threads.push_back(std::move(state));
        }
        return *tls;
    }

    // Merge one node: fold this frame's accumulation into the display state,
    // reset it, and (if still live) emit a snapshot row before its children.
    void WalkNode(ThreadState& ts, int idx, bool rollWindow, std::vector<CPUScopeStats>& out)
    {
        Node& n = ts.nodes[idx];
        const float ms = static_cast<float>(static_cast<double>(n.frameNs) * 1e-6);
        if (n.frameCalls > 0) {
            n.lastMs       = ms;
            n.avgMs        = (n.avgMs <= 0.0f) ? ms : n.avgMs + kEmaAlpha * (ms - n.avgMs);
            n.windowMax    = std::max(n.windowMax, ms);
            n.framesUnseen = 0;
        } else {
            n.lastMs = 0.0f;
            n.framesUnseen++;
        }
        const uint32_t calls = n.frameCalls;
        n.frameNs    = 0;
        n.frameCalls = 0;
        if (rollWindow) {
            n.prevWindowMax = n.windowMax;
            n.windowMax     = 0.0f;
        }

        if (n.framesUnseen < kEvictAfter)
            out.push_back({ n.name, n.depth, n.avgMs,
                            std::max(n.windowMax, n.prevWindowMax), n.lastMs, calls });

        for (int child : n.children)
            WalkNode(ts, child, rollWindow, out);
    }
}

void CPUProfiler::BeginScope(const char* name)
{
    ThreadState& ts    = GetThreadState();
    const int    parent = ts.stack.empty() ? -1 : ts.stack.back().node;
    const int    node   = ts.FindOrAddChild(parent, name);
    ts.stack.push_back({ node, Clock::now() });
}

void CPUProfiler::EndScope()
{
    const Clock::time_point end = Clock::now();
    ThreadState& ts = GetThreadState();
    if (ts.stack.empty()) return;   // mismatched End — ignore rather than crash
    const OpenScope open = ts.stack.back();
    ts.stack.pop_back();

    const long long ns =
        std::chrono::duration_cast<std::chrono::nanoseconds>(end - open.start).count();
    std::lock_guard lock(ts.mutex);
    Node& n = ts.nodes[open.node];
    n.frameNs += ns;
    n.frameCalls++;
}

void CPUProfiler::EndFrame()
{
    ThreadState& mainState = GetThreadState();  // registers the main thread on first call
    mainState.label        = "Main";

    Registry& reg = GetRegistry();
    std::lock_guard regLock(reg.mutex);

    const bool rollWindow = ++reg.windowCounter >= kMaxWindow;
    if (rollWindow) reg.windowCounter = 0;

    // Merge each thread's tree, main thread first so its rows lead the table.
    struct ThreadRows
    {
        const char* label;
        std::vector<CPUScopeStats> rows;
    };
    std::vector<ThreadRows> perThread;
    perThread.reserve(reg.threads.size());
    for (int pass = 0; pass < 2; ++pass) {
        for (auto& statePtr : reg.threads) {
            ThreadState& ts = *statePtr;
            if ((pass == 0) != (&ts == &mainState)) continue;
            std::lock_guard lock(ts.mutex);
            ThreadRows rows{ ts.label.c_str(), {} };
            for (int root : ts.roots)
                WalkNode(ts, root, rollWindow, rows.rows);
            if (!rows.rows.empty())
                perThread.push_back(std::move(rows));
        }
    }

    // Single-thread frames (the common case) skip the header rows entirely.
    reg.snapshot.clear();
    const bool multiThread = perThread.size() > 1;
    for (ThreadRows& t : perThread) {
        if (multiThread) {
            CPUScopeStats header{ t.label, 0 };
            for (const CPUScopeStats& r : t.rows) {
                if (r.depth != 0) continue;
                header.avgMs  += r.avgMs;
                header.maxMs  += r.maxMs;
                header.lastMs += r.lastMs;
            }
            reg.snapshot.push_back(header);
        }
        for (CPUScopeStats& r : t.rows) {
            if (multiThread) r.depth++;
            reg.snapshot.push_back(r);
        }
    }
}

const std::vector<CPUScopeStats>& CPUProfiler::GetSnapshot()
{
    return GetRegistry().snapshot;
}
}
