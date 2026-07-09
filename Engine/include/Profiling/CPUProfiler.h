#pragma once
#include <cstdint>
#include <vector>

// Compile-time switch: define DIAMOND_PROFILE_ENABLED=0 and every
// DIAMOND_PROFILE_* macro compiles to nothing.
#ifndef DIAMOND_PROFILE_ENABLED
#define DIAMOND_PROFILE_ENABLED 1
#endif

namespace Diamond
{
    // One row of the CPU-timer breakdown (Docs/profiler-panel-design.md,
    // CPU-scope phase). Rows arrive depth-first, so a parent immediately
    // precedes its children — `depth` drives panel indentation.
    struct CPUScopeStats
    {
        const char* name = nullptr; // points at the call site's string literal
        int      depth   = 0;
        float    avgMs   = 0.0f;    // EMA-smoothed inclusive time (α=0.15, like PassStats)
        float    maxMs   = 0.0f;    // worst frame over the last ~1-2 s
        float    lastMs  = 0.0f;    // raw previous-frame value (0 if the scope didn't run)
        uint32_t calls   = 0;       // times entered last frame
    };

    // Hierarchical frame-scoped CPU timers behind DIAMOND_PROFILE_SCOPE.
    //
    // - Keyed by name *pointer* within the parent scope (strcmp fallback for
    //   unmerged literals) — the hot path never hashes strings or allocates.
    //   `name` must therefore outlive the profiler: string literals only.
    // - Scopes may be recorded from any thread (thread-local trees, merged at
    //   EndFrame); rows from non-main threads appear under a thread header.
    // - Meant for systems and phases, not per-entity inner loops: EndScope
    //   takes an uncontended lock (~tens of ns).
    class CPUProfiler
    {
    public:
        static void BeginScope(const char* name);
        static void EndScope();

        // Main thread, once per loop iteration: merges every thread's tree,
        // updates EMA/max, rebuilds the snapshot, resets per-frame counters.
        static void EndFrame();

        // Last completed frame (as of the previous EndFrame). Main thread only.
        static const std::vector<CPUScopeStats>& GetSnapshot();
    };

    class CPUProfileScope
    {
    public:
        explicit CPUProfileScope(const char* name) { CPUProfiler::BeginScope(name); }
        ~CPUProfileScope() { CPUProfiler::EndScope(); }
        CPUProfileScope(const CPUProfileScope&)            = delete;
        CPUProfileScope& operator=(const CPUProfileScope&) = delete;
    };
}

#if DIAMOND_PROFILE_ENABLED
#define DIAMOND_PROFILE_CONCAT_INNER(a, b) a##b
#define DIAMOND_PROFILE_CONCAT(a, b) DIAMOND_PROFILE_CONCAT_INNER(a, b)
// `name` must be a string literal (or otherwise permanent — e.g. GameSystem::GetName()).
#define DIAMOND_PROFILE_SCOPE(name) \
    ::Diamond::CPUProfileScope DIAMOND_PROFILE_CONCAT(_diamondProfScope, __LINE__)(name)
#define DIAMOND_PROFILE_FUNCTION() DIAMOND_PROFILE_SCOPE(__FUNCTION__)
#else
#define DIAMOND_PROFILE_SCOPE(name) ((void)0)
#define DIAMOND_PROFILE_FUNCTION()  ((void)0)
#endif
