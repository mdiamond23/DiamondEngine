# Profiler Panel — Design

A new editor panel (`ProfilerPanel`) showing per-frame renderer and engine statistics,
for both backends (GL + Vulkan).

## Architecture decision

**The renderer owns the stats; the panel queries a getter.** (Option 2 from the
discussion, and the standard approach — Unreal's `RHI` stats, Hazel's
`Renderer2D::Statistics`, etc. all work this way.)

```
┌────────────┐  ResetStats() @ BeginFrame   ┌──────────────────┐
│ Renderer / │  passes increment counters   │ RenderStats      │
│ passes     │ ───────────────────────────► │ (plain struct,   │
└────────────┘                              │  owned by the    │
                                            │  renderer)       │
┌────────────┐  GetStats() — a snapshot of  └──────────────────┘
│ Profiler   │  the *last completed* frame          ▲
│ Panel      │ ◄────────────────────────────────────┘
└────────────┘
```

### Double-buffered snapshot

Passes write into a *working* struct; at `EndFrame` it's copied into a *snapshot*
struct and cleared. `GetStats()` returns the snapshot. The panel therefore always
sees a complete, self-consistent frame — never a half-accumulated one — regardless
of where in the loop it draws.

## The struct

```cpp
// Engine/include/Renderer/RenderStats.h — backend-neutral, POD, no RHI types.
namespace Diamond {

struct RenderStats {
    // Timing
    float fps            = 0.0f;   // smoothed (EMA over ~0.5 s)
    float cpuFrameMs     = 0.0f;   // full main-loop iteration, measured by the app
    float gpuFrameMs     = 0.0f;   // GPU timestamp span (see Timing below)

    // Submission
    uint32_t drawCalls        = 0; // vkCmdDraw*/glDrawElements* issued
    uint32_t dispatchCalls    = 0; // compute dispatches (0 on GL today)
    uint64_t trianglesSubmitted = 0; // sum of indexCount/3 × instances

    // Scene / culling
    uint32_t visibleObjects   = 0; // draws surviving frustum culling (main view)
    uint32_t culledObjects    = 0; // draws rejected by frustum culling
    uint32_t shadowCasters    = 0; // draws submitted across all shadow passes

    // Bindings / resources
    uint32_t materialsBound   = 0; // unique materials referenced this frame
    uint32_t texturesUsed     = 0; // unique textures referenced this frame
    uint32_t descriptorWrites = 0; // vkUpdateDescriptorSets writes (0 on GL)
    uint32_t bufferUploads    = 0; // map/memcpy or staging copies this frame

    // Memory
    uint64_t vramBytes        = 0; // estimate — see VRAM below
};

} // namespace Diamond
```

Notes on the trickier fields:

- **materialsBound / texturesUsed** — counted as *unique per frame* via a small
  `std::unordered_set<const void*>` in the working accumulator (cleared each frame),
  not raw bind calls. Raw bind counts are also easy to add later if wanted.
- **dispatchCalls** — the deferred chain is raster-only today; the counter exists so
  SSAO-as-compute / GI work lands in it later. Expected to read 0 for now.
- **descriptorWrites** — Vulkan-only by nature; the GL backend reports 0 (the panel
  greys out backend-inapplicable rows rather than hiding them).

## Timing

- **FPS / CPU time** — measured by the app loop (`EditorBackend` shared loop), not
  the renderer: delta between loop iterations, exponentially smoothed. Passed into
  the stats via a tiny `SceneRenderer::SetCPUTiming(float ms)`-style setter, or the
  panel can read them from the app directly — either is fine; plan says the app
  writes them into the same struct so the panel has one source.
- **GPU time** —
  - *Vulkan*: `vkCmdWriteTimestamp` at the top and bottom of the frame's command
    buffer, `VK_QUERY_TYPE_TIMESTAMP` pool sized per frame-in-flight, results read
    with `vkGetQueryPoolResults` **for the frame that just finished on the GPU**
    (i.e. results are N-frames-in-flight old — that's normal and fine for a
    profiler). Convert via `VkPhysicalDeviceLimits::timestampPeriod`.
  - *GL*: `GL_TIME_ELAPSED` query objects, double-buffered (query frame N, read
    frame N-1) to avoid stalling. On macOS GL 4.1 timer queries exist but are
    flaky — if unavailable, report 0 and the panel shows "n/a".
  - Phase 2 nicety: per-pass GPU times (one timestamp pair per render-graph pass)
    shown as an expandable breakdown. The render graph makes this cheap to add.

## VRAM estimate

- *Vulkan*: real numbers from VMA — `vmaGetHeapBudgets()` gives usage per heap
  (device-local sum = the number we show). No bookkeeping needed.
- *GL*: no portable query, so keep a running tally at resource creation/destruction
  (texture size × mip chain, buffer sizes). It's labeled an *estimate* in the UI for
  exactly this reason. `GL_NVX_gpu_memory_info`/ATI equivalents optional later.

## Where counters get incremented

Deep-pass counters (drawCalls, triangles, shadowCasters, descriptorWrites,
bufferUploads) are incremented at the **choke points**, not sprinkled per-pass:

- *Vulkan*: `RHICommandList::DrawIndexed/Draw/Dispatch` wrappers — every pass
  already goes through them, so one increment site covers everything. Descriptor
  writes counted in the RHI resource layer; buffer uploads in the dynamic-UBO /
  staging paths.
- *GL*: the equivalent draw-submission helpers in the GL pass classes; if draws are
  raw `glDrawElements` calls today, this is the nudge to route them through one
  small `GLDraw()` helper (a worthwhile cleanup on its own).
- Culling counters increment where `Frustum` tests already run (visible++ / culled++).
- Shadow casters increment in the CSM/point/spot shadow passes' draw loops
  (they naturally also land in drawCalls — the panel presents them as a subset,
  not additive).

The working accumulator lives with the command-list/renderer internals (per-device,
not a process-global), so passes don't need a stats parameter threaded through.

## Public API surface

```cpp
// SceneRenderer.h (Vulkan) — and the mirror on the GL renderer:
virtual const RenderStats& GetStats() const = 0;   // last completed frame
```

`EditorBackend` exposes it to panels the same way viewport textures are exposed
today (via `EditorFrameInput` or a getter on the backend), so `ProfilerPanel`
stays backend-agnostic.

## The panel

`Sandbox/src/Editor/Panels/ProfilerPanel.{h,cpp}`, registered like MixerPanel.

- **Header strip**: FPS + CPU/GPU ms, with a small history graph
  (`ImGui::PlotLines` over a ~120-frame ring buffer kept panel-side).
- **Table** (ImGui table, right-aligned numbers): the remaining counters grouped as
  *Submission* / *Scene* / *Bindings* / *Memory*. Backend-inapplicable rows (e.g.
  descriptor writes on GL) shown greyed with "n/a".
- Update cadence: read `GetStats()` every frame, but refresh the displayed *numbers*
  at ~4 Hz (graphs update per frame) so values are readable, with a "pause" button
  to freeze a frame for inspection.
- No FontAwesome icons (per MixerPanel lesson — icon font isn't merged).

## Phases

1. **Struct + plumbing + panel, CPU-side counters only** — RenderStats.h, working/
   snapshot accumulators in both backends, draw/triangle/culling/shadow/material/
   texture/upload counters, FPS + CPU ms from the app loop, panel with table +
   FPS graph. *(Everything except gpuFrameMs, descriptorWrites, vramBytes.)*
2. **GPU timing** — Vulkan timestamp pool + GL timer queries, whole-frame first.
3. **Memory + Vulkan-specifics** — VMA heap budgets, GL creation-tally estimate,
   descriptor-write counter.
4. **Per-pass breakdown — DONE, Vulkan-only by decision** (GL keeps whole-frame
   stats only; per-pass `GL_TIME_ELAPSED` queries can't coexist with the
   whole-frame one, and GL retires at parity anyway). What shipped:
   - `PassStats` (scope, name, gpuMs, cpuMs, draws, tris, target size) in
     `RendererStats::passes`, in execution order.
   - `RHIDevice::Begin/EndPassProfile` (default no-op; Vulkan implements): a
     second timestamp pool (64 pass pairs × frames-in-flight), pass names +
     CPU counters captured at record time, resolved at the slot's next fence
     wait — rows are frames-in-flight old, like `gpuFrameMs`. GPU/CPU times
     EMA-smoothed (α=0.15) keyed `scope/name`.
   - `RHIRenderGraph::SetProfileScope("Main View"/"Game View")` brackets every
     pass in Execute; default-off so preview/thumbnail graphs stay out of the
     breakdown. Spot/point shadow `Record()` bracketed directly in
     `RenderToSwapchain` under scope "Shadows".
   - Draw/triangle attribution: `RecordDraw` also increments the open pass.
   - Panel: 6-column pass table grouped by scope headers, plus an "Other" row
     (`gpuFrameMs − Σ pass gpuMs`, clamped ≥ 0 — adjacent passes can overlap
     on the GPU, and uploads/seed-copies/transitions live outside passes).
   - **RenderDoc integration**: `VK_EXT_debug_utils` now enabled whenever
     available (decoupled from validation; the messenger stays
     validation-only). `RHICommandList::Begin/EndDebugLabel` wraps every graph
     pass + the shadow records; render-graph pooled textures carry
     `RHITextureDesc::debugName` → named images in the resource inspector.
5. **CPU scope timers — DONE, backend-agnostic.** What shipped:
   - `Engine/include/Profiling/CPUProfiler.h`: `DIAMOND_PROFILE_SCOPE("name")` /
     `DIAMOND_PROFILE_FUNCTION()` RAII macros (compile to nothing under
     `DIAMOND_PROFILE_ENABLED=0`). Zero-allocation hot path: scopes are keyed
     by the name literal's *pointer* within their parent (strcmp fallback for
     unmerged literals), so names must be string literals / permanent.
   - Thread-safe by design: thread-local scope trees, merged at
     `CPUProfiler::EndFrame()` (called by the shared editor loop in main.cpp,
     right after `RenderFrame`). Rows from non-main threads appear under a
     thread header row. Not for per-entity inner loops — EndScope takes an
     uncontended mutex.
   - Every `GameSystem` is timed automatically: `Scene::UpdateSystems` wraps
     each `OnUpdate` in a scope named by `GameSystem::GetName()`, which
     `DECLARE_SYSTEM` overrides with the class name — user scripts get a
     profiler row with zero effort. Engine phases instrumented directly:
     Scene Systems (+ per-system children), Transforms, Anim State Machines,
     Animators, IK, Ragdoll Sync, Audio Update, Editor ImGui, Render Submit
     (includes the vsync/present wait by design).
   - Per scope: EMA avg (α=0.15), raw last-frame, max over the last ~1–2 s
     window (so hitches survive the smoothing), calls/frame. Rows unseen for
     120 frames are evicted (play-mode-only scopes disappear ~2 s after stop).
   - Panel: "CPU Timers" table (Scope/Avg/Last/Max/Calls) indented by depth,
     plus an "Other" row = `cpuFrameMs − Σ depth-0 lastMs`, clamped ≥ 0.
     Panel reads `CPUProfiler::GetSnapshot()` directly on the same 4 Hz
     refresh/pause cadence as the renderer stats.
   - Panel UX: every section (Frame, Renderer Stats, Passes, CPU Timers) is a
     `CollapsingHeader`; the Passes + CPU tables are sortable (click a column;
     numeric columns default descending; tri-state — third click restores the
     grouped/tree view, since a sorted hierarchy is meaningless the active
     sort flattens it) and hideable (right-click the header row).
6. *(Optional later)* per-view culling split, bind-call counts.

## Open questions

- Editor-view + game-view: sum the two views (simplest, phase 1) or a per-view
  dropdown? Plan: **sum**, with per-view split deferred to phase 4.
- Thumbnail/preview renders (ThumbnailService, particle preview): excluded from
  stats where practical; they're sporadic and would make graphs spiky.
