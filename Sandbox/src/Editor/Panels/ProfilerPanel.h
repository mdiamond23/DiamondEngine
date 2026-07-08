#pragma once
#include "Panels.h"
#include "../EditorContext.h"
#include <glm/glm.hpp>
#include "Profiling/ProfilerStats.h"
#include <cstdint>
#include <vector>

// Renderer/engine stats viewer — see Docs/profiler-panel-design.md.
//
// The panel doesn't query the renderer itself (SceneRenderer::GetStats()
// plumbing is separate, out-of-scope work); whoever owns that hookup calls
// SetStats() once per completed frame. SetStats() just records the latest
// snapshot — the panel decides on its own cadence when to actually display it
// (see m_DisplayStats below), so the caller can call it every frame for free.
class ProfilerPanel : public Panel {
public:
    ProfilerPanel();
    ~ProfilerPanel() override = default;

    void SetContext(EditorContext* ctx) { m_Context = ctx; }
    void OnImGuiRender() override;

    // Latest completed-frame snapshot, handed in by the renderer/backend glue.
    void SetStats(const Diamond::RendererStats& stats) { m_LatestStats = stats; }

    // Which fields are meaningful for the running backend (e.g. descriptorWrites
    // is Vulkan-only); the panel greys out backend-inapplicable rows rather than
    // hiding them. Defaults to Vulkan (the more feature-complete backend) until
    // the caller reports otherwise.
    void SetBackendIsVulkan(bool isVulkan) { m_VulkanBackend = isVulkan; }

private:
    static constexpr int   kHistorySize     = 120;   // ~2s of samples at 60Hz
    static constexpr float kRefreshInterval = 0.25f; // 4 Hz, per design doc

    EditorContext* m_Context = nullptr;

    Diamond::RendererStats m_LatestStats;  // written by SetStats(), every frame
    Diamond::RendererStats m_DisplayStats; // throttled copy actually drawn

    float m_RefreshTimer = 0.0f;
    bool  m_Paused        = false;
    bool  m_VulkanBackend = true;

    float m_FrameTimeHistory[kHistorySize] = {};
    int   m_HistoryOffset = 0;
};