#include "ProfilerPanel.h"
#include <imgui.h>
#include <algorithm>
#include <cstdio>

using Diamond::RendererStats;

namespace {
    // Right-aligns 'text' within the current table column.
    void RightAlignedText(const char* text) {
        const float avail = ImGui::GetContentRegionAvail().x;
        const float width  = ImGui::CalcTextSize(text).x;
        ImGui::SetCursorPosX(ImGui::GetCursorPosX() + std::max(0.0f, avail - width));
        ImGui::TextUnformatted(text);
    }

    void SectionHeader(const char* name) {
        ImGui::TableNextRow();
        ImGui::TableSetColumnIndex(0);
        ImGui::TextColored(ImVec4(0.55f, 0.75f, 1.0f, 1.0f), "%s", name);
        ImGui::TableSetColumnIndex(1);
    }

    // A stat row whose value is always meaningful for the running backend.
    void StatRow(const char* label, const char* valueText) {
        ImGui::TableNextRow();
        ImGui::TableSetColumnIndex(0);
        ImGui::TextUnformatted(label);
        ImGui::TableSetColumnIndex(1);
        RightAlignedText(valueText);
    }

    // A stat row that's backend-inapplicable right now — greyed with "n/a"
    // rather than hidden, so the row layout stays stable across backends.
    void NARow(const char* label) {
        ImGui::TableNextRow();
        ImGui::TableSetColumnIndex(0);
        ImGui::TextDisabled("%s", label);
        ImGui::TableSetColumnIndex(1);
        ImGui::TextDisabled("n/a");
    }

    void StatRowU32(const char* label, uint32_t value) {
        char buf[32];
        std::snprintf(buf, sizeof(buf), "%u", value);
        StatRow(label, buf);
    }

    void StatRowU64(const char* label, uint64_t value) {
        char buf[32];
        std::snprintf(buf, sizeof(buf), "%llu", static_cast<unsigned long long>(value));
        StatRow(label, buf);
    }
}

ProfilerPanel::ProfilerPanel() = default;

void ProfilerPanel::OnImGuiRender()
{
    if (!m_Open) return;
    if (!ImGui::Begin("Profiler", &m_Open)) { ImGui::End(); return; }

    ImGui::Checkbox("Pause", &m_Paused);
    ImGui::SameLine();
    ImGui::TextDisabled(m_Paused ? "(frozen)" : "(live)");

    // Graphs sample every frame; the numbers refresh at a fixed cadence so
    // they stay readable. Both freeze while paused, so the panel shows one
    // stable frame for inspection.
    if (!m_Paused) {
        m_FrameTimeHistory[m_HistoryOffset] = m_LatestStats.cpuFrameMs;
        m_HistoryOffset = (m_HistoryOffset + 1) % kHistorySize;

        m_RefreshTimer += ImGui::GetIO().DeltaTime;
        if (m_RefreshTimer >= kRefreshInterval) {
            m_RefreshTimer = 0.0f;
            m_DisplayStats = m_LatestStats;
        }
    }

    ImGui::Separator();

    ImGui::Text("FPS: %.1f", m_DisplayStats.fps);
    ImGui::SameLine(150.0f);
    ImGui::Text("CPU: %.2f ms", m_DisplayStats.cpuFrameMs);
    ImGui::SameLine(300.0f);
    if (m_DisplayStats.gpuFrameMs > 0.0f)
        ImGui::Text("GPU: %.2f ms", m_DisplayStats.gpuFrameMs);
    else
        ImGui::TextDisabled("GPU: n/a");

    ImGui::PlotLines("##frametime", m_FrameTimeHistory, kHistorySize, m_HistoryOffset,
                      nullptr, 0.0f, FLT_MAX, ImVec2(0.0f, 60.0f));

    ImGui::Separator();

    const RendererStats& s = m_DisplayStats;
    if (ImGui::BeginTable("##profiler_stats", 2,
                           ImGuiTableFlags_RowBg | ImGuiTableFlags_BordersInnerH)) {
        ImGui::TableSetupColumn("Stat", ImGuiTableColumnFlags_WidthStretch);
        ImGui::TableSetupColumn("Value", ImGuiTableColumnFlags_WidthFixed, 100.0f);

        SectionHeader("Submission");
        StatRowU32("Draw calls", s.drawCalls);
        StatRowU32("Dispatch calls", s.dispatchCalls);
        StatRowU64("Triangles", s.trianglesSubmitted);

        SectionHeader("Scene");
        StatRowU32("Visible objects", s.visibleObjects);
        StatRowU32("Culled objects", s.culledObjects);
        StatRowU32("Shadow casters", s.shadowCasters);

        SectionHeader("Bindings");
        StatRowU32("Materials bound", s.materialsBound);
        StatRowU32("Textures used", s.texturesUsed);
        if (m_VulkanBackend)
            StatRowU32("Descriptor writes", s.descriptorWrites);
        else
            NARow("Descriptor writes");
        StatRowU32("Buffer uploads", s.bufferUploads);

        SectionHeader("Memory");
        char vramBuf[32];
        std::snprintf(vramBuf, sizeof(vramBuf), "%.1f MB",
                      static_cast<double>(s.vramBytes) / (1024.0 * 1024.0));
        StatRow("VRAM (est.)", vramBuf);

        ImGui::EndTable();
    }

    ImGui::End();
}
