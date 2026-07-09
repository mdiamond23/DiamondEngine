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

    // One row of the per-pass table. cpuMs < 0 renders as "—" (the synthetic
    // "Other" row has no CPU recording time); width 0 likewise.
    void PassRow(const char* name, float gpuMs, float cpuMs,
                 uint32_t draws, uint64_t tris, uint32_t w, uint32_t h, bool indent) {
        char buf[48];
        ImGui::TableNextRow();

        ImGui::TableSetColumnIndex(0);
        if (indent) ImGui::Indent(12.0f);
        ImGui::TextUnformatted(name);
        if (indent) ImGui::Unindent(12.0f);

        ImGui::TableSetColumnIndex(1);
        std::snprintf(buf, sizeof(buf), "%.2f", gpuMs);
        RightAlignedText(buf);

        ImGui::TableSetColumnIndex(2);
        if (cpuMs >= 0.0f) {
            std::snprintf(buf, sizeof(buf), "%.2f", cpuMs);
            RightAlignedText(buf);
        } else {
            ImGui::TextDisabled("-");
        }

        ImGui::TableSetColumnIndex(3);
        std::snprintf(buf, sizeof(buf), "%u", draws);
        RightAlignedText(buf);

        ImGui::TableSetColumnIndex(4);
        if (tris >= 1000000)
            std::snprintf(buf, sizeof(buf), "%.1fM", static_cast<double>(tris) / 1.0e6);
        else if (tris >= 1000)
            std::snprintf(buf, sizeof(buf), "%.1fk", static_cast<double>(tris) / 1.0e3);
        else
            std::snprintf(buf, sizeof(buf), "%llu", static_cast<unsigned long long>(tris));
        RightAlignedText(buf);

        ImGui::TableSetColumnIndex(5);
        if (w > 0) {
            std::snprintf(buf, sizeof(buf), "%ux%u", w, h);
            RightAlignedText(buf);
        } else {
            ImGui::TextDisabled("-");
        }
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

    // Per-pass breakdown (Vulkan only — empty on GL, section hidden). Passes
    // arrive in execution order, already grouped by scope (Shadows, Game View,
    // Main View), so a header row on each scope change is enough.
    if (!s.passes.empty()) {
        ImGui::Separator();
        ImGui::TextUnformatted("Passes");

        if (ImGui::BeginTable("##profiler_passes", 6,
                               ImGuiTableFlags_RowBg | ImGuiTableFlags_BordersInnerH |
                               ImGuiTableFlags_SizingStretchProp)) {
            ImGui::TableSetupColumn("Pass",   ImGuiTableColumnFlags_WidthStretch);
            ImGui::TableSetupColumn("GPU ms", ImGuiTableColumnFlags_WidthFixed, 60.0f);
            ImGui::TableSetupColumn("CPU ms", ImGuiTableColumnFlags_WidthFixed, 60.0f);
            ImGui::TableSetupColumn("Draws",  ImGuiTableColumnFlags_WidthFixed, 50.0f);
            ImGui::TableSetupColumn("Tris",   ImGuiTableColumnFlags_WidthFixed, 70.0f);
            ImGui::TableSetupColumn("Size",   ImGuiTableColumnFlags_WidthFixed, 80.0f);
            ImGui::TableHeadersRow();

            float gpuSum = 0.0f;
            const std::string* lastScope = nullptr;
            for (const Diamond::PassStats& p : s.passes) {
                if (!lastScope || *lastScope != p.scope) {
                    lastScope = &p.scope;
                    ImGui::TableNextRow();
                    ImGui::TableSetColumnIndex(0);
                    ImGui::TextColored(ImVec4(0.55f, 0.75f, 1.0f, 1.0f), "%s", p.scope.c_str());
                }
                gpuSum += p.gpuMs;
                PassRow(p.name.c_str(), p.gpuMs, p.cpuMs, p.drawCalls, p.triangles,
                        p.width, p.height, /*indent*/ true);
            }

            // Work outside profiled passes: uploads, layout transitions, the
            // point-shadow cube seed copies, present transitions. Adjacent
            // passes can overlap on the GPU, so the sum may slightly exceed
            // the frame span — clamp instead of showing a negative.
            if (s.gpuFrameMs > 0.0f)
                PassRow("Other", std::max(0.0f, s.gpuFrameMs - gpuSum), -1.0f,
                        0, 0, 0, 0, /*indent*/ false);

            ImGui::EndTable();
        }
    }

    ImGui::End();
}
