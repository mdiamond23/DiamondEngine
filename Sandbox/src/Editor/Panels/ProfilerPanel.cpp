#include "ProfilerPanel.h"
#include <imgui.h>
#include <algorithm>
#include <cstdio>
#include <cstring>
#include <vector>

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

    // three-way compare for floats (sort comparators below)
    int Cmp(float a, float b)     { return (a < b) ? -1 : (a > b) ? 1 : 0; }
    int Cmp(uint64_t a, uint64_t b) { return (a < b) ? -1 : (a > b) ? 1 : 0; }

    // Sorted row order for the pass table, per the active sort column. Stable,
    // so ties keep execution order. Sorting flattens the scope grouping — the
    // caller renders names as "Pass (Scope)" instead of under headers.
    std::vector<const Diamond::PassStats*> SortedPassRows(
        const std::vector<Diamond::PassStats>& passes, const ImGuiTableColumnSortSpecs& spec)
    {
        std::vector<const Diamond::PassStats*> rows;
        rows.reserve(passes.size());
        for (const Diamond::PassStats& p : passes) rows.push_back(&p);

        const bool asc = spec.SortDirection == ImGuiSortDirection_Ascending;
        std::stable_sort(rows.begin(), rows.end(),
            [&](const Diamond::PassStats* a, const Diamond::PassStats* b) {
                int cmp = 0;
                switch (spec.ColumnIndex) {
                    case 0: cmp = a->name.compare(b->name);            break;
                    case 1: cmp = Cmp(a->gpuMs, b->gpuMs);             break;
                    case 2: cmp = Cmp(a->cpuMs, b->cpuMs);             break;
                    case 3: cmp = Cmp((uint64_t)a->drawCalls, (uint64_t)b->drawCalls); break;
                    case 4: cmp = Cmp(a->triangles, b->triangles);     break;
                }
                return asc ? cmp < 0 : cmp > 0;
            });
        return rows;
    }

    // Same for the CPU-timer table. Sorting flattens the tree (indentation is
    // dropped); note parents include their children's time, so both appear.
    std::vector<const Diamond::CPUScopeStats*> SortedCpuRows(
        const std::vector<Diamond::CPUScopeStats>& scopes, const ImGuiTableColumnSortSpecs& spec)
    {
        std::vector<const Diamond::CPUScopeStats*> rows;
        rows.reserve(scopes.size());
        for (const Diamond::CPUScopeStats& c : scopes) rows.push_back(&c);

        const bool asc = spec.SortDirection == ImGuiSortDirection_Ascending;
        std::stable_sort(rows.begin(), rows.end(),
            [&](const Diamond::CPUScopeStats* a, const Diamond::CPUScopeStats* b) {
                int cmp = 0;
                switch (spec.ColumnIndex) {
                    case 0: cmp = std::strcmp(a->name, b->name);       break;
                    case 1: cmp = Cmp(a->avgMs, b->avgMs);             break;
                    case 2: cmp = Cmp(a->lastMs, b->lastMs);           break;
                    case 3: cmp = Cmp(a->maxMs, b->maxMs);             break;
                    case 4: cmp = Cmp((uint64_t)a->calls, (uint64_t)b->calls); break;
                }
                return asc ? cmp < 0 : cmp > 0;
            });
        return rows;
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
            m_DisplayCpuScopes = Diamond::CPUProfiler::GetSnapshot();
        }
    }

    ImGui::Separator();

    // Each section collapses independently (ImGui persists the open state);
    // the sampling above keeps running regardless, so reopening is current.
    if (ImGui::CollapsingHeader("Frame", ImGuiTreeNodeFlags_DefaultOpen)) {
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
    }

    const RendererStats& s = m_DisplayStats;
    if (ImGui::CollapsingHeader("Renderer Stats", ImGuiTreeNodeFlags_DefaultOpen) &&
        ImGui::BeginTable("##profiler_stats", 2,
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

    // Per-pass breakdown (Vulkan only — empty on GL, section hidden). Default
    // (no sort) shows execution order grouped by scope headers (Shadows, Game
    // View, Main View). Clicking a column sorts and flattens the grouping —
    // SortTristate means a third click returns to the grouped view. Hideable:
    // right-click the header row to toggle columns.
    if (!s.passes.empty() && ImGui::CollapsingHeader("Passes", ImGuiTreeNodeFlags_DefaultOpen)) {
        if (ImGui::BeginTable("##profiler_passes", 6,
                               ImGuiTableFlags_RowBg | ImGuiTableFlags_BordersInnerH |
                               ImGuiTableFlags_SizingStretchProp |
                               ImGuiTableFlags_Sortable | ImGuiTableFlags_SortTristate |
                               ImGuiTableFlags_Hideable)) {
            const ImGuiTableColumnFlags numFlags =
                ImGuiTableColumnFlags_WidthFixed | ImGuiTableColumnFlags_PreferSortDescending;
            ImGui::TableSetupColumn("Pass",   ImGuiTableColumnFlags_WidthStretch |
                                              ImGuiTableColumnFlags_NoHide);
            ImGui::TableSetupColumn("GPU ms", numFlags, 60.0f);
            ImGui::TableSetupColumn("CPU ms", numFlags, 60.0f);
            ImGui::TableSetupColumn("Draws",  numFlags, 50.0f);
            ImGui::TableSetupColumn("Tris",   numFlags, 70.0f);
            ImGui::TableSetupColumn("Size",   ImGuiTableColumnFlags_WidthFixed |
                                              ImGuiTableColumnFlags_NoSort, 80.0f);
            ImGui::TableHeadersRow();

            const ImGuiTableSortSpecs* sort = ImGui::TableGetSortSpecs();
            const bool sortActive = sort && sort->SpecsCount > 0;

            float gpuSum = 0.0f;
            for (const Diamond::PassStats& p : s.passes) gpuSum += p.gpuMs;

            if (sortActive) {
                char nameBuf[80];
                for (const Diamond::PassStats* p : SortedPassRows(s.passes, sort->Specs[0])) {
                    std::snprintf(nameBuf, sizeof(nameBuf), "%s (%s)",
                                  p->name.c_str(), p->scope.c_str());
                    PassRow(nameBuf, p->gpuMs, p->cpuMs, p->drawCalls, p->triangles,
                            p->width, p->height, /*indent*/ false);
                }
            } else {
                const std::string* lastScope = nullptr;
                for (const Diamond::PassStats& p : s.passes) {
                    if (!lastScope || *lastScope != p.scope) {
                        lastScope = &p.scope;
                        ImGui::TableNextRow();
                        ImGui::TableSetColumnIndex(0);
                        ImGui::TextColored(ImVec4(0.55f, 0.75f, 1.0f, 1.0f), "%s", p.scope.c_str());
                    }
                    PassRow(p.name.c_str(), p.gpuMs, p.cpuMs, p.drawCalls, p.triangles,
                            p.width, p.height, /*indent*/ true);
                }
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

    // CPU scope timers (DIAMOND_PROFILE_SCOPE) — engine systems + user
    // scripts. Default (no sort) is the tree, depth-first with `depth`
    // driving indentation; an active sort flattens it (parents include their
    // children's time, so both appear in the ranking). Backend-agnostic.
    if (!m_DisplayCpuScopes.empty() &&
        ImGui::CollapsingHeader("CPU Timers", ImGuiTreeNodeFlags_DefaultOpen)) {
        if (ImGui::BeginTable("##profiler_cpu", 5,
                               ImGuiTableFlags_RowBg | ImGuiTableFlags_BordersInnerH |
                               ImGuiTableFlags_SizingStretchProp |
                               ImGuiTableFlags_Sortable | ImGuiTableFlags_SortTristate |
                               ImGuiTableFlags_Hideable)) {
            const ImGuiTableColumnFlags numFlags =
                ImGuiTableColumnFlags_WidthFixed | ImGuiTableColumnFlags_PreferSortDescending;
            ImGui::TableSetupColumn("Scope",  ImGuiTableColumnFlags_WidthStretch |
                                              ImGuiTableColumnFlags_NoHide);
            ImGui::TableSetupColumn("Avg ms", numFlags, 60.0f);
            ImGui::TableSetupColumn("Last ms",numFlags, 60.0f);
            ImGui::TableSetupColumn("Max ms", numFlags, 60.0f);
            ImGui::TableSetupColumn("Calls",  numFlags, 50.0f);
            ImGui::TableHeadersRow();

            const ImGuiTableSortSpecs* sort = ImGui::TableGetSortSpecs();
            const bool sortActive = sort && sort->SpecsCount > 0;

            std::vector<const Diamond::CPUScopeStats*> rows;
            if (sortActive) {
                rows = SortedCpuRows(m_DisplayCpuScopes, sort->Specs[0]);
            } else {
                rows.reserve(m_DisplayCpuScopes.size());
                for (const Diamond::CPUScopeStats& c : m_DisplayCpuScopes)
                    rows.push_back(&c);
            }

            char buf[48];
            float rootLastSum = 0.0f;   // top-level scopes only — children are subsets
            for (const Diamond::CPUScopeStats* c : rows) {
                if (c->depth == 0) rootLastSum += c->lastMs;

                ImGui::TableNextRow();
                ImGui::TableSetColumnIndex(0);
                const float indent = sortActive ? 0.0f : 12.0f * static_cast<float>(c->depth);
                if (indent > 0.0f) ImGui::Indent(indent);
                ImGui::TextUnformatted(c->name);
                if (indent > 0.0f) ImGui::Unindent(indent);

                ImGui::TableSetColumnIndex(1);
                std::snprintf(buf, sizeof(buf), "%.2f", c->avgMs);
                RightAlignedText(buf);

                ImGui::TableSetColumnIndex(2);
                std::snprintf(buf, sizeof(buf), "%.2f", c->lastMs);
                RightAlignedText(buf);

                ImGui::TableSetColumnIndex(3);
                std::snprintf(buf, sizeof(buf), "%.2f", c->maxMs);
                RightAlignedText(buf);

                ImGui::TableSetColumnIndex(4);
                std::snprintf(buf, sizeof(buf), "%u", c->calls);
                RightAlignedText(buf);
            }

            // Unattributed main-loop time (input, event pump, everything not
            // under a scope). Clamped: cpuFrameMs and the scopes are measured
            // a frame apart, so a hitch can transiently push the sum over.
            ImGui::TableNextRow();
            ImGui::TableSetColumnIndex(0);
            ImGui::TextUnformatted("Other");
            ImGui::TableSetColumnIndex(2);
            std::snprintf(buf, sizeof(buf), "%.2f",
                          std::max(0.0f, s.cpuFrameMs - rootLastSum));
            RightAlignedText(buf);

            ImGui::EndTable();
        }
    }

    ImGui::End();
}
