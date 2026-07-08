#include "ConsolePanel.h"
#include <imgui.h>

void ConsolePanel::Log(LogLevel level, const std::string& message)
{
    switch (level)
    {
        case LogLevel::Warning: Debug::Warn(message);  break;
        case LogLevel::Error:   Debug::Error(message); break;
        default:                Debug::Log(message);   break;
    }
}

void ConsolePanel::Clear()
{
    Debug::Clear();
}

void ConsolePanel::OnImGuiRender()
{
    if (!m_Open) return;
    ImGui::Begin("Console", &m_Open);

    if (ImGui::Button("Clear"))
        Clear();
    ImGui::SameLine();
    ImGui::Checkbox("Auto-scroll", &m_AutoScroll);
    ImGui::Separator();

    ImGui::BeginChild("ScrollRegion", ImVec2(0, 0), false, ImGuiWindowFlags_HorizontalScrollbar);
    for (const auto& entry : Debug::GetEntries())
    {
        ImVec4 color;
        switch (entry.level)
        {
            case LogLevel::Warning: color = { 1.0f, 0.8f, 0.0f, 1.0f }; break;
            case LogLevel::Error:   color = { 1.0f, 0.3f, 0.3f, 1.0f }; break;
            default:                color = { 0.9f, 0.9f, 0.9f, 1.0f }; break;
        }
        ImGui::TextColored(color, "%s", entry.message.c_str());
    }
    if (m_AutoScroll)
        ImGui::SetScrollHereY(1.0f);
    ImGui::EndChild();

    ImGui::End();
}
