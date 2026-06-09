#include "GameViewportPanel.h"
#include <imgui.h>

void GameViewportPanel::OnImGuiRender()
{
    ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, {0, 0});
    ImGui::Begin("Game");
    ImGui::PopStyleVar();

    ImVec2 size = ImGui::GetContentRegionAvail();

    bool playing = m_Context && m_Context->ActiveScene->IsPlaying();

    if (playing && m_TextureID != 0)
    {
        ImGui::Image((ImTextureID)(uintptr_t)m_TextureID, size, {0, 1}, {1, 0});
    }
    else
    {
        // Centered placeholder text
        const char* msg = playing ? "No primary camera in scene"
                                  : "Press Play to run the game";
        ImVec2 textSz = ImGui::CalcTextSize(msg);
        ImGui::SetCursorPos({
            (size.x - textSz.x) * 0.5f,
            (size.y - textSz.y) * 0.5f
        });
        ImGui::TextDisabled("%s", msg);
    }

    ImGui::End();
}
