#include "GameViewportPanel.h"
#include <imgui.h>

void GameViewportPanel::OnImGuiRender()
{
    ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, {0, 0});
    ImGui::Begin("Game");
    ImGui::PopStyleVar();

    m_IsActive = ImGui::IsWindowFocused();

    ImVec2 size = ImGui::GetContentRegionAvail();
    ImVec2 pos  = ImGui::GetCursorScreenPos();

    // Publish the cursor's position within the game viewport (0..1) so the HUD
    // layer can map the OS mouse into the game-viewport UI framebuffer.
    if (m_Context) {
        ImVec2 m = ImGui::GetMousePos();
        m_Context->gameViewportMouseNorm = {
            size.x > 0.0f ? (m.x - pos.x) / size.x : -1.0f,
            size.y > 0.0f ? (m.y - pos.y) / size.y : -1.0f
        };
    }

    bool playing = m_Context && m_Context->ActiveScene->IsPlaying();

    if (playing && m_TextureID != 0)
    {
        ImGui::Image((ImTextureID)(uintptr_t)m_TextureID, size, {0, 1}, {1, 0});

        if (!m_IsActive)
        {
            const char* hint = "Click to focus";
            ImVec2 textSz = ImGui::CalcTextSize(hint);
            ImGui::SetCursorPos({ (size.x - textSz.x) * 0.5f, size.y - textSz.y - 8.0f });
            ImGui::TextDisabled("%s", hint);
        }
    }
    else
    {
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
