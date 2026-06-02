#include "ViewportPanel.h"
#include <imgui.h>

void ViewportPanel::OnImGuiRender() {
    ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, {0, 0});
    ImGui::Begin("Viewport");

    if (m_TextureID != 0) {
        ImVec2 size = ImGui::GetContentRegionAvail();
        ImGui::Image(
            (ImTextureID)(intptr_t)m_TextureID,
            size,
            {0, 1}, {1, 0}  // flip Y: OpenGL origin is bottom-left
        );
    }

    // Activate camera mode on right-click while hovering the viewport
    if (ImGui::IsWindowHovered() && ImGui::IsMouseClicked(ImGuiMouseButton_Right))
        m_IsViewportActive = true;
    // Deactivate as soon as right button is released (fires even if cursor left the window)
    if (!ImGui::IsMouseDown(ImGuiMouseButton_Right))
        m_IsViewportActive = false;

    ImGui::End();
    ImGui::PopStyleVar();
}
