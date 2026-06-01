#include "ContentPanel.h"
#include <imgui.h>

void ContentPanel::OnImGuiRender() {
    ImGui::Begin("Content Browser");
    ImGui::Text("(empty)");
    ImGui::End();
}
