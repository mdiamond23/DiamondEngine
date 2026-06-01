#include "InspectorPanel.h"
#include <imgui.h>

void InspectorPanel::OnImGuiRender() {
    ImGui::Begin("Inspector");
    ImGui::Text("(nothing selected)");
    ImGui::End();
}
