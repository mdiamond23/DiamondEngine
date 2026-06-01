#include "HierarchyPanel.h"
#include <imgui.h>

void HierarchyPanel::OnImGuiRender() {
    ImGui::Begin("Hierarchy");
    
    if (m_Context && m_Context->ActiveScene)
    {
        for (auto& [entity, name] : m_Context->ActiveScene->GetEntityNames())
        {
            bool selected = (m_Context->selectedEntity == entity);
            if (ImGui::Selectable(name.c_str(), selected))
                m_Context->selectedEntity = entity;
        }
    }

    ImGui::End();
}
