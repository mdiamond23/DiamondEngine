#include "HierarchyPanel.h"
#include <imgui.h>

void HierarchyPanel::OnImGuiRender() {
    ImGui::Begin("Hierarchy");

    if (m_Context && m_Context->ActiveScene)
    {
        if (ImGui::Button("+ Add Entity"))
            m_Context->ActiveScene->CreateEntity("Empty Entity");

        ImGui::Separator();

        entt::entity toDelete = entt::null;

        for (auto& [entity, name] : m_Context->ActiveScene->GetEntityNames())
        {
            bool selected = (m_Context->selectedEntity == entity);
            if (ImGui::Selectable(name.c_str(), selected))
                m_Context->selectedEntity = entity;

            if (ImGui::BeginPopupContextItem())
            {
                if (ImGui::MenuItem("Delete Entity"))
                    toDelete = entity;
                ImGui::EndPopup();
            }
        }

        if (toDelete != entt::null)
        {
            if (m_Context->selectedEntity == toDelete)
                m_Context->selectedEntity = entt::null;
            m_Context->ActiveScene->DestroyEntity(toDelete);
        }
    }

    ImGui::End();
}
