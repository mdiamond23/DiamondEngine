#include "HierarchyPanel.h"
#include <imgui.h>
#include <cstring>

void HierarchyPanel::OnImGuiRender() {
    ImGui::Begin("Hierarchy");

    if (m_Context && m_Context->ActiveScene)
    {
        if (ImGui::Button("+ Add Entity"))
            m_Context->ActiveScene->CreateEntity("Empty Entity");

        ImGui::Separator();

        entt::entity toDelete = entt::null;
        Scene* scene = m_Context->ActiveScene;

        for (auto& [entity, name] : scene->GetEntityNames())
        {
            bool selected = (m_Context->selectedEntity == entity);

            if (m_RenamingEntity == entity)
            {
                // Auto-focus the input on the first frame it appears
                if (!m_RenameFocusSet) {
                    ImGui::SetKeyboardFocusHere();
                    m_RenameFocusSet = true;
                }

                ImGui::SetNextItemWidth(-1.0f);
                if (ImGui::InputText("##rename", m_RenameBuffer, sizeof(m_RenameBuffer),
                                     ImGuiInputTextFlags_EnterReturnsTrue))
                {
                    // Commit on Enter
                    if (m_RenameBuffer[0] != '\0')
                        scene->SetEntityName(entity, m_RenameBuffer);
                    m_RenamingEntity = entt::null;
                }
                else if (!ImGui::IsItemActive() && ImGui::IsItemDeactivated())
                {
                    // Commit on click-away (lost focus without Enter)
                    if (m_RenameBuffer[0] != '\0')
                        scene->SetEntityName(entity, m_RenameBuffer);
                    m_RenamingEntity = entt::null;
                }
                else if (ImGui::IsKeyPressed(ImGuiKey_Escape))
                {
                    // Cancel on Escape — discard changes
                    m_RenamingEntity = entt::null;
                }
            }
            else
            {
                if (ImGui::Selectable(name.c_str(), selected))
                    m_Context->selectedEntity = entity;

                if (ImGui::BeginPopupContextItem())
                {
                    if (ImGui::MenuItem("Rename"))
                    {
                        m_RenamingEntity = entity;
                        m_RenameFocusSet = false;
                        std::strncpy(m_RenameBuffer, name.c_str(), sizeof(m_RenameBuffer) - 1);
                        m_RenameBuffer[sizeof(m_RenameBuffer) - 1] = '\0';
                    }
                    if (ImGui::MenuItem("Delete"))
                        toDelete = entity;
                    ImGui::EndPopup();
                }
            }
        }

        if (toDelete != entt::null)
        {
            if (m_Context->selectedEntity == toDelete)
                m_Context->selectedEntity = entt::null;
            scene->DestroyEntity(toDelete);
        }
    }

    ImGui::End();
}
