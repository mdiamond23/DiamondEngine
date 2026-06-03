#include "InspectorPanel.h"
#include <imgui.h>
#include <glm/gtc/type_ptr.hpp>
#include "Scene/Components.h"

void InspectorPanel::OnImGuiRender() {
    ImGui::Begin("Inspector");
    
    if (!m_Context || !m_Context->HasSelection())
    {
        ImGui::Text("(nothing selected)");
        ImGui::End();
        return;
    }

    entt::entity entity = m_Context->selectedEntity;
    auto &       registry = m_Context->ActiveScene->GetRegistry();

    // Entity Name 
    const std::string& name = m_Context->ActiveScene->GetEntityName(entity);
    ImGui::Text("%s", name.c_str());
    ImGui::Separator();

    // Transform,
    if (registry.all_of<TransformComponent>(entity))
    {
        auto& tc = registry.get<TransformComponent>(entity);

        ImGui::Text("Transform");
        ImGui::DragFloat3("Position", glm::value_ptr(tc.position), 0.1f);

        glm::vec3 euler = tc.GetEulerAngles();
        if (ImGui::DragFloat3("Rotation", glm::value_ptr(euler), 0.5f))
            tc.rotation = glm::quat(glm::radians(euler));

        ImGui::DragFloat3("Scale", glm::value_ptr(tc.scale), 0.1f, 0.001f);
    }

    // Mesh Component
    if (registry.all_of<MeshComponent>(entity))
    {
        ImGui::Separator();
        auto& mc = registry.get<MeshComponent>(entity);

        ImGui::Text("Mesh Renderer");

        ImGui::Checkbox("Visible",          &mc.visible);
        ImGui::Checkbox("Cast Shadows",     &mc.castsShadow);
        ImGui::Checkbox("Receive Shadows",  &mc.receivesShadow);

        if (mc.material)
        {
            ImGui::Spacing();
            ImGui::Text("Material");
            ImGui::DragFloat("UV Scale",          &mc.material->UVScale,          0.01f, 0.01f, 64.0f);
            ImGui::DragFloat("Emissive Strength", &mc.material->EmissiveStrength, 0.01f, 0.0f,  100.0f);
        }
    }

    ImGui::End();
}
