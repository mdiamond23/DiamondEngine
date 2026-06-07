#include "ViewportPanel.h"
#include "../Command.h"
#include <imgui.h>
#define GLM_ENABLE_EXPERIMENTAL
#include <glm/glm.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <glm/gtx/matrix_decompose.hpp>
#include <algorithm>
#include <cfloat>
#include "Scene/Components.h"
#include "Scene/Scene.h"

void ViewportPanel::OnImGuiRender() {
    ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, {0, 0});
    ImGui::Begin("Viewport");

    ImVec2 vpSize = ImGui::GetContentRegionAvail();
    ImVec2 vpPos  = ImGui::GetCursorScreenPos();

    if (m_TextureID != 0) {
        ImGui::Image(
            (ImTextureID)(intptr_t)m_TextureID,
            vpSize,
            {0, 1}, {1, 0}
        );
    }

    if (ImGui::IsWindowHovered() && !m_IsViewportActive) {
        if (ImGui::IsKeyPressed(ImGuiKey_W)) m_GizmoOp = ImGuizmo::TRANSLATE;
        if (ImGui::IsKeyPressed(ImGuiKey_E)) m_GizmoOp = ImGuizmo::ROTATE;
        if (ImGui::IsKeyPressed(ImGuiKey_R)) m_GizmoOp = ImGuizmo::SCALE;
    }

    // Left-click picking — ray vs world-space AABBs.
    if (m_Context
        && m_Context->ActiveScene
        && !m_IsViewportActive
        && !ImGuizmo::IsOver()
        && vpSize.x > 0 && vpSize.y > 0
        && ImGui::IsWindowHovered()
        && ImGui::IsMouseClicked(ImGuiMouseButton_Left))
    {
        ImVec2 mouse = ImGui::GetMousePos();
        float mx = mouse.x - vpPos.x;
        float my = mouse.y - vpPos.y;

        if (mx >= 0 && my >= 0 && mx < vpSize.x && my < vpSize.y) {
            float ndcX =  (mx / vpSize.x) * 2.0f - 1.0f;
            float ndcY = -(my / vpSize.y) * 2.0f + 1.0f;

            glm::mat4 invVP = glm::inverse(m_Context->projMatrix * m_Context->viewMatrix);
            glm::vec4 near4 = invVP * glm::vec4(ndcX, ndcY, -1.0f, 1.0f);
            glm::vec4 far4  = invVP * glm::vec4(ndcX, ndcY,  1.0f, 1.0f);
            glm::vec3 rayOri = glm::vec3(near4) / near4.w;
            glm::vec3 rayDir = glm::normalize(glm::vec3(far4) / far4.w - rayOri);

            float        minT   = FLT_MAX;
            entt::entity picked = entt::null;

            auto& reg = m_Context->ActiveScene->GetRegistry();
            auto& ts  = m_Context->ActiveScene->GetTransformSystem();

            for (auto [e, mc] : reg.view<MeshComponent>().each()) {
                if (!mc.mesh || !mc.visible) continue;

                Diamond::AABB world = mc.localBounds.Transform(ts.GetWorldMatrix(e));
                glm::vec3 invD = 1.0f / rayDir;
                glm::vec3 t0   = (world.min - rayOri) * invD;
                glm::vec3 t1   = (world.max - rayOri) * invD;

                float tEnter = std::max(std::max(std::min(t0.x, t1.x),
                                                  std::min(t0.y, t1.y)),
                                                  std::min(t0.z, t1.z));
                float tExit  = std::min(std::min(std::max(t0.x, t1.x),
                                                  std::max(t0.y, t1.y)),
                                                  std::max(t0.z, t1.z));

                if (tExit >= std::max(tEnter, 0.0f) && tEnter < minT) {
                    minT   = tEnter;
                    picked = e;
                }
            }

            if (picked != entt::null) {
                if (ImGui::GetIO().KeyCtrl)
                    m_Context->ToggleSelect(picked);
                else
                    m_Context->SelectOnly(picked);
            } else if (!ImGui::GetIO().KeyCtrl) {
                m_Context->ClearSelection();
            }
        }
    }

    // Transform gizmo — only shown when exactly one entity is selected.
    if (m_Context && m_Context->selectedEntities.size() == 1 && vpSize.x > 0 && vpSize.y > 0) {
        auto& reg = m_Context->ActiveScene->GetRegistry();
        auto& ts  = m_Context->ActiveScene->GetTransformSystem();
        entt::entity sel = m_Context->PrimarySelection();

        if (reg.all_of<TransformComponent>(sel)) {
            ImGuizmo::SetOrthographic(false);
            ImGuizmo::SetDrawlist();
            ImGuizmo::SetRect(vpPos.x, vpPos.y, vpSize.x, vpSize.y);

            glm::mat4 worldModel = ts.GetWorldMatrix(sel);

            bool gizmoUsing = ImGuizmo::IsUsing();

            // Snapshot the full transform on the first frame the gizmo becomes active.
            if (gizmoUsing && !m_GizmoWasUsing) {
                auto& tc      = reg.get<TransformComponent>(sel);
                m_GizmoOldPos   = tc.position;
                m_GizmoOldRot   = tc.rotation;
                m_GizmoOldEuler = tc.eulerDegrees;
                m_GizmoOldScale = tc.scale;
            }

            if (ImGuizmo::Manipulate(
                    glm::value_ptr(m_Context->viewMatrix),
                    glm::value_ptr(m_Context->projMatrix),
                    m_GizmoOp,
                    ImGuizmo::LOCAL,
                    glm::value_ptr(worldModel)))
            {
                glm::mat4 parentWorld(1.0f);
                if (reg.all_of<HierarchyComponent>(sel)) {
                    entt::entity parent = reg.get<HierarchyComponent>(sel).parent;
                    if (parent != entt::null)
                        parentWorld = ts.GetWorldMatrix(parent);
                }
                glm::mat4 localModel = glm::inverse(parentWorld) * worldModel;

                glm::vec3 pos, scale, skew;
                glm::vec4 persp;
                glm::quat rot;
                glm::decompose(localModel, scale, rot, pos, skew, persp);

                auto& tc        = reg.get<TransformComponent>(sel);
                tc.position     = pos;
                tc.rotation     = rot;
                tc.eulerDegrees = glm::degrees(glm::eulerAngles(rot));
                tc.scale        = scale;
            }

            // Submit one command when the drag ends.
            if (!gizmoUsing && m_GizmoWasUsing) {
                auto& tc    = reg.get<TransformComponent>(sel);
                Scene* scene = m_Context->ActiveScene;

                struct TransformState {
                    glm::vec3 pos, euler, scale;
                    glm::quat rot;
                };
                TransformState oldState { m_GizmoOldPos, m_GizmoOldEuler, m_GizmoOldScale, m_GizmoOldRot };
                TransformState newState { tc.position,   tc.eulerDegrees, tc.scale,         tc.rotation   };

                const char* desc = m_GizmoOp == ImGuizmo::TRANSLATE ? "Move Entity" :
                                   m_GizmoOp == ImGuizmo::ROTATE    ? "Rotate Entity" : "Scale Entity";

                m_Context->Commands.ExecuteCommand(std::make_unique<ValueChangeCommand<TransformState>>(
                    [scene, sel](const TransformState& s) {
                        auto& t     = scene->GetRegistry().get<TransformComponent>(sel);
                        t.position     = s.pos;
                        t.rotation     = s.rot;
                        t.eulerDegrees = s.euler;
                        t.scale        = s.scale;
                    },
                    oldState, newState, desc));
            }

            m_GizmoWasUsing = gizmoUsing;
        }
    }

    if (ImGui::IsWindowHovered() && ImGui::IsMouseClicked(ImGuiMouseButton_Right))
        m_IsViewportActive = true;
    if (!ImGui::IsMouseDown(ImGuiMouseButton_Right))
        m_IsViewportActive = false;

    ImGui::End();
    ImGui::PopStyleVar();
}
