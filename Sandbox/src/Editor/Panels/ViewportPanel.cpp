#include "ViewportPanel.h"
#include <imgui.h>
#define GLM_ENABLE_EXPERIMENTAL
#include <glm/glm.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <glm/gtx/matrix_decompose.hpp>
#include <algorithm>
#include <cfloat>
#include "Scene/Components.h"

void ViewportPanel::OnImGuiRender() {
    ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, {0, 0});
    ImGui::Begin("Viewport");

    // Record where the image starts and how large it is — needed for NDC conversion and gizmo rect.
    ImVec2 vpSize = ImGui::GetContentRegionAvail();
    ImVec2 vpPos  = ImGui::GetCursorScreenPos();

    if (m_TextureID != 0) {
        ImGui::Image(
            (ImTextureID)(intptr_t)m_TextureID,
            vpSize,
            {0, 1}, {1, 0}  // flip Y: OpenGL origin is bottom-left
        );
    }

    // W / E / R switch gizmo mode (only when not in free-look camera mode).
    if (ImGui::IsWindowHovered() && !m_IsViewportActive) {
        if (ImGui::IsKeyPressed(ImGuiKey_W)) m_GizmoOp = ImGuizmo::TRANSLATE;
        if (ImGui::IsKeyPressed(ImGuiKey_E)) m_GizmoOp = ImGuizmo::ROTATE;
        if (ImGui::IsKeyPressed(ImGuiKey_R)) m_GizmoOp = ImGuizmo::SCALE;
    }

    // Left-click picking — skip if the cursor is over a gizmo handle.
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
            for (auto [e, tc, mc] : reg.view<TransformComponent, MeshComponent>().each()) {
                if (!mc.mesh || !mc.visible) continue;

                Diamond::AABB world = mc.localBounds.Transform(tc.GetLocalMatrix());
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

            m_Context->selectedEntity = picked;
        }
    }

    // Transform gizmo — drawn over the viewport image using ImGui's draw list.
    if (m_Context && m_Context->HasSelection() && vpSize.x > 0 && vpSize.y > 0) {
        auto& reg = m_Context->ActiveScene->GetRegistry();
        if (reg.all_of<TransformComponent>(m_Context->selectedEntity)) {
            auto& tc = reg.get<TransformComponent>(m_Context->selectedEntity);

            ImGuizmo::SetOrthographic(false);
            ImGuizmo::SetDrawlist();
            ImGuizmo::SetRect(vpPos.x, vpPos.y, vpSize.x, vpSize.y);

            glm::mat4 model = tc.GetLocalMatrix();

            if (ImGuizmo::Manipulate(
                    glm::value_ptr(m_Context->viewMatrix),
                    glm::value_ptr(m_Context->projMatrix),
                    m_GizmoOp,
                    ImGuizmo::LOCAL,
                    glm::value_ptr(model)))
            {
                glm::vec3 pos, scale, skew;
                glm::vec4 persp;
                glm::quat rot;
                glm::decompose(model, scale, rot, pos, skew, persp);

                tc.position     = pos;
                tc.rotation     = rot;
                tc.eulerDegrees = glm::degrees(glm::eulerAngles(rot));
                tc.scale        = scale;
            }
        }
    }

    // Right-click activates free-look camera mode; releasing deactivates it.
    if (ImGui::IsWindowHovered() && ImGui::IsMouseClicked(ImGuiMouseButton_Right))
        m_IsViewportActive = true;
    if (!ImGui::IsMouseDown(ImGuiMouseButton_Right))
        m_IsViewportActive = false;

    ImGui::End();
    ImGui::PopStyleVar();
}
