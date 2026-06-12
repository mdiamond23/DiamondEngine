#include "ViewportPanel.h"
#include "../Command.h"
#include <imgui.h>
#define GLM_ENABLE_EXPERIMENTAL
#include <glm/glm.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <glm/gtx/matrix_decompose.hpp>
#include <algorithm>
#include <cfloat>
#include <cmath>
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

        // F — frame selected entity
        if (ImGui::IsKeyPressed(ImGuiKey_F)
            && m_Context && m_Context->HasSelection() && m_Context->EditorCamera)
        {
            auto& reg = m_Context->ActiveScene->GetRegistry();
            auto& ts  = m_Context->ActiveScene->GetTransformSystem();
            entt::entity sel = m_Context->PrimarySelection();

            glm::vec3 center(0.0f);
            float radius = 1.0f;

            if (reg.all_of<MeshComponent>(sel)) {
                auto& mc = reg.get<MeshComponent>(sel);
                Diamond::AABB world = mc.localBounds.Transform(ts.GetWorldMatrix(sel));
                center = (world.min + world.max) * 0.5f;
                radius = glm::length(world.max - world.min) * 0.5f;
                if (radius < 0.01f) radius = 1.0f;
            } else {
                center = glm::vec3(ts.GetWorldMatrix(sel)[3]);
            }

            Diamond::Camera* cam = m_Context->EditorCamera;
            float fovRad  = glm::radians(cam->Zoom);
            float distance = std::max(radius / std::tan(fovRad * 0.5f) * 1.2f, radius * 2.0f);

            // Smooth swoop: UpdateOrbit glides toward the new pivot/distance.
            cam->Focus(center, distance);
        }
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

    // Transform gizmo — drives the whole selection. Entities with a selected
    // ancestor are filtered out: the ancestor's matrix already carries its
    // subtree, so including both would double-transform the descendant.
    if (m_Context && m_Context->ActiveScene && m_Context->HasSelection()
        && vpSize.x > 0 && vpSize.y > 0)
    {
        auto&  reg   = m_Context->ActiveScene->GetRegistry();
        auto&  ts    = m_Context->ActiveScene->GetTransformSystem();
        Scene* scene = m_Context->ActiveScene;

        std::vector<entt::entity> targets;
        for (auto e : m_Context->selectedEntities) {
            if (!reg.valid(e) || !reg.all_of<TransformComponent>(e)) continue;
            bool covered = false;
            for (auto other : m_Context->selectedEntities) {
                if (other != e && reg.valid(other) && scene->IsAncestorOf(other, e)) {
                    covered = true;
                    break;
                }
            }
            if (!covered) targets.push_back(e);
        }

        // Writes a world matrix back into an entity's local TransformComponent.
        auto applyWorld = [&](entt::entity e, const glm::mat4& world) {
            glm::mat4 parentWorld(1.0f);
            if (reg.all_of<HierarchyComponent>(e)) {
                entt::entity parent = reg.get<HierarchyComponent>(e).parent;
                if (parent != entt::null)
                    parentWorld = ts.GetWorldMatrix(parent);
            }
            glm::mat4 localModel = glm::inverse(parentWorld) * world;

            glm::vec3 pos, scale, skew;
            glm::vec4 persp;
            glm::quat rot;
            glm::decompose(localModel, scale, rot, pos, skew, persp);

            auto& tc        = reg.get<TransformComponent>(e);
            tc.position     = pos;
            tc.rotation     = rot;
            tc.eulerDegrees = glm::degrees(glm::eulerAngles(rot));
            tc.scale        = scale;
        };

        if (!targets.empty()) {
            ImGuizmo::SetOrthographic(false);
            ImGuizmo::SetDrawlist();
            ImGuizmo::SetRect(vpPos.x, vpPos.y, vpSize.x, vpSize.y);

            bool gizmoUsing = ImGuizmo::IsUsing();

            // Snapshot every target on the first frame the gizmo is grabbed.
            if (gizmoUsing && !m_GizmoWasUsing) {
                m_GizmoOldXf.clear();
                for (auto e : targets) {
                    auto& tc = reg.get<TransformComponent>(e);
                    m_GizmoOldXf.push_back({e, {tc.position, tc.eulerDegrees, tc.scale, tc.rotation}});
                }
            }

            if (targets.size() == 1) {
                glm::mat4 worldModel = ts.GetWorldMatrix(targets[0]);
                if (ImGuizmo::Manipulate(
                        glm::value_ptr(m_Context->viewMatrix),
                        glm::value_ptr(m_Context->projMatrix),
                        m_GizmoOp,
                        ImGuizmo::LOCAL,
                        glm::value_ptr(worldModel)))
                {
                    applyWorld(targets[0], worldModel);
                }
            } else {
                // Anchor a world-aligned gizmo at the selection centroid while
                // idle. During the drag, replay the gizmo's per-frame delta on
                // every target's world matrix — translation moves them all,
                // rotate/scale orbit them around the shared anchor.
                if (!gizmoUsing) {
                    glm::vec3 centroid(0.0f);
                    for (auto e : targets)
                        centroid += glm::vec3(ts.GetWorldMatrix(e)[3]);
                    m_MultiGizmoMatrix = glm::translate(glm::mat4(1.0f), centroid / (float)targets.size());
                }

                glm::mat4 before = m_MultiGizmoMatrix;
                if (ImGuizmo::Manipulate(
                        glm::value_ptr(m_Context->viewMatrix),
                        glm::value_ptr(m_Context->projMatrix),
                        m_GizmoOp,
                        ImGuizmo::WORLD,
                        glm::value_ptr(m_MultiGizmoMatrix)))
                {
                    glm::mat4 delta = m_MultiGizmoMatrix * glm::inverse(before);
                    for (auto e : targets)
                        applyWorld(e, delta * ts.GetWorldMatrix(e));
                }
            }

            // Submit one command covering the whole selection when the drag ends.
            if (!gizmoUsing && m_GizmoWasUsing && !m_GizmoOldXf.empty()) {
                std::vector<std::pair<entt::entity, XfState>> oldStates, newStates;
                for (auto& [e, s] : m_GizmoOldXf) {
                    if (!reg.valid(e) || !reg.all_of<TransformComponent>(e)) continue;
                    auto& tc = reg.get<TransformComponent>(e);
                    oldStates.push_back({e, s});
                    newStates.push_back({e, {tc.position, tc.eulerDegrees, tc.scale, tc.rotation}});
                }
                m_GizmoOldXf.clear();

                auto apply = [scene](const std::vector<std::pair<entt::entity, XfState>>& states) {
                    auto& r = scene->GetRegistry();
                    for (auto& [e, s] : states) {
                        if (!r.valid(e) || !r.all_of<TransformComponent>(e)) continue;
                        auto& t        = r.get<TransformComponent>(e);
                        t.position     = s.pos;
                        t.rotation     = s.rot;
                        t.eulerDegrees = s.euler;
                        t.scale        = s.scale;
                    }
                };

                bool plural = oldStates.size() > 1;
                const char* desc = m_GizmoOp == ImGuizmo::TRANSLATE ? (plural ? "Move Entities"   : "Move Entity") :
                                   m_GizmoOp == ImGuizmo::ROTATE    ? (plural ? "Rotate Entities" : "Rotate Entity")
                                                                    : (plural ? "Scale Entities"  : "Scale Entity");

                m_Context->Commands.RecordCommand(std::make_unique<FunctionCommand>(
                    [apply, newStates]() { apply(newStates); },
                    [apply, oldStates]() { apply(oldStates); },
                    desc));
            }

            m_GizmoWasUsing = gizmoUsing;
        }
    }

    if (ImGui::IsWindowHovered() && ImGui::IsMouseClicked(ImGuiMouseButton_Right))
        m_IsViewportActive = true;
    if (!ImGui::IsMouseDown(ImGuiMouseButton_Right))
        m_IsViewportActive = false;

    // Middle-mouse orbit (Shift = pan), scroll dolly — suspended while RMB fly
    // mode owns the camera (main.cpp re-syncs orbit targets each fly frame).
    if (m_Context && m_Context->EditorCamera) {
        Diamond::Camera* cam = m_Context->EditorCamera;
        ImGuiIO& io = ImGui::GetIO();

        if (ImGui::IsWindowHovered() && ImGui::IsMouseClicked(ImGuiMouseButton_Middle))
            m_OrbitActive = true;
        if (!ImGui::IsMouseDown(ImGuiMouseButton_Middle))
            m_OrbitActive = false;

        if (!m_IsViewportActive) {
            if (m_OrbitActive) {
                if (io.KeyShift)
                    cam->Pan(io.MouseDelta.x, io.MouseDelta.y, vpSize.y);
                else
                    cam->Orbit(io.MouseDelta.x, io.MouseDelta.y);
            }
            if (ImGui::IsWindowHovered() && io.MouseWheel != 0.0f)
                cam->Dolly(io.MouseWheel);
            cam->UpdateOrbit(io.DeltaTime);
        }
    }

    ImGui::End();
    ImGui::PopStyleVar();
}
