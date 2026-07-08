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
#include "Animation/IKComponent.h"
#include "Animation/AnimationComponents.h"

void ViewportPanel::OnImGuiRender() {
    if (!m_Open) return;
    ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, {0, 0});
    ImGui::Begin("Viewport", &m_Open);

    ImVec2 vpSize = ImGui::GetContentRegionAvail();
    ImVec2 vpPos  = ImGui::GetCursorScreenPos();

    // Publish the cursor's position within the viewport (0..1) so the UI layer can
    // map the OS mouse into the full-window UI framebuffer for hit-testing.
    if (m_Context) {
        ImVec2 m = ImGui::GetMousePos();
        m_Context->viewportMouseNorm = {
            vpSize.x > 0.0f ? (m.x - vpPos.x) / vpSize.x : -1.0f,
            vpSize.y > 0.0f ? (m.y - vpPos.y) / vpSize.y : -1.0f
        };
    }

    if (m_TextureID != 0) {
        ImGui::Image(
            m_TextureID,
            vpSize,
            m_FlipY ? ImVec2{0, 1} : ImVec2{0, 0},
            m_FlipY ? ImVec2{1, 0} : ImVec2{1, 1}
        );
    }

    // Debug-draw toggle overlay (top-left). One switch for collider wireframes,
    // ragdoll bodies, and IK chain/target visualization + the IK target gizmo.
    bool overlayHovered = false;
    if (m_Context) {
        ImGui::SetCursorScreenPos({ vpPos.x + 8.0f, vpPos.y + 8.0f });
        ImGui::PushStyleColor(ImGuiCol_FrameBg, ImVec4(0.0f, 0.0f, 0.0f, 0.45f));
        ImGui::Checkbox("Debug Draw", &m_Context->showDebugDraw);
        overlayHovered = ImGui::IsItemHovered();
        ImGui::Checkbox("Show UI", &m_Context->showUI);
        overlayHovered |= ImGui::IsItemHovered();
        ImGui::PopStyleColor();
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
        && !overlayHovered
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

    // IK target gizmo — a TRANSLATE handle on the active chain's world-point target.
    // Only world-point chains get a handle (entity-follow chains move with their
    // target entity; foot chains derive the target from a raycast). Gated by the
    // same Debug Draw toggle as the chain visualization.
    if (m_Context && m_Context->ActiveScene && m_Context->showDebugDraw
        && vpSize.x > 0 && vpSize.y > 0)
    {
        auto&  reg   = m_Context->ActiveScene->GetRegistry();
        Scene* scene = m_Context->ActiveScene;
        entt::entity sel = m_Context->PrimarySelection();

        IKChain* chain = nullptr;
        if (sel != entt::null && reg.valid(sel) && reg.all_of<IKComponent>(sel)) {
            auto& ik = reg.get<IKComponent>(sel);
            int ci = m_Context->activeIKChain;
            if (ci >= 0 && ci < (int)ik.chains.size()) {
                IKChain& c = ik.chains[ci];
                if (!c.isFootChain && c.targetEntity == entt::null)
                    chain = &c;
            }
        }

        if (chain) {
            const int ci = m_Context->activeIKChain;
            ImGuizmo::SetOrthographic(false);
            ImGuizmo::SetDrawlist();
            ImGuizmo::SetRect(vpPos.x, vpPos.y, vpSize.x, vpSize.y);
            ImGuizmo::PushID(1);   // distinct from the entity transform gizmo

            bool ikUsing = ImGuizmo::IsUsing();
            if (ikUsing && !m_IKGizmoWasUsing)
                m_IKGizmoOldTarget = chain->targetWorldPos;

            glm::mat4 m = glm::translate(glm::mat4(1.0f), chain->targetWorldPos + chain->targetOffset);
            if (ImGuizmo::Manipulate(
                    glm::value_ptr(m_Context->viewMatrix),
                    glm::value_ptr(m_Context->projMatrix),
                    ImGuizmo::TRANSLATE,
                    ImGuizmo::WORLD,
                    glm::value_ptr(m)))
            {
                chain->targetWorldPos = glm::vec3(m[3]) - chain->targetOffset;
            }

            // Record one undoable command when the drag ends.
            if (!ikUsing && m_IKGizmoWasUsing) {
                glm::vec3 oldT = m_IKGizmoOldTarget;
                glm::vec3 newT = chain->targetWorldPos;
                if (oldT != newT) {
                    auto setTarget = [scene, sel, ci](const glm::vec3& v) {
                        auto& r = scene->GetRegistry();
                        if (!r.valid(sel) || !r.all_of<IKComponent>(sel)) return;
                        auto& ik = r.get<IKComponent>(sel);
                        if (ci >= 0 && ci < (int)ik.chains.size())
                            ik.chains[ci].targetWorldPos = v;
                    };
                    m_Context->Commands.RecordCommand(std::make_unique<FunctionCommand>(
                        [setTarget, newT]() { setTarget(newT); },
                        [setTarget, oldT]() { setTarget(oldT); },
                        "Move IK Target"));
                }
            }

            ImGuizmo::PopID();
            m_IKGizmoWasUsing = ikUsing;
        } else {
            m_IKGizmoWasUsing = false;
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
