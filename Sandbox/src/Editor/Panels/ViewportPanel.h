#pragma once
#include "Panels.h"
#include "../EditorContext.h"
#include <imgui.h>
#include <ImGuizmo.h>
#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>
#include <cstdint>
#include <vector>
#include <utility>

class ViewportPanel : public Panel {
public:
    void OnImGuiRender() override;
    void SetTexture(uint32_t textureID) { m_TextureID = textureID; }
    void SetContext(EditorContext* ctx)  { m_Context = ctx; }
    bool IsActive() const { return m_IsViewportActive; }

private:
    uint32_t            m_TextureID        = 0;
    bool                m_IsViewportActive = false;
    bool                m_OrbitActive      = false;  // middle-mouse orbit/pan drag
    EditorContext*       m_Context         = nullptr;
    ImGuizmo::OPERATION m_GizmoOp         = ImGuizmo::TRANSLATE;

    // Gizmo undo/redo state — snapshot of every dragged entity's transform,
    // taken when the gizmo is grabbed and submitted as one command on release.
    struct XfState { glm::vec3 pos, euler, scale; glm::quat rot; };
    bool m_GizmoWasUsing = false;
    std::vector<std::pair<entt::entity, XfState>> m_GizmoOldXf;

    // Multi-select gizmo anchor — world-aligned, at the selection centroid.
    glm::mat4 m_MultiGizmoMatrix { 1.0f };
};
