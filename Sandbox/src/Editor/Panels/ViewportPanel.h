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
    // Opaque backend image handle (GL texture id or Vulkan descriptor set, both
    // cast to ImTextureID by the caller). flipY: GL FBO textures are bottom-up
    // and need flipped UVs; Vulkan offscreen targets are top-down and must not.
    void SetTexture(ImTextureID textureID, bool flipY) { m_TextureID = textureID; m_FlipY = flipY; }
    void SetContext(EditorContext* ctx)  { m_Context = ctx; }
    bool IsActive() const { return m_IsViewportActive; }

private:
    ImTextureID         m_TextureID        = 0;
    bool                m_FlipY            = false;
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

    // IK target gizmo — translates the active chain's world-point target. Tracked
    // separately so its drag undo doesn't entangle with the entity transform gizmo.
    bool      m_IKGizmoWasUsing = false;
    glm::vec3 m_IKGizmoOldTarget { 0.0f };
};
