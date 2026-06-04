#pragma once
#include "Panels.h"
#include "../EditorContext.h"
#include <imgui.h>
#include <ImGuizmo.h>
#include <cstdint>

class ViewportPanel : public Panel {
public:
    void OnImGuiRender() override;
    void SetTexture(uint32_t textureID) { m_TextureID = textureID; }
    void SetContext(EditorContext* ctx)  { m_Context = ctx; }
    bool IsActive() const { return m_IsViewportActive; }

private:
    uint32_t           m_TextureID        = 0;
    bool               m_IsViewportActive = false;
    EditorContext*      m_Context         = nullptr;
    ImGuizmo::OPERATION m_GizmoOp        = ImGuizmo::TRANSLATE;
};
