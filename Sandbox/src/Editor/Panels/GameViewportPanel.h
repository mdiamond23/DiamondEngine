#pragma once
#include "Panels.h"
#include "../EditorContext.h"
#include <imgui.h>

class GameViewportPanel : public Panel {
public:
    void SetContext(EditorContext* ctx)  { m_Context   = ctx; }
    // Opaque backend image handle; flipY = true for GL FBO textures (bottom-up),
    // false for Vulkan offscreen targets. Pass 0 to show the placeholder text.
    void SetTexture(ImTextureID id, bool flipY) { m_TextureID = id; m_FlipY = flipY; }
    void OnImGuiRender() override;
    bool IsActive() const { return m_IsActive; }

private:
    EditorContext* m_Context   = nullptr;
    ImTextureID    m_TextureID = 0;
    bool           m_FlipY     = false;
    bool           m_IsActive  = false;
};
