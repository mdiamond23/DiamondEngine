#pragma once
#include "Panels.h"
#include "../EditorContext.h"

class GameViewportPanel : public Panel {
public:
    void SetContext(EditorContext* ctx)  { m_Context   = ctx; }
    void SetTexture(uint32_t id)         { m_TextureID = id;  }
    void OnImGuiRender() override;
    bool IsActive() const { return m_IsActive; }

private:
    EditorContext* m_Context   = nullptr;
    uint32_t       m_TextureID = 0;
    bool           m_IsActive  = false;
};
