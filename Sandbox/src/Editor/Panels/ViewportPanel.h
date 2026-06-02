#pragma once
#include "Panels.h"
#include <cstdint>

class ViewportPanel : public Panel {
public:
    void OnImGuiRender() override;
    void SetTexture(uint32_t textureID) { m_TextureID = textureID; }
    bool IsActive() const { return m_IsViewportActive; }

private:
    uint32_t m_TextureID        = 0;
    bool     m_IsViewportActive = false;
};
