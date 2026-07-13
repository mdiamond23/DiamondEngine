#pragma once
#include "Panels.h"
#include <imgui.h>
#include <functional>

// Renderer settings (exposure, FXAA, ...) as a regular dockable panel. The
// controls are backend-specific, so the backend supplies the widget body
// (EditorBackend::DrawRendererSettings, wired through EditorLayer in main.cpp);
// this panel owns the window chrome — View-menu toggle, docking, close button —
// that the old backend-drawn ImGui::Begin("Renderer") window lacked.
class RendererPanel : public Panel {
public:
    void SetDrawCallback(std::function<void()> fn) { m_Draw = std::move(fn); }

    void OnImGuiRender() override {
        if (!m_Open) return;
        if (!ImGui::Begin("Renderer", &m_Open)) { ImGui::End(); return; }
        if (m_Draw) m_Draw();
        ImGui::End();
    }
    const char* GetName() const override { return "Renderer"; }

private:
    std::function<void()> m_Draw;
};
