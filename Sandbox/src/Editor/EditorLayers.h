#pragma once
#include <imgui.h>
#include "EditorContext.h"
#include "Scene/Scene.h"
#include "Panels/HierarchyPanel.h"
#include "Panels/ViewportPanel.h"
#include "Panels/InspectorPanel.h"
#include "Panels/ContentPanel.h"

class EditorLayer {
public:
    explicit EditorLayer(Scene* scene);
    void SetupDockspace();
    void OnImGuiRender();
    void SetViewportTexture(uint32_t textureID);
    bool IsViewportActive() const { return m_Viewport.IsActive(); }

private:
    bool           m_LayoutInitialized = false;
    HierarchyPanel m_Hierarchy;
    ViewportPanel  m_Viewport;
    InspectorPanel m_Inspector;
    ContentPanel   m_Content;
    EditorContext  m_Context;
};
