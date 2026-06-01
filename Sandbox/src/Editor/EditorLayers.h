#pragma once
#include <imgui.h>
#include "Panels/HierarchyPanel.h"
#include "Panels/ViewportPanel.h"
#include "Panels/InspectorPanel.h"
#include "Panels/ContentPanel.h"

class EditorLayer {
public:
    void SetupDockspace();
    void OnImGuiRender();
    void SetViewportTexture(uint32_t textureID);

private:
    bool           m_LayoutInitialized = false;
    HierarchyPanel m_Hierarchy;
    ViewportPanel  m_Viewport;
    InspectorPanel m_Inspector;
    ContentPanel   m_Content;
};
