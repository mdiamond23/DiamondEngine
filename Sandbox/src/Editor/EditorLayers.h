#pragma once
#include <imgui.h>
#include <string>
#include "EditorContext.h"
#include "Scene/Scene.h"
#include "Panels/HierarchyPanel.h"
#include "Panels/ViewportPanel.h"
#include "Panels/InspectorPanel.h"
#include "Panels/ContentPanel.h"
#include "Panels/ConsolePanel.h"

class EditorLayer {
public:
    explicit EditorLayer(Scene* scene, ImFont* iconFont = nullptr);
    void SetupDockspace();
    void OnImGuiRender();
    void SetViewportTexture(uint32_t textureID);
    void UpdateCamera(const glm::mat4& view, const glm::mat4& proj, const glm::vec3& camPos);
    bool IsViewportActive() const { return m_Viewport.IsActive(); }
    void NotifyDroppedFiles(int count, const char** paths) { m_Content.QueueDroppedFiles(count, paths); }

private:
    void DrawMenuBar();
    void DrawToolbar();

    static std::string OpenFileDialog();
    static std::string SaveFileDialog();

    bool           m_LayoutInitialized = false;
    HierarchyPanel m_Hierarchy;
    ViewportPanel  m_Viewport;
    InspectorPanel m_Inspector;
    ContentPanel   m_Content;
    ConsolePanel   m_Console;
    EditorContext  m_Context;
};
