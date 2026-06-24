#pragma once
#include <imgui.h>
#include <string>
#include "EditorContext.h"
#include "Scene/Scene.h"
#include "Panels/HierarchyPanel.h"
#include "Panels/ViewportPanel.h"
#include "Panels/GameViewportPanel.h"
#include "Panels/InspectorPanel.h"
#include "Panels/ContentPanel.h"
#include "Panels/ConsolePanel.h"
#include "Panels/AnimatorPanel.h"

class EditorLayer {
public:
    explicit EditorLayer(Scene* scene, ImFont* iconFont = nullptr);
    void SetupDockspace();
    void OnImGuiRender();
    void SetViewportTexture(uint32_t textureID);
    void SetGameViewportTexture(uint32_t textureID);
    void SetEditorCamera(Diamond::Camera* cam) { m_Context.EditorCamera = cam; }
    void UpdateCamera(const glm::mat4& view, const glm::mat4& proj, const glm::vec3& camPos);
    EditorContext& GetContext() { return m_Context; }
    bool IsViewportActive() const     { return m_Viewport.IsActive(); }
    bool IsGameViewportActive() const { return m_GameViewport.IsActive(); }
    void NotifyDroppedFiles(int count, const char** paths) { m_Content.QueueDroppedFiles(count, paths); }

private:
    void DrawMenuBar();
    void DrawToolbar();

    static std::string OpenFileDialog();
    static std::string SaveFileDialog();

    void DrawNewScriptDialog();

    bool              m_LayoutInitialized   = false;
    bool              m_OpenNewScriptDialog = false;
    std::string       m_SceneSnapshot;
    char              m_NewScriptNameBuf[128]{};
    std::string       m_NewScriptError;
    HierarchyPanel    m_Hierarchy;
    ViewportPanel     m_Viewport;
    GameViewportPanel m_GameViewport;
    InspectorPanel    m_Inspector;
    ContentPanel      m_Content;
    ConsolePanel      m_Console;
    AnimatorPanel     m_Animator;
    EditorContext     m_Context;
};
