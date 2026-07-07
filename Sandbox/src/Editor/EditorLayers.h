#pragma once
#include <imgui.h>
#include <functional>
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
#include "Panels/ParticlePreviewPanel.h"
#include "Panels/MixerPanel.h"

class EditorLayer {
public:
    explicit EditorLayer(Scene* scene, ImFont* iconFont = nullptr);
    void SetupDockspace();
    void OnImGuiRender();
    // Opaque backend image handles for the scene/game viewport panels (a GL
    // texture id or a Vulkan descriptor set, pre-cast to ImTextureID). flipY:
    // true for GL FBO textures (bottom-up), false for Vulkan offscreen targets.
    void SetViewportTexture(ImTextureID textureID, bool flipY);
    void SetGameViewportTexture(ImTextureID textureID, bool flipY);
    void SetEditorCamera(Diamond::Camera* cam) { m_Context.EditorCamera = cam; }
    // The backend's asset-preview baker, routed to the content browser (which
    // the inspector's thumbnail lookups also go through).
    void SetThumbnailService(Diamond::ThumbnailService* svc) { m_Content.SetThumbnailService(svc); }
    // The backend's live-material-edit hook (no-op under GL); routed to the
    // Inspector, which calls it on edit-release.
    void SetMaterialInvalidator(std::function<void(const Diamond::PBRMaterial*)> fn) {
        m_Inspector.SetMaterialInvalidator(std::move(fn));
    }
    // The backend's scene-reload hook (no-op under GL): called after the scene
    // is rebuilt (open/new scene, play-stop snapshot restore) so the Vulkan
    // renderer drops GPU caches keyed by the old scene's object addresses.
    void SetSceneCacheInvalidator(std::function<void()> fn) {
        m_SceneCacheInvalidator = std::move(fn);
    }
    void UpdateCamera(const glm::mat4& view, const glm::mat4& proj, const glm::vec3& camPos);
    EditorContext& GetContext() { return m_Context; }
    bool IsViewportActive() const     { return m_Viewport.IsActive(); }
    bool IsGameViewportActive() const { return m_GameViewport.IsActive(); }
    ParticlePreviewPanel& GetParticlePreviewPanel() { return m_ParticlePreview; }
    void NotifyDroppedFiles(int count, const char** paths) { m_Content.QueueDroppedFiles(count, paths); }

private:
    void DrawMenuBar();
    void DrawToolbar();

    static std::string OpenFileDialog();
    static std::string SaveFileDialog();

    void DrawNewScriptDialog();

    std::function<void()> m_SceneCacheInvalidator;
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
    ParticlePreviewPanel m_ParticlePreview;
    MixerPanel        m_Mixer;
    EditorContext     m_Context;
};
