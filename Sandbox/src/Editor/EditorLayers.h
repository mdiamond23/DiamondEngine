#pragma once
#include <imgui.h>
#include <array>
#include <functional>
#include <string>
#include "EditorContext.h"
#include "Scene/Scene.h"
#include "Panels/Panels.h"
#include "Panels/HierarchyPanel.h"
#include "Panels/ViewportPanel.h"
#include "Panels/GameViewportPanel.h"
#include "Panels/InspectorPanel.h"
#include "Panels/ContentPanel.h"
#include "Panels/ConsolePanel.h"
#include "Panels/AnimatorPanel.h"
#include "Panels/ParticlePreviewPanel.h"
#include "Panels/MixerPanel.h"
#include "Panels/ProfilerPanel.h"
#include "Panels/RendererPanel.h"

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
    // Last-completed-frame renderer stats, pushed once per frame by the app
    // loop (see Sandbox/src/main.cpp) and read by ProfilerPanel.
    void SetStats(const Diamond::RendererStats& stats) { m_Profiler.SetStats(stats); }
    // The backend's renderer-settings widgets (exposure, FXAA, ...), drawn as
    // the body of the dockable Renderer panel.
    void SetRendererSettingsDraw(std::function<void()> fn) {
        m_RendererSettings.SetDrawCallback(std::move(fn));
    }
    void SetBackendIsVulkan(bool isVulkan) { m_Profiler.SetBackendIsVulkan(isVulkan); }
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

    void SaveScene();

    // Prefab edit mode: the scene is snapshotted and replaced by a lone
    // instance of the prefab, edited with the full editor toolset; Ctrl+S
    // writes it back to the .prefab. Exiting restores the snapshot — its
    // prefab references re-instantiate from the updated file, so open-scene
    // instances pick up the changes on the way out.
    void  OpenPrefabForEdit(const std::string& path);
    void  SavePrefabEdit();
    void  ExitPrefabEdit();
    float DrawPrefabEditBanner(float y);   // returns the banner height

    std::function<void()> m_SceneCacheInvalidator;
    bool              m_LayoutInitialized   = false;
    bool              m_OpenNewScriptDialog = false;
    std::string       m_SceneSnapshot;
    bool              m_PrefabEditMode      = false;
    std::string       m_PrefabEditPath;
    uint64_t          m_PrefabEditRootUuid  = 0;
    std::string       m_PrefabEditSnapshot;
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
    ProfilerPanel     m_Profiler;
    RendererPanel     m_RendererSettings;

    // Every panel above, for the generic render loop and the "View" menu's
    // checkbox list. Add new panels to both the members above and this list.
    std::array<Panel*, 11> m_Panels {
        &m_Hierarchy, &m_Viewport, &m_GameViewport, &m_Inspector, &m_Content,
        &m_Console, &m_Animator, &m_ParticlePreview, &m_Mixer, &m_Profiler,
        &m_RendererSettings
    };

    EditorContext     m_Context;
};
