#pragma once

#include "Renderer/ParticleRenderer.h"

#include <cstdint>
#include <functional>
#include <memory>
#include <vector>

#include <glm/glm.hpp>

struct GLFWwindow;
class Scene;

namespace Diamond {

class Camera;
class ThumbnailService;
struct PBRMaterial;

// Opaque panel image handle: an ImTextureID-compatible value (a GL texture id
// or a Vulkan descriptor set) plus the UV orientation the panel must draw it
// with. textureId 0 means "nothing to show" — the panel draws its placeholder.
struct EditorViewImage {
    uint64_t textureId = 0;
    bool     flipY     = false;   // true for GL FBO textures (bottom-up)
};

// Per-frame data the shared editor loop hands the renderer backend.
struct EditorFrameInput {
    Scene*  scene        = nullptr;
    Camera* editorCamera = nullptr;

    // Editor-camera matrices at the window-framebuffer aspect — the same pair
    // handed to ImGuizmo, so gizmos line up with the rendered viewport image.
    glm::mat4 view { 1.0f };
    glm::mat4 proj { 1.0f };

    // Primary-game-camera matrices; valid only when wantGameView.
    glm::mat4 gameView { 1.0f };
    glm::mat4 gameProj { 1.0f };

    float dt = 0.0f;
    bool  playing      = false;   // scene play mode (paused still counts)
    bool  showUI       = false;   // editor-viewport UI-canvas preview toggle
    bool  wantGameView = false;   // playing && the scene has a primary camera

    // Game-panel cursor normalized to [0,1) (top-left origin), components < 0
    // or >= 1 mean outside — maps the OS mouse into the in-game UI for
    // hit-testing during play.
    glm::vec2 gameMouseNorm { -1.0f, -1.0f };

    // ── Particle preview panel (editor-wiring) ───────────────────────────────
    // Vulkan-only: the GL backend drives ParticlePreviewPanel directly (it
    // holds an EditorLayer* via AttachEditorLayer) and ignores these. The
    // Vulkan backend can't include Sandbox's panel headers (Engine can't
    // depend on Sandbox), so the loop extracts the panel's per-frame state
    // into engine-visible types here instead. previewActive false means no
    // emitter is selected — still size the target from previewSize so the
    // panel's placeholder isn't stretched oddly once a real frame arrives.
    bool                         previewActive   = false;
    glm::ivec2                   previewSize     { 1, 1 };
    glm::vec3                    previewBgColor  { 0.10f, 0.10f, 0.12f };
    glm::mat4                    previewView     { 1.0f };
    glm::mat4                    previewProj     { 1.0f };
    std::vector<RenderParticle>  previewParticles;         // this frame's live particles
    const Texture*               previewTexture  = nullptr; // nullptr = renderer's default dot
    ParticleBlend                previewBlend    = ParticleBlend::Alpha;
};

// One graphics backend under the shared Sandbox editor loop: owns the window,
// the device/context, the ImGui platform + renderer bindings, and all scene
// rendering. The loop itself (Sandbox main.cpp) is backend-agnostic; anything
// GL- or Vulkan-specific lives behind this interface.
//
// Lifecycle: the caller creates the ImGui context and loads fonts, then calls
// Init(); Shutdown() tears down the backend's objects AND destroys the ImGui
// context (both ImGui renderer bindings assume they outlive their context).
class EditorBackend {
public:
    virtual ~EditorBackend() = default;

    // Window + device/context + ImGui backend bindings. Returns false on
    // unrecoverable failure (caller logs and exits). glfwInit has already run.
    virtual bool Init(uint32_t width, uint32_t height, const char* title) = 0;
    virtual void Shutdown() = 0;

    virtual GLFWwindow* Window() const = 0;
    virtual const char* Name() const = 0;   // "OpenGL" / "Vulkan" — title bar, stats

    // The backend owns the GLFW window callbacks; dropped files are forwarded
    // here (the loop routes them to the editor's content browser).
    virtual void SetDropHandler(std::function<void(int, const char**)> handler) = 0;

    // ImGui_Impl*_NewFrame + ImGui::NewFrame.
    virtual void BeginImGuiFrame() = 0;

    // Backend-specific settings windows (exposure, FXAA, ...) — drawn between
    // BeginImGuiFrame and ImGui::Render like any other editor window.
    virtual void OnImGuiPanels() {}

    // Renders and presents one frame: scene views, in-game UI, then the ImGui
    // draw data over everything. Call after ImGui::Render().
    virtual void RenderFrame(const EditorFrameInput& input) = 0;

    // Panel images to show this frame — query while building ImGui windows.
    virtual EditorViewImage SceneViewImage() const = 0;
    virtual EditorViewImage GameViewImage() const = 0;

    // The particle-preview panel's rendered image (Vulkan only — see
    // EditorFrameInput's preview* fields). GL drives the panel directly and
    // never needs this; default no-op returns "nothing to show".
    virtual EditorViewImage ParticlePreviewImage() const { return {}; }

    // Asset-preview bakes for the content browser / inspector (editor-wiring
    // step 4). Valid between Init() and Shutdown().
    virtual ThumbnailService* Thumbnails() = 0;

    // Live material edits (Inspector, on edit-release): the GL backend re-binds
    // every texture/uniform per draw, so it needs no notification — default
    // no-op. The Vulkan backend bakes a descriptor set per material and must
    // drop the cached one so it rebuilds from the material's current state.
    virtual void InvalidateMaterial(const PBRMaterial*) {}
};

// Factory for the engine-owned Vulkan implementation (the Vulkan headers and
// defines are private to MyEngine, so it can't live app-side). Returns nullptr
// when this build carries no Vulkan backend (DIAMOND_ENABLE_VULKAN off). The
// GL implementation lives in Sandbox, next to the editor's GL render path.
std::unique_ptr<EditorBackend> CreateVulkanEditorBackend();

} // namespace Diamond
