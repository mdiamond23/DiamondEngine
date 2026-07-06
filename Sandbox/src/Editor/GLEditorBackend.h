#pragma once

#include "Editor/EditorBackend.h"

#include <cstdint>
#include <functional>
#include <memory>

class EditorLayer;

// The OpenGL implementation of the shared editor loop's backend: the original
// main.cpp GL render path — deferred render graph into viewport FBOs, debug
// draw, UI passes, the particle preview — behind the EditorBackend interface.
// All GL objects live in an Init-created Gfx struct so nothing touches GL
// before the context exists.
class GLEditorBackend final : public Diamond::EditorBackend {
public:
    GLEditorBackend();
    ~GLEditorBackend() override;

    bool Init(uint32_t width, uint32_t height, const char* title) override;
    void Shutdown() override;

    GLFWwindow* Window() const override { return m_Window; }
    const char* Name() const override { return "OpenGL"; }

    void SetDropHandler(std::function<void(int, const char**)> handler) override;
    void BeginImGuiFrame() override;
    void OnImGuiPanels() override;
    void RenderFrame(const Diamond::EditorFrameInput& input) override;

    Diamond::EditorViewImage SceneViewImage() const override;
    Diamond::EditorViewImage GameViewImage() const override;

    // GL-only editor extras (debug draw, the particle preview panel) read the
    // editor layer directly; the Vulkan backend gets these through the step-4
    // editor services instead.
    void AttachEditorLayer(EditorLayer* layer) { m_Editor = layer; }

private:
    struct Gfx;

    void BuildGraph(int w, int h);
    void CullAndExecute(uint32_t targetFBO);

    GLFWwindow*  m_Window = nullptr;
    EditorLayer* m_Editor = nullptr;
    std::unique_ptr<Gfx> m_G;
    std::function<void(int, const char**)> m_DropHandler;

    float m_ShaderReloadTimer = 0.0f;   // throttles the hot-reload mtime poll
    int   m_LastFbW = 0, m_LastFbH = 0;
    bool  m_LastGameViewValid = false;  // game FBO holds a fresh frame
    bool  m_FxaaEnabled = true;
};
