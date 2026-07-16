// DiamondEngine standalone runtime (Milestone 7): load a .scene, StartPlay,
// and run the game loop — no editor, no ImGui, the game camera fills the
// window. Vulkan-only: SceneRenderer's swapchain mode is the shipping path
// ("the VulkanScene demo / a shipped game"); this target isn't built when
// DIAMOND_ENABLE_VULKAN is off (see the root CMakeLists).
//
// Boot: Runtime.exe [path/to/scene.scene] — defaults to a test scene for now;
// the M7 packager step replaces this with a boot config next to the exe.

#define GLFW_INCLUDE_NONE
#include <GLFW/glfw3.h>
#include <glm/glm.hpp>
#include <spdlog/spdlog.h>

#include "Scene/Scene.h"
#include "Scene/SceneSerializer.h"
#include "Scene/Components.h"
#include "Scene/UISystem.h"
#include "Scene/UIRenderSystem.h"
#include "Scene/UIInputSystem.h"
#include "Scene/UINavigationSystem.h"
#include "Core/Input.h"
#include "Audio/AudioEngine.h"
#include "Audio/AudioAPI.h"
#include "Renderer/SceneRenderer.h"
#include "Renderer/RendererAPI.h"
#include "Renderer/RHI/RHIDevice.h"
#include "Renderer/RHI/RHIEnums.h"
#include "Renderer/TextureData.h"
#include "Profiling/CPUProfiler.h"

// Engine-internal header (Engine/src on the include path, like Sandbox): the
// in-game UI overlay needs a Renderer2D bound to this frame's command list,
// and the backend-neutral Renderer2D::Create() is the GL path.
#include "Platform/Vulkan/Resources/VulkanRenderer2D.h"

// Game code: script systems + component registrations (DECLARE_SYSTEM /
// DECLARE_COMPONENT inline statics register on TU inclusion — without this
// include, scripted components in the scene file fail to deserialize).
#include "Scripts/AllScripts.h"

#include <cstdio>
#include <memory>
#include <string>

using namespace Diamond;

int main(int argc, char** argv)
{
    std::string scenePath = ASSETS_DIR "/Scenes/ProceduralTest.scene";
    if (argc > 1) scenePath = argv[1];

    if (!glfwInit()) {
        std::fprintf(stderr, "glfwInit failed\n");
        return 1;
    }
    if (!glfwVulkanSupported()) {
        spdlog::critical("[Runtime] no Vulkan loader present");
        glfwTerminate();
        return 1;
    }

    glfwWindowHint(GLFW_CLIENT_API, GLFW_NO_API);
    // Fixed-size for now: the deferred render targets are built once at window
    // size, and SceneRenderer::Resize is offscreen-mode-only. Resizable window
    // + swapchain-mode target rebuild is a follow-up.
    glfwWindowHint(GLFW_RESIZABLE, GLFW_FALSE);
    GLFWwindow* window = glfwCreateWindow(1600, 900, "DiamondEngine", nullptr, nullptr);
    if (!window) {
        spdlog::critical("[Runtime] glfwCreateWindow failed");
        glfwTerminate();
        return 1;
    }

    // Device must outlive everything GPU-backed (declared first → reset last).
    std::unique_ptr<RHIDevice> device = RHIDevice::Create(window, RHIBackend::Vulkan);
    if (!device) {
        spdlog::critical("[Runtime] Vulkan device creation failed");
        glfwDestroyWindow(window);
        glfwTerminate();
        return 1;
    }

    // Route the static factories through Vulkan BEFORE the scene loads: every
    // Mesh/Texture the serializer creates must come out Vulkan-backed (same
    // order the editor backend uses).
    RendererAPI::SetAPI(RendererAPI::API::Vulkan);
    Texture::SetResourceDevice(device.get());

    int fbW = 0, fbH = 0;
    glfwGetFramebufferSize(window, &fbW, &fbH);

    // Swapchain mode (offscreen = false): Tonemap writes the backbuffer — the
    // scene fills the window, no compositor in between.
    std::unique_ptr<SceneRenderer> renderer =
        SceneRenderer::Create(device.get(), (uint32_t)fbW, (uint32_t)fbH);
    if (!renderer) {
        spdlog::critical("[Runtime] SceneRenderer::Create failed");
        Texture::SetResourceDevice(nullptr);
        device.reset();
        glfwDestroyWindow(window);
        glfwTerminate();
        return 1;
    }

    // In-game UI batcher, recorded into the swapchain overlay scope (so it
    // targets the swapchain's format, not the editor's RGBA8 offscreen one).
    auto r2d = std::make_unique<VulkanRenderer2D>(
        device.get(), DIAMOND_VULKAN_SHADER_DIR, device->SwapchainFormat());

    Input::Init(window);

    AudioEngine audioEngine;
    audioEngine.Init();

    Scene scene;
    if (!SceneSerializer::Load(scene, scenePath)) {
        spdlog::critical("[Runtime] failed to load scene '{}'", scenePath);
        audioEngine.Shutdown();
        r2d.reset();
        Texture::SetResourceDevice(nullptr);
        renderer.reset();
        device.reset();
        glfwDestroyWindow(window);
        glfwTerminate();
        return 1;
    }
    spdlog::info("[Runtime] playing '{}'", scenePath);
    scene.StartPlay();

    // In-game UI over the tonemapped scene: resolve canvases at window size,
    // hit-test the OS cursor, run gamepad focus nav, draw. Mirrors the editor
    // backend's game-view HUD overlay, minus the panel-space cursor remap —
    // here the window IS the game viewport.
    const SceneRenderer::OverlayFn uiOverlay = [&](RHICommandList* cmd) {
        auto& reg = scene.GetRegistry();
        const glm::vec2 screen { (float)fbW, (float)fbH };
        r2d->SetCommandList(cmd);
        UISystem::Resolve(reg, screen);

        double mx = 0.0, my = 0.0;
        glfwGetCursorPos(window, &mx, &my);
        UIInputSystem::Update(reg, glm::vec2((float)mx, (float)my),
                              Input::IsMouseButtonHeld(MouseButton::Left));

        UINavigationSystem::UINavInput nav = UI::PollGamepadNav();
        nav.mouseMoved = glm::length(Input::GetMouseDelta()) > 0.0f ||
                         Input::IsMouseButtonHeld(MouseButton::Left);
        UINavigationSystem::Update(reg, nav);

        UIRenderSystem::Render(reg, *r2d, screen);
    };

    float lastFrame = (float)glfwGetTime();
    while (!glfwWindowShouldClose(window))
    {
        glfwPollEvents();

        const float now = (float)glfwGetTime();
        const float deltaTime = now - lastFrame;
        lastFrame = now;

        // Alt-F4/close box is the real quit; ESC is a dev nicety until the
        // game layer owns pause/quit UI.
        if (glfwGetKey(window, GLFW_KEY_ESCAPE) == GLFW_PRESS)
            glfwSetWindowShouldClose(window, true);

        // Minimized: keep pumping events, render nothing.
        int w = 0, h = 0;
        glfwGetFramebufferSize(window, &w, &h);
        if (w == 0 || h == 0) continue;

        // One engine frame: systems → transforms → state machines → animators
        // → IK → ragdoll readback.
        scene.TickFrame(deltaTime);

        // Reclaim finished one-shot voices, enforce the voice cap.
        audioEngine.Update();

        // 3D listener: a scene AudioListenerComponent wins (AudioSystem syncs
        // it inside UpdateSystems); otherwise follow the game camera so
        // spatial audio tracks what the player sees.
        bool sceneHasListener = false;
        for (auto e : scene.View<AudioListenerComponent>()) { (void)e; sceneHasListener = true; break; }
        if (!sceneHasListener) {
            const entt::entity cam = scene.GetPrimaryCamera();
            if (cam != entt::null) {
                const glm::mat4 camW = scene.GetTransformSystem().GetWorldMatrix(cam);
                Audio::SetListener(glm::vec3(camW[3]), -glm::vec3(camW[2]), glm::vec3(camW[1]));
            }
        }

        renderer->RenderToSwapchain(scene, uiOverlay);

        CPUProfiler::EndFrame();
        Input::Update();
    }

    // Teardown: GPU idle first, then everything VMA-backed dies before the
    // device (same discipline as the demos). Scene entities hold Vulkan-backed
    // Mesh/Texture shared_ptrs — Clear() releases them while the device lives.
    device->WaitIdle();
    scene.StopPlay();
    audioEngine.Shutdown();
    scene.Clear();
    r2d.reset();
    Texture::SetResourceDevice(nullptr);
    renderer.reset();
    device.reset();
    glfwDestroyWindow(window);
    glfwTerminate();
    return 0;
}
