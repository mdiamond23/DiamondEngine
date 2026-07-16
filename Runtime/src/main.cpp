// DiamondEngine standalone runtime (Milestone 7): load a .scene, StartPlay,
// and run the game loop — no editor, no ImGui, the game camera fills the
// window. Vulkan-only: SceneRenderer's swapchain mode is the shipping path
// ("the VulkanScene demo / a shipped game"); this target isn't built when
// DIAMOND_ENABLE_VULKAN is off (see the root CMakeLists).
//
// Boot: a boot.json next to the exe (what the packager writes) makes this a
// packaged game — assets + shaders resolve relative to the exe and the config
// picks the scene/window/icon. No boot.json = dev mode: compile-time dev-tree
// paths, optional scene from argv[1] (which also overrides a boot.json scene).

// windows.h must precede glfw3.h (APIENTRY redefinition otherwise).
#ifdef _WIN32
    #define WIN32_LEAN_AND_MEAN
    #define NOMINMAX
    #include <windows.h>
#endif

#define GLFW_INCLUDE_NONE
#include <GLFW/glfw3.h>
#include <glm/glm.hpp>
#include <spdlog/spdlog.h>
#include <spdlog/sinks/basic_file_sink.h>
#include <spdlog/sinks/stdout_color_sinks.h>

#include "Scene/Scene.h"
#include "Scene/SceneSerializer.h"
#include "Scene/SceneSystem.h"
#include "Assets/AssetPathUtils.h"
#include "Assets/ImageLoader.h"
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

#include <nlohmann/json.hpp>

#include <cstdio>
#include <filesystem>
#include <fstream>
#include <memory>
#include <string>
#include <vector>

using namespace Diamond;
namespace fs = std::filesystem;

namespace {

// Directory holding the running executable — the root of a packaged game.
// argv[0] is the fallback: unreliable in general (PATH lookups), but only
// reached on non-Windows, where a packaged Runtime doesn't ship yet anyway.
fs::path ExecutableDir(const char* argv0)
{
#ifdef _WIN32
    wchar_t buf[MAX_PATH];
    const DWORD n = GetModuleFileNameW(nullptr, buf, MAX_PATH);
    if (n > 0 && n < MAX_PATH) return fs::path(buf).parent_path();
#endif
    std::error_code ec;
    const fs::path p = fs::canonical(fs::path(argv0), ec);
    return ec ? fs::current_path() : p.parent_path();
}

// boot.json, written by the editor's packager. All fields optional. "scenes"
// is the build's scene list (portable "Assets/..." paths, entry 0 = boot
// scene) — the single source of truth the packager copies from and the future
// SceneManager transition API will load by name/index through. The legacy
// single-"scene" field still reads as a one-entry list.
struct BootConfig {
    std::vector<std::string> scenes;
    std::string title  = "DiamondEngine";
    int         width  = 1600;
    int         height = 900;
    std::string icon;   // portable path to a PNG for the window/taskbar icon
};

bool LoadBootConfig(const fs::path& path, BootConfig& out)
{
    std::ifstream f(path);
    if (!f.is_open()) return false;
    try {
        const nlohmann::json j = nlohmann::json::parse(f);
        if (j.contains("scenes"))
            out.scenes = j["scenes"].get<std::vector<std::string>>();
        else if (j.contains("scene"))
            out.scenes = { j["scene"].get<std::string>() };
        out.title  = j.value("title",  out.title);
        out.width  = j.value("width",  out.width);
        out.height = j.value("height", out.height);
        out.icon   = j.value("icon",   out.icon);
        return true;
    } catch (const std::exception& e) {
        spdlog::error("[Runtime] boot.json is malformed ({}) — falling back to dev mode", e.what());
        return false;
    }
}

// Every listed scene must be present in the package — a missing one means a
// transition would fail mid-game, so say it at boot while someone is looking.
// Only a missing BOOT scene is fatal (the load itself catches that below).
void ValidateSceneList(const std::vector<std::string>& scenes)
{
    for (const std::string& s : scenes) {
        std::error_code ec;
        if (!fs::exists(AssetPaths::Resolve(s), ec))
            spdlog::error("[Runtime] boot.json lists '{}' but it is not in the package", s);
    }
}

// Window/taskbar icon (not the exe's Explorer icon — that's baked at package
// time). GLFW wants tightly-packed RGBA8; expand RGB, reject anything odder.
void SetWindowIcon(GLFWwindow* window, const std::string& pngPath)
{
    const ImageData img = ImageLoader::Load(pngPath, /*flipVertically*/ false);
    if (img.Pixels.empty()) return;   // ImageLoader already logged the failure
    if (img.Channels != 3 && img.Channels != 4) {
        spdlog::warn("[Runtime] icon '{}' has {} channels — expected RGB/RGBA, skipping",
                     pngPath, img.Channels);
        return;
    }

    std::vector<uint8_t> rgba;
    const uint8_t* pixels = img.Pixels.data();
    if (img.Channels == 3) {
        rgba.resize((size_t)img.Width * img.Height * 4);
        for (size_t i = 0, n = (size_t)img.Width * img.Height; i < n; ++i) {
            rgba[i * 4 + 0] = img.Pixels[i * 3 + 0];
            rgba[i * 4 + 1] = img.Pixels[i * 3 + 1];
            rgba[i * 4 + 2] = img.Pixels[i * 3 + 2];
            rgba[i * 4 + 3] = 255;
        }
        pixels = rgba.data();
    }

    GLFWimage icon { img.Width, img.Height, const_cast<unsigned char*>(pixels) };
    glfwSetWindowIcon(window, 1, &icon);
}

} // namespace

int main(int argc, char** argv)
{
    // Packaged mode is declared by a boot.json next to the exe: rebase the
    // asset root and SPIR-V dir onto the exe's directory BEFORE anything
    // resolves a path or loads a shader.
    const fs::path exeDir = ExecutableDir(argv[0]);

    // Windows-subsystem exe: there is no console, so stdout goes nowhere. If a
    // parent terminal launched us, re-attach to it (dev runs still print);
    // either way the durable record is runtime.log next to the exe.
#ifdef _WIN32
    if (AttachConsole(ATTACH_PARENT_PROCESS)) {
        FILE* io = nullptr;
        freopen_s(&io, "CONOUT$", "w", stdout);
        freopen_s(&io, "CONOUT$", "w", stderr);
    }
#endif
    try {
        auto consoleSink = std::make_shared<spdlog::sinks::stdout_color_sink_mt>();
        auto fileSink    = std::make_shared<spdlog::sinks::basic_file_sink_mt>(
            (exeDir / "runtime.log").string(), /*truncate*/ true);
        spdlog::set_default_logger(std::make_shared<spdlog::logger>(
            "runtime", spdlog::sinks_init_list{ consoleSink, fileSink }));
        // A crash is exactly when the log matters — don't buffer it away.
        spdlog::flush_on(spdlog::level::info);
    } catch (const spdlog::spdlog_ex& e) {
        // Read-only install dir etc. — keep the default console-only logger.
        std::fprintf(stderr, "runtime.log unavailable: %s\n", e.what());
    }
    BootConfig boot;
    const bool packaged = LoadBootConfig(exeDir / "boot.json", boot);

    std::string shaderDir = DIAMOND_VULKAN_SHADER_DIR;
    std::string scenePath = ASSETS_DIR "/Scenes/ProceduralTest.scene";
    if (packaged) {
        AssetPaths::SetProjectRoot(exeDir);
        shaderDir = (exeDir / "shaders").string();
        SceneRenderer::SetShaderDirectory(shaderDir);
        if (!boot.scenes.empty()) scenePath = AssetPaths::Resolve(boot.scenes[0]);
        spdlog::info("[Runtime] packaged mode — root '{}', {} scene(s) in build",
                     exeDir.string(), boot.scenes.size());
        ValidateSceneList(boot.scenes);
    }
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
    GLFWwindow* window = glfwCreateWindow(boot.width, boot.height,
                                          boot.title.c_str(), nullptr, nullptr);
    if (!window) {
        spdlog::critical("[Runtime] glfwCreateWindow failed");
        glfwTerminate();
        return 1;
    }
    if (!boot.icon.empty())
        SetWindowIcon(window, AssetPaths::Resolve(boot.icon));

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
        device.get(), shaderDir, device->SwapchainFormat());

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

    // Scene transitions: the build's scene list is boot.json's (dev mode: just
    // the loaded scene). Current = the loaded scene's list entry, if it has
    // one — an argv override outside the list plays fine but can't Reload.
    {
        std::vector<std::string> list = boot.scenes;
        if (list.empty()) list.push_back(AssetPaths::ToPortable(scenePath));
        const std::string loaded = AssetPaths::LowerGeneric(AssetPaths::ToPortable(scenePath));
        uint32_t current = SceneSystem::kNoScene;
        for (uint32_t i = 0; i < (uint32_t)list.size(); ++i)
            if (AssetPaths::LowerGeneric(list[i]) == loaded) { current = i; break; }
        SceneSystem::SetSceneList(std::move(list));
        SceneSystem::SetCurrent(current);
    }

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

        // Window resized (or restored at a new size): rebuild the renderer's
        // internal targets to match. WaitIdle hitch, so only on change — the
        // device recreates the swapchain itself via OUT_OF_DATE.
        if (w != fbW || h != fbH) {
            renderer->Resize((uint32_t)w, (uint32_t)h);
            fbW = w;
            fbH = h;
        }

        // Deferred scene transition, requested by a script last frame via
        // SceneSystem::LoadScene. Swapped here, at the frame boundary — never
        // mid-UpdateSystems (the load destroys the systems and the registry
        // they iterate). GPU must be idle before the old scene's resources
        // die; InvalidateSceneCaches WaitIdles and drops the pointer-keyed
        // renderer caches a same-address reallocation would alias.
        if (const auto next = SceneSystem::ConsumePendingRequest()) {
            const std::string nextPath = AssetPaths::Resolve(SceneSystem::SceneList()[*next]);
            spdlog::info("[Runtime] scene transition -> '{}'", nextPath);
            renderer->InvalidateSceneCaches();
            scene.StopPlay();
            if (SceneSerializer::Load(scene, nextPath)) {   // clears the old scene itself
                scene.StartPlay();
                SceneSystem::SetCurrent(*next);
            } else {
                // The old world is already cleared — nothing sane to render.
                spdlog::critical("[Runtime] transition failed to load '{}' — exiting", nextPath);
                glfwSetWindowShouldClose(window, true);
            }
        }

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

#ifdef _WIN32
// WIN32_EXECUTABLE (no console window behind the game) makes the CRT expect
// WinMain; forward to the portable main. __argc/__argv are the CRT's already-
// parsed command line, so argv[1] scene overrides keep working.
int WINAPI WinMain(HINSTANCE, HINSTANCE, LPSTR, int)
{
    return main(__argc, __argv);
}
#endif
