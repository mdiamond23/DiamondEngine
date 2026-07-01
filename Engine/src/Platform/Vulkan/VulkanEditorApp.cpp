#include "Platform/VulkanDemo.h"

// The Sandbox editor's Vulkan mode (`--vulkan`). Lives inside the engine — like
// the demos — because the Vulkan backend's headers, defines, and volk include
// path are private to MyEngine; the app only sees RunVulkanEditor().
//
// This TU is compiled unconditionally: without the Vulkan backend it collapses
// to a stub (same pattern as SceneRenderer.cpp), so Sandbox links either way.

#ifdef DIAMOND_ENABLE_VULKAN

#include "Renderer/SceneRenderer.h"
#include "Renderer/RHI/RHIDevice.h"
#include "Renderer/RHI/RHIEnums.h"
#include "Renderer/MeshData.h"
#include "Renderer/Renderer2D.h"
#include "Renderer/TextureData.h"
#include "Renderer/Font.h"
#include "Core/Camera.h"
#include "Scene/Scene.h"
#include "Scene/Components.h"
#include "Scene/ParticleSim.h"
#include "Scene/UISystem.h"
#include "Scene/UIRenderSystem.h"
#include "Scene/UIInputSystem.h"

#include "Platform/Vulkan/VulkanImGuiLayer.h"
#include "Platform/Vulkan/RHI/VulkanRHIDevice.h"
#include "Platform/Vulkan/RHI/VulkanRHIResources.h"
#include "Platform/Vulkan/Resources/VulkanRenderer2D.h"

#include "imgui.h"
#include "backends/imgui_impl_vulkan.h"

// Match the SceneRenderer's clip-space depth convention.
#define GLM_FORCE_DEPTH_ZERO_TO_ONE
#define GLFW_INCLUDE_NONE
#include <GLFW/glfw3.h>
#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>
#include <spdlog/spdlog.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <memory>

namespace Diamond {

namespace {

// A GPU-free Mesh handle (see VulkanSceneDemo): the engine's Mesh::Create issues
// GL calls, which would crash in this GL-less window. The MeshComponent only
// needs a stable pointer identity to key SceneRenderer's mesh cache; the real
// geometry is handed over via RegisterMesh. Real asset meshes under Vulkan are
// the material/asset-pipeline slice.
class StubMesh final : public Mesh {
public:
    void Draw(const Shader&) const override {}
};

void OnFramebufferResize(GLFWwindow* window, int /*w*/, int /*h*/) {
    auto* device = static_cast<RHIDevice*>(glfwGetWindowUserPointer(window));
    if (device) device->NotifyResize();
}

entt::entity SpawnMesh(Scene& scene, std::string_view name,
                       const std::shared_ptr<Mesh>& mesh,
                       const glm::vec3& pos, const glm::vec3& scale) {
    entt::entity e = scene.CreateEntity(name);
    auto& t = scene.Get<TransformComponent>(e);
    t.position = pos;
    t.scale    = scale;
    auto& mc = scene.Add<MeshComponent>(e);
    mc.mesh = mesh;
    return e;
}

// The same lit test scene the VulkanScene demo uses (floor + sphere + cubes +
// lights + a spark emitter), plus a HUD canvas exercising every widget type.
void BuildScene(Scene& scene, const std::shared_ptr<Mesh>& cubeMesh,
                const std::shared_ptr<Mesh>& sphereMesh,
                const std::shared_ptr<Font>& font) {
    SpawnMesh(scene, "Floor",  cubeMesh,   { 0.0f, -0.55f, 0.0f }, { 6.0f, 0.1f, 6.0f });
    SpawnMesh(scene, "Sphere", sphereMesh, { 0.0f,  0.3f,  0.0f }, { 0.8f, 0.8f, 0.8f });
    SpawnMesh(scene, "CubeA",  cubeMesh,   {  1.8f, 0.0f,  0.0f }, { 0.6f, 0.6f, 0.6f });
    SpawnMesh(scene, "CubeB",  cubeMesh,   { -1.8f, 0.0f,  0.6f }, { 0.6f, 0.6f, 0.6f });
    SpawnMesh(scene, "CubeC",  cubeMesh,   {  0.4f, 0.0f, -1.9f }, { 0.6f, 0.6f, 0.6f });

    {
        entt::entity e = scene.CreateEntity("PointWarm");
        scene.Get<TransformComponent>(e).position = { 1.8f, 1.2f, 1.8f };
        auto& lc = scene.Add<LightComponent>(e);
        lc.type = LightType::Point;  lc.color = { 1.0f, 0.42f, 0.17f };  lc.intensity = 6.0f;
    }
    {
        entt::entity e = scene.CreateEntity("PointCool");
        scene.Get<TransformComponent>(e).position = { -1.6f, 1.0f, -1.2f };
        auto& lc = scene.Add<LightComponent>(e);
        lc.type = LightType::Point;  lc.color = { 0.25f, 0.5f, 1.0f };  lc.intensity = 4.0f;
    }
    {
        entt::entity e = scene.CreateEntity("Sun");
        scene.Get<TransformComponent>(e).rotation =
            glm::quat(glm::radians(glm::vec3(55.0f, -25.0f, 0.0f)));
        auto& lc = scene.Add<LightComponent>(e);
        lc.type = LightType::Sun;  lc.color = { 1.0f, 0.97f, 0.9f };  lc.intensity = 3.0f;
    }

    {
        entt::entity e = scene.CreateEntity("Additive Sparks");
        scene.Get<TransformComponent>(e).position = { 0.0f, -0.35f, 0.85f };
        auto& em = scene.Add<ParticleEmitterComponent>(e);
        em.spawnRate     = 90.0f;
        em.maxParticles  = 512;
        em.shape         = EmissionShape::Cone;
        em.coneAngle     = 18.0f;
        em.coneRadius    = 0.08f;
        em.speedRange    = { 1.4f, 2.6f };
        em.lifetimeRange = { 0.8f, 1.4f };
        em.sizeRange     = { 0.08f, 0.18f };
        em.gravity       = { 0.0f, -1.8f, 0.0f };
        em.startColor    = { 1.0f, 0.72f, 0.22f, 0.95f };
        em.endColor      = { 1.0f, 0.08f, 0.0f, 0.0f };
        em.endSize       = 0.02f;
        em.blend         = ParticleBlend::Additive;
        ParticleSim::Reset(em);
    }

    // HUD canvas — drawn by the SceneRenderer's UI2D pass into the scene output,
    // so it lives inside the viewport image like the GL game viewport's HUD.
    entt::entity canvas = scene.CreateEntity("HUD");
    scene.Add<CanvasComponent>(canvas);

    {
        entt::entity bar = scene.CreateEntity("HealthBar");
        scene.SetParent(bar, canvas);
        auto& rt = scene.Add<RectTransformComponent>(bar);
        rt.anchorMin = rt.anchorMax = { 0.0f, 0.0f };
        rt.pivot     = { 0.0f, 0.0f };
        rt.position  = { 40.0f, 40.0f };
        rt.size      = { 420.0f, 30.0f };
        auto& pb = scene.Add<UIProgressBarComponent>(bar);
        pb.progress = 0.72f;
    }
    if (font) {
        entt::entity label = scene.CreateEntity("HudLabel");
        scene.SetParent(label, canvas);
        auto& rt = scene.Add<RectTransformComponent>(label);
        rt.anchorMin = rt.anchorMax = { 0.0f, 0.0f };
        rt.pivot     = { 0.0f, 0.0f };
        rt.position  = { 40.0f, 78.0f };
        rt.size      = { 480.0f, 40.0f };
        auto& txt = scene.Add<UITextComponent>(label);
        txt.text   = "DiamondEngine — Vulkan editor HUD";
        txt.font   = font;
        txt.sizePx = 26.0f;
        txt.color  = { 0.9f, 0.95f, 1.0f, 1.0f };
    }
    {
        entt::entity button = scene.CreateEntity("HudButton");
        scene.SetParent(button, canvas);
        auto& rt = scene.Add<RectTransformComponent>(button);
        rt.anchorMin = rt.anchorMax = { 0.0f, 1.0f };   // bottom-left
        rt.pivot     = { 0.0f, 1.0f };
        rt.position  = { 40.0f, -40.0f };
        rt.size      = { 200.0f, 56.0f };
        auto& img = scene.Add<UIImageComponent>(button);
        img.tint = { 0.16f, 0.42f, 0.75f, 0.9f };
        auto& btn = scene.Add<UIButtonComponent>(button);
        btn.onClick = [] { spdlog::info("[VulkanEditor] HUD button clicked"); };
        if (font) {
            auto& txt = scene.Add<UITextComponent>(button);
            txt.text   = "Click me";
            txt.font   = font;
            txt.sizePx = 24.0f;
            txt.hAlign = UITextComponent::HAlign::Center;
            txt.vAlign = UITextComponent::VAlign::Middle;
        }
    }
}

} // namespace

int RunVulkanEditor() {
    if (!glfwInit()) {
        spdlog::critical("[VulkanEditor] glfwInit failed");
        return 1;
    }
    if (!glfwVulkanSupported()) {
        spdlog::critical("[VulkanEditor] GLFW reports no Vulkan loader present");
        glfwTerminate();
        return 1;
    }

    glfwWindowHint(GLFW_CLIENT_API, GLFW_NO_API);
    GLFWwindow* window = glfwCreateWindow(1600, 900, "DiamondEngine — Vulkan Editor", nullptr, nullptr);
    if (!window) {
        spdlog::critical("[VulkanEditor] glfwCreateWindow failed");
        glfwTerminate();
        return 1;
    }

    std::unique_ptr<RHIDevice> device = RHIDevice::Create(window, RHIBackend::Vulkan);
    if (!device) {
        spdlog::critical("[VulkanEditor] device creation failed");
        glfwDestroyWindow(window);
        glfwTerminate();
        return 1;
    }
    auto* vkDevice = static_cast<VulkanRHIDevice*>(device.get());
    glfwSetWindowUserPointer(window, device.get());
    glfwSetFramebufferSizeCallback(window, OnFramebufferResize);

    // Route the Texture factory through this device before any font/UI texture
    // bakes, so they come out Vulkan-backed.
    Texture::SetResourceDevice(device.get());
    std::shared_ptr<Font> font =
        Font::Create(DIAMOND_ASSETS_DIR "/Fonts/OpenSans-Regular.ttf", 32.0f);
    if (!font)
        spdlog::warn("[VulkanEditor] font bake failed — HUD text disabled");

    // Scene renders offscreen: Tonemap lands in OutputColor(), which ImGui shows
    // inside a dockable viewport panel; the backbuffer belongs to ImGui.
    uint32_t sceneW = 1280, sceneH = 720;
    std::unique_ptr<SceneRenderer> renderer =
        SceneRenderer::Create(device.get(), sceneW, sceneH, /*offscreen*/ true);
    if (!renderer) {
        spdlog::critical("[VulkanEditor] SceneRenderer::Create returned null");
        Texture::SetResourceDevice(nullptr);
        device.reset();
        glfwDestroyWindow(window);
        glfwTerminate();
        return 1;
    }

    // In-game UI batcher, recorded by the SceneRenderer's UI2D pass into the
    // RGBA8 scene output (not the swapchain — that's ImGui's).
    auto r2d = std::make_unique<VulkanRenderer2D>(
        device.get(), DIAMOND_VULKAN_SHADER_DIR, RHIFormat::RGBA8);

    Scene scene;
    auto cubeMesh   = std::make_shared<StubMesh>();
    auto sphereMesh = std::make_shared<StubMesh>();
    BuildScene(scene, cubeMesh, sphereMesh, font);
    renderer->RegisterMesh(cubeMesh.get(),   MeshData::UnitCube());
    renderer->RegisterMesh(sphereMesh.get(), MeshData::UVSphere());

    // UI2D pass body — shared state below is written during ImGui widget building
    // and consumed when the graph pass executes later the same frame.
    glm::vec2 uiScreen  { (float)sceneW, (float)sceneH };
    glm::vec2 uiPointer { -1.0f, -1.0f };
    bool      uiDown = false;
    renderer->SetUIOverlay([&](RHICommandList* cmd) {
        r2d->SetCommandList(cmd);
        UISystem::Resolve(scene.GetRegistry(), uiScreen);
        UIInputSystem::Update(scene.GetRegistry(), uiPointer, uiDown);
        UIRenderSystem::Render(scene.GetRegistry(), *r2d, uiScreen);
    });

    VulkanImGuiLayer imgui;
    imgui.Init(window, vkDevice);

    // One viewport descriptor per frame-in-flight: the offscreen target is a
    // per-frame image internally, and each frame ImGui must sample the copy the
    // graph is writing this frame (transitioned to SHADER_READ_ONLY by the
    // EditorOverlay pass's read before ImGui draws).
    std::array<VkDescriptorSet, VulkanRHIDevice::kFramesInFlight> viewportSets{};
    auto registerViewportTexture = [&] {
        auto* tex = static_cast<VulkanRHITexture*>(renderer->OutputColor());
        for (uint32_t i = 0; i < VulkanRHIDevice::kFramesInFlight; ++i) {
            if (viewportSets[i]) ImGui_ImplVulkan_RemoveTexture(viewportSets[i]);
            viewportSets[i] = ImGui_ImplVulkan_AddTexture(
                tex->Sampler(), tex->View(i), VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL);
        }
    };
    registerViewportTexture();

    Camera camera(glm::vec3(4.0f, 2.6f, 5.0f));
    camera.LookAt(glm::vec3(0.0f, 0.2f, 0.0f));

    // Debounced viewport resize: panels resize every frame of a drag, but Resize
    // is a WaitIdle + pass rebuild — only fire once the size has settled.
    glm::ivec2 wantSize { (int)sceneW, (int)sceneH };
    int        wantStableFrames = 0;
    constexpr int kResizeSettleFrames = 15;

    double lastTime = glfwGetTime();
    while (!glfwWindowShouldClose(window)) {
        glfwPollEvents();

        const double nowTime = glfwGetTime();
        const float  dt      = (float)(nowTime - lastTime);
        lastTime = nowTime;

        // Apply a settled viewport resize before any ImGui state references the
        // old target (WaitIdle inside Resize makes the descriptor swap safe).
        if (((uint32_t)wantSize.x != sceneW || (uint32_t)wantSize.y != sceneH) &&
            wantStableFrames >= kResizeSettleFrames) {
            sceneW = (uint32_t)wantSize.x;
            sceneH = (uint32_t)wantSize.y;
            renderer->Resize(sceneW, sceneH);
            registerViewportTexture();
            uiScreen = { (float)sceneW, (float)sceneH };
        }

        scene.GetTransformSystem().Update(scene.GetRegistry());
        for (auto [e, em] : scene.GetRegistry().view<ParticleEmitterComponent>().each())
            ParticleSim::Step(em, dt, scene.GetTransformSystem().GetWorldMatrix(e));

        // Animate the HUD bar so the UI pass is visibly live.
        for (auto [e, pb] : scene.GetRegistry().view<UIProgressBarComponent>().each())
            pb.progress = 0.5f + 0.5f * std::sin((float)nowTime * 0.8f);

        imgui.BeginFrame();
        ImGui::DockSpaceOverViewport();

        // Scene viewport panel — the offscreen target as an ImGui image.
        uiPointer = { -1.0f, -1.0f };
        uiDown    = false;
        ImGui::SetNextWindowSize(ImVec2(1280, 720), ImGuiCond_FirstUseEver);
        if (ImGui::Begin("Viewport")) {
            const ImVec2 avail = ImGui::GetContentRegionAvail();
            const glm::ivec2 size { std::max((int)avail.x, 64), std::max((int)avail.y, 64) };
            if (size == wantSize) ++wantStableFrames;
            else                  { wantSize = size; wantStableFrames = 0; }

            ImGui::Image((ImTextureID)(uintptr_t)viewportSets[vkDevice->CurrentFrame()],
                         avail);

            // Map the cursor into scene-output pixels for the in-game UI while
            // the image is hovered.
            const ImVec2 rMin  = ImGui::GetItemRectMin();
            const ImVec2 rMax  = ImGui::GetItemRectMax();
            const ImVec2 mouse = ImGui::GetMousePos();
            if (ImGui::IsItemHovered() && rMax.x > rMin.x && rMax.y > rMin.y) {
                uiPointer = { (mouse.x - rMin.x) / (rMax.x - rMin.x) * (float)sceneW,
                              (mouse.y - rMin.y) / (rMax.y - rMin.y) * (float)sceneH };
                uiDown = ImGui::IsMouseDown(ImGuiMouseButton_Left);
            }

            // Editor camera: RMB-drag to look, WASD to fly, while hovered.
            if (ImGui::IsItemHovered() && ImGui::IsMouseDown(ImGuiMouseButton_Right)) {
                const ImGuiIO& io = ImGui::GetIO();
                camera.ProcessMouseMovement(io.MouseDelta.x, -io.MouseDelta.y);
                if (glfwGetKey(window, GLFW_KEY_W) == GLFW_PRESS)
                    camera.ProcessKeyboard(CameraMovement::Forward,  dt);
                if (glfwGetKey(window, GLFW_KEY_S) == GLFW_PRESS)
                    camera.ProcessKeyboard(CameraMovement::Backward, dt);
                if (glfwGetKey(window, GLFW_KEY_A) == GLFW_PRESS)
                    camera.ProcessKeyboard(CameraMovement::Left,     dt);
                if (glfwGetKey(window, GLFW_KEY_D) == GLFW_PRESS)
                    camera.ProcessKeyboard(CameraMovement::Right,    dt);
            }
        }
        ImGui::End();

        if (ImGui::Begin("Stats")) {
            ImGui::Text("Backend: Vulkan (RTX-class, deferred)");
            ImGui::Text("FPS: %.1f (%.2f ms)", ImGui::GetIO().Framerate,
                        1000.0f / ImGui::GetIO().Framerate);
            ImGui::Text("Scene output: %ux%u", sceneW, sceneH);
            ImGui::Separator();
            ImGui::TextWrapped("RMB-drag in the viewport to look, WASD to fly. "
                               "The HUD (bar/text/button) is the in-game UI pass.");
        }
        ImGui::End();

        ImGui::Render();
        renderer->RenderToSwapchain(scene, camera, [&](RHICommandList* cmd) {
            imgui.Submit(cmd);
        });
    }

    // Teardown: everything VMA-backed dies before the device; ImGui's descriptors
    // (viewport sets included) die with its own pool in Shutdown.
    device->WaitIdle();
    imgui.Shutdown();
    r2d.reset();
    font.reset();
    Texture::SetResourceDevice(nullptr);
    renderer.reset();
    device.reset();
    glfwDestroyWindow(window);
    glfwTerminate();
    return 0;
}

} // namespace Diamond

#else  // !DIAMOND_ENABLE_VULKAN

#include <spdlog/spdlog.h>

namespace Diamond {
int RunVulkanEditor() {
    spdlog::critical("--vulkan requested but this build has no Vulkan backend "
                     "(DIAMOND_ENABLE_VULKAN is off)");
    return 1;
}
} // namespace Diamond

#endif
