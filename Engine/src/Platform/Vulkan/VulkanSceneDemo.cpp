#include "Platform/VulkanDemo.h"
#include "Renderer/SceneRenderer.h"
#include "Renderer/RHI/RHIDevice.h"
#include "Renderer/RHI/RHIEnums.h"
#include "Renderer/MeshData.h"
#include "Core/Camera.h"
#include "Scene/Scene.h"
#include "Scene/Components.h"

#include "Platform/Vulkan/VulkanImGuiLayer.h"
#include "Platform/Vulkan/RHI/VulkanRHIDevice.h"
#include "Platform/Vulkan/Resources/VulkanRenderer2D.h"
#include "Platform/Vulkan/Resources/VulkanTexture2D.h"
#include "Renderer/RHI/RHIEnums.h"
#include "Renderer/Renderer2D.h"
#include "Renderer/TextureData.h"
#include "Renderer/Font.h"
#include "imgui.h"

#include <array>
#include <cstdint>

// Match the SceneRenderer's clip-space depth convention.
#define GLM_FORCE_DEPTH_ZERO_TO_ONE
#define GLFW_INCLUDE_NONE
#include <GLFW/glfw3.h>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/quaternion.hpp>
#include <spdlog/spdlog.h>

#include <cmath>
#include <memory>

namespace Diamond {

namespace {

// A GPU-free Mesh handle. The engine's real Mesh::Create issues OpenGL calls in
// its constructor, which would crash in this GL-less Vulkan window. Here the
// MeshComponent only needs a stable pointer identity to key SceneRenderer's mesh
// cache; the actual geometry is handed to the renderer via RegisterMesh. (The
// asset-pipeline path that produces real Mesh handles under Vulkan is Slice 3.)
class StubMesh final : public Mesh {
public:
    void Draw(const Shader&) const override {}
};

// A small RGBA checkerboard so the overlay has a real sampled texture to prove
// the Renderer2D textured-quad path (per-texture descriptor set + UV sub-rects).
std::shared_ptr<VulkanTexture2D> MakeCheckerTexture(RHIDevice* device) {
    constexpr int kN = 64;
    std::array<uint8_t, kN * kN * 4> px{};
    for (int y = 0; y < kN; ++y) {
        for (int x = 0; x < kN; ++x) {
            const bool on = ((x / 8) + (y / 8)) % 2 == 0;
            const size_t i = (static_cast<size_t>(y) * kN + x) * 4;
            px[i + 0] = on ? 235 : 40;
            px[i + 1] = on ? 200 : 60;
            px[i + 2] = on ? 60  : 90;
            px[i + 3] = 255;
        }
    }
    return std::make_shared<VulkanTexture2D>(device, px.data(), kN, kN, 4,
                                             RHIFilter::Nearest);
}

void OnFramebufferResize(GLFWwindow* window, int /*w*/, int /*h*/) {
    auto* device = static_cast<RHIDevice*>(glfwGetWindowUserPointer(window));
    if (device) device->NotifyResize();
}

// Spawns a mesh entity with a transform. Returns the entity so lights/animation
// can reference it.
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

} // namespace

int RunVulkanSceneDemo() {
    if (!glfwInit()) {
        spdlog::critical("[SceneRenderer] glfwInit failed");
        return 1;
    }
    if (!glfwVulkanSupported()) {
        spdlog::critical("[SceneRenderer] GLFW reports no Vulkan loader present");
        glfwTerminate();
        return 1;
    }

    glfwWindowHint(GLFW_CLIENT_API, GLFW_NO_API);   // Vulkan window
    GLFWwindow* window = glfwCreateWindow(1280, 720, "DiamondEngine — Vulkan Scene", nullptr, nullptr);
    if (!window) {
        spdlog::critical("[SceneRenderer] glfwCreateWindow failed");
        glfwTerminate();
        return 1;
    }

    // Device must outlive the renderer (declared first → destroyed last).
    std::unique_ptr<RHIDevice> device = RHIDevice::Create(window, RHIBackend::Vulkan);
    if (!device) {
        spdlog::critical("[SceneRenderer] device creation failed");
        glfwDestroyWindow(window);
        glfwTerminate();
        return 1;
    }
    glfwSetWindowUserPointer(window, device.get());
    glfwSetFramebufferSizeCallback(window, OnFramebufferResize);

    // ImGui overlay — reaches into the Vulkan device directly (backend-specific).
    VulkanImGuiLayer imgui;
    imgui.Init(window, static_cast<VulkanRHIDevice*>(device.get()));

    // 2D overlay batcher — draws HUD quads over the tonemapped scene, recorded
    // into the same swapchain overlay scope as ImGui. Heap-owned so its RHI
    // resources are released before the device (VMA allocator) is torn down.
    auto r2d = std::make_unique<VulkanRenderer2D>(
        device.get(), DIAMOND_VULKAN_SHADER_DIR, device->SwapchainFormat());

    // A sampled texture for the overlay's textured-quad demo.
    auto checker = MakeCheckerTexture(device.get());

    // Route the Texture factory through this device so Font::Create (which bakes
    // its atlas via Texture::CreateFromPixels) produces a Vulkan-backed atlas.
    Texture::SetResourceDevice(device.get());
    auto font = Font::Create(DIAMOND_ASSETS_DIR "/Fonts/OpenSans-Regular.ttf", 32.0f);
    if (!font)
        spdlog::warn("[SceneRenderer] font bake failed — text overlay disabled");

    std::unique_ptr<SceneRenderer> renderer = SceneRenderer::Create(device.get(), 1280, 720);
    if (!renderer) {
        spdlog::critical("[SceneRenderer] SceneRenderer::Create returned null");
        device.reset();
        glfwDestroyWindow(window);
        glfwTerminate();
        return 1;
    }

    // ── Build a real Scene through the ECS ──────────────────────────────────────
    Scene scene;

    auto cubeMesh   = std::make_shared<StubMesh>();
    auto sphereMesh = std::make_shared<StubMesh>();

    // Floor slab (a flattened cube), a centerpiece sphere, and a ring of cubes so
    // SSAO contact creases + cascade shadows have something to fall on.
    SpawnMesh(scene, "Floor",  cubeMesh,   { 0.0f, -0.55f, 0.0f }, { 6.0f, 0.1f, 6.0f });
    SpawnMesh(scene, "Sphere", sphereMesh, { 0.0f,  0.3f,  0.0f }, { 0.8f, 0.8f, 0.8f });
    SpawnMesh(scene, "CubeA",  cubeMesh,   {  1.8f, 0.0f,  0.0f }, { 0.6f, 0.6f, 0.6f });
    SpawnMesh(scene, "CubeB",  cubeMesh,   { -1.8f, 0.0f,  0.6f }, { 0.6f, 0.6f, 0.6f });
    SpawnMesh(scene, "CubeC",  cubeMesh,   {  0.4f, 0.0f, -1.9f }, { 0.6f, 0.6f, 0.6f });

    // Two warm/cool point lights (color * intensity ≈ the mesh demo's magnitudes).
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

    // Sun — its shadow direction comes from the transform's rotation applied to
    // local -Y (matching the engine's light convention), tilted for a slanted cast.
    {
        entt::entity e = scene.CreateEntity("Sun");
        scene.Get<TransformComponent>(e).rotation =
            glm::quat(glm::radians(glm::vec3(55.0f, -25.0f, 0.0f)));
        auto& lc = scene.Add<LightComponent>(e);
        lc.type = LightType::Sun;  lc.color = { 1.0f, 0.97f, 0.9f };  lc.intensity = 3.0f;
    }

    // Hand the geometry to the renderer once (keyed by the StubMesh pointers).
    renderer->RegisterMesh(cubeMesh.get(),   MeshData::UnitCube());
    renderer->RegisterMesh(sphereMesh.get(), MeshData::UVSphere());

    // Orbit camera looking at the scene center.
    Camera camera;

    while (!glfwWindowShouldClose(window)) {
        glfwPollEvents();

        const float t = static_cast<float>(glfwGetTime());
        camera.Position = glm::vec3(std::sin(t * 0.3f) * 5.0f, 2.6f, std::cos(t * 0.3f) * 5.0f);
        camera.LookAt(glm::vec3(0.0f, 0.2f, 0.0f));

        // Build the UI, finalize its draw data, then let the renderer composite it
        // over the tonemapped scene via the swapchain overlay hook.
        imgui.BeginFrame();
        ImGui::ShowDemoWindow();
        ImGui::Render();
        renderer->RenderToSwapchain(scene, camera, [&](RHICommandList* cmd) {
            // 2D HUD quads first, then ImGui composited on top.
            r2d->SetCommandList(cmd);
            r2d->Begin(Renderer2D::OrthoProjection(1280.0f, 720.0f));
            // Solid quads (mode 0 over the 1x1 white texture)...
            r2d->DrawQuad({ 40.0f,  40.0f }, { 260.0f, 90.0f }, { 0.10f, 0.55f, 0.85f, 0.75f });
            r2d->DrawQuad({ 40.0f, 150.0f }, { 150.0f, 150.0f }, { 0.90f, 0.35f, 0.15f, 0.85f });
            // ...and a textured quad sampling the checkerboard — this exercises the
            // per-texture descriptor set + a second batch in one Begin/End.
            r2d->DrawTexturedQuad({ 210.0f, 150.0f }, { 150.0f, 150.0f }, *checker,
                                  glm::vec4(1.0f), glm::vec2(0.0f), glm::vec2(1.0f));
            // Interleave one more solid quad to prove batches re-order correctly
            // (white -> checker -> white = three runs, two texture switches).
            r2d->DrawQuad({ 40.0f, 320.0f }, { 320.0f, 40.0f }, { 0.20f, 0.85f, 0.45f, 0.85f });
            // Text (mode 1 — glyph coverage sampled from the font atlas). Shares the
            // same batch machinery; the atlas is just another per-texture run.
            if (font) {
                r2d->DrawText(*font, "DiamondEngine — Vulkan", { 44.0f, 384.0f },
                              { 1.0f, 1.0f, 1.0f, 1.0f }, 1.0f);
                r2d->DrawText(*font, "Renderer2D: quads + textures + text",
                              { 44.0f, 424.0f }, { 0.85f, 0.9f, 1.0f, 1.0f }, 0.7f);
            }
            r2d->End();
            imgui.Submit(cmd);
        });
    }

    device->WaitIdle();
    imgui.Shutdown();          // tears down ImGui's Vulkan objects (waits idle)
    r2d.reset();               // free RHI resources before the device (VMA allocator)
    checker.reset();           // ditto — its RHITexture is VMA-backed
    font.reset();              // its atlas is a VulkanTexture2D (VMA-backed)
    Texture::SetResourceDevice(nullptr);   // stop handing out the soon-dead device
    renderer.reset();
    device.reset();
    glfwDestroyWindow(window);
    glfwTerminate();
    return 0;
}

} // namespace Diamond
