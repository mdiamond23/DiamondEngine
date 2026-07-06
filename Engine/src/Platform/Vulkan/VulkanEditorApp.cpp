#include "Platform/VulkanDemo.h"

// The Sandbox editor's Vulkan mode (`--vulkan`). Lives inside the engine — like
// the demos — because the Vulkan backend's headers, defines, and volk include
// path are private to MyEngine; the app only sees RunVulkanEditor().
//
// This TU is compiled unconditionally: without the Vulkan backend it collapses
// to a stub (same pattern as SceneRenderer.cpp), so Sandbox links either way.

#ifdef DIAMOND_ENABLE_VULKAN

#include "Renderer/SceneRenderer.h"
#include "Renderer/RendererAPI.h"
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
#include "Platform/Vulkan/Resources/VulkanThumbnailRenderer.h"

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

void OnFramebufferResize(GLFWwindow* window, int /*w*/, int /*h*/) {
    auto* device = static_cast<RHIDevice*>(glfwGetWindowUserPointer(window));
    if (device) device->NotifyResize();
}

entt::entity SpawnMesh(Scene& scene, std::string_view name,
                       const std::shared_ptr<Mesh>& mesh,
                       const glm::vec3& pos, const glm::vec3& scale,
                       const std::shared_ptr<PBRMaterial>& material = nullptr,
                       const char* meshPath = "") {
    entt::entity e = scene.CreateEntity(name);
    auto& t = scene.Get<TransformComponent>(e);
    t.position = pos;
    t.scale    = scale;
    auto& mc = scene.Add<MeshComponent>(e);
    mc.mesh     = mesh;
    mc.material = material;
    mc.meshPath = meshPath;   // serializer token ("__cube"/"__sphere") or asset path
    return e;
}

// Slice 3's per-material proof: two real PBR texture sets from the asset library
// (loaded through Texture::Create, which is Vulkan-backed here). Diamond plate on
// the hard surfaces, emissive lava on the sphere; the floor gets its own material
// instance so the plate tiles instead of stretching (distinct uvScale = a second
// MaterialCache entry over the same textures).
struct SceneMaterials {
    std::shared_ptr<PBRMaterial> plate;
    std::shared_ptr<PBRMaterial> floorPlate;
    std::shared_ptr<PBRMaterial> lava;
};

SceneMaterials LoadMaterials() {
    const std::string plateDir = DIAMOND_ASSETS_DIR "/Materials/DiamondPlate006C_2K-PNG/";
    const std::string lavaDir  = DIAMOND_ASSETS_DIR "/Materials/Lava004_2K-PNG/";

    SceneMaterials m;
    m.plate = std::make_shared<PBRMaterial>();
    m.plate->Albedo    = Texture::Create(plateDir + "DiamondPlate006C_2K-PNG_Color.png");
    m.plate->Normal    = Texture::Create(plateDir + "DiamondPlate006C_2K-PNG_NormalGL.png");
    m.plate->Metallic  = Texture::Create(plateDir + "DiamondPlate006C_2K-PNG_Metalness.png");
    m.plate->Roughness = Texture::Create(plateDir + "DiamondPlate006C_2K-PNG_Roughness.png");
    m.plate->AO        = Texture::Create(plateDir + "DiamondPlate006C_2K-PNG_AmbientOcclusion.png");

    m.floorPlate  = std::make_shared<PBRMaterial>(*m.plate);   // shares the textures
    m.floorPlate->UVScale = 4.0f;

    m.lava = std::make_shared<PBRMaterial>();
    m.lava->Albedo           = Texture::Create(lavaDir + "Lava004_2K-PNG_Color.png");
    m.lava->Normal           = Texture::Create(lavaDir + "Lava004_2K-PNG_NormalGL.png");
    m.lava->Roughness        = Texture::Create(lavaDir + "Lava004_2K-PNG_Roughness.png");
    m.lava->Emissive         = Texture::Create(lavaDir + "Lava004_2K-PNG_Emission.png");
    m.lava->EmissiveStrength = 2.5f;
    m.lava->UVScale          = 2.0f;
    return m;
}

// The same lit test scene the VulkanScene demo uses (floor + sphere + cubes +
// lights + a spark emitter), plus a HUD canvas exercising every widget type.
void BuildScene(Scene& scene, const std::shared_ptr<Mesh>& cubeMesh,
                const std::shared_ptr<Mesh>& sphereMesh,
                const std::shared_ptr<Mesh>& quadMesh,
                const std::shared_ptr<Font>& font,
                const SceneMaterials& mats) {
    SpawnMesh(scene, "Floor",  cubeMesh,   { 0.0f, -0.55f, 0.0f }, { 6.0f, 0.1f, 6.0f }, mats.floorPlate, "__cube");
    SpawnMesh(scene, "Sphere", sphereMesh, { 0.0f,  0.3f,  0.0f }, { 0.8f, 0.8f, 0.8f }, mats.lava, "__sphere");
    SpawnMesh(scene, "CubeA",  cubeMesh,   {  1.8f, 0.0f,  0.0f }, { 0.6f, 0.6f, 0.6f }, mats.plate, "__cube");
    SpawnMesh(scene, "CubeB",  cubeMesh,   { -1.8f, 0.0f,  0.6f }, { 0.6f, 0.6f, 0.6f }, mats.plate, "__cube");
    SpawnMesh(scene, "CubeC",  cubeMesh,   {  0.4f, 0.0f, -1.9f }, { 0.6f, 0.6f, 0.6f }, nullptr, "__cube");   // default checkerboard

    // Glass windows for the forward transparency pass — the GL editor's window
    // quads, ECS-driven: a blue-glass 1×1 albedo (alpha 160/255), three panes at
    // staggered depths so the back-to-front sort is visibly exercised.
    {
        auto glass = std::make_shared<PBRMaterial>();
        const uint8_t glassPixel[4] = { 100, 180, 230, 160 };
        glass->Albedo = Texture::CreateFromPixels(glassPixel, 1, 1, 4);

        const glm::vec3 panePos[] = {
            { 0.0f, 0.35f, 2.2f }, { 1.3f, 0.40f, 1.0f }, { -1.5f, 0.55f, 1.5f } };
        for (int i = 0; i < 3; ++i) {
            entt::entity e = SpawnMesh(scene, "GlassPane" + std::to_string(i),
                                       quadMesh, panePos[i], glm::vec3(0.55f), glass);
            auto& mc = scene.Get<MeshComponent>(e);
            mc.transparent = true;
        }
    }

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

    // Primary game camera — drives the SceneRenderer's second render view (the
    // Game panel). A fixed low-angle framing, deliberately different from the
    // editor camera so the two images are visibly two views of one scene.
    {
        entt::entity e = scene.CreateEntity("GameCamera");
        auto& t = scene.Get<TransformComponent>(e);
        t.position = { -3.0f, 1.4f, 4.2f };
        const glm::vec3 target { 0.0f, 0.2f, 0.0f };
        t.rotation = glm::quatLookAt(glm::normalize(target - t.position),
                                     glm::vec3(0.0f, 1.0f, 0.0f));
        auto& cc = scene.Add<CameraComponent>(e);   // isPrimary defaults to true
        cc.fov = 55.0f;
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

    // Route the static factories to the Vulkan backend before any resource is
    // created: Mesh::Create hands out CPU-retained meshes the SceneRenderer
    // uploads lazily, and the Texture factory goes through this device so
    // font/UI bakes come out Vulkan-backed.
    RendererAPI::SetAPI(RendererAPI::API::Vulkan);
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

    // In-game UI batchers, recorded by the SceneRenderer's UI2D passes into the
    // RGBA8 scene outputs (not the swapchain — that's ImGui's). One per render
    // view: both overlays run in the same frame, and a Renderer2D's dynamic
    // vertex stream is written once per frame slot.
    auto r2d = std::make_unique<VulkanRenderer2D>(
        device.get(), DIAMOND_VULKAN_SHADER_DIR, RHIFormat::RGBA8);
    auto r2dGame = std::make_unique<VulkanRenderer2D>(
        device.get(), DIAMOND_VULKAN_SHADER_DIR, RHIFormat::RGBA8);

    Scene scene;
    // CPU-retained meshes (Vulkan mode): the SceneRenderer uploads each one
    // lazily the first frame it's drawn — no RegisterMesh calls needed.
    auto cubeMesh   = Mesh::Create(MeshData::UnitCube());
    auto sphereMesh = Mesh::Create(MeshData::UVSphere());
    auto quadMesh   = Mesh::Create(MeshData::FullscreenQuad());   // glass panes (transparency pass)
    const SceneMaterials mats = LoadMaterials();
    BuildScene(scene, cubeMesh, sphereMesh, quadMesh, font, mats);

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

    // Second render view: the scene's primary camera into the Game panel. Its
    // HUD overlay re-resolves the canvases at the game view's size and draws
    // them visual-only (interaction stays on the main viewport's pass, which
    // runs after this one and re-resolves at its own size).
    uint32_t gameW = sceneW, gameH = sceneH;   // created at the main view's size
    glm::vec2 gameUIScreen { (float)gameW, (float)gameH };
    renderer->SetGameViewEnabled(true);
    renderer->SetGameUIOverlay([&](RHICommandList* cmd) {
        r2dGame->SetCommandList(cmd);
        UISystem::Resolve(scene.GetRegistry(), gameUIScreen);
        UIRenderSystem::Render(scene.GetRegistry(), *r2dGame, gameUIScreen);
    });

    VulkanImGuiLayer imgui;
    imgui.Init(window, vkDevice);

    // Asset previews — one-shot bakes (mesh studio renders + material balls),
    // registered once with ImGui and shown in a Thumbnails panel. This is the
    // content-browser preview path proven end-to-end; the full ContentPanel
    // arrives with the editor-wiring slice.
    auto thumbs = std::make_unique<VulkanThumbnailRenderer>(
        device.get(), DIAMOND_VULKAN_SHADER_DIR);
    struct NamedThumb { const char* label; VkDescriptorSet set; };
    std::vector<NamedThumb> thumbSets;
    {
        auto add = [&](const char* label, VulkanThumbnailRenderer::Thumb t) {
            if (!t.view) return;
            thumbSets.push_back({ label, ImGui_ImplVulkan_AddTexture(
                t.sampler, t.view, VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL) });
        };
        add("Cube (mesh)",    thumbs->RenderMesh({ MeshData::UnitCube() }));
        add("Sphere (mesh)",  thumbs->RenderMesh({ MeshData::UVSphere() }));
        add("Diamond plate",  thumbs->RenderMaterial(*mats.plate));
        add("Lava (emissive)",thumbs->RenderMaterial(*mats.lava));
    }

    // One descriptor per frame-in-flight per scene image: the offscreen targets
    // are per-frame images internally, and each frame ImGui must sample the copy
    // the graph is writing this frame (transitioned to SHADER_READ_ONLY before
    // ImGui draws — by the EditorOverlay pass's read for the main viewport, by
    // the SceneRenderer's post-execute transition for the game view).
    using ImageSets = std::array<VkDescriptorSet, VulkanRHIDevice::kFramesInFlight>;
    auto registerSceneTexture = [&](ImageSets& sets, RHITexture* rhiTex) {
        auto* tex = static_cast<VulkanRHITexture*>(rhiTex);
        for (uint32_t i = 0; i < VulkanRHIDevice::kFramesInFlight; ++i) {
            if (sets[i]) ImGui_ImplVulkan_RemoveTexture(sets[i]);
            sets[i] = ImGui_ImplVulkan_AddTexture(
                tex->Sampler(), tex->View(i), VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL);
        }
    };
    ImageSets viewportSets{};
    ImageSets gameSets{};
    registerSceneTexture(viewportSets, renderer->OutputColor());
    registerSceneTexture(gameSets,     renderer->GameViewColor());

    Camera camera(glm::vec3(4.0f, 2.6f, 5.0f));
    camera.LookAt(glm::vec3(0.0f, 0.2f, 0.0f));

    // Debounced viewport resizes: panels resize every frame of a drag, but a
    // Resize is a WaitIdle + pass rebuild — only fire once the size has settled.
    glm::ivec2 wantSize { (int)sceneW, (int)sceneH };
    int        wantStableFrames = 0;
    glm::ivec2 gameWantSize { (int)gameW, (int)gameH };
    int        gameStableFrames = 0;
    constexpr int kResizeSettleFrames = 15;

    double lastTime = glfwGetTime();
    while (!glfwWindowShouldClose(window)) {
        glfwPollEvents();

        const double nowTime = glfwGetTime();
        const float  dt      = (float)(nowTime - lastTime);
        lastTime = nowTime;

        // Apply settled viewport resizes before any ImGui state references the
        // old targets (WaitIdle inside Resize makes the descriptor swap safe).
        if (((uint32_t)wantSize.x != sceneW || (uint32_t)wantSize.y != sceneH) &&
            wantStableFrames >= kResizeSettleFrames) {
            sceneW = (uint32_t)wantSize.x;
            sceneH = (uint32_t)wantSize.y;
            renderer->Resize(sceneW, sceneH);
            registerSceneTexture(viewportSets, renderer->OutputColor());
            uiScreen = { (float)sceneW, (float)sceneH };
        }
        if (((uint32_t)gameWantSize.x != gameW || (uint32_t)gameWantSize.y != gameH) &&
            gameStableFrames >= kResizeSettleFrames) {
            gameW = (uint32_t)gameWantSize.x;
            gameH = (uint32_t)gameWantSize.y;
            renderer->ResizeGameView(gameW, gameH);
            registerSceneTexture(gameSets, renderer->GameViewColor());
            gameUIScreen = { (float)gameW, (float)gameH };
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

        // Game panel — the second render view, from the scene's primary camera.
        ImGui::SetNextWindowSize(ImVec2(640, 360), ImGuiCond_FirstUseEver);
        if (ImGui::Begin("Game")) {
            const ImVec2 avail = ImGui::GetContentRegionAvail();
            const glm::ivec2 size { std::max((int)avail.x, 64), std::max((int)avail.y, 64) };
            if (size == gameWantSize) ++gameStableFrames;
            else                      { gameWantSize = size; gameStableFrames = 0; }

            // GameViewActive reflects the last rendered frame; on the frames it
            // is false (startup, no primary camera) the image holds no valid
            // frame, so show a hint instead — like the GL Game panel.
            if (renderer->GameViewActive()) {
                ImGui::Image((ImTextureID)(uintptr_t)gameSets[vkDevice->CurrentFrame()],
                             avail);
            } else {
                const char* msg = "No primary camera in scene";
                const ImVec2 textSz = ImGui::CalcTextSize(msg);
                ImGui::SetCursorPos({ (avail.x - textSz.x) * 0.5f,
                                      (avail.y - textSz.y) * 0.5f });
                ImGui::TextDisabled("%s", msg);
            }
        }
        ImGui::End();

        if (ImGui::Begin("Thumbnails")) {
            const float cell = (float)VulkanThumbnailRenderer::kSize;
            for (size_t i = 0; i < thumbSets.size(); ++i) {
                ImGui::BeginGroup();
                ImGui::Image((ImTextureID)(uintptr_t)thumbSets[i].set, ImVec2(cell, cell));
                ImGui::TextUnformatted(thumbSets[i].label);
                ImGui::EndGroup();
                if (i + 1 < thumbSets.size()) ImGui::SameLine();
            }
        }
        ImGui::End();

        if (ImGui::Begin("Stats")) {
            ImGui::Text("Backend: Vulkan (RTX-class, deferred)");
            ImGui::Text("FPS: %.1f (%.2f ms)", ImGui::GetIO().Framerate,
                        1000.0f / ImGui::GetIO().Framerate);
            ImGui::Text("Scene output: %ux%u", sceneW, sceneH);
            ImGui::Separator();
            // Global exposure (HDR scale before the ACES tonemap; 1.0 = GL parity).
            static float exposure = 1.0f;
            if (ImGui::SliderFloat("Exposure", &exposure, 0.1f, 2.0f, "%.2f"))
                renderer->SetExposure(exposure);
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
    imgui.Shutdown();          // frees the thumb + viewport/game descriptors with its pool
    thumbs.reset();            // then the baked images they pointed at
    r2dGame.reset();
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
