// DiamondEngine editor entry point: flag → backend → scene → loop.
//
// The loop below is backend-agnostic (editor-wiring step 3): everything that
// talks to a graphics API lives behind Diamond::EditorBackend — the GL
// implementation in Editor/GLEditorBackend, the Vulkan one inside the engine
// (CreateVulkanEditorBackend). `--vulkan` selects the Vulkan backend; the
// scene, editor layer, systems, audio, and ImGui build are shared.

#define GLFW_INCLUDE_NONE
#include <GLFW/glfw3.h>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/quaternion.hpp>
#include <imgui.h>
#include <ImGuizmo.h>
#include <IconsFontAwesome5.h>
#include <spdlog/spdlog.h>

#include "Editor/EditorBackend.h"
#include "Editor/GLEditorBackend.h"
#include "Editor/EditorLayers.h"

#include "Scene/Scene.h"
#include "Scene/Components.h"
#include "Scene/Physics/PhysicsAPI.h"
#include "Core/Camera.h"
#include "Core/Input.h"
#include "Scripts/AllScripts.h"
#include "Renderer/MeshData.h"
#include "Renderer/Material.h"
#include "Renderer/TextureData.h"
#include "Assets/ModelImporter.h"
#include "Assets/GltfImporter.h"
#include "Audio/AudioEngine.h"
#include "Audio/AudioAPI.h"
#include "Animation/AnimationComponents.h"
#include "Animation/AnimationSystem.h"
#include "Animation/IKSystem.h"
#include "Profiling/CPUProfiler.h"

#include <cmath>
#include <cstdio>
#include <cstring>
#include <memory>
#include <string>
#include <vector>

using namespace Diamond;

static Camera g_camera(glm::vec3(0.0f, 0.0f, 5.0f));
static bool   g_viewportActive = false;

static void processInput(GLFWwindow* window, float dt)
{
    if (glfwGetKey(window, GLFW_KEY_ESCAPE) == GLFW_PRESS)
        glfwSetWindowShouldClose(window, true);
    if (!g_viewportActive) return;
    if (glfwGetKey(window, GLFW_KEY_W) == GLFW_PRESS)
        g_camera.ProcessKeyboard(CameraMovement::Forward,  dt);
    if (glfwGetKey(window, GLFW_KEY_S) == GLFW_PRESS)
        g_camera.ProcessKeyboard(CameraMovement::Backward, dt);
    if (glfwGetKey(window, GLFW_KEY_A) == GLFW_PRESS)
        g_camera.ProcessKeyboard(CameraMovement::Left,     dt);
    if (glfwGetKey(window, GLFW_KEY_D) == GLFW_PRESS)
        g_camera.ProcessKeyboard(CameraMovement::Right,    dt);
}

// The default editor scene, built through the backend-routed factories
// (Mesh::Create / Texture::Create), so it loads identically under GL and
// Vulkan. Call only after the backend has initialized.
static void BuildDefaultScene(Scene& scene)
{
    // --- Materials ---
    static const std::string MAT =
        ASSETS_DIR "/Materials/DiamondPlate006C_2K-PNG/DiamondPlate006C_2K-PNG";
    auto diamondMaterial = std::make_shared<PBRMaterial>();
    diamondMaterial->Albedo    = Texture::Create(MAT + "_Color.png",            false);
    diamondMaterial->Normal    = Texture::Create(MAT + "_NormalGL.png",         false);
    diamondMaterial->Metallic  = Texture::Create(MAT + "_Metalness.png",        false);
    diamondMaterial->Roughness = Texture::Create(MAT + "_Roughness.png",        false);
    diamondMaterial->AO        = Texture::Create(MAT + "_AmbientOcclusion.png", false);
    diamondMaterial->AlbedoPath    = MAT + "_Color.png";
    diamondMaterial->NormalPath    = MAT + "_NormalGL.png";
    diamondMaterial->MetallicPath  = MAT + "_Metalness.png";
    diamondMaterial->RoughnessPath = MAT + "_Roughness.png";
    diamondMaterial->AOPath        = MAT + "_AmbientOcclusion.png";

    static const std::string LAVA = ASSETS_DIR "/Materials/Lava004_2K-PNG/Lava004_2K-PNG";
    auto lavaMaterial = std::make_shared<PBRMaterial>();
    lavaMaterial->Albedo          = Texture::Create(LAVA + "_Color.png",     false);
    lavaMaterial->Normal          = Texture::Create(LAVA + "_NormalGL.png",  false);
    lavaMaterial->Roughness       = Texture::Create(LAVA + "_Roughness.png", false);
    lavaMaterial->Emissive        = Texture::Create(LAVA + "_Emission.png",  false);
    lavaMaterial->EmissiveStrength = 2.5f;
    lavaMaterial->UVScale          = 5.0f;
    lavaMaterial->AlbedoPath    = LAVA + "_Color.png";
    lavaMaterial->NormalPath    = LAVA + "_NormalGL.png";
    lavaMaterial->RoughnessPath = LAVA + "_Roughness.png";
    lavaMaterial->EmissivePath  = LAVA + "_Emission.png";

    static const std::string CERB =
        ASSETS_DIR "/Models/Cerberus_by_Andrew_Maximov/Textures/Cerberus";
    auto cerberusMaterial = std::make_shared<PBRMaterial>();
    cerberusMaterial->Albedo    = Texture::Create(CERB + "_A.tga", false);
    cerberusMaterial->Normal    = Texture::Create(CERB + "_N.tga", false);
    cerberusMaterial->Metallic  = Texture::Create(CERB + "_M.tga", false);
    cerberusMaterial->Roughness = Texture::Create(CERB + "_R.tga", false);
    cerberusMaterial->AlbedoPath    = CERB + "_A.tga";
    cerberusMaterial->NormalPath    = CERB + "_N.tga";
    cerberusMaterial->MetallicPath  = CERB + "_M.tga";
    cerberusMaterial->RoughnessPath = CERB + "_R.tga";

    // --- Primitive geometry ---
    auto sphereData = MeshData::UVSphere();
    AABB sphereAABB = sphereData.ComputeAABB();
    auto sphere     = Mesh::Create(sphereData);

    auto cubeData = MeshData::UnitCube();
    AABB cubeAABB = cubeData.ComputeAABB();
    auto cube     = Mesh::Create(cubeData);

    // --- Entities ---
    {
        auto e = scene.CreateEntity("Sphere");
        auto& tc = scene.GetRegistry().get<TransformComponent>(e);
        tc.position = glm::vec3(-3.0f, -0.3f, 1.0f);
        scene.GetRegistry().emplace<MeshComponent>(e, sphere, diamondMaterial, sphereAABB).meshPath = "__sphere";
    }
    {
        auto e = scene.CreateEntity("Floor");
        auto& tc = scene.GetRegistry().get<TransformComponent>(e);
        tc.position = glm::vec3(0.0f, -1.7f, 0.0f);
        tc.scale    = glm::vec3(20.0f, 0.4f, 20.0f);
        scene.GetRegistry().emplace<MeshComponent>(e, cube, lavaMaterial, cubeAABB).meshPath = "__cube";
    }
    {
        struct CubeDesc { glm::vec3 pos; glm::vec3 scale; };
        CubeDesc cubes[] = {
            {{ 0.5f, -0.3f, -3.5f}, {1.0f, 1.0f, 1.0f}},
            {{ 3.5f,  0.7f,  1.0f}, {1.0f, 2.0f, 1.0f}},
            {{-4.5f,  1.7f, -2.0f}, {1.0f, 3.0f, 1.0f}},
            {{-1.5f, -0.3f,  3.5f}, {1.2f, 1.2f, 1.2f}},
            {{ 5.5f, -0.3f, -3.0f}, {1.0f, 1.0f, 1.0f}},
        };
        for (int i = 0; i < 5; ++i) {
            auto e = scene.CreateEntity("Cube " + std::to_string(i + 1));
            auto& tc = scene.GetRegistry().get<TransformComponent>(e);
            tc.position = cubes[i].pos;
            tc.scale    = cubes[i].scale;
            scene.GetRegistry().emplace<MeshComponent>(e, cube, diamondMaterial, cubeAABB).meshPath = "__cube";
        }
    }
    {
        auto cerberusMeshData = ModelImporter::Load(
            ASSETS_DIR "/Models/Cerberus_by_Andrew_Maximov/Revolver.FBX");
        std::vector<std::shared_ptr<Mesh>> cerberusGpuMeshes;
        for (auto& md : cerberusMeshData)
            cerberusGpuMeshes.push_back(Mesh::Create(md));

        for (int i = 0; i < (int)cerberusMeshData.size(); ++i) {
            auto e = scene.CreateEntity("Cerberus " + std::to_string(i));
            auto& tc = scene.GetRegistry().get<TransformComponent>(e);
            tc.position = glm::vec3(1.5f, 1.5f, 1.0f);
            tc.rotation = glm::angleAxis(glm::radians(-90.0f), glm::vec3(1.0f, 0.0f, 0.0f));
            tc.scale    = glm::vec3(0.05f);
            auto& mc = scene.GetRegistry().emplace<MeshComponent>(e,
                cerberusGpuMeshes[i], cerberusMaterial,
                cerberusMeshData[i].ComputeAABB());
            mc.meshPath     = ASSETS_DIR "/Models/Cerberus_by_Andrew_Maximov/Revolver.FBX";
            mc.meshSubIndex = i;
        }
    }
    {
        // Skinned-character test: CesiumMan walks via GPU skinning. (Renders
        // under GL only for now — Vulkan skinning is editor-wiring step 5.)
        const char* path = ASSETS_DIR "/Models/CesiumMan.glb";
        ImportedModel model = GltfImporter::LoadModel(path);
        if (!model.skeleton.bones.empty() && !model.meshes.empty()) {
            auto e = scene.CreateEntity("CesiumMan");
            auto& tc = scene.GetRegistry().get<TransformComponent>(e);
            tc.position = glm::vec3(-1.5f, 0.0f, 1.0f);

            auto& smc    = scene.GetRegistry().emplace<SkinnedMeshComponent>(e);
            smc.material = model.material ? model.material : diamondMaterial;
            smc.skeleton = std::move(model.skeleton);
            smc.clips    = std::move(model.animations);
            smc.meshPath = path;
            AABB bounds;
            for (auto& md : model.meshes) {
                smc.meshes.push_back(Mesh::Create(md));
                AABB b = md.ComputeAABB();
                bounds.min = glm::min(bounds.min, b.min);
                bounds.max = glm::max(bounds.max, b.max);
            }
            smc.localBounds = bounds;

            auto& anim = scene.GetRegistry().emplace<AnimatorComponent>(e);
            anim.clip  = 0;   // CesiumMan's single walk clip
            spdlog::info("CesiumMan loaded: {} bones, {} clips, {} submeshes",
                         smc.skeleton.bones.size(), smc.clips.size(), smc.meshes.size());
        }
    }
    {
        // Window quads — transparent MeshComponents drawn by the forward
        // transparency pass (blue glass; alpha 160/255). The quad is a -1..1
        // XY plane scaled into world space.
        auto glass = std::make_shared<PBRMaterial>();
        const uint8_t glassPixel[4] = { 100, 180, 230, 160 };
        glass->Albedo = Texture::CreateFromPixels(glassPixel, 1, 1, 4);

        auto windowQuad = Mesh::Create(MeshData::FullscreenQuad());
        AABB quadAABB   = MeshData::FullscreenQuad().ComputeAABB();
        const glm::vec3 panePos[] = {
            { 0.0f, 0.0f, 2.5f }, { 2.0f, 0.0f, 0.5f }, { -2.0f, 0.5f, 1.0f } };
        for (int i = 0; i < 3; ++i) {
            auto e = scene.CreateEntity("Window " + std::to_string(i + 1));
            auto& tc = scene.GetRegistry().get<TransformComponent>(e);
            tc.position = panePos[i];
            tc.scale    = glm::vec3(0.75f);
            auto& mc = scene.GetRegistry().emplace<MeshComponent>(e, windowQuad, glass, quadAABB);
            mc.transparent = true;
            mc.castsShadow = false;
        }
    }

    // --- Lights ---
    {
        struct PLDesc { const char* name; glm::vec3 pos; glm::vec3 color; float intensity; };
        PLDesc ptLights[] = {
            { "Point Light 1", { 1.0f, 3.0f,  2.0f}, {1.000f, 0.333f, 0.167f}, 600.0f },
            { "Point Light 2", {-5.0f, 2.0f,  4.0f}, {0.133f, 0.333f, 1.000f}, 600.0f },
            { "Point Light 3", { 8.0f, 4.0f, -6.0f}, {0.500f, 1.000f, 0.333f}, 600.0f },
            { "Point Light 4", {-8.0f, 4.0f, -6.0f}, {1.000f, 0.800f, 0.100f}, 500.0f },
        };
        for (auto& d : ptLights) {
            auto e = scene.CreateEntity(d.name);
            auto& tc = scene.GetRegistry().get<TransformComponent>(e);
            tc.position = d.pos;
            auto& lc = scene.GetRegistry().emplace<LightComponent>(e);
            lc.type      = LightType::Point;
            lc.color     = d.color;
            lc.intensity = d.intensity;
        }
    }
    {
        auto e = scene.CreateEntity("Sun");
        auto& tc = scene.GetRegistry().get<TransformComponent>(e);
        // Rotate so the local -Y axis points toward normalize(-0.5,-1,-0.3)
        glm::vec3 dir  = glm::normalize(glm::vec3(-0.5f, -1.0f, -0.3f));
        glm::vec3 axis = glm::cross(glm::vec3(0,-1,0), dir);
        float     len  = glm::length(axis);
        if (len > 1e-4f) {
            float angle = std::acos(glm::clamp(glm::dot(glm::vec3(0,-1,0), dir), -1.0f, 1.0f));
            tc.rotation = glm::angleAxis(angle, axis / len);
        }
        tc.eulerDegrees = glm::degrees(glm::eulerAngles(tc.rotation));
        auto& lc = scene.GetRegistry().emplace<LightComponent>(e);
        lc.type      = LightType::Sun;
        lc.color     = { 1.0f, 1.0f, 1.0f };
        lc.intensity = 3.0f;
    }
}

int main(int argc, char** argv)
{
    // Renderer backend flag. `--vulkan` runs the editor through the Vulkan
    // backend; macOS is GL-only (no MoltenVK) — ignore the flag there.
    bool wantVulkan = false;
    for (int i = 1; i < argc; ++i) {
        if (std::strcmp(argv[i], "--vulkan") == 0) {
#ifdef __APPLE__
            std::fprintf(stderr, "--vulkan is unsupported on macOS (GL only); "
                                 "continuing with OpenGL.\n");
#else
            wantVulkan = true;
#endif
        }
    }

    if (!glfwInit()) return -1;

    // ImGui context + fonts, shared by both backends. The backend's Init hooks
    // up the platform/renderer bindings; its Shutdown destroys the context.
    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImGui::GetIO().ConfigFlags |= ImGuiConfigFlags_DockingEnable;
    ImGui::StyleColorsDark();

    ImFont* iconFont = nullptr;
    {
        ImGuiIO& io = ImGui::GetIO();
        io.Fonts->AddFontDefault();
        static const ImWchar icon_ranges[] = { ICON_MIN_FA, ICON_MAX_FA, 0 };
        ImFontConfig icon_cfg;
        icon_cfg.PixelSnapH = true;
        iconFont = io.Fonts->AddFontFromFileTTF(FONTS_DIR "/fa-solid-900.ttf", 16.0f, &icon_cfg, icon_ranges);
    }

    std::unique_ptr<EditorBackend> backend;
    GLEditorBackend* glBackend = nullptr;   // for the GL-only editor extras
    if (wantVulkan) {
        backend = CreateVulkanEditorBackend();
        if (!backend) { glfwTerminate(); return 1; }   // factory already logged why
    } else {
        auto gl = std::make_unique<GLEditorBackend>();
        glBackend = gl.get();
        backend   = std::move(gl);
    }

    if (!backend->Init(1280, 720, "DiamondEngine")) {
        backend.reset();
        glfwTerminate();
        return -1;
    }
    GLFWwindow* window = backend->Window();

    Input::Init(window);

    // Audio backend (miniaudio). One engine for the app lifetime; the 3D listener
    // is driven from the active camera each frame and Update() reclaims finished
    // voices.
    AudioEngine audioEngine;
    audioEngine.Init();

    Scene scene;
    EditorLayer editorLayer(&scene, iconFont);
    editorLayer.SetEditorCamera(&g_camera);
    backend->SetDropHandler([&editorLayer](int count, const char** paths) {
        editorLayer.NotifyDroppedFiles(count, paths);
    });
    editorLayer.SetThumbnailService(backend->Thumbnails());
    editorLayer.SetBackendIsVulkan(wantVulkan);
    editorLayer.SetMaterialInvalidator([&backend](const PBRMaterial* mat) {
        backend->InvalidateMaterial(mat);
    });
    editorLayer.SetSceneCacheInvalidator([&backend] {
        backend->InvalidateSceneCaches();
    });
    if (glBackend) glBackend->AttachEditorLayer(&editorLayer);

    BuildDefaultScene(scene);

    // Point the 3D audio listener at whatever camera produced `v`. Derived from
    // the inverse view matrix: column 3 = position, -column 2 = forward, column
    // 1 = up.
    auto updateListener = [](const glm::mat4& v) {
        glm::mat4 inv = glm::inverse(v);
        Audio::SetListener(glm::vec3(inv[3]), -glm::vec3(inv[2]), glm::vec3(inv[1]));
    };

    // --- Editor loop (backend-agnostic) ---
    float deltaTime = 0.0f, lastFrame = 0.0f, fpsTimer = 0.0f;
    int   frameCount = 0;
    float smoothedFps = 0.0f;   // EMA, ~0.5s time constant (RendererStats::fps)

    while (!glfwWindowShouldClose(window))
    {
        const float now = (float)glfwGetTime();
        deltaTime = now - lastFrame;
        lastFrame = now;

        // EMA-smoothed FPS for the profiler panel (~0.5s time constant — stable
        // even through per-frame jitter, unlike the instantaneous 1/deltaTime).
        if (deltaTime > 0.0f) {
            const float instFps = 1.0f / deltaTime;
            const float alpha   = 1.0f - std::exp(-deltaTime / 0.5f);
            smoothedFps = (smoothedFps <= 0.0f) ? instFps
                                                 : smoothedFps + alpha * (instFps - smoothedFps);
        }

        // fps counter
        fpsTimer += deltaTime;
        frameCount++;
        if (fpsTimer >= 0.5f) {
            const int fps = (int)(frameCount / fpsTimer);
            std::string title = std::string("DiamondEngine (") + backend->Name() +
                                ") | FPS: " + std::to_string(fps);
            glfwSetWindowTitle(window, title.c_str());
            fpsTimer   = 0.0f;
            frameCount = 0;
        }

        processInput(window, deltaTime);

        // Skip rendering while minimized; keep pumping events.
        int fbW = 0, fbH = 0;
        glfwGetFramebufferSize(window, &fbW, &fbH);
        if (fbW == 0 || fbH == 0) {
            glfwPollEvents();
            continue;
        }

        // Gameplay input only reaches scripts during unpaused play with the
        // game viewport focused.
        Input::SetEnabled(scene.IsPlaying() && !scene.IsPaused() && editorLayer.IsGameViewportActive());
        scene.UpdateSystems(deltaTime);
        Input::SetEnabled(true);

        // Audio housekeeping: reclaim finished one-shot voices, enforce the voice cap.
        audioEngine.Update();

        // Update world transforms — rebuilds sorted arrays if hierarchy changed,
        // then linear pass.
        scene.GetTransformSystem().Update(scene.GetRegistry());

        // Drive animation state machines, then advance the animators they control
        // and rebuild bone palettes. Both gated on play mode (paused/editor holds
        // the current pose); the SM must run first so the animator advances the
        // clip the SM selected this frame.
        const bool animAdvance = scene.IsPlaying() && !scene.IsPaused();
        UpdateStateMachines(scene.GetRegistry(), deltaTime, animAdvance);
        UpdateAnimators(scene.GetRegistry(), deltaTime, animAdvance);

        // Procedural IK (hand reach / foot placement): pull end-effector bones to
        // their targets and blend into the pose UpdateAnimators just built, before
        // skinning consumes the palette. Skips entities whose ragdoll owns the pose.
        UpdateIK(scene, deltaTime);

        // Ragdoll readback: for any limp ragdoll, overwrite the palette
        // UpdateAnimators just produced with the simulated body transforms. Must
        // run after the physics step (in UpdateSystems above) and after
        // UpdateAnimators, so it wins.
        Physics::SyncRagdollPoses(scene);

        // --- Frame input for the backend ---
        EditorFrameInput frame;
        frame.scene        = &scene;
        frame.editorCamera = &g_camera;
        frame.dt           = deltaTime;
        frame.view         = g_camera.GetViewMatrix();
        frame.proj         = glm::perspective(glm::radians(g_camera.Zoom),
                                              (float)fbW / (float)fbH, 0.1f, 100.0f);
        frame.playing      = scene.IsPlaying();
        frame.showUI       = editorLayer.GetContext().showUI;
        frame.gameMouseNorm = editorLayer.GetContext().gameViewportMouseNorm;
        frame.showDebugDraw  = editorLayer.GetContext().showDebugDraw;
        frame.selectedEntity = editorLayer.GetContext().PrimarySelection();
        frame.activeIKChain  = editorLayer.GetContext().activeIKChain;

        const entt::entity primaryCam = scene.GetPrimaryCamera();
        frame.wantGameView = frame.playing && primaryCam != entt::null;
        if (frame.wantGameView) {
            auto& cc = scene.Get<CameraComponent>(primaryCam);
            const glm::mat4 camW = scene.GetTransformSystem().GetWorldMatrix(primaryCam);
            frame.gameView = glm::inverse(camW);
            frame.gameProj = glm::perspective(glm::radians(cc.fov),
                                              (float)fbW / (float)fbH,
                                              cc.nearClip, cc.farClip);
        }

        // Audio listener: edit mode auditions from the editor camera. In play
        // mode the scene owns it (AudioListenerComponent via AudioSystem), or
        // the game camera as fallback so spatial audio tracks what the player
        // actually hears from.
        if (!scene.IsPlaying()) {
            updateListener(frame.view);
        } else if (frame.wantGameView) {
            bool sceneHasListener = false;
            for (auto le : scene.View<AudioListenerComponent>()) { (void)le; sceneHasListener = true; break; }
            if (!sceneHasListener) updateListener(frame.gameView);
        }

        // --- ImGui build ---
        backend->BeginImGuiFrame();
        ImGuizmo::BeginFrame();

        {
            const EditorViewImage img = backend->SceneViewImage();
            editorLayer.SetViewportTexture((ImTextureID)img.textureId, img.flipY);
        }
        {
            const EditorViewImage img = backend->GameViewImage();
            editorLayer.SetGameViewportTexture((ImTextureID)img.textureId, img.flipY);
        }
        // Vulkan only: GL drives ParticlePreviewPanel directly (AttachEditorLayer)
        // and would have its real texture from last frame's RenderFrame clobbered
        // by this default no-op fetch.
        if (!glBackend) {
            const EditorViewImage img = backend->ParticlePreviewImage();
            editorLayer.GetParticlePreviewPanel().SetTexture((ImTextureID)img.textureId, img.flipY);
        }
        {
            DIAMOND_PROFILE_SCOPE("Editor ImGui");
            editorLayer.UpdateCamera(frame.view, frame.proj, g_camera.Position);
            editorLayer.OnImGuiRender();
            backend->OnImGuiPanels();
        }

        // Vulkan only: gather the particle-preview panel's per-frame state into
        // engine-visible types (RenderParticle array) so the Vulkan backend can
        // render it without depending on this Sandbox-side panel type. DesiredSize
        // is only valid after OnImGuiRender ran the panel this frame. GL instead
        // ticks and renders the panel itself inside GLEditorBackend::RenderFrame.
        if (!glBackend) {
            auto& preview = editorLayer.GetParticlePreviewPanel();
            preview.Tick(deltaTime);
            frame.previewActive  = preview.HasParticles();
            frame.previewSize    = preview.DesiredSize();
            frame.previewBgColor = preview.BackgroundColor();
            frame.previewView    = preview.ViewMatrix();
            frame.previewProj    = preview.ProjMatrix();
            frame.previewParticles.clear();
            frame.previewTexture = nullptr;
            if (frame.previewActive) {
                const auto& em = preview.Emitter();
                frame.previewParticles.reserve(em.liveCount);
                for (std::size_t i = 0; i < em.liveCount; ++i) {
                    const auto& p = em.pool[i];
                    frame.previewParticles.push_back({ p.pos, p.size, p.color, p.rotation });
                }
                frame.previewTexture = em.texture.get();
                frame.previewBlend   = em.blend;
            }
        }

        // Read viewport active state and drive camera from ImGui mouse delta.
        // io.MouseDelta is valid here (after NewFrame, before Render).
        g_viewportActive = editorLayer.IsViewportActive();
        if (g_viewportActive) {
            ImGuiIO& io = ImGui::GetIO();
            g_camera.ProcessMouseMovement(io.MouseDelta.x, -io.MouseDelta.y);
            g_camera.ProcessMouseScroll(io.MouseWheel);
            // Fly moved the camera directly — re-anchor the orbit pivot so
            // MMB orbit/pan resume from here instead of easing back.
            g_camera.SyncOrbitTargets();
        }

        {
            // Wall time of record + submit + present — at vsync this includes
            // the swap wait, so it reads large on an idle frame by design.
            DIAMOND_PROFILE_SCOPE("Render Submit");
            ImGui::Render();
            backend->RenderFrame(frame);
        }

        // Flip the CPU profiler's frame: merge every thread's scope tree into
        // the snapshot ProfilerPanel reads next frame via GetSnapshot().
        CPUProfiler::EndFrame();

        // Renderer counters are this backend's last-completed frame; FPS/CPU ms
        // are the app loop's own measurements, merged in here (Docs/profiler-
        // panel-design.md — neither backend owns timing). Feeds next frame's
        // ProfilerPanel draw, one frame latent like GetStats() itself already is.
        RendererStats stats = backend->GetStats();
        stats.fps        = smoothedFps;
        stats.cpuFrameMs = deltaTime * 1000.0f;
        editorLayer.SetStats(stats);

        Input::Update();
        glfwPollEvents();
    }

    audioEngine.Shutdown();
    backend->Shutdown();   // also destroys the ImGui context
    backend.reset();

    glfwTerminate();
    return 0;
}
