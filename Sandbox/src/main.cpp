#include <glad/gl.h>
#define GLFW_INCLUDE_NONE
#include <GLFW/glfw3.h>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/quaternion.hpp>
#include <vector>
#include <unordered_map>
#include <string>
#include <imgui.h>
#include <backends/imgui_impl_glfw.h>
#include <backends/imgui_impl_opengl3.h>
#include <ImGuizmo.h>
#include "Editor/EditorLayers.h"
#include "Scene/Scene.h"
#include "Scene/Components.h"
#include "Scene/Physics/PhysicsSystem.h"
#include "Scene/Physics/PhysicsAPI.h"
#include "DebugDraw.h"

#include "Core/Camera.h"
#include "Core/Input.h"
#include "Scripts/AllScripts.h"
#include <IconsFontAwesome5.h>
#include "Renderer/MeshData.h"
#include "Renderer/Material.h"
#include "Renderer/TextureData.h"
#include "Renderer/RenderGraph.h"
#include "Platform/OpenGL/Resources/OpenGLRenderTypes.h"
#include "Platform/OpenGL/Resources/OpenGLShader.h"
#include "Platform/OpenGL/Resources/OpenGLTexture.h"
#include "Platform/OpenGL/Passes/IBL/OpenGLIBLPass.h"
#include "Platform/OpenGL/Passes/Shadows/OpenGLShadowPass.h"
#include "Platform/OpenGL/Passes/Shadows/OpenGLCSMPass.h"
#include "Platform/OpenGL/Passes/Deferred/OpenGLGBufferPass.h"
#include "Platform/OpenGL/Passes/Deferred/OpenGLSSAOPass.h"
#include "Platform/OpenGL/Passes/Deferred/OpenGLDeferredLightingPass.h"
#include "Platform/OpenGL/Passes/PostProcess/OpenGLTonemapPass.h"
#include "Platform/OpenGL/Passes/PostProcess/OpenGLBloomPass.h"
#include "Platform/OpenGL/Passes/PostProcess/OpenGLFXAAPass.h"
#include "Renderer/Frustum.h"
#include "Platform/OpenGL/Passes/Forward/OpenGLTransparencyPass.h"
#include "Assets/ModelImporter.h"
#include "Assets/GltfImporter.h"
#include "Animation/AnimationComponents.h"
#include "Animation/AnimationSystem.h"

using namespace Diamond;

static Camera g_camera(glm::vec3(0.0f, 0.0f, 5.0f));
static float  g_deltaTime = 0.0f, g_lastFrame = 0.0f;
static bool   g_viewportActive = false;

// timing
float deltaTime = 0.0f;
float lastFrame = 0.0f;
float fpsTimer = 0.0f;
int frameCount = 0;

static void framebufferSizeCallback(GLFWwindow*, int w, int h) { glViewport(0, 0, w, h); }

static void dropCallback(GLFWwindow* window, int count, const char** paths) {
    auto* layer = static_cast<EditorLayer*>(glfwGetWindowUserPointer(window));
    if (layer) layer->NotifyDroppedFiles(count, paths);
}

static void processInput(GLFWwindow* window)
{
    if (glfwGetKey(window, GLFW_KEY_ESCAPE) == GLFW_PRESS)
        glfwSetWindowShouldClose(window, true);
    if (!g_viewportActive) return;
    if (glfwGetKey(window, GLFW_KEY_W) == GLFW_PRESS)
        g_camera.ProcessKeyboard(CameraMovement::Forward,  g_deltaTime);
    if (glfwGetKey(window, GLFW_KEY_S) == GLFW_PRESS)
        g_camera.ProcessKeyboard(CameraMovement::Backward, g_deltaTime);
    if (glfwGetKey(window, GLFW_KEY_A) == GLFW_PRESS)
        g_camera.ProcessKeyboard(CameraMovement::Left,     g_deltaTime);
    if (glfwGetKey(window, GLFW_KEY_D) == GLFW_PRESS)
        g_camera.ProcessKeyboard(CameraMovement::Right,    g_deltaTime);
}

int main()
{
    glfwInit();
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 4);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 1);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
#ifdef __APPLE__
    glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE);
#endif

    GLFWwindow* window = glfwCreateWindow(1280, 720, "DiamondEngine", nullptr, nullptr);
    if (!window) { glfwTerminate(); return -1; }

    glfwMakeContextCurrent(window);
    glfwSwapInterval(1);
    glfwSetInputMode(window, GLFW_CURSOR, GLFW_CURSOR_NORMAL);
    glfwSetFramebufferSizeCallback(window, framebufferSizeCallback);

    if (!gladLoadGL(glfwGetProcAddress)) { glfwTerminate(); return -1; }

    Input::Init(window);

    // ImGui init
    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImGuiIO& io = ImGui::GetIO();
    io.ConfigFlags |= ImGuiConfigFlags_DockingEnable;
    ImGui::StyleColorsDark();
    ImGui_ImplGlfw_InitForOpenGL(window, true);
    ImGui_ImplOpenGL3_Init("#version 410");

    ImFont* iconFont = nullptr;
    {
        ImGuiIO& io = ImGui::GetIO();
        io.Fonts->AddFontDefault();
        static const ImWchar icon_ranges[] = { ICON_MIN_FA, ICON_MAX_FA, 0 };
        ImFontConfig icon_cfg;
        icon_cfg.PixelSnapH = true;
        iconFont = io.Fonts->AddFontFromFileTTF(FONTS_DIR "/fa-solid-900.ttf", 16.0f, &icon_cfg, icon_ranges);
    }

    Scene scene;
    EditorLayer editorLayer(&scene, iconFont);
    editorLayer.SetEditorCamera(&g_camera);
    glfwSetWindowUserPointer(window, &editorLayer);
    glfwSetDropCallback(window, dropCallback);

    glEnable(GL_DEPTH_TEST);
    glEnable(GL_TEXTURE_CUBE_MAP_SEAMLESS);

    // --- Shaders ---
    OpenGLShader gbufferShader(
        ENGINE_SHADERS_DIR "/Deferred/gbuffer.vert",
        ENGINE_SHADERS_DIR "/Deferred/gbuffer.frag");
    // Skinned variant — same fragment stage, GPU vertex skinning.
    OpenGLShader gbufferSkinnedShader(
        ENGINE_SHADERS_DIR "/Deferred/gbuffer_skinned.vert",
        ENGINE_SHADERS_DIR "/Deferred/gbuffer.frag");
    OpenGLShader ssaoShader(
        ENGINE_SHADERS_DIR "/Deferred/quad.vert",
        ENGINE_SHADERS_DIR "/Deferred/ssao.frag");
    OpenGLShader ssaoBlurShader(
        ENGINE_SHADERS_DIR "/Deferred/quad.vert",
        ENGINE_SHADERS_DIR "/Deferred/ssao_blur.frag");
    OpenGLShader lightingShader(
        ENGINE_SHADERS_DIR "/Deferred/quad.vert",
        ENGINE_SHADERS_DIR "/Deferred/lighting.frag");
    OpenGLShader equirectShader(
        ENGINE_SHADERS_DIR "/IBL/cubemap.vert",
        ENGINE_SHADERS_DIR "/IBL/equirect_to_cubemap.frag");
    OpenGLShader irradianceShader(
        ENGINE_SHADERS_DIR "/IBL/cubemap.vert",
        ENGINE_SHADERS_DIR "/IBL/irradiance.frag");
    OpenGLShader prefilterShader(
        ENGINE_SHADERS_DIR "/IBL/cubemap.vert",
        ENGINE_SHADERS_DIR "/IBL/prefilter.frag");
    OpenGLShader brdfShader(
        ENGINE_SHADERS_DIR "/IBL/brdf.vert",
        ENGINE_SHADERS_DIR "/IBL/brdf.frag");
    OpenGLShader backgroundShader(
        ENGINE_SHADERS_DIR "/IBL/background.vert",
        ENGINE_SHADERS_DIR "/IBL/background.frag");
    OpenGLShader blurShader(
        ENGINE_SHADERS_DIR "/Bloom/blur.vert",
        ENGINE_SHADERS_DIR "/Bloom/blur.frag");
    OpenGLShader bloomFinalShader(
        ENGINE_SHADERS_DIR "/Bloom/bloomfinal.vert",
        ENGINE_SHADERS_DIR "/Bloom/bloomfinal.frag");
    OpenGLShader fxaaShader(
        ENGINE_SHADERS_DIR "/PostProcess/tonemap.vert",
        ENGINE_SHADERS_DIR "/PostProcess/fxaa.frag");
    OpenGLShader depthShader(
        ENGINE_SHADERS_DIR "/Shadows/depth.vert",
        ENGINE_SHADERS_DIR "/Shadows/depth.frag");
    OpenGLShader depthSkinnedShader(
        ENGINE_SHADERS_DIR "/Shadows/depth_skinned.vert",
        ENGINE_SHADERS_DIR "/Shadows/depth.frag");
    OpenGLShader pointDepthShader(
        ENGINE_SHADERS_DIR "/Shadows/point_depth.vert",
        ENGINE_SHADERS_DIR "/Shadows/point_depth.geom",
        ENGINE_SHADERS_DIR "/Shadows/point_depth.frag");
    OpenGLShader pointDepthSkinnedShader(
        ENGINE_SHADERS_DIR "/Shadows/point_depth_skinned.vert",
        ENGINE_SHADERS_DIR "/Shadows/point_depth.geom",
        ENGINE_SHADERS_DIR "/Shadows/point_depth.frag");
    OpenGLShader transparencyShader(
        ENGINE_SHADERS_DIR "/Forward/transparent.vert",
        ENGINE_SHADERS_DIR "/Forward/transparent.frag");

    // --- Pass objects ---
    OpenGLIBLPass              iblPass;
    OpenGLShadowPass           shadowPass;
    OpenGLCSMPass              csmPass;
    OpenGLGBufferPass          gbufferPass;
    OpenGLSSAOPass             ssaoPass;
    OpenGLDeferredLightingPass lightingPass;
    OpenGLBloomPass            bloomPass;
    OpenGLFXAAPass             fxaaPass;
    OpenGLTransparencyPass     transparencyPass;
    bool fxaaEnabled = true;

    // IBL bake — one-time
    {
        OpenGLTexture envTex(ASSETS_DIR "/Textures/citrus_orchard_road_puresky_4k.hdr", true, true);
        iblPass.BakeEnvironment(envTex, equirectShader, irradianceShader,
                                prefilterShader, brdfShader);
    }

    // Shadow maps — persistent, not managed by the graph
    csmPass.Setup(2048);
    shadowPass.SetupPointShadowMaps(4, 512, 25.0f);

    // --- Scene geometry ---
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

    auto sphereData = MeshData::UVSphere();
    AABB sphereAABB = sphereData.ComputeAABB();
    auto sphere     = Mesh::Create(sphereData);

    auto cubeData = MeshData::UnitCube();
    AABB cubeAABB = cubeData.ComputeAABB();
    auto cube     = Mesh::Create(cubeData);

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

    // --- Populate scene with entities ---
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
        // Skinned-character test: CesiumMan walks via GPU skinning.
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
            Diamond::AABB bounds;
            for (auto& md : model.meshes) {
                smc.meshes.push_back(Mesh::Create(md));
                Diamond::AABB b = md.ComputeAABB();
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

    // Window quads — FullscreenQuad is a -1..1 XY plane, scaled/translated into world space
    auto windowQuad = Mesh::Create(MeshData::FullscreenQuad());
    std::vector<DrawCall> windowDraws = {
        { windowQuad.get(), glm::scale(glm::translate(glm::mat4(1.0f), { 0.0f,  0.0f, 2.5f}), glm::vec3(0.75f)) },
        { windowQuad.get(), glm::scale(glm::translate(glm::mat4(1.0f), { 2.0f,  0.0f, 0.5f}), glm::vec3(0.75f)) },
        { windowQuad.get(), glm::scale(glm::translate(glm::mat4(1.0f), {-2.0f,  0.5f, 1.0f}), glm::vec3(0.75f)) },
    };

    // 1x1 blue-glass RGBA texture — replace with a real window PNG when available
    uint32_t windowTex;
    glGenTextures(1, &windowTex);
    glBindTexture(GL_TEXTURE_2D, windowTex);
    uint8_t windowPixel[4] = { 100, 180, 230, 160 };
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA8, 1, 1, 0, GL_RGBA, GL_UNSIGNED_BYTE, windowPixel);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
    glBindTexture(GL_TEXTURE_2D, 0);

    // --- Create light entities (replaces hardcoded arrays) ---
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

    // Per-frame light data gathered from ECS each frame
    std::vector<glm::vec3> pointPositions;
    std::vector<glm::vec3> pointColors;
    std::vector<glm::vec3> spotPositions;
    std::vector<glm::vec3> spotDirs;
    std::vector<glm::vec3> spotColors;
    std::vector<float>     spotCosInner;
    std::vector<float>     spotCosOuter;
    Diamond::SunLight      sun;

    // --- Per-frame state captured by render graph lambdas ---
    std::vector<DrawCall> allDraws;
    std::vector<DrawCall> shadowDraws;
    std::unordered_map<PBRMaterial*, std::vector<DrawCall>> culledBatches;
    glm::mat4 view, proj;
    int fbW = 0, fbH = 0;

    // --- Render graph ---
    RenderGraph     graph;
    RGTextureHandle gViewPos, gViewNormal, gAlbedo, gMaterial, gEmissive;
    RGTextureHandle hSsaoRaw, hSsaoBlur;
    RGTextureHandle hdrBuffer, brightBuffer;
    RGTextureHandle pingPong[2];
    RGTextureHandle ldrBuffer;

    // Viewport FBOs — managed outside the render graph so they aren't freed between frames
    uint32_t viewportFBO      = 0;
    uint32_t viewportTexture  = 0;
    uint32_t gameViewportFBO  = 0;
    uint32_t gameViewportTex  = 0;
    uint32_t outputFBO        = 0;  // set before each graph.Execute() to select render target

    auto buildGraph = [&](int w, int h)
    {
        fbW = w; fbH = h;
        graph.Clear();

        // G-buffer — 5 attachments sharing one FBO (gViewPos owns it)
        gViewPos    = graph.DeclareTexture("gViewPos",    { w, h, GL_RGBA16F, true  });
        gViewNormal = graph.DeclareTexture("gViewNormal", { w, h, GL_RGBA16F, false, gViewPos, 1 });
        gAlbedo     = graph.DeclareTexture("gAlbedo",     { w, h, GL_RGBA8,   false, gViewPos, 2 });
        gMaterial   = graph.DeclareTexture("gMaterial",   { w, h, GL_RGBA8,   false, gViewPos, 3 });
        gEmissive   = graph.DeclareTexture("gEmissive",   { w, h, GL_RGB16F,  false, gViewPos, 4 });

        // SSAO intermediates
        hSsaoRaw  = graph.DeclareTexture("ssaoRaw",  { w, h, GL_R16F, false });
        hSsaoBlur = graph.DeclareTexture("ssaoBlur", { w, h, GL_R16F, false });

        // HDR + bloom extract (MRT)
        hdrBuffer    = graph.DeclareTexture("hdrBuffer",    { w, h, GL_RGBA16F, true  });
        brightBuffer = graph.DeclareTexture("brightBuffer", { w, h, GL_RGBA16F, false, hdrBuffer, 1 });

        // Bloom ping-pong
        pingPong[0] = graph.DeclareTexture("bloomPing", { w, h, GL_RGBA16F, false });
        pingPong[1] = graph.DeclareTexture("bloomPong", { w, h, GL_RGBA16F, false });

        // LDR intermediate — bloom composite writes here, FXAA reads and outputs to viewport
        ldrBuffer = graph.DeclareTexture("ldrBuffer", { w, h, GL_RGBA8, true });

        // Viewport FBOs — recreate on resize
        auto makeViewportFBO = [](uint32_t& fbo, uint32_t& tex, int w, int h) {
            if (tex) glDeleteTextures(1,      &tex);
            if (fbo) glDeleteFramebuffers(1,  &fbo);
            glGenTextures(1, &tex);
            glBindTexture(GL_TEXTURE_2D, tex);
            glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA8, w, h, 0, GL_RGBA, GL_UNSIGNED_BYTE, nullptr);
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
            glGenFramebuffers(1, &fbo);
            glBindFramebuffer(GL_FRAMEBUFFER, fbo);
            glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, tex, 0);
            glBindFramebuffer(GL_FRAMEBUFFER, 0);
        };
        makeViewportFBO(viewportFBO,     viewportTexture, w, h);
        makeViewportFBO(gameViewportFBO, gameViewportTex, w, h);

        // Shadow passes — sinks (no graph writes), always alive
        graph.AddPass("DirectionalShadow")
            .SetExecute([&]{
                csmPass.Render(depthShader, depthSkinnedShader, shadowDraws, sun, view, proj, 0.1f, 100.0f);
            });

        graph.AddPass("PointShadow")
            .SetExecute([&]{
                shadowPass.RenderPointShadowPass(pointDepthShader, pointDepthSkinnedShader, shadowDraws,
                    pointPositions.data(), (int)pointPositions.size());
            });

        // Geometry pass — fills all 5 G-buffer attachments
        graph.AddPass("GBuffer")
            .Write(gViewPos).Write(gViewNormal).Write(gAlbedo).Write(gMaterial).Write(gEmissive)
            .SetExecute([&]{
                // Clear once, then render all material batches into the same FBO
                glBindFramebuffer(GL_FRAMEBUFFER, graph.GetFBO(gViewPos));
                glClearColor(0.0f, 0.0f, 0.0f, 1.0f);
                glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

                for (auto& [mat, batch] : culledBatches)
                    if (!batch.empty())
                        gbufferPass.Render(gbufferShader, gbufferSkinnedShader, batch, *mat, view, proj, graph.GetFBO(gViewPos), fbW, fbH);
            });

        // SSAO — raw occlusion
        graph.AddPass("SSAO")
            .Read(gViewPos).Read(gViewNormal)
            .Write(hSsaoRaw)
            .SetExecute([&]{
                ssaoPass.RenderSSAO(ssaoShader,
                                    graph.GetTexture(gViewPos),
                                    graph.GetTexture(gViewNormal),
                                    proj,
                                    graph.GetFBO(hSsaoRaw),
                                    fbW, fbH);
            });

        // SSAO blur
        graph.AddPass("SSAOBlur")
            .Read(hSsaoRaw)
            .Write(hSsaoBlur)
            .SetExecute([&]{
                ssaoPass.RenderBlur(ssaoBlurShader,
                                    graph.GetTexture(hSsaoRaw),
                                    graph.GetFBO(hSsaoBlur),
                                    fbW, fbH);
            });

        // Deferred lighting + skybox
        graph.AddPass("DeferredLighting")
            .Read(gViewPos).Read(gViewNormal).Read(gAlbedo).Read(gMaterial)
            .Read(hSsaoBlur).Read(gEmissive)
            .Write(hdrBuffer).Write(brightBuffer)
            .SetExecute([&]{
                lightingPass.Render(lightingShader,
                                    graph.GetTexture(gViewPos),
                                    graph.GetTexture(gViewNormal),
                                    graph.GetTexture(gAlbedo),
                                    graph.GetTexture(gMaterial),
                                    graph.GetTexture(hSsaoBlur),
                                    graph.GetTexture(gEmissive),
                                    view, g_camera.Position,
                                    pointPositions, pointColors,
                                    spotPositions, spotDirs, spotColors,
                                    spotCosInner, spotCosOuter, sun,
                                    shadowPass, csmPass, iblPass,
                                    graph.GetFBO(hdrBuffer),
                                    fbW, fbH);

                // Blit G-buffer depth → HDR FBO so the skybox can depth-test
                glBindFramebuffer(GL_READ_FRAMEBUFFER, graph.GetFBO(gViewPos));
                glBindFramebuffer(GL_DRAW_FRAMEBUFFER, graph.GetFBO(hdrBuffer));
                glBlitFramebuffer(0, 0, fbW, fbH, 0, 0, fbW, fbH,
                                  GL_DEPTH_BUFFER_BIT, GL_NEAREST);

                // Skybox renders into the HDR FBO using the blitted depth
                glBindFramebuffer(GL_FRAMEBUFFER, graph.GetFBO(hdrBuffer));
                lightingPass.DrawSkybox(backgroundShader, iblPass, view, proj, fbW, fbH);
            });

        // Forward transparency — sorts window draws back-to-front, blits depth internally
        graph.AddPass("Transparency")
            .Read(gViewPos)
            .Read(hdrBuffer)
            .SetExecute([&]{
                transparencyPass.Render(transparencyShader, windowDraws, windowTex,
                                        graph.GetFBO(gViewPos),
                                        graph.GetFBO(hdrBuffer),
                                        view, proj, g_camera.Position,
                                        fbW, fbH);
            });

        // Bloom blur ping-pong
        graph.AddPass("BloomBlur")
            .Read(brightBuffer)
            .Write(pingPong[0]).Write(pingPong[1])
            .SetExecute([&]{
                bool horizontal = true;
                uint32_t src = graph.GetTexture(brightBuffer);
                for (int i = 0; i < 6; ++i) {
                    int dst = i % 2;
                    bloomPass.RenderBlurPass(blurShader, src, graph.GetFBO(pingPong[dst]), horizontal);
                    src        = graph.GetTexture(pingPong[dst]);
                    horizontal = !horizontal;
                }
            });

        // Composite + tonemap → LDR intermediate
        graph.AddPass("BloomComposite")
            .Read(hdrBuffer).Read(pingPong[1])
            .Write(ldrBuffer)
            .SetExecute([&]{
                bloomPass.RenderCompositePass(bloomFinalShader,
                    graph.GetTexture(hdrBuffer),
                    graph.GetTexture(pingPong[1]),
                    true, 0.1f,
                    graph.GetFBO(ldrBuffer));
            });

        // FXAA — sink pass, renders into whichever FBO outputFBO points to
        graph.AddPass("FXAA")
            .Read(ldrBuffer)
            .SetExecute([&]{
                fxaaPass.Render(fxaaShader,
                    graph.GetTexture(ldrBuffer),
                    fbW, fbH, fxaaEnabled,
                    outputFBO);
            });

        graph.Compile();
    };

    // --- Render loop ---
    int lastFbW = 0, lastFbH = 0;

    while (!glfwWindowShouldClose(window))
    {
        // per-frame time logic
        // --------------------
        float currentFrame = static_cast<float>(glfwGetTime());
        deltaTime = currentFrame - lastFrame;
        lastFrame = currentFrame;

        // fps counter
        fpsTimer += deltaTime;
        frameCount++;
        if (fpsTimer >= 0.5f) {
            std::string title = "LearnOpenGL | FPS: " + std::to_string((int)(frameCount / fpsTimer));
            glfwSetWindowTitle(window, title.c_str());
            fpsTimer = 0.0f;
            frameCount = 0;
        }

        float now   = static_cast<float>(glfwGetTime());
        g_deltaTime = now - g_lastFrame;
        g_lastFrame = now;

        processInput(window);

        glfwGetFramebufferSize(window, &fbW, &fbH);

        if (fbW != lastFbW || fbH != lastFbH) {
            buildGraph(fbW, fbH);
            lastFbW = fbW;
            lastFbH = fbH;
        }

        Input::SetEnabled(scene.IsPlaying() && !scene.IsPaused() && editorLayer.IsGameViewportActive());
        scene.UpdateSystems(deltaTime);
        Input::SetEnabled(true);

        // Update world transforms — rebuilds sorted arrays if hierarchy changed, then linear pass.
        scene.GetTransformSystem().Update(scene.GetRegistry());

        // Advance skinned-mesh animators and rebuild their bone palettes for this frame.
        Diamond::UpdateAnimators(scene.GetRegistry(), deltaTime);

        // Build draw calls from scene — uses world matrices so parented entities render correctly.
        allDraws.clear();
        std::unordered_map<PBRMaterial*, std::vector<DrawCall>> materialBatches;
        for (auto [entity, mc] : scene.GetRegistry().view<MeshComponent>().each()) {
            if (!mc.mesh || !mc.material || !mc.visible) continue;
            DrawCall dc{ mc.mesh.get(), scene.GetTransformSystem().GetWorldMatrix(entity), mc.localBounds, mc.castsShadow };
            materialBatches[mc.material.get()].push_back(dc);
            allDraws.push_back(dc);
        }

        // Skinned meshes — each draw carries the animator's bone palette so both
        // the G-buffer and shadow passes select the skinning shader. Added to
        // allDraws too so the shadowDraws filter below picks up casters.
        for (auto [entity, smc] : scene.GetRegistry().view<SkinnedMeshComponent>().each()) {
            if (!smc.visible || !smc.material || smc.meshes.empty()) continue;

            const glm::mat4* palette   = nullptr;
            int              boneCount = 0;
            if (auto* anim = scene.GetRegistry().try_get<AnimatorComponent>(entity);
                anim && !anim->palette.empty()) {
                palette   = anim->palette.data();
                boneCount = std::min((int)anim->palette.size(), 100); // matches shader MAX_BONES
            }

            glm::mat4 world = scene.GetTransformSystem().GetWorldMatrix(entity);
            for (auto& mesh : smc.meshes) {
                DrawCall dc{ mesh.get(), world, smc.localBounds, smc.castsShadow };
                dc.bonePalette = palette;
                dc.boneCount   = boneCount;
                materialBatches[smc.material.get()].push_back(dc);
                allDraws.push_back(dc);
            }
        }

        // Shadow draws are a subset — only casters
        shadowDraws.clear();
        for (const auto& dc : allDraws)
            if (dc.castsShadow) shadowDraws.push_back(dc);

        // Gather lights from ECS
        pointPositions.clear();
        pointColors.clear();
        spotPositions.clear();
        spotDirs.clear();
        spotColors.clear();
        spotCosInner.clear();
        spotCosOuter.clear();
        sun = Diamond::SunLight{ glm::vec3(0,-1,0), glm::vec3(0) };
        bool foundSun = false;
        for (auto [entity, lc] : scene.GetRegistry().view<LightComponent>().each()) {
            const glm::mat4& worldMat = scene.GetTransformSystem().GetWorldMatrix(entity);
            glm::vec3 worldPos = glm::vec3(worldMat[3]);
            glm::vec3 worldDir = glm::normalize(glm::mat3(worldMat) * glm::vec3(0.0f, -1.0f, 0.0f));

            if (lc.type == LightType::Point && (int)pointPositions.size() < 4) {
                pointPositions.push_back(worldPos);
                pointColors.push_back(lc.color * lc.intensity);
            } else if (lc.type == LightType::Spot && (int)spotPositions.size() < 4) {
                spotPositions.push_back(worldPos);
                spotDirs.push_back(worldDir);
                spotColors.push_back(lc.color * lc.intensity);
                spotCosInner.push_back(std::cos(glm::radians(lc.innerConeAngle)));
                spotCosOuter.push_back(std::cos(glm::radians(lc.outerConeAngle)));
            } else if (lc.type == LightType::Sun && !foundSun) {
                sun.direction = worldDir;
                sun.color     = lc.color * lc.intensity;
                foundSun = true;
            }
        }

        // Cull and execute the render graph into the given FBO
        auto cullAndExecute = [&](uint32_t targetFBO) {
            Frustum frustum = Frustum::Extract(proj * view);
            culledBatches.clear();
            for (auto& [mat, batch] : materialBatches) {
                auto& out = culledBatches[mat];
                out.clear();
                for (const auto& dc : batch)
                    if (frustum.TestAABB(dc.localBounds.Transform(dc.modelMatrix)))
                        out.push_back(dc);
            }
            outputFBO = targetFBO;
            graph.Execute();
        };

        // --- Scene viewport: editor camera ---
        view = g_camera.GetViewMatrix();
        proj = glm::perspective(glm::radians(g_camera.Zoom), (float)fbW / (float)fbH, 0.1f, 100.0f);
        cullAndExecute(viewportFBO);

        // Collider debug wireframes — drawn into the editor viewport after the scene pass
        glBindFramebuffer(GL_FRAMEBUFFER, viewportFBO);
        //Physics::DrawColliders(scene);
        DebugDraw::Flush(proj * view);

        // --- Game viewport: primary camera entity (play mode only) ---
        entt::entity primaryCam = scene.GetPrimaryCamera();
        if (scene.IsPlaying() && primaryCam != entt::null)
        {
            auto& cc       = scene.Get<CameraComponent>(primaryCam);
            glm::mat4 camW = scene.GetTransformSystem().GetWorldMatrix(primaryCam);
            view = glm::inverse(camW);
            proj = glm::perspective(glm::radians(cc.fov), (float)fbW / (float)fbH, cc.nearClip, cc.farClip);
            cullAndExecute(gameViewportFBO);
            editorLayer.SetGameViewportTexture(gameViewportTex);
        }
        else
        {
            editorLayer.SetGameViewportTexture(0);
        }

        // Restore editor camera matrices for ImGuizmo / editor systems
        view = g_camera.GetViewMatrix();
        proj = glm::perspective(glm::radians(g_camera.Zoom), (float)fbW / (float)fbH, 0.1f, 100.0f);

        // Clear backbuffer for ImGui — scene now lives in viewportTex
        glBindFramebuffer(GL_FRAMEBUFFER, 0);
        glClearColor(0.1f, 0.1f, 0.1f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT);

        // ImGui frame
        ImGui_ImplOpenGL3_NewFrame();
        ImGui_ImplGlfw_NewFrame();
        ImGui::NewFrame();
        ImGuizmo::BeginFrame();

        editorLayer.SetViewportTexture(viewportTexture);
        editorLayer.UpdateCamera(view, proj, g_camera.Position);
        editorLayer.OnImGuiRender();

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

        ImGui::Render();
        ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());

        glfwSwapBuffers(window);
        Input::Update();
        glfwPollEvents();
    }

    if (viewportTexture) glDeleteTextures(1,     &viewportTexture);
    if (viewportFBO)     glDeleteFramebuffers(1, &viewportFBO);
    if (gameViewportTex) glDeleteTextures(1,     &gameViewportTex);
    if (gameViewportFBO) glDeleteFramebuffers(1, &gameViewportFBO);

    ImGui_ImplOpenGL3_Shutdown();
    ImGui_ImplGlfw_Shutdown();
    ImGui::DestroyContext();

    glfwTerminate();
    return 0;
}
