#include <glad/gl.h>
#define GLFW_INCLUDE_NONE
#include <GLFW/glfw3.h>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <vector>

#include "Core/Camera.h"
#include "Renderer/MeshData.h"
#include "Renderer/Material.h"
#include "Renderer/TextureData.h"
#include "Renderer/RenderGraph.h"
#include "Platform/OpenGL/Resources/OpenGLRenderTypes.h"
#include "Platform/OpenGL/Resources/OpenGLShader.h"
#include "Platform/OpenGL/Resources/OpenGLTexture.h"
#include "Platform/OpenGL/Passes/IBL/OpenGLIBLPass.h"
#include "Platform/OpenGL/Passes/Shadows/OpenGLShadowPass.h"
#include "Platform/OpenGL/Passes/Deferred/OpenGLGBufferPass.h"
#include "Platform/OpenGL/Passes/Deferred/OpenGLSSAOPass.h"
#include "Platform/OpenGL/Passes/Deferred/OpenGLDeferredLightingPass.h"
#include "Platform/OpenGL/Passes/PostProcess/OpenGLTonemapPass.h"
#include "Platform/OpenGL/Passes/PostProcess/OpenGLBloomPass.h"
#include "Platform/OpenGL/Passes/Forward/OpenGLTransparencyPass.h"
#include "Assets/ModelImporter.h"

using namespace Diamond;

static Camera g_camera(glm::vec3(0.0f, 0.0f, 5.0f));
static float  g_lastX = 640.0f, g_lastY = 360.0f;
static bool   g_firstMouse = true;
static float  g_deltaTime = 0.0f, g_lastFrame = 0.0f;

// timing
float deltaTime = 0.0f;
float lastFrame = 0.0f;
float fpsTimer = 0.0f;
int frameCount = 0;

static void framebufferSizeCallback(GLFWwindow*, int w, int h) { glViewport(0, 0, w, h); }

static void mouseCallback(GLFWwindow*, double x, double y)
{
    float fx = static_cast<float>(x), fy = static_cast<float>(y);
    if (g_firstMouse) { g_lastX = fx; g_lastY = fy; g_firstMouse = false; }
    g_camera.ProcessMouseMovement(fx - g_lastX, g_lastY - fy);
    g_lastX = fx; g_lastY = fy;
}

static void scrollCallback(GLFWwindow*, double, double y)
{
    g_camera.ProcessMouseScroll(static_cast<float>(y));
}

static void processInput(GLFWwindow* window)
{
    if (glfwGetKey(window, GLFW_KEY_ESCAPE) == GLFW_PRESS)
        glfwSetWindowShouldClose(window, true);
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
    glfwSetInputMode(window, GLFW_CURSOR, GLFW_CURSOR_DISABLED);
    glfwSetFramebufferSizeCallback(window, framebufferSizeCallback);
    glfwSetCursorPosCallback(window, mouseCallback);
    glfwSetScrollCallback(window, scrollCallback);

    if (!gladLoadGL(glfwGetProcAddress)) { glfwTerminate(); return -1; }

    glEnable(GL_DEPTH_TEST);
    glEnable(GL_TEXTURE_CUBE_MAP_SEAMLESS);

    // --- Shaders ---
    OpenGLShader gbufferShader(
        ENGINE_SHADERS_DIR "/Deferred/gbuffer.vert",
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
    OpenGLShader depthShader(
        ENGINE_SHADERS_DIR "/Shadows/depth.vert",
        ENGINE_SHADERS_DIR "/Shadows/depth.frag");
    OpenGLShader pointDepthShader(
        ENGINE_SHADERS_DIR "/Shadows/point_depth.vert",
        ENGINE_SHADERS_DIR "/Shadows/point_depth.geom",
        ENGINE_SHADERS_DIR "/Shadows/point_depth.frag");
    OpenGLShader transparencyShader(
        ENGINE_SHADERS_DIR "/Forward/transparent.vert",
        ENGINE_SHADERS_DIR "/Forward/transparent.frag");

    // --- Pass objects ---
    OpenGLIBLPass              iblPass;
    OpenGLShadowPass           shadowPass;
    OpenGLGBufferPass          gbufferPass;
    OpenGLSSAOPass             ssaoPass;
    OpenGLDeferredLightingPass lightingPass;
    OpenGLBloomPass            bloomPass;
    OpenGLTransparencyPass     transparencyPass;

    // IBL bake — one-time
    {
        OpenGLTexture envTex(ASSETS_DIR "/Textures/debris_basement_corridor_2k.hdr", true, true);
        iblPass.BakeEnvironment(envTex, equirectShader, irradianceShader,
                                prefilterShader, brdfShader);
    }

    // Shadow maps — persistent, not managed by the graph
    shadowPass.SetupShadowMap(2048);
    shadowPass.SetupPointShadowMaps(4, 512, 25.0f);

    // --- Scene geometry ---
    static const std::string MAT =
        ASSETS_DIR "/Materials/DiamondPlate006C_2K-PNG/DiamondPlate006C_2K-PNG";
    PBRMaterial material;
    material.Albedo    = Texture::Create(MAT + "_Color.png",            false);
    material.Normal    = Texture::Create(MAT + "_NormalGL.png",         false);
    material.Metallic  = Texture::Create(MAT + "_Metalness.png",        false);
    material.Roughness = Texture::Create(MAT + "_Roughness.png",        false);
    material.AO        = Texture::Create(MAT + "_AmbientOcclusion.png", false);

    auto sphere = Mesh::Create(MeshData::UVSphere());
    glm::mat4 sphereModel = glm::translate(glm::mat4(1.0f), glm::vec3(-3.0f, -0.3f, 1.0f));
    std::vector<DrawCall> draws = { { sphere.get(), sphereModel } };

    auto cube = Mesh::Create(MeshData::UnitCube());
    auto makeMat = [](glm::vec3 t, glm::vec3 s) {
        return glm::scale(glm::translate(glm::mat4(1.0f), t), s);
    };
    std::vector<DrawCall> cubeDraws = {
        { cube.get(), makeMat({  0.0f, -1.7f,  0.0f}, {20.0f, 0.4f, 20.0f}) },
        { cube.get(), makeMat({  0.5f, -0.3f, -3.5f}, { 1.0f, 1.0f,  1.0f}) },
        { cube.get(), makeMat({  3.5f,  0.7f,  1.0f}, { 1.0f, 2.0f,  1.0f}) },
        { cube.get(), makeMat({ -4.5f,  1.7f, -2.0f}, { 1.0f, 3.0f,  1.0f}) },
        { cube.get(), makeMat({ -1.5f, -0.3f,  3.5f}, { 1.2f, 1.2f,  1.2f}) },
        { cube.get(), makeMat({  5.5f, -0.3f, -3.0f}, { 1.0f, 1.0f,  1.0f}) },
    };

    static const std::string CERB =
        ASSETS_DIR "/Models/Cerberus_by_Andrew_Maximov/Textures/Cerberus";
    PBRMaterial cerberusMaterial;
    cerberusMaterial.Albedo    = Texture::Create(CERB + "_A.tga", false);
    cerberusMaterial.Normal    = Texture::Create(CERB + "_N.tga", false);
    cerberusMaterial.Metallic  = Texture::Create(CERB + "_M.tga", false);
    cerberusMaterial.Roughness = Texture::Create(CERB + "_R.tga", false);

    auto cerberusMeshData = ModelImporter::Load(
        ASSETS_DIR "/Models/Cerberus_by_Andrew_Maximov/Cerberus_LP.FBX");
    std::vector<std::shared_ptr<Mesh>> cerberusGpuMeshes;
    std::vector<DrawCall> cerberusDraws;
    glm::mat4 cerberusModel = glm::translate(glm::mat4(1.0f), glm::vec3(1.5f, 1.5f, 1.0f));
    cerberusModel = glm::rotate(cerberusModel, glm::radians(-90.0f), glm::vec3(1.0f, 0.0f, 0.0f));
    cerberusModel = glm::scale(cerberusModel, glm::vec3(0.05f));
    for (auto& md : cerberusMeshData) {
        cerberusGpuMeshes.push_back(Mesh::Create(md));
        cerberusDraws.push_back({ cerberusGpuMeshes.back().get(), cerberusModel });
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

    const glm::vec3 lightPositions[4] = {
        {  1.0f, 3.0f,  2.0f },
        { -5.0f, 2.0f,  4.0f },
        {  8.0f, 4.0f, -6.0f },
        { -8.0f, 4.0f, -6.0f },
    };
    const glm::vec3 lightColors[4] = {
        { 600.0f, 200.0f, 100.0f },
        {  80.0f, 200.0f, 600.0f },
        { 300.0f, 600.0f, 200.0f },
        { 500.0f, 400.0f,  50.0f },
    };

    Diamond::SunLight sun;
    sun.direction = glm::normalize(glm::vec3(-0.5f, -1.0f, -0.3f));
    sun.color     = glm::vec3(0.0f);
    glm::mat4 lightProj = glm::ortho(-15.0f, 15.0f, -15.0f, 15.0f, 1.0f, 50.0f);
    glm::vec3 lightPos  = -sun.direction * 20.0f;
    glm::mat4 lightView = glm::lookAt(lightPos, glm::vec3(0.0f), glm::vec3(0.0f, 1.0f, 0.0f));
    sun.lightSpaceMatrix = lightProj * lightView;

    // --- Per-frame state captured by render graph lambdas ---
    std::vector<DrawCall> allDraws;
    glm::mat4 view, proj;
    int fbW = 0, fbH = 0;

    // --- Render graph ---
    RenderGraph     graph;
    RGTextureHandle gViewPos, gViewNormal, gAlbedo, gMaterial;
    RGTextureHandle hSsaoRaw, hSsaoBlur;
    RGTextureHandle hdrBuffer, brightBuffer;
    RGTextureHandle pingPong[2];

    auto buildGraph = [&](int w, int h)
    {
        fbW = w; fbH = h;
        graph.Clear();

        // G-buffer — 4 attachments sharing one FBO (gViewPos owns it)
        gViewPos    = graph.DeclareTexture("gViewPos",    { w, h, GL_RGBA16F, true  });
        gViewNormal = graph.DeclareTexture("gViewNormal", { w, h, GL_RGBA16F, false, gViewPos, 1 });
        gAlbedo     = graph.DeclareTexture("gAlbedo",     { w, h, GL_RGBA8,   false, gViewPos, 2 });
        gMaterial   = graph.DeclareTexture("gMaterial",   { w, h, GL_RGBA8,   false, gViewPos, 3 });

        // SSAO intermediates
        hSsaoRaw  = graph.DeclareTexture("ssaoRaw",  { w, h, GL_R16F, false });
        hSsaoBlur = graph.DeclareTexture("ssaoBlur", { w, h, GL_R16F, false });

        // HDR + bloom extract (MRT)
        hdrBuffer    = graph.DeclareTexture("hdrBuffer",    { w, h, GL_RGBA16F, true  });
        brightBuffer = graph.DeclareTexture("brightBuffer", { w, h, GL_RGBA16F, false, hdrBuffer, 1 });

        // Bloom ping-pong
        pingPong[0] = graph.DeclareTexture("bloomPing", { w, h, GL_RGBA16F, false });
        pingPong[1] = graph.DeclareTexture("bloomPong", { w, h, GL_RGBA16F, false });

        // Shadow passes — sinks (no graph writes), always alive
        graph.AddPass("DirectionalShadow")
            .SetExecute([&]{
                shadowPass.RenderShadowPass(depthShader, allDraws, sun);
            });

        graph.AddPass("PointShadow")
            .SetExecute([&]{
                shadowPass.RenderPointShadowPass(pointDepthShader, allDraws, lightPositions, 4);
            });

        // Geometry pass — fills all 4 G-buffer attachments
        graph.AddPass("GBuffer")
            .Write(gViewPos).Write(gViewNormal).Write(gAlbedo).Write(gMaterial)
            .SetExecute([&]{
                // Clear once, then render all material batches into the same FBO
                glBindFramebuffer(GL_FRAMEBUFFER, graph.GetFBO(gViewPos));
                glClearColor(0.0f, 0.0f, 0.0f, 1.0f);
                glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

                gbufferPass.Render(gbufferShader, draws,         material,         view, proj, graph.GetFBO(gViewPos), fbW, fbH);
                gbufferPass.Render(gbufferShader, cubeDraws,     material,         view, proj, graph.GetFBO(gViewPos), fbW, fbH);
                gbufferPass.Render(gbufferShader, cerberusDraws, cerberusMaterial, view, proj, graph.GetFBO(gViewPos), fbW, fbH);
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
            .Read(hSsaoBlur)
            .Write(hdrBuffer).Write(brightBuffer)
            .SetExecute([&]{
                lightingPass.Render(lightingShader,
                                    graph.GetTexture(gViewPos),
                                    graph.GetTexture(gViewNormal),
                                    graph.GetTexture(gAlbedo),
                                    graph.GetTexture(gMaterial),
                                    graph.GetTexture(hSsaoBlur),
                                    view, g_camera.Position,
                                    lightPositions, lightColors, sun,
                                    shadowPass, iblPass,
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
                for (int i = 0; i < 10; ++i) {
                    int dst = i % 2;
                    bloomPass.RenderBlurPass(blurShader, src, graph.GetFBO(pingPong[dst]), horizontal);
                    src        = graph.GetTexture(pingPong[dst]);
                    horizontal = !horizontal;
                }
            });

        // Composite + tonemap → backbuffer
        graph.AddPass("BloomComposite")
            .Read(hdrBuffer).Read(pingPong[1])
            .SetExecute([&]{
                bloomPass.RenderCompositePass(bloomFinalShader,
                    graph.GetTexture(hdrBuffer),
                    graph.GetTexture(pingPong[1]),
                    true, 0.1f);
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

        allDraws.clear();
        allDraws.insert(allDraws.end(), draws.begin(),        draws.end());
        allDraws.insert(allDraws.end(), cerberusDraws.begin(), cerberusDraws.end());
        allDraws.insert(allDraws.end(), cubeDraws.begin(),    cubeDraws.end());

        view = g_camera.GetViewMatrix();
        proj = glm::perspective(
            glm::radians(g_camera.Zoom), (float)fbW / (float)fbH, 0.1f, 100.0f);

        graph.Execute();

        glfwSwapBuffers(window);
        glfwPollEvents();
    }

    glfwTerminate();
    return 0;
}
