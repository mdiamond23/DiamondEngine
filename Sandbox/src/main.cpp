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
#include "Platform/OpenGL/OpenGLRenderTypes.h"
#include "Platform/OpenGL/OpenGLIBLPass.h"
#include "Platform/OpenGL/OpenGLShadowPass.h"
#include "Platform/OpenGL/OpenGLPBRSurfacePass.h"
#include "Platform/OpenGL/OpenGLTonemapPass.h"
#include "Platform/OpenGL/OpenGLBloomPass.h"
#include "Platform/OpenGL/OpenGLShader.h"
#include "Platform/OpenGL/OpenGLTexture.h"
#include "Assets/ModelImporter.h"

using namespace Diamond;

static Camera g_camera(glm::vec3(0.0f, 0.0f, 5.0f));
static float  g_lastX = 640.0f, g_lastY = 360.0f;
static bool   g_firstMouse = true;
static float  g_deltaTime = 0.0f, g_lastFrame = 0.0f;

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
    OpenGLShader pbrShader(
        ENGINE_SHADERS_DIR "/PBR/pbr_textured.vert",
        ENGINE_SHADERS_DIR "/PBR/pbr_textured.frag");
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
    OpenGLShader flatShader(
        ENGINE_SHADERS_DIR "/Unlit/flat.vert",
        ENGINE_SHADERS_DIR "/Unlit/flat.frag");
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

    // --- Pass objects ---
    OpenGLIBLPass        iblPass;
    OpenGLShadowPass     shadowPass;
    OpenGLPBRSurfacePass surfacePass;
    OpenGLBloomPass      bloomPass;

    // IBL bake — one-time, not part of the per-frame graph
    {
        OpenGLTexture envTex(ASSETS_DIR "/Textures/debris_basement_corridor_2k.hdr", true, true);
        iblPass.BakeEnvironment(envTex, equirectShader, irradianceShader,
                                prefilterShader, brdfShader);
    }

    // Shadow maps — pass manages its own FBOs (persistent, not transient)
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
    glm::mat4 cerberusModel = glm::translate(glm::mat4(1.0f), glm::vec3(4.0f, 0.5f, -3.5f));
    cerberusModel = glm::rotate(cerberusModel, glm::radians(-90.0f), glm::vec3(1.0f, 0.0f, 0.0f));
    cerberusModel = glm::scale(cerberusModel, glm::vec3(0.05f));
    for (auto& md : cerberusMeshData) {
        cerberusGpuMeshes.push_back(Mesh::Create(md));
        cerberusDraws.push_back({ cerberusGpuMeshes.back().get(), cerberusModel });
    }

    auto lightMarker = Mesh::Create(MeshData::UVSphere());

    const glm::vec3 lightPositions[4] = {
        {  1.0f, 3.0f,  2.0f },
        { -5.0f, 2.0f,  4.0f },
        {  8.0f, 4.0f, -6.0f },
        { -8.0f, 4.0f, -6.0f },
    };
    const glm::vec3 lightColors[4] = {
        { 600.0f, 200.0f, 100.0f },  // warm orange-red
        {  80.0f, 200.0f, 600.0f },  // cool blue
        { 300.0f, 600.0f, 200.0f },  // bright green
        { 500.0f, 400.0f,  50.0f },  // golden yellow
    };

    Diamond::SunLight sun;
    sun.direction = glm::normalize(glm::vec3(-0.5f, -1.0f, -0.3f));
    sun.color     = glm::vec3(0.0f);
    glm::mat4 lightProj = glm::ortho(-15.0f, 15.0f, -15.0f, 15.0f, 1.0f, 50.0f);
    glm::vec3 lightPos  = -sun.direction * 20.0f;
    glm::mat4 lightView = glm::lookAt(lightPos, glm::vec3(0.0f), glm::vec3(0.0f, 1.0f, 0.0f));
    sun.lightSpaceMatrix = lightProj * lightView;

    // --- Per-frame state captured by the render graph lambdas ---
    std::vector<DrawCall> allDraws;
    glm::mat4 view, proj;
    int fbW = 0, fbH = 0;

    // --- Render graph ---
    RenderGraph     graph;
    RGTextureHandle hdrBuffer;
    RGTextureHandle brightBuffer;
    RGTextureHandle pingPong[2];

    // Rebuilds the graph whenever the framebuffer resolution changes
    auto buildGraph = [&](int w, int h)
    {
        fbW = w;
        fbH = h;
        graph.Clear();

        hdrBuffer    = graph.DeclareTexture("hdrBuffer",    { w, h, GL_RGBA16F, true });
        brightBuffer = graph.DeclareTexture("brightBuffer", { w, h, GL_RGBA16F, false, hdrBuffer });
        pingPong[0]  = graph.DeclareTexture("bloomPing",    { w, h, GL_RGBA16F, false });
        pingPong[1]  = graph.DeclareTexture("bloomPong",    { w, h, GL_RGBA16F, false });

        // Directional shadow pass — sink (no writes to graph), always alive
        graph.AddPass("DirectionalShadow")
            .SetExecute([&]{
                shadowPass.RenderShadowPass(depthShader, allDraws, sun);
            });

        // Point shadow pass — sink, always alive
        graph.AddPass("PointShadow")
            .SetExecute([&]{
                shadowPass.RenderPointShadowPass(pointDepthShader, allDraws, lightPositions, 4);
            });

        // PBR lighting + skybox into the graph-managed HDR buffer
        graph.AddPass("PBR")
            .Write(hdrBuffer)
            .Write(brightBuffer)
            .SetExecute([&]{
                glBindFramebuffer(GL_FRAMEBUFFER, graph.GetFBO(hdrBuffer));
                glViewport(0, 0, fbW, fbH);
                glClearColor(0.08f, 0.09f, 0.11f, 1.0f);
                glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

                surfacePass.Render(pbrShader, draws, material,
                                   view, proj, g_camera.Position,
                                   lightPositions, lightColors, sun,
                                   shadowPass, iblPass, fbW, fbH);
                surfacePass.Render(pbrShader, cubeDraws, material,
                                   view, proj, g_camera.Position,
                                   lightPositions, lightColors, sun,
                                   shadowPass, iblPass, fbW, fbH);
                surfacePass.Render(pbrShader, cerberusDraws, cerberusMaterial,
                                   view, proj, g_camera.Position,
                                   lightPositions, lightColors, sun,
                                   shadowPass, iblPass, fbW, fbH);

                flatShader.Bind();
                flatShader.SetMat4("projection", proj);
                flatShader.SetMat4("view", view);
                for (int i = 0; i < 4; ++i) {
                    glm::mat4 m = glm::scale(
                        glm::translate(glm::mat4(1.0f), lightPositions[i]),
                        glm::vec3(0.15f));
                    flatShader.SetMat4("model", m);
                    glm::vec3 c = glm::min(lightColors[i] / 500.0f + glm::vec3(0.3f), glm::vec3(1.0f));
                    flatShader.SetVec3("color", c);
                    lightMarker->Draw(flatShader);
                }

                surfacePass.DrawSkybox(backgroundShader, iblPass, view, proj, fbW, fbH);
            });

        // Blur ping-pong — 10 iterations, final result in pingPong[1]
        graph.AddPass("BloomBlur")
            .Read(brightBuffer)
            .Write(pingPong[0])
            .Write(pingPong[1])
            .SetExecute([&]{
                bool horizontal = true;
                uint32_t src = graph.GetTexture(brightBuffer);
                for (int i = 0; i < 10; ++i)
                {
                    int dst = i % 2;
                    bloomPass.RenderBlurPass(blurShader, src, graph.GetFBO(pingPong[dst]), horizontal);
                    src        = graph.GetTexture(pingPong[dst]);
                    horizontal = !horizontal;
                }
            });

        // Composite — blends HDR scene + blurred bloom, tonemaps to backbuffer
        graph.AddPass("BloomComposite")
            .Read(hdrBuffer)
            .Read(pingPong[1])
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
        float now   = static_cast<float>(glfwGetTime());
        g_deltaTime = now - g_lastFrame;
        g_lastFrame = now;

        processInput(window);

        glfwGetFramebufferSize(window, &fbW, &fbH);

        // Rebuild graph on first frame or window resize
        if (fbW != lastFbW || fbH != lastFbH)
        {
            buildGraph(fbW, fbH);
            lastFbW = fbW;
            lastFbH = fbH;
        }

        // Update per-frame state the lambdas read
        allDraws.clear();
        allDraws.insert(allDraws.end(), draws.begin(), draws.end());
        allDraws.insert(allDraws.end(), cerberusDraws.begin(), cerberusDraws.end());
        allDraws.insert(allDraws.end(), cubeDraws.begin(), cubeDraws.end());

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
