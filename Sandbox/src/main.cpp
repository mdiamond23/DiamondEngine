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
#include "Platform/OpenGL/OpenGLPBR.h"
#include "Platform/OpenGL/OpenGLShader.h"
#include "Platform/OpenGL/OpenGLTexture.h"
#include "Assets/ModelImporter.h"

using namespace Diamond;

static Camera g_camera(glm::vec3(0.0f, 0.0f, 5.0f));
static float  g_lastX = 640.0f, g_lastY = 360.0f;
static bool   g_firstMouse = true;
static float  g_deltaTime = 0.0f, g_lastFrame = 0.0f;

static void framebufferSizeCallback(GLFWwindow*, int w, int h)
{
    glViewport(0, 0, w, h);
}

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

    // Shaders
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

    // Flat unlit shader for light markers
    OpenGLShader flatShader(
        ENGINE_SHADERS_DIR "/Unlit/flat.vert",
        ENGINE_SHADERS_DIR "/Unlit/flat.frag");

    // Post-processing: ACES tonemap + gamma correct
    OpenGLShader tonemapShader(
        ENGINE_SHADERS_DIR "/PostProcess/tonemap.vert",
        ENGINE_SHADERS_DIR "/PostProcess/tonemap.frag");

    // Directional shadow depth shader
    OpenGLShader depthShader(
        ENGINE_SHADERS_DIR "/Shadows/depth.vert",
        ENGINE_SHADERS_DIR "/Shadows/depth.frag");

    // Omnidirectional point shadow depth shader (geometry shader fans to 6 faces)
    OpenGLShader pointDepthShader(
        ENGINE_SHADERS_DIR "/Shadows/point_depth.vert",
        ENGINE_SHADERS_DIR "/Shadows/point_depth.geom",
        ENGINE_SHADERS_DIR "/Shadows/point_depth.frag");

    // Bake IBL from environment map
    OpenGLPBRPass pbrPass;
    {
        OpenGLTexture envTex(ASSETS_DIR "/Textures/debris_basement_corridor_2k.hdr", true, true);
        pbrPass.BakeEnvironment(envTex, equirectShader, irradianceShader,
                                prefilterShader, brdfShader);
    }
    pbrPass.SetupShadowMap(2048);
    pbrPass.SetupPointShadowMaps(4, 512, 25.0f);
    pbrPass.SetupHDRFramebuffer(1280, 720);


    // Material (DiamondPlate PBR textures)
    static const std::string MAT =
        ASSETS_DIR "/Materials/DiamondPlate006C_2K-PNG/DiamondPlate006C_2K-PNG";
    PBRMaterial material;
    material.Albedo    = Texture::Create(MAT + "_Color.png",            false);
    material.Normal    = Texture::Create(MAT + "_NormalGL.png",         false);
    material.Metallic  = Texture::Create(MAT + "_Metalness.png",        false);
    material.Roughness = Texture::Create(MAT + "_Roughness.png",        false);
    material.AO        = Texture::Create(MAT + "_AmbientOcclusion.png", false);

    // Sphere: left of centre, sitting on the ground (y = -0.3 puts bottom at ground level)
    auto sphere = Mesh::Create(MeshData::UVSphere());
    glm::mat4 sphereModel = glm::translate(glm::mat4(1.0f), glm::vec3(-3.0f, -0.3f, 1.0f));
    std::vector<DrawCall> draws = { { sphere.get(), sphereModel } };

    // Ground plane + cubes
    auto cube = Mesh::Create(MeshData::UnitCube());
    auto makeMat = [](glm::vec3 t, glm::vec3 s) {
        return glm::scale(glm::translate(glm::mat4(1.0f), t), s);
    };
    std::vector<DrawCall> cubeDraws = {
        { cube.get(), makeMat({  0.0f, -1.7f,  0.0f}, {20.0f, 0.4f, 20.0f}) }, // ground
        { cube.get(), makeMat({  0.5f, -0.3f, -3.5f}, { 1.0f, 1.0f,  1.0f}) }, // centre-back, short
        { cube.get(), makeMat({  3.5f,  0.7f,  1.0f}, { 1.0f, 2.0f,  1.0f}) }, // right-front, medium pillar
        { cube.get(), makeMat({ -4.5f,  1.7f, -2.0f}, { 1.0f, 3.0f,  1.0f}) }, // left-back, tall pillar
        { cube.get(), makeMat({ -1.5f, -0.3f,  3.5f}, { 1.2f, 1.2f,  1.2f}) }, // front-left, short
        { cube.get(), makeMat({  5.5f, -0.3f, -3.0f}, { 1.0f, 1.0f,  1.0f}) }, // far right, short
    };

    // Cerberus gun: right-back, clear of all cubes
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

    // Small sphere used to mark each light position visually
    auto lightMarker = Mesh::Create(MeshData::UVSphere());

    // Point lights: two close (cast visible shadows), two far fill
    const glm::vec3 lightPositions[4] = {
        {  1.0f, 3.0f,  2.0f },  // overhead-ish, main caster
        { -5.0f, 2.0f,  4.0f },  // upper left-front
        {  8.0f, 4.0f, -6.0f },  // far right fill
        { -8.0f, 4.0f, -6.0f },  // far left fill
    };
    const glm::vec3 lightColors[4] = {
        { 500.0f, 480.0f, 440.0f },  // bright warm white
        { 300.0f, 260.0f, 220.0f },  // warm side fill
        {  40.0f,  40.0f,  40.0f },  // dim far fill
        {  40.0f,  40.0f,  40.0f },  // dim far fill
    };

    // Directional sun light
    Diamond::SunLight sun;
    sun.direction = glm::normalize(glm::vec3(-0.5f, -1.0f, -0.3f));
    // sun.color     = glm::vec3(8.0f, 7.6f, 7.0f); // warm white, dominant
    sun.color     = glm::vec3(0.0f); // off — isolating point light shadows

    // Orthographic frustum fitted around the scene (-15..15 world units)
    glm::mat4 lightProj = glm::ortho(-15.0f, 15.0f, -15.0f, 15.0f, 1.0f, 50.0f);
    glm::vec3 lightPos  = -sun.direction * 20.0f; // pull back along sun dir
    glm::mat4 lightView = glm::lookAt(lightPos, glm::vec3(0.0f), glm::vec3(0.0f, 1.0f, 0.0f));
    sun.lightSpaceMatrix = lightProj * lightView;

    while (!glfwWindowShouldClose(window))
    {
        float now   = static_cast<float>(glfwGetTime());
        g_deltaTime = now - g_lastFrame;
        g_lastFrame = now;

        processInput(window);

        int fbW, fbH;
        glfwGetFramebufferSize(window, &fbW, &fbH);

        // --- build combined draw list once per frame ---
        std::vector<Diamond::DrawCall> allDraws;
        allDraws.insert(allDraws.end(), draws.begin(), draws.end());
        allDraws.insert(allDraws.end(), cerberusDraws.begin(), cerberusDraws.end());
        allDraws.insert(allDraws.end(), cubeDraws.begin(), cubeDraws.end());

        // --- directional shadow pass ---
        pbrPass.RenderShadowPass(depthShader, allDraws, sun);

        // --- point shadow pass ---
        pbrPass.RenderPointShadowPass(pointDepthShader, allDraws, lightPositions, 4);

        // --- lighting pass (into HDR FBO) ---
        pbrPass.BeginHDR(fbW, fbH);

        glm::mat4 view = g_camera.GetViewMatrix();
        glm::mat4 proj = glm::perspective(
            glm::radians(g_camera.Zoom), (float)fbW / (float)fbH, 0.1f, 100.0f);

        pbrPass.Render(pbrShader, draws, material,
                       view, proj, g_camera.Position,
                       lightPositions, lightColors, sun,
                       fbW, fbH);
        
        pbrPass.Render(pbrShader, cubeDraws, material,
                view, proj, g_camera.Position,
                lightPositions, lightColors, sun, fbW, fbH);              

        pbrPass.Render(pbrShader, cerberusDraws, cerberusMaterial,
                       view, proj, g_camera.Position,
                       lightPositions, lightColors, sun, fbW, fbH);

        // Draw a small bright sphere at each point light position
        flatShader.Bind();
        flatShader.SetMat4("projection", proj);
        flatShader.SetMat4("view", view);
        for (int i = 0; i < 4; ++i)
        {
            glm::mat4 m = glm::scale(
                glm::translate(glm::mat4(1.0f), lightPositions[i]),
                glm::vec3(0.15f));
            flatShader.SetMat4("model", m);
            // clamp HDR light colour to visible [0,1] range for the marker
            glm::vec3 c = glm::min(lightColors[i] / 500.0f + glm::vec3(0.3f), glm::vec3(1.0f));
            flatShader.SetVec3("color", c);
            lightMarker->Draw(flatShader);
        }

        pbrPass.DrawSkybox(backgroundShader, view, proj, fbW, fbH);

        // --- tonemap pass (HDR FBO → default framebuffer) ---
        pbrPass.RenderTonemapPass(tonemapShader);

        glfwSwapBuffers(window);
        glfwPollEvents();
    }

    glfwTerminate();
    return 0;
}