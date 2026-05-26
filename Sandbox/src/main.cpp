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

    // Bake IBL from environment map
    OpenGLPBRPass pbrPass;
    {
        OpenGLTexture envTex(ASSETS_DIR "/Textures/debris_basement_corridor_2k.hdr", true, true);
        pbrPass.BakeEnvironment(envTex, equirectShader, irradianceShader,
                                prefilterShader, brdfShader);
    }


    // Material (DiamondPlate PBR textures)
    static const std::string MAT =
        ASSETS_DIR "/Materials/DiamondPlate006C_2K-PNG/DiamondPlate006C_2K-PNG";
    PBRMaterial material;
    material.Albedo    = Texture::Create(MAT + "_Color.png",            true);
    material.Normal    = Texture::Create(MAT + "_NormalGL.png",         false);
    material.Metallic  = Texture::Create(MAT + "_Metalness.png",        false);
    material.Roughness = Texture::Create(MAT + "_Roughness.png",        false);
    material.AO        = Texture::Create(MAT + "_AmbientOcclusion.png", false);

    // Sphere mesh
    auto sphere = Mesh::Create(MeshData::UVSphere());
    std::vector<DrawCall> draws = { { sphere.get(), glm::mat4(1.0f) } };

    // Cerberus gun
    static const std::string CERB =
        ASSETS_DIR "/Models/Cerberus_by_Andrew_Maximov/Textures/Cerberus";
    PBRMaterial cerberusMaterial;
    cerberusMaterial.Albedo    = Texture::Create(CERB + "_A.tga", true);
    cerberusMaterial.Normal    = Texture::Create(CERB + "_N.tga", true);
    cerberusMaterial.Metallic  = Texture::Create(CERB + "_M.tga", true);
    cerberusMaterial.Roughness = Texture::Create(CERB + "_R.tga", true);

    auto cerberusMeshData = ModelImporter::Load(
        ASSETS_DIR "/Models/Cerberus_by_Andrew_Maximov/Cerberus_LP.FBX");
    std::vector<std::shared_ptr<Mesh>> cerberusGpuMeshes;
    std::vector<DrawCall> cerberusDraws;
    glm::mat4 cerberusModel = glm::mat4(1.0f);
    cerberusModel = glm::translate(cerberusModel, glm::vec3(3.0f, 0.0f, 0.0f));
    cerberusModel = glm::rotate(cerberusModel, glm::radians(-90.0f), glm::vec3(1.0f, 0.0f, 0.0f));
    cerberusModel = glm::scale(cerberusModel, glm::vec3(0.05f));
    for (auto& md : cerberusMeshData) {
        cerberusGpuMeshes.push_back(Mesh::Create(md));
        cerberusDraws.push_back({ cerberusGpuMeshes.back().get(), cerberusModel });
    }


    // Lights
    const glm::vec3 lightPositions[4] = {
        { -10.0f,  10.0f, 10.0f }, {  10.0f,  10.0f, 10.0f },
        { -10.0f, -10.0f, 10.0f }, {  10.0f, -10.0f, 10.0f },
    };
    const glm::vec3 lightColors[4] = {
        { 3000.0f, 3000.0f, 3000.0f }, { 3000.0f, 3000.0f, 3000.0f },
        { 3000.0f, 3000.0f, 3000.0f }, { 3000.0f, 3000.0f, 3000.0f },
    };

    while (!glfwWindowShouldClose(window))
    {
        float now   = static_cast<float>(glfwGetTime());
        g_deltaTime = now - g_lastFrame;
        g_lastFrame = now;

        processInput(window);

        int fbW, fbH;
        glfwGetFramebufferSize(window, &fbW, &fbH);

        glClearColor(0.08f, 0.09f, 0.11f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        glm::mat4 view = g_camera.GetViewMatrix();
        glm::mat4 proj = glm::perspective(
            glm::radians(g_camera.Zoom), (float)fbW / (float)fbH, 0.1f, 100.0f);

        // render sphere
        pbrPass.Render(pbrShader, draws, material,
                       view, proj, g_camera.Position,
                       lightPositions, lightColors,
                       fbW, fbH);

        // render gun
        pbrPass.Render(pbrShader, cerberusDraws, cerberusMaterial,
               view, proj, g_camera.Position,
               lightPositions, lightColors, fbW, fbH);

        pbrPass.DrawSkybox(backgroundShader, view, proj, fbW, fbH);

        glfwSwapBuffers(window);
        glfwPollEvents();
    }

    glfwTerminate();
    return 0;
}