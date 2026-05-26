#include "OpenGLPBR.h"
#include <glad/gl.h>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <string>
#include <spdlog/spdlog.h>
#include "OpenGLShader.h"
#include "Renderer/MeshData.h"

namespace {
static void CheckGL(const char* label)
{
    GLenum err = glGetError();
    if (err != GL_NO_ERROR)
        spdlog::error("OpenGL error at [{}]: 0x{:04X}", label, err);
}
static void CheckFBO(const char* label)
{
    GLenum status = glCheckFramebufferStatus(GL_FRAMEBUFFER);
    if (status != GL_FRAMEBUFFER_COMPLETE)
        spdlog::error("FBO incomplete at [{}]: 0x{:04X}", label, status);
    else
        spdlog::info("FBO complete at [{}]", label);
}
}

namespace Diamond {

OpenGLPBRPass::OpenGLPBRPass()
    : m_Cube(MeshData::UnitCube())
    , m_Quad(MeshData::FullscreenQuad())
{
}

OpenGLPBRPass::~OpenGLPBRPass() 
{
    // delete the generated textures to free GPU memory
    glDeleteTextures(1, &envCubemap);
    glDeleteTextures(1, &irradianceMap);
    glDeleteTextures(1, &prefilterMap);
    glDeleteTextures(1, &brdfLUTTexture);
}

// Requires: all the necessary resources (environment map, shaders) to be provided by the caller. This is because the environment baking process is expensive and should only be done once per environment, so we want to give the caller control over when it happens and what resources are used.
// Modifies: the internal state of the OpenGLPBRPass object by generating and storing the baked IBL textures (envCubemap, irradianceMap, prefilterMap, brdfLUTTexture) that will be used for rendering in the BindIBL function.
// Effects: generates the environment cubemap, irradiance map, prefilter map, and BRDF LUT textures by rendering to a framebuffer using the provided shaders and the input environment map. These textures are then stored in the OpenGLPBRPass object for later use in rendering.
void OpenGLPBRPass::BakeEnvironment(
    const OpenGLTexture& environmentMap, 
    const OpenGLShader& equirectangularToCubemapShader, 
    const OpenGLShader& irradianceShader, 
    const OpenGLShader& prefilterShader, 
    const OpenGLShader& brdfShader) 
{

    //  configure and bind capture framebuffer
    unsigned int captureFBO, captureRBO;
    glGenFramebuffers(1, &captureFBO);
    glGenRenderbuffers(1, &captureRBO);
    glBindFramebuffer(GL_FRAMEBUFFER, captureFBO);
    glBindRenderbuffer(GL_RENDERBUFFER, captureRBO);
    
    // pbr: we need a framebuffer to render the cubemap faces to and a renderbuffer for depth testing.
    glRenderbufferStorage(GL_RENDERBUFFER, GL_DEPTH_COMPONENT24, 512, 512);
    glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_RENDERBUFFER, captureRBO);

    // Environment Cubemaps
    glGenTextures(1, &envCubemap);
    glBindTexture(GL_TEXTURE_CUBE_MAP, envCubemap);
        for (unsigned int i = 0; i < 6; ++i)
    {
        glTexImage2D(GL_TEXTURE_CUBE_MAP_POSITIVE_X + i, 0, GL_RGB16F, 512, 512, 0, GL_RGB, GL_FLOAT, nullptr);
    }
    glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_WRAP_R, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_LINEAR); // enable pre-filter mipmap sampling (combatting visible dots artifact)
    glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_MAG_FILTER, GL_LINEAR);

    // 6 cubemap face directions for capturing data onto the cubemap
    glm::mat4 captureProjection = glm::perspective(glm::radians(90.0f), 1.0f, 0.1f, 10.0f);
    glm::mat4 captureViews[] =
    {
        glm::lookAt(glm::vec3(0.0f, 0.0f, 0.0f), glm::vec3(1.0f,  0.0f,  0.0f), glm::vec3(0.0f, -1.0f,  0.0f)),
        glm::lookAt(glm::vec3(0.0f, 0.0f, 0.0f), glm::vec3(-1.0f,  0.0f,  0.0f), glm::vec3(0.0f, -1.0f,  0.0f)),
        glm::lookAt(glm::vec3(0.0f, 0.0f, 0.0f), glm::vec3(0.0f,  1.0f,  0.0f), glm::vec3(0.0f,  0.0f,  1.0f)),
        glm::lookAt(glm::vec3(0.0f, 0.0f, 0.0f), glm::vec3(0.0f, -1.0f,  0.0f), glm::vec3(0.0f,  0.0f, -1.0f)),
        glm::lookAt(glm::vec3(0.0f, 0.0f, 0.0f), glm::vec3(0.0f,  0.0f,  1.0f), glm::vec3(0.0f, -1.0f,  0.0f)),
        glm::lookAt(glm::vec3(0.0f, 0.0f, 0.0f), glm::vec3(0.0f,  0.0f, -1.0f), glm::vec3(0.0f, -1.0f,  0.0f))
    };

    equirectangularToCubemapShader.Bind();
    equirectangularToCubemapShader.SetInt("equirectangularMap", 0);
    equirectangularToCubemapShader.SetMat4("projection", captureProjection);
    glActiveTexture(GL_TEXTURE0);
    environmentMap.Bind(0);

    glViewport(0, 0, 512, 512); // don't forget to configure the viewport to the capture dimensions.
    glBindFramebuffer(GL_FRAMEBUFFER, captureFBO);
    CheckGL("pre-equirect bake");
    for (unsigned int i = 0; i < 6; ++i)
    {
        equirectangularToCubemapShader.SetMat4("view", captureViews[i]);
        glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_CUBE_MAP_POSITIVE_X + i, envCubemap, 0);
        if (i == 0) CheckFBO("equirect->cubemap face 0");
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        m_Cube.Draw(equirectangularToCubemapShader);
    }

    // then let OpenGL generate mipmaps from first mip face (combatting visible dots artifact)
    glBindTexture(GL_TEXTURE_CUBE_MAP, envCubemap);
    glGenerateMipmap(GL_TEXTURE_CUBE_MAP);

    glGenTextures(1, &irradianceMap);
    glBindTexture(GL_TEXTURE_CUBE_MAP, irradianceMap);
    for (unsigned int i = 0; i < 6; ++i)
    {
        glTexImage2D(GL_TEXTURE_CUBE_MAP_POSITIVE_X + i, 0, GL_RGB16F, 32, 32, 0, GL_RGB, GL_FLOAT, nullptr);
    }
    glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_WRAP_R, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_MAG_FILTER, GL_LINEAR);

    glBindFramebuffer(GL_FRAMEBUFFER, captureFBO);
    glBindRenderbuffer(GL_RENDERBUFFER, captureRBO);
    glRenderbufferStorage(GL_RENDERBUFFER, GL_DEPTH_COMPONENT24, 32, 32);

    // Solve diffuse integral by convolution to create an irradiance (cube)map.
    irradianceShader.Bind();
    irradianceShader.SetInt("environmentMap", 0);
    irradianceShader.SetMat4("projection", captureProjection);
    glActiveTexture(GL_TEXTURE0);
    glBindTexture(GL_TEXTURE_CUBE_MAP, envCubemap);

    glViewport(0, 0, 32, 32); // don't forget to configure the viewport to the capture dimensions.
    glBindFramebuffer(GL_FRAMEBUFFER, captureFBO);
    for (unsigned int i = 0; i < 6; ++i)
    {
        irradianceShader.SetMat4("view", captureViews[i]);
        glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_CUBE_MAP_POSITIVE_X + i, irradianceMap, 0);
        if (i == 0) CheckFBO("irradiance face 0");
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        m_Cube.Draw(irradianceShader);
    }

    glBindFramebuffer(GL_FRAMEBUFFER, 0);

    // create prefilter cubemap, and re-scale capture FBO to prefilter scale.
    glGenTextures(1, &prefilterMap);
    glBindTexture(GL_TEXTURE_CUBE_MAP, prefilterMap);
    for (unsigned int i = 0; i < 6; ++i)
    {
        glTexImage2D(GL_TEXTURE_CUBE_MAP_POSITIVE_X + i, 0, GL_RGB16F, 128, 128, 0, GL_RGB, GL_FLOAT, nullptr);
    }
    glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_WRAP_R, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_LINEAR); // be sure to set minification filter to mip_linear 
    glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    // generate mipmaps for the cubemap so OpenGL automatically allocates the required memory.
    glGenerateMipmap(GL_TEXTURE_CUBE_MAP);

    // pre-filter the environment map and store the result in the prefilter cubemap.
    prefilterShader.Bind();
    prefilterShader.SetInt("environmentMap", 0);
    prefilterShader.SetMat4("projection", captureProjection);
    glActiveTexture(GL_TEXTURE0);
    glBindTexture(GL_TEXTURE_CUBE_MAP, envCubemap);

    glBindFramebuffer(GL_FRAMEBUFFER, captureFBO);
    unsigned int maxMipLevels = 5;
    for (unsigned int mip = 0; mip < maxMipLevels; ++mip){
        // reisze framebuffer according to mip-level size.
        unsigned int mipWidth  = static_cast<unsigned int>(128 * std::pow(0.5, mip));
        unsigned int mipHeight = static_cast<unsigned int>(128 * std::pow(0.5, mip));
        glBindRenderbuffer(GL_RENDERBUFFER, captureRBO);
        glRenderbufferStorage(GL_RENDERBUFFER, GL_DEPTH_COMPONENT24, mipWidth, mipHeight);
        glViewport(0, 0, mipWidth, mipHeight);

        float roughness = (float)mip / (float)(maxMipLevels - 1);
        prefilterShader.SetFloat("roughness", roughness);
        for (unsigned int i = 0; i < 6; ++i)
        {
            prefilterShader.SetMat4("view", captureViews[i]);
            glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_CUBE_MAP_POSITIVE_X + i, prefilterMap, mip);

            glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
            m_Cube.Draw(prefilterShader);
        }
    }
    glBindFramebuffer(GL_FRAMEBUFFER, 0);

    // generate a 2D LUT from the BRDF equations used.
    glGenTextures(1, &brdfLUTTexture);
    // pre-allocate enough memory for the LUT texture.
    glBindTexture(GL_TEXTURE_2D, brdfLUTTexture);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RG16F, 512, 512, 0, GL_RG, GL_FLOAT, nullptr);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    // then re-configure capture framebuffer object and render screen-space quad with BRDF shader.
    glBindFramebuffer(GL_FRAMEBUFFER, captureFBO);
    glBindRenderbuffer(GL_RENDERBUFFER, captureRBO);
    glRenderbufferStorage(GL_RENDERBUFFER, GL_DEPTH_COMPONENT24, 512, 512);
    glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, brdfLUTTexture, 0);
    glViewport(0, 0, 512, 512);
    brdfShader.Bind();
    CheckFBO("brdfLUT");
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    m_Quad.Draw(brdfShader);
    CheckGL("after brdfLUT bake");

    // delete framebuffer and renderbuffer
    glBindFramebuffer(GL_FRAMEBUFFER, 0);
    glDeleteFramebuffers(1, &captureFBO);
    glDeleteRenderbuffers(1, &captureRBO);
    spdlog::info("OpenGLPBRPass: BakeEnvironment complete — envCubemap={} irradiance={} prefilter={} brdfLUT={}",
        envCubemap, irradianceMap, prefilterMap, brdfLUTTexture);
}

void OpenGLPBRPass::BindIBL(const OpenGLShader& PBRShader) 
{
    // Implementation for binding IBL
    PBRShader.SetInt("irradianceMap", 5);
    PBRShader.SetInt("prefilterMap",  6);
    PBRShader.SetInt("brdfLUT",       7);
    glActiveTexture(GL_TEXTURE5);
    glBindTexture(GL_TEXTURE_CUBE_MAP, irradianceMap);
    glActiveTexture(GL_TEXTURE6);
    glBindTexture(GL_TEXTURE_CUBE_MAP, prefilterMap);
    glActiveTexture(GL_TEXTURE7);
    glBindTexture(GL_TEXTURE_2D, brdfLUTTexture);
}


// Per-frame surface pass: sets all uniforms and draws each mesh.
void OpenGLPBRPass::Render(
    const OpenGLShader& shader,
    const std::vector<DrawCall>& draws,
    const PBRMaterial& material,
    const glm::mat4& view,
    const glm::mat4& projection,
    const glm::vec3& cameraPos,
    const glm::vec3 lightPositions[4],
    const glm::vec3 lightColors[4],
    int viewportW, int viewportH)
{
    glViewport(0, 0, viewportW, viewportH);

    // Set up shader
    shader.Bind();
    shader.SetMat4("view", view);
    shader.SetMat4("projection", projection);
    shader.SetVec3("camPos", cameraPos);

    // set lights for object
    for (int i = 0; i < 4; ++i)
    {
        shader.SetVec3("lightPositions[" + std::to_string(i) + "]", lightPositions[i]);
        shader.SetVec3("lightColors["    + std::to_string(i) + "]", lightColors[i]);
    }

    BindIBL(shader);
    material.Bind(shader);

    for (const auto& draw : draws)
    {
        shader.SetMat4("model", draw.modelMatrix);
        shader.SetMat3("normalMatrix",
            glm::transpose(glm::inverse(glm::mat3(draw.modelMatrix))));
        draw.mesh->Draw(shader);
    }
}

// Skybox pass: renders env cubemap as background (call after Render).
void OpenGLPBRPass::DrawSkybox(
    const OpenGLShader& backgroundShader,
    const glm::mat4& view,
    const glm::mat4& projection,
    int viewportW, int viewportH)
{
    glViewport(0, 0, viewportW, viewportH);
    glDepthFunc(GL_LEQUAL);

    backgroundShader.Bind();
    backgroundShader.SetMat4("view", view);
    backgroundShader.SetMat4("projection", projection);
    backgroundShader.SetInt("environmentMap", 0);
    glActiveTexture(GL_TEXTURE0);
    glBindTexture(GL_TEXTURE_CUBE_MAP, envCubemap);

    m_Cube.Draw(backgroundShader);

    glDepthFunc(GL_LESS);

}

} // namespace Diamond