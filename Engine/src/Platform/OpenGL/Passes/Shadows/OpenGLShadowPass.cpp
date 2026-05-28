#include "Platform/OpenGL/Passes/Shadows/OpenGLShadowPass.h"
#include <glad/gl.h>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <string>
#include <spdlog/spdlog.h>

namespace {

static void CheckFBO(const char* label)
{
    GLenum status = glCheckFramebufferStatus(GL_FRAMEBUFFER);
    if (status != GL_FRAMEBUFFER_COMPLETE)
        spdlog::error("FBO incomplete at [{}]: 0x{:04X}", label, status);
    else
        spdlog::info("FBO complete at [{}]", label);
}

} // namespace

namespace Diamond {

OpenGLShadowPass::~OpenGLShadowPass()
{
    if (m_ShadowMapFBO) glDeleteFramebuffers(1, &m_ShadowMapFBO);
    if (m_ShadowMap)    glDeleteTextures(1, &m_ShadowMap);
    for (int i = 0; i < m_PointShadowCount; ++i) {
        if (m_PointShadowFBOs[i]) glDeleteFramebuffers(1, &m_PointShadowFBOs[i]);
        if (m_PointShadowMaps[i]) glDeleteTextures(1, &m_PointShadowMaps[i]);
    }
}

void OpenGLShadowPass::SetupShadowMap(int resolution)
{
    m_ShadowRes = resolution;

    glGenTextures(1, &m_ShadowMap);
    glBindTexture(GL_TEXTURE_2D, m_ShadowMap);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_DEPTH_COMPONENT,
                 resolution, resolution, 0, GL_DEPTH_COMPONENT, GL_FLOAT, nullptr);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_BORDER);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_BORDER);
    float borderColor[] = { 1.0f, 1.0f, 1.0f, 1.0f };
    glTexParameterfv(GL_TEXTURE_2D, GL_TEXTURE_BORDER_COLOR, borderColor);

    glGenFramebuffers(1, &m_ShadowMapFBO);
    glBindFramebuffer(GL_FRAMEBUFFER, m_ShadowMapFBO);
    glFramebufferTexture2D(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_TEXTURE_2D, m_ShadowMap, 0);
    glDrawBuffer(GL_NONE);
    glReadBuffer(GL_NONE);
    CheckFBO("shadow map");
    glBindFramebuffer(GL_FRAMEBUFFER, 0);
}

void OpenGLShadowPass::RenderShadowPass(
    const OpenGLShader&          depthShader,
    const std::vector<DrawCall>& draws,
    const SunLight&              sun)
{
    glViewport(0, 0, m_ShadowRes, m_ShadowRes);
    glBindFramebuffer(GL_FRAMEBUFFER, m_ShadowMapFBO);
    glClear(GL_DEPTH_BUFFER_BIT);

    depthShader.Bind();
    depthShader.SetMat4("lightSpaceMatrix", sun.lightSpaceMatrix);

    glEnable(GL_DEPTH_TEST);
    glCullFace(GL_FRONT);
    for (const auto& draw : draws) {
        depthShader.SetMat4("model", draw.modelMatrix);
        draw.mesh->Draw(depthShader);
    }
    glCullFace(GL_BACK);

    glBindFramebuffer(GL_FRAMEBUFFER, 0);
}

void OpenGLShadowPass::SetupPointShadowMaps(int count, int resolution, float farPlane)
{
    m_PointShadowCount    = count;
    m_PointShadowRes      = resolution;
    m_PointShadowFarPlane = farPlane;

    for (int i = 0; i < count; ++i) {
        glGenTextures(1, &m_PointShadowMaps[i]);
        glBindTexture(GL_TEXTURE_CUBE_MAP, m_PointShadowMaps[i]);
        for (int face = 0; face < 6; ++face)
            glTexImage2D(GL_TEXTURE_CUBE_MAP_POSITIVE_X + face, 0, GL_DEPTH_COMPONENT,
                         resolution, resolution, 0, GL_DEPTH_COMPONENT, GL_FLOAT, nullptr);
        glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
        glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
        glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
        glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
        glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_WRAP_R, GL_CLAMP_TO_EDGE);

        glGenFramebuffers(1, &m_PointShadowFBOs[i]);
        glBindFramebuffer(GL_FRAMEBUFFER, m_PointShadowFBOs[i]);
        glFramebufferTexture(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, m_PointShadowMaps[i], 0);
        glDrawBuffer(GL_NONE);
        glReadBuffer(GL_NONE);
        CheckFBO("point shadow cubemap");
        glBindFramebuffer(GL_FRAMEBUFFER, 0);
    }
}

void OpenGLShadowPass::RenderPointShadowPass(
    const OpenGLShader&          depthShader,
    const std::vector<DrawCall>& draws,
    const glm::vec3              lightPositions[],
    int                          lightCount)
{
    glViewport(0, 0, m_PointShadowRes, m_PointShadowRes);
    depthShader.Bind();
    depthShader.SetFloat("farPlane", m_PointShadowFarPlane);

    glm::mat4 shadowProj = glm::perspective(
        glm::radians(90.0f), 1.0f, 0.1f, m_PointShadowFarPlane);

    for (int i = 0; i < lightCount && i < m_PointShadowCount; ++i) {
        const glm::vec3& lp = lightPositions[i];
        glm::mat4 views[6] = {
            shadowProj * glm::lookAt(lp, lp + glm::vec3( 1, 0, 0), glm::vec3(0,-1, 0)),
            shadowProj * glm::lookAt(lp, lp + glm::vec3(-1, 0, 0), glm::vec3(0,-1, 0)),
            shadowProj * glm::lookAt(lp, lp + glm::vec3( 0, 1, 0), glm::vec3(0, 0, 1)),
            shadowProj * glm::lookAt(lp, lp + glm::vec3( 0,-1, 0), glm::vec3(0, 0,-1)),
            shadowProj * glm::lookAt(lp, lp + glm::vec3( 0, 0, 1), glm::vec3(0,-1, 0)),
            shadowProj * glm::lookAt(lp, lp + glm::vec3( 0, 0,-1), glm::vec3(0,-1, 0)),
        };
        for (int f = 0; f < 6; ++f)
            depthShader.SetMat4("shadowMatrices[" + std::to_string(f) + "]", views[f]);
        depthShader.SetVec3("lightPos", lp);

        glBindFramebuffer(GL_FRAMEBUFFER, m_PointShadowFBOs[i]);
        glClear(GL_DEPTH_BUFFER_BIT);
        glEnable(GL_DEPTH_TEST);
        for (const auto& draw : draws) {
            depthShader.SetMat4("model", draw.modelMatrix);
            draw.mesh->Draw(depthShader);
        }
    }
    glBindFramebuffer(GL_FRAMEBUFFER, 0);
}

} // namespace Diamond
