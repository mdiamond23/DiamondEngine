#include "Platform/OpenGL/Passes/Shadows/OpenGLShadowPass.h"
#include "Profiling/GLRendererStats.h"
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
    for (int i = 0; i < m_PointShadowCount; ++i) {
        if (m_PointShadowFBOs[i]) glDeleteFramebuffers(1, &m_PointShadowFBOs[i]);
        if (m_PointShadowMaps[i]) glDeleteTextures(1, &m_PointShadowMaps[i]);
    }
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
    const OpenGLShader&          skinnedShader,
    const std::vector<DrawCall>& draws,
    const glm::vec3              lightPositions[],
    int                          lightCount)
{
    glViewport(0, 0, m_PointShadowRes, m_PointShadowRes);

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

        glBindFramebuffer(GL_FRAMEBUFFER, m_PointShadowFBOs[i]);
        glClear(GL_DEPTH_BUFFER_BIT);
        glEnable(GL_DEPTH_TEST);

        // The per-light cube matrices + lightPos must be set on whichever shader
        // is bound, so push them again on each shader switch.
        const OpenGLShader* bound = nullptr;
        auto bindShader = [&](const OpenGLShader& s) {
            s.Bind();
            s.SetFloat("farPlane", m_PointShadowFarPlane);
            for (int f = 0; f < 6; ++f)
                s.SetMat4("shadowMatrices[" + std::to_string(f) + "]", views[f]);
            s.SetVec3("lightPos", lp);
            bound = &s;
        };

        for (const auto& draw : draws) {
            const bool          skinned = (draw.bonePalette != nullptr && draw.boneCount > 0);
            const OpenGLShader& shader  = skinned ? skinnedShader : depthShader;
            if (bound != &shader) bindShader(shader);
            shader.SetMat4("model", draw.modelMatrix);
            if (skinned)
                shader.SetMat4Array("uBones", draw.bonePalette, draw.boneCount);
            draw.mesh->Draw(shader);
            GLStats::RecordShadowCaster();
        }
    }
    glBindFramebuffer(GL_FRAMEBUFFER, 0);
}

} // namespace Diamond
