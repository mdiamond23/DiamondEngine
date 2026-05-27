#include "OpenGLTonemapPass.h"
#include <glad/gl.h>
#include <spdlog/spdlog.h>
#include "Renderer/MeshData.h"

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

OpenGLTonemapPass::OpenGLTonemapPass()
    : m_Quad(MeshData::FullscreenQuad())
{}

OpenGLTonemapPass::~OpenGLTonemapPass()
{
    if (m_HDRFBO)      glDeleteFramebuffers(1,  &m_HDRFBO);
    if (m_HDRColorTex) glDeleteTextures(1,       &m_HDRColorTex);
    if (m_HDRDepthRBO) glDeleteRenderbuffers(1,  &m_HDRDepthRBO);
}

void OpenGLTonemapPass::SetupHDRFramebuffer(int w, int h)
{
    m_HDRWidth  = w;
    m_HDRHeight = h;

    glGenFramebuffers(1, &m_HDRFBO);
    glBindFramebuffer(GL_FRAMEBUFFER, m_HDRFBO);

    glGenTextures(1, &m_HDRColorTex);
    glBindTexture(GL_TEXTURE_2D, m_HDRColorTex);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA16F, w, h, 0, GL_RGBA, GL_FLOAT, nullptr);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, m_HDRColorTex, 0);

    glGenRenderbuffers(1, &m_HDRDepthRBO);
    glBindRenderbuffer(GL_RENDERBUFFER, m_HDRDepthRBO);
    glRenderbufferStorage(GL_RENDERBUFFER, GL_DEPTH_COMPONENT24, w, h);
    glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_RENDERBUFFER, m_HDRDepthRBO);

    CheckFBO("HDR framebuffer");
    glBindFramebuffer(GL_FRAMEBUFFER, 0);
}

void OpenGLTonemapPass::BeginHDR(int w, int h)
{
    if (w != m_HDRWidth || h != m_HDRHeight) {
        glDeleteTextures(1,      &m_HDRColorTex);
        glDeleteRenderbuffers(1, &m_HDRDepthRBO);
        glDeleteFramebuffers(1,  &m_HDRFBO);
        m_HDRFBO = m_HDRColorTex = m_HDRDepthRBO = 0;
        SetupHDRFramebuffer(w, h);
    }
    glBindFramebuffer(GL_FRAMEBUFFER, m_HDRFBO);
    glViewport(0, 0, w, h);
    glClearColor(0.08f, 0.09f, 0.11f, 1.0f);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
}

void OpenGLTonemapPass::RenderTonemapPass(const OpenGLShader& tonemapShader)
{
    RenderTonemapPassFromTex(tonemapShader, m_HDRColorTex);
}

void OpenGLTonemapPass::RenderTonemapPassFromTex(const OpenGLShader& tonemapShader, uint32_t hdrColorTex)
{
    glBindFramebuffer(GL_FRAMEBUFFER, 0);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    tonemapShader.Bind();
    tonemapShader.SetInt("hdrBuffer", 0);
    glActiveTexture(GL_TEXTURE0);
    glBindTexture(GL_TEXTURE_2D, hdrColorTex);

    m_Quad.Draw(tonemapShader);
}

} // namespace Diamond
