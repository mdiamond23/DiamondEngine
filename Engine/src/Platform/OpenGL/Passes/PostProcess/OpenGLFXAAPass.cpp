#include "Platform/OpenGL/Passes/PostProcess/OpenGLFXAAPass.h"
#include "Renderer/MeshData.h"
#include <glad/gl.h>
#include <glm/glm.hpp>

namespace Diamond {

OpenGLFXAAPass::OpenGLFXAAPass()
    : m_Quad(MeshData::FullscreenQuad())
{}

void OpenGLFXAAPass::Render(const OpenGLShader& shader,
                             uint32_t sourceTex,
                             int viewportW, int viewportH,
                             bool enabled)
{
    glBindFramebuffer(GL_FRAMEBUFFER, 0);
    glViewport(0, 0, viewportW, viewportH);
    glDisable(GL_DEPTH_TEST);
    glClear(GL_COLOR_BUFFER_BIT);

    shader.Bind();
    shader.SetInt("screenTex", 0);
    shader.SetVec2("texelSize", glm::vec2(1.0f / viewportW, 1.0f / viewportH));
    shader.SetBool("enabled", enabled);

    glActiveTexture(GL_TEXTURE0);
    glBindTexture(GL_TEXTURE_2D, sourceTex);

    m_Quad.Draw(shader);
    glEnable(GL_DEPTH_TEST);
}

} // namespace Diamond
