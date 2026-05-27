#include "OpenGLBloomPass.h"
#include <glad/gl.h>
#include <spdlog/spdlog.h>
#include "Renderer/MeshData.h"


namespace Diamond {

OpenGLBloomPass::OpenGLBloomPass()
    : m_Quad(MeshData::FullscreenQuad())
{}

void OpenGLBloomPass::RenderBlurPass(const OpenGLShader& blurShader,
                    uint32_t sourceTex,
                    uint32_t destFBO,
                    bool horizontal)
{
    glBindFramebuffer(GL_FRAMEBUFFER, destFBO);

    blurShader.Bind();
    blurShader.SetInt("image", 0);
    blurShader.SetBool("horizontal", horizontal);
    glActiveTexture(GL_TEXTURE0);
    glBindTexture(GL_TEXTURE_2D, sourceTex);

    m_Quad.Draw(blurShader);
}

void OpenGLBloomPass::RenderCompositePass(const OpenGLShader& bloomFinalShader,
                        uint32_t sceneTex,
                        uint32_t bloomBlurTex,
                        bool bloomEnabled,
                        float exposure)
{
    glBindFramebuffer(GL_FRAMEBUFFER, 0);
    glDisable(GL_DEPTH_TEST);
    glClear(GL_COLOR_BUFFER_BIT);

    bloomFinalShader.Bind();
    bloomFinalShader.SetInt("hdrBuffer", 0);
    bloomFinalShader.SetInt("bloomBlur", 1);
    bloomFinalShader.SetBool("bloom", bloomEnabled);
    bloomFinalShader.SetFloat("exposure", exposure);

    glActiveTexture(GL_TEXTURE0);
    glBindTexture(GL_TEXTURE_2D, sceneTex);
    glActiveTexture(GL_TEXTURE1);
    glBindTexture(GL_TEXTURE_2D, bloomBlurTex);

    m_Quad.Draw(bloomFinalShader);
    glEnable(GL_DEPTH_TEST);
}


}

