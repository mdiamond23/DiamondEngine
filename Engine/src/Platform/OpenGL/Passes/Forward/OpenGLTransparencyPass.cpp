#include "Platform/OpenGL/Passes/Forward/OpenGLTransparencyPass.h"
#include <glad/gl.h>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <algorithm>

namespace Diamond {

void OpenGLTransparencyPass::Render(
    const OpenGLShader&   shader,
    std::vector<DrawCall> draws,
    uint32_t              albedoTex,
    uint32_t              gBufferFBO,
    uint32_t              hdrFBO,
    const glm::mat4&      view,
    const glm::mat4&      projection,
    const glm::vec3&      cameraPos,
    int viewportW, int viewportH)
{
    // Blit depth from G-buffer into HDR FBO so transparent objects depth-test against opaques
    glBindFramebuffer(GL_READ_FRAMEBUFFER, gBufferFBO);
    glBindFramebuffer(GL_DRAW_FRAMEBUFFER, hdrFBO);
    glBlitFramebuffer(0, 0, viewportW, viewportH,
                      0, 0, viewportW, viewportH,
                      GL_DEPTH_BUFFER_BIT, GL_NEAREST);

    glBindFramebuffer(GL_FRAMEBUFFER, hdrFBO);
    glViewport(0, 0, viewportW, viewportH);

    // Sort back-to-front so farther objects blend correctly
    std::sort(draws.begin(), draws.end(), [&cameraPos](const DrawCall& a, const DrawCall& b) {
        float da = glm::length(glm::vec3(a.modelMatrix[3]) - cameraPos);
        float db = glm::length(glm::vec3(b.modelMatrix[3]) - cameraPos);
        return da > db;
    });

    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    glDepthMask(GL_FALSE);

    shader.Bind();
    shader.SetMat4("view", view);
    shader.SetMat4("projection", projection);
    shader.SetInt("albedo", 0);
    glActiveTexture(GL_TEXTURE0);
    glBindTexture(GL_TEXTURE_2D, albedoTex);

    for (const auto& draw : draws) {
        shader.SetMat4("model", draw.modelMatrix);
        draw.mesh->Draw(shader);
    }

    glDepthMask(GL_TRUE);
    glDisable(GL_BLEND);
}

} // namespace Diamond