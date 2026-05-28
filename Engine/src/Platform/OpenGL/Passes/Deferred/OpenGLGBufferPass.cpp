#include "Platform/OpenGL/Passes/Deferred/OpenGLGBufferPass.h"
#include <glad/gl.h>
#include <glm/gtc/matrix_transform.hpp>

namespace Diamond {

void OpenGLGBufferPass::Render(
    const OpenGLShader&          shader,
    const std::vector<DrawCall>& draws,
    const PBRMaterial&           material,
    const glm::mat4&             view,
    const glm::mat4&             projection,
    uint32_t                     fbo,
    int viewportW, int viewportH)
{
    glBindFramebuffer(GL_FRAMEBUFFER, fbo);
    glViewport(0, 0, viewportW, viewportH);
    glEnable(GL_DEPTH_TEST);

    shader.Bind();
    shader.SetMat4("view",       view);
    shader.SetMat4("projection", projection);

    material.Bind(shader);

    for (const auto& draw : draws) {
        shader.SetMat4("model", draw.modelMatrix);
        shader.SetMat3("normalMatrix",
            glm::transpose(glm::inverse(glm::mat3(draw.modelMatrix))));
        draw.mesh->Draw(shader);
    }

    glBindFramebuffer(GL_FRAMEBUFFER, 0);
}

} // namespace Diamond
