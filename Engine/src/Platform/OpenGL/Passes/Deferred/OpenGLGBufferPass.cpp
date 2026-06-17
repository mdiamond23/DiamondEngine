#include "Platform/OpenGL/Passes/Deferred/OpenGLGBufferPass.h"
#include <glad/gl.h>
#include <glm/gtc/matrix_transform.hpp>

namespace Diamond {

void OpenGLGBufferPass::Render(
    const OpenGLShader&          staticShader,
    const OpenGLShader&          skinnedShader,
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

    // Per-shader setup (view/projection + material) is only re-done when the
    // active shader actually changes, so an all-static or all-skinned batch pays
    // for it once.
    const OpenGLShader* bound = nullptr;
    auto bindShader = [&](const OpenGLShader& s) {
        s.Bind();
        s.SetMat4("view",       view);
        s.SetMat4("projection", projection);
        material.Bind(s);
        bound = &s;
    };

    for (const auto& draw : draws) {
        const bool          skinned = (draw.bonePalette != nullptr && draw.boneCount > 0);
        const OpenGLShader& shader  = skinned ? skinnedShader : staticShader;
        if (bound != &shader) bindShader(shader);

        shader.SetMat4("model", draw.modelMatrix);
        shader.SetMat3("normalMatrix",
            glm::transpose(glm::inverse(glm::mat3(draw.modelMatrix))));
        if (skinned)
            shader.SetMat4Array("uBones", draw.bonePalette, draw.boneCount);

        draw.mesh->Draw(shader);
    }

    glBindFramebuffer(GL_FRAMEBUFFER, 0);
}

} // namespace Diamond
