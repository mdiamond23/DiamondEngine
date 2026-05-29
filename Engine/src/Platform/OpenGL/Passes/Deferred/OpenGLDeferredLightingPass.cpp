#include "Platform/OpenGL/Passes/Deferred/OpenGLDeferredLightingPass.h"
#include "Renderer/MeshData.h"
#include <glad/gl.h>
#include <glm/gtc/matrix_transform.hpp>
#include <string>

namespace Diamond {

OpenGLDeferredLightingPass::OpenGLDeferredLightingPass()
    : m_Quad(MeshData::FullscreenQuad())
    , m_Cube(MeshData::UnitCube())
{}

void OpenGLDeferredLightingPass::Render(
    const OpenGLShader&      shader,
    uint32_t                 gViewPosTex,
    uint32_t                 gViewNormalTex,
    uint32_t                 gAlbedoTex,
    uint32_t                 gMaterialTex,
    uint32_t                 ssaoTex,
    uint32_t                 gEmissiveTex,
    const glm::mat4&         view,
    const glm::vec3&         camPos,
    const glm::vec3          lightPositions[4],
    const glm::vec3          lightColors[4],
    const SunLight&          sun,
    const OpenGLShadowPass&  shadowPass,
    const OpenGLIBLPass&     iblPass,
    uint32_t                 fbo,
    int viewportW, int viewportH)
{
    glBindFramebuffer(GL_FRAMEBUFFER, fbo);
    glViewport(0, 0, viewportW, viewportH);
    glClearColor(0.08f, 0.09f, 0.11f, 1.0f);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glDisable(GL_DEPTH_TEST);

    shader.Bind();

    // G-buffer inputs (slots 0-3) + SSAO (slot 4)
    shader.SetInt("gViewPos",    0);
    shader.SetInt("gViewNormal", 1);
    shader.SetInt("gAlbedo",     2);
    shader.SetInt("gMaterial",   3);
    shader.SetInt("ssaoTex",     4);
    glActiveTexture(GL_TEXTURE0); glBindTexture(GL_TEXTURE_2D, gViewPosTex);
    glActiveTexture(GL_TEXTURE1); glBindTexture(GL_TEXTURE_2D, gViewNormalTex);
    glActiveTexture(GL_TEXTURE2); glBindTexture(GL_TEXTURE_2D, gAlbedoTex);
    glActiveTexture(GL_TEXTURE3); glBindTexture(GL_TEXTURE_2D, gMaterialTex);
    glActiveTexture(GL_TEXTURE4); glBindTexture(GL_TEXTURE_2D, ssaoTex);

    // IBL (slots 5-7, managed by BindIBL)
    iblPass.BindIBL(shader);

    // Directional shadow (slot 8)
    shader.SetInt("shadowMap", 8);
    glActiveTexture(GL_TEXTURE8);
    glBindTexture(GL_TEXTURE_2D, shadowPass.GetShadowMap());

    // Point shadows (slots 9-12)
    shader.SetFloat("shadowFarPlane", shadowPass.GetPointShadowFarPlane());
    for (int i = 0; i < shadowPass.GetPointShadowCount(); ++i) {
        int slot = 9 + i;
        shader.SetInt("pointShadowMaps[" + std::to_string(i) + "]", slot);
        glActiveTexture(GL_TEXTURE0 + slot);
        glBindTexture(GL_TEXTURE_CUBE_MAP, shadowPass.GetPointShadowMap(i));
    }

    // Emissive G-buffer (slot 13)
    shader.SetInt("gEmissive", 13);
    glActiveTexture(GL_TEXTURE13); glBindTexture(GL_TEXTURE_2D, gEmissiveTex);

    shader.SetMat4("view",             view);
    shader.SetMat4("lightSpaceMatrix", sun.lightSpaceMatrix);
    shader.SetVec3("camPos",           camPos);
    shader.SetVec3("sunDirection",     sun.direction);
    shader.SetVec3("sunColor",         sun.color);
    for (int i = 0; i < 4; ++i) {
        shader.SetVec3("lightPositions[" + std::to_string(i) + "]", lightPositions[i]);
        shader.SetVec3("lightColors["    + std::to_string(i) + "]", lightColors[i]);
    }

    m_Quad.Draw(shader);
    // FBO intentionally left bound — caller blits G-buffer depth then draws skybox
}

void OpenGLDeferredLightingPass::DrawSkybox(
    const OpenGLShader&  bgShader,
    const OpenGLIBLPass& iblPass,
    const glm::mat4&     view,
    const glm::mat4&     projection,
    int viewportW, int viewportH)
{
    glViewport(0, 0, viewportW, viewportH);
    glEnable(GL_DEPTH_TEST);
    glDepthFunc(GL_LEQUAL);

    bgShader.Bind();
    bgShader.SetMat4("view",       view);
    bgShader.SetMat4("projection", projection);
    bgShader.SetInt("environmentMap", 0);
    glActiveTexture(GL_TEXTURE0);
    glBindTexture(GL_TEXTURE_CUBE_MAP, iblPass.GetEnvCubemap());

    m_Cube.Draw(bgShader);

    glDepthFunc(GL_LESS);
    glBindFramebuffer(GL_FRAMEBUFFER, 0);
}

} // namespace Diamond
