#include "OpenGLPBRSurfacePass.h"
#include "OpenGLShadowPass.h"
#include "OpenGLIBLPass.h"
#include <glad/gl.h>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <string>
#include "Renderer/MeshData.h"

namespace Diamond {

OpenGLPBRSurfacePass::OpenGLPBRSurfacePass()
    : m_Cube(MeshData::UnitCube())
{}

void OpenGLPBRSurfacePass::Render(
    const OpenGLShader&          shader,
    const std::vector<DrawCall>& draws,
    const PBRMaterial&           material,
    const glm::mat4&             view,
    const glm::mat4&             projection,
    const glm::vec3&             cameraPos,
    const glm::vec3              lightPositions[4],
    const glm::vec3              lightColors[4],
    const SunLight&              sun,
    const OpenGLShadowPass&      shadowPass,
    const OpenGLIBLPass&         iblPass,
    int viewportW, int viewportH)
{
    glViewport(0, 0, viewportW, viewportH);

    shader.Bind();
    shader.SetMat4("view", view);
    shader.SetMat4("projection", projection);
    shader.SetVec3("camPos", cameraPos);
    shader.SetMat4("lightSpaceMatrix", sun.lightSpaceMatrix);
    shader.SetVec3("sunDirection", sun.direction);
    shader.SetVec3("sunColor",     sun.color);
    shader.SetInt("shadowMap", 8);
    glActiveTexture(GL_TEXTURE8);
    glBindTexture(GL_TEXTURE_2D, shadowPass.GetShadowMap());

    for (int i = 0; i < 4; ++i) {
        shader.SetVec3("lightPositions[" + std::to_string(i) + "]", lightPositions[i]);
        shader.SetVec3("lightColors["    + std::to_string(i) + "]", lightColors[i]);
    }

    iblPass.BindIBL(shader);

    shader.SetFloat("shadowFarPlane", shadowPass.GetPointShadowFarPlane());
    for (int i = 0; i < shadowPass.GetPointShadowCount(); ++i) {
        int slot = 9 + i;
        shader.SetInt("pointShadowMaps[" + std::to_string(i) + "]", slot);
        glActiveTexture(GL_TEXTURE0 + slot);
        glBindTexture(GL_TEXTURE_CUBE_MAP, shadowPass.GetPointShadowMap(i));
    }

    material.Bind(shader);

    for (const auto& draw : draws) {
        shader.SetMat4("model", draw.modelMatrix);
        shader.SetMat3("normalMatrix",
            glm::transpose(glm::inverse(glm::mat3(draw.modelMatrix))));
        draw.mesh->Draw(shader);
    }
}

void OpenGLPBRSurfacePass::DrawSkybox(
    const OpenGLShader&  backgroundShader,
    const OpenGLIBLPass& iblPass,
    const glm::mat4&     view,
    const glm::mat4&     projection,
    int viewportW, int viewportH)
{
    glViewport(0, 0, viewportW, viewportH);
    glDepthFunc(GL_LEQUAL);

    backgroundShader.Bind();
    backgroundShader.SetMat4("view", view);
    backgroundShader.SetMat4("projection", projection);
    backgroundShader.SetInt("environmentMap", 0);
    glActiveTexture(GL_TEXTURE0);
    glBindTexture(GL_TEXTURE_CUBE_MAP, iblPass.GetEnvCubemap());

    m_Cube.Draw(backgroundShader);

    glDepthFunc(GL_LESS);
}

} // namespace Diamond
