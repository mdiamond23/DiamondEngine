#pragma once
#include "OpenGLShader.h"
#include "OpenGLMesh.h"
#include "OpenGLRenderTypes.h"
#include "Renderer/Material.h"
#include <glm/glm.hpp>
#include <vector>

namespace Diamond {

class OpenGLIBLPass;
class OpenGLShadowPass;

class OpenGLPBRSurfacePass {
public:
    OpenGLPBRSurfacePass();

    void Render(const OpenGLShader&          shader,
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
                int viewportW, int viewportH);

    void DrawSkybox(const OpenGLShader&  backgroundShader,
                    const OpenGLIBLPass& iblPass,
                    const glm::mat4&     view,
                    const glm::mat4&     projection,
                    int viewportW, int viewportH);

private:
    OpenGLMesh m_Cube;
};

} // namespace Diamond
