#pragma once
#include "OpenGLTexture.h"
#include "OpenGLShader.h"
#include "OpenGLMesh.h"
#include "Renderer/Material.h"
#include <glm/glm.hpp>
#include <vector>

namespace Diamond {

struct DrawCall {
    const Mesh* mesh;
    glm::mat4   modelMatrix;
};

class OpenGLPBRPass {
public:
    OpenGLPBRPass();
    ~OpenGLPBRPass();

    void BakeEnvironment(const OpenGLTexture& environmentMap, const OpenGLShader& equirectangularToCubemapShader, const OpenGLShader& irradianceShader, const OpenGLShader& prefilterShader, const OpenGLShader& brdfShader);
    void BindIBL(const OpenGLShader& PBRShader);

    // Per-frame surface pass: sets all uniforms and draws each mesh.
    void Render(const OpenGLShader&          shader,
                const std::vector<DrawCall>& draws,
                const PBRMaterial&           material,
                const glm::mat4&             view,
                const glm::mat4&             projection,
                const glm::vec3&             cameraPos,
                const glm::vec3              lightPositions[4],
                const glm::vec3              lightColors[4],
                int viewportW, int viewportH);

    // Skybox pass: renders env cubemap as background (call after Render).
    void DrawSkybox(const OpenGLShader& backgroundShader,
                    const glm::mat4&    view,
                    const glm::mat4&    projection,
                    int viewportW, int viewportH);

private:
    unsigned int envCubemap     = 0;
    unsigned int irradianceMap  = 0;
    unsigned int prefilterMap   = 0;
    unsigned int brdfLUTTexture = 0;

    OpenGLMesh m_Cube;
    OpenGLMesh m_Quad;
};

} // namespace Diamond

