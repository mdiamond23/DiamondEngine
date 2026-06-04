#pragma once
#include "Platform/OpenGL/Resources/OpenGLShader.h"
#include "Platform/OpenGL/Resources/OpenGLMesh.h"
#include "Platform/OpenGL/Resources/OpenGLRenderTypes.h"
#include "Platform/OpenGL/Passes/IBL/OpenGLIBLPass.h"
#include "Platform/OpenGL/Passes/Shadows/OpenGLShadowPass.h"
#include "Platform/OpenGL/Passes/Shadows/OpenGLCSMPass.h"
#include <glm/glm.hpp>
#include <cstdint>
#include <vector>

namespace Diamond {

class OpenGLDeferredLightingPass {
public:
    OpenGLDeferredLightingPass();

    // Fullscreen lighting pass — reads G-buffer + SSAO + emissive, outputs into fbo (HDR + bright MRT).
    // Leaves fbo bound so the caller can blit G-buffer depth and draw the skybox.
    void Render(const OpenGLShader&      shader,
                uint32_t                 gViewPosTex,
                uint32_t                 gViewNormalTex,
                uint32_t                 gAlbedoTex,
                uint32_t                 gMaterialTex,
                uint32_t                 ssaoTex,
                uint32_t                 gEmissiveTex,
                const glm::mat4&         view,
                const glm::vec3&         camPos,
                const std::vector<glm::vec3>& ptPositions,
                const std::vector<glm::vec3>& ptColors,
                const std::vector<glm::vec3>& spotPositions,
                const std::vector<glm::vec3>& spotDirections,
                const std::vector<glm::vec3>& spotColors,
                const std::vector<float>&     spotCosInner,
                const std::vector<float>&     spotCosOuter,
                const SunLight&               sun,
                const OpenGLShadowPass&  shadowPass,
                const OpenGLCSMPass&     csmPass,
                const OpenGLIBLPass&     iblPass,
                uint32_t                 fbo,
                int viewportW, int viewportH);

    // Draw skybox into the currently bound FBO using depth already written there.
    void DrawSkybox(const OpenGLShader&  bgShader,
                    const OpenGLIBLPass& iblPass,
                    const glm::mat4&     view,
                    const glm::mat4&     projection,
                    int viewportW, int viewportH);

private:
    OpenGLMesh m_Quad;
    OpenGLMesh m_Cube;
};

} // namespace Diamond
