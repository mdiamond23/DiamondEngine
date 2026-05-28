#pragma once
#include "Platform/OpenGL/Resources/OpenGLShader.h"
#include "Platform/OpenGL/Resources/OpenGLMesh.h"
#include <glm/glm.hpp>
#include <cstdint>
#include <vector>

namespace Diamond {

class OpenGLSSAOPass {
public:
    OpenGLSSAOPass();
    ~OpenGLSSAOPass();

    // Renders raw (noisy) SSAO into fbo. fbo must have a GL_R16F color attachment.
    void RenderSSAO(const OpenGLShader& shader,
                    uint32_t gViewPosTex,
                    uint32_t gViewNormalTex,
                    const glm::mat4& projection,
                    uint32_t fbo,
                    int w, int h);

    // 4×4 box blur to smooth the raw SSAO result.
    void RenderBlur(const OpenGLShader& blurShader,
                    uint32_t ssaoRawTex,
                    uint32_t fbo,
                    int w, int h);

private:
    std::vector<glm::vec3> m_Kernel;
    uint32_t               m_NoiseTex = 0;
    OpenGLMesh             m_Quad;
};

} // namespace Diamond
