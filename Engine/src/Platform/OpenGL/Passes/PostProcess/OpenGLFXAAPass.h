#pragma once
#include "Platform/OpenGL/Resources/OpenGLShader.h"
#include "Platform/OpenGL/Resources/OpenGLMesh.h"
#include <cstdint>

namespace Diamond {

class OpenGLFXAAPass {
public:
    OpenGLFXAAPass();

    void Render(const OpenGLShader& shader,
                uint32_t sourceTex,
                int viewportW, int viewportH,
                bool enabled = true,
                uint32_t outputFBO = 0);

private:
    OpenGLMesh m_Quad;
};

} // namespace Diamond
