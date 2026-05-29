#pragma once
#include "Platform/OpenGL/Resources/OpenGLShader.h"
#include "Platform/OpenGL/Resources/OpenGLMesh.h"
#include <cstdint>

namespace Diamond {

class OpenGLFXAAPass {
public:
    OpenGLFXAAPass();

    // Renders sourceTex → backbuffer (FBO 0) with FXAA applied when enabled.
    void Render(const OpenGLShader& shader,
                uint32_t sourceTex,
                int viewportW, int viewportH,
                bool enabled = true);

private:
    OpenGLMesh m_Quad;
};

} // namespace Diamond
