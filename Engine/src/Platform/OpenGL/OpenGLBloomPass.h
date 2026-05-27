#pragma once
#include "OpenGLShader.h"
#include "OpenGLMesh.h"

namespace Diamond {

class OpenGLBloomPass {
public:
    OpenGLBloomPass();
    ~OpenGLBloomPass() = default;

    void RenderBlurPass(const OpenGLShader& blurShader,
                        uint32_t sourceTex,
                        uint32_t destFBO,
                        bool horizontal);

    void RenderCompositePass(const OpenGLShader& bloomFinalShader,
                            uint32_t sceneTex,
                            uint32_t bloomBlurTex,
                            bool bloomEnabled,
                            float exposure);
private:
    OpenGLMesh m_Quad;

};

} // namespace Diamond