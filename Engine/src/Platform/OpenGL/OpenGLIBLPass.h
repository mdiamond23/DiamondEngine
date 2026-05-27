#pragma once
#include "OpenGLTexture.h"
#include "OpenGLShader.h"
#include "OpenGLMesh.h"

namespace Diamond {

class OpenGLIBLPass {
public:
    OpenGLIBLPass();
    ~OpenGLIBLPass();

    void BakeEnvironment(
        const OpenGLTexture& environmentMap,
        const OpenGLShader&  equirectangularToCubemapShader,
        const OpenGLShader&  irradianceShader,
        const OpenGLShader&  prefilterShader,
        const OpenGLShader&  brdfShader);

    void BindIBL(const OpenGLShader& pbrShader) const;

    unsigned int GetEnvCubemap() const { return m_EnvCubemap; }

private:
    unsigned int m_EnvCubemap     = 0;
    unsigned int m_IrradianceMap  = 0;
    unsigned int m_PrefilterMap   = 0;
    unsigned int m_BrdfLUT        = 0;

    OpenGLMesh m_Cube;
    OpenGLMesh m_Quad;
};

} // namespace Diamond
