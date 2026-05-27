#pragma once
#include "OpenGLShader.h"
#include "OpenGLMesh.h"

namespace Diamond {

class OpenGLTonemapPass {
public:
    OpenGLTonemapPass();
    ~OpenGLTonemapPass();

    void SetupHDRFramebuffer(int w, int h);
    void BeginHDR(int w, int h);
    void RenderTonemapPass(const OpenGLShader& tonemapShader);

    // Variant used by the render graph — takes an externally-managed HDR texture ID
    void RenderTonemapPassFromTex(const OpenGLShader& tonemapShader, uint32_t hdrColorTex);

private:
    unsigned int m_HDRFBO      = 0;
    unsigned int m_HDRColorTex = 0;
    unsigned int m_HDRDepthRBO = 0;
    int          m_HDRWidth    = 0;
    int          m_HDRHeight   = 0;

    OpenGLMesh m_Quad;
};

} // namespace Diamond
