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

struct SunLight {
    glm::vec3 direction   = glm::vec3(-0.3f, -1.0f, -0.5f);
    glm::vec3 color       = glm::vec3(5.0f);
    glm::mat4 lightSpaceMatrix = glm::mat4(1.0f);
};

class OpenGLPBRPass {
public:
    OpenGLPBRPass();
    ~OpenGLPBRPass();

    void BakeEnvironment(const OpenGLTexture& environmentMap, const OpenGLShader& equirectangularToCubemapShader, const OpenGLShader& irradianceShader, const OpenGLShader& prefilterShader, const OpenGLShader& brdfShader);
    void BindIBL(const OpenGLShader& PBRShader);

    // Allocates the directional shadow map FBO + depth texture. Call once after GL init.
    void SetupShadowMap(int resolution = 2048);

    // Depth pass: renders draw calls into the shadow map from the sun's perspective.
    void RenderShadowPass(const OpenGLShader& depthShader,
                          const std::vector<DrawCall>& draws,
                          const SunLight& sun);

    // Allocates cubemap FBOs for omnidirectional point shadows. Call once after GL init.
    void SetupPointShadowMaps(int count = 4, int resolution = 512, float farPlane = 25.0f);

    // Depth pass: renders draw calls into each point light's cubemap shadow map.
    void RenderPointShadowPass(const OpenGLShader& depthShader,
                               const std::vector<DrawCall>& draws,
                               const glm::vec3 lightPositions[],
                               int lightCount);

    // Per-frame surface pass: sets all uniforms and draws each mesh.
    void Render(const OpenGLShader& shader,
                const std::vector<DrawCall>& draws,
                const PBRMaterial& material,
                const glm::mat4& view,
                const glm::mat4& projection,
                const glm::vec3& cameraPos,
                const glm::vec3 lightPositions[4],
                const glm::vec3 lightColors[4],
                const SunLight& sun,
                int viewportW, int viewportH);

    // Skybox pass: renders env cubemap as background (call after Render).
    void DrawSkybox(const OpenGLShader& backgroundShader,
                    const glm::mat4&    view,
                    const glm::mat4&    projection,
                    int viewportW, int viewportH);

    void SetupHDRFramebuffer(int w, int h);
    void BeginHDR(int w, int h);
    void RenderTonemapPass(const OpenGLShader& tonemapShader);

private:
    // PBR and IBL
    unsigned int envCubemap = 0;
    unsigned int irradianceMap = 0;
    unsigned int prefilterMap = 0;
    unsigned int brdfLUTTexture = 0;

    // Directional Shadows
    unsigned int m_ShadowMapFBO = 0;
    unsigned int m_ShadowMap = 0;
    int m_ShadowRes = 0;

    // Point Shadows
    static constexpr int k_MaxPointLights = 4;
    unsigned int m_PointShadowFBOs[k_MaxPointLights] = {};
    unsigned int m_PointShadowMaps[k_MaxPointLights] = {};
    int m_PointShadowRes = 0;
    float m_PointShadowFarPlane = 25.0f;
    int m_PointShadowCount = 0;

    // Post Processing
    unsigned int m_HDRFBO = 0;
    unsigned int m_HDRColorTex = 0; // GL_RGBA16F
    unsigned int m_HDRDepthRBO = 0; // renderbuffer for depth
    int m_HDRWidth  = 0;
    int m_HDRHeight = 0;

    OpenGLMesh m_Cube;
    OpenGLMesh m_Quad;
};

} // namespace Diamond

