#pragma once
#include "Platform/OpenGL/Resources/OpenGLShader.h"
#include "Platform/OpenGL/Resources/OpenGLRenderTypes.h"
#include <glm/glm.hpp>
#include <vector>

namespace Diamond {

class OpenGLShadowPass {
public:
    OpenGLShadowPass() = default;
    ~OpenGLShadowPass();

    void SetupShadowMap(int resolution = 2048);
    void RenderShadowPass(const OpenGLShader&          depthShader,
                          const std::vector<DrawCall>& draws,
                          const SunLight&              sun);

    void SetupPointShadowMaps(int count = 4, int resolution = 512, float farPlane = 25.0f);
    void RenderPointShadowPass(const OpenGLShader&          depthShader,
                               const std::vector<DrawCall>& draws,
                               const glm::vec3              lightPositions[],
                               int                          lightCount);

    unsigned int GetShadowMap()            const { return m_ShadowMap; }
    unsigned int GetPointShadowMap(int i)  const { return m_PointShadowMaps[i]; }
    float        GetPointShadowFarPlane()  const { return m_PointShadowFarPlane; }
    int          GetPointShadowCount()     const { return m_PointShadowCount; }

private:
    unsigned int m_ShadowMapFBO = 0;
    unsigned int m_ShadowMap    = 0;
    int          m_ShadowRes    = 0;

    static constexpr int k_MaxPointLights = 4;
    unsigned int m_PointShadowFBOs[k_MaxPointLights] = {};
    unsigned int m_PointShadowMaps[k_MaxPointLights] = {};
    int          m_PointShadowRes      = 0;
    float        m_PointShadowFarPlane = 25.0f;
    int          m_PointShadowCount    = 0;
};

} // namespace Diamond
