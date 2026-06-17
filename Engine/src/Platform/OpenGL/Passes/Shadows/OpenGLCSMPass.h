#pragma once
#include "Platform/OpenGL/Resources/OpenGLShader.h"
#include "Platform/OpenGL/Resources/OpenGLRenderTypes.h"
#include <glm/glm.hpp>
#include <array>
#include <vector>
#include <cstdint>

namespace Diamond {

class OpenGLCSMPass {
public:
    static constexpr int NUM_CASCADES = 4;

    OpenGLCSMPass() = default;
    ~OpenGLCSMPass();

    // Allocates the GL_TEXTURE_2D_ARRAY (NUM_CASCADES layers) and per-cascade FBOs.
    void Setup(int resolution = 2048);

    // Each frame: recomputes cascade splits + light-space matrices, then renders all cascades.
    // Draws with a bone palette use `skinnedShader`, the rest `depthShader`.
    void Render(const OpenGLShader&          depthShader,
                const OpenGLShader&          skinnedShader,
                const std::vector<DrawCall>& draws,
                const SunLight&              sun,
                const glm::mat4&             cameraView,
                const glm::mat4&             cameraProj,
                float                        cameraNear,
                float                        cameraFar);

    // Consumed by the lighting pass.
    uint32_t GetShadowArray() const { return m_ShadowArray; }
    const std::array<glm::mat4, NUM_CASCADES>& GetLightMatrices() const { return m_LightMatrices; }
    // View-space far-plane distance of each cascade; fragment shader uses these to select a layer.
    const std::array<float, NUM_CASCADES>& GetSplitDepths() const { return m_SplitDepths; }

private:
    // Fills m_LightMatrices and m_SplitDepths from camera frustum + sun direction.
    void ComputeCascades(const SunLight&  sun,
                         const glm::mat4& cameraView,
                         const glm::mat4& cameraProj,
                         float            near,
                         float            far);

    uint32_t m_FBOs[NUM_CASCADES] = {};
    uint32_t m_ShadowArray        = 0;   // GL_TEXTURE_2D_ARRAY with NUM_CASCADES layers
    int      m_Resolution         = 0;

    std::array<glm::mat4, NUM_CASCADES> m_LightMatrices{};
    std::array<float,     NUM_CASCADES> m_SplitDepths{};
};

} // namespace Diamond
