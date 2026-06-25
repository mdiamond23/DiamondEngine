#pragma once

#include "Renderer/ParticleRenderer.h"

#include <cstdint>
#include <glm/glm.hpp>
#include <vector>

namespace Diamond {

// OpenGL billboard particle renderer. Each Draw expands its particles into
// camera-facing quads on the CPU (using the right/up axes pulled from the view
// matrix in Begin) and issues one dynamic-VBO draw call, mirroring the
// Renderer2D batching pattern. World-space, depth-tested against scene geometry
// but never writing depth, so overlapping transparent particles blend correctly.
class OpenGLParticleRenderer : public ParticleRenderer {
public:
    OpenGLParticleRenderer();
    ~OpenGLParticleRenderer() override;

    void Begin(const glm::mat4& view, const glm::mat4& proj) override;
    void Draw(const std::vector<RenderParticle>& particles,
              const Texture& texture, ParticleBlend blend) override;
    void End() override;

private:
    struct Vertex {
        glm::vec3 pos;     // world space
        glm::vec2 uv;
        glm::vec4 color;
    };

    uint32_t  m_VAO = 0, m_VBO = 0, m_Program = 0;
    glm::mat4 m_ViewProj {1.0f};
    glm::vec3 m_CamRight {1.0f, 0.0f, 0.0f};   // world-space camera axes from view
    glm::vec3 m_CamUp    {0.0f, 1.0f, 0.0f};
    std::vector<Vertex> m_Verts;

    // Saved GL state restored in End().
    bool    m_SavedDepthTest  = false;
    int32_t m_SavedDepthMask  = 1;
    bool    m_SavedBlend      = false;
    int32_t m_SavedBlendSrc   = 0, m_SavedBlendDst = 0;
};

} // namespace Diamond
