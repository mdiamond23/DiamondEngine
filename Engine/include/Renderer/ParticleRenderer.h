#pragma once

#include <glm/glm.hpp>
#include <memory>
#include <vector>

namespace Diamond {

class Texture;

// One particle in the form the renderer consumes. The simulation produces arrays
// of these; the renderer knows nothing about how they were spawned or integrated.
// This struct is the contract between the simulation and rendering layers — keep
// it renderer-agnostic so mesh/ribbon renderers can consume the same data later.
struct RenderParticle {
    glm::vec3 position {0.0f};   // world-space center of the billboard
    float     size     = 1.0f;   // world-space edge length of the quad
    glm::vec4 color    {1.0f};   // straight (non-premultiplied) RGBA
    float     rotation = 0.0f;   // radians, spin around the view axis (screen-facing)
};

enum class ParticleBlend { Alpha, Additive };

// Draws particles as camera-facing textured billboards into the currently bound
// framebuffer. World-space and depth-tested (read-only) against scene geometry.
//
// v1 builds billboards on the CPU and batches them into one dynamic VBO per Draw,
// mirroring Renderer2D. A future GPU/instanced backend can replace this without
// touching the simulation, since the input is just RenderParticle arrays.
//
// Concrete backends are created via Create() and dispatched on the active
// RendererAPI, mirroring Shader/Texture/Renderer2D.
class ParticleRenderer {
public:
    virtual ~ParticleRenderer() = default;

    // Begin a frame. view/proj come from the active camera; the renderer pulls the
    // camera right/up axes out of `view` to orient every billboard. Sets up
    // depth-tested, depth-write-disabled blending for the duration.
    virtual void Begin(const glm::mat4& view, const glm::mat4& proj) = 0;

    // Draw a batch of particles sharing one texture and blend mode. Alpha-blended
    // batches should be sorted back-to-front by the caller; additive needs no sort.
    virtual void Draw(const std::vector<RenderParticle>& particles,
                      const Texture& texture, ParticleBlend blend) = 0;

    // Restore the prior GL/render state.
    virtual void End() = 0;

    static std::unique_ptr<ParticleRenderer> Create();
};

} // namespace Diamond
