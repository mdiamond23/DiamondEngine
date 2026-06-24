#pragma once

#include <glm/glm.hpp>
#include <memory>
#include <string>

namespace Diamond {

class Texture;
class Font;

// Backend-agnostic 2D overlay renderer: batches solid-color quads, textured
// quads, and text in a screen-space (pixel) coordinate system for HUDs and UI.
//
// Concrete backends are created via Renderer2D::Create() and dispatched on the
// active RendererAPI, mirroring Shader/Mesh/Texture — a Vulkan implementation
// slots in alongside the OpenGL one without touching call sites.
//
// Coordinate convention: top-left origin, +X right, +Y down. Use a projection
// from OrthoProjection() for a 1:1 pixel canvas.
class Renderer2D {
public:
    virtual ~Renderer2D() = default;

    // Begin a batch. `projection` maps the 2D coordinate space to clip space.
    // Sets up alpha blending and disables depth testing for the duration.
    virtual void Begin(const glm::mat4& projection) = 0;
    // Flush any batched geometry and restore the prior GL/render state.
    virtual void End() = 0;

    // Solid-color quad. `color` is straight (non-premultiplied) RGBA.
    virtual void DrawQuad(const glm::vec2& pos, const glm::vec2& size,
                          const glm::vec4& color) = 0;

    // Textured quad, modulated by `tint`. `uvMin`/`uvMax` select a sub-rect of
    // the texture (top-left origin to match the screen-space convention).
    virtual void DrawTexturedQuad(const glm::vec2& pos, const glm::vec2& size,
                                  const Texture& texture,
                                  const glm::vec4& tint  = glm::vec4(1.0f),
                                  const glm::vec2& uvMin = glm::vec2(0.0f),
                                  const glm::vec2& uvMax = glm::vec2(1.0f)) = 0;

    // Draw `text` with its top-left at `pos`, scaled by `scale` (1.0 = the
    // font's baked pixel height). Honors '\n'. Returns the advance width in px
    // of the last line.
    virtual float DrawText(const Font& font, const std::string& text,
                           const glm::vec2& pos, const glm::vec4& color,
                           float scale = 1.0f) = 0;

    // Top-left-origin orthographic projection for a width x height pixel canvas.
    static glm::mat4 OrthoProjection(float width, float height);

    static std::unique_ptr<Renderer2D> Create();
};

} // namespace Diamond
