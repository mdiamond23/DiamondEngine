#pragma once

#include "Renderer/Renderer2D.h"

#include <cstdint>
#include <glm/glm.hpp>
#include <vector>

namespace Diamond {

// OpenGL 2D batcher. Accumulates quads into a CPU vertex buffer and flushes one
// draw call per contiguous run that shares a texture (the bound texture is part
// of the batch state). Solid quads use a 1x1 white texture; text uses the font
// atlas with a per-vertex "sample as coverage" mode, so quads, sprites, and text
// share a single shader and can be interleaved within a frame.
class OpenGLRenderer2D : public Renderer2D {
public:
    OpenGLRenderer2D();
    ~OpenGLRenderer2D() override;

    void Begin(const glm::mat4& projection) override;
    void End() override;

    void  DrawQuad(const glm::vec2& pos, const glm::vec2& size,
                   const glm::vec4& color) override;
    void  DrawTexturedQuad(const glm::vec2& pos, const glm::vec2& size,
                           const Texture& texture, const glm::vec4& tint,
                           const glm::vec2& uvMin, const glm::vec2& uvMax) override;
    float DrawText(const Font& font, const std::string& text,
                   const glm::vec2& pos, const glm::vec4& color, float scale) override;

private:
    struct Vertex {
        glm::vec2 pos;
        glm::vec2 uv;
        glm::vec4 color;
        float     mode;   // 0 = modulate RGBA, 1 = R channel as coverage (text)
    };

    void Flush();                      // upload + draw the current batch, then clear it
    void SetTexture(uint32_t texID);   // switch bound texture, flushing first if it changed
    void PushQuad(const glm::vec2& pos, const glm::vec2& size,
                  const glm::vec2& uvMin, const glm::vec2& uvMax,
                  const glm::vec4& color, float mode);

    uint32_t  m_VAO = 0, m_VBO = 0, m_Program = 0;
    uint32_t  m_WhiteTex   = 0;   // 1x1 white — solid-color quads
    uint32_t  m_CurrentTex = 0;   // texture bound for the in-progress batch
    glm::mat4 m_Projection{1.0f};
    std::vector<Vertex> m_Verts;

    // Saved GL state restored in End().
    bool    m_InFrame      = false;
    bool    m_SavedDepth   = false;
    bool    m_SavedBlend   = false;
    int32_t m_SavedBlendSrc = 0, m_SavedBlendDst = 0;
};

} // namespace Diamond
