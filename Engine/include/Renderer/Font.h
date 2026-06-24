#pragma once

#include <glm/glm.hpp>
#include <cstdint>
#include <memory>
#include <string>
#include <unordered_map>

namespace Diamond {

class Texture;

// One baked glyph, all measurements in pixels at the font's baked pixel height.
struct Glyph {
    glm::vec2 uvMin   {0.0f};   // top-left UV in the atlas
    glm::vec2 uvMax   {0.0f};   // bottom-right UV in the atlas
    glm::vec2 size    {0.0f};   // quad size in px
    glm::vec2 offset  {0.0f};   // pen-baseline -> quad top-left, in px (y negative = above baseline)
    float     advance {0.0f};   // px to advance the pen after this glyph
};

// A bitmap font baked from a TTF/OTF into a single-channel (R8) atlas texture.
//
// Backend-agnostic: glyph rasterization is CPU-side (stb_truetype) and the atlas
// is uploaded through the abstract Texture interface, so the same Font works
// unchanged on any renderer backend (OpenGL today, Vulkan later).
class Font {
public:
    // Bake the printable ASCII range (32..126) of a TTF/OTF at the given pixel
    // height. Returns nullptr on failure (missing file, bad font, atlas overflow).
    static std::shared_ptr<Font> Create(const std::string& ttfPath, float pixelHeight);

    const Glyph* GetGlyph(uint32_t codepoint) const;
    const std::shared_ptr<Texture>& Atlas() const { return m_Atlas; }

    float PixelHeight() const { return m_PixelHeight; }
    float Ascent()      const { return m_Ascent; }      // px above baseline (positive)
    float Descent()     const { return m_Descent; }     // px below baseline (negative)
    float LineHeight()  const { return m_LineHeight; }   // px, baseline-to-baseline

    // Bounding size in px of `text` at scale 1.0 (accounts for '\n').
    glm::vec2 Measure(const std::string& text) const;

private:
    Font() = default;

    std::unordered_map<uint32_t, Glyph> m_Glyphs;
    std::shared_ptr<Texture>            m_Atlas;
    float m_PixelHeight = 0.0f;
    float m_Ascent      = 0.0f;
    float m_Descent     = 0.0f;
    float m_LineHeight  = 0.0f;
};

} // namespace Diamond
