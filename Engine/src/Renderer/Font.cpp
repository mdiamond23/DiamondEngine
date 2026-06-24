#include "Renderer/Font.h"
#include "Renderer/TextureData.h"

#include <spdlog/spdlog.h>
#include <algorithm>
#include <fstream>
#include <vector>

// stb_truetype is header-only; this translation unit owns the implementation.
#define STB_TRUETYPE_IMPLEMENTATION
#include <stb_truetype.h>

namespace Diamond {

namespace {
constexpr uint32_t kFirstChar = 32;    // space
constexpr uint32_t kLastChar  = 126;   // '~'
// Atlas width is a multiple of 4 so the single-channel (R8) upload is safe under
// the default GL_UNPACK_ALIGNMENT of 4 — no per-row padding surprises.
constexpr int      kAtlasW    = 512;
constexpr int      kAtlasH    = 512;
} // namespace

std::shared_ptr<Font> Font::Create(const std::string& ttfPath, float pixelHeight)
{
    // --- Read the whole font file into memory ---
    std::ifstream file(ttfPath, std::ios::binary | std::ios::ate);
    if (!file) {
        spdlog::error("Font: failed to open '{}'", ttfPath);
        return nullptr;
    }
    std::streamsize size = file.tellg();
    file.seekg(0);
    std::vector<unsigned char> ttf(static_cast<size_t>(size));
    if (size <= 0 || !file.read(reinterpret_cast<char*>(ttf.data()), size)) {
        spdlog::error("Font: failed to read '{}'", ttfPath);
        return nullptr;
    }

    // --- Pack the glyph range into a single R8 atlas ---
    const int                     count = static_cast<int>(kLastChar - kFirstChar + 1);
    std::vector<unsigned char>    bitmap(static_cast<size_t>(kAtlasW) * kAtlasH, 0);
    std::vector<stbtt_packedchar> packed(static_cast<size_t>(count));

    stbtt_pack_context pc;
    if (!stbtt_PackBegin(&pc, bitmap.data(), kAtlasW, kAtlasH, /*stride*/ 0, /*padding*/ 1, nullptr)) {
        spdlog::error("Font: stbtt_PackBegin failed for '{}'", ttfPath);
        return nullptr;
    }
    stbtt_PackSetOversampling(&pc, 1, 1);
    if (!stbtt_PackFontRange(&pc, ttf.data(), 0, pixelHeight,
                             static_cast<int>(kFirstChar), count, packed.data())) {
        spdlog::error("Font: glyphs did not fit the atlas for '{}' at {}px", ttfPath, pixelHeight);
        stbtt_PackEnd(&pc);
        return nullptr;
    }
    stbtt_PackEnd(&pc);

    auto font = std::shared_ptr<Font>(new Font());
    font->m_PixelHeight = pixelHeight;

    // --- Vertical metrics, scaled to the baked pixel height ---
    stbtt_fontinfo info;
    if (!stbtt_InitFont(&info, ttf.data(), stbtt_GetFontOffsetForIndex(ttf.data(), 0))) {
        spdlog::error("Font: stbtt_InitFont failed for '{}'", ttfPath);
        return nullptr;
    }
    const float scale = stbtt_ScaleForPixelHeight(&info, pixelHeight);
    int ascent = 0, descent = 0, lineGap = 0;
    stbtt_GetFontVMetrics(&info, &ascent, &descent, &lineGap);
    font->m_Ascent     = ascent  * scale;          // positive
    font->m_Descent    = descent * scale;          // negative
    font->m_LineHeight = (ascent - descent + lineGap) * scale;

    // --- Convert packed glyphs into backend-neutral Glyphs ---
    const float invW = 1.0f / kAtlasW;
    const float invH = 1.0f / kAtlasH;
    for (int i = 0; i < count; ++i) {
        const stbtt_packedchar& p = packed[static_cast<size_t>(i)];
        Glyph g;
        g.uvMin   = { p.x0 * invW, p.y0 * invH };
        g.uvMax   = { p.x1 * invW, p.y1 * invH };
        g.size    = { static_cast<float>(p.x1 - p.x0), static_cast<float>(p.y1 - p.y0) };
        g.offset  = { p.xoff, p.yoff };
        g.advance = p.xadvance;
        font->m_Glyphs.emplace(kFirstChar + static_cast<uint32_t>(i), g);
    }

    // --- Upload the atlas through the abstract Texture interface (R8) ---
    font->m_Atlas = Texture::CreateFromPixels(bitmap.data(), kAtlasW, kAtlasH, 1);
    if (!font->m_Atlas) {
        spdlog::error("Font: atlas upload failed for '{}'", ttfPath);
        return nullptr;
    }

    spdlog::info("Font: baked '{}' at {}px ({} glyphs, {}x{} atlas)",
                 ttfPath, pixelHeight, count, kAtlasW, kAtlasH);
    return font;
}

const Glyph* Font::GetGlyph(uint32_t codepoint) const
{
    auto it = m_Glyphs.find(codepoint);
    return it == m_Glyphs.end() ? nullptr : &it->second;
}

glm::vec2 Font::Measure(const std::string& text) const
{
    float maxWidth = 0.0f, lineWidth = 0.0f;
    int   lines = 1;
    for (char c : text) {
        if (c == '\n') {
            maxWidth  = std::max(maxWidth, lineWidth);
            lineWidth = 0.0f;
            ++lines;
            continue;
        }
        if (const Glyph* g = GetGlyph(static_cast<uint32_t>(static_cast<unsigned char>(c))))
            lineWidth += g->advance;
    }
    maxWidth = std::max(maxWidth, lineWidth);
    return { maxWidth, lines * m_LineHeight };
}

} // namespace Diamond
