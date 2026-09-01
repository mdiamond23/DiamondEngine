#pragma once

#include "Renderer/TextureData.h"

namespace Diamond {

struct DDSData;

class OpenGLTexture : public Texture {
public:
    OpenGLTexture(const std::string& path, bool flipVertically);
    OpenGLTexture(const std::string& path, bool flipVertically, bool isHDR);
    OpenGLTexture(const uint8_t* pixels, int width, int height, int channels);
    OpenGLTexture(const DDSData& dds);
    ~OpenGLTexture() override;

    void     Bind(uint32_t slot = 0) const override;
    uint32_t GetWidth()  const override { return m_Width; }
    uint32_t GetHeight() const override { return m_Height; }
    bool     Reload()    override;

    // Native GL texture name. Backend-only — not part of the abstract Texture
    // interface; used by other OpenGL code (e.g. the 2D batcher) that needs to
    // group draws by texture handle.
    uint32_t GetRendererID() const { return m_RendererID; }

private:
    uint32_t m_RendererID = 0;
    uint32_t m_Width      = 0;
    uint32_t m_Height     = 0;
    uint64_t m_ByteSize   = 0;   // VRAM estimate — see Docs/profiler-panel-design.md

    // Source info for Reload() — empty m_Path means CreateFromPixels (no file
    // to watch/reload from).
    std::string m_Path;
    bool        m_FlipVertically = false;
    bool        m_IsHDR          = false;
};

} // namespace Diamond
