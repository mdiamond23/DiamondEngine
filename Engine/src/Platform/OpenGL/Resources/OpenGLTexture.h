#pragma once

#include "Renderer/TextureData.h"

namespace Diamond {

class OpenGLTexture : public Texture {
public:
    OpenGLTexture(const std::string& path, bool flipVertically);
    OpenGLTexture(const std::string& path, bool flipVertically, bool isHDR);
    OpenGLTexture(const uint8_t* pixels, int width, int height, int channels);
    ~OpenGLTexture() override;

    void     Bind(uint32_t slot = 0) const override;
    uint32_t GetWidth()  const override { return m_Width; }
    uint32_t GetHeight() const override { return m_Height; }

private:
    uint32_t m_RendererID = 0;
    uint32_t m_Width      = 0;
    uint32_t m_Height     = 0;
};

} // namespace Diamond
