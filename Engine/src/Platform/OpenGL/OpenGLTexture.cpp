#include "OpenGLTexture.h"

#include <glad/gl.h>
#include "Assets/ImageLoader.h"
#include <spdlog/spdlog.h>

namespace Diamond {

OpenGLTexture::OpenGLTexture(const std::string& path, bool flipVertically)
{
    ImageData img = ImageLoader::Load(path, flipVertically);
    if (img.Pixels.empty()) {
        spdlog::error("OpenGLTexture: no pixel data for '{}'", path);
        return;
    }

    m_Width  = static_cast<uint32_t>(img.Width);
    m_Height = static_cast<uint32_t>(img.Height);

    GLenum internalFormat = GL_RGB8;
    GLenum dataFormat     = GL_RGB;
    if (img.Channels == 1) { internalFormat = GL_R8;    dataFormat = GL_RED;  }
    if (img.Channels == 4) { internalFormat = GL_RGBA8; dataFormat = GL_RGBA; }

    glGenTextures(1, &m_RendererID);
    glBindTexture(GL_TEXTURE_2D, m_RendererID);

    glTexImage2D(GL_TEXTURE_2D, 0, internalFormat,
                 img.Width, img.Height, 0,
                 dataFormat, GL_UNSIGNED_BYTE, img.Pixels.data());
    glGenerateMipmap(GL_TEXTURE_2D);

    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S,     GL_REPEAT);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T,     GL_REPEAT);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);

    glBindTexture(GL_TEXTURE_2D, 0);
}

OpenGLTexture::~OpenGLTexture()
{
    glDeleteTextures(1, &m_RendererID);
}

void OpenGLTexture::Bind(uint32_t slot) const
{
    glActiveTexture(GL_TEXTURE0 + slot);
    glBindTexture(GL_TEXTURE_2D, m_RendererID);
}

} // namespace Diamond