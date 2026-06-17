#include "Renderer/TextureData.h"
#include "Renderer/RendererAPI.h"
#include "Platform/OpenGL/Resources/OpenGLTexture.h"

namespace Diamond {

std::shared_ptr<Texture> Texture::Create(const std::string& path, bool flipVertically)
{
    switch (RendererAPI::GetAPI()) {
        case RendererAPI::API::OpenGL: return std::make_shared<OpenGLTexture>(path, flipVertically);
        default:                       return nullptr;
    }
}

std::shared_ptr<Texture> Texture::CreateFromPixels(const uint8_t* pixels,
                                                   int width, int height, int channels)
{
    switch (RendererAPI::GetAPI()) {
        case RendererAPI::API::OpenGL: return std::make_shared<OpenGLTexture>(pixels, width, height, channels);
        default:                       return nullptr;
    }
}

} // namespace Diamond