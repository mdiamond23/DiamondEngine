#include "Renderer/TextureData.h"
#include "Renderer/RendererAPI.h"
#include "Platform/OpenGL/Resources/OpenGLTexture.h"

#ifdef DIAMOND_ENABLE_VULKAN
#include "Platform/Vulkan/Resources/VulkanTexture2D.h"
#include "Assets/ImageLoader.h"
#include <spdlog/spdlog.h>
#endif

namespace Diamond {

namespace {
// The device Vulkan-backed textures upload through. When non-null the factory
// prefers a VulkanTexture2D over the GL RendererAPI dispatch. Left null in a GL
// run, so OpenGL behavior is unchanged.
RHIDevice* s_ResourceDevice = nullptr;
} // namespace

void Texture::SetResourceDevice(RHIDevice* device)
{
#ifdef DIAMOND_ENABLE_VULKAN
    s_ResourceDevice = device;
#else
    (void)device;
#endif
}

std::shared_ptr<Texture> Texture::Create(const std::string& path, bool flipVertically)
{
#ifdef DIAMOND_ENABLE_VULKAN
    if (s_ResourceDevice) {
        // Same decode + flip convention as the GL path (texel (0,0) maps to v=0 in
        // both APIs, so the pixels must be identical for mesh UVs to match), with
        // mips generated at upload like GL's glGenerateMipmap.
        ImageData img = ImageLoader::Load(path, flipVertically);
        if (img.Pixels.empty()) {
            spdlog::error("[Texture] no pixel data for '{}'", path);
            return nullptr;
        }
        return std::make_shared<VulkanTexture2D>(
            s_ResourceDevice, img.Pixels.data(), img.Width, img.Height, img.Channels,
            RHIFilter::Linear, /*generateMips*/ true, path, flipVertically);
    }
#endif
    switch (RendererAPI::GetAPI()) {
        case RendererAPI::API::OpenGL: return std::make_shared<OpenGLTexture>(path, flipVertically);
        default:                       return nullptr;
    }
}

std::shared_ptr<Texture> Texture::CreateFromPixels(const uint8_t* pixels,
                                                   int width, int height, int channels)
{
#ifdef DIAMOND_ENABLE_VULKAN
    if (s_ResourceDevice)
        return std::make_shared<VulkanTexture2D>(s_ResourceDevice, pixels, width, height, channels);
#endif
    switch (RendererAPI::GetAPI()) {
        case RendererAPI::API::OpenGL: return std::make_shared<OpenGLTexture>(pixels, width, height, channels);
        default:                       return nullptr;
    }
}

} // namespace Diamond
