#include "Renderer/TextureData.h"
#include "Renderer/RendererAPI.h"
#include "Platform/OpenGL/Resources/OpenGLTexture.h"

#ifdef DIAMOND_ENABLE_VULKAN
#include "Platform/Vulkan/Resources/VulkanTexture2D.h"
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
        // File decode for the Vulkan backend isn't wired yet (CreateFromPixels is
        // all the font/2D path needs). Returning null keeps us off the GL path,
        // which would issue GL calls with no context.
        spdlog::warn("[Texture] Create(path) not implemented for the Vulkan backend: '{}'", path);
        return nullptr;
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
