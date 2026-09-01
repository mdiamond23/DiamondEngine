#include "Renderer/TextureData.h"
#include "Renderer/RendererAPI.h"
#include "Platform/OpenGL/Resources/OpenGLTexture.h"
#include "Assets/ImageLoader.h"
#include "Assets/DDSLoader.h"
#include <spdlog/spdlog.h>

#include <vector>

#ifdef DIAMOND_ENABLE_VULKAN
#include "Platform/Vulkan/Resources/VulkanTexture2D.h"
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
        // Cooked BCn fast path: a fresh Assets/Cache .dds skips the stb decode
        // and runtime mip generation and stays compressed in VRAM (see
        // Docs/asset-pipeline-design.md §4). Cooked files are baked unflipped,
        // so flipped callers always take the source path. Any miss — no cooked
        // file, stale (source newer), parse failure — falls through unchanged.
        if (!flipVertically) {
            DDSData dds = DDSLoader::LoadCookedFor(path);
            if (dds.IsValid()) {
                spdlog::info("[Texture] cooked load '{}' ({}, {} mips)", path, RHIFormatName(dds.Format), dds.MipCount);
                return std::make_shared<VulkanTexture2D>(
                    s_ResourceDevice, dds, RHIFilter::Linear, path, flipVertically);
            }
        }

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

std::shared_ptr<Texture> Texture::CreateChannel(const std::string& path, uint32_t channel)
{
    if (channel > 3) return nullptr;

    DDSData dds = DDSLoader::LoadCookedChannelFor(path, channel);
    if (dds.IsValid()) {
        spdlog::info("[Texture] cooked channel load '{}' channel {} ({}, {} mips)",
                     path, channel, RHIFormatName(dds.Format), dds.MipCount);
#ifdef DIAMOND_ENABLE_VULKAN
        if (s_ResourceDevice)
            return std::make_shared<VulkanTexture2D>(s_ResourceDevice, dds);
#endif
        if (RendererAPI::GetAPI() == RendererAPI::API::OpenGL)
            return std::make_shared<OpenGLTexture>(dds);
        return nullptr;
    }

    ImageData img = ImageLoader::Load(path, /*flipVertically=*/false);
    if (img.Pixels.empty() || img.Channels <= static_cast<int>(channel)) return nullptr;

    const size_t count = static_cast<size_t>(img.Width) * static_cast<size_t>(img.Height);
    std::vector<uint8_t> pixels(count);
    for (size_t i = 0; i < count; ++i)
        pixels[i] = img.Pixels[i * static_cast<size_t>(img.Channels) + channel];
    return CreateFromPixels(pixels.data(), img.Width, img.Height, 1);
}

} // namespace Diamond
