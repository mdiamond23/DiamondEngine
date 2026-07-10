#include "Platform/Vulkan/Resources/VulkanTexture2D.h"
#include "Renderer/RHI/RHIDevice.h"
#include "Assets/ImageLoader.h"

#include <vector>

namespace Diamond {

namespace {

// Expand whatever channel count we were handed to RGBA8 — the RHI upload path
// is RGBA8-only. R8 replicates into RGB (alpha 255) so the text coverage path
// (samples .r) still works; RGB8 gets an opaque alpha.
std::vector<uint8_t> ExpandToRGBA(const uint8_t* pixels, int width, int height, int channels)
{
    const size_t count = static_cast<size_t>(width) * static_cast<size_t>(height);
    std::vector<uint8_t> rgba(count * 4, 255);
    for (size_t i = 0; i < count; ++i) {
        uint8_t r = 255, g = 255, b = 255, a = 255;
        if (channels == 1) {
            r = g = b = pixels[i];
        } else if (channels == 3) {
            r = pixels[i * 3 + 0]; g = pixels[i * 3 + 1]; b = pixels[i * 3 + 2];
        } else {   // 4
            r = pixels[i * 4 + 0]; g = pixels[i * 4 + 1];
            b = pixels[i * 4 + 2]; a = pixels[i * 4 + 3];
        }
        rgba[i * 4 + 0] = r; rgba[i * 4 + 1] = g;
        rgba[i * 4 + 2] = b; rgba[i * 4 + 3] = a;
    }
    return rgba;
}

} // namespace

VulkanTexture2D::VulkanTexture2D(RHIDevice* device, const uint8_t* pixels,
                                 int width, int height, int channels, RHIFilter filter,
                                 bool generateMips, std::string sourcePath, bool flipVertically)
    : m_Device(device),
      m_Width(static_cast<uint32_t>(width)),
      m_Height(static_cast<uint32_t>(height)),
      m_Path(std::move(sourcePath)),
      m_FlipVertically(flipVertically),
      m_Filter(filter),
      m_GenerateMips(generateMips)
{
    std::vector<uint8_t> rgba = ExpandToRGBA(pixels, width, height, channels);

    RHITextureDesc desc;
    desc.width        = m_Width;
    desc.height       = m_Height;
    desc.format       = RHIFormat::RGBA8;
    desc.usage        = RHITextureUsage::Sampled;
    desc.initialData  = rgba.data();
    desc.filter       = filter;
    desc.generateMips = generateMips;
    m_Texture = device->CreateTexture(desc);
}

VulkanTexture2D::~VulkanTexture2D() = default;

void VulkanTexture2D::Bind(uint32_t) const {}

bool VulkanTexture2D::Reload()
{
    if (m_Path.empty() || !m_Device) return false;   // no source file to reload from

    ImageData img = ImageLoader::Load(m_Path, m_FlipVertically);
    if (img.Pixels.empty()) return false;   // decode failed (e.g. file mid-write) — keep old texture

    std::vector<uint8_t> rgba = ExpandToRGBA(img.Pixels.data(), img.Width, img.Height, img.Channels);

    RHITextureDesc desc;
    desc.width        = static_cast<uint32_t>(img.Width);
    desc.height        = static_cast<uint32_t>(img.Height);
    desc.format        = RHIFormat::RGBA8;
    desc.usage         = RHITextureUsage::Sampled;
    desc.initialData   = rgba.data();
    desc.filter        = m_Filter;
    desc.generateMips  = m_GenerateMips;

    std::unique_ptr<RHITexture> newTexture = m_Device->CreateTexture(desc);
    if (!newTexture) return false;

    // Caller must have already WaitIdle()'d the device -- this destroys the
    // old RHITexture (VkImage/VkImageView/memory), which is unsafe while a
    // prior frame's command buffer might still be sampling it.
    m_Texture = std::move(newTexture);
    m_Width   = desc.width;
    m_Height  = desc.height;
    return true;
}

} // namespace Diamond
