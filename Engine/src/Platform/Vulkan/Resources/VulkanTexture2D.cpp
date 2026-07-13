#include "Platform/Vulkan/Resources/VulkanTexture2D.h"
#include "Renderer/RHI/RHIDevice.h"
#include "Assets/ImageLoader.h"
#include "Assets/DDSLoader.h"

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

// A cooked DDS uploads exactly as parsed: block-compressed payload, pre-baked
// mip chain (mipCount routes the RHI around its blit-downsample path).
RHITextureDesc DescFromDDS(const DDSData& dds, RHIFilter filter)
{
    RHITextureDesc desc;
    desc.width       = dds.Width;
    desc.height      = dds.Height;
    desc.format      = dds.Format;
    desc.usage       = RHITextureUsage::Sampled;
    desc.initialData = dds.Payload.data();
    desc.filter      = filter;
    desc.mipCount    = dds.MipCount;
    return desc;
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

VulkanTexture2D::VulkanTexture2D(RHIDevice* device, const DDSData& dds, RHIFilter filter,
                                 std::string sourcePath, bool flipVertically)
    : m_Device(device),
      m_Width(dds.Width),
      m_Height(dds.Height),
      m_Path(std::move(sourcePath)),
      m_FlipVertically(flipVertically),
      m_Filter(filter),
      m_GenerateMips(true)   // Reload()'s uncooked fallback is a material texture — wants mips
{
    m_Texture = device->CreateTexture(DescFromDDS(dds, filter));
}

VulkanTexture2D::~VulkanTexture2D() = default;

void VulkanTexture2D::Bind(uint32_t) const {}

bool VulkanTexture2D::Reload()
{
    if (m_Path.empty() || !m_Device) return false;   // no source file to reload from

    std::unique_ptr<RHITexture> newTexture;
    uint32_t newWidth = 0, newHeight = 0;

    // Same decision as Texture::Create: a fresh cooked DDS wins; a stale one
    // (source just edited) falls through to the source decode, so hot reload
    // serves the uncooked image until the next cook pass. Cooked files are
    // baked unflipped — flipped callers never take this path.
    if (!m_FlipVertically) {
        DDSData dds = DDSLoader::LoadCookedFor(m_Path);
        if (dds.IsValid()) {
            newTexture = m_Device->CreateTexture(DescFromDDS(dds, m_Filter));
            newWidth   = dds.Width;
            newHeight  = dds.Height;
        }
    }

    if (!newTexture) {
        ImageData img = ImageLoader::Load(m_Path, m_FlipVertically);
        if (img.Pixels.empty()) return false;   // decode failed (e.g. file mid-write) — keep old texture

        std::vector<uint8_t> rgba = ExpandToRGBA(img.Pixels.data(), img.Width, img.Height, img.Channels);

        RHITextureDesc desc;
        desc.width        = static_cast<uint32_t>(img.Width);
        desc.height       = static_cast<uint32_t>(img.Height);
        desc.format       = RHIFormat::RGBA8;
        desc.usage        = RHITextureUsage::Sampled;
        desc.initialData  = rgba.data();
        desc.filter       = m_Filter;
        desc.generateMips = m_GenerateMips;

        newTexture = m_Device->CreateTexture(desc);
        newWidth   = desc.width;
        newHeight  = desc.height;
    }
    if (!newTexture) return false;

    // Caller must have already WaitIdle()'d the device -- this destroys the
    // old RHITexture (VkImage/VkImageView/memory), which is unsafe while a
    // prior frame's command buffer might still be sampling it.
    m_Texture = std::move(newTexture);
    m_Width   = newWidth;
    m_Height  = newHeight;
    return true;
}

} // namespace Diamond
