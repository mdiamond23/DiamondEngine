#include "Platform/Vulkan/Resources/VulkanTexture2D.h"
#include "Renderer/RHI/RHIDevice.h"

#include <vector>

namespace Diamond {

VulkanTexture2D::VulkanTexture2D(RHIDevice* device, const uint8_t* pixels,
                                 int width, int height, int channels, RHIFilter filter)
    : m_Width(static_cast<uint32_t>(width)),
      m_Height(static_cast<uint32_t>(height))
{
    // Expand whatever channel count we were handed to RGBA8 — the RHI upload path
    // is RGBA8-only. R8 replicates into RGB (alpha 255) so the text coverage path
    // (samples .r) still works; RGB8 gets an opaque alpha.
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

    RHITextureDesc desc;
    desc.width       = m_Width;
    desc.height      = m_Height;
    desc.format      = RHIFormat::RGBA8;
    desc.usage       = RHITextureUsage::Sampled;
    desc.initialData = rgba.data();
    desc.filter      = filter;
    m_Texture = device->CreateTexture(desc);
}

VulkanTexture2D::~VulkanTexture2D() = default;

void VulkanTexture2D::Bind(uint32_t) const {}

} // namespace Diamond
