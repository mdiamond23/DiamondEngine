#pragma once

#include "Renderer/TextureData.h"
#include "Renderer/RHI/RHIEnums.h"

#include <cstdint>
#include <memory>

namespace Diamond {

class RHIDevice;
class RHITexture;

// A Vulkan-backed implementation of the engine's abstract Texture: it wraps an
// RHITexture so Renderer2D (and, later, UI/material sampling) can bind an uploaded
// image through the RHI's descriptor-set path.
//
// Constructed directly with a device — the Texture::Create factory has no device
// to hand out (that factory wiring is a later chunk), so callers under Vulkan make
// one of these explicitly.
//
// Single-channel (R8, e.g. a font atlas) and RGB8 source pixels are expanded to
// RGBA8 on the CPU so the RHI upload path stays RGBA8-only. For R8 the value is
// replicated into RGB with alpha 255, so the "sample .r as coverage" text path
// reads it unchanged.
class VulkanTexture2D : public Texture {
public:
    // channels: 1 = R8, 3 = RGB8, 4 = RGBA8. 'filter' is Nearest for pixel-exact
    // sprites/atlases, Linear otherwise. 'generateMips' builds the full mip chain
    // at upload — on for file-loaded material textures (matching the GL texture
    // path's glGenerateMipmap), off for the font/2D atlases baked at native size.
    VulkanTexture2D(RHIDevice* device, const uint8_t* pixels,
                    int width, int height, int channels,
                    RHIFilter filter = RHIFilter::Linear,
                    bool generateMips = false);
    ~VulkanTexture2D() override;

    void     Bind(uint32_t slot = 0) const override;   // no-op: Vulkan binds via sets
    uint32_t GetWidth()  const override { return m_Width; }
    uint32_t GetHeight() const override { return m_Height; }

    // The underlying RHI texture, for building a descriptor set to sample it.
    RHITexture* Rhi() const { return m_Texture.get(); }

private:
    uint32_t m_Width  = 0;
    uint32_t m_Height = 0;
    std::unique_ptr<RHITexture> m_Texture;
};

} // namespace Diamond
