#pragma once

#include "Renderer/TextureData.h"
#include "Renderer/RHI/RHIEnums.h"

#include <cstdint>
#include <memory>
#include <string>

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
    // 'sourcePath' is empty for pixel-only construction (font atlases, GLB-embedded
    // textures) -- Reload() is a no-op without it, same contract as OpenGLTexture.
    VulkanTexture2D(RHIDevice* device, const uint8_t* pixels,
                    int width, int height, int channels,
                    RHIFilter filter = RHIFilter::Linear,
                    bool generateMips = false,
                    std::string sourcePath = {},
                    bool flipVertically = false);
    ~VulkanTexture2D() override;

    void     Bind(uint32_t slot = 0) const override;   // no-op: Vulkan binds via sets
    uint32_t GetWidth()  const override { return m_Width; }
    uint32_t GetHeight() const override { return m_Height; }

    // Re-decode m_Path and upload into a NEW RHITexture, then replace m_Texture
    // (destroying the old one). The caller MUST have already WaitIdle()'d the
    // device before calling this — replacing m_Texture frees the old VkImage/
    // VkImageView, which a prior frame's command buffer may still be sampling
    // from if the device isn't idle. See EditorBackend::WaitIdle() and
    // Docs/asset-pipeline-design.md, section 2. Returns false (texture
    // unchanged) if there's no source path or the decode fails.
    bool Reload() override;

    // The underlying RHI texture, for building a descriptor set to sample it.
    RHITexture* Rhi() const { return m_Texture.get(); }

private:
    RHIDevice*   m_Device        = nullptr;
    uint32_t     m_Width         = 0;
    uint32_t     m_Height        = 0;
    std::unique_ptr<RHITexture> m_Texture;

    // Reload() inputs -- empty m_Path means not file-backed (see sourcePath above).
    std::string m_Path;
    bool        m_FlipVertically = false;
    RHIFilter   m_Filter         = RHIFilter::Linear;
    bool        m_GenerateMips   = false;
};

} // namespace Diamond
