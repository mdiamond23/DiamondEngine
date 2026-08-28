#pragma once

#include <string>
#include <vector>
#include <cstdint>

namespace Diamond {

struct ImageData {
    int                  Width    = 0;
    int                  Height   = 0;
    int                  Channels = 0;
    std::vector<uint8_t> Pixels;
};

struct FloatImageData {
    int                Width    = 0;
    int                Height   = 0;
    int                Channels = 0;
    std::vector<float> Pixels;
};

class ImageLoader {
public:
    // Longest edge an LDR texture may keep on load; anything larger is
    // box-downsampled by successive halving before it ever reaches the GPU.
    //
    // This is a VRAM guard, not a quality preference. An uncompressed 4096²
    // RGBA8 texture costs 67 MB, and ~89.5 MB once mips are generated at
    // upload — a scene like Sponza ships ~150 of them, which is ~13 GB and does
    // not fit in a 12 GB card. Once the working set exceeds VRAM the driver
    // evicts to system RAM and everything collapses: even a depth-only shadow
    // pass with an empty fragment shader ends up fetching its vertex buffers
    // over PCIe at ~25 GB/s instead of ~500 GB/s local.
    //
    // Halving the longest edge quarters the cost (89.5 MB -> 22.4 MB at 2048²),
    // the same saving BC7 gives — and unlike BC7 it needs no external cooker.
    // Cooked DDS files, when present, are loaded ahead of this path entirely
    // (see Texture::Create), so this only ever applies to raw PNG/JPG/TGA.
    // 0 disables the cap.
    static void     SetMaxDimension(uint32_t maxDim);
    static uint32_t MaxDimension();

    static ImageData      Load     (const std::string& path, bool flipVertically = true);
    static FloatImageData LoadFloat(const std::string& path, bool flipVertically = true);

    // Decodes an encoded image (PNG/JPEG/...) held in memory — used for textures
    // embedded inside a GLB's binary buffer.
    static ImageData      LoadFromMemory(const uint8_t* bytes, size_t size,
                                         bool flipVertically = true);
};

} // namespace Diamond
