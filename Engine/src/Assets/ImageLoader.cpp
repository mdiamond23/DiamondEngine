#define STB_IMAGE_IMPLEMENTATION
#include <stb_image.h>

#include "Assets/ImageLoader.h"
#include <spdlog/spdlog.h>

#include <algorithm>
#include <vector>

namespace Diamond {

namespace {

// Default cap. 2048 keeps a 4K source at a quarter the VRAM while still
// out-resolving most on-screen surfaces at 4K — see the header for why this
// exists at all.
uint32_t g_MaxDimension = 2048;

// One 2x2 box halving step. Averaging in the raw 8-bit encoding is not strictly
// gamma-correct for color maps, but it matches what the GPU's own mip generation
// does at upload (vkCmdBlitImage with a linear filter), so the downscaled chain
// stays consistent with the levels built on top of it.
void HalveInPlace(std::vector<uint8_t>& pixels, int& w, int& h, int channels)
{
    const int nw = std::max(1, w / 2);
    const int nh = std::max(1, h / 2);

    std::vector<uint8_t> dst(static_cast<size_t>(nw) * nh * channels);
    for (int y = 0; y < nh; ++y) {
        // Clamp so an odd source dimension doesn't read past the last row/column.
        const int y0 = std::min(y * 2,     h - 1);
        const int y1 = std::min(y * 2 + 1, h - 1);
        for (int x = 0; x < nw; ++x) {
            const int x0 = std::min(x * 2,     w - 1);
            const int x1 = std::min(x * 2 + 1, w - 1);
            for (int c = 0; c < channels; ++c) {
                const uint32_t sum =
                    pixels[(static_cast<size_t>(y0) * w + x0) * channels + c] +
                    pixels[(static_cast<size_t>(y0) * w + x1) * channels + c] +
                    pixels[(static_cast<size_t>(y1) * w + x0) * channels + c] +
                    pixels[(static_cast<size_t>(y1) * w + x1) * channels + c];
                dst[(static_cast<size_t>(y) * nw + x) * channels + c] =
                    static_cast<uint8_t>((sum + 2) / 4);
            }
        }
    }

    pixels.swap(dst);
    w = nw;
    h = nh;
}

// Halve until the longest edge fits the cap. Successive halving (rather than one
// arbitrary resample) keeps every step a clean 2x2 box and preserves
// power-of-two sizes, which is what the mip chain wants anyway.
void ClampToMaxDimension(ImageData& img, const char* what)
{
    const uint32_t cap = g_MaxDimension;
    if (cap == 0 || img.Pixels.empty()) return;
    if (img.Width <= (int)cap && img.Height <= (int)cap) return;

    const int srcW = img.Width, srcH = img.Height;
    while ((img.Width > (int)cap || img.Height > (int)cap) &&
           (img.Width > 1 || img.Height > 1)) {
        HalveInPlace(img.Pixels, img.Width, img.Height, img.Channels);
    }
    spdlog::info("[ImageLoader] downscaled {} {}x{} -> {}x{} (cap {})",
                 what, srcW, srcH, img.Width, img.Height, cap);
}

} // namespace

void     ImageLoader::SetMaxDimension(uint32_t maxDim) { g_MaxDimension = maxDim; }
uint32_t ImageLoader::MaxDimension()                   { return g_MaxDimension; }

ImageData ImageLoader::Load(const std::string& path, bool flipVertically)
{
    stbi_set_flip_vertically_on_load(flipVertically ? 1 : 0);

    ImageData result;
    uint8_t* data = stbi_load(path.c_str(),
                               &result.Width, &result.Height, &result.Channels, 0);
    if (!data) {
        spdlog::error("ImageLoader: failed to load '{}'", path);
        return result;
    }

    size_t byteCount = static_cast<size_t>(result.Width) *
                       static_cast<size_t>(result.Height) *
                       static_cast<size_t>(result.Channels);
    result.Pixels.assign(data, data + byteCount);
    stbi_image_free(data);
    ClampToMaxDimension(result, path.c_str());
    return result;
}


ImageData ImageLoader::LoadFromMemory(const uint8_t* bytes, size_t size, bool flipVertically)
{
    stbi_set_flip_vertically_on_load(flipVertically ? 1 : 0);

    ImageData result;
    uint8_t* data = stbi_load_from_memory(bytes, (int)size,
                                          &result.Width, &result.Height, &result.Channels, 0);
    if (!data) {
        spdlog::error("ImageLoader: failed to decode in-memory image ({} bytes)", size);
        return result;
    }

    size_t byteCount = static_cast<size_t>(result.Width) *
                       static_cast<size_t>(result.Height) *
                       static_cast<size_t>(result.Channels);
    result.Pixels.assign(data, data + byteCount);
    stbi_image_free(data);
    ClampToMaxDimension(result, "embedded image");
    return result;
}

FloatImageData ImageLoader::LoadFloat(const std::string& path, bool flipVertically)
{
    stbi_set_flip_vertically_on_load(flipVertically ? 1 : 0);

    FloatImageData result;
    float* data = stbi_loadf(path.c_str(),
                             &result.Width, &result.Height, &result.Channels, 0);
    if (!data) {
        spdlog::error("ImageLoader: failed to load float image '{}'", path);
        return result;
    }

    size_t count = static_cast<size_t>(result.Width) *
                   static_cast<size_t>(result.Height) *
                   static_cast<size_t>(result.Channels);
    result.Pixels.assign(data, data + count);
    stbi_image_free(data);
    return result;
}

} // namespace Diamond
