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

// Loads an image from disk via stb_image. Returns empty Pixels on failure.
class ImageLoader {
public:
    static ImageData Load(const std::string& path, bool flipVertically = true);
};

} // namespace Diamond
