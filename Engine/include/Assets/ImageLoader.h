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
    static ImageData      Load     (const std::string& path, bool flipVertically = true);
    static FloatImageData LoadFloat(const std::string& path, bool flipVertically = true);
};

} // namespace Diamond
