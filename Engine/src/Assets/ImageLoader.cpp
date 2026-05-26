#define STB_IMAGE_IMPLEMENTATION
#include <stb_image.h>

#include "Assets/ImageLoader.h"
#include <spdlog/spdlog.h>

namespace Diamond {

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
