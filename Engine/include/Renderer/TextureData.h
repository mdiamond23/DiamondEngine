#pragma once

#include <cstdint>
#include <string>
#include <memory>

namespace Diamond {

// Abstract GPU texture. Backend-specific instances are created via Texture::Create().
class Texture {
public:
    virtual ~Texture() = default;
    virtual void     Bind(uint32_t slot = 0) const = 0;
    virtual uint32_t GetWidth()  const = 0;
    virtual uint32_t GetHeight() const = 0;

    // Load from file (flips vertically by default to match OpenGL UV convention).
    static std::shared_ptr<Texture> Create(const std::string& path, bool flipVertically = true);
};

} // namespace Diamond