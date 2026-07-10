#pragma once

#include <cstdint>
#include <string>
#include <memory>

namespace Diamond {

class RHIDevice;

// Abstract GPU texture. Backend-specific instances are created via Texture::Create().
class Texture {
public:
    virtual ~Texture() = default;
    virtual void     Bind(uint32_t slot = 0) const = 0;
    virtual uint32_t GetWidth()  const = 0;
    virtual uint32_t GetHeight() const = 0;

    // Re-decode from the source file and upload in place (new GPU image
    // internally, old one freed) so every shared_ptr<Texture> handed out stays
    // valid across a reload. Returns false and leaves the texture unchanged if
    // there's no source file to reload from (CreateFromPixels) or the decode
    // fails (e.g. file mid-write) — callers should keep the old texture and
    // retry later. Default: not reloadable.
    virtual bool Reload() { return false; }

    // Load from file (flips vertically by default to match OpenGL UV convention).
    static std::shared_ptr<Texture> Create(const std::string& path, bool flipVertically = true);

    // Create from already-decoded pixels (channels: 1=R8, 3=RGB8, 4=RGBA8). Used
    // for textures decoded out of a GLB's embedded buffer and for font atlases.
    static std::shared_ptr<Texture> CreateFromPixels(const uint8_t* pixels,
                                                     int width, int height, int channels);

    // Register the RHI device the Vulkan backend uploads through. When set, the
    // factory above produces Vulkan-backed textures instead of dispatching on the
    // GL RendererAPI — this is how Font (which bakes its atlas through
    // CreateFromPixels) gets a Vulkan atlas without threading a device to it. Pass
    // nullptr before the device is destroyed. No-op in a build without Vulkan.
    static void SetResourceDevice(RHIDevice* device);
};

} // namespace Diamond