#pragma once

#include <memory>
#include <cstdint>
#include <glm/glm.hpp>

namespace Diamond {

// Low-level renderer abstraction. Swap the API enum + add a new platform subclass to support Vulkan.
class RendererAPI {
public:
    enum class API { None = 0, OpenGL, Vulkan };

    virtual ~RendererAPI() = default;

    virtual void Init() = 0;
    virtual void SetClearColor(const glm::vec4& color) = 0;
    virtual void Clear() = 0;
    virtual void SetViewport(uint32_t x, uint32_t y, uint32_t width, uint32_t height) = 0;

    static API                         GetAPI()  { return s_API; }
    static std::unique_ptr<RendererAPI> Create();

private:
    static API s_API;
};

} // namespace Diamond