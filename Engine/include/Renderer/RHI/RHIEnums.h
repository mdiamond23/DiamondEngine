#pragma once

#include <cstdint>

// Backend-neutral enums shared by every RHI interface. No backend (GL/Vulkan)
// type appears here; each backend maps these to its native constants. Kept
// deliberately small — formats/usages grow only as passes need them.
namespace Diamond {

enum class RHIBackend { OpenGL, Vulkan };

// Render-target / texture pixel formats.
enum class RHIFormat {
    Undefined = 0,
    RGBA8,
    BGRA8_SRGB,   // common swapchain format
    RGBA16F,
    R16F,
    Depth32F,
};

// Vertex attribute component layout.
enum class RHIVertexFormat {
    Float,
    Float2,
    Float3,
    Float4,
};

enum class RHIIndexType { U16, U32 };

enum class RHIPrimitiveTopology { TriangleList, TriangleStrip, LineList };

enum class RHICullMode { None, Back, Front };

// Resource kinds a descriptor binding can hold (grows with textures/storage).
enum class RHIResourceType { UniformBuffer };

// ── Bitmask enums ────────────────────────────────────────────────────────────
// Which buffer roles a buffer may serve.
enum class RHIBufferUsage : uint32_t {
    None    = 0,
    Vertex  = 1u << 0,
    Index   = 1u << 1,
    Uniform = 1u << 2,
};
constexpr RHIBufferUsage operator|(RHIBufferUsage a, RHIBufferUsage b) {
    return static_cast<RHIBufferUsage>(static_cast<uint32_t>(a) | static_cast<uint32_t>(b));
}
constexpr bool HasFlag(RHIBufferUsage value, RHIBufferUsage flag) {
    return (static_cast<uint32_t>(value) & static_cast<uint32_t>(flag)) != 0;
}

// Which shader stages a binding or push-constant range is visible to.
enum class RHIShaderStage : uint32_t {
    None     = 0,
    Vertex   = 1u << 0,
    Fragment = 1u << 1,
};
constexpr RHIShaderStage operator|(RHIShaderStage a, RHIShaderStage b) {
    return static_cast<RHIShaderStage>(static_cast<uint32_t>(a) | static_cast<uint32_t>(b));
}
constexpr bool HasFlag(RHIShaderStage value, RHIShaderStage flag) {
    return (static_cast<uint32_t>(value) & static_cast<uint32_t>(flag)) != 0;
}

} // namespace Diamond
