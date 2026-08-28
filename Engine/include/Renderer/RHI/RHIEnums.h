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
    BGRA8,        // swapchain format (UNORM — shaders apply gamma manually, GL parity)
    BGRA8_SRGB,   // sRGB variant (hardware-encoded; unused by the swapchain, see BGRA8)
    RGBA16F,
    // Packed 32-bit HDR: 11/11/10 unsigned float, NO alpha. Half the bytes of
    // RGBA16F for the scene-color chain, which is written and read back once per
    // post stage — at 4K that chain is the largest single source of framebuffer
    // traffic in the frame. Unsigned is fine here: the one shader that can
    // produce negatives (taa_resolve's YCoCg round-trip) already clamps to 0.
    // Only for targets whose alpha is genuinely unused — hdrSSR carries SSR's
    // reflection weight in alpha and must stay RGBA16F.
    R11G11B10F,
    R16F,
    RG16F,        // 2-channel HDR (screen-space motion vectors)
    Depth32F,
    BC7,          // block-compressed (UNORM — cooked textures keep the manual shader gamma, upload-only, never a render target
    BC5,          // 2-channel (RG) block-compressed normal maps; Z reconstructed in-shader
    BC4,          // 1-channel (R) block-compressed masks (roughness/metallic/AO)
};

// Byte size of one mip level in 'format'. Block-compressed formats are 4×4-texel
// blocks with dimensions rounded up to whole blocks — a 1×1 tail mip still
// occupies a full block (16 bytes for BC7/BC5, 8 for BC4 — half the channels,
// half the block).
inline uint64_t RHIFormatLevelSize(RHIFormat format, uint32_t width, uint32_t height) {
    switch (format) {
        case RHIFormat::BC7:
        case RHIFormat::BC5:
            return uint64_t((width + 3) / 4) * ((height + 3) / 4) * 16;
        case RHIFormat::BC4:
            return uint64_t((width + 3) / 4) * ((height + 3) / 4) * 8;
        case RHIFormat::RGBA16F: return uint64_t(width) * height * 8;
        case RHIFormat::R16F:    return uint64_t(width) * height * 2;
        case RHIFormat::RG16F:   return uint64_t(width) * height * 4;
        case RHIFormat::R11G11B10F: return uint64_t(width) * height * 4;
        default:                 return uint64_t(width) * height * 4;   // 8-bit RGBA variants, Depth32F
    }
}

inline const char* RHIFormatName(RHIFormat format) {
    switch (format) {
        case RHIFormat::BC7: return "BC7";
        case RHIFormat::BC5: return "BC5";
        case RHIFormat::BC4: return "BC4";
        default:             return "?";
    }
}

// Vertex attribute component layout.
enum class RHIVertexFormat {
    Float,
    Float2,
    Float3,
    Float4,
    Int4,     // signed 32-bit ivec4 — skinning bone indices (read as ivec4 in-shader)
};

enum class RHIIndexType { U16, U32 };

enum class RHIPrimitiveTopology { TriangleList, TriangleStrip, LineList };

enum class RHICullMode { None, Back, Front };

// Color-blend mode for a pipeline's color attachments. Opaque overwrites (the
// default for every geometry/post pass so far); Alpha is standard src-over
// blending (src.a, 1-src.a) for forward transparency.
enum class RHIBlendMode { Opaque, Alpha, Additive };

// Depth comparison. Less is the default for opaque geometry; LessEqual lets a
// skybox drawn at the far plane (depth == 1) pass against a depth buffer cleared
// to 1, filling only the pixels no geometry wrote.
enum class RHICompareOp { Less, LessEqual };

// Resource kinds a descriptor binding can hold. The storage variants are
// compute-side: written (and read) through image2D/buffer blocks rather than
// sampled, so they need the General layout and a storage barrier, not a sampler.
enum class RHIResourceType {
    UniformBuffer,
    CombinedImageSampler,
    StorageBuffer,
    StorageImage,
    AccelerationStructure,
};

// Texture sampling filter for minification/magnification.
enum class RHIFilter { Nearest, Linear };

// Layout/usage state a texture can be transitioned to between render passes.
// Drives the image-memory barriers the command list (and later the render graph)
// inserts so a written attachment can be safely sampled by a later pass.
// Storage covers both compute reads and writes through an image2D binding — the
// underlying layout permits both, so one state serves either direction.
enum class RHITextureState { SampledRead, ColorTarget, DepthTarget, Storage };

// What roles a texture must support. A render target sampled by a later pass is
// ColorAttachment | Sampled; a depth buffer is DepthAttachment; an uploaded
// image read in shaders is Sampled. Storage means a compute shader binds it as
// an image2D — combine with Sampled for the usual write-then-sample chain.
enum class RHITextureUsage : uint32_t {
    None            = 0,
    Sampled         = 1u << 0,
    ColorAttachment = 1u << 1,
    DepthAttachment = 1u << 2,
    Storage         = 1u << 3,
};
constexpr RHITextureUsage operator|(RHITextureUsage a, RHITextureUsage b) {
    return static_cast<RHITextureUsage>(static_cast<uint32_t>(a) | static_cast<uint32_t>(b));
}
constexpr bool HasFlag(RHITextureUsage value, RHITextureUsage flag) {
    return (static_cast<uint32_t>(value) & static_cast<uint32_t>(flag)) != 0;
}

// ── Bitmask enums ────────────────────────────────────────────────────────────
// Which buffer roles a buffer may serve.
enum class RHIBufferUsage : uint32_t {
    None                = 0,
    Vertex              = 1u << 0,
    Index               = 1u << 1,
    Uniform             = 1u << 2,
    Storage             = 1u << 3,   // read/written by a shader through a buffer block
    ShaderDeviceAddress = 1u << 4,   // buffer's GPU address readable in shaders
    AccelStructInput    = 1u << 5,   // BLAS build reads vertices/indices from it
    AccelStructStorage  = 1u << 6,   // backing store of an acceleration structure
    ShaderBindingTable  = 1u << 7,
};
constexpr RHIBufferUsage operator|(RHIBufferUsage a, RHIBufferUsage b) {
    return static_cast<RHIBufferUsage>(static_cast<uint32_t>(a) | static_cast<uint32_t>(b));
}
constexpr bool HasFlag(RHIBufferUsage value, RHIBufferUsage flag) {
    return (static_cast<uint32_t>(value) & static_cast<uint32_t>(flag)) != 0;
}

// Which shader stages a binding or push-constant range is visible to.
enum class RHIShaderStage : uint32_t {
    None       = 0,
    Vertex     = 1u << 0,
    Fragment   = 1u << 1,
    Compute    = 1u << 2,
    RayGen     = 1u << 3,
    Miss       = 1u << 4,
    ClosestHit = 1u << 5,
};
constexpr RHIShaderStage operator|(RHIShaderStage a, RHIShaderStage b) {
    return static_cast<RHIShaderStage>(static_cast<uint32_t>(a) | static_cast<uint32_t>(b));
}
constexpr bool HasFlag(RHIShaderStage value, RHIShaderStage flag) {
    return (static_cast<uint32_t>(value) & static_cast<uint32_t>(flag)) != 0;
}

} // namespace Diamond
