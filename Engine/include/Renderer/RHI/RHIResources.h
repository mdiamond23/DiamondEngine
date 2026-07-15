#pragma once

#include "Renderer/RHI/RHIEnums.h"

#include <array>
#include <cstddef>
#include <cstdint>
#include <vector>

// Opaque GPU-resource interfaces plus the plain-data descriptors used to create
// them. Resources are created through RHIDevice and owned by the caller via
// unique_ptr; the device must outlive them.
namespace Diamond {

// ── Buffer ───────────────────────────────────────────────────────────────────
struct RHIBufferDesc {
    uint64_t       size  = 0;
    RHIBufferUsage usage = RHIBufferUsage::None;

    // Dynamic buffers are host-visible and rewritten by the CPU every frame; the
    // device keeps one copy per frame-in-flight so writing next frame's data
    // never races the GPU still reading this frame's. Static buffers are
    // device-local and uploaded once from initialData.
    bool        dynamic     = false;
    const void* initialData = nullptr;   // required for static buffers
};

class RHIBuffer {
public:
    virtual ~RHIBuffer() = default;
    // Dynamic buffers only: copy 'size' bytes into the current frame's copy.
    // Must be called after RHIDevice::BeginFrame (which frees the frame slot).
    virtual void Update(const void* data, uint64_t size) = 0;
};

// ── Texture ──────────────────────────────────────────────────────────────────
struct RHITextureDesc {
    uint32_t  width  = 0;
    uint32_t  height = 0;
    RHIFormat format = RHIFormat::RGBA8;

    RHITextureUsage usage = RHITextureUsage::Sampled;

    // Tightly-packed pixels matching 'format' (e.g. RGBA8 = 4 bytes/pixel),
    // uploaded once through a staging buffer. Null for render targets, which are
    // produced on the GPU rather than uploaded. When non-null the texture is
    // static (single image); render targets are double-buffered internally so two
    // in-flight frames never write the same image.
    const void* initialData = nullptr;

    RHIFilter filter = RHIFilter::Linear;

    // Uploaded textures only: build the full mip chain from initialData at upload
    // (blit downsample), sampled trilinearly. Material textures want this — the GL
    // texture path always generates mipmaps; render targets ignore it.
    bool generateMips = false;

    // Pre-baked mip chain (cooked DDS): when > 0, initialData holds this many
    // mips tightly packed largest-first, each RHIFormatLevelSize() bytes, and
    // generateMips is ignored — block-compressed images can't be blit-downsampled.
    // 0 = initialData is a single mip-0 image (the behavior above).
    uint32_t mipCount = 0;

    // Render targets only: keep ONE image instead of one per frame-in-flight, so
    // content written in frame N is what frame N+1 samples — required for
    // GPU-persistent accumulation buffers (TAA history). Only safe for textures
    // that are exclusively GPU-written/GPU-read with explicit transitions: the
    // per-FIF duplication this opts out of protects attachment reuse across
    // concurrent frames, which same-queue barriers already order here.
    bool singleBuffered = false;

    // Diagnostic name shown by capture tools (RenderDoc resource inspector).
    // Empty = unnamed. Must outlive the CreateTexture call only.
    const char* debugName = nullptr;
};

// A texture usable as a sampled image and/or a render-pass attachment, depending
// on its usage flags. Static (uploaded) textures are a single image; render
// targets keep one image per frame-in-flight so the frame loop never races them.
class RHITexture {
public:
    virtual ~RHITexture() = default;
};

// ── Render pass (dynamic-rendering scope on the command list) ─────────────────
struct RHIAttachment {
    RHITexture*         texture    = nullptr;          // ignored for the swapchain color target
    bool                clear      = true;             // CLEAR vs LOAD the existing contents
    std::array<float, 4> clearColor { 0.0f, 0.0f, 0.0f, 1.0f };
};

// Describes the attachments a BeginRendering scope writes to. Either the
// swapchain backbuffer (toSwapchain) or a set of offscreen textures.
struct RHIRenderPass {
    bool                       toSwapchain = false;
    std::vector<RHIAttachment> colorAttachments;       // toSwapchain: [0] holds the backbuffer clear

    // Offscreen depth target; ignored when toSwapchain (use useDeviceDepth there).
    RHITexture* depthTexture    = nullptr;
    bool        useDeviceDepth  = false;               // toSwapchain only
    bool        clearDepth      = true;
    float       depthClearValue = 1.0f;
};

// ── Shader ───────────────────────────────────────────────────────────────────
struct RHIShaderDesc {
    RHIShaderStage  stage     = RHIShaderStage::None;
    const uint32_t* spirv     = nullptr;   // SPIR-V words (GLSL→SPIR-V offline at build)
    size_t          wordCount = 0;
};

class RHIShader {
public:
    virtual ~RHIShader() = default;
};

// ── Pipeline ─────────────────────────────────────────────────────────────────
struct RHIVertexAttribute {
    uint32_t        location = 0;
    RHIVertexFormat format   = RHIVertexFormat::Float;
    uint32_t        offset   = 0;
};

struct RHIVertexLayout {
    uint32_t                        stride = 0;
    std::vector<RHIVertexAttribute> attributes;
};

// One binding in descriptor set 0 (per-frame resources). Material/per-draw sets
// arrive with later milestones.
struct RHIResourceBinding {
    uint32_t        binding = 0;
    RHIResourceType type    = RHIResourceType::UniformBuffer;
    RHIShaderStage  stages  = RHIShaderStage::None;
};

struct RHIPushConstantRange {
    RHIShaderStage stages = RHIShaderStage::None;
    uint32_t       size   = 0;   // 0 = no push constants
};

struct RHIPipelineDesc {
    RHIShader* vertexShader   = nullptr;
    RHIShader* fragmentShader = nullptr;

    RHIVertexLayout                 vertexLayout;
    std::vector<RHIResourceBinding> resourceBindings;   // descriptor set 0
    // Optional descriptor set 1. Empty (the default) means the pipeline layout
    // declares a single set. Skinned pipelines put the per-entity bone-matrix UBO
    // here (set 0 stays the shared material/camera set), so a skinned draw reuses
    // the same set-0 resources as its static counterpart and only swaps set 1.
    std::vector<RHIResourceBinding> resourceBindings1;
    RHIPushConstantRange            pushConstants;

    RHIPrimitiveTopology topology    = RHIPrimitiveTopology::TriangleList;
    RHICullMode          cullMode    = RHICullMode::None;
    RHIFormat            colorFormat = RHIFormat::Undefined;   // single render-target format

    // MRT: one format per color attachment, in attachment (frag `location`) order.
    // When non-empty this overrides colorFormat — a deferred G-buffer pass declares
    // its targets here. Single-target passes keep using colorFormat.
    std::vector<RHIFormat> colorFormats;

    // Depth-buffer state. depthFormat == Undefined means the pipeline renders
    // without a depth attachment (depthTest/depthWrite ignored). When set, it
    // must match the device's DepthFormat() so dynamic rendering is compatible.
    RHIFormat    depthFormat  = RHIFormat::Undefined;
    bool         depthTest    = false;
    bool         depthWrite   = false;
    RHICompareOp depthCompare = RHICompareOp::Less;

    // Color blending. Opaque (overwrite) for every geometry/post pass; Alpha
    // (src-over) for forward transparency. Applied to all color attachments.
    RHIBlendMode blendMode = RHIBlendMode::Opaque;
};

class RHIPipeline {
public:
    virtual ~RHIPipeline() = default;
};

// ── Resource set (descriptor set analogue) ───────────────────────────────────
struct RHIBufferBinding {
    uint32_t   binding = 0;
    RHIBuffer* buffer  = nullptr;
};

struct RHITextureBinding {
    uint32_t    binding = 0;
    RHITexture* texture = nullptr;
};

class RHIResourceSet {
public:
    virtual ~RHIResourceSet() = default;
};

} // namespace Diamond
