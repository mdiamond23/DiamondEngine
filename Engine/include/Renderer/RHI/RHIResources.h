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
    // Static buffers upload this once at creation. Null leaves the allocation
    // uninitialized — correct for a GPU-produced storage buffer, wrong for a
    // vertex/index buffer that nothing else fills.
    const void* initialData = nullptr;
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

// ── Compute pipeline ─────────────────────────────────────────────────────────
// One shader plus its binding layout — no vertex input, no attachments, no
// fixed-function state, so it gets its own descriptor rather than leaving most
// of RHIPipelineDesc meaningless. The result is an ordinary RHIPipeline: it
// binds and takes resource sets exactly like a graphics one, and the command
// list picks the bind point from the pipeline itself.
struct RHIComputePipelineDesc {
    RHIShader* computeShader = nullptr;

    std::vector<RHIResourceBinding> resourceBindings;    // descriptor set 0
    std::vector<RHIResourceBinding> resourceBindings1;   // optional set 1
    RHIPushConstantRange            pushConstants;
};

// ── Acceleration structures ──────────────────────────────────────────────────
// The BVH ray traversal walks. Two levels: a BLAS holds one mesh's triangles in
// local space and is built once at mesh registration; the TLAS holds instances
// of BLASes with world transforms and is rebuilt when the static set changes.
// Only available when RHIDevice::SupportsRayTracing().
class RHIAccelStruct {
public:
    virtual ~RHIAccelStruct() = default;
};

// One BLAS from one indexed triangle mesh. Both buffers must have been created
// with AccelStructInput | ShaderDeviceAddress — the build reads them by GPU
// address, not through a descriptor. Position must sit at offset 0 of the vertex.
struct RHIBLASDesc {
    RHIBuffer* vertexBuffer = nullptr;
    uint32_t   vertexCount  = 0;
    uint32_t   vertexStride = 0;
    RHIBuffer* indexBuffer  = nullptr;   // 32-bit indices
    uint32_t   indexCount   = 0;
    const char* debugName   = nullptr;
};

// One instance in the TLAS. 'transform' is a ROW-major 3x4 world matrix — the
// transpose of the top three columns of a glm::mat4, which is column-major.
struct RHITLASInstance {
    float           transform[12] { 1,0,0,0,  0,1,0,0,  0,0,1,0 };
    RHIAccelStruct* blas        = nullptr;
    // Surfaces as gl_InstanceCustomIndexEXT in the hit shader — the index into
    // the per-instance geometry table that slice 3 adds. Unused in slice 2.
    uint32_t        customIndex = 0;
};

// ── Ray-tracing pipeline ─────────────────────────────────────────────────────
// Raygen + miss + closest-hit, and the shader binding table wiring them to the
// traversal. Like the compute variant this yields an ordinary RHIPipeline; the
// command list picks the ray-tracing bind point from the pipeline itself and
// TraceRays reads the SBT regions off it.
struct RHIRayTracingPipelineDesc {
    RHIShader* raygenShader     = nullptr;
    RHIShader* missShader       = nullptr;
    RHIShader* closestHitShader = nullptr;

    std::vector<RHIResourceBinding> resourceBindings;    // descriptor set 0
    RHIPushConstantRange            pushConstants;

    // Depth of traceRayEXT calls made from *within* a hit/miss shader. 1 means
    // primary rays only, which is all DDGI needs — the recursive bounce term
    // comes from the previous frame's probe irradiance, not from a recursive ray.
    uint32_t maxRecursionDepth = 1;
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

struct RHIAccelStructBinding {
    uint32_t        binding = 0;
    RHIAccelStruct* accel   = nullptr;
};

class RHIResourceSet {
public:
    virtual ~RHIResourceSet() = default;
};

} // namespace Diamond
