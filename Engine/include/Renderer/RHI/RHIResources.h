#pragma once

#include "Renderer/RHI/RHIEnums.h"

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
    RHIPushConstantRange            pushConstants;

    RHIPrimitiveTopology topology    = RHIPrimitiveTopology::TriangleList;
    RHICullMode          cullMode    = RHICullMode::None;
    RHIFormat            colorFormat = RHIFormat::Undefined;   // render-target format
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

class RHIResourceSet {
public:
    virtual ~RHIResourceSet() = default;
};

} // namespace Diamond
