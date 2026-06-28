#pragma once

#include "Renderer/RHI/RHIEnums.h"

#include <cstdint>

namespace Diamond {

class RHIPipeline;
class RHIResourceSet;
class RHIBuffer;

// Records draw work for the current frame. Obtained from RHIDevice::BeginFrame
// and valid only until the matching EndFrame. The render target (swapchain
// image) is already bound and cleared and the viewport/scissor already cover the
// full target, so pass code starts straight at BindPipeline. This is the surface
// a render pass is written against — once, for every backend.
class RHICommandList {
public:
    virtual ~RHICommandList() = default;

    virtual void BindPipeline(RHIPipeline* pipeline) = 0;
    virtual void BindResourceSet(uint32_t setIndex, RHIResourceSet* set) = 0;
    virtual void PushConstants(RHIShaderStage stages, uint32_t offset,
                               uint32_t size, const void* data) = 0;

    virtual void BindVertexBuffer(RHIBuffer* buffer) = 0;
    virtual void BindIndexBuffer(RHIBuffer* buffer, RHIIndexType type) = 0;

    virtual void DrawIndexed(uint32_t indexCount, uint32_t instanceCount = 1,
                             uint32_t firstIndex = 0) = 0;
};

} // namespace Diamond
