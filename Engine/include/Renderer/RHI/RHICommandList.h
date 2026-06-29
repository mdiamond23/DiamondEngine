#pragma once

#include "Renderer/RHI/RHIEnums.h"
#include "Renderer/RHI/RHIResources.h"

#include <cstdint>

namespace Diamond {

class RHIPipeline;
class RHIResourceSet;
class RHIBuffer;
class RHITexture;

// Records draw work for the current frame. Obtained from RHIDevice::BeginFrame
// and valid only until the matching EndFrame. Nothing is bound on entry: pass
// code opens a render scope with BeginRendering (offscreen targets or the
// swapchain backbuffer), records draws, then EndRendering. The scope binds and
// clears its attachments and sets a full-target viewport/scissor. This is the
// surface a render pass is written against — once, for every backend.
class RHICommandList {
public:
    virtual ~RHICommandList() = default;

    // Opens/closes a dynamic-rendering scope. Draw calls are only valid between a
    // BeginRendering / EndRendering pair; passes do not nest.
    virtual void BeginRendering(const RHIRenderPass& pass) = 0;
    virtual void EndRendering() = 0;

    // Transitions a texture to a new usage state outside a render scope — e.g.
    // moving an offscreen color target from ColorTarget to SampledRead so a later
    // pass can sample it. The render graph will drive these automatically; here it
    // is explicit. Must be called outside BeginRendering/EndRendering.
    virtual void TransitionTexture(RHITexture* texture, RHITextureState state) = 0;

    virtual void BindPipeline(RHIPipeline* pipeline) = 0;
    virtual void BindResourceSet(uint32_t setIndex, RHIResourceSet* set) = 0;
    virtual void PushConstants(RHIShaderStage stages, uint32_t offset,
                               uint32_t size, const void* data) = 0;

    virtual void BindVertexBuffer(RHIBuffer* buffer) = 0;
    virtual void BindIndexBuffer(RHIBuffer* buffer, RHIIndexType type) = 0;

    virtual void DrawIndexed(uint32_t indexCount, uint32_t instanceCount = 1,
                             uint32_t firstIndex = 0) = 0;

    // Non-indexed draw — used by vertex-less passes (e.g. a fullscreen triangle
    // generated from gl_VertexIndex) that bind no vertex/index buffer.
    virtual void Draw(uint32_t vertexCount, uint32_t instanceCount = 1,
                      uint32_t firstVertex = 0) = 0;
};

} // namespace Diamond
