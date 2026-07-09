#pragma once

#include "Renderer/RHI/RHIEnums.h"
#include "Renderer/RHI/RHIResources.h"
#include "Profiling/ProfilerStats.h"

#include <array>
#include <memory>
#include <vector>

struct GLFWwindow;

namespace Diamond {

class RHICommandList;

// Backend-neutral rendering device: creates GPU resources and drives the frame
// loop. The concrete backend (Vulkan now, OpenGL next) is selected by Create and
// never leaks its native types through this interface — the same discipline the
// engine already applies to its OpenGL classes, extended so a single pass body
// runs unchanged on either backend.
class RHIDevice {
public:
    virtual ~RHIDevice() = default;

    // 'window' must be created with GLFW_NO_API when 'backend' is Vulkan. Returns
    // nullptr if the requested backend isn't compiled into this build.
    static std::unique_ptr<RHIDevice> Create(GLFWwindow* window, RHIBackend backend);

    virtual std::unique_ptr<RHIBuffer>   CreateBuffer(const RHIBufferDesc& desc) = 0;
    virtual std::unique_ptr<RHIShader>   CreateShader(const RHIShaderDesc& desc) = 0;
    virtual std::unique_ptr<RHITexture>  CreateTexture(const RHITextureDesc& desc) = 0;
    virtual std::unique_ptr<RHIPipeline> CreatePipeline(const RHIPipelineDesc& desc) = 0;
    virtual std::unique_ptr<RHIResourceSet> CreateResourceSet(
        RHIPipeline* pipeline, uint32_t setIndex,
        const std::vector<RHIBufferBinding>&  buffers,
        const std::vector<RHITextureBinding>& textures = {}) = 0;

    // Format of the swapchain color target — feed into RHIPipelineDesc::colorFormat
    // so pipelines match the backbuffer they render to.
    virtual RHIFormat SwapchainFormat() const = 0;
    // Format of the device's per-frame depth attachment — feed into
    // RHIPipelineDesc::depthFormat for any pipeline that depth-tests.
    virtual RHIFormat DepthFormat() const = 0;

    // Frame loop. BeginFrame acquires the next backbuffer image and returns a
    // recorder — or nullptr when the frame must be skipped (minimized / swapchain
    // rebuilt), in which case EndFrame must NOT be called. No rendering scope is
    // open on return: pass code drives BeginRendering/EndRendering itself,
    // including the final pass that targets the swapchain. EndFrame finishes
    // recording, submits, and presents.
    virtual RHICommandList* BeginFrame() = 0;
    virtual void EndFrame() = 0;

    // Call from the window framebuffer-resize callback.
    virtual void NotifyResize() = 0;
    // Block until the GPU is idle (before tearing resources down).
    virtual void WaitIdle() = 0;

    // ── Frame stats (Docs/profiler-panel-design.md, Phase 1: CPU-side counters
    // only) ───────────────────────────────────────────────────────────────────
    // A working accumulator every draw/upload/culling choke point in this
    // device (and the SceneRenderer built on top of it) writes into, snapshotted
    // into a stable last-completed-frame copy GetStats() returns. Culling runs
    // before BeginFrame, so ResetFrameStats/FinalizeFrameStats bracket the whole
    // SceneRenderer::RenderToSwapchain call, not just BeginFrame/EndFrame.
    virtual void ResetFrameStats() = 0;
    virtual void RecordDraw(uint64_t triangleCount) = 0;
    virtual void RecordBufferUpload() = 0;
    virtual void RecordVisible(uint32_t count) = 0;
    virtual void RecordCulled(uint32_t count) = 0;
    virtual void RecordShadowCaster() = 0;
    // Deduped internally (unique per frame) — pass any stable pointer identity.
    virtual void RecordMaterialBound(const void* material) = 0;
    virtual void RecordTextureUsed(const void* texture) = 0;
    virtual void FinalizeFrameStats() = 0;
    virtual const RendererStats& GetStats() const = 0;

    // ── Per-pass profiling (Docs/profiler-panel-design.md, per-pass phase) ────
    // Brackets the recording of one render pass, between BeginFrame/EndFrame:
    // writes GPU timestamps around it, times the CPU recording, and attributes
    // RecordDraw calls to it. 'scope' groups passes for display ("Main View",
    // "Shadows"); width/height 0 means the backbuffer. Does not nest — a Begin
    // while a pass is open is ignored (with its matching End). Default no-ops:
    // the Vulkan device implements them; profiling is Vulkan-only by design.
    virtual void BeginPassProfile(const char* /*scope*/, const char* /*name*/,
                                  uint32_t /*width*/, uint32_t /*height*/) {}
    virtual void EndPassProfile() {}
};

} // namespace Diamond
