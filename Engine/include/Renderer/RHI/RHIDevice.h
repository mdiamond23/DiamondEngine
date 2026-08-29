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
    // Compute counterpart. Returns the same handle type, so resource sets and
    // BindPipeline are shared with graphics.
    virtual std::unique_ptr<RHIPipeline> CreateComputePipeline(
        const RHIComputePipelineDesc& desc) = 0;
    virtual std::unique_ptr<RHIResourceSet> CreateResourceSet(
        RHIPipeline* pipeline, uint32_t setIndex,
        const std::vector<RHIBufferBinding>&  buffers,
        const std::vector<RHITextureBinding>& textures = {},
        const std::vector<RHIAccelStructBinding>& accelStructs = {}) = 0;

    // ── Ray tracing (Docs/gi-design.md slice 2) ──────────────────────────────
    // False on any device without the KHR ray-tracing extensions. Everything
    // below returns nullptr in that case, so callers gate on this once at setup
    // rather than null-checking every result.
    virtual bool SupportsRayTracing() const { return false; }

    // One BLAS per registered mesh, built synchronously (blocks on the GPU).
    virtual std::unique_ptr<RHIAccelStruct> CreateBLAS(const RHIBLASDesc& /*desc*/) {
        return nullptr;
    }
    // TLAS over BLAS instances. An empty list yields nullptr — an empty scene
    // has nothing to trace against.
    virtual std::unique_ptr<RHIAccelStruct> CreateTLAS(
        const std::vector<RHITLASInstance>& /*instances*/) { return nullptr; }
    // Re-point an existing TLAS at a new instance set (the static geometry
    // changed). Blocks like the build does; the handle stays valid, so resource
    // sets built against it need no rebuild — unless it returns false, which
    // means the instance count outgrew the allocation and the caller must
    // recreate the TLAS (and any set that binds it).
    virtual bool RebuildTLAS(RHIAccelStruct* /*tlas*/,
                             const std::vector<RHITLASInstance>& /*instances*/) {
        return false;
    }
    // Update an existing TLAS in place for new instance TRANSFORMS only. The
    // topology must be unchanged — same BLASes, same count, same order — which
    // is what makes it cheap enough to run every frame for moving objects.
    // False means the update was not legal and the caller must fall back to
    // RebuildTLAS. The handle is unaffected either way.
    virtual bool RefitTLAS(RHIAccelStruct* /*tlas*/,
                           const std::vector<RHITLASInstance>& /*instances*/) {
        return false;
    }

    virtual std::unique_ptr<RHIPipeline> CreateRayTracingPipeline(
        const RHIRayTracingPipelineDesc& /*desc*/) { return nullptr; }

    // GPU address of a buffer created with ShaderDeviceAddress usage — what a
    // shader dereferences through GL_EXT_buffer_reference. DDGI's closest-hit
    // shader reaches mesh vertex/index data this way instead of through
    // descriptors, so the per-instance geometry table is just addresses.
    // Returns 0 on a backend without device addresses, or for a null buffer.
    virtual uint64_t BufferAddress(RHIBuffer* /*buffer*/) const { return 0; }

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
