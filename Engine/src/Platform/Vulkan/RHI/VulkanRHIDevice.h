#pragma once

#include "Renderer/RHI/RHIDevice.h"
#include "Renderer/RHI/RHICommandList.h"
#include "Platform/Vulkan/VulkanContext.h"
#include "Platform/Vulkan/VulkanSwapchain.h"
#include "Platform/Vulkan/VulkanImage.h"

// Tracy GPU zones (phase 2): TracyVulkan.hpp needs Vulkan types in scope —
// VulkanContext.h above pulls in volk, whose global function pointers also
// satisfy Tracy's direct vk* calls under VK_NO_PROTOTYPES.
#ifdef DIAMOND_TRACY
#include <tracy/TracyVulkan.hpp>
#include <optional>
#endif

#include <array>
#include <chrono>
#include <cstdint>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

struct GLFWwindow;

namespace Diamond {

class VulkanRHIDevice;

// Vulkan implementation of the RHI command recorder. Thin: each call forwards to
// vkCmd* on the frame's command buffer. Bound pipeline state (its layout) is
// cached so descriptor-set and push-constant calls can reference it.
class VulkanRHICommandList : public RHICommandList {
public:
    explicit VulkanRHICommandList(VulkanRHIDevice* device) : m_Device(device) {}

    // Re-targets the recorder at a freshly-begun command buffer for a new frame.
    void Reset(VkCommandBuffer cmd) {
        m_Cmd       = cmd;
        m_Layout    = VK_NULL_HANDLE;
        m_BindPoint = VK_PIPELINE_BIND_POINT_GRAPHICS;
        m_Bound     = nullptr;
    }

    // The frame's raw command buffer — for Vulkan-only overlays (e.g. ImGui) that
    // record directly rather than through the RHI. Valid between BeginFrame/EndFrame.
    VkCommandBuffer Handle() const { return m_Cmd; }

    void BeginRendering(const RHIRenderPass& pass) override;
    void EndRendering() override;
    void TransitionTexture(RHITexture* texture, RHITextureState state) override;

    void BindPipeline(RHIPipeline* pipeline) override;
    void BindResourceSet(uint32_t setIndex, RHIResourceSet* set) override;
    void PushConstants(RHIShaderStage stages, uint32_t offset,
                       uint32_t size, const void* data) override;
    void BindVertexBuffer(RHIBuffer* buffer) override;
    void BindIndexBuffer(RHIBuffer* buffer, RHIIndexType type) override;
    void DrawIndexed(uint32_t indexCount, uint32_t instanceCount,
                     uint32_t firstIndex) override;
    void Draw(uint32_t vertexCount, uint32_t instanceCount,
              uint32_t firstVertex) override;
    void Dispatch(uint32_t groupsX, uint32_t groupsY, uint32_t groupsZ) override;
    void TraceRays(uint32_t width, uint32_t height, uint32_t depth) override;
    void StorageBarrier() override;

    void BeginDebugLabel(const char* name) override;
    void EndDebugLabel() override;

private:
    VulkanRHIDevice* m_Device;
    VkCommandBuffer  m_Cmd    = VK_NULL_HANDLE;
    // Layout + bind point of the currently-bound pipeline; descriptor sets and
    // push constants target whichever one BindPipeline last selected.
    VkPipelineLayout    m_Layout    = VK_NULL_HANDLE;
    VkPipelineBindPoint m_BindPoint = VK_PIPELINE_BIND_POINT_GRAPHICS;
    // The bound pipeline itself — TraceRays reads its SBT regions.
    class VulkanRHIPipeline* m_Bound = nullptr;
};

// Vulkan RHIDevice: owns the context + swapchain + per-frame synchronization and
// the acquire→record→present loop (the logic the M1 VulkanRenderer prototyped,
// now behind the backend-neutral interface). Backend resource classes reach
// device internals (VkDevice, allocator, current frame slot, descriptor pool)
// through the accessors below.
class VulkanRHIDevice : public RHIDevice {
public:
    static constexpr uint32_t kFramesInFlight = 2;

    explicit VulkanRHIDevice(GLFWwindow* window);
    ~VulkanRHIDevice() override;

    // RHIDevice interface.
    std::unique_ptr<RHIBuffer>   CreateBuffer(const RHIBufferDesc& desc) override;
    std::unique_ptr<RHIShader>   CreateShader(const RHIShaderDesc& desc) override;
    std::unique_ptr<RHITexture>  CreateTexture(const RHITextureDesc& desc) override;
    std::unique_ptr<RHIPipeline> CreatePipeline(const RHIPipelineDesc& desc) override;
    std::unique_ptr<RHIPipeline> CreateComputePipeline(
        const RHIComputePipelineDesc& desc) override;
    std::unique_ptr<RHIResourceSet> CreateResourceSet(
        RHIPipeline* pipeline, uint32_t setIndex,
        const std::vector<RHIBufferBinding>&  buffers,
        const std::vector<RHITextureBinding>& textures = {},
        // Same default as the base declaration — callers holding a
        // VulkanRHIDevice* bind against THIS declaration, so omitting it here
        // would break every existing two-list call site.
        const std::vector<RHIAccelStructBinding>& accelStructs = {}) override;
    bool SupportsRayTracing() const override { return m_Ctx.RayTracingSupported(); }
    std::unique_ptr<RHIAccelStruct> CreateBLAS(const RHIBLASDesc& desc) override;
    std::unique_ptr<RHIAccelStruct> CreateTLAS(
        const std::vector<RHITLASInstance>& instances) override;
    bool RebuildTLAS(RHIAccelStruct* tlas,
                     const std::vector<RHITLASInstance>& instances) override;
    std::unique_ptr<RHIPipeline> CreateRayTracingPipeline(
        const RHIRayTracingPipelineDesc& desc) override;
    uint64_t BufferAddress(RHIBuffer* buffer) const override;
    RHIFormat SwapchainFormat() const override;
    RHIFormat DepthFormat() const override;
    RHICommandList* BeginFrame() override;
    void EndFrame() override;
    void NotifyResize() override { m_FramebufferResized = true; }
    void WaitIdle() override { vkDeviceWaitIdle(m_Ctx.Device()); }

    // RHIDevice frame-stats interface (see RHIDevice.h).
    void ResetFrameStats() override;
    void RecordDraw(uint64_t triangleCount) override;
    void RecordBufferUpload() override;
    void RecordVisible(uint32_t count) override;
    void RecordCulled(uint32_t count) override;
    void RecordShadowCaster() override;
    void RecordMaterialBound(const void* material) override;
    void RecordTextureUsed(const void* texture) override;
    void FinalizeFrameStats() override;
    const RendererStats& GetStats() const override { return m_SnapshotStats; }
    void BeginPassProfile(const char* scope, const char* name,
                          uint32_t width, uint32_t height) override;
    void EndPassProfile() override;

    // Per-frame depth attachment format (D32_SFLOAT); also surfaced through the
    // RHI DepthFormat() so pipelines match.
    static constexpr VkFormat kDepthFormat = VK_FORMAT_D32_SFLOAT;

    // Backend-internal accessors used by the resource classes.
    VulkanContext&  Ctx()             { return m_Ctx; }
    uint32_t        CurrentFrame()    const { return m_CurrentFrame; }
    VkDescriptorPool DescriptorPool() const { return m_DescriptorPool; }

    // Swapchain properties an ImGui/overlay backend needs to build its pipeline.
    VkFormat SwapchainVkFormat()   const { return m_Swapchain.ImageFormat(); }
    uint32_t SwapchainImageCount() const { return m_Swapchain.ImageCount(); }

    // Acquired backbuffer + the device depth buffer for the in-flight frame —
    // used by the command list when a render pass targets the swapchain.
    VkImage     SwapchainImage()     const { return m_Swapchain.Image(m_AcquiredImageIndex); }
    VkImageView SwapchainImageView() const { return m_Swapchain.ImageView(m_AcquiredImageIndex); }
    VkExtent2D  SwapchainExtent()    const { return m_Swapchain.Extent(); }
    VkImageView DeviceDepthView()    const { return m_DepthImages[m_CurrentFrame].view; }

private:
    void CreateFrameResources();
    void DestroyFrameResources();
    void CreateDepthResources();
    void DestroyDepthResources();
    void CreateDescriptorPool();
    void RecreateSwapchain();
    void RecreateRenderFinishedSemaphores();
    // GPU frame timing (Docs/profiler-panel-design.md, Phase 2). Query pool
    // sized per frame-in-flight slot; results for slot N are read back the next
    // time slot N is reused, at which point the fence wait already guarantees
    // that submission has finished on the GPU.
    void CreateTimestampPool();
    void DestroyTimestampPool();
    // Publishes slot's per-pass records (GPU spans + captured CPU counters)
    // into m_WorkingStats.passes, then clears them. Called from BeginFrame
    // right after the fence wait proves the slot's submission finished.
    void ResolvePassRecords(uint32_t slot);

    GLFWwindow*     m_Window = nullptr;
    VulkanContext   m_Ctx;
    VulkanSwapchain m_Swapchain;

    struct Frame {
        VkCommandPool   commandPool    = VK_NULL_HANDLE;
        VkCommandBuffer commandBuffer  = VK_NULL_HANDLE;
        VkSemaphore     imageAvailable = VK_NULL_HANDLE;
        VkFence         inFlight       = VK_NULL_HANDLE;
    };
    std::array<Frame, kFramesInFlight> m_Frames;

    // One depth image per frame-in-flight (a single shared depth buffer would be
    // raced by two concurrent frames). Sized to the swapchain extent and
    // recreated with it. Cleared every frame (loadOp CLEAR), never stored.
    std::array<VulkanImage, kFramesInFlight> m_DepthImages;

    // One render-finished semaphore per swapchain image (present waits can't be
    // reused while still pending; acquisition order isn't ours to control).
    std::vector<VkSemaphore> m_RenderFinished;

    // Shared pool all resource sets allocate from (per-frame reset strategy is a
    // later optimization; M2.1 allocates persistent sets at create time).
    VkDescriptorPool m_DescriptorPool = VK_NULL_HANDLE;

    VulkanRHICommandList m_CommandList{ this };

    // Frame-stats accumulator: m_WorkingStats collects the frame currently being
    // built (RebuildDrawList..EndFrame), m_SnapshotStats is the last completed
    // frame GetStats() returns. The sets exist only to dedup materialsBound/
    // texturesUsed; their sizes are folded into m_WorkingStats at finalize time.
    RendererStats m_WorkingStats;
    RendererStats m_SnapshotStats;
    std::unordered_set<const void*> m_MaterialsSeen;
    std::unordered_set<const void*> m_TexturesSeen;

    // GPU frame timing (Phase 2): timestamp pool with 2 queries (top/bottom of
    // frame) per frame-in-flight slot. m_TimestampValid[slot] is false until
    // that slot has completed one full submission, so the first couple frames
    // read no result (RendererStats::gpuFrameMs stays 0 -> panel shows "n/a").
    VkQueryPool m_TimestampPool       = VK_NULL_HANDLE;
    bool        m_TimestampsSupported = false;
    std::array<bool, kFramesInFlight> m_TimestampValid{};

    // Per-pass profiling (per-pass phase): a second timestamp pool with one
    // begin/end query pair per profiled pass, kMaxProfiledPasses pairs per
    // frame-in-flight slot. Each slot's PassRecord list is written while its
    // frame records (names + CPU-side counters captured then, since the graph
    // may be rebuilt before readback) and resolved against the query results
    // when the slot's fence wait proves the submission finished — so published
    // rows are kFramesInFlight frames old, like gpuFrameMs.
    static constexpr uint32_t kMaxProfiledPasses = 64;
    struct PassRecord {
        std::string scope;
        std::string name;
        uint32_t    width = 0, height = 0;
        uint32_t    drawCalls = 0;
        uint64_t    triangles = 0;
        float       cpuMs = 0.0f;
    };
    std::array<std::vector<PassRecord>, kFramesInFlight> m_PassRecords;
    VkQueryPool m_PassQueryPool = VK_NULL_HANDLE;
    int         m_ActivePass    = -1;   // index into the current slot's records
    uint32_t    m_PassSkipDepth = 0;    // Begins ignored while a pass was open
    std::chrono::steady_clock::time_point m_PassCpuStart;
    // EMA-smoothed (gpuMs, cpuMs) keyed "scope/name" — raw per-pass times are
    // too jittery to read. Entries for passes that stop running just linger.
    std::unordered_map<std::string, std::pair<float, float>> m_PassSmoothed;

    uint32_t m_CurrentFrame       = 0;
    uint32_t m_AcquiredImageIndex = 0;   // valid between BeginFrame and EndFrame
    bool     m_FramebufferResized = false;
    bool     m_FrameActive        = false;

#ifdef DIAMOND_TRACY
    // Tracy GPU track: one zone per profiled pass, riding the same
    // Begin/EndPassProfile hooks as the panel's timestamps. The RAII zone must
    // outlive the Begin call, so it's stored — a single slot suffices because
    // pass profiles never nest (m_PassSkipDepth enforces it).
    void CreateTracyVkContext();
    tracy::VkCtx*                    m_TracyVkCtx = nullptr;
    std::optional<tracy::VkCtxScope> m_TracyPassZone;
#endif
};

// Defined in VulkanRHIDevice.cpp; called by the always-compiled RHIDevice::Create.
std::unique_ptr<RHIDevice> CreateVulkanRHIDevice(GLFWwindow* window);

} // namespace Diamond
