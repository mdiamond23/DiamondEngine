#pragma once

#include "Renderer/RHI/RHIDevice.h"
#include "Renderer/RHI/RHICommandList.h"
#include "Platform/Vulkan/VulkanContext.h"
#include "Platform/Vulkan/VulkanSwapchain.h"
#include "Platform/Vulkan/VulkanImage.h"

#include <array>
#include <cstdint>
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
    void Reset(VkCommandBuffer cmd) { m_Cmd = cmd; m_Layout = VK_NULL_HANDLE; }

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

private:
    VulkanRHIDevice* m_Device;
    VkCommandBuffer  m_Cmd    = VK_NULL_HANDLE;
    VkPipelineLayout m_Layout = VK_NULL_HANDLE;   // of the currently-bound pipeline
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
    std::unique_ptr<RHIResourceSet> CreateResourceSet(
        RHIPipeline* pipeline, uint32_t setIndex,
        const std::vector<RHIBufferBinding>&  buffers,
        const std::vector<RHITextureBinding>& textures) override;
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

    uint32_t m_CurrentFrame       = 0;
    uint32_t m_AcquiredImageIndex = 0;   // valid between BeginFrame and EndFrame
    bool     m_FramebufferResized = false;
    bool     m_FrameActive        = false;
};

// Defined in VulkanRHIDevice.cpp; called by the always-compiled RHIDevice::Create.
std::unique_ptr<RHIDevice> CreateVulkanRHIDevice(GLFWwindow* window);

} // namespace Diamond
