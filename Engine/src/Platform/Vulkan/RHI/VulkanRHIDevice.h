#pragma once

#include "Renderer/RHI/RHIDevice.h"
#include "Renderer/RHI/RHICommandList.h"
#include "Platform/Vulkan/VulkanContext.h"
#include "Platform/Vulkan/VulkanSwapchain.h"
#include "Platform/Vulkan/VulkanImage.h"

#include <array>
#include <cstdint>
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

    void BindPipeline(RHIPipeline* pipeline) override;
    void BindResourceSet(uint32_t setIndex, RHIResourceSet* set) override;
    void PushConstants(RHIShaderStage stages, uint32_t offset,
                       uint32_t size, const void* data) override;
    void BindVertexBuffer(RHIBuffer* buffer) override;
    void BindIndexBuffer(RHIBuffer* buffer, RHIIndexType type) override;
    void DrawIndexed(uint32_t indexCount, uint32_t instanceCount,
                     uint32_t firstIndex) override;

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
    RHICommandList* BeginFrame(const std::array<float, 4>& clearColor) override;
    void EndFrame() override;
    void NotifyResize() override { m_FramebufferResized = true; }
    void WaitIdle() override { vkDeviceWaitIdle(m_Ctx.Device()); }

    // Per-frame depth attachment format (D32_SFLOAT); also surfaced through the
    // RHI DepthFormat() so pipelines match.
    static constexpr VkFormat kDepthFormat = VK_FORMAT_D32_SFLOAT;

    // Backend-internal accessors used by the resource classes.
    VulkanContext&  Ctx()             { return m_Ctx; }
    uint32_t        CurrentFrame()    const { return m_CurrentFrame; }
    VkDescriptorPool DescriptorPool() const { return m_DescriptorPool; }

private:
    void CreateFrameResources();
    void DestroyFrameResources();
    void CreateDepthResources();
    void DestroyDepthResources();
    void CreateDescriptorPool();
    void RecreateSwapchain();
    void RecreateRenderFinishedSemaphores();

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

    uint32_t m_CurrentFrame       = 0;
    uint32_t m_AcquiredImageIndex = 0;   // valid between BeginFrame and EndFrame
    bool     m_FramebufferResized = false;
    bool     m_FrameActive        = false;
};

// Defined in VulkanRHIDevice.cpp; called by the always-compiled RHIDevice::Create.
std::unique_ptr<RHIDevice> CreateVulkanRHIDevice(GLFWwindow* window);

} // namespace Diamond
