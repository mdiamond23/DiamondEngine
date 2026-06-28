#pragma once

#include "Platform/Vulkan/VulkanContext.h"
#include "Platform/Vulkan/VulkanSwapchain.h"
#include <array>
#include <cstdint>

struct GLFWwindow;

namespace Diamond {

// M0 frame driver: owns the context + swapchain + per-frame synchronization and
// runs the acquire → clear → present loop. This is the seed the RHI grows from;
// at M1 the empty dynamic-rendering scope gains a pipeline + draw.
class VulkanRenderer {
public:
    // Number of frames the CPU may record ahead of the GPU. Two is the standard
    // balance of latency vs. throughput.
    static constexpr uint32_t kFramesInFlight = 2;

    void Init(GLFWwindow* window);
    void Shutdown();

    // Records and submits one frame that clears the screen to 'clearColor'.
    void DrawFrame(const std::array<float, 4>& clearColor);

    // Marks the swapchain stale (call from the GLFW framebuffer-resize callback).
    void NotifyResize() { m_FramebufferResized = true; }

    VulkanContext& Context() { return m_Ctx; }

private:
    void CreateFrameResources();
    void DestroyFrameResources();
    void RecreateSwapchain();
    void RecreateRenderFinishedSemaphores();

    GLFWwindow*     m_Window = nullptr;
    VulkanContext   m_Ctx;
    VulkanSwapchain m_Swapchain;

    // Per-frame-in-flight state.
    struct Frame {
        VkCommandPool   commandPool   = VK_NULL_HANDLE;
        VkCommandBuffer commandBuffer = VK_NULL_HANDLE;
        VkSemaphore     imageAvailable = VK_NULL_HANDLE; // signaled by acquire
        VkFence         inFlight       = VK_NULL_HANDLE;  // CPU waits before reusing the slot
    };
    std::array<Frame, kFramesInFlight> m_Frames;

    // One render-finished semaphore per *swapchain image* (not per frame) — a
    // present's wait semaphore can't be reused while the present is still
    // pending, and image acquisition order is not under our control.
    std::vector<VkSemaphore> m_RenderFinished;

    uint32_t m_CurrentFrame      = 0;
    bool     m_FramebufferResized = false;
};

} // namespace Diamond
