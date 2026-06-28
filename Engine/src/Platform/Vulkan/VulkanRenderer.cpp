#include "Platform/Vulkan/VulkanRenderer.h"

#define GLFW_INCLUDE_NONE
#include <GLFW/glfw3.h>

namespace Diamond {

namespace {

// synchronization2 image-layout transition. Stage/access masks are scoped to a
// color-attachment write so the barrier is no broader than it needs to be.
void TransitionImage(VkCommandBuffer cmd, VkImage image,
                     VkImageLayout oldLayout, VkImageLayout newLayout,
                     VkPipelineStageFlags2 srcStage, VkAccessFlags2 srcAccess,
                     VkPipelineStageFlags2 dstStage, VkAccessFlags2 dstAccess) {
    VkImageMemoryBarrier2 barrier{ VK_STRUCTURE_TYPE_IMAGE_MEMORY_BARRIER_2 };
    barrier.srcStageMask  = srcStage;
    barrier.srcAccessMask = srcAccess;
    barrier.dstStageMask  = dstStage;
    barrier.dstAccessMask = dstAccess;
    barrier.oldLayout     = oldLayout;
    barrier.newLayout     = newLayout;
    barrier.image         = image;
    barrier.subresourceRange.aspectMask = VK_IMAGE_ASPECT_COLOR_BIT;
    barrier.subresourceRange.levelCount = 1;
    barrier.subresourceRange.layerCount = 1;

    VkDependencyInfo dep{ VK_STRUCTURE_TYPE_DEPENDENCY_INFO };
    dep.imageMemoryBarrierCount = 1;
    dep.pImageMemoryBarriers    = &barrier;
    vkCmdPipelineBarrier2(cmd, &dep);
}

} // namespace

void VulkanRenderer::Init(GLFWwindow* window) {
    m_Window = window;
    m_Ctx.Init(window);
    m_Swapchain.Init(&m_Ctx, window);
    CreateFrameResources();
}

void VulkanRenderer::CreateFrameResources() {
    VkDevice device = m_Ctx.Device();

    VkSemaphoreCreateInfo semInfo{ VK_STRUCTURE_TYPE_SEMAPHORE_CREATE_INFO };
    VkFenceCreateInfo fenceInfo{ VK_STRUCTURE_TYPE_FENCE_CREATE_INFO };
    fenceInfo.flags = VK_FENCE_CREATE_SIGNALED_BIT; // so the first wait returns immediately

    for (Frame& frame : m_Frames) {
        VkCommandPoolCreateInfo poolInfo{ VK_STRUCTURE_TYPE_COMMAND_POOL_CREATE_INFO };
        poolInfo.queueFamilyIndex = m_Ctx.Queues().graphics;
        // We reset the whole pool each frame, so no per-buffer reset flag needed.
        VK_CHECK(vkCreateCommandPool(device, &poolInfo, nullptr, &frame.commandPool));

        VkCommandBufferAllocateInfo allocInfo{ VK_STRUCTURE_TYPE_COMMAND_BUFFER_ALLOCATE_INFO };
        allocInfo.commandPool        = frame.commandPool;
        allocInfo.level              = VK_COMMAND_BUFFER_LEVEL_PRIMARY;
        allocInfo.commandBufferCount = 1;
        VK_CHECK(vkAllocateCommandBuffers(device, &allocInfo, &frame.commandBuffer));

        VK_CHECK(vkCreateSemaphore(device, &semInfo, nullptr, &frame.imageAvailable));
        VK_CHECK(vkCreateFence(device, &fenceInfo, nullptr, &frame.inFlight));
    }

    RecreateRenderFinishedSemaphores();
}

void VulkanRenderer::RecreateRenderFinishedSemaphores() {
    VkDevice device = m_Ctx.Device();
    for (VkSemaphore s : m_RenderFinished)
        vkDestroySemaphore(device, s, nullptr);
    m_RenderFinished.assign(m_Swapchain.ImageCount(), VK_NULL_HANDLE);

    VkSemaphoreCreateInfo semInfo{ VK_STRUCTURE_TYPE_SEMAPHORE_CREATE_INFO };
    for (VkSemaphore& s : m_RenderFinished)
        VK_CHECK(vkCreateSemaphore(device, &semInfo, nullptr, &s));
}

void VulkanRenderer::RecreateSwapchain() {
    // Wait out minimization — a zero-area framebuffer can't host a swapchain.
    int w = 0, h = 0;
    glfwGetFramebufferSize(m_Window, &w, &h);
    while (w == 0 || h == 0) {
        glfwWaitEvents();
        glfwGetFramebufferSize(m_Window, &w, &h);
    }

    m_Swapchain.Recreate();
    // Image count can change across recreate — keep the per-image semaphores in step.
    RecreateRenderFinishedSemaphores();
}

void VulkanRenderer::DrawFrame(const std::array<float, 4>& clearColor) {
    if (m_Swapchain.IsZeroSize()) { RecreateSwapchain(); return; }

    VkDevice device = m_Ctx.Device();
    Frame&   frame  = m_Frames[m_CurrentFrame];

    // 1. Wait until this frame slot's previous submission has retired.
    VK_CHECK(vkWaitForFences(device, 1, &frame.inFlight, VK_TRUE, UINT64_MAX));

    // 2. Acquire the next image. OUT_OF_DATE means the swapchain no longer
    //    matches the surface — rebuild and try again next frame.
    uint32_t imageIndex = 0;
    VkResult acquire = vkAcquireNextImageKHR(device, m_Swapchain.Handle(), UINT64_MAX,
                                             frame.imageAvailable, VK_NULL_HANDLE, &imageIndex);
    if (acquire == VK_ERROR_OUT_OF_DATE_KHR) { RecreateSwapchain(); return; }
    if (acquire != VK_SUCCESS && acquire != VK_SUBOPTIMAL_KHR) {
        spdlog::critical("[Vulkan] vkAcquireNextImageKHR failed: {}", VkResultToString(acquire));
        std::abort();
    }

    // Only reset the fence once we know we're submitting work this frame.
    VK_CHECK(vkResetFences(device, 1, &frame.inFlight));
    VK_CHECK(vkResetCommandPool(device, frame.commandPool, 0));

    // 3. Record: transition to attachment layout, clear via dynamic rendering,
    //    transition to present layout.
    VkCommandBuffer cmd = frame.commandBuffer;
    VkCommandBufferBeginInfo begin{ VK_STRUCTURE_TYPE_COMMAND_BUFFER_BEGIN_INFO };
    begin.flags = VK_COMMAND_BUFFER_USAGE_ONE_TIME_SUBMIT_BIT;
    VK_CHECK(vkBeginCommandBuffer(cmd, &begin));

    TransitionImage(cmd, m_Swapchain.Image(imageIndex),
                    VK_IMAGE_LAYOUT_UNDEFINED, VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL,
                    VK_PIPELINE_STAGE_2_COLOR_ATTACHMENT_OUTPUT_BIT, 0,
                    VK_PIPELINE_STAGE_2_COLOR_ATTACHMENT_OUTPUT_BIT,
                    VK_ACCESS_2_COLOR_ATTACHMENT_WRITE_BIT);

    VkRenderingAttachmentInfo color{ VK_STRUCTURE_TYPE_RENDERING_ATTACHMENT_INFO };
    color.imageView   = m_Swapchain.ImageView(imageIndex);
    color.imageLayout = VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL;
    color.loadOp      = VK_ATTACHMENT_LOAD_OP_CLEAR;
    color.storeOp     = VK_ATTACHMENT_STORE_OP_STORE;
    color.clearValue.color = { { clearColor[0], clearColor[1], clearColor[2], clearColor[3] } };

    VkRenderingInfo rendering{ VK_STRUCTURE_TYPE_RENDERING_INFO };
    rendering.renderArea.extent  = m_Swapchain.Extent();
    rendering.layerCount         = 1;
    rendering.colorAttachmentCount = 1;
    rendering.pColorAttachments  = &color;

    vkCmdBeginRendering(cmd, &rendering);
    // M0: nothing drawn — loadOp CLEAR fills the frame.
    vkCmdEndRendering(cmd);

    TransitionImage(cmd, m_Swapchain.Image(imageIndex),
                    VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL, VK_IMAGE_LAYOUT_PRESENT_SRC_KHR,
                    VK_PIPELINE_STAGE_2_COLOR_ATTACHMENT_OUTPUT_BIT,
                    VK_ACCESS_2_COLOR_ATTACHMENT_WRITE_BIT,
                    VK_PIPELINE_STAGE_2_BOTTOM_OF_PIPE_BIT, 0);

    VK_CHECK(vkEndCommandBuffer(cmd));

    // 4. Submit (synchronization2). Wait on image-available before the color
    //    output stage; signal this image's render-finished semaphore + the fence.
    VkSemaphore signalSem = m_RenderFinished[imageIndex];

    VkCommandBufferSubmitInfo cmdSubmit{ VK_STRUCTURE_TYPE_COMMAND_BUFFER_SUBMIT_INFO };
    cmdSubmit.commandBuffer = cmd;

    VkSemaphoreSubmitInfo waitSem{ VK_STRUCTURE_TYPE_SEMAPHORE_SUBMIT_INFO };
    waitSem.semaphore = frame.imageAvailable;
    waitSem.stageMask = VK_PIPELINE_STAGE_2_COLOR_ATTACHMENT_OUTPUT_BIT;

    VkSemaphoreSubmitInfo signalSemInfo{ VK_STRUCTURE_TYPE_SEMAPHORE_SUBMIT_INFO };
    signalSemInfo.semaphore = signalSem;
    signalSemInfo.stageMask = VK_PIPELINE_STAGE_2_COLOR_ATTACHMENT_OUTPUT_BIT;

    VkSubmitInfo2 submit{ VK_STRUCTURE_TYPE_SUBMIT_INFO_2 };
    submit.waitSemaphoreInfoCount   = 1;
    submit.pWaitSemaphoreInfos      = &waitSem;
    submit.commandBufferInfoCount   = 1;
    submit.pCommandBufferInfos      = &cmdSubmit;
    submit.signalSemaphoreInfoCount = 1;
    submit.pSignalSemaphoreInfos    = &signalSemInfo;
    VK_CHECK(vkQueueSubmit2(m_Ctx.GraphicsQueue(), 1, &submit, frame.inFlight));

    // 5. Present.
    VkPresentInfoKHR present{ VK_STRUCTURE_TYPE_PRESENT_INFO_KHR };
    present.waitSemaphoreCount = 1;
    present.pWaitSemaphores    = &signalSem;
    present.swapchainCount     = 1;
    VkSwapchainKHR swapchain   = m_Swapchain.Handle();
    present.pSwapchains        = &swapchain;
    present.pImageIndices      = &imageIndex;

    VkResult presentRes = vkQueuePresentKHR(m_Ctx.PresentQueue(), &present);
    if (presentRes == VK_ERROR_OUT_OF_DATE_KHR || presentRes == VK_SUBOPTIMAL_KHR ||
        m_FramebufferResized) {
        m_FramebufferResized = false;
        RecreateSwapchain();
    } else if (presentRes != VK_SUCCESS) {
        spdlog::critical("[Vulkan] vkQueuePresentKHR failed: {}", VkResultToString(presentRes));
        std::abort();
    }

    m_CurrentFrame = (m_CurrentFrame + 1) % kFramesInFlight;
}

void VulkanRenderer::DestroyFrameResources() {
    VkDevice device = m_Ctx.Device();
    for (VkSemaphore s : m_RenderFinished)
        vkDestroySemaphore(device, s, nullptr);
    m_RenderFinished.clear();

    for (Frame& frame : m_Frames) {
        vkDestroyFence(device, frame.inFlight, nullptr);
        vkDestroySemaphore(device, frame.imageAvailable, nullptr);
        // Command buffers are freed with their pool.
        vkDestroyCommandPool(device, frame.commandPool, nullptr);
        frame = {};
    }
}

void VulkanRenderer::Shutdown() {
    vkDeviceWaitIdle(m_Ctx.Device());   // never destroy resources the GPU is still using
    DestroyFrameResources();
    m_Swapchain.Destroy();
    m_Ctx.Shutdown();
}

} // namespace Diamond
