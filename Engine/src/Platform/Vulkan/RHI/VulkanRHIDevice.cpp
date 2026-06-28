#include "Platform/Vulkan/RHI/VulkanRHIDevice.h"
#include "Platform/Vulkan/RHI/VulkanRHIResources.h"
#include "Platform/Vulkan/RHI/VulkanRHIEnums.h"

#define GLFW_INCLUDE_NONE
#include <GLFW/glfw3.h>

namespace Diamond {

namespace {

// synchronization2 image-layout transition, scoped to a color-attachment write.
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

// ── Command list ─────────────────────────────────────────────────────────────

void VulkanRHICommandList::BindPipeline(RHIPipeline* pipeline) {
    auto* vp = static_cast<VulkanRHIPipeline*>(pipeline);
    m_Layout = vp->Layout();
    vkCmdBindPipeline(m_Cmd, VK_PIPELINE_BIND_POINT_GRAPHICS, vp->Handle());
}

void VulkanRHICommandList::BindResourceSet(uint32_t setIndex, RHIResourceSet* set) {
    auto* vs = static_cast<VulkanRHIResourceSet*>(set);
    VkDescriptorSet ds = vs->Handle(m_Device->CurrentFrame());
    vkCmdBindDescriptorSets(m_Cmd, VK_PIPELINE_BIND_POINT_GRAPHICS, m_Layout,
                            setIndex, 1, &ds, 0, nullptr);
}

void VulkanRHICommandList::PushConstants(RHIShaderStage stages, uint32_t offset,
                                         uint32_t size, const void* data) {
    vkCmdPushConstants(m_Cmd, m_Layout, ToVkShaderStages(stages), offset, size, data);
}

void VulkanRHICommandList::BindVertexBuffer(RHIBuffer* buffer) {
    auto* vb = static_cast<VulkanRHIBuffer*>(buffer);
    VkBuffer handle = vb->Handle(m_Device->CurrentFrame());
    const VkDeviceSize offset = 0;
    vkCmdBindVertexBuffers(m_Cmd, 0, 1, &handle, &offset);
}

void VulkanRHICommandList::BindIndexBuffer(RHIBuffer* buffer, RHIIndexType type) {
    auto* ib = static_cast<VulkanRHIBuffer*>(buffer);
    vkCmdBindIndexBuffer(m_Cmd, ib->Handle(m_Device->CurrentFrame()), 0, ToVkIndexType(type));
}

void VulkanRHICommandList::DrawIndexed(uint32_t indexCount, uint32_t instanceCount,
                                       uint32_t firstIndex) {
    vkCmdDrawIndexed(m_Cmd, indexCount, instanceCount, firstIndex, 0, 0);
}

// ── Device lifetime ──────────────────────────────────────────────────────────

VulkanRHIDevice::VulkanRHIDevice(GLFWwindow* window) : m_Window(window) {
    m_Ctx.Init(window);
    m_Swapchain.Init(&m_Ctx, window);
    CreateFrameResources();
    CreateDescriptorPool();
}

VulkanRHIDevice::~VulkanRHIDevice() {
    vkDeviceWaitIdle(m_Ctx.Device());
    if (m_DescriptorPool) vkDestroyDescriptorPool(m_Ctx.Device(), m_DescriptorPool, nullptr);
    DestroyFrameResources();
    m_Swapchain.Destroy();
    m_Ctx.Shutdown();
}

void VulkanRHIDevice::CreateFrameResources() {
    VkDevice device = m_Ctx.Device();

    VkSemaphoreCreateInfo semInfo{ VK_STRUCTURE_TYPE_SEMAPHORE_CREATE_INFO };
    VkFenceCreateInfo fenceInfo{ VK_STRUCTURE_TYPE_FENCE_CREATE_INFO };
    fenceInfo.flags = VK_FENCE_CREATE_SIGNALED_BIT;

    for (Frame& frame : m_Frames) {
        VkCommandPoolCreateInfo poolInfo{ VK_STRUCTURE_TYPE_COMMAND_POOL_CREATE_INFO };
        poolInfo.queueFamilyIndex = m_Ctx.Queues().graphics;
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

void VulkanRHIDevice::CreateDescriptorPool() {
    // Generously sized for M2.1's handful of per-frame UBO sets; the per-frame
    // reset / caching-allocator strategy is a later optimization.
    VkDescriptorPoolSize poolSize{};
    poolSize.type            = VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER;
    poolSize.descriptorCount = 64;

    VkDescriptorPoolCreateInfo poolInfo{ VK_STRUCTURE_TYPE_DESCRIPTOR_POOL_CREATE_INFO };
    poolInfo.maxSets       = 64;
    poolInfo.poolSizeCount = 1;
    poolInfo.pPoolSizes    = &poolSize;
    VK_CHECK(vkCreateDescriptorPool(m_Ctx.Device(), &poolInfo, nullptr, &m_DescriptorPool));
}

void VulkanRHIDevice::RecreateRenderFinishedSemaphores() {
    VkDevice device = m_Ctx.Device();
    for (VkSemaphore s : m_RenderFinished) vkDestroySemaphore(device, s, nullptr);
    m_RenderFinished.assign(m_Swapchain.ImageCount(), VK_NULL_HANDLE);

    VkSemaphoreCreateInfo semInfo{ VK_STRUCTURE_TYPE_SEMAPHORE_CREATE_INFO };
    for (VkSemaphore& s : m_RenderFinished)
        VK_CHECK(vkCreateSemaphore(device, &semInfo, nullptr, &s));
}

void VulkanRHIDevice::RecreateSwapchain() {
    int w = 0, h = 0;
    glfwGetFramebufferSize(m_Window, &w, &h);
    while (w == 0 || h == 0) {
        glfwWaitEvents();
        glfwGetFramebufferSize(m_Window, &w, &h);
    }
    m_Swapchain.Recreate();
    RecreateRenderFinishedSemaphores();
}

void VulkanRHIDevice::DestroyFrameResources() {
    VkDevice device = m_Ctx.Device();
    for (VkSemaphore s : m_RenderFinished) vkDestroySemaphore(device, s, nullptr);
    m_RenderFinished.clear();

    for (Frame& frame : m_Frames) {
        vkDestroyFence(device, frame.inFlight, nullptr);
        vkDestroySemaphore(device, frame.imageAvailable, nullptr);
        vkDestroyCommandPool(device, frame.commandPool, nullptr);
        frame = {};
    }
}

// ── Resource creation ────────────────────────────────────────────────────────

std::unique_ptr<RHIBuffer> VulkanRHIDevice::CreateBuffer(const RHIBufferDesc& desc) {
    return std::make_unique<VulkanRHIBuffer>(this, desc);
}

std::unique_ptr<RHIShader> VulkanRHIDevice::CreateShader(const RHIShaderDesc& desc) {
    return std::make_unique<VulkanRHIShader>(this, desc);
}

std::unique_ptr<RHIPipeline> VulkanRHIDevice::CreatePipeline(const RHIPipelineDesc& desc) {
    return std::make_unique<VulkanRHIPipeline>(this, desc);
}

std::unique_ptr<RHIResourceSet> VulkanRHIDevice::CreateResourceSet(
    RHIPipeline* pipeline, uint32_t setIndex, const std::vector<RHIBufferBinding>& buffers) {
    return std::make_unique<VulkanRHIResourceSet>(
        this, static_cast<VulkanRHIPipeline*>(pipeline), setIndex, buffers);
}

RHIFormat VulkanRHIDevice::SwapchainFormat() const {
    return FromVkFormat(m_Swapchain.ImageFormat());
}

// ── Frame loop ───────────────────────────────────────────────────────────────

RHICommandList* VulkanRHIDevice::BeginFrame(const std::array<float, 4>& clearColor) {
    if (m_Swapchain.IsZeroSize()) { RecreateSwapchain(); return nullptr; }

    VkDevice device = m_Ctx.Device();
    Frame&   frame  = m_Frames[m_CurrentFrame];

    VK_CHECK(vkWaitForFences(device, 1, &frame.inFlight, VK_TRUE, UINT64_MAX));

    VkResult acquire = vkAcquireNextImageKHR(device, m_Swapchain.Handle(), UINT64_MAX,
                                             frame.imageAvailable, VK_NULL_HANDLE,
                                             &m_AcquiredImageIndex);
    if (acquire == VK_ERROR_OUT_OF_DATE_KHR) { RecreateSwapchain(); return nullptr; }
    if (acquire != VK_SUCCESS && acquire != VK_SUBOPTIMAL_KHR) {
        spdlog::critical("[Vulkan] vkAcquireNextImageKHR failed: {}", VkResultToString(acquire));
        std::abort();
    }

    VK_CHECK(vkResetFences(device, 1, &frame.inFlight));
    VK_CHECK(vkResetCommandPool(device, frame.commandPool, 0));

    VkCommandBuffer cmd = frame.commandBuffer;
    VkCommandBufferBeginInfo begin{ VK_STRUCTURE_TYPE_COMMAND_BUFFER_BEGIN_INFO };
    begin.flags = VK_COMMAND_BUFFER_USAGE_ONE_TIME_SUBMIT_BIT;
    VK_CHECK(vkBeginCommandBuffer(cmd, &begin));

    TransitionImage(cmd, m_Swapchain.Image(m_AcquiredImageIndex),
                    VK_IMAGE_LAYOUT_UNDEFINED, VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL,
                    VK_PIPELINE_STAGE_2_COLOR_ATTACHMENT_OUTPUT_BIT, 0,
                    VK_PIPELINE_STAGE_2_COLOR_ATTACHMENT_OUTPUT_BIT,
                    VK_ACCESS_2_COLOR_ATTACHMENT_WRITE_BIT);

    VkRenderingAttachmentInfo color{ VK_STRUCTURE_TYPE_RENDERING_ATTACHMENT_INFO };
    color.imageView   = m_Swapchain.ImageView(m_AcquiredImageIndex);
    color.imageLayout = VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL;
    color.loadOp      = VK_ATTACHMENT_LOAD_OP_CLEAR;
    color.storeOp     = VK_ATTACHMENT_STORE_OP_STORE;
    color.clearValue.color = { { clearColor[0], clearColor[1], clearColor[2], clearColor[3] } };

    const VkExtent2D extent = m_Swapchain.Extent();
    VkRenderingInfo rendering{ VK_STRUCTURE_TYPE_RENDERING_INFO };
    rendering.renderArea.extent    = extent;
    rendering.layerCount           = 1;
    rendering.colorAttachmentCount = 1;
    rendering.pColorAttachments    = &color;
    vkCmdBeginRendering(cmd, &rendering);

    // Y-flipped viewport so GL-style clip space maps to Vulkan; pass code never
    // touches the viewport.
    VkViewport viewport{};
    viewport.x        = 0.0f;
    viewport.y        = static_cast<float>(extent.height);
    viewport.width    = static_cast<float>(extent.width);
    viewport.height   = -static_cast<float>(extent.height);
    viewport.minDepth = 0.0f;
    viewport.maxDepth = 1.0f;
    vkCmdSetViewport(cmd, 0, 1, &viewport);

    VkRect2D scissor{};
    scissor.extent = extent;
    vkCmdSetScissor(cmd, 0, 1, &scissor);

    m_CommandList.Reset(cmd);
    m_FrameActive = true;
    return &m_CommandList;
}

void VulkanRHIDevice::EndFrame() {
    if (!m_FrameActive) return;
    m_FrameActive = false;

    Frame&          frame = m_Frames[m_CurrentFrame];
    VkCommandBuffer cmd   = frame.commandBuffer;

    vkCmdEndRendering(cmd);

    TransitionImage(cmd, m_Swapchain.Image(m_AcquiredImageIndex),
                    VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL, VK_IMAGE_LAYOUT_PRESENT_SRC_KHR,
                    VK_PIPELINE_STAGE_2_COLOR_ATTACHMENT_OUTPUT_BIT,
                    VK_ACCESS_2_COLOR_ATTACHMENT_WRITE_BIT,
                    VK_PIPELINE_STAGE_2_BOTTOM_OF_PIPE_BIT, 0);

    VK_CHECK(vkEndCommandBuffer(cmd));

    VkSemaphore signalSem = m_RenderFinished[m_AcquiredImageIndex];

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

    VkPresentInfoKHR present{ VK_STRUCTURE_TYPE_PRESENT_INFO_KHR };
    present.waitSemaphoreCount = 1;
    present.pWaitSemaphores    = &signalSem;
    present.swapchainCount     = 1;
    VkSwapchainKHR swapchain   = m_Swapchain.Handle();
    present.pSwapchains        = &swapchain;
    present.pImageIndices      = &m_AcquiredImageIndex;

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

std::unique_ptr<RHIDevice> CreateVulkanRHIDevice(GLFWwindow* window) {
    return std::make_unique<VulkanRHIDevice>(window);
}

} // namespace Diamond
