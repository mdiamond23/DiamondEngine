#include "Platform/Vulkan/RHI/VulkanRHIDevice.h"
#include "Platform/Vulkan/RHI/VulkanRHIResources.h"
#include "Platform/Vulkan/RHI/VulkanRHIEnums.h"
#include "Platform/Vulkan/VulkanComputeSelfTest.h"

#define GLFW_INCLUDE_NONE
#include <GLFW/glfw3.h>

#include <cstdlib>

#ifdef DIAMOND_TRACY
#include <cstdio>
#include <cstring>
#endif

namespace Diamond {

// ── Command list ─────────────────────────────────────────────────────────────

void VulkanRHICommandList::BindPipeline(RHIPipeline* pipeline) {
    auto* vp = static_cast<VulkanRHIPipeline*>(pipeline);
    m_Layout    = vp->Layout();
    m_BindPoint = vp->BindPoint();
    vkCmdBindPipeline(m_Cmd, m_BindPoint, vp->Handle());
}

void VulkanRHICommandList::BindResourceSet(uint32_t setIndex, RHIResourceSet* set) {
    auto* vs = static_cast<VulkanRHIResourceSet*>(set);
    VkDescriptorSet ds = vs->Handle(m_Device->CurrentFrame());
    vkCmdBindDescriptorSets(m_Cmd, m_BindPoint, m_Layout, setIndex, 1, &ds, 0, nullptr);
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
    m_Device->RecordDraw((uint64_t)(indexCount / 3) * instanceCount);
}

void VulkanRHICommandList::Draw(uint32_t vertexCount, uint32_t instanceCount,
                                uint32_t firstVertex) {
    vkCmdDraw(m_Cmd, vertexCount, instanceCount, firstVertex, 0);
    m_Device->RecordDraw((uint64_t)(vertexCount / 3) * instanceCount);
}

void VulkanRHICommandList::Dispatch(uint32_t groupsX, uint32_t groupsY, uint32_t groupsZ) {
    vkCmdDispatch(m_Cmd, groupsX, groupsY, groupsZ);
}

void VulkanRHICommandList::StorageBarrier() {
    // One global memory barrier rather than per-resource buffer barriers: the
    // callers are whole passes, so the extra scope costs nothing measurable and
    // there is no bookkeeping to get wrong. Images keep using TransitionTexture,
    // which must also change their layout.
    VkMemoryBarrier2 barrier{ VK_STRUCTURE_TYPE_MEMORY_BARRIER_2 };
    barrier.srcStageMask  = VK_PIPELINE_STAGE_2_COMPUTE_SHADER_BIT;
    barrier.srcAccessMask = VK_ACCESS_2_SHADER_STORAGE_WRITE_BIT;
    barrier.dstStageMask  = VK_PIPELINE_STAGE_2_COMPUTE_SHADER_BIT
                          | VK_PIPELINE_STAGE_2_VERTEX_SHADER_BIT
                          | VK_PIPELINE_STAGE_2_FRAGMENT_SHADER_BIT;
    barrier.dstAccessMask = VK_ACCESS_2_SHADER_STORAGE_READ_BIT
                          | VK_ACCESS_2_SHADER_STORAGE_WRITE_BIT
                          | VK_ACCESS_2_UNIFORM_READ_BIT;

    VkDependencyInfo dep{ VK_STRUCTURE_TYPE_DEPENDENCY_INFO };
    dep.memoryBarrierCount = 1;
    dep.pMemoryBarriers    = &barrier;
    vkCmdPipelineBarrier2(m_Cmd, &dep);
}

void VulkanRHICommandList::BeginDebugLabel(const char* name) {
    // Entry points are null when VK_EXT_debug_utils wasn't enabled.
    if (!m_Device->Ctx().DebugUtilsEnabled() || !vkCmdBeginDebugUtilsLabelEXT) return;
    VkDebugUtilsLabelEXT label{ VK_STRUCTURE_TYPE_DEBUG_UTILS_LABEL_EXT };
    label.pLabelName = name;
    vkCmdBeginDebugUtilsLabelEXT(m_Cmd, &label);
}

void VulkanRHICommandList::EndDebugLabel() {
    if (!m_Device->Ctx().DebugUtilsEnabled() || !vkCmdEndDebugUtilsLabelEXT) return;
    vkCmdEndDebugUtilsLabelEXT(m_Cmd);
}

// ── Render-pass scoping + barriers ───────────────────────────────────────────

namespace {

// The synchronization2 stage/access scope for work that leaves an image in a
// given layout — used as the *source* scope when transitioning away from it.
void StageAccessForLayout(VkImageLayout layout,
                          VkPipelineStageFlags2& stage, VkAccessFlags2& access) {
    switch (layout) {
        case VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL:
            stage  = VK_PIPELINE_STAGE_2_COLOR_ATTACHMENT_OUTPUT_BIT;
            access = VK_ACCESS_2_COLOR_ATTACHMENT_WRITE_BIT; break;
        case VK_IMAGE_LAYOUT_DEPTH_ATTACHMENT_OPTIMAL:
            stage  = VK_PIPELINE_STAGE_2_EARLY_FRAGMENT_TESTS_BIT
                   | VK_PIPELINE_STAGE_2_LATE_FRAGMENT_TESTS_BIT;
            access = VK_ACCESS_2_DEPTH_STENCIL_ATTACHMENT_WRITE_BIT; break;
        case VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL:
            // Either shader stage may have sampled it — a compute pass reading a
            // G-buffer leaves the image here just as a fragment pass does.
            stage  = VK_PIPELINE_STAGE_2_FRAGMENT_SHADER_BIT
                   | VK_PIPELINE_STAGE_2_COMPUTE_SHADER_BIT;
            access = VK_ACCESS_2_SHADER_READ_BIT; break;
        case VK_IMAGE_LAYOUT_GENERAL:
            stage  = VK_PIPELINE_STAGE_2_COMPUTE_SHADER_BIT;
            access = VK_ACCESS_2_SHADER_STORAGE_WRITE_BIT; break;
        default:   // UNDEFINED and anything else: no prior work to wait on
            stage  = VK_PIPELINE_STAGE_2_TOP_OF_PIPE_BIT;
            access = 0; break;
    }
}

// Target layout + the scope of work that will use it after the barrier.
void StateTarget(RHITextureState state,
                 VkImageLayout& layout, VkPipelineStageFlags2& stage, VkAccessFlags2& access) {
    switch (state) {
        case RHITextureState::ColorTarget:
            layout = VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL;
            stage  = VK_PIPELINE_STAGE_2_COLOR_ATTACHMENT_OUTPUT_BIT;
            access = VK_ACCESS_2_COLOR_ATTACHMENT_WRITE_BIT; break;
        case RHITextureState::DepthTarget:
            layout = VK_IMAGE_LAYOUT_DEPTH_ATTACHMENT_OPTIMAL;
            stage  = VK_PIPELINE_STAGE_2_EARLY_FRAGMENT_TESTS_BIT
                   | VK_PIPELINE_STAGE_2_LATE_FRAGMENT_TESTS_BIT;
            access = VK_ACCESS_2_DEPTH_STENCIL_ATTACHMENT_WRITE_BIT
                   | VK_ACCESS_2_DEPTH_STENCIL_ATTACHMENT_READ_BIT; break;
        case RHITextureState::SampledRead:
            // Covers both consumers, so a compute pass can sample a texture the
            // graph transitioned without needing a second sampled state.
            layout = VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL;
            stage  = VK_PIPELINE_STAGE_2_FRAGMENT_SHADER_BIT
                   | VK_PIPELINE_STAGE_2_COMPUTE_SHADER_BIT;
            access = VK_ACCESS_2_SHADER_READ_BIT; break;
        case RHITextureState::Storage:
            // General is the only layout that permits imageLoad *and* imageStore,
            // so one state serves compute reads and writes alike.
            layout = VK_IMAGE_LAYOUT_GENERAL;
            stage  = VK_PIPELINE_STAGE_2_COMPUTE_SHADER_BIT;
            access = VK_ACCESS_2_SHADER_STORAGE_READ_BIT
                   | VK_ACCESS_2_SHADER_STORAGE_WRITE_BIT; break;
    }
}

// Transitions one frame slot of a texture to 'state', emitting the barrier and
// updating the texture's tracked layout.
void TransitionTextureSlot(VkCommandBuffer cmd, VulkanRHITexture* tex,
                           RHITextureState state, uint32_t frame) {
    VkImageLayout& current = tex->LayoutRef(frame);

    VkPipelineStageFlags2 srcStage; VkAccessFlags2 srcAccess;
    StageAccessForLayout(current, srcStage, srcAccess);

    VkImageLayout dstLayout; VkPipelineStageFlags2 dstStage; VkAccessFlags2 dstAccess;
    StateTarget(state, dstLayout, dstStage, dstAccess);

    TransitionImageLayout(cmd, tex->Image(frame), tex->Aspect(),
                          current, dstLayout, srcStage, srcAccess, dstStage, dstAccess);
    current = dstLayout;
}

} // namespace

void VulkanRHICommandList::TransitionTexture(RHITexture* texture, RHITextureState state) {
    TransitionTextureSlot(m_Cmd, static_cast<VulkanRHITexture*>(texture),
                          state, m_Device->CurrentFrame());
}

void VulkanRHICommandList::BeginRendering(const RHIRenderPass& pass) {
    const uint32_t frame = m_Device->CurrentFrame();

    std::vector<VkRenderingAttachmentInfo> colors;
    VkRenderingAttachmentInfo depth{ VK_STRUCTURE_TYPE_RENDERING_ATTACHMENT_INFO };
    bool       hasDepth = false;
    VkExtent2D extent{ 0, 0 };

    if (pass.toSwapchain) {
        extent = m_Device->SwapchainExtent();

        const RHIAttachment a = pass.colorAttachments.empty() ? RHIAttachment{}
                                                              : pass.colorAttachments[0];
        VkRenderingAttachmentInfo c{ VK_STRUCTURE_TYPE_RENDERING_ATTACHMENT_INFO };
        c.imageView   = m_Device->SwapchainImageView();   // already COLOR_ATTACHMENT (BeginFrame)
        c.imageLayout = VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL;
        c.loadOp      = a.clear ? VK_ATTACHMENT_LOAD_OP_CLEAR : VK_ATTACHMENT_LOAD_OP_LOAD;
        c.storeOp     = VK_ATTACHMENT_STORE_OP_STORE;
        c.clearValue.color = { { a.clearColor[0], a.clearColor[1], a.clearColor[2], a.clearColor[3] } };
        colors.push_back(c);

        if (pass.useDeviceDepth) {
            depth.imageView   = m_Device->DeviceDepthView();   // already DEPTH_ATTACHMENT (BeginFrame)
            depth.imageLayout = VK_IMAGE_LAYOUT_DEPTH_ATTACHMENT_OPTIMAL;
            depth.loadOp      = pass.clearDepth ? VK_ATTACHMENT_LOAD_OP_CLEAR : VK_ATTACHMENT_LOAD_OP_LOAD;
            depth.storeOp     = VK_ATTACHMENT_STORE_OP_DONT_CARE;
            depth.clearValue.depthStencil = { pass.depthClearValue, 0 };
            hasDepth = true;
        }
    } else {
        for (const RHIAttachment& a : pass.colorAttachments) {
            auto* vt = static_cast<VulkanRHITexture*>(a.texture);
            TransitionTextureSlot(m_Cmd, vt, RHITextureState::ColorTarget, frame);

            VkRenderingAttachmentInfo c{ VK_STRUCTURE_TYPE_RENDERING_ATTACHMENT_INFO };
            c.imageView   = vt->View(frame);
            c.imageLayout = VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL;
            c.loadOp      = a.clear ? VK_ATTACHMENT_LOAD_OP_CLEAR : VK_ATTACHMENT_LOAD_OP_LOAD;
            c.storeOp     = VK_ATTACHMENT_STORE_OP_STORE;
            c.clearValue.color = { { a.clearColor[0], a.clearColor[1], a.clearColor[2], a.clearColor[3] } };
            colors.push_back(c);
            extent = vt->Extent();
        }

        if (pass.depthTexture) {
            auto* vd = static_cast<VulkanRHITexture*>(pass.depthTexture);
            TransitionTextureSlot(m_Cmd, vd, RHITextureState::DepthTarget, frame);

            depth.imageView   = vd->View(frame);
            depth.imageLayout = VK_IMAGE_LAYOUT_DEPTH_ATTACHMENT_OPTIMAL;
            depth.loadOp      = pass.clearDepth ? VK_ATTACHMENT_LOAD_OP_CLEAR : VK_ATTACHMENT_LOAD_OP_LOAD;
            depth.storeOp     = VK_ATTACHMENT_STORE_OP_STORE;
            depth.clearValue.depthStencil = { pass.depthClearValue, 0 };
            hasDepth = true;
            if (extent.width == 0) extent = vd->Extent();
        }
    }

    VkRenderingInfo rendering{ VK_STRUCTURE_TYPE_RENDERING_INFO };
    rendering.renderArea.extent    = extent;
    rendering.layerCount           = 1;
    rendering.colorAttachmentCount = static_cast<uint32_t>(colors.size());
    rendering.pColorAttachments    = colors.data();
    rendering.pDepthAttachment     = hasDepth ? &depth : nullptr;
    vkCmdBeginRendering(m_Cmd, &rendering);

    // Y-flipped viewport so GL-style clip space maps to Vulkan, uniformly for
    // every target (offscreen and swapchain). Pass code never touches it.
    VkViewport viewport{};
    viewport.x        = 0.0f;
    viewport.y        = static_cast<float>(extent.height);
    viewport.width    = static_cast<float>(extent.width);
    viewport.height   = -static_cast<float>(extent.height);
    viewport.minDepth = 0.0f;
    viewport.maxDepth = 1.0f;
    vkCmdSetViewport(m_Cmd, 0, 1, &viewport);

    VkRect2D scissor{};
    scissor.extent = extent;
    vkCmdSetScissor(m_Cmd, 0, 1, &scissor);
}

void VulkanRHICommandList::EndRendering() {
    vkCmdEndRendering(m_Cmd);
}

// ── Device lifetime ──────────────────────────────────────────────────────────

VulkanRHIDevice::VulkanRHIDevice(GLFWwindow* window) : m_Window(window) {
    m_Ctx.Init(window);
    m_Swapchain.Init(&m_Ctx, window);
    CreateFrameResources();
    CreateDepthResources();
    CreateDescriptorPool();
    CreateTimestampPool();
#ifdef DIAMOND_TRACY
    CreateTracyVkContext();
#endif

    // Opt-in compute bring-up check; see VulkanComputeSelfTest.h. Last, so it
    // runs against a fully-constructed device.
    if (std::getenv("DIAMOND_COMPUTE_SELFTEST")) {
        RunComputeSelfTest(*this, DIAMOND_VULKAN_SHADER_DIR);
        RunGraphSelfTest(*this, DIAMOND_VULKAN_SHADER_DIR);
    }
}

VulkanRHIDevice::~VulkanRHIDevice() {
    vkDeviceWaitIdle(m_Ctx.Device());
#ifdef DIAMOND_TRACY
    if (m_TracyVkCtx) { TracyVkDestroy(m_TracyVkCtx); m_TracyVkCtx = nullptr; }
#endif
    DestroyTimestampPool();
    if (m_DescriptorPool) vkDestroyDescriptorPool(m_Ctx.Device(), m_DescriptorPool, nullptr);
    DestroyDepthResources();
    DestroyFrameResources();
    m_Swapchain.Destroy();
    m_Ctx.Shutdown();
}

// GPU frame timing (Docs/profiler-panel-design.md, Phase 2). Requires a
// graphics queue family with timestampValidBits set and a nonzero
// timestampPeriod; both hold on essentially all desktop GPUs, but if either
// is missing (unusual driver/hardware) the pool is left null and
// RendererStats::gpuFrameMs simply stays 0 (the panel already shows "n/a").
void VulkanRHIDevice::CreateTimestampPool() {
    VkQueueFamilyProperties familyProps{};
    uint32_t count = 0;
    vkGetPhysicalDeviceQueueFamilyProperties(m_Ctx.PhysicalDevice(), &count, nullptr);
    std::vector<VkQueueFamilyProperties> families(count);
    vkGetPhysicalDeviceQueueFamilyProperties(m_Ctx.PhysicalDevice(), &count, families.data());
    if (m_Ctx.Queues().graphics < families.size())
        familyProps = families[m_Ctx.Queues().graphics];

    if (familyProps.timestampValidBits == 0 ||
        m_Ctx.DeviceProperties().limits.timestampPeriod <= 0.0f) {
        return;
    }

    VkQueryPoolCreateInfo poolInfo{ VK_STRUCTURE_TYPE_QUERY_POOL_CREATE_INFO };
    poolInfo.queryType  = VK_QUERY_TYPE_TIMESTAMP;
    poolInfo.queryCount = kFramesInFlight * 2;
    VK_CHECK(vkCreateQueryPool(m_Ctx.Device(), &poolInfo, nullptr, &m_TimestampPool));
    m_TimestampsSupported = true;

    // Per-pass pool: a begin/end pair per profiled pass per slot.
    poolInfo.queryCount = kFramesInFlight * kMaxProfiledPasses * 2;
    VK_CHECK(vkCreateQueryPool(m_Ctx.Device(), &poolInfo, nullptr, &m_PassQueryPool));
}

#ifdef DIAMOND_TRACY
// Tracy GPU context — calibrates GPU↔CPU clocks by recording, submitting, and
// waiting a one-shot command buffer internally, beginning it several times.
// That needs a pool with RESET_COMMAND_BUFFER_BIT (implicit re-begin resets
// the buffer), which the frame pools deliberately lack, so lend Tracy a
// throwaway pool instead; it only touches the buffer during construction.
// Same support gate as our own timestamp pools.
void VulkanRHIDevice::CreateTracyVkContext() {
    if (!m_TimestampsSupported) return;

    VkDevice device = m_Ctx.Device();
    VkCommandPoolCreateInfo poolInfo{ VK_STRUCTURE_TYPE_COMMAND_POOL_CREATE_INFO };
    poolInfo.flags            = VK_COMMAND_POOL_CREATE_TRANSIENT_BIT |
                                VK_COMMAND_POOL_CREATE_RESET_COMMAND_BUFFER_BIT;
    poolInfo.queueFamilyIndex = m_Ctx.Queues().graphics;
    VkCommandPool tracyPool = VK_NULL_HANDLE;
    VK_CHECK(vkCreateCommandPool(device, &poolInfo, nullptr, &tracyPool));

    VkCommandBufferAllocateInfo allocInfo{ VK_STRUCTURE_TYPE_COMMAND_BUFFER_ALLOCATE_INFO };
    allocInfo.commandPool        = tracyPool;
    allocInfo.level              = VK_COMMAND_BUFFER_LEVEL_PRIMARY;
    allocInfo.commandBufferCount = 1;
    VkCommandBuffer tracyCmd = VK_NULL_HANDLE;
    VK_CHECK(vkAllocateCommandBuffers(device, &allocInfo, &tracyCmd));

    m_TracyVkCtx = TracyVkContext(m_Ctx.PhysicalDevice(), device,
                                  m_Ctx.GraphicsQueue(), tracyCmd);

    vkDestroyCommandPool(device, tracyPool, nullptr);
}
#endif

void VulkanRHIDevice::DestroyTimestampPool() {
    if (m_TimestampPool)  vkDestroyQueryPool(m_Ctx.Device(), m_TimestampPool, nullptr);
    if (m_PassQueryPool)  vkDestroyQueryPool(m_Ctx.Device(), m_PassQueryPool, nullptr);
    m_TimestampPool = VK_NULL_HANDLE;
    m_PassQueryPool = VK_NULL_HANDLE;
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

void VulkanRHIDevice::CreateDepthResources() {
    const VkExtent2D extent = m_Swapchain.Extent();
    for (VulkanImage& img : m_DepthImages)
        img = CreateImage(m_Ctx, extent.width, extent.height, kDepthFormat,
                          VK_IMAGE_USAGE_DEPTH_STENCIL_ATTACHMENT_BIT,
                          VK_IMAGE_ASPECT_DEPTH_BIT);
}

void VulkanRHIDevice::DestroyDepthResources() {
    for (VulkanImage& img : m_DepthImages) DestroyImage(m_Ctx, img);
}

void VulkanRHIDevice::CreateDescriptorPool() {
    // Sized for the pass sets plus per-material sets (each material = one set per
    // frame slot holding 6 samplers + 2 UBOs; the deferred-lighting set alone holds
    // 21 samplers per slot since the shadow-map bindings grew). FREE_DESCRIPTOR_SET
    // lets resource sets return their slots on destruction — without it, every
    // scene reload (editor stop-play restores the snapshot) permanently consumes
    // pool capacity until vkAllocateDescriptorSets fails with OUT_OF_POOL_MEMORY.
    // Sized for imported level scenes / stress tests: a Sponza-class import is
    // ~30 materials but a heavy scene can reach hundreds (each material = one
    // set per frame slot × 6 samplers + 2 UBOs). Pools are host-side and cheap;
    // running one dry aborts via VK_CHECK(OUT_OF_POOL_MEMORY) at draw time, so
    // headroom here is the difference between a stress test and a crash.
    // The storage sizes are the compute path's (GI probe/ray buffers, denoiser
    // targets): a handful of bindings per pass, not per material, so they need
    // nowhere near the sampled-texture headroom.
    const VkDescriptorPoolSize poolSizes[] = {
        { VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER,         4096 },
        { VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER, 16384 },
        { VK_DESCRIPTOR_TYPE_STORAGE_BUFFER,         512 },
        { VK_DESCRIPTOR_TYPE_STORAGE_IMAGE,          512 },
    };

    VkDescriptorPoolCreateInfo poolInfo{ VK_STRUCTURE_TYPE_DESCRIPTOR_POOL_CREATE_INFO };
    poolInfo.flags         = VK_DESCRIPTOR_POOL_CREATE_FREE_DESCRIPTOR_SET_BIT;
    poolInfo.maxSets       = 4096;
    poolInfo.poolSizeCount = static_cast<uint32_t>(std::size(poolSizes));
    poolInfo.pPoolSizes    = poolSizes;
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
    DestroyDepthResources();
    CreateDepthResources();   // resize the depth buffers to the new extent
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

std::unique_ptr<RHITexture> VulkanRHIDevice::CreateTexture(const RHITextureDesc& desc) {
    return std::make_unique<VulkanRHITexture>(this, desc);
}

std::unique_ptr<RHIPipeline> VulkanRHIDevice::CreatePipeline(const RHIPipelineDesc& desc) {
    return std::make_unique<VulkanRHIPipeline>(this, desc);
}

std::unique_ptr<RHIPipeline> VulkanRHIDevice::CreateComputePipeline(
    const RHIComputePipelineDesc& desc) {
    return std::make_unique<VulkanRHIPipeline>(this, desc);
}

std::unique_ptr<RHIResourceSet> VulkanRHIDevice::CreateResourceSet(
    RHIPipeline* pipeline, uint32_t setIndex,
    const std::vector<RHIBufferBinding>& buffers,
    const std::vector<RHITextureBinding>& textures) {
    return std::make_unique<VulkanRHIResourceSet>(
        this, static_cast<VulkanRHIPipeline*>(pipeline), setIndex, buffers, textures);
}

RHIFormat VulkanRHIDevice::SwapchainFormat() const {
    return FromVkFormat(m_Swapchain.ImageFormat());
}

RHIFormat VulkanRHIDevice::DepthFormat() const {
    return FromVkFormat(kDepthFormat);
}

// ── Frame loop ───────────────────────────────────────────────────────────────

RHICommandList* VulkanRHIDevice::BeginFrame() {
    if (m_Swapchain.IsZeroSize()) { RecreateSwapchain(); return nullptr; }

    VkDevice device = m_Ctx.Device();
    Frame&   frame  = m_Frames[m_CurrentFrame];

    VK_CHECK(vkWaitForFences(device, 1, &frame.inFlight, VK_TRUE, UINT64_MAX));

    // GPU frame timing: the fence wait above just proved slot m_CurrentFrame's
    // previous submission (kFramesInFlight frames ago) finished on the GPU, so
    // its timestamp results are ready with no polling needed.
    if (m_TimestampsSupported && m_TimestampValid[m_CurrentFrame]) {
        uint64_t ts[2] = {};
        VkResult qr = vkGetQueryPoolResults(device, m_TimestampPool, m_CurrentFrame * 2, 2,
                                            sizeof(ts), ts, sizeof(uint64_t),
                                            VK_QUERY_RESULT_64_BIT);
        if (qr == VK_SUCCESS) {
            double ns = double(ts[1] - ts[0]) * m_Ctx.DeviceProperties().limits.timestampPeriod;
            m_WorkingStats.gpuFrameMs = float(ns / 1.0e6);
        }
    }

    // Per-pass results for the same finished submission: resolve each recorded
    // pass's query pair, EMA-smooth (raw per-pass spans jitter frame to frame),
    // and publish into this frame's working stats. CPU-side fields (name, draw
    // counts, record time) were captured with the record, so a graph rebuilt
    // since then can't misattribute them.
    ResolvePassRecords(m_CurrentFrame);

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

    if (m_TimestampsSupported) {
        vkCmdResetQueryPool(cmd, m_TimestampPool, m_CurrentFrame * 2, 2);
        vkCmdWriteTimestamp2(cmd, VK_PIPELINE_STAGE_2_TOP_OF_PIPE_BIT,
                             m_TimestampPool, m_CurrentFrame * 2 + 0);
        // The slot's whole pass-query range: which pairs get written this frame
        // isn't known yet, and resetting unused queries is harmless.
        vkCmdResetQueryPool(cmd, m_PassQueryPool,
                            m_CurrentFrame * kMaxProfiledPasses * 2, kMaxProfiledPasses * 2);
    }

    // Move the backbuffer + the device depth buffer into their attachment layouts
    // up front (both discard via UNDEFINED; the swapchain pass clears or loads).
    // Rendering itself is opened by pass code via the command list, not here.
    TransitionImageLayout(cmd, m_Swapchain.Image(m_AcquiredImageIndex), VK_IMAGE_ASPECT_COLOR_BIT,
                          VK_IMAGE_LAYOUT_UNDEFINED, VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL,
                          VK_PIPELINE_STAGE_2_COLOR_ATTACHMENT_OUTPUT_BIT, 0,
                          VK_PIPELINE_STAGE_2_COLOR_ATTACHMENT_OUTPUT_BIT,
                          VK_ACCESS_2_COLOR_ATTACHMENT_WRITE_BIT);

    VulkanImage& depth = m_DepthImages[m_CurrentFrame];
    TransitionImageLayout(cmd, depth.image, VK_IMAGE_ASPECT_DEPTH_BIT,
                          VK_IMAGE_LAYOUT_UNDEFINED, VK_IMAGE_LAYOUT_DEPTH_ATTACHMENT_OPTIMAL,
                          VK_PIPELINE_STAGE_2_EARLY_FRAGMENT_TESTS_BIT
                              | VK_PIPELINE_STAGE_2_LATE_FRAGMENT_TESTS_BIT, 0,
                          VK_PIPELINE_STAGE_2_EARLY_FRAGMENT_TESTS_BIT
                              | VK_PIPELINE_STAGE_2_LATE_FRAGMENT_TESTS_BIT,
                          VK_ACCESS_2_DEPTH_STENCIL_ATTACHMENT_WRITE_BIT
                              | VK_ACCESS_2_DEPTH_STENCIL_ATTACHMENT_READ_BIT);

    m_CommandList.Reset(cmd);
    m_FrameActive = true;
    return &m_CommandList;
}

void VulkanRHIDevice::EndFrame() {
    if (!m_FrameActive) return;
    m_FrameActive = false;

    Frame&          frame = m_Frames[m_CurrentFrame];
    VkCommandBuffer cmd   = frame.commandBuffer;

    // The swapchain pass's EndRendering already closed the dynamic-rendering scope.
    TransitionImageLayout(cmd, m_Swapchain.Image(m_AcquiredImageIndex), VK_IMAGE_ASPECT_COLOR_BIT,
                          VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL, VK_IMAGE_LAYOUT_PRESENT_SRC_KHR,
                          VK_PIPELINE_STAGE_2_COLOR_ATTACHMENT_OUTPUT_BIT,
                          VK_ACCESS_2_COLOR_ATTACHMENT_WRITE_BIT,
                          VK_PIPELINE_STAGE_2_BOTTOM_OF_PIPE_BIT, 0);

    if (m_TimestampsSupported) {
        vkCmdWriteTimestamp2(cmd, VK_PIPELINE_STAGE_2_BOTTOM_OF_PIPE_BIT,
                             m_TimestampPool, m_CurrentFrame * 2 + 1);
        m_TimestampValid[m_CurrentFrame] = true;
    }

#ifdef DIAMOND_TRACY
    // Harvest this frame's Tracy GPU timestamps (Tracy owns its own query pool).
    if (m_TracyVkCtx) TracyVkCollect(m_TracyVkCtx, cmd);
#endif

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

// ── Frame stats ──────────────────────────────────────────────────────────────

void VulkanRHIDevice::ResetFrameStats() {
    m_WorkingStats = RendererStats{};
    m_MaterialsSeen.clear();
    m_TexturesSeen.clear();
}

void VulkanRHIDevice::RecordDraw(uint64_t triangleCount) {
    m_WorkingStats.drawCalls++;
    m_WorkingStats.trianglesSubmitted += triangleCount;
    if (m_ActivePass >= 0) {
        PassRecord& r = m_PassRecords[m_CurrentFrame][m_ActivePass];
        r.drawCalls++;
        r.triangles += triangleCount;
    }
}

void VulkanRHIDevice::RecordBufferUpload() { m_WorkingStats.bufferUploads++; }
void VulkanRHIDevice::RecordVisible(uint32_t count) { m_WorkingStats.visibleObjects += count; }
void VulkanRHIDevice::RecordCulled(uint32_t count)  { m_WorkingStats.culledObjects  += count; }
void VulkanRHIDevice::RecordShadowCaster() { m_WorkingStats.shadowCasters++; }
void VulkanRHIDevice::RecordMaterialBound(const void* material) { m_MaterialsSeen.insert(material); }
void VulkanRHIDevice::RecordTextureUsed(const void* texture)    { m_TexturesSeen.insert(texture); }

void VulkanRHIDevice::FinalizeFrameStats() {
    m_WorkingStats.materialsBound = (uint32_t)m_MaterialsSeen.size();
    m_WorkingStats.texturesUsed   = (uint32_t)m_TexturesSeen.size();

    // VRAM estimate (Docs/profiler-panel-design.md): VMA already tracks every allocation,
    // so this is just a heap-budget query, no bookkeeping needed. Device-local heaps are
    // the ones backing actual GPU memory (as opposed to host-visible upload heaps).
    VkPhysicalDeviceMemoryProperties memProps;
    vkGetPhysicalDeviceMemoryProperties(m_Ctx.PhysicalDevice(), &memProps);
    VmaBudget budgets[VK_MAX_MEMORY_HEAPS];
    vmaGetHeapBudgets(m_Ctx.Allocator(), budgets);
    uint64_t vramBytes = 0;
    for (uint32_t i = 0; i < memProps.memoryHeapCount; ++i)
        if (memProps.memoryHeaps[i].flags & VK_MEMORY_HEAP_DEVICE_LOCAL_BIT)
            vramBytes += budgets[i].usage;
    m_WorkingStats.vramBytes = vramBytes;

    m_SnapshotStats = m_WorkingStats;
}

// ── Per-pass profiling ───────────────────────────────────────────────────────

void VulkanRHIDevice::BeginPassProfile(const char* scope, const char* name,
                                       uint32_t width, uint32_t height) {
    if (!m_FrameActive) return;
    // No nesting: a Begin while a pass is open (or after a skipped Begin) is
    // ignored, and the depth counter makes its matching End a no-op too.
    if (m_ActivePass >= 0 || m_PassSkipDepth > 0) { m_PassSkipDepth++; return; }

    std::vector<PassRecord>& records = m_PassRecords[m_CurrentFrame];
    if (records.size() >= kMaxProfiledPasses) { m_PassSkipDepth++; return; }

    if (width == 0) {   // backbuffer-targeting pass
        width  = m_Swapchain.Extent().width;
        height = m_Swapchain.Extent().height;
    }

    PassRecord r;
    r.scope  = scope;
    r.name   = name;
    r.width  = width;
    r.height = height;
    records.push_back(std::move(r));
    m_ActivePass   = static_cast<int>(records.size()) - 1;
    m_PassCpuStart = std::chrono::steady_clock::now();

    if (m_TimestampsSupported) {
        const uint32_t q = m_CurrentFrame * kMaxProfiledPasses * 2
                         + static_cast<uint32_t>(m_ActivePass) * 2;
        vkCmdWriteTimestamp2(m_Frames[m_CurrentFrame].commandBuffer,
                             VK_PIPELINE_STAGE_2_TOP_OF_PIPE_BIT, m_PassQueryPool, q);
    }

#ifdef DIAMOND_TRACY
    // Mirror the pass onto Tracy's GPU track. Emplaced (not the stack macro)
    // because the zone spans Begin→EndPassProfile; the transient-name ctor
    // copies zoneName, so the stack buffer is fine.
    if (m_TracyVkCtx) {
        char zoneName[128];
        std::snprintf(zoneName, sizeof(zoneName), "%s/%s", scope, name);
        m_TracyPassZone.emplace(m_TracyVkCtx, (uint32_t)__LINE__,
                                __FILE__, strlen(__FILE__),
                                __FUNCTION__, strlen(__FUNCTION__),
                                zoneName, strlen(zoneName),
                                m_Frames[m_CurrentFrame].commandBuffer, true);
    }
#endif
}

void VulkanRHIDevice::EndPassProfile() {
    if (m_PassSkipDepth > 0) { m_PassSkipDepth--; return; }
    if (m_ActivePass < 0) return;

    PassRecord& r = m_PassRecords[m_CurrentFrame][m_ActivePass];
    r.cpuMs = std::chrono::duration<float, std::milli>(
                  std::chrono::steady_clock::now() - m_PassCpuStart).count();

    if (m_TimestampsSupported) {
        const uint32_t q = m_CurrentFrame * kMaxProfiledPasses * 2
                         + static_cast<uint32_t>(m_ActivePass) * 2 + 1;
        vkCmdWriteTimestamp2(m_Frames[m_CurrentFrame].commandBuffer,
                             VK_PIPELINE_STAGE_2_BOTTOM_OF_PIPE_BIT, m_PassQueryPool, q);
    }
#ifdef DIAMOND_TRACY
    m_TracyPassZone.reset();   // dtor writes the zone's end timestamp
#endif
    m_ActivePass = -1;
}

void VulkanRHIDevice::ResolvePassRecords(uint32_t slot) {
    std::vector<PassRecord>& records = m_PassRecords[slot];
    if (records.empty()) return;

    // GPU spans for the finished submission — one begin/end pair per record, in
    // record order. Read only the pairs actually written (the rest of the slot's
    // range was reset but never used).
    std::vector<uint64_t> ts(records.size() * 2, 0);
    bool haveGpuTimes = false;
    if (m_TimestampsSupported && m_TimestampValid[slot]) {
        VkResult qr = vkGetQueryPoolResults(
            m_Ctx.Device(), m_PassQueryPool, slot * kMaxProfiledPasses * 2,
            static_cast<uint32_t>(ts.size()), ts.size() * sizeof(uint64_t), ts.data(),
            sizeof(uint64_t), VK_QUERY_RESULT_64_BIT);
        haveGpuTimes = (qr == VK_SUCCESS);
    }
    const double period = m_Ctx.DeviceProperties().limits.timestampPeriod;

    for (size_t i = 0; i < records.size(); ++i) {
        PassRecord& r = records[i];
        const float gpuMs = haveGpuTimes
            ? float(double(ts[i * 2 + 1] - ts[i * 2]) * period / 1.0e6)
            : 0.0f;

        // EMA smoothing, keyed by scope/name so the same pass in two views keeps
        // separate histories. First sighting seeds with the raw sample.
        constexpr float kEma = 0.15f;
        auto [it, inserted] = m_PassSmoothed.try_emplace(r.scope + "/" + r.name,
                                                         gpuMs, r.cpuMs);
        if (!inserted) {
            it->second.first  += (gpuMs   - it->second.first)  * kEma;
            it->second.second += (r.cpuMs - it->second.second) * kEma;
        }

        PassStats ps;
        ps.scope     = std::move(r.scope);
        ps.name      = std::move(r.name);
        ps.gpuMs     = it->second.first;
        ps.cpuMs     = it->second.second;
        ps.drawCalls = r.drawCalls;
        ps.triangles = r.triangles;
        ps.width     = r.width;
        ps.height    = r.height;
        m_WorkingStats.passes.push_back(std::move(ps));
    }
    records.clear();
}

std::unique_ptr<RHIDevice> CreateVulkanRHIDevice(GLFWwindow* window) {
    return std::make_unique<VulkanRHIDevice>(window);
}

} // namespace Diamond
