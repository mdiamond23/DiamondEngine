#include "Platform/Vulkan/Passes/Shadows/VulkanPointShadowPass.h"
#include "Platform/Vulkan/Passes/VulkanPassCommon.h"
#include "Platform/Vulkan/RHI/VulkanRHIResources.h"
#include "Platform/Vulkan/VulkanContext.h"
#include "Renderer/RHI/RHIDevice.h"
#include "Renderer/RHI/RHICommandList.h"

#include <glm/gtc/matrix_transform.hpp>

#include <algorithm>

namespace Diamond {

namespace {

// The vertex buffer is the shared MeshVertex (pos/normal/uv/tangent, stride 44);
// the depth pass only reads position at location 0.
constexpr uint32_t kVertexStride = sizeof(glm::vec3) * 3 + sizeof(glm::vec2);  // 44

// Push block matching point_shadow.vert: per-draw model (offset 0, pushed by the
// caller's draw callback) + per-face index (offset 64, pushed here).
constexpr uint32_t kPushSize   = sizeof(glm::mat4) + sizeof(uint32_t);   // 68
constexpr uint32_t kFaceOffset = sizeof(glm::mat4);                      // 64

// synchronization2 barrier over a whole cube (all 6 layers, depth aspect).
void CubeDepthBarrier(VkCommandBuffer cmd, VkImage image,
                      VkImageLayout oldLayout, VkImageLayout newLayout,
                      VkPipelineStageFlags2 srcStage, VkAccessFlags2 srcAccess,
                      VkPipelineStageFlags2 dstStage, VkAccessFlags2 dstAccess) {
    VkImageMemoryBarrier2 b{ VK_STRUCTURE_TYPE_IMAGE_MEMORY_BARRIER_2 };
    b.srcStageMask  = srcStage;
    b.srcAccessMask = srcAccess;
    b.dstStageMask  = dstStage;
    b.dstAccessMask = dstAccess;
    b.oldLayout     = oldLayout;
    b.newLayout     = newLayout;
    b.image         = image;
    b.subresourceRange = { VK_IMAGE_ASPECT_DEPTH_BIT, 0, 1, 0, 6 };

    VkDependencyInfo dep{ VK_STRUCTURE_TYPE_DEPENDENCY_INFO };
    dep.imageMemoryBarrierCount = 1;
    dep.pImageMemoryBarriers    = &b;
    vkCmdPipelineBarrier2(cmd, &dep);
}

} // namespace

VulkanPointShadowPass::VulkanPointShadowPass(RHIDevice* device, const std::string& shaderDir)
    : m_Device(static_cast<VulkanRHIDevice*>(device))
{
    // ── Depth-only pipeline through the RHI (raw record binds it via the command
    // list, so the RHI's per-frame descriptor plumbing still applies).
    const std::vector<uint32_t> vs = LoadSpirv(shaderDir, "point_shadow.vert.spv");
    const std::vector<uint32_t> fs = LoadSpirv(shaderDir, "point_shadow.frag.spv");
    RHIShaderDesc vsDesc{ RHIShaderStage::Vertex,   vs.data(), vs.size() };
    RHIShaderDesc fsDesc{ RHIShaderStage::Fragment, fs.data(), fs.size() };
    m_Vert = device->CreateShader(vsDesc);
    m_Frag = device->CreateShader(fsDesc);

    RHIBufferDesc uboDesc;
    uboDesc.size    = sizeof(PointShadowUBO);
    uboDesc.usage   = RHIBufferUsage::Uniform;
    uboDesc.dynamic = true;
    m_UBO = device->CreateBuffer(uboDesc);

    RHIPipelineDesc desc;
    desc.vertexShader   = m_Vert.get();
    desc.fragmentShader = m_Frag.get();
    desc.vertexLayout.stride     = kVertexStride;
    desc.vertexLayout.attributes = { { 0, RHIVertexFormat::Float3, 0 } };   // position only
    desc.resourceBindings = { { 0, RHIResourceType::UniformBuffer, RHIShaderStage::Vertex } };
    desc.pushConstants = { RHIShaderStage::Vertex, kPushSize };
    desc.depthFormat = RHIFormat::Depth32F;
    desc.depthTest   = true;
    desc.depthWrite  = true;
    // No culling — matches the GL pass (with the 0.15 distance bias, front-culling
    // would let thin casters slip through).
    desc.cullMode = RHICullMode::None;
    m_Pipeline = device->CreatePipeline(desc);

    m_Set = device->CreateResourceSet(m_Pipeline.get(), 0, { { 0, m_UBO.get() } }, {});

    VulkanContext& ctx = m_Device->Ctx();
    VkDevice dev = ctx.Device();

    // ── Sampler: nearest + clamp, matching the GL cube shadow maps.
    {
        VkSamplerCreateInfo si{ VK_STRUCTURE_TYPE_SAMPLER_CREATE_INFO };
        si.magFilter    = VK_FILTER_NEAREST;
        si.minFilter    = VK_FILTER_NEAREST;
        si.mipmapMode   = VK_SAMPLER_MIPMAP_MODE_NEAREST;
        si.addressModeU = VK_SAMPLER_ADDRESS_MODE_CLAMP_TO_EDGE;
        si.addressModeV = VK_SAMPLER_ADDRESS_MODE_CLAMP_TO_EDGE;
        si.addressModeW = VK_SAMPLER_ADDRESS_MODE_CLAMP_TO_EDGE;
        VK_CHECK(vkCreateSampler(dev, &si, nullptr, &m_Sampler));
    }

    // ── One D32 cube per light per frame slot + a cube view and 6 face views each.
    for (auto& frameCubes : m_Cubes) {
        for (Cube& c : frameCubes) {
            VkImageCreateInfo ii{ VK_STRUCTURE_TYPE_IMAGE_CREATE_INFO };
            ii.flags         = VK_IMAGE_CREATE_CUBE_COMPATIBLE_BIT;
            ii.imageType     = VK_IMAGE_TYPE_2D;
            ii.format        = VulkanRHIDevice::kDepthFormat;
            ii.extent        = { kResolution, kResolution, 1 };
            ii.mipLevels     = 1;
            ii.arrayLayers   = 6;
            ii.samples       = VK_SAMPLE_COUNT_1_BIT;
            ii.tiling        = VK_IMAGE_TILING_OPTIMAL;
            ii.usage         = VK_IMAGE_USAGE_DEPTH_STENCIL_ATTACHMENT_BIT
                             | VK_IMAGE_USAGE_SAMPLED_BIT
                             | VK_IMAGE_USAGE_TRANSFER_DST_BIT;   // creation-time clear
            ii.sharingMode   = VK_SHARING_MODE_EXCLUSIVE;
            ii.initialLayout = VK_IMAGE_LAYOUT_UNDEFINED;

            VmaAllocationCreateInfo ai{};
            ai.usage = VMA_MEMORY_USAGE_AUTO;
            ai.flags = VMA_ALLOCATION_CREATE_DEDICATED_MEMORY_BIT;
            VK_CHECK(vmaCreateImage(ctx.Allocator(), &ii, &ai, &c.image, &c.alloc, nullptr));

            VkImageViewCreateInfo vi{ VK_STRUCTURE_TYPE_IMAGE_VIEW_CREATE_INFO };
            vi.image    = c.image;
            vi.viewType = VK_IMAGE_VIEW_TYPE_CUBE;
            vi.format   = VulkanRHIDevice::kDepthFormat;
            vi.subresourceRange = { VK_IMAGE_ASPECT_DEPTH_BIT, 0, 1, 0, 6 };
            VK_CHECK(vkCreateImageView(dev, &vi, nullptr, &c.cubeView));

            for (uint32_t f = 0; f < 6; ++f) {
                vi.viewType = VK_IMAGE_VIEW_TYPE_2D;
                vi.subresourceRange = { VK_IMAGE_ASPECT_DEPTH_BIT, 0, 1, f, 1 };
                VK_CHECK(vkCreateImageView(dev, &vi, nullptr, &c.faceViews[f]));
            }
        }
    }

    // ── Clear every cube to depth 1 (no occluder) and settle SHADER_READ_ONLY, so
    // slots that never render (fewer live lights) are still valid to sample.
    ctx.ImmediateSubmit([&](VkCommandBuffer cmd) {
        for (auto& frameCubes : m_Cubes) {
            for (Cube& c : frameCubes) {
                CubeDepthBarrier(cmd, c.image,
                                 VK_IMAGE_LAYOUT_UNDEFINED, VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL,
                                 VK_PIPELINE_STAGE_2_TOP_OF_PIPE_BIT, 0,
                                 VK_PIPELINE_STAGE_2_CLEAR_BIT, VK_ACCESS_2_TRANSFER_WRITE_BIT);
                const VkClearDepthStencilValue clear{ 1.0f, 0 };
                const VkImageSubresourceRange range{ VK_IMAGE_ASPECT_DEPTH_BIT, 0, 1, 0, 6 };
                vkCmdClearDepthStencilImage(cmd, c.image, VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL,
                                            &clear, 1, &range);
                CubeDepthBarrier(cmd, c.image,
                                 VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL,
                                 VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL,
                                 VK_PIPELINE_STAGE_2_CLEAR_BIT, VK_ACCESS_2_TRANSFER_WRITE_BIT,
                                 VK_PIPELINE_STAGE_2_FRAGMENT_SHADER_BIT, VK_ACCESS_2_SHADER_READ_BIT);
            }
        }
    });
}

VulkanPointShadowPass::~VulkanPointShadowPass()
{
    VulkanContext& ctx = m_Device->Ctx();
    VkDevice dev = ctx.Device();
    for (auto& frameCubes : m_Cubes) {
        for (Cube& c : frameCubes) {
            for (VkImageView v : c.faceViews)
                if (v) vkDestroyImageView(dev, v, nullptr);
            if (c.cubeView) vkDestroyImageView(dev, c.cubeView, nullptr);
            if (c.image)    vmaDestroyImage(ctx.Allocator(), c.image, c.alloc);
            c = {};
        }
    }
    if (m_Sampler) vkDestroySampler(dev, m_Sampler, nullptr);
}

void VulkanPointShadowPass::SetLights(const std::vector<glm::vec3>& positionsWorld)
{
    m_Count = std::min(static_cast<int>(positionsWorld.size()), MAX_LIGHTS);

    // The same face set the GL pass and the IBL bake use (LearnOpenGL convention;
    // the negative-height viewport in Record supplies the Y-flip), so a world-space
    // direction samples the cube exactly like GL. Explicit _ZO per the backend rule.
    PointShadowUBO ubo{};
    const glm::mat4 proj = glm::perspectiveRH_ZO(glm::radians(90.0f), 1.0f, 0.1f, kFarPlane);
    for (int i = 0; i < m_Count; ++i) {
        const glm::vec3& lp = positionsWorld[i];
        const std::array<glm::mat4, 6> views = {
            glm::lookAt(lp, lp + glm::vec3( 1, 0, 0), glm::vec3(0, -1,  0)),
            glm::lookAt(lp, lp + glm::vec3(-1, 0, 0), glm::vec3(0, -1,  0)),
            glm::lookAt(lp, lp + glm::vec3( 0, 1, 0), glm::vec3(0,  0,  1)),
            glm::lookAt(lp, lp + glm::vec3( 0,-1, 0), glm::vec3(0,  0, -1)),
            glm::lookAt(lp, lp + glm::vec3( 0, 0, 1), glm::vec3(0, -1,  0)),
            glm::lookAt(lp, lp + glm::vec3( 0, 0,-1), glm::vec3(0, -1,  0)),
        };
        for (int f = 0; f < 6; ++f)
            ubo.faceVP[i * 6 + f] = proj * views[f];
        ubo.lightPosFar[i] = glm::vec4(lp, kFarPlane);
    }
    m_UBO->Update(&ubo, sizeof(ubo));
}

void VulkanPointShadowPass::Record(RHICommandList* cmd,
                                   const std::function<void(RHICommandList*)>& drawScene)
{
    if (m_Count == 0) return;

    VkCommandBuffer raw   = static_cast<VulkanRHICommandList*>(cmd)->Handle();
    const uint32_t  frame = m_Device->CurrentFrame();

    for (int i = 0; i < m_Count; ++i) {
        Cube& c = m_Cubes[frame][i];

        // Whole cube → depth attachment. oldLayout UNDEFINED discards the previous
        // contents (every face is cleared below); the frame fence already retired
        // this slot's last use, so no execution dependency is needed.
        CubeDepthBarrier(raw, c.image,
                         VK_IMAGE_LAYOUT_UNDEFINED, VK_IMAGE_LAYOUT_DEPTH_ATTACHMENT_OPTIMAL,
                         VK_PIPELINE_STAGE_2_TOP_OF_PIPE_BIT, 0,
                         VK_PIPELINE_STAGE_2_EARLY_FRAGMENT_TESTS_BIT
                             | VK_PIPELINE_STAGE_2_LATE_FRAGMENT_TESTS_BIT,
                         VK_ACCESS_2_DEPTH_STENCIL_ATTACHMENT_WRITE_BIT);

        for (uint32_t f = 0; f < 6; ++f) {
            VkRenderingAttachmentInfo depth{ VK_STRUCTURE_TYPE_RENDERING_ATTACHMENT_INFO };
            depth.imageView   = c.faceViews[f];
            depth.imageLayout = VK_IMAGE_LAYOUT_DEPTH_ATTACHMENT_OPTIMAL;
            depth.loadOp      = VK_ATTACHMENT_LOAD_OP_CLEAR;
            depth.storeOp     = VK_ATTACHMENT_STORE_OP_STORE;
            depth.clearValue.depthStencil = { 1.0f, 0 };

            VkRenderingInfo ri{ VK_STRUCTURE_TYPE_RENDERING_INFO };
            ri.renderArea.extent = { kResolution, kResolution };
            ri.layerCount        = 1;
            ri.pDepthAttachment  = &depth;
            vkCmdBeginRendering(raw, &ri);

            // Negative-height viewport — the same Y-flip every device render scope
            // uses, keeping the face orientation aligned with the IBL bake.
            VkViewport vp{};
            vp.x = 0.0f; vp.y = static_cast<float>(kResolution);
            vp.width    = static_cast<float>(kResolution);
            vp.height   = -static_cast<float>(kResolution);
            vp.minDepth = 0.0f; vp.maxDepth = 1.0f;
            vkCmdSetViewport(raw, 0, 1, &vp);
            VkRect2D sc{}; sc.extent = { kResolution, kResolution };
            vkCmdSetScissor(raw, 0, 1, &sc);

            cmd->BindPipeline(m_Pipeline.get());
            cmd->BindResourceSet(0, m_Set.get());
            const uint32_t idx = static_cast<uint32_t>(i) * 6u + f;
            cmd->PushConstants(RHIShaderStage::Vertex, kFaceOffset, sizeof(idx), &idx);
            drawScene(cmd);

            vkCmdEndRendering(raw);
        }

        CubeDepthBarrier(raw, c.image,
                         VK_IMAGE_LAYOUT_DEPTH_ATTACHMENT_OPTIMAL,
                         VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL,
                         VK_PIPELINE_STAGE_2_LATE_FRAGMENT_TESTS_BIT,
                         VK_ACCESS_2_DEPTH_STENCIL_ATTACHMENT_WRITE_BIT,
                         VK_PIPELINE_STAGE_2_FRAGMENT_SHADER_BIT, VK_ACCESS_2_SHADER_READ_BIT);
    }
}

} // namespace Diamond
