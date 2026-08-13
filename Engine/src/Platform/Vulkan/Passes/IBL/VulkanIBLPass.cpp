#include "Platform/Vulkan/Passes/IBL/VulkanIBLPass.h"
#include "Platform/Vulkan/Passes/VulkanPassCommon.h"
#include "Platform/Vulkan/RHI/VulkanRHIDevice.h"
#include "Platform/Vulkan/RHI/VulkanRHIResources.h"
#include "Platform/Vulkan/VulkanContext.h"
#include "Platform/Vulkan/VulkanImage.h"
#include "Platform/Vulkan/VulkanBuffer.h"
#include "Renderer/RHI/RHIDevice.h"
#include "Renderer/RHI/RHIResources.h"
#include "Renderer/MeshData.h"
#include "Assets/ImageLoader.h"

// Match the rest of the Vulkan backend: clip-space depth in [0,1]. Irrelevant to
// the bake (no depth attachment) but keeps GLM consistent across the backend.
#define GLM_FORCE_DEPTH_ZERO_TO_ONE
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/packing.hpp>   // packHalf1x16 for the equirect upload
#include <spdlog/spdlog.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstring>
#include <functional>
#include <memory>
#include <vector>

namespace Diamond {

namespace {

// All baked targets are RGBA16F: the cube faces hold HDR radiance, and the BRDF
// LUT stores its scale+bias in .rg (so no RG16F format is needed anywhere).
constexpr VkFormat kFormat = VK_FORMAT_R16G16B16A16_SFLOAT;

// Back to 512: the skybox now displays the equirect directly, so this only
// feeds the irradiance (32^2) and prefilter (128^2) bakes and DDGI's miss
// shader. None of those resolve anything a larger cube would give them.
constexpr uint32_t kEnvSize       = 512;
constexpr uint32_t kIrradianceSz  = 32;
constexpr uint32_t kPrefilterSz   = 128;
constexpr uint32_t kPrefilterMips = 5;
constexpr uint32_t kBrdfSize      = 512;

// std430-free push block shared by the cube-capture pipelines (see cubemap.vert).
struct CubePush {
    glm::mat4 viewProj;
    float     roughness;
};

uint32_t MipCount(uint32_t size) {
    return static_cast<uint32_t>(std::floor(std::log2(static_cast<float>(size)))) + 1;
}

// synchronization2 barrier over an explicit mip/layer subresource range (the
// shared TransitionImageLayout only covers a single mip/layer — cubemaps and the
// mip chain need ranges).
void CubeBarrier(VkCommandBuffer cmd, VkImage image, VkImageAspectFlags aspect,
                 uint32_t baseMip, uint32_t mipCount, uint32_t baseLayer, uint32_t layerCount,
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
    b.subresourceRange = { aspect, baseMip, mipCount, baseLayer, layerCount };

    VkDependencyInfo dep{ VK_STRUCTURE_TYPE_DEPENDENCY_INFO };
    dep.imageMemoryBarrierCount = 1;
    dep.pImageMemoryBarriers    = &b;
    vkCmdPipelineBarrier2(cmd, &dep);
}

} // namespace

VulkanIBLPass::VulkanIBLPass(RHIDevice* device, std::string shaderDir)
    : m_Device(static_cast<VulkanRHIDevice*>(device))
    , m_ShaderDir(std::move(shaderDir)) {}

// Release everything BakeEnvironment allocates. Also called at the TOP of a
// re-bake: every handle below was overwritten in place by the old code, so
// switching environments leaked the whole previous set.
void VulkanIBLPass::DestroyBakedResources() {
    VulkanContext& ctx = m_Device->Ctx();
    VkDevice dev = ctx.Device();
    auto destroyCube = [&](Cubemap& c) {
        if (c.cubeView) vkDestroyImageView(dev, c.cubeView, nullptr);
        if (c.image)    vmaDestroyImage(ctx.Allocator(), c.image, c.alloc);
        c = {};
    };
    destroyCube(m_EnvCube);
    destroyCube(m_Irradiance);
    destroyCube(m_Prefilter);
    if (m_BrdfLUT.image)  DestroyImage(ctx, m_BrdfLUT);
    if (m_Equirect.image) DestroyImage(ctx, m_Equirect);
    m_BrdfLUT  = {};
    m_Equirect = {};
    if (m_Sampler)         vkDestroySampler(dev, m_Sampler, nullptr);
    if (m_EquirectSampler) vkDestroySampler(dev, m_EquirectSampler, nullptr);
    m_Sampler         = VK_NULL_HANDLE;
    m_EquirectSampler = VK_NULL_HANDLE;
}

VulkanIBLPass::~VulkanIBLPass() { DestroyBakedResources(); }

void VulkanIBLPass::BakeEnvironment(const std::string& hdrPath) {
    VulkanContext& ctx = m_Device->Ctx();
    VkDevice dev = ctx.Device();

    // A re-bake (environment swap) replaces every handle below. Frames in flight
    // may still sample the old ones through descriptor sets that haven't been
    // rewritten yet, so drain first — this is a rare editor action.
    m_Device->WaitIdle();
    DestroyBakedResources();

    // ── Shared sampler (linear, mipmapped, clamp) for the bake sources + baked maps.
    {
        VkSamplerCreateInfo si{ VK_STRUCTURE_TYPE_SAMPLER_CREATE_INFO };
        si.magFilter    = VK_FILTER_LINEAR;
        si.minFilter    = VK_FILTER_LINEAR;
        si.mipmapMode   = VK_SAMPLER_MIPMAP_MODE_LINEAR;
        si.addressModeU = VK_SAMPLER_ADDRESS_MODE_CLAMP_TO_EDGE;
        si.addressModeV = VK_SAMPLER_ADDRESS_MODE_CLAMP_TO_EDGE;
        si.addressModeW = VK_SAMPLER_ADDRESS_MODE_CLAMP_TO_EDGE;
        si.maxLod       = VK_LOD_CLAMP_NONE;
        VK_CHECK(vkCreateSampler(dev, &si, nullptr, &m_Sampler));

        // Same, but u REPEATs: longitude wraps, so clamping leaves a hard seam
        // down the ±180° meridian where bilinear can't blend across it. v still
        // clamps — latitude genuinely ends at the poles.
        si.addressModeU = VK_SAMPLER_ADDRESS_MODE_REPEAT;
        VK_CHECK(vkCreateSampler(dev, &si, nullptr, &m_EquirectSampler));
    }

    // ── Load the equirectangular HDR → an R16G16B16A16_SFLOAT 2D source image ─────
    // stb returns 3- or 4-channel float; expand to RGBA and pack to half. Half
    // was full float until the skybox started displaying this image directly —
    // it now lives for the renderer's lifetime, and 16F halves that to ~67 MB at
    // 4k. Sky radiance peaks in the hundreds, nowhere near half's 65504 limit.
    FloatImageData hdr = ImageLoader::LoadFloat(hdrPath, /*flipVertically*/ true);
    if (hdr.Pixels.empty()) {
        spdlog::error("[IBL] failed to load HDR environment '{}'", hdrPath);
        return;
    }
    const uint32_t ew = static_cast<uint32_t>(hdr.Width);
    const uint32_t eh = static_cast<uint32_t>(hdr.Height);
    std::vector<uint16_t> rgba(static_cast<size_t>(ew) * eh * 4);
    for (size_t i = 0; i < static_cast<size_t>(ew) * eh; ++i) {
        const int   c = hdr.Channels;
        const float* src = &hdr.Pixels[i * c];
        rgba[i * 4 + 0] = glm::packHalf1x16(src[0]);
        rgba[i * 4 + 1] = glm::packHalf1x16(c > 1 ? src[1] : src[0]);
        rgba[i * 4 + 2] = glm::packHalf1x16(c > 2 ? src[2] : src[0]);
        rgba[i * 4 + 3] = glm::packHalf1x16(1.0f);
    }

    VulkanImage equirect = CreateImage(ctx, ew, eh, kFormat,
                                       VK_IMAGE_USAGE_SAMPLED_BIT | VK_IMAGE_USAGE_TRANSFER_DST_BIT,
                                       VK_IMAGE_ASPECT_COLOR_BIT);
    {
        const VkDeviceSize bytes = rgba.size() * sizeof(uint16_t);
        VmaAllocationInfo stagingInfo{};
        VulkanBuffer staging = CreateBuffer(
            ctx.Allocator(), bytes, VK_BUFFER_USAGE_TRANSFER_SRC_BIT, VMA_MEMORY_USAGE_AUTO,
            VMA_ALLOCATION_CREATE_HOST_ACCESS_SEQUENTIAL_WRITE_BIT | VMA_ALLOCATION_CREATE_MAPPED_BIT,
            &stagingInfo);
        std::memcpy(stagingInfo.pMappedData, rgba.data(), static_cast<size_t>(bytes));

        ctx.ImmediateSubmit([&](VkCommandBuffer cmd) {
            TransitionImageLayout(cmd, equirect.image, VK_IMAGE_ASPECT_COLOR_BIT,
                                  VK_IMAGE_LAYOUT_UNDEFINED, VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL,
                                  VK_PIPELINE_STAGE_2_TOP_OF_PIPE_BIT, 0,
                                  VK_PIPELINE_STAGE_2_COPY_BIT, VK_ACCESS_2_TRANSFER_WRITE_BIT);
            VkBufferImageCopy copy{};
            copy.imageSubresource.aspectMask = VK_IMAGE_ASPECT_COLOR_BIT;
            copy.imageSubresource.layerCount = 1;
            copy.imageExtent = { ew, eh, 1 };
            vkCmdCopyBufferToImage(cmd, staging.handle, equirect.image,
                                   VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL, 1, &copy);
            TransitionImageLayout(cmd, equirect.image, VK_IMAGE_ASPECT_COLOR_BIT,
                                  VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL, VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL,
                                  VK_PIPELINE_STAGE_2_COPY_BIT, VK_ACCESS_2_TRANSFER_WRITE_BIT,
                                  VK_PIPELINE_STAGE_2_FRAGMENT_SHADER_BIT, VK_ACCESS_2_SHADER_READ_BIT);
        });
        DestroyBuffer(ctx.Allocator(), staging);
    }

    // ── Create the cubemaps + BRDF target ────────────────────────────────────────
    auto createCubemap = [&](uint32_t size, uint32_t mips) {
        Cubemap c;
        c.size = size; c.mipLevels = mips; c.format = kFormat;

        VkImageCreateInfo ii{ VK_STRUCTURE_TYPE_IMAGE_CREATE_INFO };
        ii.flags         = VK_IMAGE_CREATE_CUBE_COMPATIBLE_BIT;
        ii.imageType     = VK_IMAGE_TYPE_2D;
        ii.format        = kFormat;
        ii.extent        = { size, size, 1 };
        ii.mipLevels     = mips;
        ii.arrayLayers   = 6;
        ii.samples       = VK_SAMPLE_COUNT_1_BIT;
        ii.tiling        = VK_IMAGE_TILING_OPTIMAL;
        ii.usage         = VK_IMAGE_USAGE_COLOR_ATTACHMENT_BIT | VK_IMAGE_USAGE_SAMPLED_BIT
                         | VK_IMAGE_USAGE_TRANSFER_SRC_BIT     | VK_IMAGE_USAGE_TRANSFER_DST_BIT;
        ii.sharingMode   = VK_SHARING_MODE_EXCLUSIVE;
        ii.initialLayout = VK_IMAGE_LAYOUT_UNDEFINED;

        VmaAllocationCreateInfo ai{};
        ai.usage = VMA_MEMORY_USAGE_AUTO;
        ai.flags = VMA_ALLOCATION_CREATE_DEDICATED_MEMORY_BIT;
        VK_CHECK(vmaCreateImage(ctx.Allocator(), &ii, &ai, &c.image, &c.alloc, nullptr));

        VkImageViewCreateInfo vi{ VK_STRUCTURE_TYPE_IMAGE_VIEW_CREATE_INFO };
        vi.image    = c.image;
        vi.viewType = VK_IMAGE_VIEW_TYPE_CUBE;
        vi.format   = kFormat;
        vi.subresourceRange = { VK_IMAGE_ASPECT_COLOR_BIT, 0, mips, 0, 6 };
        VK_CHECK(vkCreateImageView(dev, &vi, nullptr, &c.cubeView));
        return c;
    };

    m_EnvCube    = createCubemap(kEnvSize, MipCount(kEnvSize));
    m_Irradiance = createCubemap(kIrradianceSz, 1);
    m_Prefilter  = createCubemap(kPrefilterSz, kPrefilterMips);
    m_BrdfLUT    = CreateImage(ctx, kBrdfSize, kBrdfSize, kFormat,
                               VK_IMAGE_USAGE_COLOR_ATTACHMENT_BIT | VK_IMAGE_USAGE_SAMPLED_BIT,
                               VK_IMAGE_ASPECT_COLOR_BIT);

    // ── Unit-cube geometry (position only — cubemap.vert reads location 0) ────────
    const MeshData cubeMesh = MeshData::UnitCube();
    std::vector<glm::vec3> positions;
    positions.reserve(cubeMesh.Vertices.size());
    for (const Vertex& v : cubeMesh.Vertices) positions.push_back(v.Position);
    const std::vector<uint32_t>& indices = cubeMesh.Indices;
    const uint32_t indexCount = static_cast<uint32_t>(indices.size());

    VulkanBuffer cubeVB = CreateDeviceLocalBuffer(ctx, positions.data(),
        positions.size() * sizeof(glm::vec3), VK_BUFFER_USAGE_VERTEX_BUFFER_BIT);
    VulkanBuffer cubeIB = CreateDeviceLocalBuffer(ctx, indices.data(),
        indices.size() * sizeof(uint32_t), VK_BUFFER_USAGE_INDEX_BUFFER_BIT);

    // ── Pipelines (built through the RHI; reach the Vulkan handles for raw record) ─
    auto loadShader = [&](const char* file, RHIShaderStage stage) {
        const std::vector<uint32_t> code = LoadSpirv(m_ShaderDir, file);
        RHIShaderDesc d{ stage, code.data(), code.size() };
        return m_Device->CreateShader(d);
    };
    auto cubeVS  = loadShader("cubemap.vert.spv",                 RHIShaderStage::Vertex);
    auto fsVS    = loadShader("fullscreen.vert.spv",              RHIShaderStage::Vertex);
    auto eqFS    = loadShader("equirect_to_cubemap.frag.spv",     RHIShaderStage::Fragment);
    auto irrFS   = loadShader("irradiance_convolution.frag.spv",  RHIShaderStage::Fragment);
    auto preFS   = loadShader("prefilter.frag.spv",               RHIShaderStage::Fragment);
    auto brdfFS  = loadShader("brdf_lut.frag.spv",                RHIShaderStage::Fragment);

    auto makeCubePipeline = [&](RHIShader* frag) {
        RHIPipelineDesc d;
        d.vertexShader   = cubeVS.get();
        d.fragmentShader = frag;
        d.vertexLayout.stride = sizeof(glm::vec3);
        d.vertexLayout.attributes = { { 0, RHIVertexFormat::Float3, 0 } };
        d.resourceBindings = { { 0, RHIResourceType::CombinedImageSampler, RHIShaderStage::Fragment } };
        d.pushConstants = { RHIShaderStage::Vertex | RHIShaderStage::Fragment, sizeof(CubePush) };
        d.cullMode    = RHICullMode::None;
        d.colorFormat = RHIFormat::RGBA16F;
        return m_Device->CreatePipeline(d);
    };
    auto equirectPipe  = makeCubePipeline(eqFS.get());
    auto irradiancePipe = makeCubePipeline(irrFS.get());
    auto prefilterPipe = makeCubePipeline(preFS.get());

    std::unique_ptr<RHIPipeline> brdfPipe;
    {
        RHIPipelineDesc d;
        d.vertexShader   = fsVS.get();
        d.fragmentShader = brdfFS.get();
        d.colorFormat    = RHIFormat::RGBA16F;   // result in .rg
        brdfPipe = m_Device->CreatePipeline(d);
    }

    auto vkPipe = [](RHIPipeline* p) { return static_cast<VulkanRHIPipeline*>(p); };

    // ── Descriptor sets: equirect (2D source) + env cube (irradiance/prefilter src).
    auto allocImageSet = [&](VkDescriptorSetLayout layout, VkImageView view) {
        VkDescriptorSet set = VK_NULL_HANDLE;
        VkDescriptorSetAllocateInfo ai{ VK_STRUCTURE_TYPE_DESCRIPTOR_SET_ALLOCATE_INFO };
        ai.descriptorPool     = m_Device->DescriptorPool();
        ai.descriptorSetCount = 1;
        ai.pSetLayouts        = &layout;
        VK_CHECK(vkAllocateDescriptorSets(dev, &ai, &set));

        VkDescriptorImageInfo info{};
        info.sampler     = m_Sampler;
        info.imageView   = view;
        info.imageLayout = VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL;
        VkWriteDescriptorSet w{ VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET };
        w.dstSet          = set;
        w.dstBinding      = 0;
        w.descriptorCount = 1;
        w.descriptorType  = VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER;
        w.pImageInfo      = &info;
        vkUpdateDescriptorSets(dev, 1, &w, 0, nullptr);
        return set;
    };
    VkDescriptorSet equirectSet = allocImageSet(vkPipe(equirectPipe.get())->SetLayout(), equirect.view);
    VkDescriptorSet envCubeSet  = allocImageSet(vkPipe(irradiancePipe.get())->SetLayout(), m_EnvCube.cubeView);

    // ── Per-face capture matrices (LearnOpenGL convention; Y-flip handled by the
    //    negative-height viewport, matching the rest of the backend) ──────────────
    const glm::mat4 proj = glm::perspectiveRH_ZO(glm::radians(90.0f), 1.0f, 0.1f, 10.0f);
    const std::array<glm::mat4, 6> views = {
        glm::lookAt(glm::vec3(0.0f), glm::vec3( 1, 0, 0), glm::vec3(0, -1,  0)),
        glm::lookAt(glm::vec3(0.0f), glm::vec3(-1, 0, 0), glm::vec3(0, -1,  0)),
        glm::lookAt(glm::vec3(0.0f), glm::vec3( 0, 1, 0), glm::vec3(0,  0,  1)),
        glm::lookAt(glm::vec3(0.0f), glm::vec3( 0,-1, 0), glm::vec3(0,  0, -1)),
        glm::lookAt(glm::vec3(0.0f), glm::vec3( 0, 0, 1), glm::vec3(0, -1,  0)),
        glm::lookAt(glm::vec3(0.0f), glm::vec3( 0, 0,-1), glm::vec3(0, -1,  0)),
    };

    // Transient per-face/per-mip render views — created here, used inside the
    // submit, destroyed after it retires (ImmediateSubmit blocks, so this is safe).
    std::vector<VkImageView> transient;
    auto faceView = [&](VkImage image, uint32_t mip, uint32_t face) {
        VkImageViewCreateInfo vi{ VK_STRUCTURE_TYPE_IMAGE_VIEW_CREATE_INFO };
        vi.image    = image;
        vi.viewType = VK_IMAGE_VIEW_TYPE_2D;
        vi.format   = kFormat;
        vi.subresourceRange = { VK_IMAGE_ASPECT_COLOR_BIT, mip, 1, face, 1 };
        VkImageView v = VK_NULL_HANDLE;
        VK_CHECK(vkCreateImageView(dev, &vi, nullptr, &v));
        transient.push_back(v);
        return v;
    };

    ctx.ImmediateSubmit([&](VkCommandBuffer cmd) {
        // Begin a single-attachment dynamic-rendering scope into 'view', flip the
        // viewport like the device does, and run 'draw'.
        auto renderTo = [&](VkImageView view, uint32_t size, const std::function<void()>& draw) {
            VkRenderingAttachmentInfo color{ VK_STRUCTURE_TYPE_RENDERING_ATTACHMENT_INFO };
            color.imageView   = view;
            color.imageLayout = VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL;
            color.loadOp      = VK_ATTACHMENT_LOAD_OP_CLEAR;
            color.storeOp     = VK_ATTACHMENT_STORE_OP_STORE;
            color.clearValue.color = { { 0.0f, 0.0f, 0.0f, 1.0f } };

            VkRenderingInfo ri{ VK_STRUCTURE_TYPE_RENDERING_INFO };
            ri.renderArea.extent    = { size, size };
            ri.layerCount           = 1;
            ri.colorAttachmentCount = 1;
            ri.pColorAttachments    = &color;
            vkCmdBeginRendering(cmd, &ri);

            VkViewport vp{};
            vp.x = 0.0f; vp.y = static_cast<float>(size);
            vp.width = static_cast<float>(size); vp.height = -static_cast<float>(size);
            vp.minDepth = 0.0f; vp.maxDepth = 1.0f;
            vkCmdSetViewport(cmd, 0, 1, &vp);
            VkRect2D sc{}; sc.extent = { size, size };
            vkCmdSetScissor(cmd, 0, 1, &sc);

            draw();
            vkCmdEndRendering(cmd);
        };

        // Render the 6 cube faces of 'cube' at 'mip' with a capture pipeline.
        auto renderCubeFaces = [&](Cubemap& cube, uint32_t mip, RHIPipeline* pipe,
                                   VkDescriptorSet set, float roughness) {
            const uint32_t mipSize = cube.size >> mip;
            VkPipelineLayout layout = vkPipe(pipe)->Layout();
            for (uint32_t face = 0; face < 6; ++face) {
                CubePush push{ proj * views[face], roughness };
                renderTo(faceView(cube.image, mip, face), mipSize, [&]() {
                    vkCmdBindPipeline(cmd, VK_PIPELINE_BIND_POINT_GRAPHICS, vkPipe(pipe)->Handle());
                    vkCmdBindDescriptorSets(cmd, VK_PIPELINE_BIND_POINT_GRAPHICS, layout, 0, 1, &set, 0, nullptr);
                    vkCmdPushConstants(cmd, layout,
                        VK_SHADER_STAGE_VERTEX_BIT | VK_SHADER_STAGE_FRAGMENT_BIT,
                        0, sizeof(CubePush), &push);
                    const VkDeviceSize off = 0;
                    vkCmdBindVertexBuffers(cmd, 0, 1, &cubeVB.handle, &off);
                    vkCmdBindIndexBuffer(cmd, cubeIB.handle, 0, VK_INDEX_TYPE_UINT32);
                    vkCmdDrawIndexed(cmd, indexCount, 1, 0, 0, 0);
                });
            }
        };

        // 1. Equirect → env cubemap (mip 0).
        CubeBarrier(cmd, m_EnvCube.image, VK_IMAGE_ASPECT_COLOR_BIT, 0, 1, 0, 6,
                    VK_IMAGE_LAYOUT_UNDEFINED, VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL,
                    VK_PIPELINE_STAGE_2_TOP_OF_PIPE_BIT, 0,
                    VK_PIPELINE_STAGE_2_COLOR_ATTACHMENT_OUTPUT_BIT, VK_ACCESS_2_COLOR_ATTACHMENT_WRITE_BIT);
        renderCubeFaces(m_EnvCube, 0, equirectPipe.get(), equirectSet, 0.0f);

        // 2. Generate the env cubemap mip chain (blit down), leaving it SHADER_READ.
        {
            const uint32_t mips = m_EnvCube.mipLevels;
            // mip 0: COLOR → TRANSFER_SRC; mips 1..n-1: UNDEFINED → TRANSFER_DST.
            CubeBarrier(cmd, m_EnvCube.image, VK_IMAGE_ASPECT_COLOR_BIT, 0, 1, 0, 6,
                        VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL, VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL,
                        VK_PIPELINE_STAGE_2_COLOR_ATTACHMENT_OUTPUT_BIT, VK_ACCESS_2_COLOR_ATTACHMENT_WRITE_BIT,
                        VK_PIPELINE_STAGE_2_BLIT_BIT, VK_ACCESS_2_TRANSFER_READ_BIT);
            CubeBarrier(cmd, m_EnvCube.image, VK_IMAGE_ASPECT_COLOR_BIT, 1, mips - 1, 0, 6,
                        VK_IMAGE_LAYOUT_UNDEFINED, VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL,
                        VK_PIPELINE_STAGE_2_TOP_OF_PIPE_BIT, 0,
                        VK_PIPELINE_STAGE_2_BLIT_BIT, VK_ACCESS_2_TRANSFER_WRITE_BIT);

            int32_t mw = static_cast<int32_t>(m_EnvCube.size);
            int32_t mh = mw;
            for (uint32_t i = 1; i < mips; ++i) {
                const int32_t pw = mw, ph = mh;
                mw = std::max(1, mw / 2); mh = std::max(1, mh / 2);

                VkImageBlit blit{};
                blit.srcSubresource = { VK_IMAGE_ASPECT_COLOR_BIT, i - 1, 0, 6 };
                blit.srcOffsets[1]  = { pw, ph, 1 };
                blit.dstSubresource = { VK_IMAGE_ASPECT_COLOR_BIT, i, 0, 6 };
                blit.dstOffsets[1]  = { mw, mh, 1 };
                vkCmdBlitImage(cmd, m_EnvCube.image, VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL,
                               m_EnvCube.image, VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL,
                               1, &blit, VK_FILTER_LINEAR);
                // This mip becomes the source for the next iteration.
                CubeBarrier(cmd, m_EnvCube.image, VK_IMAGE_ASPECT_COLOR_BIT, i, 1, 0, 6,
                            VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL, VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL,
                            VK_PIPELINE_STAGE_2_BLIT_BIT, VK_ACCESS_2_TRANSFER_WRITE_BIT,
                            VK_PIPELINE_STAGE_2_BLIT_BIT, VK_ACCESS_2_TRANSFER_READ_BIT);
            }
            // All mips are TRANSFER_SRC now → SHADER_READ for the convolutions.
            CubeBarrier(cmd, m_EnvCube.image, VK_IMAGE_ASPECT_COLOR_BIT, 0, mips, 0, 6,
                        VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL, VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL,
                        VK_PIPELINE_STAGE_2_BLIT_BIT, VK_ACCESS_2_TRANSFER_READ_BIT,
                        VK_PIPELINE_STAGE_2_FRAGMENT_SHADER_BIT, VK_ACCESS_2_SHADER_READ_BIT);
        }

        // 3. Irradiance convolution (1 mip), sampling the env cube.
        CubeBarrier(cmd, m_Irradiance.image, VK_IMAGE_ASPECT_COLOR_BIT, 0, 1, 0, 6,
                    VK_IMAGE_LAYOUT_UNDEFINED, VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL,
                    VK_PIPELINE_STAGE_2_TOP_OF_PIPE_BIT, 0,
                    VK_PIPELINE_STAGE_2_COLOR_ATTACHMENT_OUTPUT_BIT, VK_ACCESS_2_COLOR_ATTACHMENT_WRITE_BIT);
        renderCubeFaces(m_Irradiance, 0, irradiancePipe.get(), envCubeSet, 0.0f);
        CubeBarrier(cmd, m_Irradiance.image, VK_IMAGE_ASPECT_COLOR_BIT, 0, 1, 0, 6,
                    VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL, VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL,
                    VK_PIPELINE_STAGE_2_COLOR_ATTACHMENT_OUTPUT_BIT, VK_ACCESS_2_COLOR_ATTACHMENT_WRITE_BIT,
                    VK_PIPELINE_STAGE_2_FRAGMENT_SHADER_BIT, VK_ACCESS_2_SHADER_READ_BIT);

        // 4. Prefilter — one roughness per mip, all sampling the env cube.
        CubeBarrier(cmd, m_Prefilter.image, VK_IMAGE_ASPECT_COLOR_BIT, 0, m_Prefilter.mipLevels, 0, 6,
                    VK_IMAGE_LAYOUT_UNDEFINED, VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL,
                    VK_PIPELINE_STAGE_2_TOP_OF_PIPE_BIT, 0,
                    VK_PIPELINE_STAGE_2_COLOR_ATTACHMENT_OUTPUT_BIT, VK_ACCESS_2_COLOR_ATTACHMENT_WRITE_BIT);
        for (uint32_t mip = 0; mip < m_Prefilter.mipLevels; ++mip) {
            const float roughness = static_cast<float>(mip) / static_cast<float>(m_Prefilter.mipLevels - 1);
            renderCubeFaces(m_Prefilter, mip, prefilterPipe.get(), envCubeSet, roughness);
        }
        CubeBarrier(cmd, m_Prefilter.image, VK_IMAGE_ASPECT_COLOR_BIT, 0, m_Prefilter.mipLevels, 0, 6,
                    VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL, VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL,
                    VK_PIPELINE_STAGE_2_COLOR_ATTACHMENT_OUTPUT_BIT, VK_ACCESS_2_COLOR_ATTACHMENT_WRITE_BIT,
                    VK_PIPELINE_STAGE_2_FRAGMENT_SHADER_BIT, VK_ACCESS_2_SHADER_READ_BIT);

        // 5. BRDF LUT — fullscreen integration into the 2D target.
        CubeBarrier(cmd, m_BrdfLUT.image, VK_IMAGE_ASPECT_COLOR_BIT, 0, 1, 0, 1,
                    VK_IMAGE_LAYOUT_UNDEFINED, VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL,
                    VK_PIPELINE_STAGE_2_TOP_OF_PIPE_BIT, 0,
                    VK_PIPELINE_STAGE_2_COLOR_ATTACHMENT_OUTPUT_BIT, VK_ACCESS_2_COLOR_ATTACHMENT_WRITE_BIT);
        renderTo(m_BrdfLUT.view, kBrdfSize, [&]() {
            vkCmdBindPipeline(cmd, VK_PIPELINE_BIND_POINT_GRAPHICS, vkPipe(brdfPipe.get())->Handle());
            vkCmdDraw(cmd, 3, 1, 0, 0);
        });
        CubeBarrier(cmd, m_BrdfLUT.image, VK_IMAGE_ASPECT_COLOR_BIT, 0, 1, 0, 1,
                    VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL, VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL,
                    VK_PIPELINE_STAGE_2_COLOR_ATTACHMENT_OUTPUT_BIT, VK_ACCESS_2_COLOR_ATTACHMENT_WRITE_BIT,
                    VK_PIPELINE_STAGE_2_FRAGMENT_SHADER_BIT, VK_ACCESS_2_SHADER_READ_BIT);
    });

    // ── Cleanup transient bake resources (submit has retired) ─────────────────────
    for (VkImageView v : transient) vkDestroyImageView(dev, v, nullptr);
    // The equirect is NOT transient any more — the skybox displays it directly.
    // Ownership moves to m_Equirect here, after the bake submit has retired.
    m_Equirect = equirect;
    DestroyBuffer(ctx.Allocator(), cubeVB);
    DestroyBuffer(ctx.Allocator(), cubeIB);
    // Pipelines/shaders release as their unique_ptrs leave scope. The bake
    // descriptor sets stay allocated from the device pool until shutdown (no
    // per-set free path yet) — negligible for a handful of one-time sets.

    spdlog::info("[IBL] baked environment from '{}' ({}x{} equirect → env {}^2 +mips, "
                 "irradiance {}^2, prefilter {}^2 x{} mips, brdf {}^2)",
                 hdrPath, ew, eh, kEnvSize, kIrradianceSz, kPrefilterSz, kPrefilterMips, kBrdfSize);
}

} // namespace Diamond
