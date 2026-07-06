#include "Platform/Vulkan/Resources/VulkanThumbnailRenderer.h"
#include "Platform/Vulkan/Resources/VulkanTexture2D.h"
#include "Platform/Vulkan/RHI/VulkanRHIResources.h"
#include "Platform/Vulkan/VulkanContext.h"
#include "Platform/Vulkan/Passes/VulkanPassCommon.h"
#include "Renderer/RHI/RHIDevice.h"
#include "Renderer/MeshData.h"
#include "Renderer/Material.h"

#include <glm/gtc/matrix_transform.hpp>

namespace Diamond {

namespace {

constexpr RHIFormat kColorFormat = RHIFormat::RGBA8;
constexpr RHIFormat kDepthFormat = RHIFormat::Depth32F;

// The shared mesh vertex layout every Vulkan geometry pass uses (SceneRenderer's
// MeshVertex): position + normal + UV + tangent, stride 44.
struct MeshVertex {
    glm::vec3 pos;
    glm::vec3 normal;
    glm::vec2 uv;
    glm::vec3 tangent;
};

std::unique_ptr<RHIShader> MakeShader(RHIDevice* device, const std::string& dir,
                                      const std::string& name, RHIShaderStage stage) {
    const std::vector<uint32_t> code = LoadSpirv(dir, name);
    RHIShaderDesc desc{ stage, code.data(), code.size() };
    return device->CreateShader(desc);
}

// Resolve an engine texture to its RHI handle, or null when the slot is empty
// (or not Vulkan-backed) — the null drives both the UBO has-flag and the
// neutral-dummy binding.
RHITexture* RhiOf(const std::shared_ptr<Texture>& tex) {
    if (const auto* vk = dynamic_cast<const VulkanTexture2D*>(tex.get()); vk && vk->Rhi())
        return vk->Rhi();
    return nullptr;
}

} // namespace

VulkanThumbnailRenderer::VulkanThumbnailRenderer(RHIDevice* device, const std::string& shaderDir)
    : m_Device(static_cast<VulkanRHIDevice*>(device))
{
    // ── Studio mesh pipeline (thumbnail.vert/frag) ─────────────────────────────
    m_MeshVert = MakeShader(device, shaderDir, "thumbnail.vert.spv", RHIShaderStage::Vertex);
    m_MeshFrag = MakeShader(device, shaderDir, "thumbnail.frag.spv", RHIShaderStage::Fragment);
    {
        RHIPipelineDesc desc;
        desc.vertexShader        = m_MeshVert.get();
        desc.fragmentShader      = m_MeshFrag.get();
        desc.vertexLayout.stride = sizeof(MeshVertex);
        desc.vertexLayout.attributes = {
            { 0, RHIVertexFormat::Float3, offsetof(MeshVertex, pos)    },
            { 1, RHIVertexFormat::Float3, offsetof(MeshVertex, normal) },
        };
        desc.resourceBindings = {
            { 0, RHIResourceType::UniformBuffer, RHIShaderStage::Vertex | RHIShaderStage::Fragment },
        };
        desc.colorFormat = kColorFormat;
        desc.depthFormat = kDepthFormat;
        desc.depthTest   = true;
        desc.depthWrite  = true;
        desc.cullMode    = RHICullMode::None;   // matches the GL thumbnail FBO state
        m_MeshPipeline = device->CreatePipeline(desc);
    }

    // ── Material-ball pipeline (material_preview.vert/frag) ────────────────────
    m_MatVert = MakeShader(device, shaderDir, "material_preview.vert.spv", RHIShaderStage::Vertex);
    m_MatFrag = MakeShader(device, shaderDir, "material_preview.frag.spv", RHIShaderStage::Fragment);
    {
        RHIPipelineDesc desc;
        desc.vertexShader        = m_MatVert.get();
        desc.fragmentShader      = m_MatFrag.get();
        desc.vertexLayout.stride = sizeof(MeshVertex);
        desc.vertexLayout.attributes = {
            { 0, RHIVertexFormat::Float3, offsetof(MeshVertex, pos)    },
            { 1, RHIVertexFormat::Float3, offsetof(MeshVertex, normal) },
            { 2, RHIVertexFormat::Float2, offsetof(MeshVertex, uv)     },
        };
        desc.resourceBindings = {
            { 0, RHIResourceType::UniformBuffer,        RHIShaderStage::Vertex | RHIShaderStage::Fragment },
            { 1, RHIResourceType::CombinedImageSampler, RHIShaderStage::Fragment },   // albedo
            { 2, RHIResourceType::CombinedImageSampler, RHIShaderStage::Fragment },   // metallic
            { 3, RHIResourceType::CombinedImageSampler, RHIShaderStage::Fragment },   // roughness
            { 4, RHIResourceType::CombinedImageSampler, RHIShaderStage::Fragment },   // ao
            { 5, RHIResourceType::CombinedImageSampler, RHIShaderStage::Fragment },   // emissive
        };
        desc.colorFormat = kColorFormat;
        desc.depthFormat = kDepthFormat;
        desc.depthTest   = true;
        desc.depthWrite  = true;
        desc.cullMode    = RHICullMode::None;
        m_MatPipeline = device->CreatePipeline(desc);
    }

    // Shared depth target + output sampler.
    VulkanContext& ctx = m_Device->Ctx();
    m_Depth = CreateImage(ctx, kSize, kSize, VK_FORMAT_D32_SFLOAT,
                          VK_IMAGE_USAGE_DEPTH_STENCIL_ATTACHMENT_BIT,
                          VK_IMAGE_ASPECT_DEPTH_BIT);

    VkSamplerCreateInfo si{ VK_STRUCTURE_TYPE_SAMPLER_CREATE_INFO };
    si.magFilter    = VK_FILTER_LINEAR;
    si.minFilter    = VK_FILTER_LINEAR;
    si.mipmapMode   = VK_SAMPLER_MIPMAP_MODE_NEAREST;
    si.addressModeU = VK_SAMPLER_ADDRESS_MODE_CLAMP_TO_EDGE;
    si.addressModeV = VK_SAMPLER_ADDRESS_MODE_CLAMP_TO_EDGE;
    si.addressModeW = VK_SAMPLER_ADDRESS_MODE_CLAMP_TO_EDGE;
    VK_CHECK(vkCreateSampler(ctx.Device(), &si, nullptr, &m_Sampler));

    // 1×1 neutral maps for unassigned material slots.
    auto makePixel = [device](uint8_t r, uint8_t g, uint8_t b) {
        const uint8_t px[4] = { r, g, b, 255 };
        RHITextureDesc one;
        one.width       = 1;
        one.height      = 1;
        one.format      = RHIFormat::RGBA8;
        one.usage       = RHITextureUsage::Sampled;
        one.initialData = px;
        return device->CreateTexture(one);
    };
    m_White = makePixel(255, 255, 255);
    m_Black = makePixel(0, 0, 0);
    m_Gray  = makePixel(128, 128, 128);

    // The preview sphere, shared by every material ball.
    const MeshData sphere = MeshData::UVSphere();
    const AABB     aabb   = sphere.ComputeAABB();
    m_SphereMin = aabb.min;
    m_SphereMax = aabb.max;
    m_Sphere    = UploadMesh(sphere);
}

VulkanThumbnailRenderer::~VulkanThumbnailRenderer() {
    // Nothing may still be sampling the thumbs (or running a bake) while the
    // images die. Cheap: thumbnails are editor-lifetime objects.
    m_Device->WaitIdle();
    VulkanContext& ctx = m_Device->Ctx();
    for (VulkanImage& t : m_Thumbs) DestroyImage(ctx, t);
    DestroyImage(ctx, m_Depth);
    vkDestroySampler(ctx.Device(), m_Sampler, nullptr);
}

void VulkanThumbnailRenderer::FrameAABB(const glm::vec3& aabbMin, const glm::vec3& aabbMax,
                                        bool pickAxis, ThumbUBO& ubo) {
    const glm::vec3 center = (aabbMin + aabbMax) * 0.5f;
    float radius = glm::length(aabbMax - aabbMin) * 0.5f;
    if (radius < 1e-5f) radius = 1.0f;

    // Face the smallest AABB axis so the largest face of the bounding box fills
    // the thumbnail (mesh previews); material balls use the fixed front view.
    glm::vec3 camDir = glm::normalize(glm::vec3(0.4f, 0.5f, 1.0f));
    if (pickAxis) {
        const glm::vec3 ext = aabbMax - aabbMin;
        if (ext.x <= ext.y && ext.x <= ext.z)
            camDir = glm::normalize(glm::vec3(1.0f, 0.5f, 0.4f));   // thin in X → from the side
        else if (ext.y <= ext.x && ext.y <= ext.z)
            camDir = glm::normalize(glm::vec3(0.5f, 1.0f, 0.5f));   // thin in Y → from above
        else
            camDir = glm::normalize(glm::vec3(0.4f, 0.5f, 1.0f));   // thin in Z → from the front
    }
    const glm::vec3 camPos = center + camDir * (radius * 2.8f);

    ubo.model      = glm::mat4(1.0f);
    ubo.view       = glm::lookAt(camPos, center, glm::vec3(0.0f, 1.0f, 0.0f));
    // Explicitly the [0,1]-depth variant (see SceneRenderer: plain glm::perspective
    // COMDAT-folds across TUs and a GL TU's [-1,1] version can win).
    ubo.projection = glm::perspectiveRH_ZO(glm::radians(45.0f), 1.0f,
                                           radius * 0.01f, radius * 20.0f);
    ubo.cameraPos  = glm::vec4(camPos, 1.0f);
}

VulkanThumbnailRenderer::DrawRange VulkanThumbnailRenderer::UploadMesh(const MeshData& data) {
    std::vector<MeshVertex> verts;
    verts.reserve(data.Vertices.size());
    for (const Vertex& v : data.Vertices)
        verts.push_back({ v.Position, v.Normal, v.TexCoords, v.Tangent });

    RHIBufferDesc vb;
    vb.size        = verts.size() * sizeof(MeshVertex);
    vb.usage       = RHIBufferUsage::Vertex;
    vb.initialData = verts.data();
    auto vertexBuffer = m_Device->CreateBuffer(vb);

    RHIBufferDesc ib;
    ib.size        = data.Indices.size() * sizeof(uint32_t);
    ib.usage       = RHIBufferUsage::Index;
    ib.initialData = data.Indices.data();
    auto indexBuffer = m_Device->CreateBuffer(ib);

    const DrawRange range{ vertexBuffer.get(), indexBuffer.get(),
                           static_cast<uint32_t>(data.Indices.size()) };
    m_Buffers.push_back(std::move(vertexBuffer));
    m_Buffers.push_back(std::move(indexBuffer));
    return range;
}

VulkanThumbnailRenderer::Thumb VulkanThumbnailRenderer::RenderMesh(
    const std::vector<MeshData>& meshes) {
    if (meshes.empty()) return {};

    // Aggregate AABB across all sub-meshes.
    AABB aabb;
    for (const MeshData& md : meshes) {
        const AABB b = md.ComputeAABB();
        aabb.min = glm::min(aabb.min, b.min);
        aabb.max = glm::max(aabb.max, b.max);
    }

    ThumbUBO ubo{};
    FrameAABB(aabb.min, aabb.max, /*pickAxis*/ true, ubo);

    RHIBufferDesc ub;
    ub.size        = sizeof(ThumbUBO);
    ub.usage       = RHIBufferUsage::Uniform;
    ub.initialData = &ubo;
    auto uboBuffer = m_Device->CreateBuffer(ub);

    auto set = m_Device->CreateResourceSet(m_MeshPipeline.get(), 0,
                                           { { 0, uboBuffer.get() } }, {});

    std::vector<DrawRange> draws;
    draws.reserve(meshes.size());
    for (const MeshData& md : meshes)
        draws.push_back(UploadMesh(md));

    const Thumb thumb = Bake(m_MeshPipeline.get(), set.get(), draws);
    m_Buffers.push_back(std::move(uboBuffer));
    m_Sets.push_back(std::move(set));
    return thumb;
}

VulkanThumbnailRenderer::Thumb VulkanThumbnailRenderer::RenderMaterial(const PBRMaterial& mat) {
    ThumbUBO ubo{};
    FrameAABB(m_SphereMin, m_SphereMax, /*pickAxis*/ false, ubo);

    RHITexture* albedo    = RhiOf(mat.Albedo);
    RHITexture* metallic  = RhiOf(mat.Metallic);
    RHITexture* roughness = RhiOf(mat.Roughness);
    RHITexture* ao        = RhiOf(mat.AO);
    RHITexture* emissive  = RhiOf(mat.Emissive);

    // GL parity: scalar fallbacks 0.8-gray albedo / metallic 0 / roughness 0.5,
    // emissive contributes only when a map is bound.
    ubo.albedoFallback = { 0.8f, 0.8f, 0.8f, mat.UVScale };
    ubo.scalars        = { 0.0f, 0.5f, emissive ? mat.EmissiveStrength : 0.0f, 0.0f };
    ubo.hasMaps        = { albedo ? 1.0f : 0.0f, metallic ? 1.0f : 0.0f,
                           roughness ? 1.0f : 0.0f, ao ? 1.0f : 0.0f };
    ubo.hasMaps2       = { emissive ? 1.0f : 0.0f, 0.0f, 0.0f, 0.0f };

    RHIBufferDesc ub;
    ub.size        = sizeof(ThumbUBO);
    ub.usage       = RHIBufferUsage::Uniform;
    ub.initialData = &ubo;
    auto uboBuffer = m_Device->CreateBuffer(ub);

    auto set = m_Device->CreateResourceSet(m_MatPipeline.get(), 0,
        { { 0, uboBuffer.get() } },
        { { 1, albedo    ? albedo    : m_White.get() },
          { 2, metallic  ? metallic  : m_Black.get() },
          { 3, roughness ? roughness : m_Gray.get()  },
          { 4, ao        ? ao        : m_White.get() },
          { 5, emissive  ? emissive  : m_Black.get() } });

    const Thumb thumb = Bake(m_MatPipeline.get(), set.get(), { m_Sphere });
    m_Buffers.push_back(std::move(uboBuffer));
    m_Sets.push_back(std::move(set));
    return thumb;
}

VulkanThumbnailRenderer::Thumb VulkanThumbnailRenderer::Bake(
    RHIPipeline* pipeline, RHIResourceSet* set, const std::vector<DrawRange>& draws) {
    VulkanContext& ctx = m_Device->Ctx();

    VulkanImage color = CreateImage(
        ctx, kSize, kSize, VK_FORMAT_R8G8B8A8_UNORM,
        VK_IMAGE_USAGE_COLOR_ATTACHMENT_BIT | VK_IMAGE_USAGE_SAMPLED_BIT,
        VK_IMAGE_ASPECT_COLOR_BIT);

    auto* pipe = static_cast<VulkanRHIPipeline*>(pipeline);
    auto* rset = static_cast<VulkanRHIResourceSet*>(set);

    ctx.ImmediateSubmit([&](VkCommandBuffer cmd) {
        TransitionImageLayout(cmd, color.image, VK_IMAGE_ASPECT_COLOR_BIT,
                              VK_IMAGE_LAYOUT_UNDEFINED,
                              VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL,
                              VK_PIPELINE_STAGE_2_TOP_OF_PIPE_BIT, VK_ACCESS_2_NONE,
                              VK_PIPELINE_STAGE_2_COLOR_ATTACHMENT_OUTPUT_BIT,
                              VK_ACCESS_2_COLOR_ATTACHMENT_WRITE_BIT);
        // UNDEFINED discards last bake's depth — exactly what a fresh clear wants.
        TransitionImageLayout(cmd, m_Depth.image, VK_IMAGE_ASPECT_DEPTH_BIT,
                              VK_IMAGE_LAYOUT_UNDEFINED,
                              VK_IMAGE_LAYOUT_DEPTH_ATTACHMENT_OPTIMAL,
                              VK_PIPELINE_STAGE_2_TOP_OF_PIPE_BIT, VK_ACCESS_2_NONE,
                              VK_PIPELINE_STAGE_2_EARLY_FRAGMENT_TESTS_BIT |
                                  VK_PIPELINE_STAGE_2_LATE_FRAGMENT_TESTS_BIT,
                              VK_ACCESS_2_DEPTH_STENCIL_ATTACHMENT_READ_BIT |
                                  VK_ACCESS_2_DEPTH_STENCIL_ATTACHMENT_WRITE_BIT);

        VkRenderingAttachmentInfo colorAtt{ VK_STRUCTURE_TYPE_RENDERING_ATTACHMENT_INFO };
        colorAtt.imageView   = color.view;
        colorAtt.imageLayout = VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL;
        colorAtt.loadOp      = VK_ATTACHMENT_LOAD_OP_CLEAR;
        colorAtt.storeOp     = VK_ATTACHMENT_STORE_OP_STORE;
        colorAtt.clearValue.color = { { 0.15f, 0.15f, 0.18f, 1.0f } };   // GL clear color

        VkRenderingAttachmentInfo depthAtt{ VK_STRUCTURE_TYPE_RENDERING_ATTACHMENT_INFO };
        depthAtt.imageView   = m_Depth.view;
        depthAtt.imageLayout = VK_IMAGE_LAYOUT_DEPTH_ATTACHMENT_OPTIMAL;
        depthAtt.loadOp      = VK_ATTACHMENT_LOAD_OP_CLEAR;
        depthAtt.storeOp     = VK_ATTACHMENT_STORE_OP_DONT_CARE;
        depthAtt.clearValue.depthStencil = { 1.0f, 0 };

        VkRenderingInfo rendering{ VK_STRUCTURE_TYPE_RENDERING_INFO };
        rendering.renderArea           = { { 0, 0 }, { kSize, kSize } };
        rendering.layerCount           = 1;
        rendering.colorAttachmentCount = 1;
        rendering.pColorAttachments    = &colorAtt;
        rendering.pDepthAttachment     = &depthAtt;
        vkCmdBeginRendering(cmd, &rendering);

        // The engine-wide Y-flipped viewport (GL-style clip space → Vulkan).
        VkViewport viewport{};
        viewport.y        = static_cast<float>(kSize);
        viewport.width    = static_cast<float>(kSize);
        viewport.height   = -static_cast<float>(kSize);
        viewport.maxDepth = 1.0f;
        vkCmdSetViewport(cmd, 0, 1, &viewport);
        VkRect2D scissor{ { 0, 0 }, { kSize, kSize } };
        vkCmdSetScissor(cmd, 0, 1, &scissor);

        vkCmdBindPipeline(cmd, VK_PIPELINE_BIND_POINT_GRAPHICS, pipe->Handle());
        // Frame slot 0: every bound resource here is static, so all slots alias.
        VkDescriptorSet ds = rset->Handle(0);
        vkCmdBindDescriptorSets(cmd, VK_PIPELINE_BIND_POINT_GRAPHICS, pipe->Layout(),
                                0, 1, &ds, 0, nullptr);

        for (const DrawRange& d : draws) {
            VkBuffer     vb  = static_cast<VulkanRHIBuffer*>(d.vertexBuffer)->Handle(0);
            VkDeviceSize off = 0;
            vkCmdBindVertexBuffers(cmd, 0, 1, &vb, &off);
            vkCmdBindIndexBuffer(cmd, static_cast<VulkanRHIBuffer*>(d.indexBuffer)->Handle(0),
                                 0, VK_INDEX_TYPE_UINT32);
            vkCmdDrawIndexed(cmd, d.indexCount, 1, 0, 0, 0);
        }

        vkCmdEndRendering(cmd);

        TransitionImageLayout(cmd, color.image, VK_IMAGE_ASPECT_COLOR_BIT,
                              VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL,
                              VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL,
                              VK_PIPELINE_STAGE_2_COLOR_ATTACHMENT_OUTPUT_BIT,
                              VK_ACCESS_2_COLOR_ATTACHMENT_WRITE_BIT,
                              VK_PIPELINE_STAGE_2_FRAGMENT_SHADER_BIT,
                              VK_ACCESS_2_SHADER_SAMPLED_READ_BIT);
    });

    m_Thumbs.push_back(color);
    return { color.view, m_Sampler };
}

} // namespace Diamond
