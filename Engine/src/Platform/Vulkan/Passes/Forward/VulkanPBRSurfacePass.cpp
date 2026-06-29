#include "Platform/Vulkan/Passes/Forward/VulkanPBRSurfacePass.h"
#include "Platform/Vulkan/Passes/VulkanPassCommon.h"
#include "Platform/Vulkan/Passes/IBL/VulkanIBLPass.h"
#include "Platform/Vulkan/RHI/VulkanRHIDevice.h"
#include "Platform/Vulkan/RHI/VulkanRHIResources.h"
#include "Renderer/RHI/RHIDevice.h"
#include "Renderer/RHI/RHICommandList.h"

#include <array>
#include <utility>

namespace Diamond {

namespace {
constexpr RHIFormat kHDRFormat   = RHIFormat::RGBA16F;
constexpr RHIFormat kDepthFormat = RHIFormat::Depth32F;
// Shared interleaved mesh vertex: position + normal + UV (matches the demo's
// MeshVertex). The skybox + copy pipelines read a subset of these attributes.
constexpr uint32_t kVertexStride = sizeof(glm::vec3) * 2 + sizeof(glm::vec2);  // 32

struct ForwardPush { glm::mat4 model; };

std::unique_ptr<RHIShader> MakeShader(RHIDevice* device, const std::string& dir,
                                      const std::string& name, RHIShaderStage stage) {
    const std::vector<uint32_t> code = LoadSpirv(dir, name);
    RHIShaderDesc desc{ stage, code.data(), code.size() };
    return device->CreateShader(desc);
}
} // namespace

VulkanPBRSurfacePass::VulkanPBRSurfacePass(RHIDevice* device, const std::string& shaderDir)
    : m_Device(device)
{
    // ── Copy pipeline (fullscreen passthrough of the previous HDR result) ──────────
    m_CopyVert = MakeShader(device, shaderDir, "fullscreen.vert.spv", RHIShaderStage::Vertex);
    m_CopyFrag = MakeShader(device, shaderDir, "copy.frag.spv",       RHIShaderStage::Fragment);
    {
        RHIPipelineDesc desc;
        desc.vertexShader     = m_CopyVert.get();
        desc.fragmentShader   = m_CopyFrag.get();
        desc.resourceBindings = { { 0, RHIResourceType::CombinedImageSampler, RHIShaderStage::Fragment } };
        desc.colorFormat      = kHDRFormat;
        // Depth attachment is bound by the shared scope; the copy neither tests nor
        // writes it, but the format must match for dynamic-rendering compatibility.
        desc.depthFormat      = kDepthFormat;
        m_CopyPipeline = device->CreatePipeline(desc);
    }

    // ── Skybox pipeline (env cubemap at the far plane) ─────────────────────────────
    m_SkyVert = MakeShader(device, shaderDir, "skybox.vert.spv", RHIShaderStage::Vertex);
    m_SkyFrag = MakeShader(device, shaderDir, "skybox.frag.spv", RHIShaderStage::Fragment);
    {
        RHIBufferDesc ub; ub.size = sizeof(CameraUBO); ub.usage = RHIBufferUsage::Uniform; ub.dynamic = true;
        m_SkyUBO = device->CreateBuffer(ub);

        RHIPipelineDesc desc;
        desc.vertexShader        = m_SkyVert.get();
        desc.fragmentShader      = m_SkyFrag.get();
        desc.vertexLayout.stride = kVertexStride;
        desc.vertexLayout.attributes = { { 0, RHIVertexFormat::Float3, 0 } };   // position only
        desc.resourceBindings = {
            { 0, RHIResourceType::UniformBuffer,        RHIShaderStage::Vertex },
            { 1, RHIResourceType::CombinedImageSampler, RHIShaderStage::Fragment },
        };
        desc.colorFormat   = kHDRFormat;
        desc.cullMode      = RHICullMode::None;   // viewed from inside the cube
        desc.depthFormat   = kDepthFormat;
        desc.depthTest     = true;
        desc.depthWrite    = false;
        desc.depthCompare  = RHICompareOp::LessEqual;   // depth == 1 at the far plane
        m_SkyPipeline = device->CreatePipeline(desc);
    }

    // ── Forward PBR pipeline (opaque, world-space Cook-Torrance + IBL) ──────────────
    m_FwdVert = MakeShader(device, shaderDir, "pbr_surface.vert.spv", RHIShaderStage::Vertex);
    m_FwdFrag = MakeShader(device, shaderDir, "pbr_surface.frag.spv", RHIShaderStage::Fragment);
    {
        RHIBufferDesc ub; ub.size = sizeof(ForwardUBO); ub.usage = RHIBufferUsage::Uniform; ub.dynamic = true;
        m_FwdUBO = device->CreateBuffer(ub);

        RHIPipelineDesc desc;
        desc.vertexShader        = m_FwdVert.get();
        desc.fragmentShader      = m_FwdFrag.get();
        desc.vertexLayout.stride = kVertexStride;
        desc.vertexLayout.attributes = {
            { 0, RHIVertexFormat::Float3, 0 },
            { 1, RHIVertexFormat::Float3, sizeof(glm::vec3) },
            { 2, RHIVertexFormat::Float2, sizeof(glm::vec3) * 2 },
        };
        desc.resourceBindings = {
            { 0, RHIResourceType::UniformBuffer,        RHIShaderStage::Vertex | RHIShaderStage::Fragment },
            { 1, RHIResourceType::CombinedImageSampler, RHIShaderStage::Fragment }, // irradiance
            { 2, RHIResourceType::CombinedImageSampler, RHIShaderStage::Fragment }, // prefilter
            { 3, RHIResourceType::CombinedImageSampler, RHIShaderStage::Fragment }, // brdf LUT
        };
        desc.pushConstants = { RHIShaderStage::Vertex, sizeof(ForwardPush) };
        desc.colorFormat   = kHDRFormat;
        desc.cullMode      = RHICullMode::Back;
        desc.depthFormat   = kDepthFormat;
        desc.depthTest     = true;
        desc.depthWrite    = true;
        m_FwdPipeline = device->CreatePipeline(desc);
    }
}

VulkanPBRSurfacePass::~VulkanPBRSurfacePass() = default;

void VulkanPBRSurfacePass::AddToGraph(RHIRenderGraph& graph,
                                      RGTextureHandle input, RGTextureHandle output, RGTextureHandle depth,
                                      std::function<void(RHICommandList*)> drawSkybox,
                                      std::function<void(RHICommandList*)> drawForward)
{
    if (!m_CopySet)
        m_CopySet = m_Device->CreateResourceSet(
            m_CopyPipeline.get(), 0, {}, { { 0, graph.GetTexture(input) } });
    if (!m_SkySet)
        m_SkySet = m_Device->CreateResourceSet(
            m_SkyPipeline.get(), 0, { { 0, m_SkyUBO.get() } });   // binding 1 (env) bound raw in BindIBL
    if (!m_FwdSet)
        m_FwdSet = m_Device->CreateResourceSet(
            m_FwdPipeline.get(), 0, { { 0, m_FwdUBO.get() } });   // bindings 1-3 (IBL) bound raw in BindIBL

    // Read the previous HDR (orders this pass after deferred lighting), write a fresh
    // HDR target + load/test the shared depth. Load() keeps the deferred depth + the
    // copied color; the copy fully overwrites the color so its load contents are moot.
    graph.AddPass("PBRSurface")
        .Read(input)
        .Write(output)
        .Write(depth)
        .Load()
        .SetExecute([this, drawSkybox = std::move(drawSkybox),
                           drawForward = std::move(drawForward)](RHICommandList* cmd) {
            // 1. Copy the deferred HDR result into this pass's target (no depth touch).
            cmd->BindPipeline(m_CopyPipeline.get());
            cmd->BindResourceSet(0, m_CopySet.get());
            cmd->Draw(3);
            // 2. Skybox fills the background (far plane, LessEqual against scene depth).
            cmd->BindPipeline(m_SkyPipeline.get());
            cmd->BindResourceSet(0, m_SkySet.get());
            drawSkybox(cmd);
            // 3. Forward-lit opaque geometry over it (depth test + write).
            cmd->BindPipeline(m_FwdPipeline.get());
            cmd->BindResourceSet(0, m_FwdSet.get());
            drawForward(cmd);
        });
}

void VulkanPBRSurfacePass::BindIBL(const VulkanIBLPass& ibl)
{
    // The baked maps are world-space cubemaps / a 2D LUT (not RHITextures), so write
    // them into every frame slot of the relevant sets by hand (mirrors deferred
    // lighting's BindIBL). Static maps → same view/sampler into both frames.
    auto* device = static_cast<VulkanRHIDevice*>(m_Device);
    auto* skySet = static_cast<VulkanRHIResourceSet*>(m_SkySet.get());
    auto* fwdSet = static_cast<VulkanRHIResourceSet*>(m_FwdSet.get());
    VkDevice  dev     = device->Ctx().Device();
    VkSampler sampler = ibl.Sampler();

    auto write = [&](VkDescriptorSet set, uint32_t binding, VkImageView view) {
        VkDescriptorImageInfo info{};
        info.sampler     = sampler;
        info.imageView   = view;
        info.imageLayout = VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL;
        VkWriteDescriptorSet w{ VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET };
        w.dstSet          = set;
        w.dstBinding      = binding;
        w.descriptorCount = 1;
        w.descriptorType  = VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER;
        w.pImageInfo      = &info;
        vkUpdateDescriptorSets(dev, 1, &w, 0, nullptr);
    };

    for (uint32_t frame = 0; frame < VulkanRHIDevice::kFramesInFlight; ++frame) {
        write(skySet->Handle(frame), 1, ibl.EnvView());
        write(fwdSet->Handle(frame), 1, ibl.IrradianceView());
        write(fwdSet->Handle(frame), 2, ibl.PrefilterView());
        write(fwdSet->Handle(frame), 3, ibl.BrdfView());
    }
}

void VulkanPBRSurfacePass::SetFrameData(const glm::mat4& view, const glm::mat4& projection,
                                        const glm::vec3& camPos,
                                        const glm::vec3 lightPositions[4], const glm::vec3 lightColors[4])
{
    m_CamData.view       = view;
    m_CamData.projection = projection;
    m_SkyUBO->Update(&m_CamData, sizeof(CameraUBO));

    m_FwdData.view       = view;
    m_FwdData.projection = projection;
    m_FwdData.camPos     = glm::vec4(camPos, 1.0f);
    m_FwdData.albedo     = glm::vec4(m_Albedo, m_Metallic);
    m_FwdData.material   = glm::vec4(m_Roughness, m_AO, 0.0f, 0.0f);
    for (int i = 0; i < 4; ++i) {
        m_FwdData.lightPositions[i] = glm::vec4(lightPositions[i], 1.0f);
        m_FwdData.lightColors[i]    = glm::vec4(lightColors[i], 0.0f);
    }
    m_FwdUBO->Update(&m_FwdData, sizeof(ForwardUBO));
}

} // namespace Diamond
