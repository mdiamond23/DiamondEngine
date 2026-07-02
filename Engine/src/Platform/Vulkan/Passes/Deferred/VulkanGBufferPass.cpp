#include "Platform/Vulkan/Passes/Deferred/VulkanGBufferPass.h"
#include "Platform/Vulkan/Passes/VulkanPassCommon.h"
#include "Renderer/RHI/RHIDevice.h"
#include "Renderer/RHI/RHICommandList.h"

#include <glm/glm.hpp>

namespace Diamond {

namespace {
// Per-draw push constant — the model matrix, matching gbuffer.vert's Push block
// (the normal matrix is derived from it in-shader).
struct GBufferPush {
    glm::mat4 model;
};

// Vertex layout the G-buffer pipeline reads: interleaved position + normal + UV
// + tangent (the SceneRenderer's MeshVertex repack of the engine Vertex).
constexpr uint32_t kVertexStride = sizeof(glm::vec3) * 3 + sizeof(glm::vec2);  // 44
} // namespace

VulkanGBufferPass::VulkanGBufferPass(RHIDevice* device, const std::string& shaderDir)
    : m_Device(device)
{
    const std::vector<uint32_t> vs = LoadSpirv(shaderDir, "gbuffer.vert.spv");
    const std::vector<uint32_t> fs = LoadSpirv(shaderDir, "gbuffer.frag.spv");
    RHIShaderDesc vsDesc{ RHIShaderStage::Vertex,   vs.data(), vs.size() };
    RHIShaderDesc fsDesc{ RHIShaderStage::Fragment, fs.data(), fs.size() };
    m_Vert = device->CreateShader(vsDesc);
    m_Frag = device->CreateShader(fsDesc);

    RHIPipelineDesc desc;
    desc.vertexShader   = m_Vert.get();
    desc.fragmentShader = m_Frag.get();
    desc.vertexLayout.stride = kVertexStride;
    desc.vertexLayout.attributes = {
        { 0, RHIVertexFormat::Float3, 0  },                                          // position
        { 1, RHIVertexFormat::Float3, sizeof(glm::vec3) },                           // normal
        { 2, RHIVertexFormat::Float2, sizeof(glm::vec3) * 2 },                       // uv
        { 3, RHIVertexFormat::Float3, sizeof(glm::vec3) * 2 + sizeof(glm::vec2) },   // tangent
    };
    // Binding 0 = per-frame camera UBO; 1-6 = the material maps in MaterialMap
    // order; 7 = the per-material params UBO. One set holds all of them, so a
    // material is a single BindResourceSet per draw.
    desc.resourceBindings = {
        { 0, RHIResourceType::UniformBuffer,        RHIShaderStage::Vertex },
        { 1, RHIResourceType::CombinedImageSampler, RHIShaderStage::Fragment },
        { 2, RHIResourceType::CombinedImageSampler, RHIShaderStage::Fragment },
        { 3, RHIResourceType::CombinedImageSampler, RHIShaderStage::Fragment },
        { 4, RHIResourceType::CombinedImageSampler, RHIShaderStage::Fragment },
        { 5, RHIResourceType::CombinedImageSampler, RHIShaderStage::Fragment },
        { 6, RHIResourceType::CombinedImageSampler, RHIShaderStage::Fragment },
        { 7, RHIResourceType::UniformBuffer,        RHIShaderStage::Fragment },
    };
    desc.pushConstants = { RHIShaderStage::Vertex, sizeof(GBufferPush) };
    // Five color attachments, in frag `location` order — the MRT the graph opens a
    // grouped render scope on. Formats mirror the GL G-buffer (RGB16F emissive →
    // RGBA16F; the RHI has no 48-bit color format and Vulkan rarely supports one).
    desc.colorFormats = {
        RHIFormat::RGBA16F,   // 0 gViewPos
        RHIFormat::RGBA16F,   // 1 gViewNormal
        RHIFormat::RGBA8,     // 2 gAlbedo
        RHIFormat::RGBA8,     // 3 gMaterial
        RHIFormat::RGBA16F,   // 4 gEmissive
    };
    desc.depthFormat = RHIFormat::Depth32F;
    desc.depthTest   = true;
    desc.depthWrite  = true;
    m_Pipeline = device->CreatePipeline(desc);
}

VulkanGBufferPass::~VulkanGBufferPass() = default;

std::unique_ptr<RHIResourceSet> VulkanGBufferPass::CreateMaterialSet(
    RHIBuffer* frameUBO,
    const std::array<RHITexture*, MapCount>& maps,
    RHIBuffer* materialUBO) const
{
    std::vector<RHITextureBinding> textures;
    textures.reserve(MapCount);
    for (size_t i = 0; i < MapCount; ++i)
        textures.push_back({ static_cast<uint32_t>(i + 1), maps[i] });

    return m_Device->CreateResourceSet(
        m_Pipeline.get(), 0,
        { { 0, frameUBO }, { 7, materialUBO } }, textures);
}

void VulkanGBufferPass::AddToGraph(RHIRenderGraph& graph,
                                   RGTextureHandle viewPos, RGTextureHandle viewNormal,
                                   RGTextureHandle albedo,  RGTextureHandle material,
                                   RGTextureHandle emissive, RGTextureHandle depth,
                                   std::function<void(RHICommandList*)> drawScene)
{
    // Writes added in attachment order — the graph forwards them to BeginRendering
    // in this order, so colorAttachments[N] matches the frag's `location = N`. Depth
    // is split out by format (Depth32F) into the depth attachment.
    graph.AddPass("GBuffer")
        .Write(viewPos)
        .Write(viewNormal)
        .Write(albedo)
        .Write(material)
        .Write(emissive)
        .Write(depth)
        .SetClearColor({ 0.0f, 0.0f, 0.0f, 1.0f })
        .SetExecute([this, drawScene = std::move(drawScene)](RHICommandList* cmd) {
            cmd->BindPipeline(m_Pipeline.get());
            drawScene(cmd);
        });
}

} // namespace Diamond
