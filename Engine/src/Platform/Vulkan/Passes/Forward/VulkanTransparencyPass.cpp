#include "Platform/Vulkan/Passes/Forward/VulkanTransparencyPass.h"
#include "Platform/Vulkan/Passes/VulkanPassCommon.h"
#include "Renderer/RHI/RHIDevice.h"
#include "Renderer/RHI/RHICommandList.h"

#include <utility>

namespace Diamond {

namespace {
constexpr RHIFormat kHDRFormat   = RHIFormat::RGBA16F;
constexpr RHIFormat kDepthFormat = RHIFormat::Depth32F;
constexpr uint32_t  kVertexStride = sizeof(glm::vec3) * 3 + sizeof(glm::vec2);  // 44 (pos/normal/uv/tangent)

struct TransparentPush { glm::mat4 model; };

std::unique_ptr<RHIShader> MakeShader(RHIDevice* device, const std::string& dir,
                                      const std::string& name, RHIShaderStage stage) {
    const std::vector<uint32_t> code = LoadSpirv(dir, name);
    RHIShaderDesc desc{ stage, code.data(), code.size() };
    return device->CreateShader(desc);
}
} // namespace

VulkanTransparencyPass::VulkanTransparencyPass(RHIDevice* device, const std::string& shaderDir)
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
        desc.depthFormat      = kDepthFormat;   // depth attachment present; copy ignores it
        m_CopyPipeline = device->CreatePipeline(desc);
    }

    // ── Transparent pipeline (src-over alpha blend, depth test, no depth write) ─────
    m_Vert = MakeShader(device, shaderDir, "transparent.vert.spv", RHIShaderStage::Vertex);
    m_Frag = MakeShader(device, shaderDir, "transparent.frag.spv", RHIShaderStage::Fragment);
    {
        RHIBufferDesc ub; ub.size = sizeof(CameraUBO); ub.usage = RHIBufferUsage::Uniform; ub.dynamic = true;
        m_UBO = device->CreateBuffer(ub);

        RHIPipelineDesc desc;
        desc.vertexShader        = m_Vert.get();
        desc.fragmentShader      = m_Frag.get();
        desc.vertexLayout.stride = kVertexStride;
        desc.vertexLayout.attributes = {
            { 0, RHIVertexFormat::Float3, 0 },                     // position
            { 2, RHIVertexFormat::Float2, sizeof(glm::vec3) * 2 }, // uv (normal skipped)
        };
        desc.resourceBindings = {
            { 0, RHIResourceType::UniformBuffer,        RHIShaderStage::Vertex },
            { 1, RHIResourceType::CombinedImageSampler, RHIShaderStage::Fragment },
        };
        desc.pushConstants = { RHIShaderStage::Vertex, sizeof(TransparentPush) };
        desc.colorFormat   = kHDRFormat;
        desc.cullMode      = RHICullMode::None;   // transparent surfaces are double-sided
        desc.depthFormat   = kDepthFormat;
        desc.depthTest     = true;
        desc.depthWrite    = false;               // blend order comes from back-to-front sort
        desc.blendMode     = RHIBlendMode::Alpha;
        m_Pipeline = device->CreatePipeline(desc);
    }
}

VulkanTransparencyPass::~VulkanTransparencyPass() = default;

void VulkanTransparencyPass::AddToGraph(RHIRenderGraph& graph,
                                        RGTextureHandle input, RGTextureHandle output, RGTextureHandle depth,
                                        RHITexture* albedo,
                                        std::function<void(RHICommandList*)> drawMeshes)
{
    if (!m_CopySet)
        m_CopySet = m_Device->CreateResourceSet(
            m_CopyPipeline.get(), 0, {}, { { 0, graph.GetTexture(input) } });
    if (!m_Set)
        m_Set = m_Device->CreateResourceSet(
            m_Pipeline.get(), 0, { { 0, m_UBO.get() } }, { { 1, albedo } });

    // Read the previous HDR (orders after the PBR-surface/forward pass), write a fresh
    // HDR target + load/test the shared depth.
    graph.AddPass("Transparency")
        .Read(input)
        .Write(output)
        .Write(depth)
        .Load()
        .SetExecute([this, drawMeshes = std::move(drawMeshes)](RHICommandList* cmd) {
            // 1. Copy the previous HDR result into this pass's target.
            cmd->BindPipeline(m_CopyPipeline.get());
            cmd->BindResourceSet(0, m_CopySet.get());
            cmd->Draw(3);
            // 2. Blend the sorted transparent meshes over it.
            cmd->BindPipeline(m_Pipeline.get());
            cmd->BindResourceSet(0, m_Set.get());
            drawMeshes(cmd);
        });
}

void VulkanTransparencyPass::SetCamera(const glm::mat4& view, const glm::mat4& projection)
{
    m_CamData.view       = view;
    m_CamData.projection = projection;
    m_UBO->Update(&m_CamData, sizeof(CameraUBO));
}

} // namespace Diamond
