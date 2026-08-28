#include "Platform/Vulkan/Passes/Deferred/VulkanGBufferPass.h"
#include "Platform/Vulkan/Passes/VulkanPassCommon.h"
#include "Renderer/RHI/RHIDevice.h"
#include "Renderer/RHI/RHICommandList.h"

#include <glm/glm.hpp>

namespace Diamond {

namespace {
// Per-draw push constant, matching gbuffer.vert's Push block (the normal matrix
// is derived from 'model' in-shader). 'prevModel' is TAA's velocity input — this
// is 128 bytes, the guaranteed-minimum Vulkan push-constant budget, so nothing
// else fits in this block.
struct GBufferPush {
    glm::mat4 model;
    glm::mat4 prevModel;
};

// Vertex layout the G-buffer pipeline reads: interleaved position + normal + UV
// + tangent (the SceneRenderer's MeshVertex repack of the engine Vertex).
constexpr uint32_t kVertexStride = sizeof(glm::vec3) * 3 + sizeof(glm::vec2);  // 44
} // namespace

VulkanGBufferPass::VulkanGBufferPass(RHIDevice* device, const std::string& shaderDir)
    : m_Device(device), m_ShaderDir(shaderDir)
{
    Build(/*isReload*/ false);
}

VulkanGBufferPass::~VulkanGBufferPass() = default;

void VulkanGBufferPass::Build(bool isReload)
{
    const std::vector<uint32_t> vs  = TryLoadSpirv(m_ShaderDir, "gbuffer.vert.spv");
    const std::vector<uint32_t> fs  = TryLoadSpirv(m_ShaderDir, "gbuffer.frag.spv");
    const std::vector<uint32_t> svs = TryLoadSpirv(m_ShaderDir, "gbuffer_skinned.vert.spv");
    if (vs.empty() || fs.empty() || svs.empty()) {
        if (isReload) {
            spdlog::warn("[VulkanGBufferPass] reload skipped — missing/corrupt SPIR-V");
            return;
        }
        spdlog::critical("[Vulkan] failed to open SPIR-V for VulkanGBufferPass");
        std::abort();
    }

    RHIShaderDesc vsDesc{ RHIShaderStage::Vertex,   vs.data(), vs.size() };
    RHIShaderDesc fsDesc{ RHIShaderStage::Fragment, fs.data(), fs.size() };
    m_Vert = m_Device->CreateShader(vsDesc);
    m_Frag = m_Device->CreateShader(fsDesc);

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
    // Six color attachments, in frag `location` order — the MRT the graph opens a
    // grouped render scope on. Formats mirror the GL G-buffer (RGB16F emissive →
    // RGBA16F; the RHI has no 48-bit color format and Vulkan rarely supports one).
    desc.colorFormats = {
        RHIFormat::RGBA16F,   // 0 gViewPos
        RHIFormat::RGBA16F,   // 1 gViewNormal
        RHIFormat::RGBA8,     // 2 gAlbedo
        RHIFormat::RGBA8,     // 3 gMaterial
        RHIFormat::RGBA16F,   // 4 gEmissive
        RHIFormat::RG16F,     // 5 gVelocity — screen-space motion (TAA)
    };
    desc.depthFormat = RHIFormat::Depth32F;
    desc.depthTest   = true;
    desc.depthWrite  = true;
    // LessEqual, not Less: the depth prepass has already written the exact depth
    // of every visible fragment, so the surviving ones arrive here EQUAL and
    // must still pass. Occluded fragments compare greater and are killed by
    // early-Z before the fragment shader runs, which is the whole point.
    // Unchanged when no prepass runs — nothing is ever equal to a cleared 1.0.
    desc.depthCompare = RHICompareOp::LessEqual;
    m_Pipeline = m_Device->CreatePipeline(desc);

    // Skinned variant: same material set-0 layout, push constant, and MRT — so a
    // skinned draw reuses its material's static set-0 descriptor — but with the
    // wider bone vertex layout and a set-1 bone-palette UBO. gbuffer_skinned.vert
    // blends the palette into a skin matrix before the model transform; the frag
    // stage is shared.
    RHIShaderDesc svsDesc{ RHIShaderStage::Vertex, svs.data(), svs.size() };
    m_SkinnedVert = m_Device->CreateShader(svsDesc);

    RHIPipelineDesc skinned = desc;
    skinned.vertexShader             = m_SkinnedVert.get();
    skinned.vertexLayout.stride      = kSkinnedVertexStride;
    skinned.vertexLayout.attributes  = SkinnedVertexAttributes();
    skinned.resourceBindings1        = BonesSetBindings();
    m_SkinnedPipeline = m_Device->CreatePipeline(skinned);

    // ── Depth prepass variants ───────────────────────────────────────────────
    // Same vertex stages, same set-0 layout (so material sets bind unchanged —
    // gbuffer.vert still needs the camera UBO at binding 0), but NO color
    // attachments and a fragment shader that is literally `void main() {}`.
    // csm_depth.frag is reused rather than adding an identical empty shader.
    const std::vector<uint32_t> dfs = TryLoadSpirv(m_ShaderDir, "csm_depth.frag.spv");
    if (dfs.empty()) {
        if (isReload) {
            spdlog::warn("[VulkanGBufferPass] depth-prepass reload skipped — missing SPIR-V");
            return;
        }
        spdlog::critical("[Vulkan] failed to open csm_depth.frag.spv for the depth prepass");
        std::abort();
    }
    RHIShaderDesc dfsDesc{ RHIShaderStage::Fragment, dfs.data(), dfs.size() };
    m_DepthFrag = m_Device->CreateShader(dfsDesc);

    RHIPipelineDesc depthOnly = desc;
    depthOnly.fragmentShader = m_DepthFrag.get();
    depthOnly.colorFormats.clear();          // depth attachment only
    depthOnly.depthCompare = RHICompareOp::Less;   // this pass is what LAYS the depth
    m_DepthPipeline = m_Device->CreatePipeline(depthOnly);

    RHIPipelineDesc depthSkinned = depthOnly;
    depthSkinned.vertexShader            = m_SkinnedVert.get();
    depthSkinned.vertexLayout.stride     = kSkinnedVertexStride;
    depthSkinned.vertexLayout.attributes = SkinnedVertexAttributes();
    depthSkinned.resourceBindings1       = BonesSetBindings();
    m_DepthSkinnedPipeline = m_Device->CreatePipeline(depthSkinned);
}

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
                                   RGTextureHandle emissive, RGTextureHandle velocity,
                                   RGTextureHandle depth,
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
        .Write(velocity)
        .Write(depth)
        .SetClearColor({ 0.0f, 0.0f, 0.0f, 1.0f })
        // Colors still clear (gViewPos = 0 is the background marker every
        // downstream pass tests), but depth LOADS what the prepass wrote.
        .LoadDepth()
        .SetExecute([this, drawScene = std::move(drawScene)](RHICommandList* cmd) {
            cmd->BindPipeline(m_Pipeline.get());
            drawScene(cmd);
        });
}

void VulkanGBufferPass::AddDepthPrepassToGraph(RHIRenderGraph& graph, RGTextureHandle depth,
                                               std::function<void(RHICommandList*)> drawScene)
{
    graph.AddPass("DepthPrepass")
        .Write(depth)
        .SetExecute([this, drawScene = std::move(drawScene)](RHICommandList* cmd) {
            cmd->BindPipeline(m_DepthPipeline.get());
            drawScene(cmd);
        });
}

} // namespace Diamond
