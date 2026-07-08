#include "Platform/Vulkan/Passes/Shadows/VulkanSpotShadowPass.h"
#include "Platform/Vulkan/Passes/VulkanPassCommon.h"
#include "Renderer/RHI/RHIDevice.h"
#include "Renderer/RHI/RHICommandList.h"

#include <glm/gtc/matrix_transform.hpp>

#include <algorithm>
#include <cmath>

namespace Diamond {

namespace {
// The vertex buffer is the shared MeshVertex (pos/normal/uv/tangent, stride 44);
// the depth pass only reads position at location 0.
constexpr uint32_t kVertexStride = sizeof(glm::vec3) * 3 + sizeof(glm::vec2);  // 44
} // namespace

VulkanSpotShadowPass::VulkanSpotShadowPass(RHIDevice* device, const std::string& shaderDir)
    : m_Device(device)
{
    // Same depth-only pipeline as the CSM pass — the csm_depth shaders are already
    // generic (push lightSpace * model, empty fragment stage).
    const std::vector<uint32_t> vs = LoadSpirv(shaderDir, "csm_depth.vert.spv");
    const std::vector<uint32_t> fs = LoadSpirv(shaderDir, "csm_depth.frag.spv");
    RHIShaderDesc vsDesc{ RHIShaderStage::Vertex,   vs.data(), vs.size() };
    RHIShaderDesc fsDesc{ RHIShaderStage::Fragment, fs.data(), fs.size() };
    m_Vert = device->CreateShader(vsDesc);
    m_Frag = device->CreateShader(fsDesc);

    RHIPipelineDesc desc;
    desc.vertexShader   = m_Vert.get();
    desc.fragmentShader = m_Frag.get();
    desc.vertexLayout.stride     = kVertexStride;
    desc.vertexLayout.attributes = { { 0, RHIVertexFormat::Float3, 0 } };   // position only
    desc.pushConstants = { RHIShaderStage::Vertex, sizeof(glm::mat4) };
    desc.depthFormat = RHIFormat::Depth32F;
    desc.depthTest   = true;
    desc.depthWrite  = true;
    desc.cullMode    = RHICullMode::Front;   // front-face cull curbs peter-panning
    m_Pipeline = device->CreatePipeline(desc);

    // Skinned variant — identical to the CSM pass's skinned depth pipeline (set 1
    // bone palette, wider vertex layout, same push + depth state).
    const std::vector<uint32_t> svs = LoadSpirv(shaderDir, "csm_depth_skinned.vert.spv");
    RHIShaderDesc svsDesc{ RHIShaderStage::Vertex, svs.data(), svs.size() };
    m_SkinnedVert = device->CreateShader(svsDesc);

    RHIPipelineDesc skinned = desc;
    skinned.vertexShader            = m_SkinnedVert.get();
    skinned.vertexLayout.stride     = kSkinnedVertexStride;
    skinned.vertexLayout.attributes = SkinnedDepthVertexAttributes();
    skinned.resourceBindings1       = BonesSetBindings();
    m_SkinnedPipeline = device->CreatePipeline(skinned);

    // Pass-owned depth maps (same desc the graph derived for its Depth32F
    // transients). Every slot is cleared + transitioned each Record, so the maps
    // are always valid to sample — no creation-time clear needed.
    RHITextureDesc td;
    td.width  = kResolution;
    td.height = kResolution;
    td.format = RHIFormat::Depth32F;
    td.usage  = RHITextureUsage::DepthAttachment | RHITextureUsage::Sampled;
    for (auto& map : m_Maps)
        map = device->CreateTexture(td);
}

VulkanSpotShadowPass::~VulkanSpotShadowPass() = default;

void VulkanSpotShadowPass::ComputeMatrices(const std::vector<SpotLightInfo>& spots)
{
    m_Count = std::min(static_cast<int>(spots.size()), MAX_SPOTS);
    for (int i = 0; i < m_Count; ++i) {
        const SpotLightInfo& s = spots[i];
        const glm::vec3 dir = glm::normalize(s.direction);
        const glm::vec3 up  = std::abs(dir.y) < 0.99f ? glm::vec3(0, 1, 0) : glm::vec3(1, 0, 0);

        // The frustum covers the full outer cone (fov = 2 × outer half-angle),
        // clamped below 180°; the far plane is the light's range. Explicit _ZO
        // per the backend rule — never rely on a per-TU GLM depth define.
        const float fov = glm::radians(glm::clamp(s.outerDeg, 1.0f, 85.0f) * 2.0f);
        const glm::mat4 view = glm::lookAt(s.position, s.position + dir, up);
        const glm::mat4 proj = glm::perspectiveRH_ZO(fov, 1.0f, 0.1f, std::max(s.range, 0.5f));
        m_LightMatrices[i] = proj * view;
    }
}

void VulkanSpotShadowPass::Record(
        RHICommandList* cmd,
        const std::function<void(RHICommandList*, const glm::mat4&)>& drawScene)
{
    // One depth scope per slot, mirroring what the graph passes did: a slot beyond
    // this frame's light count still clears its target but skips the scene draws.
    // Each map ends in SampledRead — the graphs no longer know about these
    // textures, so the transition the graph used to drive happens here.
    for (int i = 0; i < MAX_SPOTS; ++i) {
        RHIRenderPass rp;
        rp.depthTexture = m_Maps[i].get();
        rp.clearDepth   = true;
        cmd->BeginRendering(rp);
        if (i < m_Count) {
            cmd->BindPipeline(m_Pipeline.get());
            drawScene(cmd, m_LightMatrices[i]);
        }
        cmd->EndRendering();
        cmd->TransitionTexture(m_Maps[i].get(), RHITextureState::SampledRead);
    }
}

} // namespace Diamond
