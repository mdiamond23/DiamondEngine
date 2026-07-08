#include "Platform/Vulkan/Passes/Shadows/VulkanSpotShadowPass.h"
#include "Platform/Vulkan/Passes/VulkanPassCommon.h"
#include "Platform/Vulkan/RHI/VulkanRHIDevice.h"   // kFramesInFlight
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
    // transients). Working maps are seeded + drawn each Record; static maps hold
    // the cached static-caster depth, re-baked only when a slot is dirty. Both are
    // sampleable depth targets. No creation-time clear: an active slot always bakes
    // its static map (dirty on first active frame) before the copy samples it.
    RHITextureDesc td;
    td.width  = kResolution;
    td.height = kResolution;
    td.format = RHIFormat::Depth32F;
    td.usage  = RHITextureUsage::DepthAttachment | RHITextureUsage::Sampled;
    for (int i = 0; i < MAX_SPOTS; ++i) {
        m_Maps[i]   = device->CreateTexture(td);
        m_Static[i] = device->CreateTexture(td);
    }

    // Depth-copy pipeline: fullscreen triangle that writes gl_FragDepth from a
    // cached static map. LessEqual + depthWrite against a 1.0-cleared target writes
    // every fragment (the skybox-at-far trick), reproducing the static occluders so
    // dynamic casters can be drawn over them with the normal Less caster pipeline.
    const std::vector<uint32_t> cvs = LoadSpirv(shaderDir, "fullscreen.vert.spv");
    const std::vector<uint32_t> cfs = LoadSpirv(shaderDir, "spot_depth_copy.frag.spv");
    RHIShaderDesc cvsDesc{ RHIShaderStage::Vertex,   cvs.data(), cvs.size() };
    RHIShaderDesc cfsDesc{ RHIShaderStage::Fragment, cfs.data(), cfs.size() };
    m_CopyVert = device->CreateShader(cvsDesc);
    m_CopyFrag = device->CreateShader(cfsDesc);

    RHIPipelineDesc copy;
    copy.vertexShader     = m_CopyVert.get();
    copy.fragmentShader   = m_CopyFrag.get();
    copy.resourceBindings = { { 0, RHIResourceType::CombinedImageSampler, RHIShaderStage::Fragment } };
    copy.depthFormat      = RHIFormat::Depth32F;
    copy.depthTest        = true;
    copy.depthWrite       = true;
    copy.depthCompare     = RHICompareOp::LessEqual;
    copy.cullMode         = RHICullMode::None;   // fullscreen triangle — never cull
    m_CopyPipeline = device->CreatePipeline(copy);

    for (int i = 0; i < MAX_SPOTS; ++i)
        m_CopySets[i] = device->CreateResourceSet(m_CopyPipeline.get(), 0, {},
                                                  { { 0, m_Static[i].get() } });

    // Bake every active slot's static map on its first frame.
    m_StaticDirty.fill(VulkanRHIDevice::kFramesInFlight);
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

        // A slot whose light moved (matrix changed) or that a different light now
        // occupies must re-bake its static cache — its cached depth is stale under
        // the new projection. Exact compare is safe: identical input → identical
        // matrix, so a stationary light never re-bakes.
        if (i >= m_PrevCount || m_LightMatrices[i] != m_PrevMatrices[i])
            m_StaticDirty[i] = VulkanRHIDevice::kFramesInFlight;
    }
    m_PrevMatrices = m_LightMatrices;
    m_PrevCount    = m_Count;
}

void VulkanSpotShadowPass::MarkStaticDirty()
{
    for (int i = 0; i < m_Count; ++i)
        m_StaticDirty[i] = VulkanRHIDevice::kFramesInFlight;
}

void VulkanSpotShadowPass::Record(
        RHICommandList* cmd,
        const std::function<void(RHICommandList*, const glm::mat4&)>& drawStatic,
        const std::function<void(RHICommandList*, const glm::mat4&)>& drawDynamic)
{
    // Two depth scopes per active slot: (1) re-bake the static cache when dirty,
    // (2) seed the working map from that cache and draw this frame's dynamic
    // casters over it. Inactive slots still clear + transition their working map so
    // the lighting set stays valid to sample. Each map ends in SampledRead — the
    // graphs no longer know about these textures, so the transition happens here.
    for (int i = 0; i < MAX_SPOTS; ++i) {
        const bool active = i < m_Count;

        // (1) Static cache — only when dirty. Bake static casters, then leave the
        // map in SampledRead so step (2)'s copy (and future frames) can sample it.
        if (active && m_StaticDirty[i] > 0) {
            RHIRenderPass sp;
            sp.depthTexture = m_Static[i].get();
            sp.clearDepth   = true;
            cmd->BeginRendering(sp);
            cmd->BindPipeline(m_Pipeline.get());
            drawStatic(cmd, m_LightMatrices[i]);
            cmd->EndRendering();
            cmd->TransitionTexture(m_Static[i].get(), RHITextureState::SampledRead);
            --m_StaticDirty[i];
        }

        // (2) Working map: clear to 1.0, copy the cached static depth in, then draw
        // dynamic casters. The copy pipeline (LessEqual + depthWrite) writes every
        // texel of the cleared target; dynamic casters (Less) win where closer.
        RHIRenderPass rp;
        rp.depthTexture = m_Maps[i].get();
        rp.clearDepth   = true;
        cmd->BeginRendering(rp);
        if (active) {
            cmd->BindPipeline(m_CopyPipeline.get());
            cmd->BindResourceSet(0, m_CopySets[i].get());
            cmd->Draw(3);
            cmd->BindPipeline(m_Pipeline.get());
            drawDynamic(cmd, m_LightMatrices[i]);
        }
        cmd->EndRendering();
        cmd->TransitionTexture(m_Maps[i].get(), RHITextureState::SampledRead);
    }
}

} // namespace Diamond
