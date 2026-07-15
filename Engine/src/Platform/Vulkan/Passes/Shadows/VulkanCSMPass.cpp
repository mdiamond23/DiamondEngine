#include "Platform/Vulkan/Passes/Shadows/VulkanCSMPass.h"
#include "Platform/Vulkan/Passes/VulkanPassCommon.h"
#include "Renderer/RHI/RHIDevice.h"
#include "Renderer/RHI/RHICommandList.h"

#include <glm/gtc/matrix_transform.hpp>   // glm::ortho, glm::lookAt

#include <algorithm>   // glm::min/max are free functions; std::pow via <cmath>
#include <cmath>

namespace Diamond {

namespace {
// The vertex buffer is the shared MeshVertex (pos/normal/uv/tangent, stride 44);
// the depth pass only reads position at location 0.
constexpr uint32_t kVertexStride = sizeof(glm::vec3) * 3 + sizeof(glm::vec2);  // 44
} // namespace

VulkanCSMPass::VulkanCSMPass(RHIDevice* device, const std::string& shaderDir)
    : m_Device(device)
{
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
    // No descriptors — lightSpace * model arrives as a push constant.
    desc.pushConstants = { RHIShaderStage::Vertex, sizeof(glm::mat4) };
    // Depth-only: no colorFormat(s) → the render scope opens with zero color targets.
    desc.depthFormat = RHIFormat::Depth32F;
    desc.depthTest   = true;
    desc.depthWrite  = true;
    desc.cullMode    = RHICullMode::Front;   // front-face cull curbs peter-panning
    m_Pipeline = device->CreatePipeline(desc);

    // Skinned variant: bone palette at set 1 (set 0 stays empty — the depth pass
    // has no per-frame resources), wider vertex layout, same front-cull depth
    // state and lightSpace * model push. Shared by the spot-shadow pass shape too,
    // but each pass owns its own pipeline object.
    const std::vector<uint32_t> svs = LoadSpirv(shaderDir, "csm_depth_skinned.vert.spv");
    RHIShaderDesc svsDesc{ RHIShaderStage::Vertex, svs.data(), svs.size() };
    m_SkinnedVert = device->CreateShader(svsDesc);

    RHIPipelineDesc skinned = desc;
    skinned.vertexShader            = m_SkinnedVert.get();
    skinned.vertexLayout.stride     = kSkinnedVertexStride;
    skinned.vertexLayout.attributes = SkinnedDepthVertexAttributes();
    skinned.resourceBindings1       = BonesSetBindings();
    m_SkinnedPipeline = device->CreatePipeline(skinned);
}

VulkanCSMPass::~VulkanCSMPass() = default;

// Ported from OpenGLCSMPass::ComputeCascades, then stabilized: the GL version's
// tight per-frame AABB fit changes the ortho box's size and position with every
// camera move, so the shadow map re-rasterizes on a different texel grid each
// frame — shadow edges crawl and acne patches sweep across surfaces ("moving
// darkening"). Here each cascade instead gets a camera-rotation-invariant
// bounding-sphere fit plus whole-texel snapping of the box's translation.
void VulkanCSMPass::ComputeCascades(const glm::vec3& lightDir,
                                    const glm::mat4& cameraView,
                                    const glm::mat4& cameraProj,
                                    float near, float far,
                                    uint32_t shadowRes)
{
    // 1. Split depths — log/linear blend (lambda 0.5 is a good default).
    constexpr float lambda = 0.5f;
    for (int i = 1; i <= NUM_CASCADES; ++i) {
        float t     = (float)i / (float)NUM_CASCADES;
        float log_i = near * std::pow(far / near, t);
        float lin_i = near + t * (far - near);
        m_SplitDepths[i - 1] = glm::mix(lin_i, log_i, lambda);
    }

    // 2. Unproject the full camera frustum to world space. The camera projection
    // is a Vulkan [0,1]-depth matrix (perspectiveRH_ZO), so the NDC corners are
    // z = 0 (near) and z = 1 (far) — NOT GL's ±1: unprojecting z = -1 through a
    // ZO matrix lands off the frustum entirely and skews every cascade fit as
    // the camera moves. z inner loop → pairs (even = near, odd = far).
    glm::mat4 invVP = glm::inverse(cameraProj * cameraView);
    std::array<glm::vec3, 8> frustumCorners;
    {
        int idx = 0;
        for (float x : {-1.0f, 1.0f})
            for (float y : {-1.0f, 1.0f})
                for (float z : {0.0f, 1.0f}) {
                    glm::vec4 pt = invVP * glm::vec4(x, y, z, 1.0f);
                    frustumCorners[idx++] = glm::vec3(pt) / pt.w;
                }
    }

    // The unprojected corners span the CAMERA's near..far, which is deeper than
    // the shadowed range [near, far] (kFar vs kShadowFar). Slice fractions must
    // be over the corners' actual depth span — dividing by (far - near), as the
    // GL port did, stretched every cascade ~4× past its split (cascade 3 to the
    // full camera far): ~4× the fitted radius, so a quarter of the texel density
    // and per-cascade cull volumes admitting most of the scene's casters.
    const float camNear = -(cameraView * glm::vec4(frustumCorners[0], 1.0f)).z;
    const float camFar  = -(cameraView * glm::vec4(frustumCorners[1], 1.0f)).z;

    // 3. Per-cascade light-space matrix.
    glm::vec3 dir = glm::normalize(lightDir);
    glm::vec3 up  = glm::abs(dir.y) < 0.99f ? glm::vec3(0, 1, 0) : glm::vec3(1, 0, 0);

    float prevSplit = near;
    for (int i = 0; i < NUM_CASCADES; ++i) {
        float nextSplit = m_SplitDepths[i];
        float prevFrac  = (prevSplit - camNear) / (camFar - camNear);
        float nextFrac  = (nextSplit - camNear) / (camFar - camNear);

        std::array<glm::vec3, 8> cascadeCorners;
        for (int j = 0; j < 4; ++j) {
            glm::vec3 nearCorner = frustumCorners[j * 2];
            glm::vec3 farCorner  = frustumCorners[j * 2 + 1];
            glm::vec3 ray        = farCorner - nearCorner;
            cascadeCorners[j * 2]     = nearCorner + prevFrac * ray;
            cascadeCorners[j * 2 + 1] = nearCorner + nextFrac * ray;
        }

        glm::vec3 centroid(0.0f);
        for (const auto& c : cascadeCorners) centroid += c;
        centroid /= 8.0f;

        // Bounding sphere instead of a tight AABB: the slice is a rigid shape,
        // so centroid→corner distances don't change as the camera rotates or
        // moves — the ortho box keeps one size (and thus one texel density) for
        // a given FOV/aspect/split. ~√2 looser than the tight fit, the price of
        // stability. Ceil-quantized so FP noise can't wobble it frame-to-frame.
        float radius = 0.0f;
        for (const auto& c : cascadeCorners)
            radius = glm::max(radius, glm::length(c - centroid));
        radius = std::ceil(radius * 16.0f) / 16.0f;

        glm::mat4 lightView = glm::lookAt(centroid - dir * (far - near), centroid, up);

        // Ortho zNear/zFar are DISTANCES along the light view's forward axis
        // (positive in front of the eye), NOT raw light-space z coordinates.
        // The GL-ported code passed the corners' negative z here, putting the
        // depth window entirely behind the eye: cascades 0-2 clipped ALL their
        // geometry (empty maps → wrongly lit), and only cascade 3's oversized
        // window partially overlapped [0,1] — the root cause of the
        // camera-following lit/shadowed disagreement. Verified numerically
        // against glm's orthoRH_ZO source (scratchpad csm_sim.py, 2026-07-15).
        //
        // The eye sits (far - near) sunward of the centroid, so the slice
        // sphere spans distances (far - near) ∓ radius. The near plane is then
        // pulled a further zPull toward the sun (a negative distance — behind
        // the eye — is legal for ortho) so occluders ABOVE the slice, like
        // Sponza's roof over a floor-hugging cascade 0, land in the map. zPull
        // is sized by the shadowed range, not scene bounds; a taller occluder
        // would need bounds plumbed here. The far plane hugs the sphere —
        // geometry beyond it can't shade this slice.
        //
        // The _ZO variant is named explicitly: plain glm::ortho picks its depth
        // range from a per-TU define, and identical inline instantiations
        // COMDAT-fold across TUs (the RGPass ODR lesson) — the GL-depth version
        // here would break the [0,1] comparison deferred_lighting.frag does.
        const float D     = far - near;
        const float zPull = D + 2.0f * radius;
        glm::mat4 lightProj = glm::orthoRH_ZO(-radius, radius, -radius, radius,
                                              D - radius - zPull, D + radius);
        glm::mat4 shadowMat = lightProj * lightView;

        // Texel snap: the box size is constant now, but its center still tracks
        // the camera continuously. Round the matrix's XY translation to whole
        // shadow-map texels (project the world origin, snap it, apply the
        // sub-texel remainder as an NDC offset — ortho, so w == 1 and NDC spans
        // 2 units across shadowRes texels). The grid stays put in world space,
        // which is what kills edge crawl and swimming acne.
        glm::vec4 origin = shadowMat * glm::vec4(0.0f, 0.0f, 0.0f, 1.0f);
        glm::vec2 texels = glm::vec2(origin) * ((float)shadowRes * 0.5f);
        glm::vec2 snap   = (glm::round(texels) - texels) * (2.0f / (float)shadowRes);
        shadowMat[3][0] += snap.x;
        shadowMat[3][1] += snap.y;

        m_LightMatrices[i] = shadowMat;
        prevSplit = nextSplit;
    }
}

void VulkanCSMPass::AddToGraph(
        RHIRenderGraph& graph,
        const std::array<RGTextureHandle, NUM_CASCADES>& cascades,
        std::function<void(RHICommandList*, const glm::mat4&)> drawScene)
{
    // One depth pass per cascade. They share the pipeline + draw callback and differ
    // only by which cascade target they write and which light matrix they pass down.
    // drawScene is copied into each pass (it's invoked every frame).
    for (int i = 0; i < NUM_CASCADES; ++i) {
        graph.AddPass("CSM" + std::to_string(i))
            .Write(cascades[i])
            .SetExecute([this, i, drawScene](RHICommandList* cmd) {
                cmd->BindPipeline(m_Pipeline.get());
                drawScene(cmd, m_LightMatrices[i]);
            });
    }
}

} // namespace Diamond
