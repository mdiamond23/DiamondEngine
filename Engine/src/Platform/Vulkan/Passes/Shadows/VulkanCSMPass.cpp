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
}

VulkanCSMPass::~VulkanCSMPass() = default;

// Ported verbatim from OpenGLCSMPass::ComputeCascades — the math is backend-neutral.
void VulkanCSMPass::ComputeCascades(const glm::vec3& lightDir,
                                    const glm::mat4& cameraView,
                                    const glm::mat4& cameraProj,
                                    float near, float far)
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

    // 3. Per-cascade light-space matrix.
    glm::vec3 dir = glm::normalize(lightDir);
    glm::vec3 up  = glm::abs(dir.y) < 0.99f ? glm::vec3(0, 1, 0) : glm::vec3(1, 0, 0);

    float prevSplit = near;
    for (int i = 0; i < NUM_CASCADES; ++i) {
        float nextSplit = m_SplitDepths[i];
        float prevFrac  = (prevSplit - near) / (far - near);
        float nextFrac  = (nextSplit - near) / (far - near);

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

        glm::mat4 lightView = glm::lookAt(centroid - dir * (far - near), centroid, up);

        glm::vec3 lsMin = glm::vec3(lightView * glm::vec4(cascadeCorners[0], 1.0f));
        glm::vec3 lsMax = lsMin;
        for (int j = 1; j < 8; ++j) {
            glm::vec3 ls = glm::vec3(lightView * glm::vec4(cascadeCorners[j], 1.0f));
            lsMin = glm::min(lsMin, ls);
            lsMax = glm::max(lsMax, ls);
        }

        // Pull the near plane back to catch casters behind the slice. The _ZO
        // variant is named explicitly: plain glm::ortho picks its depth range
        // from a per-TU define, and identical inline instantiations COMDAT-fold
        // across TUs (the RGPass ODR lesson) — the GL-depth version here would
        // clip half of each cascade volume out of the shadow map and break the
        // [0,1] comparison deferred_lighting.frag does.
        float zPad = (lsMax.z - lsMin.z) * 0.1f;
        glm::mat4 lightProj = glm::orthoRH_ZO(lsMin.x, lsMax.x, lsMin.y, lsMax.y,
                                              lsMin.z - zPad, lsMax.z);
        m_LightMatrices[i] = lightProj * lightView;
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
