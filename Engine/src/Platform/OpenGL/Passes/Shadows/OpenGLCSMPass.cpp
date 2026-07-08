#include "Platform/OpenGL/Passes/Shadows/OpenGLCSMPass.h"
#include "Profiling/GLRendererStats.h"
#include <glad/gl.h>
#include <glm/gtc/matrix_transform.hpp>  // glm::ortho, glm::lookAt
#include <glm/gtc/type_ptr.hpp>          // glm::value_ptr
#include <cmath>                          // std::pow
#include <algorithm>                      // std::min, std::max
#include <spdlog/spdlog.h>

namespace {

static void CheckFBO(const char* label)
{
    GLenum status = glCheckFramebufferStatus(GL_FRAMEBUFFER);
    if (status != GL_FRAMEBUFFER_COMPLETE)
        spdlog::error("FBO incomplete at [{}]: 0x{:04X}", label, status);
    else
        spdlog::info("FBO complete at [{}]", label);
}

} // namespace

namespace Diamond {

OpenGLCSMPass::~OpenGLCSMPass()
{
    if (m_ShadowArray) glDeleteTextures(1, &m_ShadowArray);
    for (int i = 0; i < NUM_CASCADES; ++i)
        if (m_FBOs[i]) glDeleteFramebuffers(1, &m_FBOs[i]);
}

void OpenGLCSMPass::Setup(int resolution)
{
    m_Resolution = resolution;

    glGenTextures(1, &m_ShadowArray);
    glBindTexture(GL_TEXTURE_2D_ARRAY, m_ShadowArray);
    glTexImage3D(GL_TEXTURE_2D_ARRAY, 0, GL_DEPTH_COMPONENT32F,
                 resolution, resolution, NUM_CASCADES, 0,
                 GL_DEPTH_COMPONENT, GL_FLOAT, nullptr);
    glTexParameteri(GL_TEXTURE_2D_ARRAY, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_2D_ARRAY, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_2D_ARRAY, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_BORDER);
    glTexParameteri(GL_TEXTURE_2D_ARRAY, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_BORDER);
    float borderColor[] = { 1.0f, 1.0f, 1.0f, 1.0f };
    glTexParameterfv(GL_TEXTURE_2D_ARRAY, GL_TEXTURE_BORDER_COLOR, borderColor);

    for (int i = 0; i < NUM_CASCADES; ++i) {
        glGenFramebuffers(1, &m_FBOs[i]);
        glBindFramebuffer(GL_FRAMEBUFFER, m_FBOs[i]);
        glFramebufferTextureLayer(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, m_ShadowArray, 0, i);
        glDrawBuffer(GL_NONE);
        glReadBuffer(GL_NONE);
        CheckFBO("csm cascade");
    }

    glBindFramebuffer(GL_FRAMEBUFFER, 0);
}

void OpenGLCSMPass::ComputeCascades(
                const SunLight&  sun,
                const glm::mat4& cameraView,
                const glm::mat4& cameraProj,
                float            near,
                float            far)
{
    // 1. Compute split depths — log/linear blend (lambda 0.5 is a good default)
    constexpr float lambda = 0.5f;
    for (int i = 1; i <= NUM_CASCADES; ++i) {
        float t     = (float)i / (float)NUM_CASCADES;
        float log_i = near * std::pow(far / near, t);
        float lin_i = near + t * (far - near);
        m_SplitDepths[i - 1] = glm::mix(lin_i, log_i, lambda);
    }

    // 2. Unproject the full camera frustum to world space.
    // Iterating x/y/z in {-1,+1} gives 8 NDC corners; z inner loop means
    // pairs (even index = near z=-1, odd index = far z=+1): (0,1),(2,3),(4,5),(6,7)
    glm::mat4 invVP = glm::inverse(cameraProj * cameraView);
    std::array<glm::vec3, 8> frustumCorners;
    {
        int idx = 0;
        for (float x : {-1.0f, 1.0f})
            for (float y : {-1.0f, 1.0f})
                for (float z : {-1.0f, 1.0f}) {
                    glm::vec4 pt = invVP * glm::vec4(x, y, z, 1.0f);
                    frustumCorners[idx++] = glm::vec3(pt) / pt.w;
                }
    }

    // 3. Build each cascade's light-space matrix
    glm::vec3 lightDir = glm::normalize(sun.direction);
    glm::vec3 up = glm::abs(lightDir.y) < 0.99f ? glm::vec3(0, 1, 0) : glm::vec3(1, 0, 0);

    float prevSplit = near;
    for (int i = 0; i < NUM_CASCADES; ++i) {
        float nextSplit = m_SplitDepths[i];
        float prevFrac  = (prevSplit - near) / (far - near);
        float nextFrac  = (nextSplit - near) / (far - near);

        // Lerp full-frustum corners to this cascade's depth slice
        std::array<glm::vec3, 8> cascadeCorners;
        for (int j = 0; j < 4; ++j) {
            glm::vec3 nearCorner = frustumCorners[j * 2];
            glm::vec3 farCorner  = frustumCorners[j * 2 + 1];
            glm::vec3 ray        = farCorner - nearCorner;
            cascadeCorners[j * 2]     = nearCorner + prevFrac * ray;
            cascadeCorners[j * 2 + 1] = nearCorner + nextFrac * ray;
        }

        // Centroid: stable lookAt origin for the light view matrix
        glm::vec3 centroid(0.0f);
        for (const auto& c : cascadeCorners) centroid += c;
        centroid /= 8.0f;

        glm::mat4 lightView = glm::lookAt(
            centroid - lightDir * (far - near),
            centroid,
            up);

        // Fit a tight ortho box around the cascade corners in light space
        glm::vec3 lsMin = glm::vec3(lightView * glm::vec4(cascadeCorners[0], 1.0f));
        glm::vec3 lsMax = lsMin;
        for (int j = 1; j < 8; ++j) {
            glm::vec3 ls = glm::vec3(lightView * glm::vec4(cascadeCorners[j], 1.0f));
            lsMin = glm::min(lsMin, ls);
            lsMax = glm::max(lsMax, ls);
        }

        // Pull near plane back to catch shadow casters behind the camera
        float zPad = (lsMax.z - lsMin.z) * 0.1f;
        glm::mat4 lightProj = glm::ortho(lsMin.x, lsMax.x,
                                          lsMin.y, lsMax.y,
                                          lsMin.z - zPad, lsMax.z);
        m_LightMatrices[i] = lightProj * lightView;
        prevSplit = nextSplit;
    }
}

void OpenGLCSMPass::Render(
            const OpenGLShader&          depthShader,
            const OpenGLShader&          skinnedShader,
            const std::vector<DrawCall>& draws,
            const SunLight&              sun,
            const glm::mat4&             cameraView,
            const glm::mat4&             cameraProj,
            float                        cameraNear,
            float                        cameraFar)
{
    ComputeCascades(sun, cameraView, cameraProj, cameraNear, cameraFar);

    glViewport(0, 0, m_Resolution, m_Resolution);
    glEnable(GL_DEPTH_TEST);
    glCullFace(GL_FRONT); // eliminates peter-panning

    for (int i = 0; i < NUM_CASCADES; ++i) {
        glBindFramebuffer(GL_FRAMEBUFFER, m_FBOs[i]);
        glClear(GL_DEPTH_BUFFER_BIT);

        // lightSpaceMatrix is per-cascade, so (re)set it whenever a shader is
        // bound within this cascade.
        const OpenGLShader* bound = nullptr;
        for (const auto& draw : draws) {
            const bool          skinned = (draw.bonePalette != nullptr && draw.boneCount > 0);
            const OpenGLShader& shader  = skinned ? skinnedShader : depthShader;
            if (bound != &shader) {
                shader.Bind();
                shader.SetMat4("lightSpaceMatrix", m_LightMatrices[i]);
                bound = &shader;
            }
            shader.SetMat4("model", draw.modelMatrix);
            if (skinned)
                shader.SetMat4Array("uBones", draw.bonePalette, draw.boneCount);
            draw.mesh->Draw(shader);
            GLStats::RecordShadowCaster();
        }
    }

    glCullFace(GL_BACK);
    glBindFramebuffer(GL_FRAMEBUFFER, 0);
}

} // namespace Diamond
