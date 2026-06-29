#pragma once

#include "Renderer/RHI/RHIRenderGraph.h"

#include <glm/glm.hpp>

#include <memory>
#include <string>

namespace Diamond {

class RHIDevice;
class RHIBuffer;
class RHITexture;
class RHIShader;
class RHIPipeline;
class RHIResourceSet;

// Screen-space ambient occlusion — Vulkan port of OpenGLSSAOPass. Two fullscreen
// passes over the G-buffer: an occlusion pass samples a normal-oriented 64-sample
// hemisphere kernel (rotated by a tiled 4×4 noise) and compares projected sample
// depths against the stored geometry depth, then a 4×4 box blur smooths the noisy
// result. Both targets are single-channel (R16F).
//
// Follows the ported-pass template. The pass owns its two pipelines + shaders, the
// noise texture, the kernel/projection UBO, and the descriptor sets; the graph owns
// the transient ssaoRaw / ssaoBlurred textures. The kernel + noise are built once;
// only the projection changes per frame (SetProjection, before graph.Execute).
class VulkanSSAOPass {
public:
    // 'width'/'height' size the noise tiling (noiseScale = size / 4) — the offscreen
    // G-buffer resolution.
    VulkanSSAOPass(RHIDevice* device, const std::string& shaderDir,
                   uint32_t width, uint32_t height);
    ~VulkanSSAOPass();

    // Register both passes: occlusion reads viewPos + viewNormal (+ the pass's noise
    // and kernel UBO) and writes ssaoRaw; the blur reads ssaoRaw and writes
    // ssaoBlurred. Both transient targets are caller-declared R16F so the demo can
    // also visualize the raw result. Sets are built once.
    void AddToGraph(RHIRenderGraph& graph,
                    RGTextureHandle viewPos, RGTextureHandle viewNormal,
                    RGTextureHandle ssaoRaw, RGTextureHandle ssaoBlurred);

    // Upload the current frame's projection into the kernel UBO. Call after
    // RHIDevice::BeginFrame (which idles the frame's buffer slot) and before Execute.
    void SetProjection(const glm::mat4& projection);

private:
    // std140 layout matching ssao.frag's SSAOUBO.
    struct SSAOUBO {
        glm::mat4 projection;
        glm::vec4 samples[64];   // vec3 kernel padded to vec4
        glm::vec2 noiseScale;
        glm::vec2 _pad;
    };

    RHIDevice*                      m_Device;
    SSAOUBO                         m_UBOData{};

    std::unique_ptr<RHIShader>      m_FullscreenVert;
    std::unique_ptr<RHIShader>      m_SSAOFrag;
    std::unique_ptr<RHIShader>      m_BlurFrag;
    std::unique_ptr<RHIPipeline>    m_SSAOPipeline;
    std::unique_ptr<RHIPipeline>    m_BlurPipeline;
    std::unique_ptr<RHIBuffer>      m_UBO;          // dynamic — projection updated per frame
    std::unique_ptr<RHITexture>     m_Noise;        // static 4×4 RGBA8 rotation vectors
    std::unique_ptr<RHIResourceSet> m_SSAOSet;
    std::unique_ptr<RHIResourceSet> m_BlurSet;
};

} // namespace Diamond
