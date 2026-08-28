#pragma once

#include "Renderer/RHI/RHIRenderGraph.h"

#include <glm/glm.hpp>

#include <array>
#include <cstdint>
#include <memory>
#include <string>

namespace Diamond {

class RHIDevice;
class RHIShader;
class RHIPipeline;
class RHIResourceSet;

// Ground-Truth Ambient Occlusion — the replacement for VulkanSSAOPass on the
// Vulkan backend (Docs/gi-design.md slice 4.5). Three compute passes: a
// horizon-search occlusion pass and a depth-aware denoise, both at HALF
// resolution (mirroring VulkanSSGIPass — the horizon march is the expensive
// part and the result is low-frequency enough to upsample), then a bilateral
// upsample back to full res for deferred_lighting.frag to sample.
//
// The output contract is what makes this more than a quality bump: aoRaw/
// aoDenoised/aoBlurred are all RGBA16F carrying **rgb = view-space bent
// normal, a = visibility**, where SSAO wrote a bare R16F scalar. The bent
// normal lets deferred_lighting.frag fetch far-field irradiance along the
// average UNOCCLUDED direction instead of fetching along the geometric normal
// and scaling the result down, which is what removed slice 4's tuned tier-2
// fudge factor.
//
// Follows the ported-pass template: the pass owns its pipelines/shaders/sets,
// the graph owns the transient targets. No UBO — everything per-frame fits
// in push constants, so there is no buffer-slot ordering rule to respect.
class VulkanGTAOPass {
public:
    // 'width'/'height' are the offscreen G-buffer resolution; the horizon
    // search and denoise run at half that (rounded up on odd sizes via the
    // same max(1, x/2) rule as VulkanSSGIPass), the upsample at full res.
    VulkanGTAOPass(RHIDevice* device, const std::string& shaderDir,
                   uint32_t width, uint32_t height);
    ~VulkanGTAOPass();

    // Occlusion reads viewPos + viewNormal and writes aoRaw (half res);
    // the denoise reads aoRaw + viewPos and writes aoDenoised (half res);
    // the upsample bilaterally resolves aoDenoised against full-res viewPos/
    // viewNormal into aoBlurred (full res) — the target deferred_lighting.frag
    // actually samples. aoRaw/aoDenoised must be caller-declared HALF-res
    // RGBA16F storage textures; aoBlurred is FULL-res RGBA16F storage. Sets
    // are built once — graph textures keep their identity across frames.
    // 'depthLevels' is the linear view-depth pyramid (VulkanDepthPyramidPass)
    // the horizon march taps INSTEAD of viewPos — 2 bytes per tap rather than 8,
    // with wide strides reading a smaller, cache-resident level. viewPos is
    // still bound for the denoise pass's bilateral weight.
    void AddToGraph(RHIRenderGraph& graph,
                    RGTextureHandle viewPos, RGTextureHandle viewNormal,
                    const std::array<RGTextureHandle, 4>& depthLevels,
                    RGTextureHandle aoRaw, RGTextureHandle aoDenoised,
                    RGTextureHandle aoBlurred);

    // The world-radius → screen-pixels conversion needs the projection's
    // vertical scale. Cheap enough to re-derive every frame; call it wherever
    // the other passes take their per-frame projection.
    void SetProjection(const glm::mat4& projection);

    // Live-tunable quality/look. 'bentNormals' off writes the geometric normal
    // instead of the bent one — the A/B that isolates the directional change
    // from the AO-quality change.
    void SetParams(float radius, int slices, int steps, bool bentNormals);

private:
    // std430 push-constant block matching gtao.comp's Push. All members are
    // 4-byte scalars after the leading vec2, so the natural layout is exact.
    struct GTAOPush {
        glm::vec2 screenSize;
        glm::vec2 invProj;      // (1/proj[0][0], 1/proj[1][1]) — unprojects a tap
        float     projScale;
        float     radius;
        float     falloffStart;
        int32_t   slices;
        int32_t   steps;
        uint32_t  frameIndex;
        float     bentNormals;
    };

    RHIDevice* m_Device;
    uint32_t   m_Width;
    uint32_t   m_Height;
    uint32_t   m_HalfW;
    uint32_t   m_HalfH;

    // Pixels per view-space unit at z = -1, from the projection's [1][1].
    float     m_ProjScale = 1.0f;
    // Reciprocal projection scales, so a depth tap can be turned back into a
    // view-space position without carrying the full matrix.
    glm::vec2 m_InvProj { 1.0f };
    float    m_Radius      = 0.8f;
    int      m_Slices      = 3;
    int      m_Steps       = 6;
    bool     m_BentNormals = true;
    uint32_t m_FrameIndex  = 0;

    std::unique_ptr<RHIShader>      m_GTAOComp;
    std::unique_ptr<RHIShader>      m_DenoiseComp;
    std::unique_ptr<RHIShader>      m_UpsampleComp;
    std::unique_ptr<RHIPipeline>    m_GTAOPipeline;
    std::unique_ptr<RHIPipeline>    m_DenoisePipeline;
    std::unique_ptr<RHIPipeline>    m_UpsamplePipeline;
    std::unique_ptr<RHIResourceSet> m_GTAOSet;
    std::unique_ptr<RHIResourceSet> m_DenoiseSet;
    std::unique_ptr<RHIResourceSet> m_UpsampleSet;
};

} // namespace Diamond
