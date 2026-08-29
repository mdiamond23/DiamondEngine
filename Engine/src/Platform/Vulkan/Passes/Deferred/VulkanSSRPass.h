#pragma once

#include "Renderer/RHI/RHIRenderGraph.h"

#include <glm/glm.hpp>

#include <cstdint>
#include <memory>
#include <string>

namespace Diamond {

class RHIDevice;
class RHIBuffer;
class RHITexture;
class RHIShader;
class RHIPipeline;
class RHIResourceSet;

// Screen-space reflections — THREE fullscreen passes over the G-buffer.
//
//   SSRTrace   (HALF res)  reflects the view ray about the stored view-space
//                          normal and marches it, writing WHERE the ray landed
//                          (a screen UV) rather than what it saw.
//   SSRResolve (FULL res)  turns those hits into reflection colour. Mirrors
//                          (roughness below ssr_resolve.frag's SHARP_BEGIN) are
//                          re-traced here per full-res pixel; everything rougher
//                          reuses the four low-res rays covering it, weighted by
//                          its own GGX lobe. Writes the ssrColor contract the
//                          old single full-res trace wrote — rgb = reflection,
//                          a = confidence — so nothing downstream changed.
//   SSRComposite (FULL res) weights that by fresnel/metallic/roughness and
//                          resolves scene + reflections into a NEW target.
//
// The trace/resolve split is what makes the resolution cut safe. Reflected
// colour is never blended between pixels: a depth+normal bilateral filter reads
// two pixels on one flat wall as "same surface, safe to blend" when the things
// they reflect are unrelated, which is why upsampling SSR COLOUR looks like
// mush. Only the choice of where to look is traced at reduced density; the
// fetch of the reflected image stays full-res.
//
// The composite cannot blend in-place into hdrLit: the graph points readers at a
// texture's LAST writer, so "trace reads hdrLit, composite writes hdrLit" is a
// dependency cycle. Downstream passes consume the composite's output instead.
//
// Follows the ported-pass template. The pass owns its three pipelines + shaders,
// the projection UBO, and the descriptor sets; the graph owns the transient
// ssrHit/ssrColor targets. Only the projection changes per frame
// (SetProjection, before graph.Execute).
class VulkanSSRPass {
public:
    // 'width'/'height' is the offscreen G-buffer resolution. The trace runs at
    // half of it — the same shared half-res grid GTAO and SSGI use, see
    // SceneRenderer's halfW/halfH, which must agree with TraceDim below since it
    // declares the ssrHit target.
    VulkanSSRPass(RHIDevice* device, const std::string& shaderDir,
                  uint32_t width, uint32_t height);
    ~VulkanSSRPass();

    // Trace resolution along one axis, given the full-res extent. The single
    // definition both this pass and the ssrHit declaration derive from.
    static uint32_t TraceDim(uint32_t fullDim) {
        return fullDim / 2 > 1u ? fullDim / 2 : 1u;
    }

    // Register all three passes.
    //
    // 'linearDepth' is depth_pyramid.comp's level 0 (R16F, positive linear view
    // depth, 0 = sky) — what BOTH marches tap. viewPos/viewNormal stay bound for
    // per-pixel ray setup and for the resolve's hit-point reconstruction, which
    // is a handful of fetches against the march's hundreds.
    //
    // Trace reads viewPos + viewNormal (ray setup), linearDepth (the march) and
    // gMaterial (its roughness cutoff) and writes ssrHit (caller-declared
    // transient at TraceDim(width) x TraceDim(height), RGBA16F — rg = hit UV,
    // b = confidence). It no longer reads sceneColor at all, which also frees it
    // to run alongside lighting instead of after it.
    //
    // Resolve reads ssrHit + the G-buffer + sceneColor (the lit HDR image the
    // reflections are fetched from, i.e. hdrLit *after* lighting + skybox) and
    // writes ssrColor (caller-declared full-res transient, RGBA16F).
    //
    // Composite reads sceneColor + gMaterial + the reflection texture and
    // resolves mix(scene, reflection, weight) into outColor (caller-declared,
    // RGBA16F) — downstream passes must consume outColor, not sceneColor.
    // Insert after lighting + skybox and before transparency, so glass/particles
    // draw over reflections.
    //
    // 'compositeSource' is the reflection texture the COMPOSITE samples, which
    // is not always the one the resolve wrote: RT reflections (see
    // VulkanRTReflectionPass) fill SSR's misses into a separate target and hand
    // that in here. Left invalid it defaults to ssrColor, which is exactly the
    // tier 0/1 path — no RT hardware, no behaviour change.
    void AddToGraph(RHIRenderGraph& graph,
                    RGTextureHandle viewPos, RGTextureHandle viewNormal,
                    RGTextureHandle linearDepth, RGTextureHandle coarseDepth,
                    RGTextureHandle sceneColor, RGTextureHandle ssrHit,
                    RGTextureHandle ssrColor,
                    RGTextureHandle gMaterial, RGTextureHandle outColor,
                    RGTextureHandle compositeSource = {});

    // Upload the current frame's projection (used to project marched view-space
    // positions to screen UVs, in both the trace and the resolve's mirror path).
    // Call after RHIDevice::BeginFrame (which idles the frame's buffer slot) and
    // before Execute.
    void SetProjection(const glm::mat4& projection);

private:
    // std140 layout matching ssr.frag / ssr_resolve.frag's SSRUBO.
    struct SSRUBO {
        glm::mat4 projection;
        glm::vec2 fullSize;    // G-buffer resolution
        glm::vec2 traceSize;   // ssrHit resolution
    };

    RHIDevice*                      m_Device;
    SSRUBO                          m_UBOData{};

    std::unique_ptr<RHIShader>      m_FullscreenVert;
    std::unique_ptr<RHIShader>      m_SSRFrag;
    std::unique_ptr<RHIShader>      m_SSRResolveFrag;
    std::unique_ptr<RHIShader>      m_SSRCompositeFrag;
    std::unique_ptr<RHIPipeline>    m_SSRPipeline;
    std::unique_ptr<RHIPipeline>    m_SSRResolvePipeline;
    std::unique_ptr<RHIPipeline>    m_SSRCompositePipeline;
    std::unique_ptr<RHIBuffer>      m_UBO;          // dynamic — projection updated per frame
    std::unique_ptr<RHIResourceSet> m_SSRSet;
    std::unique_ptr<RHIResourceSet> m_SSRResolveSet;
    std::unique_ptr<RHIResourceSet> m_SSRCompositeSet;
};

} // namespace Diamond
