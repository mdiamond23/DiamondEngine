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

// Screen-space reflections — two fullscreen passes over the G-buffer. Trace: for
// each pixel, reflect the view ray about the stored view-space normal and
// ray-march it in view space, projecting each step into screen space (via the
// projection uploaded in SetProjection) and comparing against the G-buffer's
// viewPos.z until a hit; the hit's lit scene color becomes the reflection, with
// hit confidence in alpha (misses fade to 0). Composite: weights the trace by
// the material's fresnel/metallic/roughness response and resolves scene +
// reflections into a NEW target — reflective pixels lerp toward the reflection,
// everything else passes through with its IBL specular.
//
// The composite cannot blend in-place into hdrLit: the graph points readers at a
// texture's LAST writer, so "trace reads hdrLit, composite writes hdrLit" is a
// dependency cycle. Downstream passes consume the composite's output instead.
//
// Follows the ported-pass template. The pass owns its two pipelines + shaders,
// the projection UBO, and the descriptor sets; the graph owns the transient
// ssrColor target. Only the projection changes per frame (SetProjection, before
// graph.Execute).
class VulkanSSRPass {
public:
    // 'width'/'height' is the offscreen G-buffer resolution — used to derive the
    // per-pixel step in screen space for the ray march.
    VulkanSSRPass(RHIDevice* device, const std::string& shaderDir,
                  uint32_t width, uint32_t height);
    ~VulkanSSRPass();

    // Register both passes. Trace reads viewPos + viewNormal (ray setup and march
    // depth comparison) and sceneColor (the lit HDR image the rays fetch
    // reflections from, i.e. hdrLit *after* lighting + skybox) and writes ssrColor
    // (caller-declared transient, RGBA16F — rgb = reflection, a = confidence).
    // Composite reads sceneColor + ssrColor + gMaterial (metallic/roughness
    // weighting) and resolves mix(scene, reflection, weight) into outColor
    // (caller-declared, RGBA16F) — downstream passes must consume outColor, not
    // sceneColor. Insert after lighting + skybox and before transparency, so
    // glass/particles draw over reflections.
    //
    // 'compositeSource' is the reflection texture the COMPOSITE samples, which
    // is not always the one the trace wrote: RT reflections (see
    // VulkanRTReflectionPass) fill SSR's misses into a separate target and hand
    // that in here. Left invalid it defaults to ssrColor, which is exactly the
    // tier 0/1 path — no RT hardware, no behaviour change.
    void AddToGraph(RHIRenderGraph& graph,
                    RGTextureHandle viewPos, RGTextureHandle viewNormal,
                    RGTextureHandle sceneColor, RGTextureHandle ssrColor,
                    RGTextureHandle gMaterial, RGTextureHandle outColor,
                    RGTextureHandle compositeSource = {});

    // Upload the current frame's projection into the UBO (used to project marched
    // view-space positions to screen UVs). Call after RHIDevice::BeginFrame (which
    // idles the frame's buffer slot) and before Execute.
    void SetProjection(const glm::mat4& projection);

private:
    // std140 layout matching ssr.frag's SSRUBO.
    struct SSRUBO {
        glm::mat4 projection;
        glm::vec2 screenSize;
        glm::vec2 _pad;
    };

    RHIDevice*                      m_Device;
    SSRUBO                          m_UBOData{};

    std::unique_ptr<RHIShader>      m_FullscreenVert;
    std::unique_ptr<RHIShader>      m_SSRFrag;
    std::unique_ptr<RHIShader>      m_SSRCompositeFrag;
    std::unique_ptr<RHIPipeline>    m_SSRPipeline;
    std::unique_ptr<RHIPipeline>    m_SSRCompositePipeline;
    std::unique_ptr<RHIBuffer>      m_UBO;          // dynamic — projection updated per frame
    std::unique_ptr<RHIResourceSet> m_SSRSet;
    std::unique_ptr<RHIResourceSet> m_SSRCompositeSet;
};

} // namespace Diamond
