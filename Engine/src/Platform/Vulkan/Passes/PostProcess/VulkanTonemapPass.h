#pragma once

#include "Renderer/RHI/RHIRenderGraph.h"

#include <memory>
#include <string>

namespace Diamond {

class RHIDevice;
class RHIShader;
class RHIPipeline;
class RHIResourceSet;

// Post-process tonemap. A vertex-less fullscreen triangle samples an HDR color
// texture, applies ACES filmic tonemapping + gamma, and writes the swapchain
// backbuffer. Vulkan port of OpenGLTonemapPass's tonemap step.
//
// This is the template every ported Vulkan pass follows. Ownership split:
//   * the render graph owns the transient textures that flow between passes;
//   * the pass owns its pipeline state (shaders + pipeline) and the descriptor
//     set that binds its graph-owned read texture into that pipeline.
// AddToGraph wires the pass into the RHIRenderGraph with the right reads/writes so
// the graph drives ordering + the layout barriers; construction stays out of the
// frame loop.
//
// The output target is chosen at construction: pass the swapchain format (the
// default) to write the backbuffer directly, or an offscreen color format (e.g.
// RGBA8) when a later post pass — FXAA — consumes the tonemapped LDR result.
class VulkanTonemapPass {
public:
    // 'outputFormat' is the color format of the render target this pass writes; it
    // must match either the swapchain (when writing the backbuffer) or the graph
    // texture handed to AddToGraph.
    VulkanTonemapPass(RHIDevice* device, const std::string& shaderDir,
                      RHIFormat outputFormat);
    ~VulkanTonemapPass();

    // Register the tonemap as a graph pass: reads 'hdrInput' and writes 'output' —
    // or the swapchain backbuffer when 'output' is an invalid handle (the default).
    // The sampler resource set is built once from the input's pooled texture (which
    // the graph keeps stable for its lifetime).
    void AddToGraph(RHIRenderGraph& graph, RGTextureHandle hdrInput,
                    RGTextureHandle output = {});

    // Scales the HDR input before the ACES curve (pushed per frame). 1.0 (the
    // default) reproduces the GL tonemap exactly; lower to darken the frame.
    void SetExposure(float exposure) { m_Exposure = exposure; }

private:
    RHIDevice*                      m_Device;
    float                           m_Exposure = 1.0f;
    std::unique_ptr<RHIShader>      m_Vert;
    std::unique_ptr<RHIShader>      m_Frag;
    std::unique_ptr<RHIPipeline>    m_Pipeline;
    std::unique_ptr<RHIResourceSet> m_Set;
};

} // namespace Diamond
