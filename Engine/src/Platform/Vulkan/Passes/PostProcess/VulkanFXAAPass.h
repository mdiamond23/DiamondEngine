#pragma once

#include "Renderer/RHI/RHIRenderGraph.h"

#include <memory>
#include <string>

namespace Diamond {

class RHIDevice;
class RHIShader;
class RHIPipeline;
class RHIResourceSet;

// Post-process FXAA. A vertex-less fullscreen triangle samples a tonemapped LDR
// color texture, runs an edge-directed two-tap blend, and writes its output (the
// swapchain by default). Vulkan port of OpenGLFXAAPass.
//
// Follows the same ownership split as VulkanTonemapPass: the render graph owns the
// transient input texture; the pass owns its pipeline (shaders + state) and the
// descriptor set binding that input. AddToGraph wires reads/writes so the graph
// drives ordering + layout barriers. FXAA runs in LDR space, so it is placed after
// tonemap in the chain (HDR scene → tonemap → LDR → FXAA → swapchain).
class VulkanFXAAPass {
public:
    // 'outputFormat' is the color format of the render target this pass writes; it
    // must match either the swapchain or the graph texture handed to AddToGraph.
    VulkanFXAAPass(RHIDevice* device, const std::string& shaderDir,
                   RHIFormat outputFormat);
    ~VulkanFXAAPass();

    // Register FXAA as a graph pass: reads 'ldrInput' and writes 'output' — or the
    // swapchain backbuffer when 'output' is an invalid handle (the default). The
    // sampler set is built once from the input's pooled texture.
    void AddToGraph(RHIRenderGraph& graph, RGTextureHandle ldrInput,
                    RGTextureHandle output = {});

private:
    RHIDevice*                      m_Device;
    std::unique_ptr<RHIShader>      m_Vert;
    std::unique_ptr<RHIShader>      m_Frag;
    std::unique_ptr<RHIPipeline>    m_Pipeline;
    std::unique_ptr<RHIResourceSet> m_Set;
};

} // namespace Diamond
