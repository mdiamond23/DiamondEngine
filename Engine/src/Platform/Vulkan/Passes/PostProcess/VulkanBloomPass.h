#pragma once

#include "Renderer/RHI/RHIRenderGraph.h"

#include <memory>
#include <string>
#include <vector>

namespace Diamond {

class RHIDevice;
class RHIShader;
class RHIPipeline;
class RHIResourceSet;

// Post-process bloom. Vulkan mirror of OpenGLBloomPass, expressed as a small chain
// of graph passes: a bright-pass extract from the HDR scene, a separable Gaussian
// blur ping-ponged across two scratch targets, and an additive composite back onto
// the scene. The result stays HDR — tonemapping is left to the tonemap pass that
// runs after this.
//
// Ownership follows the M5 split: the render graph owns the transient textures
// (this pass declares its own bright + ping-pong scratch on the graph), while the
// pass owns its pipelines (shaders + state) and the descriptor sets that bind each
// stage's inputs. The blur stages share one pipeline; the axis is a push constant.
class VulkanBloomPass {
public:
    // 'width'/'height' size the internal scratch targets (HDR). 'outputFormat' is
    // the color format the composite writes — match the graph texture handed to
    // AddToGraph (the HDR scene-plus-bloom target) or the swapchain.
    VulkanBloomPass(RHIDevice* device, const std::string& shaderDir,
                    uint32_t width, uint32_t height, RHIFormat outputFormat);
    ~VulkanBloomPass();

    // Wire the bloom chain into the graph: reads 'hdrInput', writes 'output' (the
    // scene with bloom added) — or the swapchain when 'output' is invalid. Declares
    // the bright + ping-pong scratch textures and builds every stage's set once.
    void AddToGraph(RHIRenderGraph& graph, RGTextureHandle hdrInput,
                    RGTextureHandle output = {});

private:
    RHIDevice* m_Device;
    uint32_t   m_Width;
    uint32_t   m_Height;

    // Shared vertex-less fullscreen triangle, reused by every stage's pipeline.
    std::unique_ptr<RHIShader> m_Vert;

    // Bright-pass extract: HDR scene → bright (thresholded).
    std::unique_ptr<RHIShader>      m_ExtractFrag;
    std::unique_ptr<RHIPipeline>    m_ExtractPipeline;
    std::unique_ptr<RHIResourceSet> m_ExtractSet;

    // Separable blur, one pipeline reused for every ping-pong step (axis = push
    // constant); one set per step, each binding that step's source texture.
    std::unique_ptr<RHIShader>                   m_BlurFrag;
    std::unique_ptr<RHIPipeline>                 m_BlurPipeline;
    std::vector<std::unique_ptr<RHIResourceSet>> m_BlurSets;

    // Additive composite: HDR scene + blurred bloom → output.
    std::unique_ptr<RHIShader>      m_CompositeFrag;
    std::unique_ptr<RHIPipeline>    m_CompositePipeline;
    std::unique_ptr<RHIResourceSet> m_CompositeSet;
};

} // namespace Diamond
