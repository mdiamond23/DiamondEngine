#pragma once

#include "Renderer/RHI/RHIRenderGraph.h"

#include <functional>
#include <memory>
#include <string>

namespace Diamond {

class RHIDevice;
class RHIBuffer;
class RHITexture;
class RHIShader;
class RHIPipeline;
class RHIResourceSet;
class RHICommandList;

// Deferred geometry pass — fills the G-buffer. Vulkan port of OpenGLGBufferPass,
// and the first pass to write Multiple Render Targets: one geometry pass produces
// view-space position + normal, albedo, packed material, and emissive in a single
// dynamic-rendering scope. This is the milestone's real test of the graph's
// grouped-color-attachment path (the post passes only ever wrote one target).
//
// Follows the ported-pass template: the pass owns its pipeline + shaders and the
// descriptor set binding the per-frame UBO + albedo texture; the graph owns the
// transient G-buffer textures. AddToGraph wires the five color writes + the depth
// write so the graph drives ordering and the layout barriers.
//
// MRT slice: albedo comes from one texture, the other channels are shader
// constants, and the normal is the interpolated vertex normal (no tangent-space
// normal mapping yet). The full PBR material binding lands with deferred lighting.
class VulkanGBufferPass {
public:
    VulkanGBufferPass(RHIDevice* device, const std::string& shaderDir);
    ~VulkanGBufferPass();

    // Register the geometry pass. Writes the five G-buffer targets (in frag
    // `location` order) plus depth; 'frameUBO' (view + viewProj) and 'albedo' feed
    // the descriptor set, built once. 'drawScene' records the geometry inside the
    // pass scope — bind vertex/index buffers, push the per-draw model matrix, draw —
    // after the pass has bound the pipeline + set.
    void AddToGraph(RHIRenderGraph& graph,
                    RGTextureHandle viewPos, RGTextureHandle viewNormal,
                    RGTextureHandle albedo,  RGTextureHandle material,
                    RGTextureHandle emissive, RGTextureHandle depth,
                    RHIBuffer* frameUBO, RHITexture* albedoTexture,
                    std::function<void(RHICommandList*)> drawScene);

private:
    RHIDevice*                      m_Device;
    std::unique_ptr<RHIShader>      m_Vert;
    std::unique_ptr<RHIShader>      m_Frag;
    std::unique_ptr<RHIPipeline>    m_Pipeline;
    std::unique_ptr<RHIResourceSet> m_Set;
};

} // namespace Diamond
