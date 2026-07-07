#pragma once

#include "Renderer/RHI/RHIRenderGraph.h"

#include <array>
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
// now with the full 6-texture PBR material: one geometry pass produces view-space
// position + normal-mapped normal, albedo, packed material, and emissive in a
// single dynamic-rendering scope.
//
// Follows the ported-pass template with one twist: the pass owns its pipeline +
// shaders, but the descriptor sets are per-MATERIAL — built by the caller (the
// SceneRenderer's MaterialCache) through CreateMaterialSet and bound per draw
// inside 'drawScene'. The graph owns the transient G-buffer textures.
class VulkanGBufferPass {
public:
    // Index order of the material texture slots, matching gbuffer.frag bindings
    // 1..6 (and the GL PBRMaterial bind slots 0-5).
    enum MaterialMap : size_t {
        Albedo = 0, Normal, Metallic, Roughness, AO, Emissive,
        MapCount
    };

    VulkanGBufferPass(RHIDevice* device, const std::string& shaderDir);
    ~VulkanGBufferPass();

    // Build a descriptor set for one material against this pass's pipeline:
    // binding 0 = the shared per-frame camera UBO (view + viewProj), bindings 1-6
    // = the material maps in MaterialMap order, binding 7 = the material params
    // UBO (uvScale + emissiveStrength, static). The caller owns the set and binds
    // it (set 0) per draw inside 'drawScene'.
    std::unique_ptr<RHIResourceSet> CreateMaterialSet(
        RHIBuffer* frameUBO,
        const std::array<RHITexture*, MapCount>& maps,
        RHIBuffer* materialUBO) const;

    // Register the geometry pass. Writes the five G-buffer targets (in frag
    // `location` order) plus depth. 'drawScene' records the geometry inside the
    // pass scope — bind the material set, vertex/index buffers, push the per-draw
    // model matrix, draw — after the pass has bound the (static) pipeline. Skinned
    // geometry rebinds SkinnedPipeline() itself before its draws.
    void AddToGraph(RHIRenderGraph& graph,
                    RGTextureHandle viewPos, RGTextureHandle viewNormal,
                    RGTextureHandle albedo,  RGTextureHandle material,
                    RGTextureHandle emissive, RGTextureHandle depth,
                    std::function<void(RHICommandList*)> drawScene);

    // The skinned-geometry pipeline: identical set-0 material layout to the static
    // pipeline (so material sets are reused unchanged), a wider vertex layout with
    // bone indices/weights, and set 1 = the per-entity bone palette. drawScene
    // binds this for skinned draws, plus the material set (set 0) and a bone set
    // (set 1) built via the device against this pipeline's set 1.
    RHIPipeline* SkinnedPipeline() const { return m_SkinnedPipeline.get(); }

private:
    RHIDevice*                   m_Device;
    std::unique_ptr<RHIShader>   m_Vert;
    std::unique_ptr<RHIShader>   m_Frag;
    std::unique_ptr<RHIShader>   m_SkinnedVert;
    std::unique_ptr<RHIPipeline> m_Pipeline;
    std::unique_ptr<RHIPipeline> m_SkinnedPipeline;
};

} // namespace Diamond
