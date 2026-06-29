#pragma once

#include "Renderer/RHI/RHIRenderGraph.h"

#include <glm/glm.hpp>

#include <functional>
#include <memory>
#include <string>

namespace Diamond {

class RHIDevice;
class RHIBuffer;
class RHIShader;
class RHIPipeline;
class RHIResourceSet;
class RHITexture;
class RHICommandList;

// Forward transparency — Vulkan port of OpenGLTransparencyPass. Draws alpha-blended
// geometry over the lit opaque scene, depth-testing (but not writing) against the
// shared scene depth so transparent objects are occluded by opaques.
//
// Like the forward PBR pass it composes over the previous HDR result: a pass can't
// hardware-blend over the attachment it also samples, so it first fullscreen-copies
// the previous HDR ('input') into its own 'output' target, then renders the
// back-to-front-sorted transparent meshes with src-over blending. This preserves the
// graph's single-writer invariant (the copy stands in for the GL renderer drawing
// straight into the populated HDR framebuffer). Back-to-front sorting is the caller's
// job (done in the draw callback), matching how the GL pass sorted its draw list.
//
// Ported-pass template: owns its copy + transparent pipelines, the camera UBO, and the
// descriptor sets; the graph owns the transient textures.
class VulkanTransparencyPass {
public:
    VulkanTransparencyPass(RHIDevice* device, const std::string& shaderDir);
    ~VulkanTransparencyPass();

    // Register the pass. 'input' is the previous HDR result (copied in first), 'output'
    // the fresh HDR target, 'depth' the shared scene depth (loaded, tested). 'albedo' is
    // the transparent material texture. 'drawMeshes' records the sorted transparent draws
    // (bind buffers, push model, draw) after the pass binds the transparent pipeline+set.
    void AddToGraph(RHIRenderGraph& graph,
                    RGTextureHandle input, RGTextureHandle output, RGTextureHandle depth,
                    RHITexture* albedo,
                    std::function<void(RHICommandList*)> drawMeshes);

    // Upload this frame's camera. Call after BeginFrame, before graph.Execute.
    void SetCamera(const glm::mat4& view, const glm::mat4& projection);

private:
    struct CameraUBO {        // transparent.vert
        glm::mat4 view;
        glm::mat4 projection;
    };

    RHIDevice* m_Device;
    CameraUBO  m_CamData{};

    // Copy: fullscreen.vert + copy.frag.
    std::unique_ptr<RHIShader>      m_CopyVert;
    std::unique_ptr<RHIShader>      m_CopyFrag;
    std::unique_ptr<RHIPipeline>    m_CopyPipeline;
    std::unique_ptr<RHIResourceSet> m_CopySet;

    // Transparent meshes: transparent.vert/frag (alpha blend, depth test no write).
    std::unique_ptr<RHIShader>      m_Vert;
    std::unique_ptr<RHIShader>      m_Frag;
    std::unique_ptr<RHIPipeline>    m_Pipeline;
    std::unique_ptr<RHIBuffer>      m_UBO;
    std::unique_ptr<RHIResourceSet> m_Set;
};

} // namespace Diamond
