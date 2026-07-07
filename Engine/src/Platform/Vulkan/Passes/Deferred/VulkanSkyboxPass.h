#pragma once

#include "Renderer/RHI/RHIRenderGraph.h"

#include <glm/glm.hpp>

#include <memory>
#include <string>

namespace Diamond {

class RHIDevice;
class RHIBuffer;
class RHIShader;
class RHIPipeline;
class RHIResourceSet;
class VulkanIBLPass;

// Skybox background for the deferred chain — the Vulkan counterpart of the GL
// renderer's DrawSkybox inside its deferred-lighting pass. Composites the baked
// IBL radiance cubemap into the lit HDR target at the far plane: gl_Position.xyww
// puts every fragment at depth 1, and LessEqual against the loaded G-buffer depth
// fills only the pixels no geometry wrote. Runs between DeferredLighting and
// Particles; the write-after-write on the shared HDR target resolves by insertion
// order (the particles pass's pattern).
//
// Self-contained: the pass owns its pipeline, camera UBO, and a position-only
// unit-cube buffer, so AddToGraph needs no draw callback. The env cubemap isn't an
// RHITexture — BindIBL writes its raw view into the set (the BindIBL pattern the
// lighting + forward passes use).
class VulkanSkyboxPass {
public:
    VulkanSkyboxPass(RHIDevice* device, const std::string& shaderDir);
    ~VulkanSkyboxPass();

    // Register the pass: load + composite into 'color' (HDR), load + test 'depth'.
    void AddToGraph(RHIRenderGraph& graph, RGTextureHandle color, RGTextureHandle depth);

    // Write the baked env cubemap into the set. Call after AddToGraph (which
    // creates the set) and after each BakeEnvironment; the map is static.
    void BindIBL(const VulkanIBLPass& ibl);

    // Upload this frame's camera. Call after BeginFrame, before graph.Execute.
    // The shader strips the view translation itself, so pass the full view.
    void SetFrameData(const glm::mat4& view, const glm::mat4& projection);

    // Hot-reload (editor only): recompiles skybox.vert/frag from shaderDir's
    // current .spv, rebuilds the pipeline IN PLACE (same 'this' — the graph
    // pass reads m_Pipeline at Execute time, no graph rebuild), and rebuilds
    // the descriptor set eagerly (unlike the material/transparency caches,
    // AddToGraph only runs once at graph-build time, so nothing else would
    // ever repopulate it). The caller must re-call BindIBL right after — the
    // fresh set's binding 1 (env cubemap) starts unwritten. Caller must
    // WaitIdle() first. A no-op (logs) if the new SPIR-V is missing/corrupt.
    void Reload();

private:
    struct CameraUBO {   // skybox.vert
        glm::mat4 view;
        glm::mat4 projection;
    };

    RHIDevice*  m_Device;
    std::string m_ShaderDir;

    std::unique_ptr<RHIShader>      m_Vert;
    std::unique_ptr<RHIShader>      m_Frag;
    std::unique_ptr<RHIPipeline>    m_Pipeline;
    std::unique_ptr<RHIBuffer>      m_UBO;
    std::unique_ptr<RHIResourceSet> m_Set;

    std::unique_ptr<RHIBuffer> m_CubeVB;   // position-only unit cube
    std::unique_ptr<RHIBuffer> m_CubeIB;
    uint32_t                   m_CubeIndexCount = 0;
};

} // namespace Diamond
