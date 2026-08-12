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

// Screen-space GI — four graph passes over the G-buffer, traced at HALF
// resolution. Trace (compute): cosine hemisphere rays marched through gViewPos,
// gathering radiance from the lit scene. Temporal (compute): reprojects last
// frame through gVelocity and accumulates, which is the only reason a 4-8 ray
// budget is viable. History copy: moves the resolved image into the persistent
// history, mirroring VulkanTAAPass. Composite (raster): bilateral upsample to
// full res, then REPLACES the far-field irradiance the lighting pass used
// (gIndirect) where SSGI is confident, rather than stacking on top of it.
//
// Like SSR, the composite resolves into a NEW target — the graph points readers
// at a texture's last writer, so reading hdrLit and writing it back would cycle.
class VulkanSSGIPass {
public:
    // 'width'/'height' is the full-res G-buffer size; the trace runs at half.
    VulkanSSGIPass(RHIDevice* device, const std::string& shaderDir,
                   uint32_t width, uint32_t height);
    ~VulkanSSGIPass();

    // ssgiRaw/ssgiResolved must be declared HALF res with storage = true (both
    // are compute-written). ssgiHistory is half res and persistent = true but
    // needs NO storage flag — the raster copy writes it, the temporal pass
    // samples it. 'indirect' is deferred_lighting's second MRT output. Insert
    // after lighting + skybox and before SSR, which then consumes outColor
    // instead of hdrLit.
    void AddToGraph(RHIRenderGraph& graph,
                    RGTextureHandle viewPos, RGTextureHandle viewNormal,
                    RGTextureHandle sceneColor, RGTextureHandle velocity,
                    RGTextureHandle albedo, RGTextureHandle material,
                    RGTextureHandle ssao, RGTextureHandle indirect,
                    RGTextureHandle ssgiRaw, RGTextureHandle ssgiResolved,
                    RGTextureHandle ssgiHistory, RGTextureHandle outColor);

    // Upload the frame's projection (projects marched view-space positions to
    // screen UVs). After BeginFrame, before Execute.
    void SetProjection(const glm::mat4& projection);

    // While disabled the trace/temporal dispatches are skipped outright and the
    // composite passes through — the graph is compiled once, so the passes stay
    // registered either way. Disabling drops the history so re-enabling reseeds.
    void SetEnabled(bool enabled);
    bool IsEnabled() const { return m_Enabled; }

    // Forget the accumulation — for discontinuities gVelocity can't express,
    // like a camera teleport.
    void InvalidateHistory() { m_HistoryValid = false; }

    // Live tuning. Rays are per half-res pixel per frame; alpha is the temporal
    // blend toward the current frame (lower = smoother, more lag).
    void SetRayCount(int rays);
    void SetIntensity(float intensity);
    void SetMaxDistance(float distance);
    void SetBlendFactor(float alpha);

private:
    // std140, matching ssgi.comp's SSGIUBO.
    struct SSGIUBO {
        glm::mat4 projection;
        glm::vec2 halfSize;
        glm::vec2 _pad;
    };

    struct TracePush {
        uint32_t frameIndex;
        int32_t  rayCount;
        float    maxDistance;
        float    intensity;
    };

    struct TemporalPush {
        float alpha;   // 1.0 = ignore history
    };

    struct CompositePush {
        float strength;   // 0 = passthrough
    };

    RHIDevice* m_Device;
    uint32_t   m_HalfW;
    uint32_t   m_HalfH;
    SSGIUBO    m_UBOData{};

    uint32_t m_FrameIndex   = 0;
    bool     m_Enabled      = true;
    bool     m_HistoryValid = false;   // alpha = 1 passthrough until seeded
    int32_t  m_RayCount     = 8;
    float    m_Intensity    = 1.0f;
    float    m_MaxDistance  = 8.0f;    // view-space units
    float    m_Alpha        = 0.1f;    // steady-state temporal blend

    std::unique_ptr<RHIShader>      m_FullscreenVert;
    std::unique_ptr<RHIShader>      m_TraceComp;
    std::unique_ptr<RHIShader>      m_TemporalComp;
    std::unique_ptr<RHIShader>      m_CompositeFrag;
    std::unique_ptr<RHIShader>      m_CopyFrag;

    std::unique_ptr<RHIPipeline>    m_TracePipeline;
    std::unique_ptr<RHIPipeline>    m_TemporalPipeline;
    std::unique_ptr<RHIPipeline>    m_CompositePipeline;
    std::unique_ptr<RHIPipeline>    m_CopyPipeline;

    std::unique_ptr<RHIBuffer>      m_UBO;   // dynamic — projection per frame

    std::unique_ptr<RHIResourceSet> m_TraceSet;
    std::unique_ptr<RHIResourceSet> m_TemporalSet;
    std::unique_ptr<RHIResourceSet> m_CompositeSet;
    std::unique_ptr<RHIResourceSet> m_CopySet;
};

} // namespace Diamond
