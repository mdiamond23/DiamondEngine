#pragma once

#include <cstdint>
#include <functional>
#include <memory>
#include <string>

// Scene is engine-global (no namespace), like the rest of the ECS layer.
class Scene;

namespace Diamond {

class Camera;
class Mesh;
struct MeshData;
class RHIDevice;
class RHICommandList;
class RHITexture;

// Backend-neutral bridge from the engine's Scene/ECS to the RHI render graph.
//
// The public surface names no backend type: it takes a Scene + Camera + an
// RHIDevice and drives a full deferred frame (G-buffer → SSAO → CSM → deferred
// lighting → tonemap → swapchain) built once and executed per frame. The concrete
// implementation lives in SceneRenderer.cpp behind DIAMOND_ENABLE_VULKAN; on a
// build without the Vulkan backend, Create returns nullptr.
//
// Geometry is uploaded once via RegisterMesh, keyed on the engine Mesh* the
// MeshComponent already holds — so the ECS carries the key and the GPU buffers
// live here, decoupled from whichever backend created the Mesh handle.
class SceneRenderer {
public:
    // Builds the render graph + deferred passes sized to width x height (the
    // offscreen resolution; window-independent). Returns nullptr if the Vulkan
    // backend isn't compiled into this build.
    //
    // Two output modes, chosen at construction:
    //   * offscreen == false (default): Tonemap writes the swapchain backbuffer —
    //     the scene fills the window (the VulkanScene demo / a shipped game).
    //   * offscreen == true: Tonemap writes an LDR color texture (OutputColor())
    //     instead, and a final graph pass clears the swapchain and invokes the
    //     RenderToSwapchain overlay over it. Made for an editor: ImGui samples
    //     OutputColor() inside a viewport panel while owning the backbuffer.
    static std::unique_ptr<SceneRenderer> Create(RHIDevice* device,
                                                 uint32_t width, uint32_t height,
                                                 bool offscreen = false);

    virtual ~SceneRenderer() = default;

    // Upload a mesh's GPU vertex/index buffers once, keyed on 'key' (the same
    // Mesh* stored on the MeshComponent). Idempotent: re-registering the same key
    // is a no-op. A draw whose mesh was never registered is silently skipped.
    virtual void RegisterMesh(Mesh* key, const MeshData& data) = 0;

    // Re-bake the image-based-lighting maps from an equirectangular HDR. Optional:
    // a default environment is baked at construction, so the ambient term is always
    // valid without calling this.
    virtual void SetEnvironment(const std::string& hdrPath) = 0;

    // An overlay recorded into the swapchain after the scene is drawn (e.g. an
    // ImGui pass). Invoked inside a swapchain render scope that loads the
    // tonemapped image, so it composites on top, just before present.
    using OverlayFn = std::function<void(RHICommandList*)>;

    // Render one frame of 'scene' from 'camera' into the swapchain backbuffer.
    // Owns BeginFrame/EndFrame; rebuilds the draw list + lights from the ECS each
    // call. A no-op for the frame if the swapchain is unavailable (minimized) —
    // in which case 'overlay' is not invoked. 'overlay' may be empty.
    //
    // In offscreen mode the scene lands in OutputColor() and 'overlay' runs inside
    // a graph pass that owns (and clears) the backbuffer, with OutputColor()
    // already transitioned for sampling — so ImGui can show it as a viewport image.
    virtual void RenderToSwapchain(Scene& scene, const Camera& camera,
                                   const OverlayFn& overlay = {}) = 0;

    // The scene's tonemapped LDR output texture (offscreen mode only; nullptr in
    // swapchain mode). Invalidated by Resize — re-query (and re-register with
    // ImGui) after any resize.
    virtual RHITexture* OutputColor() const = 0;

    // Rebuild the graph's render targets at a new resolution (offscreen mode
    // only; a no-op otherwise or when the size is unchanged). Waits for the GPU
    // to go idle and recreates the size-dependent passes — call it between
    // frames, not mid-record, and expect it to be a hitch, not a per-frame op.
    virtual void Resize(uint32_t width, uint32_t height) = 0;

    // An optional 2D pass drawn over the tonemapped scene (into OutputColor() in
    // offscreen mode, the backbuffer otherwise) — the in-game UI hook. Runs
    // inside an LDR render scope that loads the scene, once per frame, before
    // the swapchain overlay. Set once; capture what it draws by reference.
    virtual void SetUIOverlay(const OverlayFn& fn) = 0;

    // Global exposure: scales the HDR scene color before the ACES tonemap curve.
    // 1.0 (the default) matches the GL renderer's tonemap exactly; lower values
    // darken the whole frame. Takes effect next frame; survives Resize.
    virtual void SetExposure(float exposure) = 0;
};

} // namespace Diamond
