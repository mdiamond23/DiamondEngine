#pragma once

#include "Renderer/ParticleRenderer.h"
#include "Profiling/ProfilerStats.h"

#include <cstdint>
#include <functional>
#include <memory>
#include <string>

#include <glm/glm.hpp>

// Scene is engine-global (no namespace), like the rest of the ECS layer.
class Scene;

namespace Diamond {

class Camera;
class Mesh;
struct MeshData;
class RHIDevice;
class RHICommandList;
class RHITexture;
struct PBRMaterial;

// Display curve applied by the tonemap pass. ACES is the Narkowicz fit the GL
// renderer uses — cheap, but saturated highlights shift hue as they clip. AgX
// rolls them off toward white instead, which holds up far better on emissive
// and bloomed content. Values are pushed to tonemap.frag as-is.
enum class Tonemapper : int {
    ACES = 0,
    AgX  = 1,
};

// Eye adaptation. The renderer meters the scene's average log-luminance each
// frame, eases the retained value toward it, and maps that onto 'keyValue' to
// get an exposure multiplier — which is applied ON TOP of SetExposure, so the
// manual value stays useful as EV compensation.
struct AutoExposureSettings {
    // Off by default: a fixed exposure is the predictable thing for a scene
    // that was authored against one, and adaptation changes every frame.
    bool  enabled = false;

    // Luminance the adapted average maps to — "middle grey". Raise to expose
    // the scene brighter overall.
    float keyValue = 0.18f;

    // Metering range in log2 luminance (stops). The measured average is clamped
    // here, which bounds how far exposure can swing in either direction — the
    // guard against a mostly-black or mostly-blown frame running away.
    float minLogLuminance = -8.0f;
    float maxLogLuminance =  4.0f;

    // Adaptation rate, roughly stops per second. Higher settles faster.
    float speed = 2.0f;
};

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

    // Packaged builds: override the compiled-in dev-tree SPIR-V directory
    // (DIAMOND_VULKAN_SHADER_DIR) with an exe-relative one. Must be called
    // before Create() — every pass loads its modules from here, including
    // ones rebuilt later by resize and hot-reload.
    static void SetShaderDirectory(std::string dir);

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

    // Runtime variant: the main view renders from the scene's primary
    // CameraComponent — the same matrix derivation the editor's game view
    // uses — instead of an editor free-camera. A scene with no primary camera
    // still presents frames (from the origin, warned once) so a bad boot scene
    // shows a window instead of appearing to hang. Meant for swapchain mode
    // (a shipped game's whole frame); in offscreen mode prefer the game view.
    virtual void RenderToSwapchain(Scene& scene, const OverlayFn& overlay = {}) = 0;

    // The scene's tonemapped LDR output texture (offscreen mode only; nullptr in
    // swapchain mode). Invalidated by Resize — re-query (and re-register with
    // ImGui) after any resize.
    virtual RHITexture* OutputColor() const = 0;

    // Rebuild the graph's render targets at a new resolution (a no-op when the
    // size is unchanged). In swapchain mode this is the window-resize hook: the
    // device recreates the backbuffer on its own, this resizes the internal
    // G-buffer/HDR chain to match. Waits for the GPU to go idle and recreates
    // the size-dependent passes — call it between frames, not mid-record, and
    // expect it to be a hitch, not a per-frame op.
    virtual void Resize(uint32_t width, uint32_t height) = 0;

    // ── Second render view: the game viewport ────────────────────────────────
    // An editor shows two images of the same scene per frame: the editor camera
    // (OutputColor) and the scene's primary CameraComponent (GameViewColor). The
    // game view is offscreen-mode only, created on first enable at the main
    // view's size, and rendered inside the same RenderToSwapchain frame — its
    // whole deferred chain runs again from the game camera, so expect roughly
    // double the GPU cost while it's on.
    virtual void SetGameViewEnabled(bool enabled) = 0;

    // The game view's LDR output. Non-null once SetGameViewEnabled(true) has run
    // (offscreen mode); invalidated by ResizeGameView — re-query after a resize.
    virtual RHITexture* GameViewColor() const = 0;

    // Whether the last RenderToSwapchain actually rendered the game view (it is
    // skipped when disabled or the scene has no primary camera — in which case
    // GameViewColor holds a stale frame that shouldn't be shown).
    virtual bool GameViewActive() const = 0;

    // Resize's counterpart for the game view (same idle-wait + pass rebuild).
    virtual void ResizeGameView(uint32_t width, uint32_t height) = 0;

    // The game view's in-game UI pass — SetUIOverlay's counterpart, drawn over
    // the game image. The two overlays run in the same frame, so they must not
    // share dynamic GPU resources (use a second Renderer2D, not the main view's).
    virtual void SetGameUIOverlay(const OverlayFn& fn) = 0;

    // An optional 2D pass drawn over the tonemapped scene (into OutputColor() in
    // offscreen mode, the backbuffer otherwise) — the in-game UI hook. Runs
    // inside an LDR render scope that loads the scene, once per frame, before
    // the swapchain overlay. Set once; capture what it draws by reference.
    virtual void SetUIOverlay(const OverlayFn& fn) = 0;

    // Global exposure: scales the HDR scene color before the tonemap curve.
    // 1.0 (the default) matches the GL renderer's tonemap exactly; lower values
    // darken the whole frame. Takes effect next frame; survives Resize.
    virtual void SetExposure(float exposure) = 0;

    // Display curve (default ACES, matching the GL renderer). Takes effect next
    // frame; survives Resize.
    virtual void SetTonemapper(Tonemapper curve) = 0;

    // Eye adaptation (default off — see AutoExposureSettings). Takes effect next
    // frame; survives Resize. Toggling on snaps to the metered luminance rather
    // than easing from a stale retained value.
    virtual void SetAutoExposure(const AutoExposureSettings& settings) = 0;

    // Bloom bright-pass cutoff in linear HDR luminance (default 1.0) and the
    // strength the blurred result is added back at (default 1.0). Threshold is
    // absolute, so it wants re-tuning whenever scene exposure changes materially
    // — until auto-exposure lands and it can key off the adapted level.
    virtual void SetBloomParams(float threshold, float intensity) = 0;

    // Temporal anti-aliasing (default on). Checked per frame: off = no
    // projection jitter, no history blend, no history copy — the resolve pass
    // degrades to a passthrough. Re-enabling self-seeds over two frames (one
    // passthrough frame, then accumulation resumes); survives Resize.
    virtual void SetTAAEnabled(bool enabled) = 0;

    // Screen-space GI (default on). Off skips the trace/temporal dispatches
    // outright and the composite passes the lit scene through unchanged, so the
    // ambient falls back to plain IBL. Re-enabling reseeds the accumulation.
    virtual void SetSSGIEnabled(bool enabled) = 0;
    // Ray count per half-res pixel, gathered radiance scale, and view-space
    // march distance. Live-tunable; the defaults are 8 / 1.0 / 8.0.
    // 'strength' scales how far the composite moves the image off the far-field
    // baseline (see VulkanSSGIPass::SetStrength) — the lever for "the GI is too
    // dark" that does not just crank exposure back up over the whole frame.
    virtual void SetSSGIParams(int rayCount, float intensity, float maxDistance,
                               float strength) = 0;

    // Ray-tracing bring-up view (Docs/gi-design.md slice 2), default off: traces
    // one primary ray per pixel against the scene TLAS and replaces the finished
    // image with the hit distance (white = near, black = miss). A no-op on
    // hardware without ray tracing — check SupportsRayTracing() before offering
    // it in UI. 'maxDistance' is both the ray length and the ramp's far end.
    virtual void SetRTDebugEnabled(bool enabled) = 0;
    virtual void SetRTDebugMaxDistance(float distance) = 0;

    // DDGI probes (Docs/gi-design.md slice 4), default on but inert until the
    // scene has a DDGIVolumeComponent. When on — and only when ray tracing and a
    // volume are both present — probe irradiance REPLACES the IBL cubemap as the
    // far-field indirect diffuse term, which is the tier-2 path. Off falls back
    // to IBL, so this is the tier 2 vs tier 1 A/B. Per-volume tuning lives on
    // the component, in the inspector, not here.
    virtual void SetDDGIEnabled(bool enabled) = 0;

    // Ray-traced reflections (Docs/rt-reflections-design.md), default on but
    // inert without ray-tracing hardware. RT is a FALLBACK under SSR: it fills
    // only the pixels SSR's screen-space march gave up on — off-screen
    // geometry, backfaces, rays that leave the frame. Off hands the composite
    // SSR's result unchanged, which is the A/B for what RT is contributing.
    virtual void SetRTReflectionsEnabled(bool enabled) = 0;
    // Whether the scene actually has a probe volume — lets the editor say
    // "add a DDGI Volume" instead of showing dead controls.
    virtual bool HasDDGIVolume() const = 0;
    // Whether this device can ray trace at all — the GI tier-2 gate.
    virtual bool SupportsRayTracing() const = 0;
    // Which GI tier is actually live this frame: 2 = DDGI probes, 1 = IBL
    // far field. Editor-facing readout, so the A/B says what it is doing.
    virtual int  GIMode() const = 0;

    // How strongly screen-space occlusion attenuates the INDIRECT DIFFUSE term
    // (0..1, default 1). It does not multiply the whole ambient: baked AO still
    // does, and the specular term gets a roughness-aware occlusion instead,
    // because a hemispherical AO factor blacks out sharp reflections in any
    // crevice. Applied at face value in every tier since slice 4.5 — the bent
    // normal makes it a solid-angle fraction, not a second occlusion test.
    virtual void  SetSSAOStrength(float strength) = 0;
    virtual float GetSSAOStrength() const = 0;

    // GTAO quality/look (Docs/gi-design.md slice 4.5). 'radius' is the
    // world-space horizon-search radius; 'slices' and 'steps' are directions
    // per pixel and taps per direction, both linear in cost. 'bentNormals' off
    // makes the pass emit the geometric normal instead of the average
    // unoccluded direction, which reverts the indirect diffuse lookup to its
    // pre-4.5 behaviour — the A/B for judging what the bent normal buys.
    // Defaults are 0.8 / 3 / 6 / on.
    virtual void SetGTAOParams(float radius, int slices, int steps,
                               bool bentNormals) = 0;

    // Live material edits (Inspector): drop the baked G-buffer descriptor set
    // for 'mat' so the next frame rebuilds it from the material's current
    // textures/params. A no-op if the material was never drawn (nothing baked
    // yet to invalidate). Call on edit-release, not per-keystroke — this stalls
    // the GPU (WaitIdle) to safely retire descriptor sets a prior frame may
    // still be reading.
    virtual void InvalidateMaterial(const PBRMaterial* mat) = 0;

    // Scene (re)load: every Mesh/PBRMaterial the old scene owned is destroyed,
    // and the allocator readily recycles their addresses for the new scene's
    // objects — a pointer-keyed cache hit on such an address would silently
    // draw the old scene's geometry/textures. Drops every cache keyed on
    // scene-owned pointers (meshes, materials, transparency albedo sets,
    // per-entity skinning state); all rebuild lazily over the next frame.
    // Same WaitIdle stall as InvalidateMaterial — scene-load rate, so fine.
    virtual void InvalidateSceneCaches() = 0;

    // ── Particle preview panel (editor-only) ─────────────────────────────────
    // A standalone render target for ParticlePreviewPanel: no G-buffer, lighting,
    // or shadows — just a clear-to-bgColor pass followed by a billboard draw.
    // Call before RenderToSwapchain, never mid-frame: lazily creates the target
    // on first call, or WaitIdle()s + rebuilds it on a size change (Resize's
    // policy). A no-op if width/height are unchanged from the last call.
    virtual void EnsureParticlePreview(uint32_t width, uint32_t height,
                                       const glm::vec3& bgColor) = 0;

    // The preview's per-frame-in-flight color output; nullptr until the first
    // EnsureParticlePreview call. Re-query (and re-register with ImGui) after
    // any call that changes the size.
    virtual RHITexture* ParticlePreviewColor() const = 0;

    using ParticleOverlayFn = std::function<void(ParticleRenderer&)>;

    // The preview's per-frame draw hook, run inside RenderToSwapchain against
    // the preview's own ParticleRenderer (after the clear) — 'fn' must itself
    // call Begin(view,proj) / Draw(...) / End(). Set once; capture whatever
    // panel state it reads by reference, like SetGameUIOverlay. A no-op frame
    // if EnsureParticlePreview was never called.
    virtual void SetParticlePreviewOverlay(const ParticleOverlayFn& fn) = 0;

    // The engine's soft-dot fallback texture (same one the main scene's
    // particle pass uses for emitters with no assigned texture) — for the
    // preview overlay to fall back on identically.
    virtual std::shared_ptr<Texture> DefaultParticleTexture() const = 0;

    // ── Shader hot-reload (editor only) ──────────────────────────────────────
    // Vulkan-only, and scoped to the shading passes an artist actually iterates
    // on live: G-buffer, deferred lighting, skybox, transparency, particles.
    // Shadow-depth/SSAO/tonemap/IBL-bake/thumbnail shaders stay build-time-only.
    // Recompiles via the same glslangValidator the offline build uses (shelled
    // out to, not linked in) and rebuilds the affected pipeline(s) in place —
    // WaitIdle first, so this is a hitch, not a per-frame cost, exactly like
    // Resize()/InvalidateMaterial(). A no-op if glslangValidator wasn't found
    // at build time (DIAMOND_GLSLANG_VALIDATOR undefined).
    virtual void ReloadChangedShaders() = 0;   // throttled per-file mtime poll
    virtual void ReloadAllShaders() = 0;       // force recompile — F5

    // Last-completed-frame renderer stats (Docs/profiler-panel-design.md).
    // A thin passthrough to the RHIDevice this renderer draws through — see
    // RHIDevice::GetStats().
    virtual RendererStats GetStats() const = 0;
};

} // namespace Diamond
