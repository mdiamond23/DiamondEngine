#include "Renderer/SceneRenderer.h"

#ifdef DIAMOND_ENABLE_VULKAN

// Vulkan expects clip-space depth in [0, 1] (OpenGL uses [-1, 1]); make GLM's
// perspective matrix match before any glm header is pulled in.
#define GLM_FORCE_DEPTH_ZERO_TO_ONE

#include "Renderer/RHI/RHIDevice.h"
#include "Renderer/RHI/RHICommandList.h"
#include "Renderer/RHI/RHIRenderGraph.h"
#include "Renderer/MeshData.h"
#include "Renderer/Material.h"
#include "Renderer/Frustum.h"
#include "Assets/AssetPathUtils.h"   // default IBL HDR resolves via ProjectRoot
#include "Core/Camera.h"
#include "Scene/Scene.h"
#include "Scene/Components.h"
#include "Scene/Physics/Rigidbody.h"   // RigidBodyComponent / BodyType (static-cache classification)
#include "Animation/AnimationComponents.h"

#include "Platform/Vulkan/Passes/Deferred/VulkanGBufferPass.h"
#include "Platform/Vulkan/Passes/Deferred/VulkanSSAOPass.h"
#include "Platform/Vulkan/Passes/Deferred/VulkanSSRPass.h"
#include "Platform/Vulkan/Passes/Deferred/VulkanDeferredLightingPass.h"
#include "Platform/Vulkan/Passes/Deferred/VulkanSkyboxPass.h"
#include "Platform/Vulkan/Passes/Debug/VulkanDebugDrawPass.h"
#include "Platform/Vulkan/Passes/Shadows/VulkanCSMPass.h"
#include "Platform/Vulkan/Passes/Shadows/VulkanSpotShadowPass.h"
#include "Platform/Vulkan/Passes/Shadows/VulkanPointShadowPass.h"
#include "Platform/Vulkan/Passes/Forward/VulkanTransparencyPass.h"
#include "Platform/Vulkan/Passes/PostProcess/VulkanTonemapPass.h"
#include "Platform/Vulkan/Passes/PostProcess/VulkanTAAPass.h"
#include "Platform/Vulkan/Passes/PostProcess/VulkanBloomPass.h"
#include "Platform/Vulkan/Passes/PostProcess/VulkanAutoExposurePass.h"
#include "Platform/Vulkan/Passes/IBL/VulkanIBLPass.h"
#include "Platform/Vulkan/Passes/VulkanPassCommon.h"   // RecompileSpirv (shader hot-reload)
#include "Platform/Vulkan/Resources/VulkanParticleRenderer.h"
#include "Platform/Vulkan/Resources/VulkanTexture2D.h"

#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <spdlog/spdlog.h>

#include <algorithm>
#include <array>
#include <filesystem>
#include <unordered_map>
#include <vector>

namespace Diamond {

namespace {

// Where every pass loads its SPIR-V from. Defaults to the build tree's baked
// DIAMOND_VULKAN_SHADER_DIR; a packaged runtime points it next to the exe via
// SceneRenderer::SetShaderDirectory before Create().
std::string& ShaderDir() {
    static std::string dir = DIAMOND_VULKAN_SHADER_DIR;
    return dir;
}

// gbuffer.vert's vertex inputs — a repacked subset of the engine's full Vertex
// (position + normal + UV + tangent; the bitangent is rebuilt in-shader from
// N×T like the GL vertex stage). MeshCache repacks into this layout on upload.
struct MeshVertex {
    glm::vec3 pos;
    glm::vec3 normal;
    glm::vec2 uv;
    glm::vec3 tangent;
};

// The skinned mesh vertex — MeshVertex plus glTF bone indices/weights, matching
// the skinned pipelines' vertex layout (SkinnedVertexAttributes, stride 76). Only
// meshes referenced by a SkinnedMeshComponent upload in this wider format.
struct SkinnedVertex {
    glm::vec3  pos;
    glm::vec3  normal;
    glm::vec2  uv;
    glm::vec3  tangent;
    glm::ivec4 boneIDs;
    glm::vec4  weights;
};
static_assert(sizeof(SkinnedVertex) == 76, "SkinnedVertex must match kSkinnedVertexStride");

// Bone-palette size — matches the skinned shaders' MAX_BONES and the GL renderer's
// uBones[100]. A humanoid skeleton stays well under this; extra slots are identity.
constexpr int kMaxBones = 100;

// Per-frame camera data for the G-buffer pass. view → view-space pos/normal;
// viewProj → clip space. Matches GBufferUBO in the ported mesh demo.
struct CameraUBO {
    glm::mat4 view;
    glm::mat4 viewProj;
    glm::mat4 viewProjUnjittered;   // for TAA, the viewProj matrix before the Halton jitter is applied
    glm::mat4 prevViewProjUnjittered;   // for TAA, the viewProj matrix before the Halton jitter is applied
};

// G-buffer per-draw push constants — matches Push in gbuffer.vert/gbuffer_skinned.vert.
// 128 bytes: the guaranteed-minimum Vulkan push-constant budget, so there's no
// room left in this block for anything else.
struct GBufferPushConstants {
    glm::mat4 model;
    glm::mat4 prevModel;   // TAA velocity's per-object "previous" term
};

// The shadowed range is kept tighter than the camera far plane so the cascades
// pack around the visible scene.
constexpr float kNear      = 0.1f;
constexpr float kFar       = 100.0f;
constexpr float kShadowFar = 25.0f;
constexpr uint32_t kShadowRes     = 2048;

// TAA jitter — Halton(2,3), 1-indexed (Halton(0,b) == 0, so start at i=1). Cycled
// over kTAASampleCount frames. A low-discrepancy sequence spreads sub-pixel
// samples evenly over the accumulation window; uniform random clusters and
// leaves gaps, which converges slower for the same frame count.
constexpr uint32_t kTAASampleCount = 8;

float Halton(uint32_t index, uint32_t base) {
    float result = 0.0f;
    float f = 1.0f;
    while (index > 0) {
        f /= static_cast<float>(base);
        result += f * static_cast<float>(index % base);
        index /= base;
    }
    return result;
}

// Desired sub-pixel screen-space shift for this frame, in NDC units. Applied by
// subtracting it from proj[2][0]/[2][1] (see RenderView) — adding a delta there
// shifts clip-space x/y by a constant amount independent of view-space depth,
// since the z term cancels out in the perspective divide.
glm::vec2 ComputeTAAJitter(uint64_t frameIndex, uint32_t width, uint32_t height) {
    const uint32_t haltonIndex = static_cast<uint32_t>(frameIndex % kTAASampleCount) + 1;
    const glm::vec2 texelOffset = glm::vec2(Halton(haltonIndex, 2), Halton(haltonIndex, 3)) - 0.5f;
    return texelOffset * 2.0f / glm::vec2(static_cast<float>(width), static_cast<float>(height));
}

// Per-material params, std140 layout matching gbuffer.frag's MaterialUBO
// (vec4 at offset 0, scalars at 16/20/24, padded to 32).
struct MaterialParams {
    glm::vec4 baseColorFactor { 1.0f };
    float uvScale          = 1.0f;
    float emissiveStrength = 0.0f;
    float alphaCutoff      = 0.0f;   // 0 = alpha test off (Opaque/Blend materials)
    float _pad0 = 0.0f;
};

// Fallback albedo for meshes with no material assigned: a checkerboard so
// unlit surface detail is still visible.
std::vector<uint8_t> MakeCheckerboard(uint32_t size, uint32_t cell) {
    std::vector<uint8_t> pixels(static_cast<size_t>(size) * size * 4);
    for (uint32_t y = 0; y < size; ++y) {
        for (uint32_t x = 0; x < size; ++x) {
            const bool even = ((x / cell) + (y / cell)) % 2 == 0;
            uint8_t* p = &pixels[(static_cast<size_t>(y) * size + x) * 4];
            p[0] = even ? 210 : 150;
            p[1] = even ? 210 : 150;
            p[2] = even ? 215 : 160;
            p[3] = 255;
        }
    }
    return pixels;
}

std::vector<uint8_t> MakeParticleDot(uint32_t size) {
    std::vector<uint8_t> pixels(static_cast<size_t>(size) * size * 4);
    const float center = (static_cast<float>(size) - 1.0f) * 0.5f;
    const float radius = center > 0.0f ? center : 1.0f;

    for (uint32_t y = 0; y < size; ++y) {
        for (uint32_t x = 0; x < size; ++x) {
            const float dx = (static_cast<float>(x) - center) / radius;
            const float dy = (static_cast<float>(y) - center) / radius;
            const float d2 = dx * dx + dy * dy;
            const float alpha = glm::clamp(1.0f - d2, 0.0f, 1.0f);
            const uint8_t a = static_cast<uint8_t>(alpha * alpha * 255.0f);

            uint8_t* p = &pixels[(static_cast<size_t>(y) * size + x) * 4];
            p[0] = 255;
            p[1] = 255;
            p[2] = 255;
            p[3] = a;
        }
    }
    return pixels;
}

} // namespace

// ── Concrete Vulkan implementation ───────────────────────────────────────────
//
// Renders up to two VIEWS of the same scene per frame: the main view (editor
// camera → OutputColor or the swapchain) and an optional game view (the scene's
// primary CameraComponent → GameViewColor). Both views execute inside one
// BeginFrame/EndFrame, so anything camera-dependent that lives in a dynamic GPU
// buffer (written once per frame slot) is duplicated per view: the camera UBO,
// the SSAO/lighting/skybox/transparency passes, the particle vertex stream, and
// the per-material descriptor sets that embed the camera UBO. Record-time state
// (push constants, draw lists, the CSM cascade fit) is simply re-set between the
// two graph executions, so the G-buffer/CSM/spot-shadow passes stay shared.
class SceneRendererVk final : public SceneRenderer {
public:
    SceneRendererVk(RHIDevice* device, uint32_t width, uint32_t height, bool offscreen)
        : m_Device(device), m_Offscreen(offscreen) {
        BuildShared();
        m_Views[kMainView] = CreateView(width, height,
                                        /*offscreen*/ m_Offscreen,
                                        /*editorComposite*/ m_Offscreen,
                                        /*external*/ false, kMainView);
    }

    void RegisterMesh(Mesh* key, const MeshData& data) override {
        if (!key || m_Meshes.count(key)) return;

        std::vector<MeshVertex> verts;
        verts.reserve(data.Vertices.size());
        for (const Vertex& v : data.Vertices)
            verts.push_back({ v.Position, v.Normal, v.TexCoords, v.Tangent });

        GpuMesh gm;
        RHIBufferDesc vb;
        vb.size        = verts.size() * sizeof(MeshVertex);
        vb.usage       = RHIBufferUsage::Vertex;
        vb.initialData = verts.data();
        gm.vertexBuffer = m_Device->CreateBuffer(vb);

        RHIBufferDesc ib;
        ib.size        = data.Indices.size() * sizeof(uint32_t);
        ib.usage       = RHIBufferUsage::Index;
        ib.initialData = data.Indices.data();
        gm.indexBuffer = m_Device->CreateBuffer(ib);
        gm.indexCount  = static_cast<uint32_t>(data.Indices.size());
        gm.bounds      = data.ComputeAABB();   // local-space; culled against per draw

        m_Meshes.emplace(key, std::move(gm));
    }

    void SetEnvironment(const std::string& hdrPath) override {
        m_IBL->BakeEnvironment(hdrPath);
        for (auto& v : m_Views) {
            if (!v) continue;
            v->lighting->BindIBL(*m_IBL);
            v->skybox->BindIBL(*m_IBL);
        }
    }

    RHITexture* OutputColor() const override {
        const View& main = *m_Views[kMainView];
        return m_Offscreen ? main.graph.GetTexture(main.outputColor) : nullptr;
    }

    void SetUIOverlay(const OverlayFn& fn) override     { m_UIFn = fn; }
    void SetGameUIOverlay(const OverlayFn& fn) override { m_GameUIFn = fn; }

    void SetExposure(float exposure) override {
        m_Exposure = exposure;
        for (auto& v : m_Views)
            if (v) {
                v->tonemap->SetExposure(exposure);
                // The bright pass thresholds against the exposed image, so it
                // needs the same manual term the tonemap applies.
                v->bloom->SetExposure(exposure);
            }
    }

    void SetAutoExposure(const AutoExposureSettings& settings) override {
        m_AutoExposure = settings;
        for (auto& v : m_Views)
            if (v) ApplyAutoExposureSettings(*v->autoExposure);
    }

    void SetTonemapper(Tonemapper curve) override {
        m_Tonemapper = curve;
        for (auto& v : m_Views)
            if (v) v->tonemap->SetTonemapper(curve);
    }

    void SetBloomParams(float threshold, float intensity) override {
        m_BloomThreshold = threshold;
        m_BloomIntensity = intensity;
        for (auto& v : m_Views)
            if (v) v->bloom->SetParams(threshold, intensity);
    }

    // Checked once per view per frame (RenderView): gates the projection
    // jitter and the history copy; the resolve pass itself always runs but
    // degrades to a passthrough draw while off (invalid history → alpha 1).
    void SetTAAEnabled(bool enabled) override { m_TAAEnabled = enabled; }

    // Drop the baked G-buffer descriptor set(s) for a material edited in the
    // Inspector, so the next frame's GetOrCreateMaterialSet rebuilds it from
    // the material's current textures/params. WaitIdle first: the cached sets
    // may still be read by an in-flight command buffer from a prior frame
    // (same rationale as VulkanThumbnailService::Release — rare, edit-release
    // only, so a stall is an acceptable price for correctness).
    void InvalidateMaterial(const PBRMaterial* mat) override {
        if (!mat || m_Materials.find(mat) == m_Materials.end()) return;
        m_Device->WaitIdle();
        m_Materials.erase(mat);
    }

    // See SceneRenderer.h — every map here is keyed on a pointer (or entt id)
    // the old scene owned; a new scene's allocations can land on the same
    // addresses, so stale entries don't just leak, they alias.
    void InvalidateSceneCaches() override {
        m_Device->WaitIdle();
        m_Meshes.clear();
        m_SkinnedMeshes.clear();
        m_Materials.clear();
        m_Skinned.clear();
        m_TransparentAlbedos.clear();
        for (auto& v : m_Views)
            if (v) v->transparency->DropAlbedoSets();
    }

    void EnsureParticlePreview(uint32_t width, uint32_t height, const glm::vec3& bgColor) override {
        if (width == 0 || height == 0) return;
        if (m_Preview && width == m_Preview->width && height == m_Preview->height) return;

        // Same rationale as Resize/InvalidateMaterial: the pooled texture the old
        // pass wrote may still be read by ImGui's in-flight draw data.
        if (m_Preview) m_Device->WaitIdle();

        if (!m_Preview) {
            m_Preview = std::make_unique<ParticlePreview>(m_Device);
            const std::string shaderDir = ShaderDir();
            m_Preview->particles =
                std::make_unique<VulkanParticleRenderer>(m_Device, shaderDir, RHIFormat::RGBA8);
        }
        m_Preview->width  = width;
        m_Preview->height = height;

        RHIRenderGraph& g = m_Preview->graph;
        g.ResetPasses();
        m_Preview->color = g.DeclareTexture("previewColor", { width, height, RHIFormat::RGBA8 });
        const RGTextureHandle depth =
            g.DeclareTexture("previewDepth", { width, height, RHIFormat::Depth32F });

        g.AddPass("ParticlePreview")
            .Write(m_Preview->color)
            .Write(depth)
            .SetClearColor({ bgColor.r, bgColor.g, bgColor.b, 1.0f })
            .SetExecute([this](RHICommandList* cmd) {
                m_Preview->particles->SetCommandList(cmd);
                if (m_PreviewFn) m_PreviewFn(*m_Preview->particles);
            });
        // Nothing inside this graph reads the color output — ImGui samples it
        // externally, so mark it to survive dead-pass culling.
        g.MarkOutput(m_Preview->color);
        g.Compile();
    }

    RHITexture* ParticlePreviewColor() const override {
        return m_Preview ? m_Preview->graph.GetTexture(m_Preview->color) : nullptr;
    }

    void SetParticlePreviewOverlay(const ParticleOverlayFn& fn) override { m_PreviewFn = fn; }

    std::shared_ptr<Texture> DefaultParticleTexture() const override {
        return m_DefaultParticleTexture;
    }

    void ReloadChangedShaders() override {
        bool gbuffer = false, lighting = false, skybox = false, transparency = false, particles = false;
        CheckWatch(m_GBufferWatch, gbuffer);
        CheckWatch(m_LightingWatch, lighting);
        CheckWatch(m_SkyboxWatch, skybox);
        CheckWatch(m_TransparencyWatch, transparency);
        CheckWatch(m_ParticlesWatch, particles);
        if (gbuffer || lighting || skybox || transparency || particles)
            DoReload(gbuffer, lighting, skybox, transparency, particles);
    }

    void ReloadAllShaders() override {
        DoReload(true, true, true, true, true);
    }

    RendererStats GetStats() const override { return m_Device->GetStats(); }

    void Resize(uint32_t width, uint32_t height) override {
        // Both modes: offscreen rebuilds the ImGui-sampled targets; swapchain
        // mode is a window resize (the device recreates the backbuffer itself
        // on OUT_OF_DATE — this rebuilds the internal targets to match).
        ResizeView(*m_Views[kMainView], kMainView, width, height);
    }

    void SetGameViewEnabled(bool enabled) override {
        m_GameViewEnabled = enabled && m_Offscreen;
        if (m_GameViewEnabled && !m_Views[kGameView]) {
            // First enable: build the second view at the main view's size. Sized
            // independently afterwards via ResizeGameView. Do this between
            // frames, like Resize.
            const View& main = *m_Views[kMainView];
            m_Views[kGameView] = CreateView(main.width, main.height,
                                            /*offscreen*/ true,
                                            /*editorComposite*/ false,
                                            /*external*/ true, kGameView);
        }
    }

    RHITexture* GameViewColor() const override {
        const View* game = m_Views[kGameView].get();
        return game ? game->graph.GetTexture(game->outputColor) : nullptr;
    }

    bool GameViewActive() const override { return m_GameViewActive; }

    void ResizeGameView(uint32_t width, uint32_t height) override {
        if (m_Views[kGameView])
            ResizeView(*m_Views[kGameView], kGameView, width, height);
    }

    void RenderToSwapchain(Scene& scene, const Camera& camera,
                           const OverlayFn& overlay) override {
        const View& main = *m_Views[kMainView];
        const float aspect = static_cast<float>(main.width) / static_cast<float>(main.height);
        // Explicitly the [0,1]-depth variant: plain glm::perspective resolves its
        // depth range from a per-TU define, and its inline instantiations
        // COMDAT-fold across TUs — a GL TU's [-1,1] version can silently win.
        const glm::mat4 proj = glm::perspectiveRH_ZO(glm::radians(camera.Zoom), aspect, kNear, kFar);
        RenderFrame(scene, camera.GetViewMatrix(), proj, camera.Position, overlay);
    }

    void RenderToSwapchain(Scene& scene, const OverlayFn& overlay) override {
        // Runtime path: the main view IS the game camera — same derivation as
        // the game view below, but feeding the primary view/swapchain.
        const View& main = *m_Views[kMainView];
        const float aspect = static_cast<float>(main.width) / static_cast<float>(main.height);
        glm::mat4 view(1.0f);
        glm::mat4 proj = glm::perspectiveRH_ZO(glm::radians(60.0f), aspect, kNear, kFar);
        const entt::entity camEntity = scene.GetPrimaryCamera();
        if (camEntity != entt::null) {
            const auto& cc = scene.GetRegistry().get<CameraComponent>(camEntity);
            view = glm::inverse(scene.GetTransformSystem().GetWorldMatrix(camEntity));
            proj = glm::perspectiveRH_ZO(glm::radians(cc.fov), aspect,
                                         cc.nearClip, cc.farClip);
        } else {
            static bool warned = false;
            if (!warned) {
                spdlog::warn("[SceneRenderer] scene has no primary camera — "
                             "rendering from the origin");
                warned = true;
            }
        }
        RenderFrame(scene, view, proj, glm::vec3(glm::inverse(view)[3]), overlay);
    }

    // The shared frame body behind both RenderToSwapchain overloads: everything
    // after the main view's matrices are known.
    void RenderFrame(Scene& scene, const glm::mat4& view, const glm::mat4& proj,
                     const glm::vec3& camPos, const OverlayFn& overlay) {
        // Culling (RebuildDrawList, below) runs before BeginFrame, so the frame
        // stats window brackets this whole call, not just BeginFrame/EndFrame.
        m_Device->ResetFrameStats();
        ++m_FrameIndex;

        // Frame delta for auto-exposure adaptation. Clamped: a hitch (or the
        // first frame after a scene load) must not let adaptation jump a stop.
        const auto now = std::chrono::high_resolution_clock::now();
        if (m_HasLastFrameTime) {
            const std::chrono::duration<float> elapsed = now - m_LastFrameTime;
            m_DeltaTime = std::clamp(elapsed.count(), 0.0f, 0.1f);
        }
        m_LastFrameTime    = now;
        m_HasLastFrameTime = true;

        View& main = *m_Views[kMainView];
        const Frustum mainFrustum = Frustum::Extract(proj * view, /*zeroToOneDepth*/ true);

        //scene.GetTransformSystem().Update(scene.GetRegistry());

        // Game view: rendered only when enabled AND the scene has a primary
        // camera to render from (mirrors the GL editor's game viewport).
        // Computed before GatherLights so its frustum is available for
        // light-range culling below.
        m_GameViewActive = false;
        View* game = (m_GameViewEnabled && m_Views[kGameView]) ? m_Views[kGameView].get() : nullptr;
        glm::mat4 gameView(1.0f), gameProj(1.0f);
        Frustum gameFrustum{};
        if (game) {
            const entt::entity camEntity = scene.GetPrimaryCamera();
            if (camEntity != entt::null) {
                const auto& cc = scene.GetRegistry().get<CameraComponent>(camEntity);
                gameView = glm::inverse(scene.GetTransformSystem().GetWorldMatrix(camEntity));
                gameProj = glm::perspectiveRH_ZO(
                    glm::radians(cc.fov),
                    static_cast<float>(game->width) / static_cast<float>(game->height),
                    cc.nearClip, cc.farClip);
                gameFrustum = Frustum::Extract(gameProj * gameView, /*zeroToOneDepth*/ true);
                m_GameViewActive = true;
            }
        }

        // Gather this frame's shared (view-independent) scene data: lights
        // (culled against the active camera frustum(s) — a light whose range
        // sphere misses every view skips its shadow pass and the lighting
        // pass, since both consume the vectors GatherLights fills) and
        // particle batches.
        GatherLights(scene, mainFrustum, m_GameViewActive ? &gameFrustum : nullptr);
        GatherParticles(scene);

        // Per-view draw lists (frustum-culled per camera). Built before
        // BeginFrame so lazy mesh/material uploads keep their pre-frame timing.
        // The shadow list is view-independent and shared — each shadow pass
        // culls it against its own light frustum / range at draw time.
        RebuildDrawList(scene, mainFrustum, camPos, main, kMainView);
        if (m_GameViewActive)
            RebuildDrawList(scene, gameFrustum,
                            glm::vec3(glm::inverse(gameView)[3]), *game, kGameView);

        // Snapshot this frame's transforms for next frame's TAA velocity — after
        // both draw lists above have read last frame's snapshot via PrevWorld.
        CommitPrevTransforms();

        RHICommandList* cmd = m_Device->BeginFrame();
        if (!cmd) return;   // swapchain unavailable (minimized / rebuilding)

        // Now the frame slot is idle: push this frame's animated bone palettes into
        // each skinned entity's dynamic UBO (dynamic Update must follow BeginFrame).
        UpdateSkinnedPalettes(scene);

        // View-independent shadow work, once per frame: spot and point maps are
        // light-space (unlike the CSM cascades, whose fit is per-camera), so they
        // render once into pass-owned targets before either view's graph — both
        // views' lighting sets bind the same maps, halving the shadow draws the
        // old per-view-graph spot passes recorded when the game view was live.
        // Static shadow caching: if the set/pose of static casters changed since
        // last frame, invalidate the cached static maps (the caches detect a moved
        // light themselves, but not moved/added/removed static geometry). Dynamic
        // casters are excluded from the hash, so they never force a re-bake.
        const size_t staticHash = ComputeStaticShadowHash();
        const bool   staticChanged = staticHash != m_LastStaticShadowHash;
        m_LastStaticShadowHash = staticHash;

        m_SpotShadow->ComputeMatrices(m_SpotLights);
        if (staticChanged) m_SpotShadow->MarkStaticDirty();
        cmd->BeginDebugLabel("SpotShadows");
        m_Device->BeginPassProfile("Shadows", "Spot Maps",
                                   VulkanSpotShadowPass::kResolution,
                                   VulkanSpotShadowPass::kResolution);
        m_SpotShadow->Record(cmd,
            [this](RHICommandList* c, const glm::mat4& lightSpace) {   // static cache bake
                DrawShadow(c, lightSpace, m_SpotShadow->SkinnedPipeline(), ShadowFilter::StaticOnly);
            },
            [this](RHICommandList* c, const glm::mat4& lightSpace) {   // dynamic, over the cache
                DrawShadow(c, lightSpace, m_SpotShadow->SkinnedPipeline(), ShadowFilter::DynamicOnly);
            });
        m_Device->EndPassProfile();
        cmd->EndDebugLabel();
        m_PointShadow->SetLights(m_PointPos);   // also marks moved-light slots dirty
        if (staticChanged) m_PointShadow->MarkStaticDirty();
        cmd->BeginDebugLabel("PointShadows");
        m_Device->BeginPassProfile("Shadows", "Point Cubes",
                                   VulkanPointShadowPass::kResolution,
                                   VulkanPointShadowPass::kResolution);
        m_PointShadow->Record(cmd,
            [this](RHICommandList* c, uint32_t faceIdx) {   // static cache bake
                DrawPointShadow(c, faceIdx, ShadowFilter::StaticOnly);
            },
            [this](RHICommandList* c, uint32_t faceIdx) {   // dynamic, over the cache
                DrawPointShadow(c, faceIdx, ShadowFilter::DynamicOnly);
            });
        m_Device->EndPassProfile();
        cmd->EndDebugLabel();

        // Offscreen mode routes the overlay through the graph's EditorOverlay
        // pass (which owns the backbuffer and has already transitioned the scene
        // output for sampling); the pass callback reads this member per frame.
        m_Overlay = overlay;

        // Game view first: the main view's editor overlay (ImGui) samples the
        // game output, so it must be rendered — and transitioned — before then.
        if (m_GameViewActive)
            RenderView(*game, gameView, gameProj, cmd);
        RenderView(main, view, proj, cmd);

        // Particle preview panel: independent of the scene views above, same
        // command buffer/frame — no extra submission needed.
        if (m_Preview) m_Preview->graph.Execute(cmd);

        // Swapchain mode: composite an optional overlay (ImGui) over the
        // tonemapped backbuffer, loading its contents so the scene shows through.
        // The backbuffer is still in COLOR_ATTACHMENT layout here; EndFrame
        // transitions it to present.
        if (!m_Offscreen && overlay) {
            RHIRenderPass pass;
            pass.toSwapchain = true;
            RHIAttachment color;
            color.clear = false;   // load — draw on top of the scene
            pass.colorAttachments.push_back(color);
            cmd->BeginRendering(pass);
            overlay(cmd);
            cmd->EndRendering();
        }

        m_Device->EndFrame();
        m_Device->FinalizeFrameStats();
    }

private:
    static constexpr int kMainView = 0;
    static constexpr int kGameView = 1;
    static constexpr int kMaxViews = 2;

    struct GpuMesh {
        std::unique_ptr<RHIBuffer> vertexBuffer;
        std::unique_ptr<RHIBuffer> indexBuffer;
        uint32_t                   indexCount = 0;
        AABB                       bounds;
    };
    // A cached material: the static params buffer, the engine textures kept
    // alive, and one descriptor set PER VIEW against the G-buffer pipeline
    // (each set embeds that view's camera UBO — no RHI multi-set support).
    // Baked lazily per (PBRMaterial pointer, view) — a material edited after
    // first use keeps its baked look until re-registered (live material editing
    // is a later slice).
    struct GpuMaterial {
        std::unique_ptr<RHIBuffer> params;
        std::array<std::unique_ptr<RHIResourceSet>, kMaxViews> sets{};
        std::array<std::shared_ptr<Texture>, VulkanGBufferPass::MapCount> keepAlive{};
    };
    struct DrawItem {
        const GpuMesh*  mesh;
        RHIResourceSet* material;   // G-buffer set 0 for this draw (view-specific)
        glm::mat4       world;
        glm::mat4       prevWorld;  // last frame's world — TAA velocity (see m_PrevWorld)
    };
    // A shadow caster — depth passes need no material, but each light view culls
    // against the caster's world-space bounds (cached here once per rebuild).
    struct ShadowItem {
        const GpuMesh* mesh;
        glm::mat4      world;
        AABB           worldBounds;
        bool           isStatic;   // eligible for a light's cached static map (see
                                   // MeshComponent::staticShadowCaster). Skinned
                                   // casters are always dynamic (separate list).
    };
    struct TransparentDraw {
        const GpuMesh*  mesh;
        RHIResourceSet* set;        // transparency set 0 (camera UBO + albedo)
        glm::mat4       world;
        float           dist;       // camera distance — back-to-front sort key
    };
    // A skinned draw: the material set 0 (reused from the static material cache) +
    // the entity's set-1 bone palette. Bound under the skinned G-buffer pipeline.
    struct SkinnedDrawItem {
        const GpuMesh*  mesh;
        RHIResourceSet* material;   // set 0 (view-specific)
        RHIResourceSet* bones;      // set 1 (per entity, view-independent)
        glm::mat4       world;
        glm::mat4       prevWorld;  // last frame's world — TAA velocity (see m_PrevWorld).
                                    // Captures entity-level motion only, not per-bone
                                    // animation — the bone palette isn't double-buffered.
    };
    // A skinned shadow caster — depth passes need only the bone set + world (no
    // material). Shared across views/lights like the static m_ShadowDraws.
    struct SkinnedShadowItem {
        const GpuMesh*  mesh;
        RHIResourceSet* bones;
        glm::mat4       world;
        AABB            worldBounds;   // entity bounds — shared by all its primitives
    };
    // Per-skinned-entity GPU state: a dynamic bone-palette UBO (rewritten each
    // frame from AnimatorComponent::palette) + its set-1 descriptor. Created lazily,
    // keyed by entity, and kept for the renderer's lifetime (skinned entities are
    // few; a destroyed entity's slot simply lingers).
    struct SkinnedInstance {
        std::unique_ptr<RHIBuffer>      bonesUBO;   // dynamic mat4[kMaxBones]*2 (current | previous)
        std::unique_ptr<RHIResourceSet> bonesSet;   // set 1, binding 0 = bonesUBO
        // Last frame's palette, kept CPU-side so UpdateSkinnedPalettes can upload
        // it as the UBO's second half — per-bone TAA velocity. hasPrev false =
        // first frame: previous is seeded from current (zero velocity).
        std::array<glm::mat4, kMaxBones> prevPalette;
        bool hasPrev = false;
    };
    struct PBatch {
        RHITexture* tex = nullptr;
        std::shared_ptr<Texture> texAsset;
        ParticleBlend blend = ParticleBlend::Alpha;
        std::vector<RenderParticle> parts;
    };

    // One render view: a full deferred chain from one camera into one output.
    // Holds everything camera-dependent that lives in a dynamic GPU buffer (see
    // the class comment); the stateless/record-time passes (G-buffer, CSM, spot
    // and point shadows) are shared across views.
    struct View {
        explicit View(RHIDevice* device) : graph(device) {}

        uint32_t width  = 0;
        uint32_t height = 0;
        bool offscreen       = false;  // tonemap → outputColor instead of the swapchain
        bool editorComposite = false;  // owns/clears the backbuffer + runs m_Overlay
        bool external        = false;  // output sampled outside this graph (game view)

        RHIRenderGraph  graph;
        RGTextureHandle outputColor;   // valid when offscreen

        // Last frame's unjittered viewProj for THIS camera — TAA velocity's
        // "previous" term. Per-view (not global) since main/game are different
        // cameras. hasPrevViewProj is false until RenderView has run once, so
        // the first frame seeds prevViewProj from the current frame instead of
        // reading identity (which would report a spurious full-screen velocity).
        glm::mat4 prevViewProj { 1.0f };
        bool      hasPrevViewProj = false;

        std::unique_ptr<RHIBuffer>                  cameraUBO;
        std::unique_ptr<VulkanSSAOPass>             ssao;
        std::unique_ptr<VulkanSSRPass>              ssr;
        std::unique_ptr<VulkanTAAPass>              taa;
        std::unique_ptr<VulkanDeferredLightingPass> lighting;
        std::unique_ptr<VulkanSkyboxPass>           skybox;
        std::unique_ptr<VulkanTransparencyPass>     transparency;
        std::unique_ptr<VulkanAutoExposurePass>     autoExposure;
        std::unique_ptr<VulkanBloomPass>            bloom;
        std::unique_ptr<VulkanTonemapPass>          tonemap;
        std::unique_ptr<VulkanParticleRenderer>     particles;
        std::unique_ptr<VulkanDebugDrawPass>        debugDraw;   // main view only

        std::vector<DrawItem>        drawList;          // frustum-culled — geometry pass
        std::vector<SkinnedDrawItem> skinnedDraws;      // frustum-culled — skinned geometry
        std::vector<TransparentDraw> transparentDraws;  // back-to-front — forward pass
    };

    // The particle-preview panel's standalone target: one graph, one pass
    // (clear + particles), no G-buffer/lighting/shadows. Lives outside
    // m_Views — it isn't a scene camera, just a billboard canvas.
    struct ParticlePreview {
        explicit ParticlePreview(RHIDevice* device) : graph(device) {}
        uint32_t width  = 0;
        uint32_t height = 0;
        RHIRenderGraph  graph;
        RGTextureHandle color;
        std::unique_ptr<VulkanParticleRenderer> particles;
    };

    // Shared, view-independent resources: default textures, the stateless
    // passes, the IBL bake, and the particle fallback texture.
    void BuildShared() {
        // Default material maps: checkerboard albedo for meshes with no material,
        // plus 1×1 neutral fallbacks for any slot a material leaves empty (flat
        // normal, non-metallic, mid roughness, full AO, no emissive).
        const std::vector<uint8_t> pixels = MakeCheckerboard(256, 32);
        RHITextureDesc tex;
        tex.width       = 256;
        tex.height      = 256;
        tex.format      = RHIFormat::RGBA8;
        tex.usage       = RHITextureUsage::Sampled;
        tex.initialData = pixels.data();
        m_Albedo = m_Device->CreateTexture(tex);

        auto makePixel = [this](uint8_t r, uint8_t g, uint8_t b) {
            const uint8_t px[4] = { r, g, b, 255 };
            RHITextureDesc one;
            one.width       = 1;
            one.height      = 1;
            one.format      = RHIFormat::RGBA8;
            one.usage       = RHITextureUsage::Sampled;
            one.initialData = px;
            return m_Device->CreateTexture(one);
        };
        m_DefaultWhite  = makePixel(255, 255, 255);   // albedo fallback, AO = 1
        m_DefaultNormal = makePixel(128, 128, 255);   // tangent-space +Z
        m_DefaultBlack  = makePixel(0, 0, 0);         // metallic = 0, emissive off
        m_DefaultGray   = makePixel(128, 128, 128);   // roughness = 0.5

        const std::string shaderDir = ShaderDir();
        m_GBuffer     = std::make_unique<VulkanGBufferPass>(m_Device, shaderDir);
        m_CSM         = std::make_unique<VulkanCSMPass>(m_Device, shaderDir);
        m_SpotShadow  = std::make_unique<VulkanSpotShadowPass>(m_Device, shaderDir);
        m_PointShadow = std::make_unique<VulkanPointShadowPass>(m_Device, shaderDir);

        const std::vector<uint8_t> dotPixels = MakeParticleDot(64);

        RHITextureDesc particleTex;
        particleTex.width       = 64;
        particleTex.height      = 64;
        particleTex.format      = RHIFormat::RGBA8;
        particleTex.usage       = RHITextureUsage::Sampled;
        particleTex.initialData = dotPixels.data();
        particleTex.filter      = RHIFilter::Linear;

        m_DefaultParticleRHI = m_Device->CreateTexture(particleTex);

        m_DefaultParticleTexture = std::make_shared<VulkanTexture2D>(
            m_Device, dotPixels.data(), 64, 64, 4, RHIFilter::Linear);

        // Bake a default environment so the deferred ambient term is valid even if
        // the caller never sets one. SetEnvironment can re-bake later.
        m_IBL = std::make_unique<VulkanIBLPass>(m_Device, shaderDir);
        m_IBL->BakeEnvironment(AssetPaths::Resolve(
            "Assets/Textures/citrus_orchard_road_puresky_4k.hdr"));
    }

    void ApplyAutoExposureSettings(VulkanAutoExposurePass& pass) const {
        pass.SetEnabled(m_AutoExposure.enabled);
        pass.SetKeyValue(m_AutoExposure.keyValue);
        pass.SetRange(m_AutoExposure.minLogLuminance, m_AutoExposure.maxLogLuminance);
        pass.SetSpeed(m_AutoExposure.speed);
    }

    // Bloom runs its extract + blur chain at half resolution: the result is
    // low-frequency, so the halved cost and memory cost nothing visually, and the
    // wider effective blur radius per tap actually helps the glow spread.
    static uint32_t BloomScale(uint32_t x) { return std::max(1u, x / 2); }

    std::unique_ptr<View> CreateView(uint32_t width, uint32_t height,
                                     bool offscreen, bool editorComposite,
                                     bool external, int viewIdx) {
        auto v = std::make_unique<View>(m_Device);
        v->width           = width;
        v->height          = height;
        v->offscreen       = offscreen;
        v->editorComposite = editorComposite;
        v->external        = external;

        // Per-view camera UBO (dynamic — rewritten every frame), embedded in the
        // view's material descriptor sets.
        RHIBufferDesc ubo;
        ubo.size    = sizeof(CameraUBO);
        ubo.usage   = RHIBufferUsage::Uniform;
        ubo.dynamic = true;
        v->cameraUBO = m_Device->CreateBuffer(ubo);

        const std::string shaderDir = ShaderDir();
        v->ssao         = std::make_unique<VulkanSSAOPass>(m_Device, shaderDir, width, height);
        v->ssr          = std::make_unique<VulkanSSRPass>(m_Device, shaderDir, width, height);
        v->taa          = std::make_unique<VulkanTAAPass>(m_Device, shaderDir, width, height);
        v->lighting     = std::make_unique<VulkanDeferredLightingPass>(m_Device, shaderDir);
        v->skybox       = std::make_unique<VulkanSkyboxPass>(m_Device, shaderDir);
        v->transparency = std::make_unique<VulkanTransparencyPass>(m_Device, shaderDir);
        // Bloom composites into an HDR target, so it always outputs RGBA16F —
        // tonemapping stays downstream regardless of the view's final format.
        // Metering chain is a fixed size — nothing about it depends on the view.
        v->autoExposure = std::make_unique<VulkanAutoExposurePass>(m_Device, shaderDir);
        ApplyAutoExposureSettings(*v->autoExposure);
        v->bloom = std::make_unique<VulkanBloomPass>(
            m_Device, shaderDir, BloomScale(width), BloomScale(height), RHIFormat::RGBA16F);
        v->bloom->SetParams(m_BloomThreshold, m_BloomIntensity);
        v->bloom->SetExposure(m_Exposure);
        // Offscreen views tonemap into an LDR texture ImGui can sample; the
        // swapchain-mode main view tonemaps straight into the backbuffer.
        v->tonemap = std::make_unique<VulkanTonemapPass>(
            m_Device, shaderDir,
            offscreen ? RHIFormat::RGBA8 : m_Device->SwapchainFormat());
        v->tonemap->SetExposure(m_Exposure);
        v->tonemap->SetTonemapper(m_Tonemapper);
        v->particles = std::make_unique<VulkanParticleRenderer>(m_Device, shaderDir, RHIFormat::RGBA16F);

        // Collider/ragdoll/IK/audio debug wireframes — only the main (editor)
        // view draws them, matching the GL editor's DrawColliders/DrawAudioDebug
        // + DebugDraw::Flush, which only ever targets the editor viewport FBO.
        if (viewIdx == kMainView)
            v->debugDraw = std::make_unique<VulkanDebugDrawPass>(
                m_Device, shaderDir,
                offscreen ? RHIFormat::RGBA8 : m_Device->SwapchainFormat());

        BuildViewGraph(*v, viewIdx);
        return v;
    }

    void ResizeView(View& v, int viewIdx, uint32_t width, uint32_t height) {
        if (width == 0 || height == 0) return;
        if (width == v.width && height == v.height) return;

        // Nothing referencing the old targets may be in flight while the pool
        // recreates them and the size-dependent passes rebuild their sets.
        m_Device->WaitIdle();
        v.width  = width;
        v.height = height;

        // Recreate the passes whose descriptor sets sample graph textures (the
        // pooled textures are about to be recreated at the new size). The other
        // passes (G-buffer, shadows, skybox, transparency, particles) hold no
        // graph-texture sets and their pipelines are size-independent, so they
        // re-wire as-is.
        const std::string shaderDir = ShaderDir();
        v.ssao     = std::make_unique<VulkanSSAOPass>(m_Device, shaderDir, width, height);
        v.ssr      = std::make_unique<VulkanSSRPass>(m_Device, shaderDir, width, height);
        // TAA holds a size-dependent history texture; recreating it also resets
        // its history-valid flag, so the first post-resize frame passes through.
        v.taa      = std::make_unique<VulkanTAAPass>(m_Device, shaderDir, width, height);
        v.lighting = std::make_unique<VulkanDeferredLightingPass>(m_Device, shaderDir);
        v.autoExposure = std::make_unique<VulkanAutoExposurePass>(m_Device, shaderDir);
        ApplyAutoExposureSettings(*v.autoExposure);
        v.bloom    = std::make_unique<VulkanBloomPass>(
            m_Device, shaderDir, BloomScale(width), BloomScale(height), RHIFormat::RGBA16F);
        v.bloom->SetParams(m_BloomThreshold, m_BloomIntensity);
        v.bloom->SetExposure(m_Exposure);
        // Same format split as CreateView: offscreen tonemaps into the LDR
        // texture, swapchain mode straight into the backbuffer.
        v.tonemap  = std::make_unique<VulkanTonemapPass>(
            m_Device, shaderDir,
            v.offscreen ? RHIFormat::RGBA8 : m_Device->SwapchainFormat());
        v.tonemap->SetExposure(m_Exposure);   // the recreated pass defaults to 1.0
        v.tonemap->SetTonemapper(m_Tonemapper);

        v.graph.ResetPasses();
        BuildViewGraph(v, viewIdx);   // re-declares textures + re-binds IBL/point shadows
    }

    void BuildViewGraph(View& v, int viewIdx) {
        RHIRenderGraph& g = v.graph;
        // Scene views are profiled per pass (the preview/thumbnail graphs are
        // not — they'd inject sporadic rows into the breakdown).
        g.SetProfileScope(viewIdx == kGameView ? "Game View" : "Main View");
        const RGTextureHandle gViewPos    = g.DeclareTexture("gViewPos",    { v.width, v.height, RHIFormat::RGBA16F });
        const RGTextureHandle gViewNormal = g.DeclareTexture("gViewNormal", { v.width, v.height, RHIFormat::RGBA16F });
        const RGTextureHandle gAlbedo     = g.DeclareTexture("gAlbedo",     { v.width, v.height, RHIFormat::RGBA8   });
        const RGTextureHandle gMaterial   = g.DeclareTexture("gMaterial",   { v.width, v.height, RHIFormat::RGBA8   });
        const RGTextureHandle gEmissive   = g.DeclareTexture("gEmissive",   { v.width, v.height, RHIFormat::RGBA16F });
        const RGTextureHandle gVelocity   = g.DeclareTexture("gVelocity",   { v.width, v.height, RHIFormat::RG16F   });
        const RGTextureHandle gDepth      = g.DeclareTexture("gDepth",      { v.width, v.height, RHIFormat::Depth32F });
        const RGTextureHandle ssaoRaw     = g.DeclareTexture("ssaoRaw",     { v.width, v.height, RHIFormat::R16F     });
        const RGTextureHandle ssaoBlurred = g.DeclareTexture("ssaoBlurred", { v.width, v.height, RHIFormat::R16F     });
        const RGTextureHandle ssrColor    = g.DeclareTexture("ssrColor",    { v.width, v.height, RHIFormat::RGBA16F });
        const RGTextureHandle hdrLit      = g.DeclareTexture("hdrLit",      { v.width, v.height, RHIFormat::RGBA16F });
        // Scene + reflections resolved by SSRComposite. A separate target (not a
        // blend into hdrLit) because the graph points readers at a texture's LAST
        // writer — the SSR trace reading hdrLit while a later pass writes it back
        // would be a dependency cycle. Everything after SSR consumes this.
        const RGTextureHandle hdrSSR      = g.DeclareTexture("hdrSSR",      { v.width, v.height, RHIFormat::RGBA16F });
        // TAA-resolved scene — same separate-target rule as hdrSSR. Everything
        // after the resolve (transparency/particles/tonemap) consumes this.
        const RGTextureHandle hdrTAA      = g.DeclareTexture("hdrTAA",      { v.width, v.height, RHIFormat::RGBA16F });
        // Second MRT copy of the resolve, untouched by transparency/particles —
        // the post-graph history copy reads this, never hdrTAA (see VulkanTAAPass).
        const RGTextureHandle taaHistorySrc = g.DeclareTexture("taaHistorySrc", { v.width, v.height, RHIFormat::RGBA16F });
        // Scene + bloom, still HDR. Separate target for the same reason as hdrSSR:
        // the composite reads the scene it adds onto, so it can't write it back.
        const RGTextureHandle hdrBloom    = g.DeclareTexture("hdrBloom",    { v.width, v.height, RHIFormat::RGBA16F });
        // 1x1 auto-exposure multiplier, produced by the metering chain and read
        // by both the bloom bright pass and the tonemap.
        const RGTextureHandle exposureTex = g.DeclareTexture("exposure",    { 1, 1, RHIFormat::R16F });

        std::array<RGTextureHandle, VulkanCSMPass::NUM_CASCADES> cascades;
        for (int i = 0; i < VulkanCSMPass::NUM_CASCADES; ++i)
            cascades[i] = g.DeclareTexture("csmCascade" + std::to_string(i),
                                           { kShadowRes, kShadowRes, RHIFormat::Depth32F });

        // G-buffer — fill from the view's draw list; each draw binds its
        // material's descriptor set (built lazily by GetOrCreateMaterialSet).
        m_GBuffer->AddToGraph(g, gViewPos, gViewNormal, gAlbedo, gMaterial, gEmissive,
                              gVelocity, gDepth,
                              [this, &v](RHICommandList* cmd) { DrawGeometry(cmd, v); });

        // SSAO occlusion + blur over the G-buffer.
        v.ssao->AddToGraph(g, gViewPos, gViewNormal, ssaoRaw, ssaoBlurred);

        // Shadow cascades — scene depth from the sun; push lightSpace * model.
        // The pass is shared across views: its light matrices are read at record
        // time, refreshed by ComputeCascades right before each view executes.
        m_CSM->AddToGraph(g, cascades,
                          [this](RHICommandList* cmd, const glm::mat4& lightSpace) {
                              DrawShadow(cmd, lightSpace, m_CSM->SkinnedPipeline());
                          });

        // Deferred resolve → HDR. Reads all cascade maps (also keeps the unlit
        // ones alive). The spot maps and point-shadow cubes aren't graph
        // textures — both render once pre-graph in RenderToSwapchain; the spot
        // maps bind as pass-owned RHITextures, the cubes bind raw below.
        v.lighting->AddToGraph(g, gViewPos, gViewNormal, gAlbedo, gMaterial,
                               ssaoBlurred, gEmissive, cascades,
                               m_SpotShadow->Maps(), hdrLit);
        v.lighting->BindIBL(*m_IBL);
        v.lighting->BindPointShadows(*m_PointShadow);

        // Skybox fills the background pixels (far plane, LessEqual against the
        // G-buffer depth) with the baked env cube — the same map the IBL ambient
        // samples, so reflections have a visible source. Inserted after the
        // lighting resolve and before particles; the writes on the shared HDR
        // target resolve by insertion order.
        v.skybox->AddToGraph(g, hdrLit, gDepth);
        v.skybox->BindIBL(*m_IBL);

        // SSR — reflections ray-marched off the lit scene (post-skybox so rays
        // can hit the sky), then material-weighted and resolved with the scene
        // into hdrSSR. Later passes composite over hdrSSR, not hdrLit.
        v.ssr->AddToGraph(g, gViewPos, gViewNormal, hdrLit, ssrColor, gMaterial, hdrSSR);

        // TAA — accumulates the jittered frames against its pass-owned history,
        // reprojected through gVelocity. Before transparency/particles: neither
        // has motion vectors, so they draw over the resolved image rather than
        // smearing into the history. Its history copy is recorded raw after
        // graph.Execute (RenderView), not as a graph pass — taaHistorySrc's
        // consumer is outside the graph, so keep it from being culled.
        v.taa->AddToGraph(g, hdrSSR, gVelocity, hdrTAA, taaHistorySrc);
        g.MarkOutput(taaHistorySrc);

        // Forward transparency blends over the resolved scene, depth-testing
        // (not writing) against the G-buffer depth — before particles, matching
        // the GL frame order (lighting → skybox → transparency → particles).
        v.transparency->AddToGraph(g, hdrTAA, gDepth,
            [this, &v](RHICommandList* cmd) { DrawTransparent(cmd, v); });

        // Particles composite over the lit HDR scene (before tonemap). The
        // frame view/proj members are re-set per view before its Execute.
        v.particles->AddToGraph(g, hdrTAA, gDepth,
            [this](VulkanParticleRenderer& particles) {
                particles.Begin(m_FrameView, m_FrameProj);
                for (PBatch& b : m_ParticleBatches) {
                    if (!b.texAsset || b.parts.empty()) continue;
                    particles.Draw(b.parts, *b.texAsset, b.blend);
                }
                particles.End();
            });

        // Auto-exposure — meters the finished HDR scene (post transparency and
        // particles, so emissive VFX count toward the average) down to a 1x1
        // multiplier. Must precede bloom: the bright pass thresholds against the
        // exposed image, so it consumes this frame's multiplier.
        v.autoExposure->AddToGraph(g, hdrTAA, exposureTex);

        // Bloom — bright-pass extract + separable blur at half res, added back onto
        // the scene. Last thing before tonemap so emissive surfaces, transparency
        // and particles all glow; the bright pass reads linear HDR, which is the
        // whole point of running it ahead of the tonemap curve.
        v.bloom->AddToGraph(g, hdrTAA, exposureTex, hdrBloom);

        // Tonemap HDR → swapchain (curve + gamma), or → the LDR output texture
        // for offscreen views.
        if (v.offscreen)
            v.outputColor = g.DeclareTexture("outputColor", { v.width, v.height, RHIFormat::RGBA8 });
        v.tonemap->AddToGraph(g, hdrBloom, exposureTex,
                              v.offscreen ? v.outputColor : RGTextureHandle{});

        // Collider/ragdoll/IK/audio debug wireframes — on top of the tonemapped
        // LDR scene, read-only depth test against the G-buffer depth. Matches
        // the GL editor's frame order (scene -> debug draw -> UI canvas).
        if (v.debugDraw)
            v.debugDraw->AddToGraph(g, v.outputColor, /*toSwapchain*/ !v.offscreen, gDepth);

        // In-game UI — a 2D pass over the tonemapped LDR scene. Loads (doesn't
        // clear) so widgets composite on top. Ordering: inserted after Tonemap so
        // the write-after-write on the shared target resolves by insertion order.
        {
            RGPass& ui = g.AddPass("UI2D");
            if (v.offscreen) ui.Write(v.outputColor);
            else             ui.WriteSwapchain();
            ui.Load().SetExecute([this, viewIdx](RHICommandList* cmd) {
                const OverlayFn& fn = viewIdx == kGameView ? m_GameUIFn : m_UIFn;
                if (fn) fn(cmd);
            });
        }

        // Editor composite — the only backbuffer owner in offscreen mode. Reading
        // outputColor transitions it to SampledRead, so the overlay (ImGui) can
        // sample it as a viewport image while drawing over a cleared swapchain.
        if (v.editorComposite) {
            g.AddPass("EditorOverlay")
                .Read(v.outputColor)
                .WriteSwapchain()
                .SetClearColor({ 0.1f, 0.1f, 0.1f, 1.0f })
                .SetExecute([this](RHICommandList* cmd) {
                    if (m_Overlay) m_Overlay(cmd);
                });
        }

        // The game view's output is consumed by the main view's editor overlay
        // (ImGui) — no reader in THIS graph, so keep its writers from culling.
        if (v.external)
            g.MarkOutput(v.outputColor);

        g.Compile();
    }

    // Record one view: refresh its dynamic UBOs + the shared CSM cascade fit for
    // this camera, then execute its graph. Runs between BeginFrame and EndFrame;
    // each view's dynamic buffers are its own, so two views in one frame never
    // clobber each other's data.
    void RenderView(View& v, const glm::mat4& view, const glm::mat4& proj,
                    RHICommandList* cmd) {
        // Sub-pixel jitter for TAA — everything that ends up in this view's HDR
        // target uses jitteredProj; CSM and debug-draw stay on the clean 'proj'
        // (shadow cascades and wireframes have no business jittering). With TAA
        // off there's no resolve to average the jitter away, so it must be zero
        // or the whole image vibrates.
        const glm::vec2 jitter = m_TAAEnabled
            ? ComputeTAAJitter(m_FrameIndex, v.width, v.height)
            : glm::vec2(0.0f);
        glm::mat4 jitteredProj = proj;
        jitteredProj[2][0] -= jitter.x;
        jitteredProj[2][1] -= jitter.y;

        // Read at record time by the particle pass callback during Execute.
        m_FrameView = view;
        m_FrameProj = jitteredProj;

        // TAA velocity's two clean (unjittered) viewProj terms. Cold-start: seed
        // prevViewProj from this frame so frame 1 reads zero velocity instead of
        // diffing against identity.
        const glm::mat4 unjitteredViewProj = proj * view;
        if (!v.hasPrevViewProj) {
            v.prevViewProj    = unjitteredViewProj;
            v.hasPrevViewProj = true;
        }

        CameraUBO ubo{};
        ubo.view                    = view;
        ubo.viewProj                = jitteredProj * view;
        ubo.viewProjUnjittered      = unjitteredViewProj;
        ubo.prevViewProjUnjittered  = v.prevViewProj;
        v.cameraUBO->Update(&ubo, sizeof(ubo));

        v.ssao->SetProjection(jitteredProj);
        v.ssr->SetProjection(jitteredProj);
        v.skybox->SetFrameData(view, jitteredProj);
        v.transparency->SetCamera(view, jitteredProj);
        m_CSM->ComputeCascades(m_SunDir, view, proj, kNear, kShadowFar, kShadowRes);
        v.lighting->SetFrameData(view, m_SunDir, m_SunColor,
                                 m_CSM->GetLightMatrices(), m_CSM->GetSplitDepths(),
                                 m_PointPos, m_PointColor,
                                 m_PointShadow->FarPlane(),
                                 m_SpotLights, m_SpotShadow->GetLightMatrices(),
                                 m_PointRadius);
        if (v.debugDraw) v.debugDraw->SetFrameData(proj * view);

        // Adaptation eases per second, so it needs the frame delta; both views in
        // a frame share the one measured in RenderFrame.
        v.autoExposure->SetDeltaTime(m_DeltaTime);

        // First frame only: put the never-written TAA history and auto-exposure
        // retained value in a sampleable layout — both are bound by descriptor
        // sets from frame one, even on the frames whose branches ignore them.
        v.taa->Prepare(cmd);
        v.autoExposure->Prepare(cmd);

        v.graph.Execute(cmd);

        // Auto-exposure upkeep, raw after the graph — same shape as the TAA
        // history copy below: move this frame's adapted luminance into the
        // retained image so the next frame eases from it.
        v.autoExposure->RecordAdaptedCopy(cmd);

        // TAA history upkeep, raw after the graph: enabled → copy this frame's
        // resolve into the history (marks it valid); disabled → drop the
        // accumulation, so re-enabling self-seeds with a passthrough frame
        // instead of blending against stale history. The bool gate is the whole
        // cost switch — no copy, no blend, no jitter while off.
        if (m_TAAEnabled) v.taa->RecordHistoryCopy(cmd);
        else              v.taa->InvalidateHistory();

        // Hand an externally-consumed output (the game image) to its sampler —
        // inside this graph nothing reads it, so nothing else transitions it.
        if (v.external)
            cmd->TransitionTexture(v.graph.GetTexture(v.outputColor),
                                   RHITextureState::SampledRead);

        // Commit for next frame's "previous" term — after ubo.prevViewProjUnjittered
        // above already read the old value.
        v.prevViewProj = unjitteredViewProj;
    }

    // Records a visible draw's material as "bound" this frame, plus its 6
    // per-material texture slots ("used") — deduped internally by RHIDevice.
    // The 3 IBL maps (Irradiance/Prefilter/BrdfLUT) are scene-global constants
    // shared by nearly every material, so they're excluded: counting them would
    // just add a fixed +3 that never reflects anything about this frame.
    void RecordMaterialUsage(const PBRMaterial* mat) {
        if (!mat) return;
        m_Device->RecordMaterialBound(mat);
        auto tex = [this](const std::shared_ptr<Texture>& t) {
            if (t) m_Device->RecordTextureUsed(t.get());
        };
        tex(mat->Albedo); tex(mat->Normal); tex(mat->Metallic);
        tex(mat->Roughness); tex(mat->AO); tex(mat->Emissive);
    }

    void DrawGeometry(RHICommandList* cmd, const View& v) {
        // The list is sorted by material, so the set rebind only fires on runs. The
        // static G-buffer pipeline is already bound by the pass.
        RHIResourceSet* bound = nullptr;
        for (const DrawItem& d : v.drawList) {
            if (d.material != bound) {
                cmd->BindResourceSet(0, d.material);
                bound = d.material;
            }
            cmd->BindVertexBuffer(d.mesh->vertexBuffer.get());
            cmd->BindIndexBuffer(d.mesh->indexBuffer.get(), RHIIndexType::U32);
            const GBufferPushConstants pc{ d.world, d.prevWorld };
            cmd->PushConstants(RHIShaderStage::Vertex, 0, sizeof(pc), &pc);
            cmd->DrawIndexed(d.mesh->indexCount);
        }

        // Skinned geometry: rebind to the skinned pipeline (its set-0 material
        // layout matches the static one, so the material sets carry over) and draw,
        // adding the per-entity bone palette at set 1.
        if (v.skinnedDraws.empty()) return;
        cmd->BindPipeline(m_GBuffer->SkinnedPipeline());
        RHIResourceSet* boundMat  = nullptr;
        RHIResourceSet* boundBone = nullptr;
        for (const SkinnedDrawItem& d : v.skinnedDraws) {
            if (d.material != boundMat)  { cmd->BindResourceSet(0, d.material); boundMat  = d.material; }
            if (d.bones    != boundBone) { cmd->BindResourceSet(1, d.bones);    boundBone = d.bones;    }
            cmd->BindVertexBuffer(d.mesh->vertexBuffer.get());
            cmd->BindIndexBuffer(d.mesh->indexBuffer.get(), RHIIndexType::U32);
            const GBufferPushConstants pc{ d.world, d.prevWorld };
            cmd->PushConstants(RHIShaderStage::Vertex, 0, sizeof(pc), &pc);
            cmd->DrawIndexed(d.mesh->indexCount);
        }
    }

    void DrawTransparent(RHICommandList* cmd, const View& v) {
        // Pipeline is already bound by the pass; each draw binds its albedo set
        // (list is back-to-front sorted, so blending composites correctly).
        for (const TransparentDraw& d : v.transparentDraws) {
            cmd->BindResourceSet(0, d.set);
            cmd->BindVertexBuffer(d.mesh->vertexBuffer.get());
            cmd->BindIndexBuffer(d.mesh->indexBuffer.get(), RHIIndexType::U32);
            cmd->PushConstants(RHIShaderStage::Vertex, 0, sizeof(glm::mat4), &d.world);
            cmd->DrawIndexed(d.mesh->indexCount);
        }
    }

    // Which casters a depth draw records. Static shadow caching bakes StaticOnly
    // into a light's cached map, then draws DynamicOnly over the cached copy each
    // frame; passes that don't cache (CSM cascades) use All.
    enum class ShadowFilter { All, StaticOnly, DynamicOnly };

    // Fingerprint of this frame's static shadow casters (mesh identity + world
    // transform). A change means static geometry moved / was added / removed, so
    // the cached static shadow maps must re-bake. Dynamic casters are excluded, so
    // ordinary movement never invalidates the caches. m_ShadowDraws must be built
    // (RebuildDrawList) before this is called.
    size_t ComputeStaticShadowHash() const {
        size_t h = 0;
        auto mix = [&h](size_t v) { h ^= v + 0x9e3779b97f4a7c15ull + (h << 6) + (h >> 2); };
        for (const ShadowItem& d : m_ShadowDraws) {
            if (!d.isStatic) continue;
            mix(std::hash<const void*>{}(d.mesh));
            const float* m = &d.world[0][0];
            for (int i = 0; i < 16; ++i) mix(std::hash<float>{}(m[i]));
        }
        return h;
    }

    // Depth draw for a cascade / spot map. The static depth pipeline is bound by
    // the pass; skinned casters rebind 'skinnedPipeline' (csm_depth_skinned — set 1
    // bone palette) and push lightSpace * model with the skin applied in-shader.
    //
    // Casters are culled against the light's own frustum. The matrices already
    // include their padding (the CSM near-plane zPad, the spot far = range), so
    // this only skips draws the hardware would have clipped anyway — never a
    // caster that could reach the map. 'filter' selects static/dynamic casters for
    // the shadow-cache split; skinned casters are always dynamic (never baked).
    void DrawShadow(RHICommandList* cmd, const glm::mat4& lightSpace,
                    RHIPipeline* skinnedPipeline, ShadowFilter filter = ShadowFilter::All) {
        const Frustum lightFrustum = Frustum::Extract(lightSpace, /*zeroToOneDepth*/ true);
        for (const ShadowItem& d : m_ShadowDraws) {
            if (filter == ShadowFilter::StaticOnly  && !d.isStatic) continue;
            if (filter == ShadowFilter::DynamicOnly &&  d.isStatic) continue;
            if (!lightFrustum.TestAABB(d.worldBounds)) continue;
            const glm::mat4 lm = lightSpace * d.world;
            cmd->BindVertexBuffer(d.mesh->vertexBuffer.get());
            cmd->BindIndexBuffer(d.mesh->indexBuffer.get(), RHIIndexType::U32);
            cmd->PushConstants(RHIShaderStage::Vertex, 0, sizeof(glm::mat4), &lm);
            cmd->DrawIndexed(d.mesh->indexCount);
            m_Device->RecordShadowCaster();
        }

        // Skinned casters are always dynamic — skip them entirely when baking the
        // static cache. The pipeline switch waits for the first survivor so a fully
        // culled (or filtered-out) list costs nothing.
        if (filter == ShadowFilter::StaticOnly) return;
        bool skinnedBound = false;
        RHIResourceSet* boundBone = nullptr;
        for (const SkinnedShadowItem& d : m_SkinnedShadowDraws) {
            if (!lightFrustum.TestAABB(d.worldBounds)) continue;
            if (!skinnedBound) { cmd->BindPipeline(skinnedPipeline); skinnedBound = true; }
            if (d.bones != boundBone) { cmd->BindResourceSet(1, d.bones); boundBone = d.bones; }
            const glm::mat4 lm = lightSpace * d.world;
            cmd->BindVertexBuffer(d.mesh->vertexBuffer.get());
            cmd->BindIndexBuffer(d.mesh->indexBuffer.get(), RHIIndexType::U32);
            cmd->PushConstants(RHIShaderStage::Vertex, 0, sizeof(glm::mat4), &lm);
            cmd->DrawIndexed(d.mesh->indexCount);
            m_Device->RecordShadowCaster();
        }
    }

    // Depth draw for one point-shadow cube face. The distance-cube pipeline and
    // face-matrix set are bound by the pass; only the model matrix is pushed here
    // (offset 0 — the pass pushed the face index at 64). 'filter' selects
    // static/dynamic casters for the shadow-cache split, as in DrawShadow.
    //
    // Range cull: a caster outside the light's far-plane sphere can't reach any
    // face of the cube (depth stores distance / farPlane), so skip it before
    // paying 6 draws. faceIdx = light * 6 + face.
    void DrawPointShadow(RHICommandList* cmd, uint32_t faceIdx, ShadowFilter filter) {
        const glm::vec3 lightPos = m_PointPos[faceIdx / 6];
        const float     range    = m_PointShadow->FarPlane();
        for (const ShadowItem& d : m_ShadowDraws) {
            if (filter == ShadowFilter::StaticOnly  && !d.isStatic) continue;
            if (filter == ShadowFilter::DynamicOnly &&  d.isStatic) continue;
            if (!d.worldBounds.IntersectsSphere(lightPos, range)) continue;
            cmd->BindVertexBuffer(d.mesh->vertexBuffer.get());
            cmd->BindIndexBuffer(d.mesh->indexBuffer.get(), RHIIndexType::U32);
            cmd->PushConstants(RHIShaderStage::Vertex, 0, sizeof(glm::mat4), &d.world);
            cmd->DrawIndexed(d.mesh->indexCount);
            m_Device->RecordShadowCaster();
        }

        // Skinned casters are always dynamic — skip them entirely when baking the
        // static cache. Otherwise swap to the skinned distance-cube pipeline,
        // rebind the shared face-matrix set (set 0) and re-push the face index
        // (offset 64) it expects, then draw each with its bone palette (set 1).
        // The switch waits for the first in-range caster so a fully culled list
        // costs nothing.
        if (filter == ShadowFilter::StaticOnly) return;
        bool skinnedBound = false;
        RHIResourceSet* boundBone = nullptr;
        for (const SkinnedShadowItem& d : m_SkinnedShadowDraws) {
            if (!d.worldBounds.IntersectsSphere(lightPos, range)) continue;
            if (!skinnedBound) {
                cmd->BindPipeline(m_PointShadow->SkinnedPipeline());
                cmd->BindResourceSet(0, m_PointShadow->FaceSet());
                cmd->PushConstants(RHIShaderStage::Vertex, sizeof(glm::mat4), sizeof(faceIdx), &faceIdx);
                skinnedBound = true;
            }
            if (d.bones != boundBone) { cmd->BindResourceSet(1, d.bones); boundBone = d.bones; }
            cmd->BindVertexBuffer(d.mesh->vertexBuffer.get());
            cmd->BindIndexBuffer(d.mesh->indexBuffer.get(), RHIIndexType::U32);
            cmd->PushConstants(RHIShaderStage::Vertex, 0, sizeof(glm::mat4), &d.world);
            cmd->DrawIndexed(d.mesh->indexCount);
            m_Device->RecordShadowCaster();
        }
    }

    // Last frame's world matrix for TAA velocity, or 'world' itself for an entity
    // seen for the first time (zero velocity — correct for a just-spawned object).
    // m_PrevWorld is only ever READ here; it's written once per frame by
    // CommitPrevTransforms after both views' RebuildDrawList calls, so the game
    // view (rebuilt second) doesn't read back a value the main view already
    // overwrote with this frame's transform.
    glm::mat4 PrevWorld(entt::entity entity, const glm::mat4& world) const {
        auto it = m_PrevWorld.find(entity);
        return it != m_PrevWorld.end() ? it->second : world;
    }

    void RebuildDrawList(Scene& scene, const Frustum& frustum, const glm::vec3& cameraPos,
                         View& v, int viewIdx) {
        v.drawList.clear();
        v.skinnedDraws.clear();
        v.transparentDraws.clear();
        m_ShadowDraws.clear();          // view-independent (culled per light view at draw time);
                                        // refilling is idempotent
        m_SkinnedShadowDraws.clear();   // (both cleared per rebuild — filled the same for either view)
        const TransformSystem& ts = scene.GetTransformSystem();
        for (auto [entity, mc] : scene.GetRegistry().view<MeshComponent>().each()) {
            if (!mc.visible || !mc.mesh) continue;
            auto it = m_Meshes.find(mc.mesh.get());
            if (it == m_Meshes.end()) {
                // Unseen mesh: if it retains its CPU geometry (Mesh::Create in
                // Vulkan mode), upload it now — this runs pre-BeginFrame, same
                // timing as the lazy material bake below. Otherwise skip the draw
                // (geometry was never registered).
                if (const MeshData* cpu = mc.mesh->CPUData()) {
                    RegisterMesh(mc.mesh.get(), *cpu);
                    it = m_Meshes.find(mc.mesh.get());
                }
                if (it == m_Meshes.end()) continue;
            }

            const glm::mat4& world = ts.GetWorldMatrix(entity);

            // Transparent meshes route to the forward pass: no G-buffer entry and
            // no shadow casting (an alpha surface throwing a solid shadow reads
            // wrong — matches the GL editor, whose windows never cast).
            if (mc.transparent) {
                if (!frustum.TestAABB(it->second.bounds.Transform(world))) {
                    m_Device->RecordCulled(1);
                    continue;
                }
                m_Device->RecordVisible(1);
                RecordMaterialUsage(mc.material.get());
                v.transparentDraws.push_back({
                    &it->second, GetOrCreateTransparentSet(mc.material.get(), v), world,
                    glm::length(glm::vec3(world[3]) - cameraPos) });
                continue;
            }

            const AABB worldBounds = it->second.bounds.Transform(world);
            // Mask materials don't cast: the shadow depth shaders don't sample
            // albedo, so a masked decal would throw its full quad's shadow.
            // Matches their pre-Mask behavior (routed as transparent = no
            // shadow); alpha-tested shadow shaders are the eventual fix.
            const bool masked = mc.material && mc.material->Mode == AlphaMode::Mask;
            if (mc.castsShadow && !masked) {
                // Static-cache eligibility: honor the mesh's flag, but a moving
                // rigidbody (Dynamic/Kinematic) forces dynamic even if the flag is
                // set — its shadow can't be cached. Static-type bodies (level
                // colliders) stay eligible. Evaluated live each rebuild so editor
                // component add/remove is picked up without a stale cached flag.
                bool isStatic = mc.staticShadowCaster;
                if (isStatic) {
                    if (const auto* rb = scene.GetRegistry().try_get<RigidBodyComponent>(entity))
                        isStatic = (rb->bodyType == BodyType::Static);
                }
                m_ShadowDraws.push_back({ &it->second, world, worldBounds, isStatic });
            }
            if (frustum.TestAABB(worldBounds)) {
                m_Device->RecordVisible(1);
                RecordMaterialUsage(mc.material.get());
                v.drawList.push_back({ &it->second,
                                       GetOrCreateMaterialSet(mc.material.get(), viewIdx), world,
                                       PrevWorld(entity, world) });
                m_VisibleThisFrame.push_back({ entity, world });
            } else {
                m_Device->RecordCulled(1);
            }
        }

        // Skinned characters (SkinnedMeshComponent): one entry per glTF primitive,
        // all sharing the entity's animated bone palette (set 1) and material (set
        // 0, reused from the static cache). Casters go to the skinned shadow list;
        // UpdateSkinnedPalettes fills each bone UBO after BeginFrame.
        for (auto [entity, smc] : scene.GetRegistry().view<SkinnedMeshComponent>().each()) {
            if (!smc.visible || smc.meshes.empty()) continue;

            RHIResourceSet* bones    = GetOrCreateSkinned(entity).bonesSet.get();
            RHIResourceSet* material = GetOrCreateMaterialSet(smc.material.get(), viewIdx);
            const glm::mat4& world   = ts.GetWorldMatrix(entity);
            const AABB bounds        = smc.localBounds.Transform(world);
            const bool  visible      = frustum.TestAABB(bounds);
            if (visible) m_VisibleThisFrame.push_back({ entity, world });   // once per entity, not per primitive

            // Same Mask exclusion as static meshes above.
            const bool masked = smc.material && smc.material->Mode == AlphaMode::Mask;
            for (const auto& meshPtr : smc.meshes) {
                const GpuMesh* gm = GetOrRegisterSkinnedMesh(meshPtr.get());
                if (!gm) continue;
                if (smc.castsShadow && !masked)
                    m_SkinnedShadowDraws.push_back({ gm, bones, world, bounds });
                if (visible) {
                    m_Device->RecordVisible(1);
                    RecordMaterialUsage(smc.material.get());
                    v.skinnedDraws.push_back({ gm, material, bones, world, PrevWorld(entity, world) });
                } else {
                    m_Device->RecordCulled(1);
                }
            }
        }

        // Group draws by material so DrawGeometry rebinds each set once per run.
        std::sort(v.drawList.begin(), v.drawList.end(),
                  [](const DrawItem& a, const DrawItem& b) { return a.material < b.material; });
        std::sort(v.skinnedDraws.begin(), v.skinnedDraws.end(),
                  [](const SkinnedDrawItem& a, const SkinnedDrawItem& b) { return a.material < b.material; });

        // Farther first, so nearer transparents blend over them.
        std::sort(v.transparentDraws.begin(), v.transparentDraws.end(),
                  [](const TransparentDraw& a, const TransparentDraw& b) { return a.dist > b.dist; });
    }

    // Snapshots this frame's world matrices into m_PrevWorld for next frame's TAA
    // velocity, from m_VisibleThisFrame rather than a full registry walk — a
    // culled entity isn't drawn, so there's no velocity to compute for it either;
    // skipping it here avoids paying an unordered_map upsert for every entity in
    // a scene where most are off-screen. Called exactly ONCE per RenderToSwapchain,
    // after both views' RebuildDrawList calls (main, then optionally game) have
    // already read the OLD values via PrevWorld and finished appending to
    // m_VisibleThisFrame — committing first would make the game view read back
    // the main view's just-committed current-frame transform and report zero
    // velocity for anything visible in both.
    void CommitPrevTransforms() {
        for (const auto& [entity, world] : m_VisibleThisFrame)
            m_PrevWorld[entity] = world;
        m_VisibleThisFrame.clear();
    }

    // Transparency set for a material's albedo map (white when absent). Cached by
    // the view's transparency pass per RHI texture; the engine texture is pinned
    // alongside.
    RHIResourceSet* GetOrCreateTransparentSet(const PBRMaterial* mat, View& v) {
        RHITexture* albedo = m_DefaultWhite.get();
        if (mat) {
            albedo = RhiOrDefault(mat->Albedo, m_DefaultWhite.get());
            if (albedo != m_DefaultWhite.get())
                m_TransparentAlbedos.emplace(albedo, mat->Albedo);
        }
        return v.transparency->GetOrCreateAlbedoSet(albedo);
    }

    // Resolve one engine texture slot to its RHI texture, or a neutral default
    // when the slot is empty (or holds a non-Vulkan texture).
    RHITexture* RhiOrDefault(const std::shared_ptr<Texture>& tex, RHITexture* fallback) const {
        if (const auto* vk = dynamic_cast<const VulkanTexture2D*>(tex.get()); vk && vk->Rhi())
            return vk->Rhi();
        return fallback;
    }

    // Resolve a material's texture slots (defaults for empty ones), in
    // VulkanGBufferPass::MaterialMap order. Null material = the checkerboard.
    std::array<RHITexture*, VulkanGBufferPass::MapCount> ResolveMaps(const PBRMaterial* mat) const {
        std::array<RHITexture*, VulkanGBufferPass::MapCount> maps;
        maps[VulkanGBufferPass::Albedo]    = m_Albedo.get();          // checkerboard
        maps[VulkanGBufferPass::Normal]    = m_DefaultNormal.get();
        maps[VulkanGBufferPass::Metallic]  = m_DefaultBlack.get();
        maps[VulkanGBufferPass::Roughness] = m_DefaultGray.get();
        maps[VulkanGBufferPass::AO]        = m_DefaultWhite.get();
        maps[VulkanGBufferPass::Emissive]  = m_DefaultBlack.get();
        if (mat) {
            maps[VulkanGBufferPass::Albedo]    = RhiOrDefault(mat->Albedo,    m_DefaultWhite.get());
            maps[VulkanGBufferPass::Normal]    = RhiOrDefault(mat->Normal,    m_DefaultNormal.get());
            maps[VulkanGBufferPass::Metallic]  = RhiOrDefault(mat->Metallic,  m_DefaultBlack.get());
            maps[VulkanGBufferPass::Roughness] = RhiOrDefault(mat->Roughness, m_DefaultGray.get());
            maps[VulkanGBufferPass::AO]        = RhiOrDefault(mat->AO,        m_DefaultWhite.get());
            maps[VulkanGBufferPass::Emissive]  = RhiOrDefault(mat->Emissive,  m_DefaultBlack.get());
        }
        return maps;
    }

    // Look up (or bake) the G-buffer descriptor set for a material in one view.
    // The material entry (params + pinned textures) is baked once; each view's
    // set is baked lazily the first frame the material appears in that view, and
    // lives for the renderer's lifetime.
    RHIResourceSet* GetOrCreateMaterialSet(const PBRMaterial* mat, int viewIdx) {
        auto it = m_Materials.find(mat);
        if (it == m_Materials.end()) {
            GpuMaterial gm;
            MaterialParams params;
            if (mat) {
                gm.keepAlive = { mat->Albedo, mat->Normal, mat->Metallic,
                                 mat->Roughness, mat->AO, mat->Emissive };
                params.baseColorFactor = mat->BaseColorFactor;
                params.uvScale = mat->UVScale;
                // Strength 0 disables the contribution when no map is bound (GL parity).
                params.emissiveStrength = mat->Emissive ? mat->EmissiveStrength : 0.0f;
                // Cutoff 0 turns the G-buffer alpha test into a no-op for
                // non-Mask materials (a < 0.0 never holds).
                params.alphaCutoff = mat->Mode == AlphaMode::Mask ? mat->AlphaCutoff : 0.0f;
            }

            RHIBufferDesc pb;
            pb.size        = sizeof(MaterialParams);
            pb.usage       = RHIBufferUsage::Uniform;
            pb.initialData = &params;
            gm.params = m_Device->CreateBuffer(pb);
            it = m_Materials.emplace(mat, std::move(gm)).first;
        }

        GpuMaterial& gm = it->second;
        if (!gm.sets[viewIdx])
            gm.sets[viewIdx] = m_GBuffer->CreateMaterialSet(
                m_Views[viewIdx]->cameraUBO.get(), ResolveMaps(mat), gm.params.get());
        return gm.sets[viewIdx].get();
    }

    // ── Skinning ─────────────────────────────────────────────────────────────
    // Upload (once) a skinned mesh in the wider SkinnedVertex layout, keyed on the
    // same Mesh* the SkinnedMeshComponent holds. Returns null if the mesh has no
    // retained CPU geometry to upload from.
    const GpuMesh* GetOrRegisterSkinnedMesh(Mesh* key) {
        if (!key) return nullptr;
        auto it = m_SkinnedMeshes.find(key);
        if (it != m_SkinnedMeshes.end()) return &it->second;

        const MeshData* cpu = key->CPUData();
        if (!cpu) return nullptr;

        std::vector<SkinnedVertex> verts;
        verts.reserve(cpu->Vertices.size());
        for (const Vertex& v : cpu->Vertices)
            verts.push_back({ v.Position, v.Normal, v.TexCoords, v.Tangent,
                              v.BoneIDs, v.BoneWeights });

        GpuMesh gm;
        RHIBufferDesc vb;
        vb.size        = verts.size() * sizeof(SkinnedVertex);
        vb.usage       = RHIBufferUsage::Vertex;
        vb.initialData = verts.data();
        gm.vertexBuffer = m_Device->CreateBuffer(vb);

        RHIBufferDesc ib;
        ib.size        = cpu->Indices.size() * sizeof(uint32_t);
        ib.usage       = RHIBufferUsage::Index;
        ib.initialData = cpu->Indices.data();
        gm.indexBuffer = m_Device->CreateBuffer(ib);
        gm.indexCount  = static_cast<uint32_t>(cpu->Indices.size());
        gm.bounds      = cpu->ComputeAABB();

        return &m_SkinnedMeshes.emplace(key, std::move(gm)).first->second;
    }

    // The per-entity bone state, created on first use. The palette UBO is dynamic
    // (rewritten each frame by UpdateSkinnedPalettes); its set-1 descriptor is built
    // against the skinned G-buffer pipeline's set 1 — identical to every skinned
    // pipeline's set-1 layout, so the same set binds under the shadow pipelines too.
    SkinnedInstance& GetOrCreateSkinned(entt::entity e) {
        auto it = m_Skinned.find(e);
        if (it != m_Skinned.end()) return it->second;

        SkinnedInstance inst;
        RHIBufferDesc bd;
        // Two palettes back to back — bones[kMaxBones] then prevBones[kMaxBones],
        // matching gbuffer_skinned.vert's BoneUBO. The shadow skinned shaders
        // declare only the first array; binding the larger buffer there is fine
        // (Vulkan requires bound range >= declared block size, not equality).
        // 2 * 100 * 64B = 12.8KB, under the 16KB minimum uniform-range guarantee.
        bd.size    = sizeof(glm::mat4) * kMaxBones * 2;
        bd.usage   = RHIBufferUsage::Uniform;
        bd.dynamic = true;
        inst.bonesUBO = m_Device->CreateBuffer(bd);
        inst.bonesSet = m_Device->CreateResourceSet(
            m_GBuffer->SkinnedPipeline(), 1, { { 0, inst.bonesUBO.get() } }, {});
        return m_Skinned.emplace(e, std::move(inst)).first->second;
    }

    // Push this frame's animated palettes into each skinned entity's bone UBO. Runs
    // after BeginFrame (dynamic Update targets the now-idle frame slot), before any
    // view records. A skinned entity without a live palette uploads identity (bind
    // pose). Palettes longer than kMaxBones are clamped, matching the GL renderer.
    //
    // The UBO's second half is LAST frame's palette (cached CPU-side per entity),
    // so gbuffer_skinned.vert can skin the previous position with the previous
    // pose — per-bone TAA velocity, not just whole-entity motion. First sighting
    // seeds previous = current (zero velocity), like every other TAA cold start.
    void UpdateSkinnedPalettes(Scene& scene) {
        std::array<glm::mat4, kMaxBones * 2> upload;   // [0,kMaxBones) current | prev
        for (auto [entity, smc] : scene.GetRegistry().view<SkinnedMeshComponent>().each()) {
            if (!smc.visible || smc.meshes.empty()) continue;

            const AnimatorComponent* anim =
                scene.GetRegistry().try_get<AnimatorComponent>(entity);
            const size_t count = (anim && !anim->palette.empty())
                ? std::min(anim->palette.size(), static_cast<size_t>(kMaxBones)) : 0;

            for (size_t i = 0; i < count; ++i)          upload[i] = anim->palette[i];
            for (size_t i = count; i < kMaxBones; ++i)  upload[i] = glm::mat4(1.0f);

            SkinnedInstance& inst = GetOrCreateSkinned(entity);
            if (inst.hasPrev)
                std::copy(inst.prevPalette.begin(), inst.prevPalette.end(),
                          upload.begin() + kMaxBones);
            else
                std::copy(upload.begin(), upload.begin() + kMaxBones,
                          upload.begin() + kMaxBones);

            inst.bonesUBO->Update(upload.data(), sizeof(upload));

            // Cache this frame's palette as next frame's "previous".
            std::copy(upload.begin(), upload.begin() + kMaxBones,
                      inst.prevPalette.begin());
            inst.hasPrev = true;
        }
    }

    // gameFrustum is null when the game view isn't active this frame. A point
    // or spot light survives if its range sphere is visible to *either* active
    // view — shadow maps are recorded once, view-independent, and shared by
    // both views' lighting passes (see RenderToSwapchain), so culling against
    // only one view could drop a light the other still needs.
    void GatherLights(Scene& scene, const Frustum& mainFrustum, const Frustum* gameFrustum) {
        m_PointPos.clear();
        m_PointColor.clear();
        m_PointRadius.clear();
        m_SpotLights.clear();
        m_SunDir   = glm::vec3(0.0f, -1.0f, 0.0f);
        m_SunColor = glm::vec3(0.0f);
        bool foundSun = false;

        auto visible = [&](const glm::vec3& pos, float radius) {
            if (mainFrustum.TestSphere(pos, radius)) return true;
            return gameFrustum && gameFrustum->TestSphere(pos, radius);
        };

        const TransformSystem& ts = scene.GetTransformSystem();
        for (auto [entity, lc] : scene.GetRegistry().view<LightComponent>().each()) {
            const glm::mat4& world = ts.GetWorldMatrix(entity);
            const glm::vec3 pos = glm::vec3(world[3]);
            const glm::vec3 dir = glm::normalize(glm::mat3(world) * glm::vec3(0.0f, -1.0f, 0.0f));

            if (lc.type == LightType::Point && m_PointPos.size() < 4) {
                if (!visible(pos, lc.radius)) continue;
                m_PointPos.push_back(pos);
                m_PointColor.push_back(lc.color * lc.intensity);
                m_PointRadius.push_back(lc.radius);
            } else if (lc.type == LightType::Spot
                       && m_SpotLights.size() < VulkanSpotShadowPass::MAX_SPOTS) {
                if (!visible(pos, lc.radius)) continue;
                m_SpotLights.push_back({ pos, dir, lc.color * lc.intensity,
                                         lc.innerConeAngle, lc.outerConeAngle, lc.radius });
            } else if (lc.type == LightType::Sun && !foundSun) {
                m_SunDir   = dir;
                m_SunColor = lc.color * lc.intensity;
                foundSun   = true;
            }
        }
    }

    void GatherParticles(Scene& scene) {
        m_ParticleBatches.clear();
        for (auto [entity, em] : scene.GetRegistry().view<ParticleEmitterComponent>().each()) {
            if (em.liveCount == 0) continue;

            std::shared_ptr<Texture> texture = em.texture ? em.texture : m_DefaultParticleTexture;
            const auto* vkTexture = dynamic_cast<const VulkanTexture2D*>(texture.get());
            if (!vkTexture || !vkTexture->Rhi()) {
                texture = m_DefaultParticleTexture;
                vkTexture = dynamic_cast<const VulkanTexture2D*>(texture.get());
            }
            if (!vkTexture || !vkTexture->Rhi()) continue;

            PBatch* batch = nullptr;
            for (PBatch& b : m_ParticleBatches) {
                if (b.tex == vkTexture->Rhi() && b.blend == em.blend) {
                    batch = &b;
                    break;
                }
            }
            if (!batch) {
                m_ParticleBatches.push_back({ vkTexture->Rhi(), texture, em.blend, {} });
                batch = &m_ParticleBatches.back();
            }

            batch->parts.reserve(batch->parts.size() + em.liveCount);
            for (std::size_t i = 0; i < em.liveCount; ++i) {
                const auto& p = em.pool[i];
                batch->parts.push_back({ p.pos, p.size, p.color, p.rotation });
            }
        }
    }

    // ── Shader hot-reload (editor only) ─────────────────────────────────────
    // One GLSL source filename + the mtime CheckWatch last saw it at. The
    // first poll after construction only records a baseline (no reload) — the
    // file was just compiled by the build, so treating that as "changed" would
    // reload every pass on the very first frame.
    struct ShaderWatch { std::string name; std::filesystem::file_time_type mtime{}; };

    void CheckWatch(std::vector<ShaderWatch>& files, bool& changed) {
        namespace fs = std::filesystem;
        for (ShaderWatch& f : files) {
            std::error_code ec;
            const fs::file_time_type t = fs::last_write_time(m_ShaderSrcDir + "/" + f.name, ec);
            if (ec) continue;   // source missing — leave the pass on its last-good module
            if (f.mtime == fs::file_time_type{}) { f.mtime = t; continue; }   // baseline
            if (t != f.mtime) { f.mtime = t; changed = true; }
        }
    }

    // Recompiles every watched file in the requested groups (harmless to
    // recompile an unchanged file alongside a changed sibling) and rebuilds
    // the corresponding pass(es) in place. One WaitIdle for the whole batch —
    // reloads across groups in the same poll share the stall.
    void DoReload(bool gbuffer, bool lighting, bool skybox, bool transparency, bool particles) {
        const std::string spvDir = ShaderDir();
        auto recompile = [&](std::vector<ShaderWatch>& files) {
            for (ShaderWatch& f : files) RecompileSpirv(m_ShaderSrcDir, spvDir, f.name);
        };
        if (gbuffer)      recompile(m_GBufferWatch);
        if (lighting)     recompile(m_LightingWatch);
        if (skybox)       recompile(m_SkyboxWatch);
        if (transparency) recompile(m_TransparencyWatch);
        if (particles)    recompile(m_ParticlesWatch);

        m_Device->WaitIdle();

        if (gbuffer) {
            m_GBuffer->Reload();
            // Baked against the G-buffer pipeline's set-0 layout — rebuild lazily.
            m_Materials.clear();
        }
        for (auto& v : m_Views) {
            if (!v) continue;
            if (lighting) {
                v->lighting->Reload();
                v->lighting->BindIBL(*m_IBL);
                v->lighting->BindPointShadows(*m_PointShadow);
            }
            if (skybox) {
                v->skybox->Reload();
                v->skybox->BindIBL(*m_IBL);
            }
            if (transparency) v->transparency->Reload();
            if (particles)    v->particles->Reload();
        }
        if (particles && m_Preview) m_Preview->particles->Reload();

        spdlog::info("[Vulkan] hot-reloaded shaders (gbuffer={} lighting={} skybox={} "
                     "transparency={} particles={})",
                     gbuffer, lighting, skybox, transparency, particles);
    }

    RHIDevice* m_Device;
    bool       m_Offscreen;

    // m_Views[kMainView] always exists; the game view is created on first
    // SetGameViewEnabled(true). unique_ptr keeps addresses stable — the graph
    // draw callbacks capture View&.
    std::array<std::unique_ptr<View>, kMaxViews> m_Views;
    bool m_GameViewEnabled = false;
    bool m_GameViewActive  = false;   // last frame actually rendered the game view

    OverlayFn m_UIFn;       // in-game UI pass body, main view (may be empty)
    OverlayFn m_GameUIFn;   // in-game UI pass body, game view (may be empty)
    OverlayFn m_Overlay;    // per-frame editor overlay (ImGui), offscreen mode

    std::unique_ptr<RHITexture> m_Albedo;          // checkerboard — null-material albedo
    std::unique_ptr<RHITexture> m_DefaultWhite;    // 1×1 fallbacks for empty slots
    std::unique_ptr<RHITexture> m_DefaultNormal;
    std::unique_ptr<RHITexture> m_DefaultBlack;
    std::unique_ptr<RHITexture> m_DefaultGray;

    // Shared, view-independent passes (record-time state only — see class comment).
    std::unique_ptr<VulkanGBufferPass>     m_GBuffer;
    std::unique_ptr<VulkanCSMPass>         m_CSM;
    std::unique_ptr<VulkanSpotShadowPass>  m_SpotShadow;
    std::unique_ptr<VulkanPointShadowPass> m_PointShadow;
    std::unique_ptr<VulkanIBLPass>         m_IBL;

    std::unordered_map<Mesh*, GpuMesh>                  m_Meshes;
    std::unordered_map<Mesh*, GpuMesh>                  m_SkinnedMeshes;  // wider vertex layout
    std::unordered_map<const PBRMaterial*, GpuMaterial> m_Materials;
    std::vector<ShadowItem> m_ShadowDraws;   // all casters — culled per light view at draw time
    size_t m_LastStaticShadowHash = 0;       // detects static-caster changes → re-bake cached maps

    // Per-skinned-entity GPU state (SkinnedInstance defined above), keyed by entity.
    std::unordered_map<entt::entity, SkinnedInstance> m_Skinned;
    // Last frame's world matrix per entity — TAA velocity input (PrevWorld reads
    // it, CommitPrevTransforms writes it once per frame). A stale entry from a
    // destroyed entity simply lingers, like m_Skinned above.
    std::unordered_map<entt::entity, glm::mat4> m_PrevWorld;
    // (entity, world) pairs accumulated by RebuildDrawList for every VISIBLE draw
    // this frame (both views) — CommitPrevTransforms drains this into m_PrevWorld
    // and clears it. Cleared here (not per-view) since it accumulates across the
    // main+game RebuildDrawList calls in one frame.
    std::vector<std::pair<entt::entity, glm::mat4>> m_VisibleThisFrame;
    std::vector<SkinnedShadowItem> m_SkinnedShadowDraws;   // skinned casters — culled per light view
    // Albedo textures behind cached transparency sets, kept alive with them.
    std::unordered_map<RHITexture*, std::shared_ptr<Texture>> m_TransparentAlbedos;

    // All re-applied when a resize recreates the pass that owns them.
    float                  m_Exposure       = 1.0f;
    Tonemapper             m_Tonemapper     = Tonemapper::ACES;
    float                  m_BloomThreshold = 1.0f;
    float                  m_BloomIntensity = 1.0f;
    AutoExposureSettings   m_AutoExposure   {};

    // Wall-clock frame delta, measured once per RenderFrame and shared by every
    // view. Auto-exposure adaptation is the only consumer; taking it from the
    // clock rather than the engine's dt keeps the renderer's public surface free
    // of a per-frame setter that every caller (editor, standalone runtime) would
    // otherwise have to remember to drive.
    float                                              m_DeltaTime = 1.0f / 60.0f;
    std::chrono::high_resolution_clock::time_point     m_LastFrameTime{};
    bool                                               m_HasLastFrameTime = false;
    glm::vec3              m_SunDir   { 0.0f, -1.0f, 0.0f };
    glm::vec3              m_SunColor { 0.0f };
    std::vector<glm::vec3> m_PointPos;      // world space
    std::vector<glm::vec3> m_PointColor;
    std::vector<float>     m_PointRadius;   // falloff window = the GatherLights cull sphere
    std::vector<SpotLightInfo> m_SpotLights;

    // The camera the graph's particle callback billboards against — set per view
    // right before that view's Execute (read at record time).
    glm::mat4 m_FrameView { 1.0f };
    glm::mat4 m_FrameProj { 1.0f };

    // TAA jitter phase — advances once per RenderToSwapchain call (both views in
    // a frame share the same Halton sample).
    uint64_t m_FrameIndex = 0;
    bool     m_TAAEnabled = true;

    std::unique_ptr<RHITexture> m_DefaultParticleRHI;
    std::shared_ptr<Texture>    m_DefaultParticleTexture;
    std::vector<PBatch>         m_ParticleBatches;

    std::unique_ptr<ParticlePreview> m_Preview;      // null until first EnsureParticlePreview
    ParticleOverlayFn                m_PreviewFn;     // preview panel's per-frame Begin/Draw/End

    // Shader hot-reload watch lists (editor only) — GLSL source dir + one
    // ShaderWatch per file, grouped by the pass it feeds.
    const std::string m_ShaderSrcDir = std::string(DIAMOND_ASSETS_DIR) + "/Shaders/Vulkan";
    std::vector<ShaderWatch> m_GBufferWatch {
        { "gbuffer.vert" }, { "gbuffer.frag" }, { "gbuffer_skinned.vert" } };
    std::vector<ShaderWatch> m_LightingWatch { { "fullscreen.vert" }, { "deferred_lighting.frag" } };
    std::vector<ShaderWatch> m_SkyboxWatch { { "skybox.vert" }, { "skybox.frag" } };
    std::vector<ShaderWatch> m_TransparencyWatch { { "transparent.vert" }, { "transparent.frag" } };
    std::vector<ShaderWatch> m_ParticlesWatch { { "particle.vert" }, { "particle.frag" } };
};

std::unique_ptr<SceneRenderer> SceneRenderer::Create(RHIDevice* device,
                                                     uint32_t width, uint32_t height,
                                                     bool offscreen) {
    if (!device) return nullptr;
    return std::make_unique<SceneRendererVk>(device, width, height, offscreen);
}

void SceneRenderer::SetShaderDirectory(std::string dir) {
    ShaderDir() = std::move(dir);
}

} // namespace Diamond

#else  // !DIAMOND_ENABLE_VULKAN — no backend to render through.

namespace Diamond {
std::unique_ptr<SceneRenderer> SceneRenderer::Create(RHIDevice*, uint32_t, uint32_t, bool) {
    return nullptr;
}
void SceneRenderer::SetShaderDirectory(std::string) {}
} // namespace Diamond

#endif
