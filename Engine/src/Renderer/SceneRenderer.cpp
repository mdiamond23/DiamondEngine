#include "Renderer/SceneRenderer.h"

#ifdef DIAMOND_ENABLE_VULKAN

// Vulkan expects clip-space depth in [0, 1] (OpenGL uses [-1, 1]); make GLM's
// perspective matrix match before any glm header is pulled in.
#define GLM_FORCE_DEPTH_ZERO_TO_ONE

#include "Renderer/RHI/RHIDevice.h"
#include "Renderer/RHI/RHICommandList.h"
#include "Renderer/RHI/RHIRenderGraph.h"
#include "Renderer/MeshData.h"
#include "Renderer/Frustum.h"
#include "Core/Camera.h"
#include "Scene/Scene.h"
#include "Scene/Components.h"
#include "Animation/AnimationComponents.h"

#include "Platform/Vulkan/Passes/Deferred/VulkanGBufferPass.h"
#include "Platform/Vulkan/Passes/Deferred/VulkanSSAOPass.h"
#include "Platform/Vulkan/Passes/Deferred/VulkanDeferredLightingPass.h"
#include "Platform/Vulkan/Passes/Deferred/VulkanSkyboxPass.h"
#include "Platform/Vulkan/Passes/Debug/VulkanDebugDrawPass.h"
#include "Platform/Vulkan/Passes/Shadows/VulkanCSMPass.h"
#include "Platform/Vulkan/Passes/Shadows/VulkanSpotShadowPass.h"
#include "Platform/Vulkan/Passes/Shadows/VulkanPointShadowPass.h"
#include "Platform/Vulkan/Passes/Forward/VulkanTransparencyPass.h"
#include "Platform/Vulkan/Passes/PostProcess/VulkanTonemapPass.h"
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
};

// The shadowed range is kept tighter than the camera far plane so the cascades
// pack around the visible scene.
constexpr float kNear      = 0.1f;
constexpr float kFar       = 100.0f;
constexpr float kShadowFar = 25.0f;
constexpr uint32_t kShadowRes     = 2048;
constexpr uint32_t kSpotShadowRes = 1024;

// Per-material params, std140 layout matching gbuffer.frag's MaterialUBO
// (scalars at offsets 0/4, padded to 16).
struct MaterialParams {
    float uvScale          = 1.0f;
    float emissiveStrength = 0.0f;
    float _pad0 = 0.0f, _pad1 = 0.0f;
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
            if (v) v->tonemap->SetExposure(exposure);
    }

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

    void EnsureParticlePreview(uint32_t width, uint32_t height, const glm::vec3& bgColor) override {
        if (width == 0 || height == 0) return;
        if (m_Preview && width == m_Preview->width && height == m_Preview->height) return;

        // Same rationale as Resize/InvalidateMaterial: the pooled texture the old
        // pass wrote may still be read by ImGui's in-flight draw data.
        if (m_Preview) m_Device->WaitIdle();

        if (!m_Preview) {
            m_Preview = std::make_unique<ParticlePreview>(m_Device);
            const std::string shaderDir = DIAMOND_VULKAN_SHADER_DIR;
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

    void Resize(uint32_t width, uint32_t height) override {
        if (!m_Offscreen) return;
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
        View& main = *m_Views[kMainView];
        const float aspect = static_cast<float>(main.width) / static_cast<float>(main.height);
        const glm::mat4 view = camera.GetViewMatrix();
        // Explicitly the [0,1]-depth variant: plain glm::perspective resolves its
        // depth range from a per-TU define, and its inline instantiations
        // COMDAT-fold across TUs — a GL TU's [-1,1] version can silently win.
        const glm::mat4 proj = glm::perspectiveRH_ZO(glm::radians(camera.Zoom), aspect, kNear, kFar);

        // Refresh world transforms, then gather this frame's shared (view-
        // independent) scene data: lights and particle batches.
        scene.GetTransformSystem().Update(scene.GetRegistry());
        GatherLights(scene);
        GatherParticles(scene);

        // Game view: rendered only when enabled AND the scene has a primary
        // camera to render from (mirrors the GL editor's game viewport).
        m_GameViewActive = false;
        View* game = (m_GameViewEnabled && m_Views[kGameView]) ? m_Views[kGameView].get() : nullptr;
        glm::mat4 gameView(1.0f), gameProj(1.0f);
        if (game) {
            const entt::entity camEntity = scene.GetPrimaryCamera();
            if (camEntity != entt::null) {
                const auto& cc = scene.GetRegistry().get<CameraComponent>(camEntity);
                gameView = glm::inverse(scene.GetTransformSystem().GetWorldMatrix(camEntity));
                gameProj = glm::perspectiveRH_ZO(
                    glm::radians(cc.fov),
                    static_cast<float>(game->width) / static_cast<float>(game->height),
                    cc.nearClip, cc.farClip);
                m_GameViewActive = true;
            }
        }

        // Per-view draw lists (frustum-culled per camera). Built before
        // BeginFrame so lazy mesh/material uploads keep their pre-frame timing.
        // The unculled shadow list is view-independent and shared.
        RebuildDrawList(scene, Frustum::Extract(proj * view, /*zeroToOneDepth*/ true),
                        camera.Position, main, kMainView);
        if (m_GameViewActive)
            RebuildDrawList(scene, Frustum::Extract(gameProj * gameView, /*zeroToOneDepth*/ true),
                            glm::vec3(glm::inverse(gameView)[3]), *game, kGameView);

        RHICommandList* cmd = m_Device->BeginFrame();
        if (!cmd) return;   // swapchain unavailable (minimized / rebuilding)

        // Now the frame slot is idle: push this frame's animated bone palettes into
        // each skinned entity's dynamic UBO (dynamic Update must follow BeginFrame).
        UpdateSkinnedPalettes(scene);

        // View-independent shadow work, once per frame: spot matrices are
        // light-space, and the point-light cube shadows record raw per-face
        // scopes the graph can't express — put them in the command buffer before
        // either view's graph so both lighting reads see this frame's maps.
        m_SpotShadow->ComputeMatrices(m_SpotLights);
        m_PointShadow->SetLights(m_PointPos);
        m_PointShadow->Record(cmd, [this](RHICommandList* c, uint32_t faceIdx) {
            for (const DrawItem& d : m_ShadowDraws) {
                c->BindVertexBuffer(d.mesh->vertexBuffer.get());
                c->BindIndexBuffer(d.mesh->indexBuffer.get(), RHIIndexType::U32);
                c->PushConstants(RHIShaderStage::Vertex, 0, sizeof(glm::mat4), &d.world);
                c->DrawIndexed(d.mesh->indexCount);
            }

            // Skinned casters: swap to the skinned distance-cube pipeline, rebind the
            // shared face-matrix set (set 0) and re-push the face index (offset 64)
            // it expects, then draw each with its bone palette (set 1).
            if (m_SkinnedShadowDraws.empty()) return;
            c->BindPipeline(m_PointShadow->SkinnedPipeline());
            c->BindResourceSet(0, m_PointShadow->FaceSet());
            c->PushConstants(RHIShaderStage::Vertex, sizeof(glm::mat4), sizeof(faceIdx), &faceIdx);
            RHIResourceSet* boundBone = nullptr;
            for (const SkinnedShadowItem& d : m_SkinnedShadowDraws) {
                if (d.bones != boundBone) { c->BindResourceSet(1, d.bones); boundBone = d.bones; }
                c->BindVertexBuffer(d.mesh->vertexBuffer.get());
                c->BindIndexBuffer(d.mesh->indexBuffer.get(), RHIIndexType::U32);
                c->PushConstants(RHIShaderStage::Vertex, 0, sizeof(glm::mat4), &d.world);
                c->DrawIndexed(d.mesh->indexCount);
            }
        });

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
    };
    // A skinned shadow caster — depth passes need only the bone set + world (no
    // material). Shared across views/lights like the static m_ShadowDraws.
    struct SkinnedShadowItem {
        const GpuMesh*  mesh;
        RHIResourceSet* bones;
        glm::mat4       world;
    };
    // Per-skinned-entity GPU state: a dynamic bone-palette UBO (rewritten each
    // frame from AnimatorComponent::palette) + its set-1 descriptor. Created lazily,
    // keyed by entity, and kept for the renderer's lifetime (skinned entities are
    // few; a destroyed entity's slot simply lingers).
    struct SkinnedInstance {
        std::unique_ptr<RHIBuffer>      bonesUBO;   // dynamic mat4[kMaxBones]
        std::unique_ptr<RHIResourceSet> bonesSet;   // set 1, binding 0 = bonesUBO
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

        std::unique_ptr<RHIBuffer>                  cameraUBO;
        std::unique_ptr<VulkanSSAOPass>             ssao;
        std::unique_ptr<VulkanDeferredLightingPass> lighting;
        std::unique_ptr<VulkanSkyboxPass>           skybox;
        std::unique_ptr<VulkanTransparencyPass>     transparency;
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

        const std::string shaderDir = DIAMOND_VULKAN_SHADER_DIR;
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
        m_IBL->BakeEnvironment(DIAMOND_ASSETS_DIR "/Textures/citrus_orchard_road_puresky_4k.hdr");
    }

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

        const std::string shaderDir = DIAMOND_VULKAN_SHADER_DIR;
        v->ssao         = std::make_unique<VulkanSSAOPass>(m_Device, shaderDir, width, height);
        v->lighting     = std::make_unique<VulkanDeferredLightingPass>(m_Device, shaderDir);
        v->skybox       = std::make_unique<VulkanSkyboxPass>(m_Device, shaderDir);
        v->transparency = std::make_unique<VulkanTransparencyPass>(m_Device, shaderDir);
        // Offscreen views tonemap into an LDR texture ImGui can sample; the
        // swapchain-mode main view tonemaps straight into the backbuffer.
        v->tonemap = std::make_unique<VulkanTonemapPass>(
            m_Device, shaderDir,
            offscreen ? RHIFormat::RGBA8 : m_Device->SwapchainFormat());
        v->tonemap->SetExposure(m_Exposure);
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
        if (!v.offscreen || width == 0 || height == 0) return;
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
        const std::string shaderDir = DIAMOND_VULKAN_SHADER_DIR;
        v.ssao     = std::make_unique<VulkanSSAOPass>(m_Device, shaderDir, width, height);
        v.lighting = std::make_unique<VulkanDeferredLightingPass>(m_Device, shaderDir);
        v.tonemap  = std::make_unique<VulkanTonemapPass>(m_Device, shaderDir, RHIFormat::RGBA8);
        v.tonemap->SetExposure(m_Exposure);   // the recreated pass defaults to 1.0

        v.graph.ResetPasses();
        BuildViewGraph(v, viewIdx);   // re-declares textures + re-binds IBL/point shadows
    }

    void BuildViewGraph(View& v, int viewIdx) {
        RHIRenderGraph& g = v.graph;
        const RGTextureHandle gViewPos    = g.DeclareTexture("gViewPos",    { v.width, v.height, RHIFormat::RGBA16F });
        const RGTextureHandle gViewNormal = g.DeclareTexture("gViewNormal", { v.width, v.height, RHIFormat::RGBA16F });
        const RGTextureHandle gAlbedo     = g.DeclareTexture("gAlbedo",     { v.width, v.height, RHIFormat::RGBA8   });
        const RGTextureHandle gMaterial   = g.DeclareTexture("gMaterial",   { v.width, v.height, RHIFormat::RGBA8   });
        const RGTextureHandle gEmissive   = g.DeclareTexture("gEmissive",   { v.width, v.height, RHIFormat::RGBA16F });
        const RGTextureHandle gDepth      = g.DeclareTexture("gDepth",      { v.width, v.height, RHIFormat::Depth32F });
        const RGTextureHandle ssaoRaw     = g.DeclareTexture("ssaoRaw",     { v.width, v.height, RHIFormat::R16F     });
        const RGTextureHandle ssaoBlurred = g.DeclareTexture("ssaoBlurred", { v.width, v.height, RHIFormat::R16F     });
        const RGTextureHandle hdrLit      = g.DeclareTexture("hdrLit",      { v.width, v.height, RHIFormat::RGBA16F });

        std::array<RGTextureHandle, VulkanCSMPass::NUM_CASCADES> cascades;
        for (int i = 0; i < VulkanCSMPass::NUM_CASCADES; ++i)
            cascades[i] = g.DeclareTexture("csmCascade" + std::to_string(i),
                                           { kShadowRes, kShadowRes, RHIFormat::Depth32F });

        std::array<RGTextureHandle, VulkanSpotShadowPass::MAX_SPOTS> spotMaps;
        for (int i = 0; i < VulkanSpotShadowPass::MAX_SPOTS; ++i)
            spotMaps[i] = g.DeclareTexture("spotShadow" + std::to_string(i),
                                           { kSpotShadowRes, kSpotShadowRes, RHIFormat::Depth32F });

        // G-buffer — fill from the view's draw list; each draw binds its
        // material's descriptor set (built lazily by GetOrCreateMaterialSet).
        m_GBuffer->AddToGraph(g, gViewPos, gViewNormal, gAlbedo, gMaterial, gEmissive,
                              gDepth,
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

        // Spot shadows — one perspective depth map per spot light, same push /
        // draw callback shape as the cascades (also shared; light-space is
        // view-independent, so both views record the same maps).
        m_SpotShadow->AddToGraph(g, spotMaps,
                                 [this](RHICommandList* cmd, const glm::mat4& lightSpace) {
                                     DrawShadow(cmd, lightSpace, m_SpotShadow->SkinnedPipeline());
                                 });

        // Deferred resolve → HDR. Reads all cascade + spot maps (also keeps the
        // unlit ones alive). The point-shadow cubes aren't graph textures — they
        // render pre-graph in RenderToSwapchain and bind raw below.
        v.lighting->AddToGraph(g, gViewPos, gViewNormal, gAlbedo, gMaterial,
                               ssaoBlurred, gEmissive, cascades, spotMaps, hdrLit);
        v.lighting->BindIBL(*m_IBL);
        v.lighting->BindPointShadows(*m_PointShadow);

        // Skybox fills the background pixels (far plane, LessEqual against the
        // G-buffer depth) with the baked env cube — the same map the IBL ambient
        // samples, so reflections have a visible source. Inserted after the
        // lighting resolve and before particles; the writes on the shared HDR
        // target resolve by insertion order.
        v.skybox->AddToGraph(g, hdrLit, gDepth);
        v.skybox->BindIBL(*m_IBL);

        // Forward transparency blends over the lit scene + skybox, depth-testing
        // (not writing) against the G-buffer depth — before particles, matching
        // the GL frame order (lighting → skybox → transparency → particles).
        v.transparency->AddToGraph(g, hdrLit, gDepth,
            [this, &v](RHICommandList* cmd) { DrawTransparent(cmd, v); });

        // Particles composite over the lit HDR scene (before tonemap). The
        // frame view/proj members are re-set per view before its Execute.
        v.particles->AddToGraph(g, hdrLit, gDepth,
            [this](VulkanParticleRenderer& particles) {
                particles.Begin(m_FrameView, m_FrameProj);
                for (PBatch& b : m_ParticleBatches) {
                    if (!b.texAsset || b.parts.empty()) continue;
                    particles.Draw(b.parts, *b.texAsset, b.blend);
                }
                particles.End();
            });

        // Tonemap HDR → swapchain (ACES + gamma), or → the LDR output texture
        // for offscreen views.
        if (v.offscreen)
            v.outputColor = g.DeclareTexture("outputColor", { v.width, v.height, RHIFormat::RGBA8 });
        v.tonemap->AddToGraph(g, hdrLit, v.offscreen ? v.outputColor : RGTextureHandle{});

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
        // Read at record time by the particle pass callback during Execute.
        m_FrameView = view;
        m_FrameProj = proj;

        CameraUBO ubo{};
        ubo.view     = view;
        ubo.viewProj = proj * view;
        v.cameraUBO->Update(&ubo, sizeof(ubo));

        v.ssao->SetProjection(proj);
        v.skybox->SetFrameData(view, proj);
        v.transparency->SetCamera(view, proj);
        m_CSM->ComputeCascades(m_SunDir, view, proj, kNear, kShadowFar);
        v.lighting->SetFrameData(view, m_SunDir, m_SunColor,
                                 m_CSM->GetLightMatrices(), m_CSM->GetSplitDepths(),
                                 m_PointPos, m_PointColor,
                                 m_PointShadow->FarPlane(),
                                 m_SpotLights, m_SpotShadow->GetLightMatrices());
        if (v.debugDraw) v.debugDraw->SetFrameData(proj * view);

        v.graph.Execute(cmd);

        // Hand an externally-consumed output (the game image) to its sampler —
        // inside this graph nothing reads it, so nothing else transitions it.
        if (v.external)
            cmd->TransitionTexture(v.graph.GetTexture(v.outputColor),
                                   RHITextureState::SampledRead);
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
            cmd->PushConstants(RHIShaderStage::Vertex, 0, sizeof(glm::mat4), &d.world);
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
            cmd->PushConstants(RHIShaderStage::Vertex, 0, sizeof(glm::mat4), &d.world);
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

    // Depth draw for a cascade / spot map. The static depth pipeline is bound by
    // the pass; skinned casters rebind 'skinnedPipeline' (csm_depth_skinned — set 1
    // bone palette) and push lightSpace * model with the skin applied in-shader.
    void DrawShadow(RHICommandList* cmd, const glm::mat4& lightSpace,
                    RHIPipeline* skinnedPipeline) {
        for (const DrawItem& d : m_ShadowDraws) {
            const glm::mat4 lm = lightSpace * d.world;
            cmd->BindVertexBuffer(d.mesh->vertexBuffer.get());
            cmd->BindIndexBuffer(d.mesh->indexBuffer.get(), RHIIndexType::U32);
            cmd->PushConstants(RHIShaderStage::Vertex, 0, sizeof(glm::mat4), &lm);
            cmd->DrawIndexed(d.mesh->indexCount);
        }

        if (m_SkinnedShadowDraws.empty()) return;
        cmd->BindPipeline(skinnedPipeline);
        RHIResourceSet* boundBone = nullptr;
        for (const SkinnedShadowItem& d : m_SkinnedShadowDraws) {
            if (d.bones != boundBone) { cmd->BindResourceSet(1, d.bones); boundBone = d.bones; }
            const glm::mat4 lm = lightSpace * d.world;
            cmd->BindVertexBuffer(d.mesh->vertexBuffer.get());
            cmd->BindIndexBuffer(d.mesh->indexBuffer.get(), RHIIndexType::U32);
            cmd->PushConstants(RHIShaderStage::Vertex, 0, sizeof(glm::mat4), &lm);
            cmd->DrawIndexed(d.mesh->indexCount);
        }
    }

    void RebuildDrawList(Scene& scene, const Frustum& frustum, const glm::vec3& cameraPos,
                         View& v, int viewIdx) {
        v.drawList.clear();
        v.skinnedDraws.clear();
        v.transparentDraws.clear();
        m_ShadowDraws.clear();          // unculled + view-independent; refilling is idempotent
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
                if (!frustum.TestAABB(it->second.bounds.Transform(world))) continue;
                v.transparentDraws.push_back({
                    &it->second, GetOrCreateTransparentSet(mc.material.get(), v), world,
                    glm::length(glm::vec3(world[3]) - cameraPos) });
                continue;
            }

            const DrawItem item{ &it->second,
                                 GetOrCreateMaterialSet(mc.material.get(), viewIdx), world };
            if (mc.castsShadow)
                m_ShadowDraws.push_back(item);
            if (frustum.TestAABB(it->second.bounds.Transform(world)))
                v.drawList.push_back(item);
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

            for (const auto& meshPtr : smc.meshes) {
                const GpuMesh* gm = GetOrRegisterSkinnedMesh(meshPtr.get());
                if (!gm) continue;
                if (smc.castsShadow)
                    m_SkinnedShadowDraws.push_back({ gm, bones, world });
                if (visible)
                    v.skinnedDraws.push_back({ gm, material, bones, world });
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
                params.uvScale = mat->UVScale;
                // Strength 0 disables the contribution when no map is bound (GL parity).
                params.emissiveStrength = mat->Emissive ? mat->EmissiveStrength : 0.0f;
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
        bd.size    = sizeof(glm::mat4) * kMaxBones;
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
    void UpdateSkinnedPalettes(Scene& scene) {
        std::array<glm::mat4, kMaxBones> palette;
        for (auto [entity, smc] : scene.GetRegistry().view<SkinnedMeshComponent>().each()) {
            if (!smc.visible || smc.meshes.empty()) continue;

            const AnimatorComponent* anim =
                scene.GetRegistry().try_get<AnimatorComponent>(entity);
            const size_t count = (anim && !anim->palette.empty())
                ? std::min(anim->palette.size(), static_cast<size_t>(kMaxBones)) : 0;

            for (size_t i = 0; i < count; ++i)          palette[i] = anim->palette[i];
            for (size_t i = count; i < kMaxBones; ++i)  palette[i] = glm::mat4(1.0f);

            GetOrCreateSkinned(entity).bonesUBO->Update(palette.data(), sizeof(palette));
        }
    }

    void GatherLights(Scene& scene) {
        m_PointPos.clear();
        m_PointColor.clear();
        m_SpotLights.clear();
        m_SunDir   = glm::vec3(0.0f, -1.0f, 0.0f);
        m_SunColor = glm::vec3(0.0f);
        bool foundSun = false;

        const TransformSystem& ts = scene.GetTransformSystem();
        for (auto [entity, lc] : scene.GetRegistry().view<LightComponent>().each()) {
            const glm::mat4& world = ts.GetWorldMatrix(entity);
            const glm::vec3 pos = glm::vec3(world[3]);
            const glm::vec3 dir = glm::normalize(glm::mat3(world) * glm::vec3(0.0f, -1.0f, 0.0f));

            if (lc.type == LightType::Point && m_PointPos.size() < 4) {
                m_PointPos.push_back(pos);
                m_PointColor.push_back(lc.color * lc.intensity);
            } else if (lc.type == LightType::Spot
                       && m_SpotLights.size() < VulkanSpotShadowPass::MAX_SPOTS) {
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
        const std::string spvDir = DIAMOND_VULKAN_SHADER_DIR;
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
    std::vector<DrawItem> m_ShadowDraws;   // unculled casters — CSM/spot/point passes

    // Per-skinned-entity GPU state (SkinnedInstance defined above), keyed by entity.
    std::unordered_map<entt::entity, SkinnedInstance> m_Skinned;
    std::vector<SkinnedShadowItem> m_SkinnedShadowDraws;   // unculled skinned casters
    // Albedo textures behind cached transparency sets, kept alive with them.
    std::unordered_map<RHITexture*, std::shared_ptr<Texture>> m_TransparentAlbedos;

    float                  m_Exposure = 1.0f;   // re-applied when a resize recreates a tonemap pass
    glm::vec3              m_SunDir   { 0.0f, -1.0f, 0.0f };
    glm::vec3              m_SunColor { 0.0f };
    std::vector<glm::vec3> m_PointPos;      // world space
    std::vector<glm::vec3> m_PointColor;
    std::vector<SpotLightInfo> m_SpotLights;

    // The camera the graph's particle callback billboards against — set per view
    // right before that view's Execute (read at record time).
    glm::mat4 m_FrameView { 1.0f };
    glm::mat4 m_FrameProj { 1.0f };

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

} // namespace Diamond

#else  // !DIAMOND_ENABLE_VULKAN — no backend to render through.

namespace Diamond {
std::unique_ptr<SceneRenderer> SceneRenderer::Create(RHIDevice*, uint32_t, uint32_t, bool) {
    return nullptr;
}
} // namespace Diamond

#endif
