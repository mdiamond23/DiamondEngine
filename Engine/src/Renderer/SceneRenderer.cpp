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

#include "Platform/Vulkan/Passes/Deferred/VulkanGBufferPass.h"
#include "Platform/Vulkan/Passes/Deferred/VulkanSSAOPass.h"
#include "Platform/Vulkan/Passes/Deferred/VulkanDeferredLightingPass.h"
#include "Platform/Vulkan/Passes/Deferred/VulkanSkyboxPass.h"
#include "Platform/Vulkan/Passes/Shadows/VulkanCSMPass.h"
#include "Platform/Vulkan/Passes/PostProcess/VulkanTonemapPass.h"
#include "Platform/Vulkan/Passes/IBL/VulkanIBLPass.h"
#include "Platform/Vulkan/Resources/VulkanParticleRenderer.h"
#include "Platform/Vulkan/Resources/VulkanTexture2D.h"

#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <spdlog/spdlog.h>

#include <algorithm>
#include <array>
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
constexpr uint32_t kShadowRes = 2048;

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
class SceneRendererVk final : public SceneRenderer {
public:
    SceneRendererVk(RHIDevice* device, uint32_t width, uint32_t height, bool offscreen)
        : m_Device(device), m_Width(width), m_Height(height), m_Offscreen(offscreen),
          m_Graph(device) {
        BuildResources();
        BuildGraph();
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
        m_Lighting->BindIBL(*m_IBL);
        m_Skybox->BindIBL(*m_IBL);
    }

    RHITexture* OutputColor() const override {
        return m_Offscreen ? m_Graph.GetTexture(m_OutputColor) : nullptr;
    }

    void SetUIOverlay(const OverlayFn& fn) override { m_UIFn = fn; }

    void Resize(uint32_t width, uint32_t height) override {
        if (!m_Offscreen || width == 0 || height == 0) return;
        if (width == m_Width && height == m_Height) return;

        // Nothing referencing the old targets may be in flight while the pool
        // recreates them and the size-dependent passes rebuild their sets.
        m_Device->WaitIdle();
        m_Width  = width;
        m_Height = height;

        // Recreate the passes whose descriptor sets sample graph textures (the
        // pooled textures are about to be recreated at the new size). The other
        // passes (G-buffer, CSM, particles) hold no graph-texture sets and their
        // pipelines are size-independent, so they re-wire as-is.
        const std::string shaderDir = DIAMOND_VULKAN_SHADER_DIR;
        m_SSAO     = std::make_unique<VulkanSSAOPass>(m_Device, shaderDir, m_Width, m_Height);
        m_Lighting = std::make_unique<VulkanDeferredLightingPass>(m_Device, shaderDir);
        m_Tonemap  = std::make_unique<VulkanTonemapPass>(m_Device, shaderDir, RHIFormat::RGBA8);

        m_Graph.ResetPasses();
        BuildGraph();   // re-declares textures (pool recreates changed sizes) + re-binds IBL
    }

    void RenderToSwapchain(Scene& scene, const Camera& camera,
                           const OverlayFn& overlay) override {
        const float aspect = static_cast<float>(m_Width) / static_cast<float>(m_Height);
        const glm::mat4 view = camera.GetViewMatrix();
        // Explicitly the [0,1]-depth variant: plain glm::perspective resolves its
        // depth range from a per-TU define, and its inline instantiations
        // COMDAT-fold across TUs — a GL TU's [-1,1] version can silently win.
        const glm::mat4 proj = glm::perspectiveRH_ZO(glm::radians(camera.Zoom), aspect, kNear, kFar);
        m_FrameView = view;
        m_FrameProj = proj;

        // Refresh world transforms, then gather this frame's draws + lights from
        // the ECS (the pass draw-callbacks iterate m_DrawList captured at build).
        // Geometry draws are frustum-culled ([0,1]-depth planes — this TU builds
        // proj under GLM_FORCE_DEPTH_ZERO_TO_ONE); shadow draws are not, so
        // off-screen casters still cast into the view (mirrors the GL editor).
        scene.GetTransformSystem().Update(scene.GetRegistry());
        RebuildDrawList(scene, Frustum::Extract(proj * view, /*zeroToOneDepth*/ true));
        GatherLights(scene);
        GatherParticles(scene);

        RHICommandList* cmd = m_Device->BeginFrame();
        if (!cmd) return;   // swapchain unavailable (minimized / rebuilding)

        CameraUBO ubo{};
        ubo.view     = view;
        ubo.viewProj = proj * view;
        m_CameraUBO->Update(&ubo, sizeof(ubo));

        // Per-frame pass data (uploaded after BeginFrame frees the frame's slot).
        m_SSAO->SetProjection(proj);
        m_Skybox->SetFrameData(view, proj);
        m_CSM->ComputeCascades(m_SunDir, view, proj, kNear, kShadowFar);
        m_Lighting->SetFrameData(view, m_SunDir, m_SunColor,
                                 m_CSM->GetLightMatrices(), m_CSM->GetSplitDepths(),
                                 m_PointPos, m_PointColor);

        // Offscreen mode routes the overlay through the graph's EditorOverlay
        // pass (which owns the backbuffer and has already transitioned the scene
        // output for sampling); the pass callback reads this member per frame.
        m_Overlay = overlay;

        m_Graph.Execute(cmd);

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
    struct GpuMesh {
        std::unique_ptr<RHIBuffer> vertexBuffer;
        std::unique_ptr<RHIBuffer> indexBuffer;
        uint32_t                   indexCount = 0;
        AABB                       bounds;
    };
    // A cached material: its descriptor set against the G-buffer pipeline (camera
    // UBO + 6 maps + params UBO), the static params buffer behind it, and the
    // engine textures kept alive for as long as the set references them. Baked
    // once per PBRMaterial pointer — a material edited after first use keeps its
    // baked look until re-registered (live material editing is a later slice).
    struct GpuMaterial {
        std::unique_ptr<RHIBuffer>      params;
        std::unique_ptr<RHIResourceSet> set;
        std::array<std::shared_ptr<Texture>, VulkanGBufferPass::MapCount> keepAlive{};
    };
    struct DrawItem {
        const GpuMesh*  mesh;
        RHIResourceSet* material;   // G-buffer set 0 for this draw
        glm::mat4       world;
    };
    struct PBatch {
        RHITexture* tex = nullptr;
        std::shared_ptr<Texture> texAsset;
        ParticleBlend blend = ParticleBlend::Alpha;
        std::vector<RenderParticle> parts;
    };

    void BuildResources() {
        // Per-frame camera UBO (dynamic — rewritten every frame).
        RHIBufferDesc ubo;
        ubo.size    = sizeof(CameraUBO);
        ubo.usage   = RHIBufferUsage::Uniform;
        ubo.dynamic = true;
        m_CameraUBO = m_Device->CreateBuffer(ubo);

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
        m_GBuffer  = std::make_unique<VulkanGBufferPass>(m_Device, shaderDir);
        m_SSAO     = std::make_unique<VulkanSSAOPass>(m_Device, shaderDir, m_Width, m_Height);
        m_CSM      = std::make_unique<VulkanCSMPass>(m_Device, shaderDir);
        m_Lighting = std::make_unique<VulkanDeferredLightingPass>(m_Device, shaderDir);
        m_Skybox   = std::make_unique<VulkanSkyboxPass>(m_Device, shaderDir);
        // Offscreen mode tonemaps into an LDR texture ImGui can sample; swapchain
        // mode tonemaps straight into the backbuffer.
        m_Tonemap  = std::make_unique<VulkanTonemapPass>(
            m_Device, shaderDir,
            m_Offscreen ? RHIFormat::RGBA8 : m_Device->SwapchainFormat());

        m_Particles = std::make_unique<VulkanParticleRenderer>(m_Device, shaderDir, RHIFormat::RGBA16F);
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

    void BuildGraph() {
        const RGTextureHandle gViewPos    = m_Graph.DeclareTexture("gViewPos",    { m_Width, m_Height, RHIFormat::RGBA16F });
        const RGTextureHandle gViewNormal = m_Graph.DeclareTexture("gViewNormal", { m_Width, m_Height, RHIFormat::RGBA16F });
        const RGTextureHandle gAlbedo     = m_Graph.DeclareTexture("gAlbedo",     { m_Width, m_Height, RHIFormat::RGBA8   });
        const RGTextureHandle gMaterial   = m_Graph.DeclareTexture("gMaterial",   { m_Width, m_Height, RHIFormat::RGBA8   });
        const RGTextureHandle gEmissive   = m_Graph.DeclareTexture("gEmissive",   { m_Width, m_Height, RHIFormat::RGBA16F });
        const RGTextureHandle gDepth      = m_Graph.DeclareTexture("gDepth",      { m_Width, m_Height, RHIFormat::Depth32F });
        const RGTextureHandle ssaoRaw     = m_Graph.DeclareTexture("ssaoRaw",     { m_Width, m_Height, RHIFormat::R16F     });
        const RGTextureHandle ssaoBlurred = m_Graph.DeclareTexture("ssaoBlurred", { m_Width, m_Height, RHIFormat::R16F     });
        const RGTextureHandle hdrLit      = m_Graph.DeclareTexture("hdrLit",      { m_Width, m_Height, RHIFormat::RGBA16F });

        std::array<RGTextureHandle, VulkanCSMPass::NUM_CASCADES> cascades;
        for (int i = 0; i < VulkanCSMPass::NUM_CASCADES; ++i)
            cascades[i] = m_Graph.DeclareTexture("csmCascade" + std::to_string(i),
                                                 { kShadowRes, kShadowRes, RHIFormat::Depth32F });

        // G-buffer — fill from the per-frame draw list; each draw binds its
        // material's descriptor set (built lazily by GetOrCreateMaterial).
        m_GBuffer->AddToGraph(m_Graph, gViewPos, gViewNormal, gAlbedo, gMaterial, gEmissive,
                              gDepth,
                              [this](RHICommandList* cmd) { DrawGeometry(cmd); });

        // SSAO occlusion + blur over the G-buffer.
        m_SSAO->AddToGraph(m_Graph, gViewPos, gViewNormal, ssaoRaw, ssaoBlurred);

        // Shadow cascades — scene depth from the sun; push lightSpace * model.
        m_CSM->AddToGraph(m_Graph, cascades,
                          [this](RHICommandList* cmd, const glm::mat4& lightSpace) {
                              DrawShadow(cmd, lightSpace);
                          });

        // Deferred resolve → HDR. Reads all four cascades (also keeps 1-3 alive).
        m_Lighting->AddToGraph(m_Graph, gViewPos, gViewNormal, gAlbedo, gMaterial,
                               ssaoBlurred, gEmissive, cascades, hdrLit);
        m_Lighting->BindIBL(*m_IBL);

        // Skybox fills the background pixels (far plane, LessEqual against the
        // G-buffer depth) with the baked env cube — the same map the IBL ambient
        // samples, so reflections have a visible source. Inserted after the
        // lighting resolve and before particles; the writes on the shared HDR
        // target resolve by insertion order.
        m_Skybox->AddToGraph(m_Graph, hdrLit, gDepth);
        m_Skybox->BindIBL(*m_IBL);

        // Particles composite over the lit HDR scene (before tonemap).
        m_Particles->AddToGraph(m_Graph, hdrLit, gDepth,
            [this](VulkanParticleRenderer& particles) {
                particles.Begin(m_FrameView, m_FrameProj);
                for (PBatch& b : m_ParticleBatches) {
                    if (!b.texAsset || b.parts.empty()) continue;
                    particles.Draw(b.parts, *b.texAsset, b.blend);
                }
                particles.End();
            });

        // Tonemap HDR → swapchain (ACES + gamma), or → the LDR output texture in
        // offscreen mode.
        if (m_Offscreen)
            m_OutputColor = m_Graph.DeclareTexture("outputColor", { m_Width, m_Height, RHIFormat::RGBA8 });
        m_Tonemap->AddToGraph(m_Graph, hdrLit, m_OutputColor);

        // In-game UI — a 2D pass over the tonemapped LDR scene. Loads (doesn't
        // clear) so widgets composite on top. Ordering: inserted after Tonemap so
        // the write-after-write on the shared target resolves by insertion order.
        {
            RGPass& ui = m_Graph.AddPass("UI2D");
            if (m_Offscreen) ui.Write(m_OutputColor);
            else             ui.WriteSwapchain();
            ui.Load().SetExecute([this](RHICommandList* cmd) {
                if (m_UIFn) m_UIFn(cmd);
            });
        }

        // Editor composite — the only backbuffer owner in offscreen mode. Reading
        // outputColor transitions it to SampledRead, so the overlay (ImGui) can
        // sample it as a viewport image while drawing over a cleared swapchain.
        if (m_Offscreen) {
            m_Graph.AddPass("EditorOverlay")
                .Read(m_OutputColor)
                .WriteSwapchain()
                .SetClearColor({ 0.1f, 0.1f, 0.1f, 1.0f })
                .SetExecute([this](RHICommandList* cmd) {
                    if (m_Overlay) m_Overlay(cmd);
                });
        }

        m_Graph.Compile();
    }

    void DrawGeometry(RHICommandList* cmd) {
        // The list is sorted by material, so the set rebind only fires on runs.
        RHIResourceSet* bound = nullptr;
        for (const DrawItem& d : m_DrawList) {
            if (d.material != bound) {
                cmd->BindResourceSet(0, d.material);
                bound = d.material;
            }
            cmd->BindVertexBuffer(d.mesh->vertexBuffer.get());
            cmd->BindIndexBuffer(d.mesh->indexBuffer.get(), RHIIndexType::U32);
            cmd->PushConstants(RHIShaderStage::Vertex, 0, sizeof(glm::mat4), &d.world);
            cmd->DrawIndexed(d.mesh->indexCount);
        }
    }

    void DrawShadow(RHICommandList* cmd, const glm::mat4& lightSpace) {
        for (const DrawItem& d : m_ShadowDraws) {
            const glm::mat4 lm = lightSpace * d.world;
            cmd->BindVertexBuffer(d.mesh->vertexBuffer.get());
            cmd->BindIndexBuffer(d.mesh->indexBuffer.get(), RHIIndexType::U32);
            cmd->PushConstants(RHIShaderStage::Vertex, 0, sizeof(glm::mat4), &lm);
            cmd->DrawIndexed(d.mesh->indexCount);
        }
    }

    void RebuildDrawList(Scene& scene, const Frustum& frustum) {
        m_DrawList.clear();
        m_ShadowDraws.clear();
        const TransformSystem& ts = scene.GetTransformSystem();
        for (auto [entity, mc] : scene.GetRegistry().view<MeshComponent>().each()) {
            if (!mc.visible || !mc.mesh) continue;
            auto it = m_Meshes.find(mc.mesh.get());
            if (it == m_Meshes.end()) continue;   // geometry not registered

            const glm::mat4& world = ts.GetWorldMatrix(entity);
            const DrawItem item{ &it->second, GetOrCreateMaterial(mc.material.get()).set.get(), world };
            if (mc.castsShadow)
                m_ShadowDraws.push_back(item);
            if (frustum.TestAABB(it->second.bounds.Transform(world)))
                m_DrawList.push_back(item);
        }

        // Group draws by material so DrawGeometry rebinds each set once per run.
        std::sort(m_DrawList.begin(), m_DrawList.end(),
                  [](const DrawItem& a, const DrawItem& b) { return a.material < b.material; });
    }

    // Resolve one engine texture slot to its RHI texture, or a neutral default
    // when the slot is empty (or holds a non-Vulkan texture).
    RHITexture* RhiOrDefault(const std::shared_ptr<Texture>& tex, RHITexture* fallback) const {
        if (const auto* vk = dynamic_cast<const VulkanTexture2D*>(tex.get()); vk && vk->Rhi())
            return vk->Rhi();
        return fallback;
    }

    // Look up (or bake) the G-buffer descriptor set for a material. Null material
    // = the checkerboard default. Sets are built lazily on the first frame a
    // material appears and live for the renderer's lifetime.
    const GpuMaterial& GetOrCreateMaterial(const PBRMaterial* mat) {
        auto it = m_Materials.find(mat);
        if (it != m_Materials.end()) return it->second;

        GpuMaterial gm;

        MaterialParams params;
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

        gm.set = m_GBuffer->CreateMaterialSet(m_CameraUBO.get(), maps, gm.params.get());
        return m_Materials.emplace(mat, std::move(gm)).first->second;
    }

    void GatherLights(Scene& scene) {
        m_PointPos.clear();
        m_PointColor.clear();
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
            } else if (lc.type == LightType::Sun && !foundSun) {
                m_SunDir   = dir;
                m_SunColor = lc.color * lc.intensity;
                foundSun   = true;
            }
            // Spot lights aren't in the deferred-lighting shader's scope yet.
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

    RHIDevice*      m_Device;
    uint32_t        m_Width;
    uint32_t        m_Height;
    bool            m_Offscreen;
    RHIRenderGraph  m_Graph;

    RGTextureHandle m_OutputColor;   // offscreen mode's LDR target; invalid otherwise
    OverlayFn       m_UIFn;          // in-game UI pass body (may be empty)
    OverlayFn       m_Overlay;       // per-frame editor overlay (ImGui), offscreen mode

    std::unique_ptr<RHIBuffer>  m_CameraUBO;
    std::unique_ptr<RHITexture> m_Albedo;          // checkerboard — null-material albedo
    std::unique_ptr<RHITexture> m_DefaultWhite;    // 1×1 fallbacks for empty slots
    std::unique_ptr<RHITexture> m_DefaultNormal;
    std::unique_ptr<RHITexture> m_DefaultBlack;
    std::unique_ptr<RHITexture> m_DefaultGray;

    std::unique_ptr<VulkanGBufferPass>          m_GBuffer;
    std::unique_ptr<VulkanSSAOPass>             m_SSAO;
    std::unique_ptr<VulkanCSMPass>              m_CSM;
    std::unique_ptr<VulkanDeferredLightingPass> m_Lighting;
    std::unique_ptr<VulkanSkyboxPass>           m_Skybox;
    std::unique_ptr<VulkanTonemapPass>          m_Tonemap;
    std::unique_ptr<VulkanIBLPass>              m_IBL;

    std::unordered_map<Mesh*, GpuMesh>                     m_Meshes;
    std::unordered_map<const PBRMaterial*, GpuMaterial>    m_Materials;
    std::vector<DrawItem>              m_DrawList;      // frustum-culled — geometry pass
    std::vector<DrawItem>              m_ShadowDraws;   // unculled casters — CSM pass

    glm::vec3              m_SunDir   { 0.0f, -1.0f, 0.0f };
    glm::vec3              m_SunColor { 0.0f };
    std::vector<glm::vec3> m_PointPos;
    std::vector<glm::vec3> m_PointColor;

    glm::mat4 m_FrameView { 1.0f };
    glm::mat4 m_FrameProj { 1.0f };

    std::unique_ptr<VulkanParticleRenderer> m_Particles;
    std::unique_ptr<RHITexture>             m_DefaultParticleRHI;
    std::shared_ptr<Texture>                m_DefaultParticleTexture;
    std::vector<PBatch>                     m_ParticleBatches;
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
