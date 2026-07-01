#include "Renderer/SceneRenderer.h"

#ifdef DIAMOND_ENABLE_VULKAN

// Vulkan expects clip-space depth in [0, 1] (OpenGL uses [-1, 1]); make GLM's
// perspective matrix match before any glm header is pulled in.
#define GLM_FORCE_DEPTH_ZERO_TO_ONE

#include "Renderer/RHI/RHIDevice.h"
#include "Renderer/RHI/RHICommandList.h"
#include "Renderer/RHI/RHIRenderGraph.h"
#include "Renderer/MeshData.h"
#include "Core/Camera.h"
#include "Scene/Scene.h"
#include "Scene/Components.h"

#include "Platform/Vulkan/Passes/Deferred/VulkanGBufferPass.h"
#include "Platform/Vulkan/Passes/Deferred/VulkanSSAOPass.h"
#include "Platform/Vulkan/Passes/Deferred/VulkanDeferredLightingPass.h"
#include "Platform/Vulkan/Passes/Shadows/VulkanCSMPass.h"
#include "Platform/Vulkan/Passes/PostProcess/VulkanTonemapPass.h"
#include "Platform/Vulkan/Passes/IBL/VulkanIBLPass.h"

#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <spdlog/spdlog.h>

#include <array>
#include <unordered_map>
#include <vector>

namespace Diamond {

namespace {

// gbuffer.vert's vertex inputs — a repacked subset of the engine's full Vertex
// (position + normal + UV). MeshCache repacks into this layout on upload.
struct MeshVertex {
    glm::vec3 pos;
    glm::vec3 normal;
    glm::vec2 uv;
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

// Default albedo until per-material textures land (Slice 3): a checkerboard so
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

} // namespace

// ── Concrete Vulkan implementation ───────────────────────────────────────────
class SceneRendererVk final : public SceneRenderer {
public:
    SceneRendererVk(RHIDevice* device, uint32_t width, uint32_t height)
        : m_Device(device), m_Width(width), m_Height(height), m_Graph(device) {
        BuildResources();
        BuildGraph();
    }

    void RegisterMesh(Mesh* key, const MeshData& data) override {
        if (!key || m_Meshes.count(key)) return;

        std::vector<MeshVertex> verts;
        verts.reserve(data.Vertices.size());
        for (const Vertex& v : data.Vertices)
            verts.push_back({ v.Position, v.Normal, v.TexCoords });

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

        m_Meshes.emplace(key, std::move(gm));
    }

    void SetEnvironment(const std::string& hdrPath) override {
        m_IBL->BakeEnvironment(hdrPath);
        m_Lighting->BindIBL(*m_IBL);
    }

    void RenderToSwapchain(Scene& scene, const Camera& camera,
                           const OverlayFn& overlay) override {
        // Refresh world transforms, then gather this frame's draws + lights from
        // the ECS (the pass draw-callbacks iterate m_DrawList captured at build).
        scene.GetTransformSystem().Update(scene.GetRegistry());
        RebuildDrawList(scene);
        GatherLights(scene);

        RHICommandList* cmd = m_Device->BeginFrame();
        if (!cmd) return;   // swapchain unavailable (minimized / rebuilding)

        const float aspect = static_cast<float>(m_Width) / static_cast<float>(m_Height);
        const glm::mat4 view = camera.GetViewMatrix();
        const glm::mat4 proj = glm::perspective(glm::radians(camera.Zoom), aspect, kNear, kFar);

        CameraUBO ubo{};
        ubo.view     = view;
        ubo.viewProj = proj * view;
        m_CameraUBO->Update(&ubo, sizeof(ubo));

        // Per-frame pass data (uploaded after BeginFrame frees the frame's slot).
        m_SSAO->SetProjection(proj);
        m_CSM->ComputeCascades(m_SunDir, view, proj, kNear, kShadowFar);
        m_Lighting->SetFrameData(view, m_SunDir, m_SunColor,
                                 m_CSM->GetLightMatrices(), m_CSM->GetSplitDepths(),
                                 m_PointPos, m_PointColor);

        m_Graph.Execute(cmd);

        // Composite an optional overlay (ImGui) over the tonemapped swapchain,
        // loading its contents so the scene shows through. The backbuffer is still
        // in COLOR_ATTACHMENT layout here; EndFrame transitions it to present.
        if (overlay) {
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
    };
    struct DrawItem {
        const GpuMesh* mesh;
        glm::mat4      world;
    };

    void BuildResources() {
        // Per-frame camera UBO (dynamic — rewritten every frame).
        RHIBufferDesc ubo;
        ubo.size    = sizeof(CameraUBO);
        ubo.usage   = RHIBufferUsage::Uniform;
        ubo.dynamic = true;
        m_CameraUBO = m_Device->CreateBuffer(ubo);

        // Shared default albedo (per-material textures = Slice 3).
        const std::vector<uint8_t> pixels = MakeCheckerboard(256, 32);
        RHITextureDesc tex;
        tex.width       = 256;
        tex.height      = 256;
        tex.format      = RHIFormat::RGBA8;
        tex.usage       = RHITextureUsage::Sampled;
        tex.initialData = pixels.data();
        m_Albedo = m_Device->CreateTexture(tex);

        const std::string shaderDir = DIAMOND_VULKAN_SHADER_DIR;
        m_GBuffer  = std::make_unique<VulkanGBufferPass>(m_Device, shaderDir);
        m_SSAO     = std::make_unique<VulkanSSAOPass>(m_Device, shaderDir, m_Width, m_Height);
        m_CSM      = std::make_unique<VulkanCSMPass>(m_Device, shaderDir);
        m_Lighting = std::make_unique<VulkanDeferredLightingPass>(m_Device, shaderDir);
        m_Tonemap  = std::make_unique<VulkanTonemapPass>(m_Device, shaderDir, m_Device->SwapchainFormat());

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

        // G-buffer — fill from the per-frame draw list.
        m_GBuffer->AddToGraph(m_Graph, gViewPos, gViewNormal, gAlbedo, gMaterial, gEmissive,
                              gDepth, m_CameraUBO.get(), m_Albedo.get(),
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

        // Tonemap HDR → swapchain (ACES + gamma).
        m_Tonemap->AddToGraph(m_Graph, hdrLit);

        m_Graph.Compile();
    }

    void DrawGeometry(RHICommandList* cmd) {
        for (const DrawItem& d : m_DrawList) {
            cmd->BindVertexBuffer(d.mesh->vertexBuffer.get());
            cmd->BindIndexBuffer(d.mesh->indexBuffer.get(), RHIIndexType::U32);
            cmd->PushConstants(RHIShaderStage::Vertex, 0, sizeof(glm::mat4), &d.world);
            cmd->DrawIndexed(d.mesh->indexCount);
        }
    }

    void DrawShadow(RHICommandList* cmd, const glm::mat4& lightSpace) {
        for (const DrawItem& d : m_DrawList) {
            const glm::mat4 lm = lightSpace * d.world;
            cmd->BindVertexBuffer(d.mesh->vertexBuffer.get());
            cmd->BindIndexBuffer(d.mesh->indexBuffer.get(), RHIIndexType::U32);
            cmd->PushConstants(RHIShaderStage::Vertex, 0, sizeof(glm::mat4), &lm);
            cmd->DrawIndexed(d.mesh->indexCount);
        }
    }

    void RebuildDrawList(Scene& scene) {
        m_DrawList.clear();
        const TransformSystem& ts = scene.GetTransformSystem();
        for (auto [entity, mc] : scene.GetRegistry().view<MeshComponent>().each()) {
            if (!mc.visible || !mc.mesh) continue;
            auto it = m_Meshes.find(mc.mesh.get());
            if (it == m_Meshes.end()) continue;   // geometry not registered
            m_DrawList.push_back({ &it->second, ts.GetWorldMatrix(entity) });
        }
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

    RHIDevice*      m_Device;
    uint32_t        m_Width;
    uint32_t        m_Height;
    RHIRenderGraph  m_Graph;

    std::unique_ptr<RHIBuffer>  m_CameraUBO;
    std::unique_ptr<RHITexture> m_Albedo;

    std::unique_ptr<VulkanGBufferPass>          m_GBuffer;
    std::unique_ptr<VulkanSSAOPass>             m_SSAO;
    std::unique_ptr<VulkanCSMPass>              m_CSM;
    std::unique_ptr<VulkanDeferredLightingPass> m_Lighting;
    std::unique_ptr<VulkanTonemapPass>          m_Tonemap;
    std::unique_ptr<VulkanIBLPass>              m_IBL;

    std::unordered_map<Mesh*, GpuMesh> m_Meshes;
    std::vector<DrawItem>              m_DrawList;

    glm::vec3              m_SunDir   { 0.0f, -1.0f, 0.0f };
    glm::vec3              m_SunColor { 0.0f };
    std::vector<glm::vec3> m_PointPos;
    std::vector<glm::vec3> m_PointColor;
};

std::unique_ptr<SceneRenderer> SceneRenderer::Create(RHIDevice* device,
                                                     uint32_t width, uint32_t height) {
    if (!device) return nullptr;
    return std::make_unique<SceneRendererVk>(device, width, height);
}

} // namespace Diamond

#else  // !DIAMOND_ENABLE_VULKAN — no backend to render through.

namespace Diamond {
std::unique_ptr<SceneRenderer> SceneRenderer::Create(RHIDevice*, uint32_t, uint32_t) {
    return nullptr;
}
} // namespace Diamond

#endif
