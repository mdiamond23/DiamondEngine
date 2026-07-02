#include "Platform/VulkanDemo.h"
#include "Renderer/RHI/RHIDevice.h"
#include "Renderer/RHI/RHICommandList.h"
#include "Renderer/RHI/RHIRenderGraph.h"
#include "Platform/Vulkan/Passes/VulkanPassCommon.h"
#include "Platform/Vulkan/Passes/Deferred/VulkanGBufferPass.h"
#include "Platform/Vulkan/Passes/Deferred/VulkanSSAOPass.h"
#include "Platform/Vulkan/Passes/Deferred/VulkanDeferredLightingPass.h"
#include "Platform/Vulkan/Passes/Forward/VulkanPBRSurfacePass.h"
#include "Platform/Vulkan/Passes/Forward/VulkanTransparencyPass.h"
#include "Platform/Vulkan/Passes/Shadows/VulkanCSMPass.h"
#include "Platform/Vulkan/Passes/PostProcess/VulkanTonemapPass.h"
#include "Platform/Vulkan/Passes/IBL/VulkanIBLPass.h"
#include "Renderer/MeshData.h"

// Vulkan expects clip-space depth in [0, 1] (OpenGL uses [-1, 1]); make GLM's
// perspective matrix match before any glm header is pulled in.
#define GLM_FORCE_DEPTH_ZERO_TO_ONE
#define GLFW_INCLUDE_NONE
#include <GLFW/glfw3.h>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <spdlog/spdlog.h>

#include <array>
#include <cmath>
#include <cstdint>
#include <vector>

namespace Diamond {

namespace {

// Matches gbuffer.vert's vertex inputs — a repacked subset of the engine's full
// MeshData::Vertex (position + normal + UV + tangent).
struct MeshVertex {
    glm::vec3 pos;
    glm::vec3 normal;
    glm::vec2 uv;
    glm::vec3 tangent;
};

// std140 layout of gbuffer.frag's per-material MaterialUBO.
struct MaterialParams {
    float uvScale          = 1.0f;
    float emissiveStrength = 0.0f;
    float _pad0 = 0.0f, _pad1 = 0.0f;
};

// Per-frame camera data for the G-buffer pass. view alone produces view-space
// position/normal; viewProj transforms to clip space.
struct GBufferUBO {
    glm::mat4 view;
    glm::mat4 viewProj;
};

struct PushConstants {
    glm::mat4 model;
};

// The G-buffer is filled at a fixed offscreen resolution and then visualized onto
// the swapchain by the debug pass, so the geometry pass is window-size independent.
constexpr uint32_t kOffscreenW = 1280;
constexpr uint32_t kOffscreenH = 720;

std::vector<uint8_t> MakeCheckerboard(uint32_t size, uint32_t cell) {
    std::vector<uint8_t> pixels(static_cast<size_t>(size) * size * 4);
    for (uint32_t y = 0; y < size; ++y) {
        for (uint32_t x = 0; x < size; ++x) {
            const bool even = ((x / cell) + (y / cell)) % 2 == 0;
            uint8_t* p = &pixels[(static_cast<size_t>(y) * size + x) * 4];
            p[0] = even ? 230 : 40;
            p[1] = even ? 180 : 60;
            p[2] = even ? 70  : 90;
            p[3] = 255;
        }
    }
    return pixels;
}

// Flat RGBA texture — the transparent object's material (alpha < 1 so it blends).
std::vector<uint8_t> MakeSolid(uint32_t size, uint8_t r, uint8_t g, uint8_t b, uint8_t a) {
    std::vector<uint8_t> pixels(static_cast<size_t>(size) * size * 4);
    for (size_t i = 0; i < static_cast<size_t>(size) * size; ++i) {
        pixels[i * 4 + 0] = r;
        pixels[i * 4 + 1] = g;
        pixels[i * 4 + 2] = b;
        pixels[i * 4 + 3] = a;
    }
    return pixels;
}

void OnFramebufferResize(GLFWwindow* window, int /*w*/, int /*h*/) {
    auto* device = static_cast<RHIDevice*>(glfwGetWindowUserPointer(window));
    if (device) device->NotifyResize();
}

} // namespace

int RunVulkanMeshDemo() {
    if (!glfwInit()) {
        spdlog::critical("[RHI] glfwInit failed");
        return 1;
    }
    if (!glfwVulkanSupported()) {
        spdlog::critical("[RHI] GLFW reports no Vulkan loader present");
        glfwTerminate();
        return 1;
    }

    glfwWindowHint(GLFW_CLIENT_API, GLFW_NO_API);   // Vulkan window
    GLFWwindow* window = glfwCreateWindow(1280, 720, "DiamondEngine — Vulkan Deferred Lighting", nullptr, nullptr);
    if (!window) {
        spdlog::critical("[RHI] glfwCreateWindow failed");
        glfwTerminate();
        return 1;
    }

    std::unique_ptr<RHIDevice> device = RHIDevice::Create(window, RHIBackend::Vulkan);
    if (!device) {
        spdlog::critical("[RHI] device creation failed");
        glfwDestroyWindow(window);
        glfwTerminate();
        return 1;
    }

    glfwSetWindowUserPointer(window, device.get());
    glfwSetFramebufferSizeCallback(window, OnFramebufferResize);

    // ── Cube geometry (from the engine's mesh factory) ──────────────────────────
    const MeshData cube = MeshData::UnitCube();
    std::vector<MeshVertex> vertices;
    vertices.reserve(cube.Vertices.size());
    for (const Vertex& v : cube.Vertices)
        vertices.push_back({ v.Position, v.Normal, v.TexCoords, v.Tangent });
    const std::vector<uint32_t>& indices = cube.Indices;

    RHIBufferDesc vbDesc;
    vbDesc.size        = vertices.size() * sizeof(MeshVertex);
    vbDesc.usage       = RHIBufferUsage::Vertex;
    vbDesc.initialData = vertices.data();
    auto vertexBuffer = device->CreateBuffer(vbDesc);

    RHIBufferDesc ibDesc;
    ibDesc.size        = indices.size() * sizeof(uint32_t);
    ibDesc.usage       = RHIBufferUsage::Index;
    ibDesc.initialData = indices.data();
    auto indexBuffer = device->CreateBuffer(ibDesc);
    const uint32_t indexCount = static_cast<uint32_t>(indices.size());

    // Checkerboard texture — the cube's albedo (static, uploaded once).
    const std::vector<uint8_t> texPixels = MakeCheckerboard(256, 32);
    RHITextureDesc texDesc;
    texDesc.width       = 256;
    texDesc.height      = 256;
    texDesc.format      = RHIFormat::RGBA8;
    texDesc.usage       = RHITextureUsage::Sampled;
    texDesc.initialData = texPixels.data();
    auto checker = device->CreateTexture(texDesc);

    // Translucent material for the forward transparency pass (cyan glass, alpha 0.4).
    const std::vector<uint8_t> glassPixels = MakeSolid(4, 90, 200, 230, 100);
    RHITextureDesc glassDesc;
    glassDesc.width       = 4;
    glassDesc.height      = 4;
    glassDesc.format      = RHIFormat::RGBA8;
    glassDesc.usage       = RHITextureUsage::Sampled;
    glassDesc.initialData = glassPixels.data();
    auto glass = device->CreateTexture(glassDesc);

    // ── Render graph + its G-buffer targets ─────────────────────────────────────
    // The graph owns the five transient G-buffer attachments (pooled, never freed
    // mid-frame) + the shared depth target, and drives the layout barriers from the
    // per-pass reads/writes. Formats mirror the GL G-buffer.
    RHIRenderGraph graph(device.get());
    const RGTextureHandle gViewPos    = graph.DeclareTexture(
        "gViewPos",    { kOffscreenW, kOffscreenH, RHIFormat::RGBA16F });
    const RGTextureHandle gViewNormal = graph.DeclareTexture(
        "gViewNormal", { kOffscreenW, kOffscreenH, RHIFormat::RGBA16F });
    const RGTextureHandle gAlbedo     = graph.DeclareTexture(
        "gAlbedo",     { kOffscreenW, kOffscreenH, RHIFormat::RGBA8 });
    const RGTextureHandle gMaterial   = graph.DeclareTexture(
        "gMaterial",   { kOffscreenW, kOffscreenH, RHIFormat::RGBA8 });
    const RGTextureHandle gEmissive   = graph.DeclareTexture(
        "gEmissive",   { kOffscreenW, kOffscreenH, RHIFormat::RGBA16F });
    const RGTextureHandle gDepth      = graph.DeclareTexture(
        "gDepth",      { kOffscreenW, kOffscreenH, RHIFormat::Depth32F });
    // SSAO targets — single-channel occlusion (raw, then blurred). Declared at the
    // top level (mirrors the GL graph) so the debug pass can also show the raw one.
    const RGTextureHandle ssaoRaw     = graph.DeclareTexture(
        "ssaoRaw",     { kOffscreenW, kOffscreenH, RHIFormat::R16F });
    const RGTextureHandle ssaoBlurred = graph.DeclareTexture(
        "ssaoBlurred", { kOffscreenW, kOffscreenH, RHIFormat::R16F });
    // CSM cascade depth targets — one square depth map per cascade (Depth32F, which
    // the graph now also makes Sampled so the debug pass / future lighting can read).
    constexpr uint32_t kShadowRes = 2048;
    std::array<RGTextureHandle, VulkanCSMPass::NUM_CASCADES> csmCascades;
    for (int i = 0; i < VulkanCSMPass::NUM_CASCADES; ++i)
        csmCascades[i] = graph.DeclareTexture(
            "csmCascade" + std::to_string(i),
            { kShadowRes, kShadowRes, RHIFormat::Depth32F });
    // HDR scene color — the deferred lighting pass resolves the G-buffer into this,
    // then tonemap maps it to the swapchain.
    const RGTextureHandle hdrLit = graph.DeclareTexture(
        "hdrLit", { kOffscreenW, kOffscreenH, RHIFormat::RGBA16F });
    // Forward stages compose over the deferred result into their own HDR targets
    // (keeps the graph single-writer): deferred → hdrLit → forward → transparency.
    const RGTextureHandle hdrForward = graph.DeclareTexture(
        "hdrForward", { kOffscreenW, kOffscreenH, RHIFormat::RGBA16F });
    const RGTextureHandle hdrFinal = graph.DeclareTexture(
        "hdrFinal", { kOffscreenW, kOffscreenH, RHIFormat::RGBA16F });

    // Per-frame UBO (dynamic).
    RHIBufferDesc uboDesc;
    uboDesc.size    = sizeof(GBufferUBO);
    uboDesc.usage   = RHIBufferUsage::Uniform;
    uboDesc.dynamic = true;
    auto uniformBuffer = device->CreateBuffer(uboDesc);

    const std::string shaderDir = DIAMOND_VULKAN_SHADER_DIR;

    // ── Deferred passes (the ported passes under test) ──────────────────────────
    VulkanGBufferPass          gbuffer(device.get(), shaderDir);
    VulkanSSAOPass             ssao(device.get(), shaderDir, kOffscreenW, kOffscreenH);
    VulkanCSMPass              csm(device.get(), shaderDir);
    VulkanDeferredLightingPass lighting(device.get(), shaderDir);
    VulkanPBRSurfacePass       pbrSurface(device.get(), shaderDir);   // skybox + forward object
    VulkanTransparencyPass     transparency(device.get(), shaderDir);
    VulkanTonemapPass          tonemap(device.get(), shaderDir, device->SwapchainFormat());

    // Bake the IBL maps once from an equirectangular HDR sky. The deferred-lighting
    // pass samples the resulting irradiance/prefilter/BRDF maps for its ambient term
    // (replacing the old constant placeholder).
    VulkanIBLPass ibl(device.get(), shaderDir);
    ibl.BakeEnvironment(DIAMOND_ASSETS_DIR "/Textures/citrus_orchard_road_puresky_4k.hdr");

    // Scene lights consumed by the deferred-lighting pass. The sun drives the CSM
    // shadow; a single warm point light (world space) shows the chain resolves more
    // than just the directional term. Ambient now comes from the baked IBL maps.
    const glm::vec3 sunDir   = glm::normalize(glm::vec3(-0.5f, -1.0f, -0.3f));
    const glm::vec3 sunColor = glm::vec3(3.0f, 2.9f, 2.7f);
    const std::vector<glm::vec3> pointPositions = { glm::vec3(1.8f, 1.2f, 1.8f) };
    const std::vector<glm::vec3> pointColors    = { glm::vec3(6.0f, 2.5f, 1.0f) };

    const float aspect = static_cast<float>(kOffscreenW) / kOffscreenH;

    // Per-draw model matrices. The cube spins (updated per frame); the floor is a
    // flattened slab the cube sits on, giving SSAO a contact crease to darken (a lone
    // convex cube self-occludes almost nothing). The graph is built + compiled once;
    // only these matrices and the per-frame UBOs change.
    glm::mat4 cubeModel{ 1.0f };
    const glm::mat4 floorModel =
        glm::translate(glm::mat4(1.0f), glm::vec3(0.0f, -0.55f, 0.0f)) *
        glm::scale(glm::mat4(1.0f), glm::vec3(6.0f, 0.1f, 6.0f));
    // A forward-shaded metallic cube off to one side (its own pass, full Cook-Torrance
    // + IBL) and a translucent cube in front of the scene for the transparency pass.
    const glm::mat4 forwardModel =
        glm::translate(glm::mat4(1.0f), glm::vec3(2.0f, 0.1f, 0.0f)) *
        glm::scale(glm::mat4(1.0f), glm::vec3(0.7f));
    const glm::mat4 glassModel =
        glm::translate(glm::mat4(1.0f), glm::vec3(-1.4f, 0.2f, 1.2f)) *
        glm::scale(glm::mat4(1.0f), glm::vec3(0.9f));

    // Four point lights for the forward PBR shader (reuse the deferred point light,
    // zero the rest so they contribute nothing).
    glm::vec3 fwdLightPos[4] = { pointPositions[0], glm::vec3(0.0f), glm::vec3(0.0f), glm::vec3(0.0f) };
    glm::vec3 fwdLightCol[4] = { pointColors[0],    glm::vec3(0.0f), glm::vec3(0.0f), glm::vec3(0.0f) };

    // One demo-wide material for the G-buffer's per-material descriptor set:
    // checkerboard albedo, neutral 1×1s for the other five maps.
    auto makePixel = [&](uint8_t r, uint8_t g, uint8_t b) {
        const uint8_t px[4] = { r, g, b, 255 };
        RHITextureDesc one;
        one.width       = 1;
        one.height      = 1;
        one.format      = RHIFormat::RGBA8;
        one.usage       = RHITextureUsage::Sampled;
        one.initialData = px;
        return device->CreateTexture(one);
    };
    auto flatNormal = makePixel(128, 128, 255);   // tangent-space +Z
    auto black      = makePixel(0, 0, 0);         // metallic 0, emissive off
    auto gray       = makePixel(128, 128, 128);   // roughness 0.5
    auto white      = makePixel(255, 255, 255);   // AO 1

    const MaterialParams matParams;   // uvScale 1, emissive off
    RHIBufferDesc matDesc;
    matDesc.size        = sizeof(MaterialParams);
    matDesc.usage       = RHIBufferUsage::Uniform;
    matDesc.initialData = &matParams;
    auto materialUBO = device->CreateBuffer(matDesc);

    auto materialSet = gbuffer.CreateMaterialSet(
        uniformBuffer.get(),
        { checker.get(), flatNormal.get(), black.get(), gray.get(), white.get(), black.get() },
        materialUBO.get());

    // Pass 1 — fill the G-buffer. The draw callback records the scene (floor + cube)
    // inside the pass scope after the pass binds its pipeline; each draw run binds
    // its material set (one shared material here).
    gbuffer.AddToGraph(graph, gViewPos, gViewNormal, gAlbedo, gMaterial, gEmissive,
                       gDepth,
                       [&](RHICommandList* cmd) {
                           cmd->BindResourceSet(0, materialSet.get());
                           cmd->BindVertexBuffer(vertexBuffer.get());
                           cmd->BindIndexBuffer(indexBuffer.get(), RHIIndexType::U32);
                           cmd->PushConstants(RHIShaderStage::Vertex, 0, sizeof(floorModel), &floorModel);
                           cmd->DrawIndexed(indexCount);
                           cmd->PushConstants(RHIShaderStage::Vertex, 0, sizeof(cubeModel), &cubeModel);
                           cmd->DrawIndexed(indexCount);
                       });

    // Pass 2+3 — SSAO occlusion + blur over the G-buffer's view-space pos/normal.
    ssao.AddToGraph(graph, gViewPos, gViewNormal, ssaoRaw, ssaoBlurred);

    // Shadow passes — render the scene depth into each cascade from the sun. The
    // callback receives the per-cascade light matrix and pushes lightSpace * model.
    csm.AddToGraph(graph, csmCascades,
                   [&](RHICommandList* cmd, const glm::mat4& lightSpace) {
                       cmd->BindVertexBuffer(vertexBuffer.get());
                       cmd->BindIndexBuffer(indexBuffer.get(), RHIIndexType::U32);
                       glm::mat4 lm = lightSpace * floorModel;
                       cmd->PushConstants(RHIShaderStage::Vertex, 0, sizeof(lm), &lm);
                       cmd->DrawIndexed(indexCount);
                       lm = lightSpace * cubeModel;
                       cmd->PushConstants(RHIShaderStage::Vertex, 0, sizeof(lm), &lm);
                       cmd->DrawIndexed(indexCount);
                   });

    // Pass 4 — deferred lighting. Resolves the G-buffer + blurred SSAO + the four
    // cascade shadow maps into the HDR target with Cook-Torrance PBR (sun + CSM
    // shadow + the point light). Reading all four cascades also keeps cascades 1-3
    // alive through dead-pass culling.
    lighting.AddToGraph(graph, gViewPos, gViewNormal, gAlbedo, gMaterial,
                        ssaoBlurred, gEmissive, csmCascades, hdrLit);
    // Bind the baked IBL maps into the lighting set (raw cube/2D views) now that the
    // set exists. Static maps, so this is a one-time write.
    lighting.BindIBL(ibl);

    // Pass 5 — forward PBR surface: copy the deferred HDR, draw the skybox background
    // (env cubemap at the far plane) + a forward-lit metallic cube over it.
    pbrSurface.AddToGraph(graph, hdrLit, hdrForward, gDepth,
                          [&](RHICommandList* cmd) {   // skybox cube
                              cmd->BindVertexBuffer(vertexBuffer.get());
                              cmd->BindIndexBuffer(indexBuffer.get(), RHIIndexType::U32);
                              cmd->DrawIndexed(indexCount);
                          },
                          [&](RHICommandList* cmd) {   // forward-lit object
                              cmd->BindVertexBuffer(vertexBuffer.get());
                              cmd->BindIndexBuffer(indexBuffer.get(), RHIIndexType::U32);
                              cmd->PushConstants(RHIShaderStage::Vertex, 0, sizeof(forwardModel), &forwardModel);
                              cmd->DrawIndexed(indexCount);
                          });
    pbrSurface.BindIBL(ibl);

    // Pass 6 — forward transparency: copy the forward HDR, blend a translucent cube
    // over it (depth-tested against the scene depth, no depth write).
    transparency.AddToGraph(graph, hdrForward, hdrFinal, gDepth, glass.get(),
                            [&](RHICommandList* cmd) {
                                cmd->BindVertexBuffer(vertexBuffer.get());
                                cmd->BindIndexBuffer(indexBuffer.get(), RHIIndexType::U32);
                                cmd->PushConstants(RHIShaderStage::Vertex, 0, sizeof(glassModel), &glassModel);
                                cmd->DrawIndexed(indexCount);
                            });

    // Pass 7 — tonemap the composited HDR result onto the swapchain (ACES + gamma).
    tonemap.AddToGraph(graph, hdrFinal);

    graph.Compile();

    while (!glfwWindowShouldClose(window)) {
        glfwPollEvents();

        RHICommandList* cmd = device->BeginFrame();
        if (!cmd) continue;   // frame skipped (minimized / swapchain rebuilt)

        const float t = static_cast<float>(glfwGetTime());

        const glm::vec3 eye  = glm::vec3(std::sin(t) * 4.0f, 2.5f, std::cos(t) * 4.0f);
        const glm::mat4 view = glm::lookAt(eye, glm::vec3(0.0f), glm::vec3(0.0f, 1.0f, 0.0f));
        const glm::mat4 proj = glm::perspective(glm::radians(55.0f), aspect, 0.1f, 100.0f);

        GBufferUBO ubo{};
        ubo.view     = view;
        ubo.viewProj = proj * view;
        uniformBuffer->Update(&ubo, sizeof(ubo));

        // SSAO projects view-space samples into screen space — it needs this frame's
        // projection (uploaded after BeginFrame idles the buffer slot).
        ssao.SetProjection(proj);

        cubeModel = glm::rotate(glm::mat4(1.0f), t * 0.5f, glm::vec3(0.0f, 1.0f, 0.0f));

        // Refresh the cascade light matrices from this frame's camera + a fixed sun.
        // The shadowed range (near..far) is tighter than the camera's 100-unit far so
        // the cascades pack around the cube + floor.
        csm.ComputeCascades(sunDir, view, proj, 0.1f, 25.0f);

        // Feed the lighting pass this frame's camera + lights. It folds inverse(view)
        // into each cascade matrix and transforms the sun/point lights to view space.
        lighting.SetFrameData(view, sunDir, sunColor,
                              csm.GetLightMatrices(), csm.GetSplitDepths(),
                              pointPositions, pointColors);

        // Forward passes: same camera. The PBR surface pass also needs the eye
        // position + point lights for its world-space Cook-Torrance shading.
        pbrSurface.SetFrameData(view, proj, eye, fwdLightPos, fwdLightCol);
        transparency.SetCamera(view, proj);

        graph.Execute(cmd);

        device->EndFrame();
    }

    device->WaitIdle();
    glfwDestroyWindow(window);
    glfwTerminate();
    return 0;
}

} // namespace Diamond
