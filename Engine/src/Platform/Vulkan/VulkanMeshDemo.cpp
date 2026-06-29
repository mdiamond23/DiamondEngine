#include "Platform/VulkanDemo.h"
#include "Renderer/RHI/RHIDevice.h"
#include "Renderer/RHI/RHICommandList.h"
#include "Renderer/RHI/RHIRenderGraph.h"
#include "Platform/Vulkan/Passes/VulkanPassCommon.h"
#include "Platform/Vulkan/Passes/PostProcess/VulkanTonemapPass.h"
#include "Platform/Vulkan/Passes/PostProcess/VulkanFXAAPass.h"
#include "Platform/Vulkan/Passes/PostProcess/VulkanBloomPass.h"
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

// Matches mesh.vert's vertex inputs — a repacked subset of the engine's full
// MeshData::Vertex (position + normal + UV).
struct MeshVertex {
    glm::vec3 pos;
    glm::vec3 normal;
    glm::vec2 uv;
};

struct FrameUBO {
    glm::mat4 viewProj;
    glm::vec4 lightDir;
    float     time;
};

struct PushConstants {
    glm::mat4 model;
};

// The offscreen scene is rendered at a fixed resolution and then tonemapped onto
// the swapchain by the post pass, so the cube pass is independent of window size.
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
    GLFWwindow* window = glfwCreateWindow(1280, 720, "DiamondEngine — Vulkan RHI Offscreen", nullptr, nullptr);
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
        vertices.push_back({ v.Position, v.Normal, v.TexCoords });
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

    // Checkerboard texture for the cube (static, uploaded once).
    const std::vector<uint8_t> texPixels = MakeCheckerboard(256, 32);
    RHITextureDesc texDesc;
    texDesc.width       = 256;
    texDesc.height      = 256;
    texDesc.format      = RHIFormat::RGBA8;
    texDesc.usage       = RHITextureUsage::Sampled;
    texDesc.initialData = texPixels.data();
    auto checker = device->CreateTexture(texDesc);

    // ── Render graph + its offscreen targets ────────────────────────────────────
    // The graph owns the transient color/depth textures (pooled, never freed
    // mid-frame) and, from the per-pass reads/writes, drives the layout barriers
    // that the manual two-pass version wrote by hand.
    RHIRenderGraph graph(device.get());
    // HDR color so the tonemap pass has real high-range input to map down.
    const RGTextureHandle sceneColor = graph.DeclareTexture(
        "SceneColor", { kOffscreenW, kOffscreenH, RHIFormat::RGBA16F });
    const RGTextureHandle sceneDepth = graph.DeclareTexture(
        "SceneDepth", { kOffscreenW, kOffscreenH, RHIFormat::Depth32F });
    // Bloom adds its blurred bright-pass back onto the scene, still in HDR; the
    // tonemap pass then maps that to LDR. Post chain is:
    //   HDR scene → bloom → HDR(scene+bloom) → tonemap → LDR → FXAA → backbuffer.
    const RGTextureHandle sceneBloomed = graph.DeclareTexture(
        "SceneBloomed", { kOffscreenW, kOffscreenH, RHIFormat::RGBA16F });
    const RGTextureHandle ldrColor = graph.DeclareTexture(
        "LdrColor", { kOffscreenW, kOffscreenH, RHIFormat::RGBA8 });

    // Per-frame UBO (dynamic).
    RHIBufferDesc uboDesc;
    uboDesc.size    = sizeof(FrameUBO);
    uboDesc.usage   = RHIBufferUsage::Uniform;
    uboDesc.dynamic = true;
    auto uniformBuffer = device->CreateBuffer(uboDesc);

    const std::string shaderDir = DIAMOND_VULKAN_SHADER_DIR;

    // ── Cube pipeline (renders into the offscreen color + depth targets) ────────
    const std::vector<uint32_t> meshVsSpv = LoadSpirv(shaderDir, "mesh.vert.spv");
    const std::vector<uint32_t> meshFsSpv = LoadSpirv(shaderDir, "mesh.frag.spv");
    RHIShaderDesc meshVs{ RHIShaderStage::Vertex,   meshVsSpv.data(), meshVsSpv.size() };
    RHIShaderDesc meshFs{ RHIShaderStage::Fragment, meshFsSpv.data(), meshFsSpv.size() };
    auto meshVertShader = device->CreateShader(meshVs);
    auto meshFragShader = device->CreateShader(meshFs);

    RHIPipelineDesc meshPipeDesc;
    meshPipeDesc.vertexShader   = meshVertShader.get();
    meshPipeDesc.fragmentShader = meshFragShader.get();
    meshPipeDesc.vertexLayout.stride = sizeof(MeshVertex);
    meshPipeDesc.vertexLayout.attributes = {
        { 0, RHIVertexFormat::Float3, offsetof(MeshVertex, pos)    },
        { 1, RHIVertexFormat::Float3, offsetof(MeshVertex, normal) },
        { 2, RHIVertexFormat::Float2, offsetof(MeshVertex, uv)     },
    };
    meshPipeDesc.resourceBindings = {
        { 0, RHIResourceType::UniformBuffer,        RHIShaderStage::Vertex | RHIShaderStage::Fragment },
        { 1, RHIResourceType::CombinedImageSampler, RHIShaderStage::Fragment },
    };
    meshPipeDesc.pushConstants = { RHIShaderStage::Vertex, sizeof(PushConstants) };
    meshPipeDesc.colorFormat   = RHIFormat::RGBA16F;      // offscreen HDR color format
    meshPipeDesc.depthFormat   = RHIFormat::Depth32F;     // offscreen depth format
    meshPipeDesc.depthTest     = true;
    meshPipeDesc.depthWrite    = true;
    auto meshPipeline = device->CreatePipeline(meshPipeDesc);

    auto meshSet = device->CreateResourceSet(
        meshPipeline.get(), 0, { { 0, uniformBuffer.get() } }, { { 1, checker.get() } });

    // ── Post-process chain (each a self-contained ported Vulkan pass) ───────────
    // Bloom adds a blurred bright-pass back onto the HDR scene; tonemap maps the
    // result to LDR; FXAA anti-aliases that and writes the swapchain. Each pass
    // owns its pipeline + shaders and wires itself into the graph with the
    // reads/writes that drive ordering.
    VulkanBloomPass   bloom(device.get(), shaderDir, kOffscreenW, kOffscreenH,
                            RHIFormat::RGBA16F);
    VulkanTonemapPass tonemap(device.get(), shaderDir, RHIFormat::RGBA8);
    VulkanFXAAPass    fxaa(device.get(), shaderDir, device->SwapchainFormat());

    const float aspect = static_cast<float>(kOffscreenW) / kOffscreenH;

    // Per-frame push constants, written before each Execute and read inside the
    // cube pass's recorded body. The graph is built and compiled once; only this
    // data and the UBO change per frame.
    PushConstants push{};

    // Pass 1 — cube into the offscreen color + depth targets. Writing both makes
    // the graph open a color+depth render scope and clear them.
    graph.AddPass("Cube")
        .Write(sceneColor)
        .Write(sceneDepth)
        .SetClearColor({ 0.02f, 0.02f, 0.05f, 1.0f })
        .SetExecute([&](RHICommandList* cmd) {
            cmd->BindPipeline(meshPipeline.get());
            cmd->BindResourceSet(0, meshSet.get());
            cmd->PushConstants(RHIShaderStage::Vertex, 0, sizeof(push), &push);
            cmd->BindVertexBuffer(vertexBuffer.get());
            cmd->BindIndexBuffer(indexBuffer.get(), RHIIndexType::U32);
            cmd->DrawIndexed(indexCount);
        });

    // Pass 2 — bloom. Declares its own bright + ping-pong scratch on the graph and
    // adds an extract → blur×N → composite sub-chain; reads sceneColor and writes
    // the scene-plus-bloom HDR target. The graph orders the whole sub-chain after
    // the cube pass and inserts every barrier from the reads/writes.
    bloom.AddToGraph(graph, sceneColor, sceneBloomed);

    // Pass 3 — tonemap the bloomed HDR scene into the LDR texture.
    tonemap.AddToGraph(graph, sceneBloomed, ldrColor);

    // Pass 4 — FXAA the LDR texture into the backbuffer.
    fxaa.AddToGraph(graph, ldrColor);

    graph.Compile();

    while (!glfwWindowShouldClose(window)) {
        glfwPollEvents();

        RHICommandList* cmd = device->BeginFrame();
        if (!cmd) continue;   // frame skipped (minimized / swapchain rebuilt)

        const float t = static_cast<float>(glfwGetTime());

        const glm::vec3 eye  = glm::vec3(std::sin(t) * 4.0f, 2.5f, std::cos(t) * 4.0f);
        const glm::mat4 view = glm::lookAt(eye, glm::vec3(0.0f), glm::vec3(0.0f, 1.0f, 0.0f));
        const glm::mat4 proj = glm::perspective(glm::radians(55.0f), aspect, 0.1f, 100.0f);

        FrameUBO ubo{};
        ubo.viewProj = proj * view;
        ubo.lightDir = glm::vec4(glm::normalize(glm::vec3(-0.4f, -1.0f, -0.3f)), 0.0f);
        ubo.time     = t;
        uniformBuffer->Update(&ubo, sizeof(ubo));

        push.model = glm::rotate(glm::mat4(1.0f), t * 0.5f, glm::vec3(0.0f, 1.0f, 0.0f));

        graph.Execute(cmd);

        device->EndFrame();
    }

    device->WaitIdle();
    glfwDestroyWindow(window);
    glfwTerminate();
    return 0;
}

} // namespace Diamond
