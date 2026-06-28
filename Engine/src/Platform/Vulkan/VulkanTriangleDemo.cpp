#include "Platform/VulkanDemo.h"
#include "Renderer/RHI/RHIDevice.h"
#include "Renderer/RHI/RHICommandList.h"

#define GLFW_INCLUDE_NONE
#include <GLFW/glfw3.h>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <spdlog/spdlog.h>

#include <array>
#include <cstdint>
#include <fstream>
#include <vector>

namespace Diamond {

namespace {

// Matches triangle.vert's vertex inputs.
struct Vertex {
    glm::vec2 pos;
    glm::vec3 color;
};

// std140 per-frame UBO — mirrors FrameUBO in the shaders.
struct FrameUBO {
    glm::mat4 viewProj;
    float     time;
};

// Per-draw push constant — mirrors Push in triangle.vert.
struct PushConstants {
    glm::mat4 model;
};

std::vector<uint32_t> ReadSpirv(const std::string& path) {
    std::ifstream file(path, std::ios::binary | std::ios::ate);
    if (!file) {
        spdlog::critical("[RHI] failed to open SPIR-V '{}'", path);
        std::abort();
    }
    const std::streamsize bytes = file.tellg();
    std::vector<uint32_t> words(static_cast<size_t>(bytes) / sizeof(uint32_t));
    file.seekg(0);
    file.read(reinterpret_cast<char*>(words.data()), bytes);
    return words;
}

void OnFramebufferResize(GLFWwindow* window, int /*w*/, int /*h*/) {
    auto* device = static_cast<RHIDevice*>(glfwGetWindowUserPointer(window));
    if (device) device->NotifyResize();
}

} // namespace

int RunVulkanTriangleDemo() {
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
    GLFWwindow* window = glfwCreateWindow(1280, 720, "DiamondEngine — Vulkan RHI Triangle", nullptr, nullptr);
    if (!window) {
        spdlog::critical("[RHI] glfwCreateWindow failed");
        glfwTerminate();
        return 1;
    }

    // Everything below this line is backend-neutral RHI pass code: swapping the
    // backend enum is all that picks Vulkan vs (eventually) OpenGL.
    std::unique_ptr<RHIDevice> device = RHIDevice::Create(window, RHIBackend::Vulkan);
    if (!device) {
        spdlog::critical("[RHI] device creation failed");
        glfwDestroyWindow(window);
        glfwTerminate();
        return 1;
    }

    glfwSetWindowUserPointer(window, device.get());
    glfwSetFramebufferSizeCallback(window, OnFramebufferResize);

    // Geometry (static buffers).
    const Vertex vertices[] = {
        { {  0.0f, -0.5f }, { 1.0f, 0.0f, 0.0f } },
        { {  0.5f,  0.5f }, { 0.0f, 1.0f, 0.0f } },
        { { -0.5f,  0.5f }, { 0.0f, 0.0f, 1.0f } },
    };
    const uint16_t indices[] = { 0, 1, 2 };

    RHIBufferDesc vbDesc;
    vbDesc.size        = sizeof(vertices);
    vbDesc.usage       = RHIBufferUsage::Vertex;
    vbDesc.initialData = vertices;
    auto vertexBuffer = device->CreateBuffer(vbDesc);

    RHIBufferDesc ibDesc;
    ibDesc.size        = sizeof(indices);
    ibDesc.usage       = RHIBufferUsage::Index;
    ibDesc.initialData = indices;
    auto indexBuffer = device->CreateBuffer(ibDesc);

    // Per-frame UBO (dynamic — rewritten every frame).
    RHIBufferDesc uboDesc;
    uboDesc.size    = sizeof(FrameUBO);
    uboDesc.usage   = RHIBufferUsage::Uniform;
    uboDesc.dynamic = true;
    auto uniformBuffer = device->CreateBuffer(uboDesc);

    // Shaders.
    const std::string shaderDir = DIAMOND_VULKAN_SHADER_DIR;
    const std::vector<uint32_t> vertSpv = ReadSpirv(shaderDir + "/triangle.vert.spv");
    const std::vector<uint32_t> fragSpv = ReadSpirv(shaderDir + "/triangle.frag.spv");

    RHIShaderDesc vsDesc{ RHIShaderStage::Vertex,   vertSpv.data(), vertSpv.size() };
    RHIShaderDesc fsDesc{ RHIShaderStage::Fragment, fragSpv.data(), fragSpv.size() };
    auto vertexShader   = device->CreateShader(vsDesc);
    auto fragmentShader = device->CreateShader(fsDesc);

    // Pipeline.
    RHIPipelineDesc pipeDesc;
    pipeDesc.vertexShader   = vertexShader.get();
    pipeDesc.fragmentShader = fragmentShader.get();
    pipeDesc.vertexLayout.stride = sizeof(Vertex);
    pipeDesc.vertexLayout.attributes = {
        { 0, RHIVertexFormat::Float2, offsetof(Vertex, pos)   },
        { 1, RHIVertexFormat::Float3, offsetof(Vertex, color) },
    };
    pipeDesc.resourceBindings = {
        { 0, RHIResourceType::UniformBuffer, RHIShaderStage::Vertex | RHIShaderStage::Fragment },
    };
    pipeDesc.pushConstants = { RHIShaderStage::Vertex, sizeof(PushConstants) };
    pipeDesc.colorFormat   = device->SwapchainFormat();
    auto pipeline = device->CreatePipeline(pipeDesc);

    auto resourceSet = device->CreateResourceSet(
        pipeline.get(), 0, { { 0, uniformBuffer.get() } });

    const std::array<float, 4> clear = { 0.02f, 0.02f, 0.05f, 1.0f };

    while (!glfwWindowShouldClose(window)) {
        glfwPollEvents();

        RHICommandList* cmd = device->BeginFrame(clear);
        if (!cmd) continue;   // frame skipped (minimized / swapchain rebuilt)

        const float t = static_cast<float>(glfwGetTime());

        FrameUBO ubo{};
        ubo.viewProj = glm::mat4(1.0f);
        ubo.time     = t;
        uniformBuffer->Update(&ubo, sizeof(ubo));

        PushConstants push{};
        push.model = glm::rotate(glm::mat4(1.0f), t, glm::vec3(0.0f, 0.0f, 1.0f));

        cmd->BindPipeline(pipeline.get());
        cmd->BindResourceSet(0, resourceSet.get());
        cmd->PushConstants(RHIShaderStage::Vertex, 0, sizeof(push), &push);
        cmd->BindVertexBuffer(vertexBuffer.get());
        cmd->BindIndexBuffer(indexBuffer.get(), RHIIndexType::U16);
        cmd->DrawIndexed(3);

        device->EndFrame();
    }

    device->WaitIdle();   // GPU must finish before the resource unique_ptrs free
    glfwDestroyWindow(window);
    glfwTerminate();
    return 0;
}

} // namespace Diamond
