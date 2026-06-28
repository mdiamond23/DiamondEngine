#include "Platform/VulkanDemo.h"
#include "Platform/Vulkan/VulkanRenderer.h"

#define GLFW_INCLUDE_NONE
#include <GLFW/glfw3.h>
#include <spdlog/spdlog.h>

#include <array>
#include <cmath>

namespace Diamond {

namespace {
void OnFramebufferResize(GLFWwindow* window, int /*w*/, int /*h*/) {
    auto* renderer = static_cast<VulkanRenderer*>(glfwGetWindowUserPointer(window));
    if (renderer) renderer->NotifyResize();
}
} // namespace

int RunVulkanClearDemo() {
    if (!glfwInit()) {
        spdlog::critical("[Vulkan] glfwInit failed");
        return 1;
    }
    if (!glfwVulkanSupported()) {
        spdlog::critical("[Vulkan] GLFW reports no Vulkan loader present");
        glfwTerminate();
        return 1;
    }

    // No OpenGL context — this is a Vulkan window.
    glfwWindowHint(GLFW_CLIENT_API, GLFW_NO_API);
    GLFWwindow* window = glfwCreateWindow(1280, 720, "DiamondEngine — Vulkan M0", nullptr, nullptr);
    if (!window) {
        spdlog::critical("[Vulkan] glfwCreateWindow failed");
        glfwTerminate();
        return 1;
    }

    VulkanRenderer renderer;
    renderer.Init(window);

    glfwSetWindowUserPointer(window, &renderer);
    glfwSetFramebufferSizeCallback(window, OnFramebufferResize);

    while (!glfwWindowShouldClose(window)) {
        glfwPollEvents();

        // Slow hue pulse so it's obvious the loop is live, not a static clear.
        const float t = static_cast<float>(glfwGetTime());
        std::array<float, 4> clear = {
            0.5f + 0.5f * std::sin(t),
            0.5f + 0.5f * std::sin(t + 2.094f),  // +120°
            0.5f + 0.5f * std::sin(t + 4.188f),  // +240°
            1.0f,
        };
        renderer.DrawFrame(clear);
    }

    renderer.Shutdown();
    glfwDestroyWindow(window);
    glfwTerminate();
    return 0;
}

} // namespace Diamond
