#include "Platform/Vulkan/VulkanImGuiLayer.h"
#include "Platform/Vulkan/RHI/VulkanRHIDevice.h"

#include "imgui.h"
#include "backends/imgui_impl_glfw.h"
#include "backends/imgui_impl_vulkan.h"

#include <GLFW/glfw3.h>
#include <spdlog/spdlog.h>

namespace Diamond {

namespace {
void CheckVkResult(VkResult err) {
    if (err != VK_SUCCESS)
        spdlog::error("[ImGui/Vulkan] VkResult = {}", static_cast<int>(err));
}
} // namespace

void VulkanImGuiLayer::Init(GLFWwindow* window, VulkanRHIDevice* device) {
    m_Device = device;

    IMGUI_CHECKVERSION();
    if (ImGui::GetCurrentContext() == nullptr)
        ImGui::CreateContext();
    ImGui::GetIO().ConfigFlags |= ImGuiConfigFlags_DockingEnable;
    ImGui::StyleColorsDark();

    ImGui_ImplGlfw_InitForVulkan(window, true);

    VulkanContext& ctx = device->Ctx();

    // Dynamic rendering: ImGui builds its pipeline against the swapchain format
    // rather than a VkRenderPass. The format pointer only needs to live through
    // ImGui_ImplVulkan_Init (which creates the pipeline).
    VkFormat colorFormat = device->SwapchainVkFormat();
    VkPipelineRenderingCreateInfo rendering{ VK_STRUCTURE_TYPE_PIPELINE_RENDERING_CREATE_INFO };
    rendering.colorAttachmentCount    = 1;
    rendering.pColorAttachmentFormats = &colorFormat;

    ImGui_ImplVulkan_InitInfo info{};
    info.ApiVersion         = VK_API_VERSION_1_3;
    info.Instance           = ctx.Instance();
    info.PhysicalDevice     = ctx.PhysicalDevice();
    info.Device             = ctx.Device();
    info.QueueFamily        = ctx.Queues().graphics;
    info.Queue              = ctx.GraphicsQueue();
    info.DescriptorPoolSize = 64;   // backend owns its pool (font atlas + AddTexture)
    info.MinImageCount      = VulkanRHIDevice::kFramesInFlight;
    info.ImageCount         = device->SwapchainImageCount();
    info.UseDynamicRendering = true;
    info.PipelineInfoMain.PipelineRenderingCreateInfo = rendering;
    info.CheckVkResultFn    = CheckVkResult;

    if (!ImGui_ImplVulkan_Init(&info)) {
        spdlog::critical("[ImGui/Vulkan] ImGui_ImplVulkan_Init failed");
        ImGui_ImplGlfw_Shutdown();
        return;
    }

    m_Initialized = true;
    spdlog::info("[ImGui/Vulkan] initialized (dynamic rendering, {} swapchain images)",
                 info.ImageCount);
}

void VulkanImGuiLayer::Shutdown() {
    if (!m_Initialized) return;
    // ImGui's Vulkan objects must not be in flight when destroyed.
    vkDeviceWaitIdle(m_Device->Ctx().Device());
    ImGui_ImplVulkan_Shutdown();
    ImGui_ImplGlfw_Shutdown();
    ImGui::DestroyContext();
    m_Initialized = false;
}

void VulkanImGuiLayer::BeginFrame() {
    if (!m_Initialized) return;
    ImGui_ImplVulkan_NewFrame();
    ImGui_ImplGlfw_NewFrame();
    ImGui::NewFrame();
}

void VulkanImGuiLayer::Submit(RHICommandList* cmd) {
    if (!m_Initialized) return;
    auto* vkCmd = static_cast<VulkanRHICommandList*>(cmd);
    ImGui_ImplVulkan_RenderDrawData(ImGui::GetDrawData(), vkCmd->Handle());
}

} // namespace Diamond
