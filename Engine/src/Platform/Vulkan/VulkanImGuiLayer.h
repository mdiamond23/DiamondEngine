#pragma once

struct GLFWwindow;

namespace Diamond {

class VulkanRHIDevice;
class RHICommandList;

// ImGui overlay for the Vulkan backend. Bridges imgui_impl_glfw (input) and
// imgui_impl_vulkan (rendering into the swapchain via dynamic rendering). Like
// the IBL pass, it reaches into VulkanRHIDevice/VulkanContext directly rather
// than through the backend-neutral RHI — ImGui's Vulkan backend needs raw
// handles the RHI deliberately hides.
//
// Usage per frame:
//   layer.BeginFrame();          // then build ImGui widgets
//   ImGui::Render();             // finalize draw data (caller owns this)
//   renderer->RenderToSwapchain(scene, camera,
//       [&](RHICommandList* c) { layer.Submit(c); });
class VulkanImGuiLayer {
public:
    // Creates the ImGui context (if none exists) and initializes both backends.
    // Must be called after the device is up (queues + swapchain resolved).
    void Init(GLFWwindow* window, VulkanRHIDevice* device);
    void Shutdown();

    // Starts a new ImGui frame; call before building any widgets.
    void BeginFrame();

    // Records the current frame's ImGui draw data into the command buffer. Call
    // from a swapchain render-scope overlay (after ImGui::Render()).
    void Submit(RHICommandList* cmd);

    bool IsInitialized() const { return m_Initialized; }

private:
    VulkanRHIDevice* m_Device      = nullptr;
    bool             m_Initialized = false;
};

} // namespace Diamond
