#pragma once

#include "Renderer/RHI/RHIEnums.h"
#include "Renderer/RHI/RHIResources.h"

#include <array>
#include <memory>
#include <vector>

struct GLFWwindow;

namespace Diamond {

class RHICommandList;

// Backend-neutral rendering device: creates GPU resources and drives the frame
// loop. The concrete backend (Vulkan now, OpenGL next) is selected by Create and
// never leaks its native types through this interface — the same discipline the
// engine already applies to its OpenGL classes, extended so a single pass body
// runs unchanged on either backend.
class RHIDevice {
public:
    virtual ~RHIDevice() = default;

    // 'window' must be created with GLFW_NO_API when 'backend' is Vulkan. Returns
    // nullptr if the requested backend isn't compiled into this build.
    static std::unique_ptr<RHIDevice> Create(GLFWwindow* window, RHIBackend backend);

    virtual std::unique_ptr<RHIBuffer>   CreateBuffer(const RHIBufferDesc& desc) = 0;
    virtual std::unique_ptr<RHIShader>   CreateShader(const RHIShaderDesc& desc) = 0;
    virtual std::unique_ptr<RHITexture>  CreateTexture(const RHITextureDesc& desc) = 0;
    virtual std::unique_ptr<RHIPipeline> CreatePipeline(const RHIPipelineDesc& desc) = 0;
    virtual std::unique_ptr<RHIResourceSet> CreateResourceSet(
        RHIPipeline* pipeline, uint32_t setIndex,
        const std::vector<RHIBufferBinding>&  buffers,
        const std::vector<RHITextureBinding>& textures = {}) = 0;

    // Format of the swapchain color target — feed into RHIPipelineDesc::colorFormat
    // so pipelines match the backbuffer they render to.
    virtual RHIFormat SwapchainFormat() const = 0;
    // Format of the device's per-frame depth attachment — feed into
    // RHIPipelineDesc::depthFormat for any pipeline that depth-tests.
    virtual RHIFormat DepthFormat() const = 0;

    // Frame loop. BeginFrame acquires the next backbuffer image, clears it to
    // 'clearColor', and returns a recorder — or nullptr when the frame must be
    // skipped (minimized / swapchain rebuilt), in which case EndFrame must NOT be
    // called. EndFrame finishes recording, submits, and presents.
    virtual RHICommandList* BeginFrame(const std::array<float, 4>& clearColor) = 0;
    virtual void EndFrame() = 0;

    // Call from the window framebuffer-resize callback.
    virtual void NotifyResize() = 0;
    // Block until the GPU is idle (before tearing resources down).
    virtual void WaitIdle() = 0;
};

} // namespace Diamond
