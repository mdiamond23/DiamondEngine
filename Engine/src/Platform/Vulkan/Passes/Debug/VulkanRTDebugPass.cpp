#include "Platform/Vulkan/Passes/Debug/VulkanRTDebugPass.h"
#include "Platform/Vulkan/Passes/VulkanPassCommon.h"
#include "Renderer/RHI/RHIDevice.h"
#include "Renderer/RHI/RHICommandList.h"

#include <glm/gtc/matrix_inverse.hpp>

#include <algorithm>
#include <vector>

namespace Diamond {

namespace {
constexpr RHIFormat kDebugFormat = RHIFormat::RGBA16F;
} // namespace

VulkanRTDebugPass::VulkanRTDebugPass(RHIDevice* device, const std::string& shaderDir,
                                     uint32_t width, uint32_t height, RHIFormat targetFormat)
    : m_Device(device), m_Width(width), m_Height(height)
{
    if (!device->SupportsRayTracing()) return;   // stays inert; Available() false

    RHIBufferDesc uboDesc;
    uboDesc.size    = sizeof(CameraUBO);
    uboDesc.usage   = RHIBufferUsage::Uniform;
    uboDesc.dynamic = true;
    m_CameraUBO = device->CreateBuffer(uboDesc);

    const std::vector<uint32_t> rgen = LoadSpirv(shaderDir, "rt_debug.rgen.spv");
    const std::vector<uint32_t> miss = LoadSpirv(shaderDir, "rt_debug.rmiss.spv");
    const std::vector<uint32_t> chit = LoadSpirv(shaderDir, "rt_debug.rchit.spv");
    const std::vector<uint32_t> vert = LoadSpirv(shaderDir, "fullscreen.vert.spv");
    const std::vector<uint32_t> copy = LoadSpirv(shaderDir, "copy.frag.spv");

    RHIShaderDesc rgenDesc{ RHIShaderStage::RayGen,     rgen.data(), rgen.size() };
    RHIShaderDesc missDesc{ RHIShaderStage::Miss,       miss.data(), miss.size() };
    RHIShaderDesc chitDesc{ RHIShaderStage::ClosestHit, chit.data(), chit.size() };
    RHIShaderDesc vertDesc{ RHIShaderStage::Vertex,     vert.data(), vert.size() };
    RHIShaderDesc copyDesc{ RHIShaderStage::Fragment,   copy.data(), copy.size() };

    m_Raygen         = device->CreateShader(rgenDesc);
    m_Miss           = device->CreateShader(missDesc);
    m_ClosestHit     = device->CreateShader(chitDesc);
    m_FullscreenVert = device->CreateShader(vertDesc);
    m_CopyFrag       = device->CreateShader(copyDesc);

    // ── Trace (ray tracing) ──────────────────────────────────────────────────
    // Everything is read from the raygen; miss and closest-hit touch nothing but
    // their payload, so no binding needs their stages.
    {
        RHIRayTracingPipelineDesc desc{};
        desc.raygenShader     = m_Raygen.get();
        desc.missShader       = m_Miss.get();
        desc.closestHitShader = m_ClosestHit.get();
        desc.resourceBindings = {
            { 0, RHIResourceType::StorageImage,          RHIShaderStage::RayGen }, // rtDebug
            { 1, RHIResourceType::AccelerationStructure, RHIShaderStage::RayGen }, // TLAS
            { 2, RHIResourceType::UniformBuffer,         RHIShaderStage::RayGen }, // camera
        };
        desc.pushConstants     = { RHIShaderStage::RayGen, sizeof(TracePush) };
        desc.maxRecursionDepth = 1;   // primary rays only
        m_Pipeline = device->CreateRayTracingPipeline(desc);
    }

    // ── Blit (raster) ────────────────────────────────────────────────────────
    if (m_Pipeline) {
        RHIPipelineDesc desc;
        desc.vertexShader     = m_FullscreenVert.get();
        desc.fragmentShader   = m_CopyFrag.get();
        desc.resourceBindings = {
            { 0, RHIResourceType::CombinedImageSampler, RHIShaderStage::Fragment },
        };
        desc.colorFormat = targetFormat;
        m_BlitPipeline = device->CreatePipeline(desc);
    }
}

VulkanRTDebugPass::~VulkanRTDebugPass() = default;

void VulkanRTDebugPass::AddToGraph(RHIRenderGraph& graph, RGTextureHandle rtDebug,
                                   RGTextureHandle target, bool toSwapchain)
{
    if (!Available()) return;

    m_RTDebugTexture = graph.GetTexture(rtDebug);

    if (!m_BlitSet)
        m_BlitSet = m_Device->CreateResourceSet(
            m_BlitPipeline.get(), 0, {}, { { 0, m_RTDebugTexture } });

    graph.AddPass("RTDebugTrace").AsCompute()
        .Write(rtDebug)
        .SetExecute([this](RHICommandList* cmd) {
            if (!m_Enabled || !m_TLAS) return;

            // Deferred until execute: the TLAS may not exist when the graph is
            // built, and it can be replaced (not just rebuilt) later.
            if (m_SetDirty) {
                m_Device->WaitIdle();   // the old set may still be in flight
                m_TraceSet = m_Device->CreateResourceSet(
                    m_Pipeline.get(), 0,
                    { { 2, m_CameraUBO.get() } },
                    { { 0, m_RTDebugTexture } },
                    { { 1, m_TLAS } });
                m_SetDirty = false;
            }
            if (!m_TraceSet) return;

            TracePush push{ m_MaxDistance };
            cmd->BindPipeline(m_Pipeline.get());
            cmd->BindResourceSet(0, m_TraceSet.get());
            cmd->PushConstants(RHIShaderStage::RayGen, 0, sizeof(push), &push);
            cmd->TraceRays(m_Width, m_Height);
        });

    // Loads rather than clears, and skips its draw when disabled — so the
    // tonemapped frame underneath survives untouched.
    RGPass& blit = graph.AddPass("RTDebugBlit");
    blit.Read(rtDebug);
    if (toSwapchain) blit.WriteSwapchain();
    else             blit.Write(target);
    blit.Load().SetExecute([this](RHICommandList* cmd) {
        if (!m_Enabled || !m_TLAS) return;
        cmd->BindPipeline(m_BlitPipeline.get());
        cmd->BindResourceSet(0, m_BlitSet.get());
        cmd->Draw(3);
    });
}

void VulkanRTDebugPass::SetTLAS(RHIAccelStruct* tlas) {
    if (m_TLAS == tlas) return;
    m_TLAS     = tlas;
    m_SetDirty = true;
}

void VulkanRTDebugPass::SetCamera(const glm::mat4& view, const glm::mat4& projection) {
    if (!Available()) return;
    CameraUBO ubo{};
    ubo.invView = glm::inverse(view);
    ubo.invProj = glm::inverse(projection);
    m_CameraUBO->Update(&ubo, sizeof(ubo));
}

void VulkanRTDebugPass::SetMaxDistance(float distance) {
    m_MaxDistance = std::max(0.1f, distance);
}

} // namespace Diamond
