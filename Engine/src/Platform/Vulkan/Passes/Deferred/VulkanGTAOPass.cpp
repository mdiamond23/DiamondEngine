#include "Platform/Vulkan/Passes/Deferred/VulkanGTAOPass.h"
#include "Platform/Vulkan/Passes/VulkanPassCommon.h"
#include "Renderer/RHI/RHIDevice.h"
#include "Renderer/RHI/RHICommandList.h"

#include <algorithm>
#include <vector>

namespace Diamond {

namespace {
constexpr uint32_t kLocalSize = 8;   // matches local_size_x/y in both .comp files
// Where distance attenuation begins, as a fraction of the radius. Occluders
// closer than this count in full; past it they fade to nothing at the radius.
constexpr float kFalloffStart = 0.6f;
} // namespace

VulkanGTAOPass::VulkanGTAOPass(RHIDevice* device, const std::string& shaderDir,
                               uint32_t width, uint32_t height)
    : m_Device(device), m_Width(width), m_Height(height)
{
    const std::vector<uint32_t> gtao    = LoadSpirv(shaderDir, "gtao.comp.spv");
    const std::vector<uint32_t> denoise = LoadSpirv(shaderDir, "gtao_denoise.comp.spv");

    RHIShaderDesc gtaoDesc{ RHIShaderStage::Compute, gtao.data(), gtao.size() };
    RHIShaderDesc dnDesc  { RHIShaderStage::Compute, denoise.data(), denoise.size() };
    m_GTAOComp    = device->CreateShader(gtaoDesc);
    m_DenoiseComp = device->CreateShader(dnDesc);

    {
        RHIComputePipelineDesc desc{};
        desc.computeShader    = m_GTAOComp.get();
        desc.resourceBindings = {
            { 0, RHIResourceType::StorageImage,         RHIShaderStage::Compute }, // aoRaw
            { 1, RHIResourceType::CombinedImageSampler, RHIShaderStage::Compute }, // gViewPos
            { 2, RHIResourceType::CombinedImageSampler, RHIShaderStage::Compute }, // gViewNormal
        };
        desc.pushConstants = { RHIShaderStage::Compute, sizeof(GTAOPush) };
        m_GTAOPipeline = device->CreateComputePipeline(desc);
    }
    {
        // Dimensions come from textureSize, so this one needs no push constants.
        RHIComputePipelineDesc desc{};
        desc.computeShader    = m_DenoiseComp.get();
        desc.resourceBindings = {
            { 0, RHIResourceType::StorageImage,         RHIShaderStage::Compute }, // aoBlurred
            { 1, RHIResourceType::CombinedImageSampler, RHIShaderStage::Compute }, // aoRaw
            { 2, RHIResourceType::CombinedImageSampler, RHIShaderStage::Compute }, // gViewPos
        };
        m_DenoisePipeline = device->CreateComputePipeline(desc);
    }
}

VulkanGTAOPass::~VulkanGTAOPass() = default;

void VulkanGTAOPass::AddToGraph(RHIRenderGraph& graph,
                                RGTextureHandle viewPos, RGTextureHandle viewNormal,
                                RGTextureHandle aoRaw, RGTextureHandle aoBlurred)
{
    // Sets are built once — graph textures keep their identity across frames.
    // aoRaw binds as a storage image in the pass that writes it and as a
    // sampler in the pass that reads it, never both in one set (that would
    // demand two image layouts at once).
    if (!m_GTAOSet)
        m_GTAOSet = m_Device->CreateResourceSet(
            m_GTAOPipeline.get(), 0, {},
            { { 0, graph.GetTexture(aoRaw) },
              { 1, graph.GetTexture(viewPos) },
              { 2, graph.GetTexture(viewNormal) } });

    if (!m_DenoiseSet)
        m_DenoiseSet = m_Device->CreateResourceSet(
            m_DenoisePipeline.get(), 0, {},
            { { 0, graph.GetTexture(aoBlurred) },
              { 1, graph.GetTexture(aoRaw) },
              { 2, graph.GetTexture(viewPos) } });

    const uint32_t groupsX = (m_Width  + kLocalSize - 1) / kLocalSize;
    const uint32_t groupsY = (m_Height + kLocalSize - 1) / kLocalSize;

    graph.AddPass("GTAO").AsCompute()
        .Read(viewPos).Read(viewNormal)
        .Write(aoRaw)
        .SetExecute([this, groupsX, groupsY](RHICommandList* cmd) {
            GTAOPush push{};
            push.screenSize   = glm::vec2(static_cast<float>(m_Width),
                                          static_cast<float>(m_Height));
            push.projScale    = m_ProjScale;
            push.radius       = m_Radius;
            push.falloffStart = kFalloffStart;
            push.slices       = m_Slices;
            push.steps        = m_Steps;
            push.frameIndex   = m_FrameIndex++;
            push.bentNormals  = m_BentNormals ? 1.0f : 0.0f;
            cmd->BindPipeline(m_GTAOPipeline.get());
            cmd->BindResourceSet(0, m_GTAOSet.get());
            cmd->PushConstants(RHIShaderStage::Compute, 0, sizeof(push), &push);
            cmd->Dispatch(groupsX, groupsY, 1);
        });

    graph.AddPass("GTAODenoise").AsCompute()
        .Read(aoRaw).Read(viewPos)
        .Write(aoBlurred)
        .SetExecute([this, groupsX, groupsY](RHICommandList* cmd) {
            cmd->BindPipeline(m_DenoisePipeline.get());
            cmd->BindResourceSet(0, m_DenoiseSet.get());
            cmd->Dispatch(groupsX, groupsY, 1);
        });
}

void VulkanGTAOPass::SetProjection(const glm::mat4& projection)
{
    // proj[1][1] maps view-space y at z = -1 to NDC; half the viewport height
    // converts that to pixels. The projection is right-handed with a negative
    // viewport height here, so take the magnitude.
    m_ProjScale = std::abs(projection[1][1]) * 0.5f * static_cast<float>(m_Height);
}

void VulkanGTAOPass::SetParams(float radius, int slices, int steps, bool bentNormals)
{
    m_Radius      = std::max(radius, 0.01f);
    m_Slices      = std::clamp(slices, 1, 8);
    m_Steps       = std::clamp(steps,  1, 16);
    m_BentNormals = bentNormals;
}

} // namespace Diamond
