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
    : m_Device(device), m_Width(width), m_Height(height),
      m_HalfW(std::max(1u, width  / 2)), m_HalfH(std::max(1u, height / 2))
{
    const std::vector<uint32_t> gtao     = LoadSpirv(shaderDir, "gtao.comp.spv");
    const std::vector<uint32_t> denoise  = LoadSpirv(shaderDir, "gtao_denoise.comp.spv");
    const std::vector<uint32_t> upsample = LoadSpirv(shaderDir, "gtao_upsample.comp.spv");

    RHIShaderDesc gtaoDesc{ RHIShaderStage::Compute, gtao.data(), gtao.size() };
    RHIShaderDesc dnDesc  { RHIShaderStage::Compute, denoise.data(), denoise.size() };
    RHIShaderDesc upDesc  { RHIShaderStage::Compute, upsample.data(), upsample.size() };
    m_GTAOComp     = device->CreateShader(gtaoDesc);
    m_DenoiseComp  = device->CreateShader(dnDesc);
    m_UpsampleComp = device->CreateShader(upDesc);

    {
        RHIComputePipelineDesc desc{};
        desc.computeShader    = m_GTAOComp.get();
        desc.resourceBindings = {
            { 0, RHIResourceType::StorageImage,         RHIShaderStage::Compute }, // aoRaw
            { 1, RHIResourceType::CombinedImageSampler, RHIShaderStage::Compute }, // gViewNormal
            { 2, RHIResourceType::CombinedImageSampler, RHIShaderStage::Compute }, // depth L0
            { 3, RHIResourceType::CombinedImageSampler, RHIShaderStage::Compute }, // depth L1
            { 4, RHIResourceType::CombinedImageSampler, RHIShaderStage::Compute }, // depth L2
            { 5, RHIResourceType::CombinedImageSampler, RHIShaderStage::Compute }, // depth L3
        };
        desc.pushConstants = { RHIShaderStage::Compute, sizeof(GTAOPush) };
        m_GTAOPipeline = device->CreateComputePipeline(desc);
    }
    {
        // Dimensions come from textureSize, so this one needs no push constants.
        RHIComputePipelineDesc desc{};
        desc.computeShader    = m_DenoiseComp.get();
        desc.resourceBindings = {
            { 0, RHIResourceType::StorageImage,         RHIShaderStage::Compute }, // aoDenoised
            { 1, RHIResourceType::CombinedImageSampler, RHIShaderStage::Compute }, // aoRaw
            { 2, RHIResourceType::CombinedImageSampler, RHIShaderStage::Compute }, // gViewPos
        };
        m_DenoisePipeline = device->CreateComputePipeline(desc);
    }
    {
        // Bilateral upsample, half res -> full res. Dimensions come from
        // textureSize/imageSize too.
        RHIComputePipelineDesc desc{};
        desc.computeShader    = m_UpsampleComp.get();
        desc.resourceBindings = {
            { 0, RHIResourceType::StorageImage,         RHIShaderStage::Compute }, // aoBlurred
            { 1, RHIResourceType::CombinedImageSampler, RHIShaderStage::Compute }, // aoDenoised
            { 2, RHIResourceType::CombinedImageSampler, RHIShaderStage::Compute }, // gViewPos
            { 3, RHIResourceType::CombinedImageSampler, RHIShaderStage::Compute }, // gViewNormal
        };
        m_UpsamplePipeline = device->CreateComputePipeline(desc);
    }
}

VulkanGTAOPass::~VulkanGTAOPass() = default;

void VulkanGTAOPass::AddToGraph(RHIRenderGraph& graph,
                                RGTextureHandle viewPos, RGTextureHandle viewNormal,
                                const std::array<RGTextureHandle, 4>& depthLevels,
                                RGTextureHandle aoRaw, RGTextureHandle aoDenoised,
                                RGTextureHandle aoBlurred)
{
    // Sets are built once — graph textures keep their identity across frames.
    // aoRaw/aoDenoised each bind as a storage image in the pass that writes
    // them and as a sampler in the pass that reads them, never both in one set
    // (that would demand two image layouts at once).
    if (!m_GTAOSet)
        m_GTAOSet = m_Device->CreateResourceSet(
            m_GTAOPipeline.get(), 0, {},
            { { 0, graph.GetTexture(aoRaw) },
              { 1, graph.GetTexture(viewNormal) },
              { 2, graph.GetTexture(depthLevels[0]) },
              { 3, graph.GetTexture(depthLevels[1]) },
              { 4, graph.GetTexture(depthLevels[2]) },
              { 5, graph.GetTexture(depthLevels[3]) } });

    if (!m_DenoiseSet)
        m_DenoiseSet = m_Device->CreateResourceSet(
            m_DenoisePipeline.get(), 0, {},
            { { 0, graph.GetTexture(aoDenoised) },
              { 1, graph.GetTexture(aoRaw) },
              { 2, graph.GetTexture(viewPos) } });

    if (!m_UpsampleSet)
        m_UpsampleSet = m_Device->CreateResourceSet(
            m_UpsamplePipeline.get(), 0, {},
            { { 0, graph.GetTexture(aoBlurred) },
              { 1, graph.GetTexture(aoDenoised) },
              { 2, graph.GetTexture(viewPos) },
              { 3, graph.GetTexture(viewNormal) } });

    const uint32_t halfGroupsX = (m_HalfW + kLocalSize - 1) / kLocalSize;
    const uint32_t halfGroupsY = (m_HalfH + kLocalSize - 1) / kLocalSize;
    const uint32_t fullGroupsX = (m_Width  + kLocalSize - 1) / kLocalSize;
    const uint32_t fullGroupsY = (m_Height + kLocalSize - 1) / kLocalSize;

    graph.AddPass("GTAO").AsCompute()
        .Read(viewNormal)
        .Read(depthLevels[0]).Read(depthLevels[1])
        .Read(depthLevels[2]).Read(depthLevels[3])
        .Write(aoRaw)
        .SetExecute([this, halfGroupsX, halfGroupsY](RHICommandList* cmd) {
            GTAOPush push{};
            push.screenSize   = glm::vec2(static_cast<float>(m_HalfW),
                                          static_cast<float>(m_HalfH));
            push.invProj      = m_InvProj;
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
            cmd->Dispatch(halfGroupsX, halfGroupsY, 1);
        });

    graph.AddPass("GTAODenoise").AsCompute()
        .Read(aoRaw).Read(viewPos)
        .Write(aoDenoised)
        .SetExecute([this, halfGroupsX, halfGroupsY](RHICommandList* cmd) {
            cmd->BindPipeline(m_DenoisePipeline.get());
            cmd->BindResourceSet(0, m_DenoiseSet.get());
            cmd->Dispatch(halfGroupsX, halfGroupsY, 1);
        });

    graph.AddPass("GTAOUpsample").AsCompute()
        .Read(aoDenoised).Read(viewPos).Read(viewNormal)
        .Write(aoBlurred)
        .SetExecute([this, fullGroupsX, fullGroupsY](RHICommandList* cmd) {
            cmd->BindPipeline(m_UpsamplePipeline.get());
            cmd->BindResourceSet(0, m_UpsampleSet.get());
            cmd->Dispatch(fullGroupsX, fullGroupsY, 1);
        });
}

void VulkanGTAOPass::SetProjection(const glm::mat4& projection)
{
    // proj[1][1] maps view-space y at z = -1 to NDC; half the viewport height
    // converts that to pixels. The projection is right-handed with a negative
    // viewport height here, so take the magnitude. Uses the HALF-res height —
    // the horizon march dispatches at half res, so "pixels" here means pixels
    // of that grid, not the full-res G-buffer.
    m_ProjScale = std::abs(projection[1][1]) * 0.5f * static_cast<float>(m_HalfH);
    // Reciprocals of the same scales: ndc * depth * invProj recovers view x/y
    // exactly for a standard perspective matrix (clip.w == -viewZ).
    m_InvProj = glm::vec2(1.0f / projection[0][0], 1.0f / projection[1][1]);
}

void VulkanGTAOPass::SetParams(float radius, int slices, int steps, bool bentNormals)
{
    m_Radius      = std::max(radius, 0.01f);
    m_Slices      = std::clamp(slices, 1, 8);
    m_Steps       = std::clamp(steps,  1, 16);
    m_BentNormals = bentNormals;
}

} // namespace Diamond
