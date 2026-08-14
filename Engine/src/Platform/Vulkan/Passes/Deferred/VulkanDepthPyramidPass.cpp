#include "Platform/Vulkan/Passes/Deferred/VulkanDepthPyramidPass.h"
#include "Platform/Vulkan/Passes/VulkanPassCommon.h"
#include "Renderer/RHI/RHIDevice.h"
#include "Renderer/RHI/RHICommandList.h"

#include <string>
#include <vector>

namespace Diamond {

namespace {
constexpr uint32_t kLocalSize = 8;   // matches depth_pyramid.comp
} // namespace

VulkanDepthPyramidPass::VulkanDepthPyramidPass(RHIDevice* device, const std::string& shaderDir,
                                               uint32_t width, uint32_t height)
    : m_Device(device), m_Width(width), m_Height(height)
{
    const std::vector<uint32_t> comp = LoadSpirv(shaderDir, "depth_pyramid.comp.spv");
    RHIShaderDesc desc{ RHIShaderStage::Compute, comp.data(), comp.size() };
    m_Comp = device->CreateShader(desc);

    RHIComputePipelineDesc pipe{};
    pipe.computeShader    = m_Comp.get();
    pipe.resourceBindings = {
        { 0, RHIResourceType::StorageImage,         RHIShaderStage::Compute }, // dst level
        { 1, RHIResourceType::CombinedImageSampler, RHIShaderStage::Compute }, // src
    };
    pipe.pushConstants = { RHIShaderStage::Compute, sizeof(Push) };
    m_Pipeline = device->CreateComputePipeline(pipe);
}

VulkanDepthPyramidPass::~VulkanDepthPyramidPass() = default;

void VulkanDepthPyramidPass::AddToGraph(RHIRenderGraph& graph, RGTextureHandle viewPos,
                                        const std::array<RGTextureHandle, kLevels>& levels)
{
    for (int i = 0; i < kLevels; ++i) {
        // Level 0 reads the G-buffer; every level after reads its predecessor.
        const RGTextureHandle src = (i == 0) ? viewPos : levels[i - 1];

        if (!m_Sets[i])
            m_Sets[i] = m_Device->CreateResourceSet(
                m_Pipeline.get(), 0, {},
                { { 0, graph.GetTexture(levels[i]) },
                  { 1, graph.GetTexture(src) } });

        const uint32_t w = LevelSize(m_Width,  i);
        const uint32_t h = LevelSize(m_Height, i);
        const uint32_t groupsX = (w + kLocalSize - 1) / kLocalSize;
        const uint32_t groupsY = (h + kLocalSize - 1) / kLocalSize;

        // Each level depends on the previous one, so the graph serialises them
        // through the ordinary Read/Write edges — no manual barriers.
        graph.AddPass("DepthPyramid" + std::to_string(i)).AsCompute()
            .Read(src)
            .Write(levels[i])
            .SetExecute([this, i, w, h, groupsX, groupsY](RHICommandList* cmd) {
                Push push{};
                push.dstSize = glm::ivec2(static_cast<int>(w), static_cast<int>(h));
                push.mode    = (i == 0) ? 0 : 1;
                cmd->BindPipeline(m_Pipeline.get());
                cmd->BindResourceSet(0, m_Sets[i].get());
                cmd->PushConstants(RHIShaderStage::Compute, 0, sizeof(push), &push);
                cmd->Dispatch(groupsX, groupsY, 1);
            });
    }
}

} // namespace Diamond
