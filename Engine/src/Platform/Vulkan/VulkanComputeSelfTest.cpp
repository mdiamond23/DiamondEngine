#include "Platform/Vulkan/VulkanComputeSelfTest.h"

#include "Platform/Vulkan/RHI/VulkanRHIDevice.h"
#include "Platform/Vulkan/RHI/VulkanRHIResources.h"
#include "Platform/Vulkan/Passes/VulkanPassCommon.h"
#include "Renderer/RHI/RHIRenderGraph.h"

#include <spdlog/spdlog.h>

#include <cmath>
#include <cstdint>
#include <functional>
#include <memory>
#include <vector>

namespace Diamond {

namespace {

constexpr uint32_t kSize      = 64;   // square, a multiple of the 8×8 local size
constexpr uint32_t kLocalSize = 8;

// Matches compute_selftest.comp's push-constant block.
struct Push {
    uint32_t width  = kSize;
    uint32_t height = kSize;
    uint32_t pass   = 0;
};

// The graph self-test's reduce shader takes just the extent.
struct SizePush {
    uint32_t width  = kSize;
    uint32_t height = kSize;
};

// The copy shader additionally scales what it stores, so each execute leaves a
// distinguishable value in the persistent target (see graph_selftest_copy.comp).
struct CopyPush {
    uint32_t width  = kSize;
    uint32_t height = kSize;
    float    scale  = 1.0f;
};

// Value the gradient shader writes at (x, y), summed over rgb — the quantity
// both self-tests check the GPU against.
float ExpectedGradientSum(uint32_t x, uint32_t y) {
    return float(x) / float(kSize) + float(y) / float(kSize) + 0.25f;
}

// Compares 'results' against 'expected(x, y)' and logs one PASS/FAIL line.
// Half-float storage plus the divide gives a few ulp of slack per channel.
bool CheckTexels(const char* label, const float* results,
                 const std::function<float(uint32_t, uint32_t)>& expected) {
    constexpr float kTolerance = 0.01f;
    uint32_t mismatches = 0;
    float    worst      = 0.0f;
    for (uint32_t y = 0; y < kSize; ++y) {
        for (uint32_t x = 0; x < kSize; ++x) {
            const float error = std::fabs(results[y * kSize + x] - expected(x, y));
            if (error > worst) worst = error;
            if (error > kTolerance) ++mismatches;
        }
    }

    if (mismatches == 0) {
        spdlog::info("[Vulkan] {}: PASS ({} texels, max error {:.5f})",
                     label, kSize * kSize, worst);
        return true;
    }
    spdlog::error("[Vulkan] {}: FAIL — {}/{} texels wrong (max error {:.5f})",
                  label, mismatches, kSize * kSize, worst);
    return false;
}

} // namespace

void RunComputeSelfTest(VulkanRHIDevice& device, const char* shaderDir) {
    spdlog::info("[Vulkan] compute self-test: starting");

    const std::vector<uint32_t> spirv = LoadSpirv(shaderDir, "compute_selftest.comp.spv");
    RHIShaderDesc shaderDesc{};
    shaderDesc.stage     = RHIShaderStage::Compute;
    shaderDesc.spirv     = spirv.data();
    shaderDesc.wordCount = spirv.size();
    std::unique_ptr<RHIShader> shader = device.CreateShader(shaderDesc);

    RHIComputePipelineDesc pipelineDesc{};
    pipelineDesc.computeShader    = shader.get();
    pipelineDesc.resourceBindings = {
        { 0, RHIResourceType::StorageImage,  RHIShaderStage::Compute },
        { 1, RHIResourceType::StorageBuffer, RHIShaderStage::Compute },
    };
    pipelineDesc.pushConstants = { RHIShaderStage::Compute, sizeof(Push) };
    std::unique_ptr<RHIPipeline> pipeline = device.CreateComputePipeline(pipelineDesc);

    RHITextureDesc imageDesc{};
    imageDesc.width     = kSize;
    imageDesc.height    = kSize;
    imageDesc.format    = RHIFormat::RGBA16F;   // the format GI targets will use
    imageDesc.usage     = RHITextureUsage::Storage;
    imageDesc.debugName = "ComputeSelfTestImage";
    std::unique_ptr<RHITexture> image = device.CreateTexture(imageDesc);

    // Dynamic = host-visible and persistently mapped, so the results can be read
    // straight out of the mapping instead of staging a copy back.
    RHIBufferDesc bufferDesc{};
    bufferDesc.size    = uint64_t(kSize) * kSize * sizeof(float);
    bufferDesc.usage   = RHIBufferUsage::Storage;
    bufferDesc.dynamic = true;
    std::unique_ptr<RHIBuffer> buffer = device.CreateBuffer(bufferDesc);

    std::unique_ptr<RHIResourceSet> set = device.CreateResourceSet(
        pipeline.get(), 0, { { 1, buffer.get() } }, { { 0, image.get() } });

    // No frame is in flight during device construction, so the work rides the
    // context's one-shot immediate submit — with the ordinary RHI recorder
    // pointed at that command buffer, so this exercises the real API surface.
    device.Ctx().ImmediateSubmit([&](VkCommandBuffer cmd) {
        VulkanRHICommandList list(&device);
        list.Reset(cmd);

        list.TransitionTexture(image.get(), RHITextureState::Storage);
        list.BindPipeline(pipeline.get());
        list.BindResourceSet(0, set.get());

        const uint32_t groups = kSize / kLocalSize;

        Push push{};
        push.pass = 0;   // write the gradient
        list.PushConstants(RHIShaderStage::Compute, 0, sizeof(push), &push);
        list.Dispatch(groups, groups, 1);

        list.StorageBarrier();

        push.pass = 1;   // read it back into the buffer
        list.PushConstants(RHIShaderStage::Compute, 0, sizeof(push), &push);
        list.Dispatch(groups, groups, 1);

        // StorageBarrier only reaches shader stages; the CPU read needs the host
        // stage, which nothing in the RHI expresses (nor should it — this is the
        // one place that reads GPU results back).
        VkMemoryBarrier2 hostBarrier{ VK_STRUCTURE_TYPE_MEMORY_BARRIER_2 };
        hostBarrier.srcStageMask  = VK_PIPELINE_STAGE_2_COMPUTE_SHADER_BIT;
        hostBarrier.srcAccessMask = VK_ACCESS_2_SHADER_STORAGE_WRITE_BIT;
        hostBarrier.dstStageMask  = VK_PIPELINE_STAGE_2_HOST_BIT;
        hostBarrier.dstAccessMask = VK_ACCESS_2_HOST_READ_BIT;

        VkDependencyInfo dep{ VK_STRUCTURE_TYPE_DEPENDENCY_INFO };
        dep.memoryBarrierCount = 1;
        dep.pMemoryBarriers    = &hostBarrier;
        vkCmdPipelineBarrier2(cmd, &dep);
    });

    // ImmediateSubmit blocks until the work retires, so the mapping is readable.
    const auto* results = static_cast<const float*>(
        static_cast<VulkanRHIBuffer*>(buffer.get())->Mapped(device.CurrentFrame()));

    CheckTexels("compute self-test", results, ExpectedGradientSum);

    device.WaitIdle();   // resources are destroyed as this scope unwinds
}

void RunGraphSelfTest(VulkanRHIDevice& device, const char* shaderDir) {
    spdlog::info("[Vulkan] graph self-test: starting");

    auto makeShader = [&](const char* name, std::vector<uint32_t>& storage) {
        storage = LoadSpirv(shaderDir, name);
        RHIShaderDesc desc{};
        desc.stage     = RHIShaderStage::Compute;
        desc.spirv     = storage.data();
        desc.wordCount = storage.size();
        return device.CreateShader(desc);
    };

    std::vector<uint32_t> gradientSpirv, reduceSpirv, copySpirv;
    std::unique_ptr<RHIShader> gradientShader =
        makeShader("compute_selftest.comp.spv", gradientSpirv);
    std::unique_ptr<RHIShader> reduceShader =
        makeShader("graph_selftest_reduce.comp.spv", reduceSpirv);
    std::unique_ptr<RHIShader> copyShader =
        makeShader("graph_selftest_copy.comp.spv", copySpirv);

    // Reuses the flat self-test's gradient shader — mode 0 writes the image and
    // leaves its buffer binding alone.
    RHIComputePipelineDesc gradientDesc{};
    gradientDesc.computeShader    = gradientShader.get();
    gradientDesc.resourceBindings = {
        { 0, RHIResourceType::StorageImage,  RHIShaderStage::Compute },
        { 1, RHIResourceType::StorageBuffer, RHIShaderStage::Compute },
    };
    gradientDesc.pushConstants = { RHIShaderStage::Compute, sizeof(Push) };
    std::unique_ptr<RHIPipeline> gradientPipeline = device.CreateComputePipeline(gradientDesc);

    RHIComputePipelineDesc reduceDesc{};
    reduceDesc.computeShader    = reduceShader.get();
    reduceDesc.resourceBindings = {
        { 0, RHIResourceType::CombinedImageSampler, RHIShaderStage::Compute },
        { 1, RHIResourceType::CombinedImageSampler, RHIShaderStage::Compute },
        { 2, RHIResourceType::StorageBuffer,        RHIShaderStage::Compute },
    };
    reduceDesc.pushConstants = { RHIShaderStage::Compute, sizeof(SizePush) };
    std::unique_ptr<RHIPipeline> reducePipeline = device.CreateComputePipeline(reduceDesc);

    RHIComputePipelineDesc copyDesc{};
    copyDesc.computeShader    = copyShader.get();
    copyDesc.resourceBindings = {
        { 0, RHIResourceType::StorageImage,         RHIShaderStage::Compute },
        { 1, RHIResourceType::CombinedImageSampler, RHIShaderStage::Compute },
    };
    copyDesc.pushConstants = { RHIShaderStage::Compute, sizeof(CopyPush) };
    std::unique_ptr<RHIPipeline> copyPipeline = device.CreateComputePipeline(copyDesc);

    RHIRenderGraph graph(&device);

    // gradient: compute-written, then sampled. accum: the same, plus persistent —
    // its whole job is to carry a value from one execute to the next.
    const RGTextureHandle gradient = graph.DeclareTexture(
        "selfTestGradient", { kSize, kSize, RHIFormat::RGBA16F, /*storage*/ true });
    const RGTextureHandle accum = graph.DeclareTexture(
        "selfTestAccum",
        { kSize, kSize, RHIFormat::RGBA16F, /*storage*/ true, /*persistent*/ true });
    // Dynamic so the results can be read straight out of the host mapping.
    const RGBufferHandle sums = graph.DeclareBuffer(
        "selfTestSums", { uint64_t(kSize) * kSize * sizeof(float), /*dynamic*/ true });

    // Each set holds distinct textures — an image bound as both a storage image
    // and a sampler in one set would need two layouts at once.
    std::unique_ptr<RHIResourceSet> gradientSet = device.CreateResourceSet(
        gradientPipeline.get(), 0, { { 1, graph.GetBuffer(sums) } },
        { { 0, graph.GetTexture(gradient) } });
    std::unique_ptr<RHIResourceSet> reduceSet = device.CreateResourceSet(
        reducePipeline.get(), 0, { { 2, graph.GetBuffer(sums) } },
        { { 0, graph.GetTexture(gradient) }, { 1, graph.GetTexture(accum) } });
    std::unique_ptr<RHIResourceSet> copySet = device.CreateResourceSet(
        copyPipeline.get(), 0, {},
        { { 0, graph.GetTexture(accum) }, { 1, graph.GetTexture(gradient) } });

    constexpr uint32_t kGroups = kSize / kLocalSize;

    graph.AddPass("SelfTestGradient").AsCompute()
        .Write(gradient)
        .SetExecute([&](RHICommandList* cmd) {
            Push push{};
            push.pass = 0;   // write the gradient
            cmd->BindPipeline(gradientPipeline.get());
            cmd->BindResourceSet(0, gradientSet.get());
            cmd->PushConstants(RHIShaderStage::Compute, 0, sizeof(push), &push);
            cmd->Dispatch(kGroups, kGroups, 1);
        });

    // Declared BEFORE the reduce on purpose. Nothing in the graph reads accum
    // with a real edge, so this pass survives culling only because its target is
    // persistent — and only the history read's write-after-read edge stops it
    // from running first and clobbering the value the reduce wants. Both are
    // load-bearing here rather than incidentally satisfied by declaration order.
    float copyScale = 1.0f;   // varied per execute by runOnce below
    graph.AddPass("SelfTestCopy").AsCompute()
        .Read(gradient)
        .Write(accum)
        .SetExecute([&](RHICommandList* cmd) {
            CopyPush push{};
            push.scale = copyScale;
            cmd->BindPipeline(copyPipeline.get());
            cmd->BindResourceSet(0, copySet.get());
            cmd->PushConstants(RHIShaderStage::Compute, 0, sizeof(push), &push);
            cmd->Dispatch(kGroups, kGroups, 1);
        });

    // ReadHistory, not Read: accum's value here is the PREVIOUS execute's, so
    // depending on this execute's copy pass would be a cycle.
    graph.AddPass("SelfTestReduce").AsCompute()
        .Read(gradient)
        .ReadHistory(accum)
        .WriteBuffer(sums)
        .SetExecute([&](RHICommandList* cmd) {
            SizePush push{};
            cmd->BindPipeline(reducePipeline.get());
            cmd->BindResourceSet(0, reduceSet.get());
            cmd->PushConstants(RHIShaderStage::Compute, 0, sizeof(push), &push);
            cmd->Dispatch(kGroups, kGroups, 1);
        });

    graph.Compile();

    // Run twice, as two separate submissions — the analogue of two frames. Run 1
    // reduces against an accum that has never been written (garbage, ignored);
    // run 2 reduces against what run 1's copy pass left there.
    auto runOnce = [&](float scale) {
        copyScale = scale;
        device.Ctx().ImmediateSubmit([&](VkCommandBuffer cmd) {
            VulkanRHICommandList list(&device);
            list.Reset(cmd);
            graph.Execute(&list);

            VkMemoryBarrier2 hostBarrier{ VK_STRUCTURE_TYPE_MEMORY_BARRIER_2 };
            hostBarrier.srcStageMask  = VK_PIPELINE_STAGE_2_COMPUTE_SHADER_BIT;
            hostBarrier.srcAccessMask = VK_ACCESS_2_SHADER_STORAGE_WRITE_BIT;
            hostBarrier.dstStageMask  = VK_PIPELINE_STAGE_2_HOST_BIT;
            hostBarrier.dstAccessMask = VK_ACCESS_2_HOST_READ_BIT;

            VkDependencyInfo dep{ VK_STRUCTURE_TYPE_DEPENDENCY_INFO };
            dep.memoryBarrierCount = 1;
            dep.pMemoryBarriers    = &hostBarrier;
            vkCmdPipelineBarrier2(cmd, &dep);
        });
    };

    // The differing scales are what make the second run's answer diagnostic: it
    // must reduce against run 1's accum (×1), not the one run 2's own copy pass
    // is about to write (×3).
    runOnce(1.0f);
    runOnce(3.0f);

    const auto* results = static_cast<const float*>(
        static_cast<VulkanRHIBuffer*>(graph.GetBuffer(sums))->Mapped(device.CurrentFrame()));

    // Run 2 = this execute's gradient (×1) + run 1's accum (×1) = exactly double.
    // A culled copy pass or a non-persistent accum reads back ×1; a copy that
    // raced ahead of the reduce reads back ×4.
    CheckTexels("graph self-test", results,
                [](uint32_t x, uint32_t y) { return 2.0f * ExpectedGradientSum(x, y); });

    device.WaitIdle();
}

} // namespace Diamond
