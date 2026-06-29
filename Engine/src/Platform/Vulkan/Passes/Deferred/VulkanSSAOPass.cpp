#include "Platform/Vulkan/Passes/Deferred/VulkanSSAOPass.h"
#include "Platform/Vulkan/Passes/VulkanPassCommon.h"
#include "Renderer/RHI/RHIDevice.h"
#include "Renderer/RHI/RHICommandList.h"

#include <glm/gtc/matrix_transform.hpp>

#include <cstdint>
#include <random>
#include <vector>

namespace Diamond {

namespace {
constexpr RHIFormat kSSAOFormat = RHIFormat::R16F;   // single-channel occlusion
} // namespace

VulkanSSAOPass::VulkanSSAOPass(RHIDevice* device, const std::string& shaderDir,
                               uint32_t width, uint32_t height)
    : m_Device(device)
{
    // ── Hemisphere kernel + noise (built once; identical to the GL pass) ─────────
    std::uniform_real_distribution<float> rng(0.0f, 1.0f);
    std::default_random_engine gen(42u);

    for (int i = 0; i < 64; ++i) {
        glm::vec3 sample(rng(gen) * 2.0f - 1.0f, rng(gen) * 2.0f - 1.0f, rng(gen));
        sample = glm::normalize(sample) * rng(gen);
        float scale = static_cast<float>(i) / 64.0f;
        scale = glm::mix(0.1f, 1.0f, scale * scale);   // denser near the origin
        m_UBOData.samples[i] = glm::vec4(sample * scale, 0.0f);
    }
    m_UBOData.noiseScale = glm::vec2(static_cast<float>(width)  / 4.0f,
                                     static_cast<float>(height) / 4.0f);

    // 4×4 rotation vectors around view-space Z, RGBA8-encoded ([-1,1]→[0,255]); the
    // shader decodes (*2-1). b stores 0.5 so the decoded z is ~0.
    std::vector<uint8_t> noise(16 * 4);
    for (int i = 0; i < 16; ++i) {
        noise[i * 4 + 0] = static_cast<uint8_t>((rng(gen)) * 255.0f);   // (x*0.5+0.5) == rng
        noise[i * 4 + 1] = static_cast<uint8_t>((rng(gen)) * 255.0f);
        noise[i * 4 + 2] = 128;
        noise[i * 4 + 3] = 255;
    }
    RHITextureDesc noiseDesc;
    noiseDesc.width       = 4;
    noiseDesc.height      = 4;
    noiseDesc.format      = RHIFormat::RGBA8;
    noiseDesc.usage       = RHITextureUsage::Sampled;
    noiseDesc.initialData = noise.data();
    noiseDesc.filter      = RHIFilter::Nearest;   // no interpolation across the tile
    m_Noise = device->CreateTexture(noiseDesc);

    // Dynamic UBO — kernel + noiseScale are filled now, projection per frame.
    RHIBufferDesc uboDesc;
    uboDesc.size    = sizeof(SSAOUBO);
    uboDesc.usage   = RHIBufferUsage::Uniform;
    uboDesc.dynamic = true;
    m_UBO = device->CreateBuffer(uboDesc);

    // ── Pipelines ────────────────────────────────────────────────────────────────
    const std::vector<uint32_t> vs   = LoadSpirv(shaderDir, "fullscreen.vert.spv");
    const std::vector<uint32_t> ssao = LoadSpirv(shaderDir, "ssao.frag.spv");
    const std::vector<uint32_t> blur = LoadSpirv(shaderDir, "ssao_blur.frag.spv");
    RHIShaderDesc vsDesc{ RHIShaderStage::Vertex,   vs.data(),   vs.size() };
    RHIShaderDesc ssaoDesc{ RHIShaderStage::Fragment, ssao.data(), ssao.size() };
    RHIShaderDesc blurDesc{ RHIShaderStage::Fragment, blur.data(), blur.size() };
    m_FullscreenVert = device->CreateShader(vsDesc);
    m_SSAOFrag       = device->CreateShader(ssaoDesc);
    m_BlurFrag       = device->CreateShader(blurDesc);

    {
        RHIPipelineDesc desc;
        desc.vertexShader     = m_FullscreenVert.get();
        desc.fragmentShader   = m_SSAOFrag.get();
        desc.resourceBindings = {
            { 0, RHIResourceType::CombinedImageSampler, RHIShaderStage::Fragment }, // gViewPos
            { 1, RHIResourceType::CombinedImageSampler, RHIShaderStage::Fragment }, // gViewNormal
            { 2, RHIResourceType::CombinedImageSampler, RHIShaderStage::Fragment }, // texNoise
            { 3, RHIResourceType::UniformBuffer,        RHIShaderStage::Fragment }, // SSAOUBO
        };
        desc.colorFormat = kSSAOFormat;
        m_SSAOPipeline = device->CreatePipeline(desc);
    }
    {
        RHIPipelineDesc desc;
        desc.vertexShader     = m_FullscreenVert.get();
        desc.fragmentShader   = m_BlurFrag.get();
        desc.resourceBindings = {
            { 0, RHIResourceType::CombinedImageSampler, RHIShaderStage::Fragment }, // ssaoInput
        };
        desc.colorFormat = kSSAOFormat;
        m_BlurPipeline = device->CreatePipeline(desc);
    }
}

VulkanSSAOPass::~VulkanSSAOPass() = default;

void VulkanSSAOPass::AddToGraph(RHIRenderGraph& graph,
                                RGTextureHandle viewPos, RGTextureHandle viewNormal,
                                RGTextureHandle ssaoRaw, RGTextureHandle ssaoBlurred)
{
    if (!m_SSAOSet)
        m_SSAOSet = m_Device->CreateResourceSet(
            m_SSAOPipeline.get(), 0, { { 3, m_UBO.get() } },
            { { 0, graph.GetTexture(viewPos) },
              { 1, graph.GetTexture(viewNormal) },
              { 2, m_Noise.get() } });
    if (!m_BlurSet)
        m_BlurSet = m_Device->CreateResourceSet(
            m_BlurPipeline.get(), 0, {}, { { 0, graph.GetTexture(ssaoRaw) } });

    graph.AddPass("SSAO")
        .Read(viewPos).Read(viewNormal)
        .Write(ssaoRaw)
        .SetExecute([this](RHICommandList* cmd) {
            cmd->BindPipeline(m_SSAOPipeline.get());
            cmd->BindResourceSet(0, m_SSAOSet.get());
            cmd->Draw(3);
        });

    graph.AddPass("SSAOBlur")
        .Read(ssaoRaw)
        .Write(ssaoBlurred)
        .SetExecute([this](RHICommandList* cmd) {
            cmd->BindPipeline(m_BlurPipeline.get());
            cmd->BindResourceSet(0, m_BlurSet.get());
            cmd->Draw(3);
        });
}

void VulkanSSAOPass::SetProjection(const glm::mat4& projection) {
    m_UBOData.projection = projection;
    m_UBO->Update(&m_UBOData, sizeof(SSAOUBO));
}

} // namespace Diamond
