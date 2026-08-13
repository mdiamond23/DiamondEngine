#pragma once

#include "Platform/Vulkan/VulkanCommon.h"
#include "Platform/Vulkan/VulkanImage.h"

#include <cstdint>
#include <string>

namespace Diamond {

class RHIDevice;
class VulkanRHIDevice;

// Image-based lighting bake — Vulkan port of OpenGLIBLPass. Turns an
// equirectangular HDR environment into the three maps the PBR ambient term needs:
//   * irradiance cubemap  — diffuse, sampled by world normal
//   * prefilter cubemap   — specular, mip = roughness, sampled by reflection
//   * BRDF LUT (2D)       — split-sum scale+bias, sampled by (NdotV, roughness)
//
// Unlike every other ported pass, IBL is a ONE-TIME bake, not a per-frame graph
// pass: the render graph drives full-texture passes once per frame and can't render
// into individual cube faces / mip levels. So BakeEnvironment records the whole
// bake through VulkanContext::ImmediateSubmit (the same one-shot path texture
// uploads use), reaching VulkanContext/VulkanImage directly — the RHI is Vulkan-
// only, so there's no second backend to abstract a cubemap for. The pass still owns
// its pipelines+shaders (built and torn down inside BakeEnvironment) and the baked
// maps live for the device's lifetime.
//
// The baked maps aren't RHITextures (those are 2D, per-frame-in-flight images), so
// the deferred-lighting pass binds them through the raw views/sampler exposed here.
class VulkanIBLPass {
public:
    VulkanIBLPass(RHIDevice* device, std::string shaderDir);
    ~VulkanIBLPass();

    VulkanIBLPass(const VulkanIBLPass&)            = delete;
    VulkanIBLPass& operator=(const VulkanIBLPass&) = delete;

    // Load 'hdrPath' (equirectangular .hdr) and bake all three maps. Synchronous;
    // call once at startup before the frame loop. Safe to call after the device is
    // created (uses ImmediateSubmit, not the per-frame command list).
    void BakeEnvironment(const std::string& hdrPath);

    // Raw handles for the deferred-lighting descriptor set. Valid after
    // BakeEnvironment; all three share one sampler (linear, clamp, mipmapped).
    VkImageView IrradianceView() const { return m_Irradiance.cubeView; }
    VkImageView PrefilterView()  const { return m_Prefilter.cubeView; }
    VkImageView BrdfView()       const { return m_BrdfLUT.view; }
    // The intermediate radiance cubemap. Feeds the irradiance/prefilter bakes
    // and DDGI's miss shader — NOT the visible background, which samples the
    // equirect directly (see EquirectView).
    VkImageView EnvView()        const { return m_EnvCube.cubeView; }
    VkSampler   Sampler()        const { return m_Sampler; }

    // The source equirect, kept alive past the bake so the skybox can display it
    // without going through the cube. A cube face's corners cover ~3x the solid
    // angle per texel that its centre does, so sharpness varies across every
    // face and the cube's edges become visible on a big smooth sky. Sampling the
    // equirect has no faces to see. Its sampler REPEATs in u so bilinear blends
    // across the +/-180 meridian instead of clamping into a seam.
    VkImageView EquirectView()    const { return m_Equirect.view; }
    VkSampler   EquirectSampler() const { return m_EquirectSampler; }

    // Mip count the prefilter map was baked with (deferred lighting scales roughness
    // by maxLod = NumPrefilterMips - 1 when sampling).
    uint32_t NumPrefilterMips() const { return m_Prefilter.mipLevels; }

private:
    // A cubemap image with a CUBE view over its whole mip chain (for sampling). The
    // per-face/per-mip views the bake renders into are transient (created+destroyed
    // around the bake submit), so they aren't stored here.
    struct Cubemap {
        VkImage       image     = VK_NULL_HANDLE;
        VmaAllocation alloc     = VK_NULL_HANDLE;
        VkImageView   cubeView  = VK_NULL_HANDLE;
        uint32_t      size      = 0;
        uint32_t      mipLevels = 1;
        VkFormat      format    = VK_FORMAT_UNDEFINED;
    };

    // Frees everything BakeEnvironment allocates — destructor and re-bake.
    void DestroyBakedResources();

    VulkanRHIDevice* m_Device;
    std::string      m_ShaderDir;

    VkSampler m_Sampler = VK_NULL_HANDLE;   // shared by all baked maps + bake sources
    VkSampler m_EquirectSampler = VK_NULL_HANDLE;   // REPEAT in u — see EquirectView

    VulkanImage m_Equirect{};     // source HDR, RGBA16F, outlives the bake (skybox reads it)
    Cubemap     m_EnvCube{};      // intermediate radiance cubemap (mipped, sampled by the convolutions)
    Cubemap     m_Irradiance{};
    Cubemap     m_Prefilter{};
    VulkanImage m_BrdfLUT{};      // 2D RGBA16F (result in .rg)
};

} // namespace Diamond
