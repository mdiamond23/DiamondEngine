#pragma once

#include "Platform/Vulkan/VulkanCommon.h"
#include "Platform/Vulkan/VulkanImage.h"

#include <glm/glm.hpp>

#include <memory>
#include <string>
#include <vector>

namespace Diamond {

class RHIDevice;
class RHIBuffer;
class RHIShader;
class RHIPipeline;
class RHIResourceSet;
class RHITexture;
class VulkanRHIDevice;
struct MeshData;
struct PBRMaterial;

// Editor asset previews — Vulkan port of the Sandbox's MeshThumbnailRenderer.
// Same studio-lit mesh render and material-ball preview, but instead of a GL FBO
// each thumbnail is a ONE-TIME bake through VulkanContext::ImmediateSubmit (the
// IBL pattern): render outside the frame loop into an owned RGBA8 image, leave it
// SHADER_READ_ONLY, and hand back the view+sampler for
// ImGui_ImplVulkan_AddTexture. Thumbnails are single images (not per-frame
// RHITextures) because they're written once and only ever sampled.
//
// Baked images live for the renderer's lifetime — the editor caches the returned
// handles per asset path, mirroring the GL panel's texture-ID cache. Destroy after
// WaitIdle, before the device.
class VulkanThumbnailRenderer {
public:
    static constexpr uint32_t kSize = 128;   // matches the GL MeshThumbnailRenderer

    VulkanThumbnailRenderer(RHIDevice* device, const std::string& shaderDir);
    ~VulkanThumbnailRenderer();

    VulkanThumbnailRenderer(const VulkanThumbnailRenderer&)            = delete;
    VulkanThumbnailRenderer& operator=(const VulkanThumbnailRenderer&) = delete;

    // A baked thumbnail, ready for ImGui_ImplVulkan_AddTexture(sampler, view,
    // VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL). Null view on failure.
    struct Thumb {
        VkImageView view    = VK_NULL_HANDLE;
        VkSampler   sampler = VK_NULL_HANDLE;
    };

    // Studio-lit render of a (multi-submesh) model, auto-framed by aggregate AABB.
    Thumb RenderMesh(const std::vector<MeshData>& meshes);

    // Material ball: the shared preview sphere under two analytic lights.
    Thumb RenderMaterial(const PBRMaterial& mat);

private:
    // Matches the ThumbUBO/PreviewUBO blocks (thumbnail.vert declares a prefix of
    // this; binding a larger buffer than the block is fine).
    struct ThumbUBO {
        glm::mat4 projection;
        glm::mat4 view;
        glm::mat4 model;
        glm::vec4 cameraPos;        // xyz
        glm::vec4 albedoFallback;   // rgb; w = uvScale
        glm::vec4 scalars;          // x metallicFallback, y roughnessFallback, z emissiveStrength
        glm::vec4 hasMaps;          // x albedo, y metallic, z roughness, w ao
        glm::vec4 hasMaps2;         // x emissive
    };
    struct DrawRange {
        RHIBuffer* vertexBuffer;
        RHIBuffer* indexBuffer;
        uint32_t   indexCount;
    };

    // Frame a unit-scale model: camera from AABB (largest face toward the lens),
    // view/proj/cameraPos into 'ubo'. Copies the GL framing math verbatim, with
    // the [0,1]-depth perspective.
    static void FrameAABB(const glm::vec3& aabbMin, const glm::vec3& aabbMax,
                          bool pickAxis, ThumbUBO& ubo);

    // Records the bake (transitions + dynamic rendering + draws) and returns the
    // baked color image, stored in m_Thumbs.
    Thumb Bake(RHIPipeline* pipeline, RHIResourceSet* set,
               const std::vector<DrawRange>& draws);

    // Repack MeshData into the shared 44-byte mesh vertex (pos/normal/uv/tangent)
    // and upload; returns {vb, ib, indexCount} appended to the transient list.
    DrawRange UploadMesh(const MeshData& data);

    VulkanRHIDevice* m_Device;

    VkSampler   m_Sampler = VK_NULL_HANDLE;   // linear/clamp, shared by all thumbs
    VulkanImage m_Depth{};                    // shared kSize² depth target

    std::unique_ptr<RHIShader>   m_MeshVert, m_MeshFrag;
    std::unique_ptr<RHIShader>   m_MatVert,  m_MatFrag;
    std::unique_ptr<RHIPipeline> m_MeshPipeline;
    std::unique_ptr<RHIPipeline> m_MatPipeline;

    // 1×1 neutral maps bound to unassigned material slots (descriptors must be
    // valid even when the UBO flag routes the shader to the scalar fallback).
    std::unique_ptr<RHITexture> m_White, m_Black, m_Gray;

    // The preview sphere, uploaded once (buffers owned by m_Buffers).
    DrawRange m_Sphere{};
    glm::vec3 m_SphereMin{ 0.0f }, m_SphereMax{ 0.0f };

    // Per-bake resources kept alive for the renderer's lifetime: the baked images
    // ImGui keeps sampling, plus each bake's UBO + descriptor set + geometry.
    std::vector<VulkanImage>                     m_Thumbs;
    std::vector<std::unique_ptr<RHIBuffer>>      m_Buffers;
    std::vector<std::unique_ptr<RHIResourceSet>> m_Sets;
};

} // namespace Diamond
