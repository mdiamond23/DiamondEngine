#pragma once

#include "Renderer/RHI/RHIResources.h"

#include <spdlog/spdlog.h>

#include <cstdint>
#include <cstdlib>
#include <fstream>
#include <glm/glm.hpp>
#include <string>
#include <vector>

// Shared helpers for the Vulkan render passes (Platform/Vulkan/Passes/*). Kept
// header-only and tiny — a place for utilities every pass needs without dragging
// in the device/backend types.
namespace Diamond {

// ── Skinned vertex layout ────────────────────────────────────────────────────
// The SceneRenderer uploads skinned meshes as pos/normal/uv/tangent + bone indices
// (ivec4) + weights (vec4). Every skinned pipeline (G-buffer, CSM/spot, point
// shadow) reads from this one interleaved layout, so the stride + attribute table
// live here rather than being re-derived per pass.
constexpr uint32_t kSkinnedVertexStride =
    sizeof(glm::vec3) * 3 + sizeof(glm::vec2) + sizeof(glm::ivec4) + sizeof(glm::vec4);  // 76

// Attributes 0-3 match the static MeshVertex; 4 = bone indices, 5 = bone weights.
// Full layout for the G-buffer skinned pass (needs normal/uv/tangent).
inline std::vector<RHIVertexAttribute> SkinnedVertexAttributes() {
    return {
        { 0, RHIVertexFormat::Float3, 0 },                                          // position
        { 1, RHIVertexFormat::Float3, sizeof(glm::vec3) },                          // normal
        { 2, RHIVertexFormat::Float2, sizeof(glm::vec3) * 2 },                      // uv
        { 3, RHIVertexFormat::Float3, sizeof(glm::vec3) * 2 + sizeof(glm::vec2) },  // tangent
        { 4, RHIVertexFormat::Int4,   sizeof(glm::vec3) * 3 + sizeof(glm::vec2) },  // bone indices
        { 5, RHIVertexFormat::Float4, sizeof(glm::vec3) * 3 + sizeof(glm::vec2) + sizeof(glm::ivec4) }, // weights
    };
}

// Depth-only subset (position + bone data) for the skinned shadow pipelines, which
// ignore normal/uv/tangent. Same 76-byte stride (the vertex buffer is shared) but
// only the consumed attributes are declared, so the pipeline validates cleanly.
inline std::vector<RHIVertexAttribute> SkinnedDepthVertexAttributes() {
    return {
        { 0, RHIVertexFormat::Float3, 0 },                                          // position
        { 4, RHIVertexFormat::Int4,   sizeof(glm::vec3) * 3 + sizeof(glm::vec2) },  // bone indices
        { 5, RHIVertexFormat::Float4, sizeof(glm::vec3) * 3 + sizeof(glm::vec2) + sizeof(glm::ivec4) }, // weights
    };
}

// Set-1 binding table shared by every skinned pipeline: a single vertex-stage
// uniform buffer holding the bone-matrix palette (mat4[MAX_BONES]).
inline std::vector<RHIResourceBinding> BonesSetBindings() {
    return { { 0, RHIResourceType::UniformBuffer, RHIShaderStage::Vertex } };
}

// Reads a compiled SPIR-V module (.spv) emitted by the build's glslang step. 'dir'
// is the baked DIAMOND_VULKAN_SHADER_DIR; 'name' is e.g. "tonemap.frag.spv".
// A missing module is a build-wiring bug, not a recoverable runtime state — abort.
inline std::vector<uint32_t> LoadSpirv(const std::string& dir, const std::string& name)
{
    const std::string path = dir + "/" + name;
    std::ifstream file(path, std::ios::binary | std::ios::ate);
    if (!file) {
        spdlog::critical("[Vulkan] failed to open SPIR-V '{}'", path);
        std::abort();
    }
    const std::streamsize bytes = file.tellg();
    std::vector<uint32_t> words(static_cast<size_t>(bytes) / sizeof(uint32_t));
    file.seekg(0);
    file.read(reinterpret_cast<char*>(words.data()), bytes);
    return words;
}

} // namespace Diamond
