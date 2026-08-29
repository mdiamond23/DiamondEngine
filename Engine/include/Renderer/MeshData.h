#pragma once

#include <glm/glm.hpp>
#include <vector>
#include <cstdint>
#include <memory>
#include "Renderer/Frustum.h"

namespace Diamond {

// Up to this many bones can influence a single vertex (glTF's standard limit;
// also the width of the ivec4/vec4 skinning attributes below).
inline constexpr int kMaxBoneInfluence = 4;

struct Vertex {
    glm::vec3 Position;
    glm::vec3 Normal;
    glm::vec2 TexCoords;
    glm::vec3 Tangent;
    glm::vec3 Bitangent;

    // Skinning: which bones move this vertex and how much. Zero-weighted by
    // default so non-skinned meshes (loaded through the static path) are inert
    // here — the skinning shader weights resolve to no influence.
    glm::ivec4 BoneIDs     { 0, 0, 0, 0 };
    glm::vec4  BoneWeights { 0.0f, 0.0f, 0.0f, 0.0f };
};

// CPU-side mesh data produced by ModelImporter. Pass to Mesh::Create() to upload to the GPU.
struct MeshData {
    std::vector<Vertex>   Vertices;
    std::vector<uint32_t> Indices;

    static MeshData UnitCube();
    static MeshData FullscreenQuad();
    static MeshData UVSphere();
    // Ring radius 0.75, tube radius 0.25 — outer radius 1, so it shares the
    // unit-sphere convention and a scale of 1 gives a 2-unit-wide donut.
    static MeshData Torus();

    AABB ComputeAABB() const;
};

class Shader;

// Abstract GPU mesh. Backend-specific (OpenGL, Vulkan) instances are created via Mesh::Create().
class Mesh {
public:
    virtual ~Mesh() = default;
    virtual void Draw(const Shader& shader) const = 0;

    // CPU-side geometry retained by meshes whose GPU upload is deferred to the
    // SceneRenderer (Vulkan): the renderer registers an unseen Mesh* lazily from
    // this data on first draw. Null for meshes that upload in their constructor
    // (OpenGL), and for externally-registered geometry.
    virtual const MeshData* CPUData() const { return nullptr; }

    static std::shared_ptr<Mesh> Create(const MeshData& data);
};

} // namespace Diamond
