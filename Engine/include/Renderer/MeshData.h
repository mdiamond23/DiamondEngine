#pragma once

#include <glm/glm.hpp>
#include <vector>
#include <cstdint>
#include <memory>

namespace Diamond {

struct Vertex {
    glm::vec3 Position;
    glm::vec3 Normal;
    glm::vec2 TexCoords;
    glm::vec3 Tangent;
    glm::vec3 Bitangent;
};

// CPU-side mesh data produced by ModelImporter. Pass to Mesh::Create() to upload to the GPU.
struct MeshData {
    std::vector<Vertex>   Vertices;
    std::vector<uint32_t> Indices;
};

class Shader;

// Abstract GPU mesh. Backend-specific (OpenGL, Vulkan) instances are created via Mesh::Create().
class Mesh {
public:
    virtual ~Mesh() = default;
    virtual void Draw(const Shader& shader) const = 0;

    static std::shared_ptr<Mesh> Create(const MeshData& data);
};

} // namespace Diamond
