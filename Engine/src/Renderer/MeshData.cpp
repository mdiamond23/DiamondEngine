#include "Renderer/MeshData.h"
#include "Renderer/RendererAPI.h"
#include "Platform/OpenGL/Resources/OpenGLMesh.h"
#include <cmath>

namespace Diamond {

MeshData MeshData::UnitCube()
{
    // 6 faces × 4 vertices = 24 vertices, each with full PBR attributes.
    // Tangent chosen so cross(N, T) = bitangent = dP/dV direction.
    struct Face { glm::vec3 p[4]; glm::vec2 uv[4]; glm::vec3 n, t; };
    const Face faces[6] = {
        // Front (+Z)
        { {{-1,-1,1},{1,-1,1},{1,1,1},{-1,1,1}},   {{0,0},{1,0},{1,1},{0,1}}, {0,0,1},  {1,0,0}  },
        // Back (-Z): U flipped so texture reads correctly from outside
        { {{1,-1,-1},{-1,-1,-1},{-1,1,-1},{1,1,-1}},{{0,0},{1,0},{1,1},{0,1}}, {0,0,-1}, {-1,0,0} },
        // Right (+X)
        { {{1,-1,1},{1,-1,-1},{1,1,-1},{1,1,1}},    {{0,0},{1,0},{1,1},{0,1}}, {1,0,0},  {0,0,-1} },
        // Left (-X)
        { {{-1,-1,-1},{-1,-1,1},{-1,1,1},{-1,1,-1}},{{0,0},{1,0},{1,1},{0,1}}, {-1,0,0}, {0,0,1}  },
        // Top (+Y)
        { {{-1,1,1},{1,1,1},{1,1,-1},{-1,1,-1}},    {{0,0},{1,0},{1,1},{0,1}}, {0,1,0},  {1,0,0}  },
        // Bottom (-Y)
        { {{-1,-1,-1},{1,-1,-1},{1,-1,1},{-1,-1,1}},{{0,0},{1,0},{1,1},{0,1}}, {0,-1,0}, {1,0,0}  },
    };

    MeshData data;
    data.Vertices.reserve(24);
    data.Indices.reserve(36);
    for (int f = 0; f < 6; ++f) {
        const auto& fd = faces[f];
        glm::vec3 B = glm::cross(fd.n, fd.t);
        uint32_t base = static_cast<uint32_t>(f * 4);
        for (int v = 0; v < 4; ++v) {
            Vertex vert{};
            vert.Position  = fd.p[v];
            vert.Normal    = fd.n;
            vert.TexCoords = fd.uv[v];
            vert.Tangent   = fd.t;
            vert.Bitangent = B;
            data.Vertices.push_back(vert);
        }
        data.Indices.insert(data.Indices.end(), {base,base+1,base+2, base,base+2,base+3});
    }
    return data;
}

MeshData MeshData::FullscreenQuad()
{
    auto v = [](glm::vec3 pos, glm::vec2 uv) {
        Vertex vert{};
        vert.Position  = pos;
        vert.TexCoords = uv;
        return vert;
    };
    MeshData data;
    data.Vertices = {
        v({-1,  1, 0}, {0, 1}),
        v({-1, -1, 0}, {0, 0}),
        v({ 1,  1, 0}, {1, 1}),
        v({ 1, -1, 0}, {1, 0}),
    };
    data.Indices = {0, 1, 2, 1, 3, 2};
    return data;
}

MeshData MeshData::UVSphere()
{
    const unsigned int STACKS = 64;
    const unsigned int SLICES = 64;
    const float PI = 3.14159265358979323846f;

    MeshData data;
    data.Vertices.reserve((STACKS + 1) * (SLICES + 1));
    data.Indices.reserve(STACKS * SLICES * 6);

    for (unsigned int i = 0; i <= STACKS; ++i) {
        float theta    = PI * i / STACKS;
        float sinTheta = std::sin(theta);
        float cosTheta = std::cos(theta);

        for (unsigned int j = 0; j <= SLICES; ++j) {
            float phi    = 2.0f * PI * j / SLICES;
            float sinPhi = std::sin(phi);
            float cosPhi = std::cos(phi);

            Vertex v{};
            v.Position  = { sinTheta * cosPhi, cosTheta, sinTheta * sinPhi };
            v.Normal    = v.Position;
            v.TexCoords = { (float)j / SLICES, (float)i / STACKS };
            v.Tangent   = glm::normalize(glm::vec3(-sinPhi, 0.0f, cosPhi));
            v.Bitangent = glm::normalize(glm::vec3(cosTheta * cosPhi, -sinTheta, cosTheta * sinPhi));
            data.Vertices.push_back(v);
        }
    }

    for (unsigned int i = 0; i < STACKS; ++i) {
        for (unsigned int j = 0; j < SLICES; ++j) {
            uint32_t a = i * (SLICES + 1) + j;
            uint32_t b = a + SLICES + 1;
            data.Indices.push_back(a);     data.Indices.push_back(b);     data.Indices.push_back(a + 1);
            data.Indices.push_back(b);     data.Indices.push_back(b + 1); data.Indices.push_back(a + 1);
        }
    }

    return data;
}

namespace {
// Vulkan-mode mesh: no GPU upload here — the SceneRenderer owns the vertex and
// index buffers, keyed on this object's address, and uploads lazily from the
// retained copy the first frame the mesh is drawn. Draw() is the GL immediate
// path and never runs under Vulkan.
class CPUMesh final : public Mesh {
public:
    explicit CPUMesh(const MeshData& data) : m_Data(data) {}
    void Draw(const Shader&) const override {}
    const MeshData* CPUData() const override { return &m_Data; }
private:
    MeshData m_Data;
};
} // namespace

std::shared_ptr<Mesh> Mesh::Create(const MeshData& data)
{
    switch (RendererAPI::GetAPI()) {
        case RendererAPI::API::OpenGL: return std::make_shared<OpenGLMesh>(data);
        case RendererAPI::API::Vulkan: return std::make_shared<CPUMesh>(data);
        default:                       return nullptr;
    }
}

AABB MeshData::ComputeAABB() const
{
    AABB box;
    for (const auto& v : Vertices) {
        box.min = glm::min(box.min, v.Position);
        box.max = glm::max(box.max, v.Position);
    }
    return box;
}

} // namespace Diamond