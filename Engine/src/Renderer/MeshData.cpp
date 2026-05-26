#include "Renderer/MeshData.h"
#include "Renderer/RendererAPI.h"
#include "Platform/OpenGL/OpenGLMesh.h"
#include <cmath>

namespace Diamond {

MeshData MeshData::UnitCube()
{
    static const glm::vec3 positions[36] = {
        {-1, 1,-1}, {-1,-1,-1}, { 1,-1,-1}, { 1,-1,-1}, { 1, 1,-1}, {-1, 1,-1},
        {-1,-1, 1}, {-1,-1,-1}, {-1, 1,-1}, {-1, 1,-1}, {-1, 1, 1}, {-1,-1, 1},
        { 1,-1,-1}, { 1,-1, 1}, { 1, 1, 1}, { 1, 1, 1}, { 1, 1,-1}, { 1,-1,-1},
        {-1,-1, 1}, {-1, 1, 1}, { 1, 1, 1}, { 1, 1, 1}, { 1,-1, 1}, {-1,-1, 1},
        {-1, 1,-1}, { 1, 1,-1}, { 1, 1, 1}, { 1, 1, 1}, {-1, 1, 1}, {-1, 1,-1},
        {-1,-1,-1}, {-1,-1, 1}, { 1,-1,-1}, { 1,-1,-1}, {-1,-1, 1}, { 1,-1, 1},
    };
    MeshData data;
    data.Vertices.resize(36);
    for (int i = 0; i < 36; ++i)
        data.Vertices[i].Position = positions[i];
    data.Indices.resize(36);
    for (uint32_t i = 0; i < 36; ++i)
        data.Indices[i] = i;
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

std::shared_ptr<Mesh> Mesh::Create(const MeshData& data)
{
    switch (RendererAPI::GetAPI()) {
        case RendererAPI::API::OpenGL: return std::make_shared<OpenGLMesh>(data);
        default:                       return nullptr;
    }
}

} // namespace Diamond