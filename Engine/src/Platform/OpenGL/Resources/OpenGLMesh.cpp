#include "Platform/OpenGL/Resources/OpenGLMesh.h"
#include "Profiling/GLRendererStats.h"

#include <glad/gl.h>

namespace Diamond {

OpenGLMesh::OpenGLMesh(const MeshData& data)
{
    Upload(data);
}

OpenGLMesh::~OpenGLMesh()
{
    GLStats::RecordBufferFree(m_VertexBytes + m_IndexBytes);
    glDeleteVertexArrays(1, &m_VAO);
    glDeleteBuffers(1, &m_VBO);
    glDeleteBuffers(1, &m_EBO);
}

void OpenGLMesh::Upload(const MeshData& data)
{
    m_IndexCount = static_cast<uint32_t>(data.Indices.size());

    glGenVertexArrays(1, &m_VAO);
    glGenBuffers(1, &m_VBO);
    glGenBuffers(1, &m_EBO);

    glBindVertexArray(m_VAO);

    m_VertexBytes = data.Vertices.size() * sizeof(Vertex);
    m_IndexBytes  = data.Indices.size() * sizeof(uint32_t);

    glBindBuffer(GL_ARRAY_BUFFER, m_VBO);
    glBufferData(GL_ARRAY_BUFFER,
                 m_VertexBytes,
                 data.Vertices.data(),
                 GL_STATIC_DRAW);

    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, m_EBO);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER,
                 m_IndexBytes,
                 data.Indices.data(),
                 GL_STATIC_DRAW);

    GLStats::RecordBufferAlloc(m_VertexBytes + m_IndexBytes);

    // layout (location = 0) Position
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(Vertex),
                          (void*)offsetof(Vertex, Position));
    // layout (location = 1) Normal
    glEnableVertexAttribArray(1);
    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, sizeof(Vertex),
                          (void*)offsetof(Vertex, Normal));
    // layout (location = 2) TexCoords
    glEnableVertexAttribArray(2);
    glVertexAttribPointer(2, 2, GL_FLOAT, GL_FALSE, sizeof(Vertex),
                          (void*)offsetof(Vertex, TexCoords));
    // layout (location = 3) Tangent
    glEnableVertexAttribArray(3);
    glVertexAttribPointer(3, 3, GL_FLOAT, GL_FALSE, sizeof(Vertex),
                          (void*)offsetof(Vertex, Tangent));
    // layout (location = 4) Bitangent
    glEnableVertexAttribArray(4);
    glVertexAttribPointer(4, 3, GL_FLOAT, GL_FALSE, sizeof(Vertex),
                          (void*)offsetof(Vertex, Bitangent));
    // layout (location = 5) BoneIDs — integer attribute, so glVertexAttribIPointer
    // (not the float variant) to keep the bone indices exact in the shader.
    glEnableVertexAttribArray(5);
    glVertexAttribIPointer(5, 4, GL_INT, sizeof(Vertex),
                           (void*)offsetof(Vertex, BoneIDs));
    // layout (location = 6) BoneWeights
    glEnableVertexAttribArray(6);
    glVertexAttribPointer(6, 4, GL_FLOAT, GL_FALSE, sizeof(Vertex),
                          (void*)offsetof(Vertex, BoneWeights));

    glBindVertexArray(0);
}

void OpenGLMesh::Draw(const Shader& /*shader*/) const
{
    glBindVertexArray(m_VAO);
    glDrawElements(GL_TRIANGLES, m_IndexCount, GL_UNSIGNED_INT, nullptr);
    glBindVertexArray(0);
    GLStats::RecordDraw(m_IndexCount / 3);
}

} // namespace Diamond
