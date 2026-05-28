#pragma once

#include "Renderer/MeshData.h"
#include <cstdint>

namespace Diamond {

class OpenGLMesh : public Mesh {
public:
    explicit OpenGLMesh(const MeshData& data);
    ~OpenGLMesh() override;

    void Draw(const Shader& shader) const override;

private:
    uint32_t m_VAO        = 0;
    uint32_t m_VBO        = 0;
    uint32_t m_EBO        = 0;
    uint32_t m_IndexCount = 0;

    void Upload(const MeshData& data);
};

} // namespace Diamond