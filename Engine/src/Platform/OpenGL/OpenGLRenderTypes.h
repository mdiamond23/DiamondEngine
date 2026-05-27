#pragma once
#include "Renderer/MeshData.h"
#include <glm/glm.hpp>

namespace Diamond {

struct DrawCall {
    const Mesh* mesh;
    glm::mat4   modelMatrix;
};

struct SunLight {
    glm::vec3 direction        = glm::vec3(-0.3f, -1.0f, -0.5f);
    glm::vec3 color            = glm::vec3(5.0f);
    glm::mat4 lightSpaceMatrix = glm::mat4(1.0f);
};

} // namespace Diamond
