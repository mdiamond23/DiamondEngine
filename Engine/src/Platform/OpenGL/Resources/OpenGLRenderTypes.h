#pragma once
#include "Renderer/MeshData.h"
#include "Renderer/Frustum.h"
#include <glm/glm.hpp>

namespace Diamond {

struct DrawCall {
    const Mesh* mesh;
    glm::mat4   modelMatrix;
    AABB        localBounds;
    bool        castsShadow = true;
};

struct SunLight {
    glm::vec3 direction = glm::vec3(-0.3f, -1.0f, -0.5f);
    glm::vec3 color     = glm::vec3(5.0f);
};

} // namespace Diamond
