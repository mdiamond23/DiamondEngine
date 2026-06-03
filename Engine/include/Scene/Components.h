#pragma once
#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <memory>
#include <string>
#include "Renderer/MeshData.h"
#include "Renderer/Material.h"
#include "Renderer/Frustum.h"
#include <cstdint>  

struct TransformComponent
{
    glm::vec3 position { 0, 0, 0 };
    glm::quat rotation { 1, 0, 0, 0 };  // identity
    glm::vec3 scale    { 1, 1, 1 };

    inline glm::mat4 GetLocalMatrix() const
    {
        glm::mat4 t = glm::translate(glm::mat4(1.0f), position);
        glm::mat4 r = glm::mat4_cast(rotation);
        glm::mat4 s = glm::scale(glm::mat4(1.0f), scale);
        return t * r * s;
    }

    // inspector only
    inline  glm::vec3 GetEulerAngles() const
    {
        return glm::degrees(glm::eulerAngles(rotation));
    }
};

struct MeshComponent
{
    std::shared_ptr<Diamond::Mesh>        mesh;
    std::shared_ptr<Diamond::PBRMaterial> material;
    Diamond::AABB                         localBounds;

    bool visible        = true;
    bool castsShadow    = true;
    bool receivesShadow = true;

    std::string meshPath;
    int         meshSubIndex = 0;
    std::string materialPath;
};

struct IDComponent {
    uint64_t uuid;
};
