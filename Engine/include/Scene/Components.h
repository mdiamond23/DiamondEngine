#pragma once
#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <entt/entt.hpp>
#include <memory>
#include <string>
#include <vector>
#include "Renderer/MeshData.h"
#include "Renderer/Material.h"
#include "Renderer/Frustum.h"
#include <cstdint>

enum class LightType {
    Sun,
    Point,
    Spot
};

struct TransformComponent
{
    glm::vec3 position     { 0, 0, 0 };
    glm::quat rotation     { 1, 0, 0, 0 };  // identity
    glm::vec3 eulerDegrees { 0, 0, 0 };     // editor display cache; kept in sync with rotation
    glm::vec3 scale        { 1, 1, 1 };

    inline glm::mat4 GetLocalMatrix() const
    {
        glm::mat4 t = glm::translate(glm::mat4(1.0f), position);
        glm::mat4 r = glm::mat4_cast(rotation);
        glm::mat4 s = glm::scale(glm::mat4(1.0f), scale);
        return t * r * s;
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

struct LightComponent
{
    LightType type           = LightType::Point;
    glm::vec3 color          = { 1.0f, 1.0f, 1.0f };
    float     intensity      = 100.0f;
    float     radius         = 10.0f;   // Point/Spot: max influence range
    float     innerConeAngle = 15.0f;   // Spot: inner cone, degrees
    float     outerConeAngle = 30.0f;   // Spot: outer cone, degrees
};

struct CameraComponent
{
    bool  isPrimary = true;
    float fov       = 60.0f;
    float nearClip  = 0.1f;
    float farClip   = 1000.0f;
};

struct IDComponent {
    uint64_t uuid;
};

// Scene hierarchy — parent/children stored as entity handles.
// Always mutate through Scene::SetParent / Scene::UnsetParent so the
// TransformSystem flat arrays stay consistent.
struct HierarchyComponent {
    entt::entity              parent   = entt::null;
    std::vector<entt::entity> children;
};
