#pragma once
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/quaternion.hpp>
#include <string>
#include <memory>
#include <cstdint>

struct PhysicsMaterial
{
    float staticFriction  = 0.5f;
    float dynamicFriction = 0.5f;
    float restitution     = 0.3f;  // bounciness [0, 1]
};

enum class CollisionShape { Box, Sphere, Capsule, ConvexHull, TriangleMesh };

struct ColliderComponent
{
    CollisionShape shapeType = CollisionShape::Box;

    // Box
    glm::vec3 halfExtents { 0.5f, 0.5f, 0.5f };
    // Sphere + Capsule
    float radius     = 0.5f;
    // Capsule (cylinder halfHeight, capped by radius on each end)
    float halfHeight = 0.5f;
    // ConvexHull / TriangleMesh — path to mesh asset
    std::string meshPath;

    glm::vec3 localOffset   { 0.0f, 0.0f, 0.0f };
    glm::quat localRotation { 1, 0, 0, 0 };  // identity

    bool isTrigger = false;
    std::shared_ptr<PhysicsMaterial> material;  // null = engine default

    uint32_t _bodyId = 0xFFFFFFFFu;  // set by PhysicsSystem; invalid sentinel mirrors RigidBodyComponent
};

