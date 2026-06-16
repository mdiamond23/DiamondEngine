#pragma once
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/quaternion.hpp>
#include <entt/entt.hpp>
#include <functional>
#include <string>
#include <memory>
#include <cstdint>

struct CollisionContact {
    entt::entity other;
    glm::vec3    contactPoint;
    glm::vec3    contactNormal;
};

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
    // Path to a .physmat asset file; empty = no asset (use inline material or engine defaults)
    std::string physMatPath;

    glm::vec3 localOffset   { 0.0f, 0.0f, 0.0f };
    glm::quat localRotation { 1, 0, 0, 0 };  // identity

    bool isTrigger = false;

    // Collision group. Bodies sharing the same NON-ZERO group never collide with
    // each other (group 0 = ungrouped, collides normally). Used to stop the
    // connected, overlapping bones of a ragdoll from exploding at their joints —
    // give every bone of one ragdoll the same group number; different ragdolls
    // use different numbers so they still collide with each other.
    uint32_t collisionGroup = 0;

    std::shared_ptr<PhysicsMaterial> material;  // null = engine default

    // Collision callbacks — set by scripts, fired by PhysicsSystem
    std::function<void(CollisionContact)> onCollisionEnter;
    std::function<void(CollisionContact)> onCollisionStay;
    std::function<void(CollisionContact)> onCollisionExit;
    std::function<void(entt::entity)>     onTriggerEnter;
    std::function<void(entt::entity)>     onTriggerStay;
    std::function<void(entt::entity)>     onTriggerExit;

    uint32_t _bodyId = 0xFFFFFFFFu;  // set by PhysicsSystem; invalid sentinel mirrors RigidBodyComponent
};

