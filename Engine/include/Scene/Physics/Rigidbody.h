#pragma once
#include <glm/glm.hpp>
#include <entt/entt.hpp>
#include <functional>
#include <cstdint>

enum class BodyType { Static, Dynamic, Kinematic };

struct CollisionContact {
    entt::entity other;
    glm::vec3    contactPoint;
    glm::vec3    contactNormal;
};

struct RigidBodyComponent {
    BodyType bodyType       = BodyType::Static;
    float    mass           = 1.0f;
    float    linearDamping  = 0.05f;
    float    angularDamping = 0.05f;
    float    gravityScale   = 1.0f;
    bool     lockRotX       = false;
    bool     lockRotY       = false;
    bool     lockRotZ       = false;

    // Collision callbacks — set by scripts, fired by PhysicsSystem
    std::function<void(CollisionContact)> onCollisionEnter;
    std::function<void(CollisionContact)> onCollisionStay;
    std::function<void(CollisionContact)> onCollisionExit;
    std::function<void(entt::entity)>     onTriggerEnter;
    std::function<void(entt::entity)>     onTriggerExit;

    // Internal — managed by PhysicsSystem, do not set manually
    // 0xFFFFFFFF is JPH::BodyID::cInvalidBodyID; stored as uint32_t to keep Jolt out of public headers
    uint32_t _bodyId = 0xFFFFFFFFu;
};