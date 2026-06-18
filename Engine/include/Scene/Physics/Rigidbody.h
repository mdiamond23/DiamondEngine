#pragma once
#include <glm/glm.hpp>
#include <cstdint>

enum class BodyType { Static, Dynamic, Kinematic };

struct RigidBodyComponent {
    BodyType bodyType       = BodyType::Static;
    float    mass           = 1.0f;
    float    linearDamping  = 0.05f;
    float    angularDamping = 0.05f;
    float    gravityScale   = 1.0f;
    bool     lockRotX       = false;
    bool     lockRotY       = false;
    bool     lockRotZ       = false;

    // Continuous collision detection (CCD). When true, the body sweeps its shape
    // along its motion each step (Jolt LinearCast motion quality) and stops at the
    // first hit, instead of only testing overlap at the step's end position. Enable
    // for fast, small, or thin movers (projectiles, a swung limb) that would
    // otherwise tunnel through thin geometry. Slight extra cost — off by default.
    bool     continuousCollision = false;

    // Internal — managed by PhysicsSystem, do not set manually
    // 0xFFFFFFFF is JPH::BodyID::cInvalidBodyID; stored as uint32_t to keep Jolt out of public headers
    uint32_t _bodyId = 0xFFFFFFFFu;
};