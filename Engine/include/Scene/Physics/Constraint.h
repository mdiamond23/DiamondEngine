#pragma once
#include <glm/glm.hpp>
#include <cstdint>

// A physics joint connecting this entity's body to another body (or the static
// world). Plain data only — no Jolt types — mirroring RigidBodyComponent so the
// engine's public headers stay free of the Jolt include path. The live
// JPH::Constraint is owned by PhysicsSystem and referenced here by _constraintId.
//
// Skeleton scope: a single hinge attached to the immovable world. Entity-to-
// entity targeting, limits, motors, and more constraint types arrive in later
// steps — the fields below are intentionally the minimum to stand one up.

enum class ConstraintType { Hinge };

struct ConstraintComponent {
    ConstraintType type = ConstraintType::Hinge;

    // The other body. 0 = attach to the immovable world (Body::sFixedToWorld).
    // Entity-to-entity targeting (resolved via IDComponent UUID) is a later step.
    uint64_t targetUuid = 0;

    // Anchor point and axis in WORLD space, as authored in the editor.
    // PhysicsSystem converts these to per-body local frames at creation time via
    // Jolt's WorldSpace constraint mode. Default axis is horizontal (Z) so a
    // default hinge swings under gravity like a pendulum.
    glm::vec3 anchor { 0.0f, 0.0f, 0.0f };
    glm::vec3 axis   { 0.0f, 0.0f, 1.0f };

    // Optional angular limits, in DEGREES (converted to radians at creation).
    // When off, the hinge spins freely. The zero angle is the bodies' relative
    // orientation at creation; Jolt requires min in [-180, 0] and max in [0, 180].
    bool  hasLimits = false;
    float limitMin  = -90.0f;
    float limitMax  =  90.0f;

    // Runtime handle into PhysicsSystem's constraint table; invalid sentinel
    // mirrors RigidBodyComponent::_bodyId. Set by PhysicsSystem, never serialized.
    uint32_t _constraintId = 0xFFFFFFFFu;
};