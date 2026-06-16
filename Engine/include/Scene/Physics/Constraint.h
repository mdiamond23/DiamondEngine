#pragma once
#include <glm/glm.hpp>
#include <cstdint>

// A physics joint connecting this entity's body to another body (or the static
// world). Plain data only — no Jolt types — mirroring RigidBodyComponent so the
// engine's public headers stay free of the Jolt include path. The live
// JPH::Constraint is owned by PhysicsSystem and referenced here by _constraintId.
//
// Hinge = 1-DOF rotation (doors, elbows, knees). SwingTwist = a humanoid ball
// joint with an elliptical swing cone + twist limit (shoulders, hips, neck) —
// the workhorse joint for ragdolls. More types arrive in later steps.

enum class ConstraintType { Hinge, SwingTwist, Fixed };

// Motor drive mode. Off = passive joint. Velocity = spin toward a target angular
// velocity (powered wheel). Position = drive toward a target angle like a
// servo/spring (self-closing door, muscle). Applied to the LIVE constraint after
// creation — unlike limits, which are creation-time only.
enum class MotorMode { Off, Velocity, Position };

struct ConstraintComponent {
    ConstraintType type = ConstraintType::Hinge;

    // The other body. 0 = attach to the immovable world (Body::sFixedToWorld).
    // Non-zero = another entity, by persistent IDComponent UUID (survives save/load).
    uint64_t targetUuid = 0;

    // Anchor point and primary axis in WORLD space, as authored in the editor.
    // PhysicsSystem converts these to per-body local frames at creation via Jolt's
    // WorldSpace mode. For Hinge, `axis` is the rotation axis; for SwingTwist it is
    // the twist axis (along the bone). Default axis is horizontal (Z).
    glm::vec3 anchor { 0.0f, 0.0f, 0.0f };
    glm::vec3 axis   { 0.0f, 0.0f, 1.0f };

    // --- Hinge limits ------------------------------------------------------
    // Optional angular limits, in DEGREES (converted to radians at creation).
    // When off, the hinge spins freely. The zero angle is the bodies' relative
    // orientation at creation; Jolt requires min in [-180, 0] and max in [0, 180].
    bool  hasLimits = false;
    float limitMin  = -90.0f;
    float limitMax  =  90.0f;

    // --- SwingTwist limits (ragdoll ball joint) ----------------------------
    // Swing is an elliptical cone around the twist axis; twist is rotation about
    // it. All in DEGREES. Half-cone angles in [0,180]; twist min in [-180,0],
    // max in [0,180].
    float swingNormalDeg = 45.0f;  // swing half-angle around the normal axis
    float swingPlaneDeg  = 45.0f;  // swing half-angle around the plane axis
    float twistMinDeg    = -45.0f;
    float twistMaxDeg    =  45.0f;

    // --- Motor (hinge only for now) ----------------------------------------
    // motorMaxTorque (N·m) is the budget — if the load needs more, the motor
    // stalls. Target is degrees (Position) or deg/s (Velocity). Frequency/damping
    // shape the position-mode spring.
    MotorMode motorMode      = MotorMode::Off;
    float     motorTarget    = 0.0f;
    float     motorMaxTorque = 50.0f;
    float     motorFrequency = 2.0f;   // position-mode spring stiffness (Hz)
    float     motorDamping   = 1.0f;   // position-mode spring damping ratio

    glm::vec3 motorTargetEuler { 0.0f, 0.0f, 0.0f }; // For swing and twist motor only

    // Runtime handle into PhysicsSystem's constraint table; invalid sentinel
    // mirrors RigidBodyComponent::_bodyId. Set by PhysicsSystem, never serialized.
    uint32_t _constraintId = 0xFFFFFFFFu;
};