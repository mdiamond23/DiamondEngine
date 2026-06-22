#pragma once
#include <glm/glm.hpp>
#include <string>
#include <vector>
#include "Constraint.h"   // reuse ConstraintType (Hinge / SwingTwist / ...)

// Ragdoll configuration — the data model loaded from a .ragdoll asset and used by
// PhysicsSystem to build a chain of rigid bodies + joints for a skinned character.
// Plain data only — no Jolt, no JSON — mirroring RigidBodyComponent / ConstraintComponent
// so the engine's public headers stay dependency-free. JSON load/save lives in
// Sandbox (RagdollAsset.h), same split as AnimationStateMachine.h / AnimStateMachineAsset.h.
//
// A ragdoll maps a SUBSET of a skeleton's bones onto physics bodies (the simplified
// ~15-bone physics skeleton). Bones with no body still skin off their nearest
// physics-driven ancestor — see the readback in Docs/ragdoll-design.md.

namespace Diamond { struct Skeleton; }

// One physics body in the ragdoll, bound to a skeleton bone by name.
struct RagdollBodyDef {
    std::string boneName;          // skeleton bone this body drives
    std::string parentBoneName;    // bone of the parent BODY ("" = simulation root, the hips)

    // --- collision shape (authored per-bone; auto-gen seeds a default) ---------
    enum class Shape { Capsule, Box, Sphere };
    Shape     shape       = Shape::Capsule;
    float     radius      = 0.05f;            // capsule / sphere
    float     halfHeight  = 0.10f;            // capsule cylinder half-length (excludes the hemispheres)
    glm::vec3 halfExtents { 0.08f, 0.08f, 0.08f }; // box

    float     mass        = 1.0f;             // kilograms

    // --- joint to the parent body (ignored for the root) -----------------------
    // Maps directly onto ConstraintComponent fields at build time. SwingTwist is
    // the ragdoll workhorse (shoulders/hips/spine); Hinge is for elbows/knees.
    ConstraintType jointType = ConstraintType::SwingTwist;

    // Twist axis along the bone, in the bone's LOCAL frame (auto-gen points it at
    // the child bone). PhysicsSystem rotates this into world space at build time.
    glm::vec3 twistAxisLocal { 0.0f, 1.0f, 0.0f };

    // SwingTwist limits, degrees (elliptical swing cone + twist range).
    float swingNormalDeg = 30.0f;
    float swingPlaneDeg  = 30.0f;
    float twistMinDeg    = -15.0f;
    float twistMaxDeg    =  15.0f;

    // Hinge limits, degrees (min in [-180,0], max in [0,180]).
    float hingeMinDeg = 0.0f;
    float hingeMaxDeg = 150.0f;

    // --- powered/active-ragdoll motor (per bone) -------------------------------
    // The "muscle" driving this joint toward the animation pose in RagdollMode::Powered.
    // Motors are baked on every ragdoll joint at build (Position mode) and only
    // engaged in Powered mode; Animated (kinematic) and Limp (motors off) ignore them.
    // motorMaxTorque is the budget in N·m — heavier limbs / stronger characters want
    // more. frequency/damping shape the servo spring (Hz / damping ratio).
    float motorMaxTorque = 50.0f;
    float motorFrequency = 8.0f;
    float motorDamping   = 1.0f;
};

// The whole ragdoll: an ordered list of bodies (roots first is convenient but not
// required — PhysicsSystem resolves parents by name) plus tuning shared by the rig.
struct RagdollConfig {
    std::vector<RagdollBodyDef> bodies;
    float impactThreshold = 0.0f;   // reserved for auto-on-impact (fast-follow, not v1)
};

// Best-effort auto-generation from a skeleton: classifies bones by topology + role
// hints, sizes shapes from the bind pose, and picks joint types by chain depth.
// The result is meant to be saved as an editable .ragdoll asset and hand-tuned —
// it seeds the asset, it is not authoritative. See Docs/ragdoll-design.md.
RagdollConfig BuildDefaultRagdoll(const Diamond::Skeleton& skeleton);
