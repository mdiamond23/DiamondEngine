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

    // --- two-tier auto hit-react (impact momentum in kg·m/s) -------------------
    // See AutoTriggerRagdollImpacts. impactThreshold is the KNOCKDOWN tier: a hit
    // at/above it flips the rig fully Limp (true flop). flinchThreshold is the
    // lighter STAGGER tier: a hit in [flinchThreshold, impactThreshold) doesn't
    // floor the character — it switches to Powered, drops muscle strength to
    // flinchStrength (plus a directional impulse on the struck bone), then ramps
    // strength back to 1 over flinchRecovery seconds and settles back to Animated.
    // 0 disables that tier; flinchThreshold should be < impactThreshold to matter.
    float impactThreshold = 0.0f;   // knockdown -> Limp           (0 = off)
    float flinchThreshold = 0.0f;   // stagger   -> Powered flinch (0 = off)
    float flinchStrength  = 0.3f;   // [0,1] muscle strength at the bottom of the dip
    float flinchRecovery  = 0.4f;   // seconds to ramp strength back to 1

    // --- procedural get-up (active-ragdoll recovery, no animation clip) ---------
    // After a knockdown, the rig heaves itself back upright: Limp -> Powered driving
    // the joint motors toward the bind/stand pose (cones opened so sprawled limbs can
    // straighten), strength ramping 0 -> getupStrength over getupDuration with
    // getupWobble flail noise, then handing back to animation control. Deliberately
    // under-powered + noisy = the sloppy bar-fight stagger-up. See RagdollGetUp.
    float getupStrength = 0.7f;     // [0,1] peak muscle strength while rising (<1 = wobbly)
    float getupDuration = 1.2f;     // seconds to heave from sprawled to standing
    float getupWobble   = 0.3f;     // per-joint torque noise amplitude (0 = clean, ~0.3 = drunk)
    float getupDelay    = 0.6f;     // seconds the limp rig must be ~at rest before auto get-up; 0 = manual only

    // Root assist gain: joint motors can't lift the body's root (no joint acts on it), so
    // get-up directly drives the pelvis upright + up to standing height. This scales that
    // assist. 1 = normal, higher = snappier/stronger stand, 0 = pure motors (won't rise).
    float getupBalance  = 1.0f;

    // After the heave reaches the top, hold the (kinematic-root) stand this many seconds,
    // then hand back to Animated so the character resumes its animation clip. The motors
    // alone can't balance once the kinematic root is released, so the handoff goes to
    // Animated (kinematic follow), not Powered. 0 = hold the stand indefinitely (manual
    // hand-off via the inspector / gameplay), preserving the old behavior.
    float getupHold     = 0.5f;

    // Cross-blend duration for the hand-off: the bodies ease (kinematically) from the held
    // stand pose into the live animation pose over this many seconds before switching to
    // Animated, so resuming the clip doesn't pop. 0 = instant switch (no blend).
    float getupBlend    = 0.25f;
};

// Best-effort auto-generation from a skeleton: classifies bones by topology + role
// hints, sizes shapes from the bind pose, and picks joint types by chain depth.
// The result is meant to be saved as an editable .ragdoll asset and hand-tuned —
// it seeds the asset, it is not authoritative. See Docs/ragdoll-design.md.
RagdollConfig BuildDefaultRagdoll(const Diamond::Skeleton& skeleton);
