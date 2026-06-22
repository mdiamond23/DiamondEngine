#pragma once
#include <memory>
#include <string>
#include <cstdint>
#include "RagdollConfig.h"

// Runtime ragdoll on a skinned-character entity. Plain data — no Jolt — like
// RigidBodyComponent / ConstraintComponent: the live physics bodies + joints are
// owned internally by PhysicsSystem and referenced here by _ragdollId. The config
// (RagdollConfig) is loaded from the .ragdoll asset by the editor, exactly the way
// AnimStateMachineComponent::machine holds its loaded .animsm asset.
//
// v1 is a passive (limp) ragdoll: bodies are built Kinematic and follow the
// animation each frame, then flip to Dynamic to flop (RagdollMode::Limp). Powered
// (motor-driven, fights toward an animation pose) is v2 — the motor primitives
// already exist (Physics::SetMotorTargetOrientation). See Docs/ragdoll-design.md.

enum class RagdollMode {
    Animated,   // kinematic bodies shadow the animation; skin = animation palette
    Limp,       // bodies are dynamic and flopping; skin = physics readback
    Powered     // bodies are dynamic but joint motors fight toward the animation pose
                // (active ragdoll); skin = physics readback, same as Limp
};

struct RagdollComponent {
    std::string                    assetPath;   // the .ragdoll file (for serialization)
    std::shared_ptr<RagdollConfig> config;      // loaded by the editor (may be null until loaded)
    RagdollMode                    mode = RagdollMode::Animated;

    // Powered-mode muscle strength in [0,1]: scales every joint motor's torque
    // budget each frame (1 = full authored strength, 0 = effectively limp). The
    // knob for hit-reactions, get-up blends and partial give. Ignored unless
    // mode == Powered. See Physics::SetRagdollStrength.
    float strength = 1.0f;

    // Runtime handle into PhysicsSystem's ragdoll table; invalid sentinel mirrors
    // RigidBodyComponent::_bodyId. Set by PhysicsSystem at build, never serialized.
    uint32_t _ragdollId = 0xFFFFFFFFu;
};
