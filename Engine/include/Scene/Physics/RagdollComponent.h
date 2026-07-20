#pragma once
#include <memory>
#include <string>
#include <cstdint>
#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>
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

    // Locomotion root drive (runtime, not serialized; see DriveRagdollLocomotionRoot).
    // The root/hips body has no parent joint, so joint motors can't move or support
    // it — a controller script sets these targets every frame and PhysicsSystem
    // steers the root toward them each substep. WORLD space; locomotionTargetRot is
    // a world HEADING composed on top of the root's bind orientation (identity =
    // upright, facing as built — bone frames like CesiumMan's Z-up hips are
    // composed away). Powered mode only; yields to hit-react/get-up reactions.
    bool      locomotionActive    = false;
    glm::vec3 locomotionTargetPos { 0.0f };
    glm::quat locomotionTargetRot { 1.0f, 0.0f, 0.0f, 0.0f };

    // Physical drive (default): gravity always applies; support is a one-sided
    // spring that pushes the hips up toward the target height only while a ray
    // finds ground underfoot (standing sag ~= carried-weight-accel / (2*pi*f)^2).
    // Horizontal = accel-capped velocity pursuit, so walls stop the character and
    // heavy objects shove it. False = legacy kinematic hold (infinite force).
    // upAccel is the SUPPORT CAP: at rest it's ~idle (the legs carry the weight to
    // the feet), but on a perturbation that drops/tilts the hips it catches them —
    // too high and it yanks the pelvis up, bypassing the feet (marionette). Kept low
    // so punches/leans resolve THROUGH the legs and feet, which is the whole point of
    // standing on a real base of support. Raise it if the rig sags or buckles.
    bool  locomotionDynamicRoot   = true;
    float locomotionAccel         = 60.0f;    // m/s^2 horizontal budget
    float locomotionUpAccel       = 40.0f;    // m/s^2 support spring force cap (low = feet carry perturbations)
    float locomotionSupportFreq   = 8.0f;     // Hz support stiffness (keep < ~10 at 60 Hz sim)

    // Airborne + falling faster than this -> Limp (tumble -> auto-get-up). 7 m/s
    // needs ~2.5 m of free-fall, so jumps/stair drops never trigger it. 0 = off.
    float locomotionFallLimpSpeed = 7.0f;

    // Balance: implicitly-damped righting spring on the hips toward upright+heading.
    // Freq sets the drunk lean under the leg motors' reaction torque (~20 Hz+ at the
    // 60 Hz sim for a usable lean); accel caps what can beat it outright (impacts),
    // <=0 = hard lock. Tilt (hips up-axis vs world up; yaw is commanded, never counts)
    // held past TipLimpDeg for TipLimpTime -> Limp. 0 deg = never limps, 0 s = instant.
    float locomotionBalanceFreq  = 40.0f;     // Hz righting stiffness
    float locomotionBalanceAccel = 16000.0f;  // rad/s^2 budget (<=0 = hard lock)
    float locomotionTipLimpDeg   = 70.0f;     // tilt that reads as knocked over
    float locomotionTipLimpTime  = 0.5f;      // grace before a held lean limps

    // Stage C torque righting: >0 rights the hips with a real torque (reacts through the
    // planted feet, so a deep lean recovers) instead of the velocity spring above (which
    // ground contact absorbs). N*m/rad of error; damp in N*m*s. 0 = velocity spring.
    float locomotionBalanceTorque     = 0.0f;
    float locomotionBalanceTorqueDamp = 40.0f;

    // Runtime handle into PhysicsSystem's ragdoll table; invalid sentinel mirrors
    // RigidBodyComponent::_bodyId. Set by PhysicsSystem at build, never serialized.
    uint32_t _ragdollId = 0xFFFFFFFFu;
};
