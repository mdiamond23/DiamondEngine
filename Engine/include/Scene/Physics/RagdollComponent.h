#pragma once

#include "RagdollConfig.h"
#include <cstdint>
#include <memory>
#include <string>
#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>

enum class RagdollMode {
    Animated,
    Limp,
    Powered
};

// Public ragdoll state. PhysicsSystem owns the live Jolt bodies and constraints.
struct RagdollComponent {
    std::string assetPath;
    std::shared_ptr<RagdollConfig> config;
    RagdollMode mode = RagdollMode::Animated;

    // Global multiplier for powered joint motor torque.
    float strength = 1.0f;

    // Runtime root-drive command and velocity readback.
    bool locomotionActive = false;
    glm::vec3 locomotionTargetPos { 0.0f }; // Legacy kinematic drive.
    glm::quat locomotionTargetRot { 1.0f, 0.0f, 0.0f, 0.0f };
    glm::vec3 locomotionTargetVel { 0.0f };
    glm::vec3 locomotionSupportTarget { 0.0f };
    glm::vec3 locomotionSupportTargetVel { 0.0f };
    float locomotionSupportTargetWeight = 0.0f;
    float locomotionCOMSupportFreq = 4.0f;
    float locomotionCOMSupportDamping = 1.0f;
    float locomotionCOMSupportMaxAccel = 10.0f;
    // Readback from the explicit COM support-target controller. These remain zero/false
    // when locomotionSupportTargetWeight is zero and let validation distinguish a stable
    // shift from one held together by a permanently saturated force cap.
    glm::vec3 _locomotionSupportForce { 0.0f };
    glm::vec3 _locomotionSupportPositionForce { 0.0f };
    glm::vec3 _locomotionSupportDampingForce { 0.0f };
    bool _locomotionSupportSaturated = false;
    // Small procedural offset from the authored standing hip height. Locomotion uses this
    // for continuous pelvis bob without moving the entity transform or teleporting bodies.
    float locomotionHeightOffset = 0.0f;
    glm::vec3 _locomotionRootVel { 0.0f };
    glm::vec3 _locomotionRootAngularVelocity { 0.0f };
    glm::vec3 _locomotionUprightDeltaAngularVelocity { 0.0f };
    glm::vec3 _locomotionUprightTorque { 0.0f };
    bool _locomotionUprightTorqueActive = false;
    bool _locomotionUprightSaturated = false;
    float _locomotionHeadingErrorDeg = 0.0f;
    bool _locomotionHeadingSaturated = false;
    glm::vec3 _locomotionCOM { 0.0f };
    glm::vec3 _locomotionCOMVel { 0.0f };
    bool _locomotionCOMValid = false;
    int _locomotionFootBones[2] { -1, -1 };
    bool _locomotionFootGrounded[2] { false, false };
    // Solver contact readback for each foot. Unlike _locomotionFootGrounded (a proximity
    // ray used to build a broad support base), this is true only when Jolt reports a real
    // non-sensor contact with an upward-facing support normal during the previous step.
    bool _locomotionFootContact[2] { false, false };
    glm::vec3 _locomotionFootContactNormal[2] { glm::vec3(0.0f), glm::vec3(0.0f) };
    glm::vec3 _locomotionFootContactPoint[2] { glm::vec3(0.0f), glm::vec3(0.0f) };
    // Advances after each fixed physics step refreshes the solver-backed foot contacts.
    // Diagnostics can wait on this instead of assuming one render frame equals one step.
    uint64_t _locomotionPhysicsStepSerial = 0;
    // Positive means actual overlap. Negative is a Jolt speculative contact whose shapes
    // have not touched yet. Together with soleMinY this distinguishes collision slop from
    // a debug-draw/floor-render mismatch.
    float _locomotionFootPenetration[2] { 0.0f, 0.0f };
    float _locomotionFootSoleMinY[2] { 0.0f, 0.0f };
    // World-Y component of the foot collision box's sole normal. A value near +1
    // means the sole is flat and facing upward; locomotion uses this to avoid
    // beginning a gait while a post-recovery foot is still rolled onto an edge.
    float _locomotionFootSoleUpY[2] { 0.0f, 0.0f };

    // Validation-only runtime motor mask. Each entry names the CHILD bone whose joint
    // motor is muted. SyncRagdollPowered reapplies this every substep and restores Position
    // mode as soon as the entry is cleared, so diagnostics cannot leave a stale limp joint.
    int locomotionDisabledMotorBones[4] { -1, -1, -1, -1 };

    // Previous-substep powered-leg motor readback. Jolt reports the angular impulse
    // actually applied by each motor; PhysicsSystem converts it to torque using the fixed
    // step and compares the strongest axis against that joint's configured torque limit.
    // Slots are populated by child bone index so validation code does not depend on joint
    // iteration order. These are diagnostics only and never feed a controller.
    int _locomotionMotorBones[6] { -1, -1, -1, -1, -1, -1 };
    float _locomotionMotorAppliedTorque[6] { 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f };
    float _locomotionMotorTorqueLimit[6] { 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f };
    float _locomotionMotorSaturationRatio[6] { 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f };
    bool _locomotionMotorSaturated[6] { false, false, false, false, false, false };

    float locomotionMoveForce = 400.0f;
    float locomotionBalanceForce = 2500.0f;
    float locomotionMoveLean = 0.12f;
    float locomotionFallenTilt = 55.0f;
    float locomotionUprightGain = 220.0f; // Legacy.
    float locomotionUprightDamp = 30.0f;  // Legacy.
    float locomotionFootBalanceGain = 600.0f; // Reserved; currently unused.
    bool locomotionDebug = false;

    // Dynamic pelvis support.
    bool locomotionDynamicRoot = true;
    float locomotionAccel = 60.0f;
    float locomotionUpAccel = 40.0f;
    float locomotionSupportFreq = 8.0f;

    float locomotionFallLimpSpeed = 7.0f;

    // Upright spring and knockdown thresholds.
    float locomotionBalanceFreq = 40.0f;
    float locomotionBalanceAccel = 16000.0f;
    float locomotionTipLimpDeg = 70.0f;
    float locomotionTipLimpTime = 0.5f;

    // Runtime-selectable solver-backed upright spring. When enabled it REPLACES the
    // angular-velocity spring; the two paths are never applied in the same physics step.
    // A world-to-root SixDOF motor leaves translation free, drives pitch/roll upright,
    // and uses a lower-authority yaw spring to preserve the requested heading.
    bool locomotionTorqueUpright = false;
    float locomotionTorqueUprightStiffness = 700.0f;
    float locomotionTorqueUprightDamping = 100.0f;
    float locomotionTorqueUprightMaxTorque = 250.0f;
    float locomotionTorqueHeadingStiffness = 180.0f;
    float locomotionTorqueHeadingDamping = 50.0f;
    float locomotionTorqueHeadingMaxTorque = 80.0f;

    // SIMBICON gait: the legs move the body, so the pelvis velocity force, the leash and
    // the COM-over-base forces are suppressed (they push the body toward the foot; the
    // controller instead places the foot under the body). The vertical support spring and
    // the upright spring stay -- the latter stands in for the paper's virtual torso torque
    // until tau_A = -tau_torso - tau_B is wired, and fades out via locomotionUprightScale.
    // Set per-frame: true while stepping, false while standing.
    // 0 = legacy engine balance, 1 = full SIMBICON. CROSSFADE, never a switch: flipping it
    // instantly removed every engine assist while the virtual torques were still ramping in,
    // and the character fell ~15 deg through the gap before the gait did anything.
    bool locomotionSimbicon = false;
    float locomotionSimbiconBlend = 0.0f;
    float locomotionUprightScale = 1.0f;

    // SIMBICON virtual torques (paper 3.2). The controller computes these in WORLD space
    // each frame; the engine applies them per substep along with their reaction on the root,
    // so the net torque the pelvis sees is tau_torso. The two hip joint motors are muted
    // while this is active or they would fight the virtual PDs. [0] = swing, [1] = stance.
    int locomotionHipBones[2] { -1, -1 };
    glm::vec3 locomotionHipTorque[2] { glm::vec3(0.0f), glm::vec3(0.0f) };
    // Scales the vertical hip support. 1 keeps the legacy assist, 0 makes the stance leg
    // carry the body as the paper intends.
    float locomotionSupportScale = 1.0f;

    // Assisted gait bring-up: a one-sided vertical world-space spring that unloads the
    // swing sole. Its equal-and-opposite force is applied to the pelvis, so it is an
    // internal leg-lift assist rather than a force that launches the whole character.
    int locomotionLiftBone = -1;
    float locomotionLiftTargetY = 0.0f;
    float locomotionLiftFrequency = 3.5f;
    float locomotionLiftDamping = 1.0f;
    float locomotionLiftEffectiveMass = 8.0f;
    float locomotionLiftMaxForce = 325.0f;
    float _locomotionLiftForce = 0.0f; // last applied force, diagnostic only

    // Physical stance-foot lock. Each active slot applies a capped horizontal spring to
    // the named foot body at the stored world-space plant point. This models static ground
    // friction: translation is resisted, rotation and vertical contact remain solver-owned.
    // Two slots are active during double support; only the stance slot remains during swing.
    int locomotionFootLockBones[2] { -1, -1 };
    glm::vec3 locomotionFootLockTargets[2] { glm::vec3(0.0f), glm::vec3(0.0f) };
    float locomotionFootLockFrequency = 2.5f;
    float locomotionFootLockDamping = 1.0f;
    float locomotionFootLockEffectiveMass = 12.0f;
    float locomotionFootLockMaxForce = 600.0f;
    float locomotionFootLockWeights[2] { 0.0f, 0.0f };
    float _locomotionFootLockError[2] { 0.0f, 0.0f };
    float _locomotionFootLockForce[2] { 0.0f, 0.0f };

    // Base-of-support tuning.
    float locomotionSupportRadius = 0.12f;
    float locomotionSupportFalloff = 0.15f;
    float locomotionLeashDistance = 0.15f;

    uint32_t _ragdollId = 0xFFFFFFFFu;
};
