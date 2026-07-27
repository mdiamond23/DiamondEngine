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
    glm::vec3 _locomotionRootVel { 0.0f };
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

    // Optional physical torque balance; zero uses the velocity spring.
    float locomotionBalanceTorque = 0.0f;
    float locomotionBalanceTorqueDamp = 40.0f;

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

    // Base-of-support tuning.
    float locomotionSupportRadius = 0.12f;
    float locomotionSupportFalloff = 0.15f;
    float locomotionLeashDistance = 0.15f;

    uint32_t _ragdollId = 0xFFFFFFFFu;
};
