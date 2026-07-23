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

    // Base-of-support tuning.
    float locomotionSupportRadius = 0.12f;
    float locomotionSupportFalloff = 0.15f;
    float locomotionLeashDistance = 0.15f;

    uint32_t _ragdollId = 0xFFFFFFFFu;
};
