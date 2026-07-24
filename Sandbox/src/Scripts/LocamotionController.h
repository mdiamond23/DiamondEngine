#pragma once

#include "Scene/Scripting.h"
#include "Scene/Scene.h"
#include "Scene/ComponentRegistry.h"
#include "Scene/Components.h"
#include "Core/Input.h"
#include "Scene/Physics/PhysicsAPI.h"
#include "Scene/Physics/RagdollComponent.h"
#include "Animation/AnimationComponents.h"
#include "Animation/IKSolver.h"
#include "DebugDraw.h"
#include "CameraDirector.h"

#include <imgui.h>
#include <nlohmann/json.hpp>
#include <glm/glm.hpp>
#include <glm/gtc/constants.hpp>
#include <glm/gtc/quaternion.hpp>
#include <spdlog/spdlog.h>
#include <cmath>
#include <limits>
#include <string>

// Minimal active-ragdoll gait. The engine owns pelvis support and balance; this
// controller alternates fixed foot placements and poses the hip-knee-ankle chain.
struct LocamotionControllerComponent
{
    float maxSpeed        = 1.0f;
    float moveRampTime    = 0.75f;
    float turnRate        = 10.0f;
    float deadzone        = 0.2f;
    float facingOffsetDeg = 0.0f;
    float groundRayLength = 1.2f;
    float consciousness   = 1.0f;

    std::string leftFootBone  = "leg_joint_L_5";
    std::string rightFootBone = "leg_joint_R_5";

    float minWalkSpeed    = 0.15f;
    float stepLength      = 0.35f;
    float stanceHalfWidth = 0.10f;
    float stepDuration    = 0.40f;
    float stepLift        = 0.10f;
    float doubleSupportTime = 0.08f;
    float weightShiftTime = 0.20f;
    float firstWeightShiftTime = 0.25f;
    float weightShiftTimeout = 0.40f;
    float weightShiftRadius = 0.08f;
    float weightShiftStableTime = 0.05f;
    float weightShiftDriveScale = 0.15f;
    float weightShiftInboard = 0.20f;
    float weightShiftTargetTime = 0.15f;
    float toeOffFraction = 0.20f;
    float toeOffHeight = 0.03f;
    float swingSupportFadeEnd = 0.45f;
    float postPoweredGrace  = 0.4f;
    float gaitBlendTime     = 0.2f;
    float maxReachFraction  = 0.95f;
    float maxSwingTravelFraction = 0.55f;
    float maxHipSpeedDeg = 140.0f;
    float maxKneeSpeedDeg = 220.0f;
    float maxFootSpeedLegsPerSec = 1.25f;
    float maxStepDuration = 1.0f;
    float hipLimitMarginDeg = 3.0f;
    float swingPoseWeight   = 0.35f;
    float stancePoseWeight  = 0.45f;
    float landingTolerance  = 0.10f;

    bool ikWriteEnabled = true;
    bool debug = false;

    enum class GaitPhase { Idle, WeightShift, Swing, DoubleSupport };
    enum class LegPhase { Planted, Swinging };
    struct LegState {
        int footIdx  = -1;
        int ankleIdx = -1;
        int kneeIdx  = -1;
        int hipIdx   = -1;
        glm::vec3 kneeHingeAxis { 1.0f, 0.0f, 0.0f };
        glm::vec3 hipTwistAxis { 1.0f, 0.0f, 0.0f };
        float hipSwingNormalDeg = 30.0f;
        float hipSwingPlaneDeg = 30.0f;
        float hipTwistMinDeg = -15.0f;
        float hipTwistMaxDeg = 15.0f;
        LegPhase phase = LegPhase::Planted;
        glm::vec3 plantSole       { 0.0f };
        glm::vec3 swingStartSole  { 0.0f };
        glm::vec3 swingTargetSole { 0.0f };
        glm::vec3 swingDirection  { 0.0f };
        glm::vec3 soleTarget      { 0.0f };
        float soleHeight = 0.03f;
        float swingProgress = 0.0f;
        float swingPoseBlend = 0.0f;
        float swingLiftScale = 1.0f;
        float swingPoleDeg = 0.0f;
        float swingDuration = 0.40f;
        float selectedPoleDeg = 0.0f;
        float hipLimitViolationDeg = 0.0f;
        bool hipTargetClamped = false;
        float landingWait = 0.0f;
        bool landingWarning = false;
        bool init = false;
    };

    float _yaw = 0.0f;
    float _timeSincePowered = -1.0f;
    float _gaitWeight = 0.0f;
    float _driveWeight = 0.0f;
    float _timeSinceLanding = 0.0f;
    float _phaseTime = 0.0f;
    float _weightShiftStableTime = 0.0f;
    bool _grounded = true;
    bool _wasWalking = false;
    bool _firstStep = true;
    bool _weightShiftTargetCaptured = false;
    int _landedStepCount = 0;
    bool _nextLeft = true;
    GaitPhase _gaitPhase = GaitPhase::Idle;
    glm::vec3 _weightShiftStartTarget { 0.0f };
    glm::vec3 _swingSupportTarget { 0.0f };
    LegState _legL, _legR;
};

inline const char* LocomotionGaitPhaseName(LocamotionControllerComponent::GaitPhase phase)
{
    using Phase = LocamotionControllerComponent::GaitPhase;
    switch (phase) {
        case Phase::Idle:          return "Idle";
        case Phase::WeightShift:   return "Weight Shift";
        case Phase::Swing:         return "Swing";
        case Phase::DoubleSupport: return "Double Support";
    }
    return "Unknown";
}

template<>
inline void DrawComponentInspector<LocamotionControllerComponent>(LocamotionControllerComponent& c)
{
    ImGui::DragFloat("Max Speed", &c.maxSpeed, 0.05f, 0.0f, 10.0f);
    ImGui::DragFloat("Move Ramp Time", &c.moveRampTime, 0.01f, 0.01f, 2.0f);
    ImGui::DragFloat("Turn Rate", &c.turnRate, 0.1f, 0.0f, 30.0f);
    ImGui::DragFloat("Deadzone", &c.deadzone, 0.01f, 0.0f, 0.9f);
    ImGui::DragFloat("Facing Offset", &c.facingOffsetDeg, 1.0f, -180.0f, 180.0f, "%.0f deg");
    ImGui::DragFloat("Ground Ray", &c.groundRayLength, 0.05f, 0.1f, 10.0f);
    ImGui::DragFloat("Consciousness", &c.consciousness, 0.01f, 0.0f, 1.0f);

    auto boneField = [](const char* label, std::string& name) {
        char buf[128];
        std::snprintf(buf, sizeof(buf), "%s", name.c_str());
        if (ImGui::InputText(label, buf, sizeof(buf))) name = buf;
    };
    ImGui::Separator();
    boneField("Left Foot Bone", c.leftFootBone);
    boneField("Right Foot Bone", c.rightFootBone);

    ImGui::Separator();
    ImGui::DragFloat("Min Walk Speed", &c.minWalkSpeed, 0.01f, 0.0f, 2.0f);
    ImGui::DragFloat("Step Length", &c.stepLength, 0.01f, 0.05f, 0.6f);
    ImGui::DragFloat("Stance Half Width", &c.stanceHalfWidth, 0.005f, 0.02f, 0.4f);
    ImGui::DragFloat("Step Duration", &c.stepDuration, 0.01f, 0.08f, 1.0f);
    ImGui::DragFloat("Step Lift", &c.stepLift, 0.005f, 0.0f, 0.3f);
    ImGui::DragFloat("Double Support Time", &c.doubleSupportTime, 0.01f, 0.0f, 0.5f);
    ImGui::DragFloat("Weight Shift Time", &c.weightShiftTime, 0.01f, 0.0f, 1.0f);
    ImGui::DragFloat("First Weight Shift", &c.firstWeightShiftTime, 0.01f, 0.0f, 1.0f);
    ImGui::DragFloat("Weight Shift Timeout", &c.weightShiftTimeout, 0.01f, 0.01f, 2.0f);
    ImGui::DragFloat("Weight Shift Radius", &c.weightShiftRadius, 0.005f, 0.01f, 0.5f);
    ImGui::DragFloat("Weight Shift Stable", &c.weightShiftStableTime, 0.01f, 0.0f, 0.5f);
    ImGui::DragFloat("Weight Shift Drive", &c.weightShiftDriveScale, 0.01f, 0.0f, 1.0f);
    ImGui::SliderFloat("Weight Shift Inboard", &c.weightShiftInboard, 0.0f, 0.5f);
    ImGui::DragFloat("Weight Shift Target Time", &c.weightShiftTargetTime, 0.01f, 0.01f, 1.0f);
    ImGui::SliderFloat("Toe-Off Fraction", &c.toeOffFraction, 0.05f, 0.45f);
    ImGui::DragFloat("Toe-Off Height", &c.toeOffHeight, 0.005f, 0.0f, 0.15f);
    ImGui::SliderFloat("Swing Support Fade End", &c.swingSupportFadeEnd, 0.10f, 0.80f);
    ImGui::DragFloat("Post-Powered Grace", &c.postPoweredGrace, 0.01f, 0.0f, 2.0f);
    ImGui::DragFloat("Gait Blend Time", &c.gaitBlendTime, 0.01f, 0.01f, 1.0f);
    ImGui::DragFloat("Max Reach Fraction", &c.maxReachFraction, 0.01f, 0.5f, 0.99f);
    ImGui::SliderFloat("Max Swing Travel", &c.maxSwingTravelFraction, 0.20f, 1.0f,
                       "%.2f leg lengths");
    ImGui::DragFloat("Max Hip Speed", &c.maxHipSpeedDeg, 5.0f, 30.0f, 720.0f, "%.0f deg/s");
    ImGui::DragFloat("Max Knee Speed", &c.maxKneeSpeedDeg, 5.0f, 30.0f, 720.0f, "%.0f deg/s");
    ImGui::DragFloat("Max Foot Speed", &c.maxFootSpeedLegsPerSec, 0.05f, 0.2f, 5.0f,
                     "%.2f legs/s");
    ImGui::DragFloat("Max Step Duration", &c.maxStepDuration, 0.05f, 0.1f, 2.0f);
    ImGui::DragFloat("Hip Limit Margin", &c.hipLimitMarginDeg, 0.5f, 0.0f, 20.0f, "%.1f deg");
    ImGui::DragFloat("Swing Pose Weight", &c.swingPoseWeight, 0.01f, 0.0f, 1.0f);
    ImGui::DragFloat("Stance Pose Weight", &c.stancePoseWeight, 0.01f, 0.0f, 1.0f);
    ImGui::DragFloat("Landing Tolerance", &c.landingTolerance, 0.01f, 0.02f, 0.3f);
    ImGui::Checkbox("IK Write Enabled", &c.ikWriteEnabled);
    ImGui::Checkbox("Debug", &c.debug);
    ImGui::TextDisabled(c._grounded ? "Grounded" : "Airborne");
    ImGui::TextDisabled("Gait: %s", LocomotionGaitPhaseName(c._gaitPhase));
}

template<>
inline std::string SerializeComponent<LocamotionControllerComponent>(const LocamotionControllerComponent& c)
{
    nlohmann::json j;
    j["maxSpeed"] = c.maxSpeed;
    j["moveRampTime"] = c.moveRampTime;
    j["turnRate"] = c.turnRate;
    j["deadzone"] = c.deadzone;
    j["facingOffsetDeg"] = c.facingOffsetDeg;
    j["groundRayLength"] = c.groundRayLength;
    j["consciousness"] = c.consciousness;
    j["leftFootBone"] = c.leftFootBone;
    j["rightFootBone"] = c.rightFootBone;
    j["minWalkSpeed"] = c.minWalkSpeed;
    j["stepLength"] = c.stepLength;
    j["stanceHalfWidth"] = c.stanceHalfWidth;
    j["stepDuration"] = c.stepDuration;
    j["stepLift"] = c.stepLift;
    j["doubleSupportTime"] = c.doubleSupportTime;
    j["weightShiftTime"] = c.weightShiftTime;
    j["firstWeightShiftTime"] = c.firstWeightShiftTime;
    j["weightShiftTimeout"] = c.weightShiftTimeout;
    j["weightShiftRadius"] = c.weightShiftRadius;
    j["weightShiftStableTime"] = c.weightShiftStableTime;
    j["weightShiftDriveScale"] = c.weightShiftDriveScale;
    j["weightShiftInboard"] = c.weightShiftInboard;
    j["weightShiftTargetTime"] = c.weightShiftTargetTime;
    j["toeOffFraction"] = c.toeOffFraction;
    j["toeOffHeight"] = c.toeOffHeight;
    j["swingSupportFadeEnd"] = c.swingSupportFadeEnd;
    j["postPoweredGrace"] = c.postPoweredGrace;
    j["gaitBlendTime"] = c.gaitBlendTime;
    j["maxReachFraction"] = c.maxReachFraction;
    j["maxSwingTravelFraction"] = c.maxSwingTravelFraction;
    j["maxHipSpeedDeg"] = c.maxHipSpeedDeg;
    j["maxKneeSpeedDeg"] = c.maxKneeSpeedDeg;
    j["maxFootSpeedLegsPerSec"] = c.maxFootSpeedLegsPerSec;
    j["maxStepDuration"] = c.maxStepDuration;
    j["hipLimitMarginDeg"] = c.hipLimitMarginDeg;
    j["swingPoseWeight"] = c.swingPoseWeight;
    j["stancePoseWeight"] = c.stancePoseWeight;
    j["landingTolerance"] = c.landingTolerance;
    j["ikWriteEnabled"] = c.ikWriteEnabled;
    j["debug"] = c.debug;
    return j.dump();
}

template<>
inline void DeserializeComponent<LocamotionControllerComponent>(LocamotionControllerComponent& c,
                                                                 const std::string& data)
{
    const auto j = nlohmann::json::parse(data);
    c.maxSpeed = j.value("maxSpeed", 1.0f);
    c.moveRampTime = j.value("moveRampTime", 0.75f);
    c.turnRate = j.value("turnRate", 10.0f);
    c.deadzone = j.value("deadzone", 0.2f);
    c.facingOffsetDeg = j.value("facingOffsetDeg", 0.0f);
    c.groundRayLength = j.value("groundRayLength", 1.2f);
    c.consciousness = j.value("consciousness", 1.0f);
    c.leftFootBone = j.value("leftFootBone", std::string("leg_joint_L_5"));
    c.rightFootBone = j.value("rightFootBone", std::string("leg_joint_R_5"));
    c.minWalkSpeed = j.value("minWalkSpeed", 0.15f);
    c.stepLength = j.value("stepLength", j.value("strideLength", 0.35f));
    c.stanceHalfWidth = j.value("stanceHalfWidth", 0.10f);
    c.stepDuration = j.value("stepDuration", 0.40f);
    c.stepLift = j.value("stepLift", 0.10f);
    c.doubleSupportTime = j.value("doubleSupportTime", 0.08f);
    c.weightShiftTime = j.value("weightShiftTime", 0.20f);
    c.firstWeightShiftTime = j.value("firstWeightShiftTime", 0.25f);
    c.weightShiftTimeout = j.value("weightShiftTimeout", 0.40f);
    c.weightShiftRadius = j.value("weightShiftRadius", 0.08f);
    c.weightShiftStableTime = j.value("weightShiftStableTime", 0.05f);
    c.weightShiftDriveScale = j.value("weightShiftDriveScale", 0.15f);
    c.weightShiftInboard = j.value("weightShiftInboard", 0.20f);
    c.weightShiftTargetTime = j.value("weightShiftTargetTime", 0.15f);
    c.toeOffFraction = j.value("toeOffFraction", 0.20f);
    c.toeOffHeight = j.value("toeOffHeight", 0.03f);
    c.swingSupportFadeEnd = j.value("swingSupportFadeEnd", 0.45f);
    c.postPoweredGrace = j.value("postPoweredGrace", 0.4f);
    c.gaitBlendTime = j.value("gaitBlendTime", 0.2f);
    c.maxReachFraction = j.value("maxReachFraction", 0.95f);
    c.maxSwingTravelFraction = j.value("maxSwingTravelFraction", 0.55f);
    c.maxHipSpeedDeg = j.value("maxHipSpeedDeg", 140.0f);
    c.maxKneeSpeedDeg = j.value("maxKneeSpeedDeg", 220.0f);
    c.maxFootSpeedLegsPerSec = j.value("maxFootSpeedLegsPerSec", 1.25f);
    c.maxStepDuration = j.value("maxStepDuration", 1.0f);
    c.hipLimitMarginDeg = j.value("hipLimitMarginDeg", 3.0f);
    c.swingPoseWeight = j.value("swingPoseWeight", 0.35f);
    c.stancePoseWeight = j.value("stancePoseWeight", 0.45f);
    c.landingTolerance = j.value("landingTolerance", 0.10f);
    c.ikWriteEnabled = j.value("ikWriteEnabled", true);
    c.debug = j.value("debug", false);
}

DECLARE_COMPONENT(LocamotionControllerComponent, "LocamotionController")

class LocamotionControllerSystem : public GameSystem
{
    DECLARE_SYSTEM(LocamotionControllerSystem, 100)

public:
    void OnStart(Scene&) override
    {
        // Input::BindAxis("MoveX", GamepadAxis::LeftX);
        // Input::BindAxis("MoveY", GamepadAxis::LeftY);
        Input::BindAxis("MoveX", Key::D, Key::A);
        Input::BindAxis("MoveY", Key::S, Key::W);
    }

    void OnUpdate(Scene& scene, float dt) override
    {
        const float h = Input::GetAxis("MoveX");
        const float f = -Input::GetAxis("MoveY");
        const float magnitude = glm::min(glm::length(glm::vec2(h, f)), 1.0f);

        glm::vec3 camRight, camForward;
        CameraRelativeBasis(scene, camRight, camForward);

        for (auto [entity, comp] : scene.View<LocamotionControllerComponent>().each()) {
            if (!scene.Has<RagdollComponent>(entity) || !scene.Has<TransformComponent>(entity)) continue;
            auto& rag = scene.Get<RagdollComponent>(entity);
            auto& xf = scene.Get<TransformComponent>(entity);
            rag.locomotionSupportTargetWeight = 0.0f;
            rag.locomotionSupportTargetVel = glm::vec3(0.0f);

            if (rag.mode == RagdollMode::Animated) {
                Physics::SetRagdollMode(rag, RagdollMode::Powered);
                comp._timeSincePowered = 0.0f;
                if (comp.debug) spdlog::info("[Loco] Powered mode engaged");
            }
            if (rag.mode != RagdollMode::Powered) {
                rag.locomotionActive = false;
                ResetGait(comp);
                continue;
            }

            comp._timeSincePowered += dt;
            rag.strength = glm::clamp(comp.consciousness, 0.0f, 1.0f);

            const float speed = magnitude > comp.deadzone
                ? (magnitude - comp.deadzone) / (1.0f - comp.deadzone) * comp.maxSpeed
                : 0.0f;
            glm::vec3 moveDir = camRight * h + camForward * f;
            moveDir.y = 0.0f;
            if (glm::length(moveDir) > 1e-4f) moveDir = glm::normalize(moveDir);
            else moveDir = glm::vec3(0.0f);
            const bool wantsToWalk = speed > comp.minWalkSpeed;

            if (wantsToWalk) {
                const float targetYaw = std::atan2(-moveDir.x, -moveDir.z);
                float diff = std::fmod(targetYaw - comp._yaw + glm::pi<float>(), glm::two_pi<float>());
                if (diff < 0.0f) diff += glm::two_pi<float>();
                comp._yaw += (diff - glm::pi<float>()) * glm::min(comp.turnRate * dt, 1.0f);
            }

            comp._grounded = Physics::Raycast(xf.position, glm::vec3(0, -1, 0),
                                               comp.groundRayLength, entity).hit;
            rag.locomotionActive = comp._grounded && rag.strength > 0.01f;
            rag.locomotionTargetRot = glm::angleAxis(
                comp._yaw + glm::radians(comp.facingOffsetDeg), glm::vec3(0, 1, 0));

            const float tiltDeg = Physics::GetRagdollTiltDeg(rag);
            const bool canGait = comp._grounded &&
                                 comp._timeSincePowered >= comp.postPoweredGrace &&
                                 tiltDeg < rag.locomotionFallenTilt;
            const bool gaitRequested = wantsToWalk && canGait;
            const float gaitRate = 1.0f / glm::max(comp.gaitBlendTime, 0.01f);
            comp._gaitWeight = Approach(comp._gaitWeight, gaitRequested ? 1.0f : 0.0f, gaitRate * dt);

            const bool startedWalking = wantsToWalk && !comp._wasWalking;
            const bool stoppedWalking = !wantsToWalk && comp._wasWalking;
            if (startedWalking) {
                comp._landedStepCount = 0;
                comp._driveWeight = 0.0f;
                comp._timeSinceLanding = comp.doubleSupportTime;
                comp._phaseTime = 0.0f;
                comp._weightShiftStableTime = 0.0f;
                comp._firstStep = true;
                comp._weightShiftTargetCaptured = false;
                comp._gaitPhase = LocamotionControllerComponent::GaitPhase::Idle;
            }
            if (stoppedWalking) {
                comp._legL = {};
                comp._legR = {};
                comp._nextLeft = true;
                comp._phaseTime = 0.0f;
                comp._weightShiftStableTime = 0.0f;
                comp._firstStep = true;
                comp._weightShiftTargetCaptured = false;
                comp._gaitPhase = LocamotionControllerComponent::GaitPhase::Idle;
            }
            if (!wantsToWalk || !canGait) comp._landedStepCount = 0;
            comp._wasWalking = wantsToWalk;

            const float driveTarget = gaitRequested ? 1.0f : 0.0f;
            const float driveRate = 1.0f / glm::max(comp.moveRampTime, 0.01f);
            comp._driveWeight = Approach(comp._driveWeight, driveTarget, driveRate * dt);

            float activeStepDuration = comp.stepDuration;
            if (comp._legL.phase == LocamotionControllerComponent::LegPhase::Swinging)
                activeStepDuration = comp._legL.swingDuration;
            else if (comp._legR.phase == LocamotionControllerComponent::LegPhase::Swinging)
                activeStepDuration = comp._legR.swingDuration;
            const float gaitCycle =
                2.0f * (activeStepDuration + comp.doubleSupportTime + comp.weightShiftTime);
            const float gaitSpeedLimit = comp.stepLength / glm::max(gaitCycle, 0.01f);
            const float driveSpeed = glm::min(speed, gaitSpeedLimit);
            const float tiltT = glm::clamp((tiltDeg - 20.0f) / 10.0f, 0.0f, 1.0f);
            const float tiltScale = 1.0f - tiltT * tiltT * (3.0f - 2.0f * tiltT);

            UpdateGait(scene, entity, comp, rag, moveDir, gaitRequested, dt);

            const float phaseDriveScale =
                comp._gaitPhase == LocamotionControllerComponent::GaitPhase::WeightShift
                    ? glm::clamp(comp.weightShiftDriveScale, 0.0f, 1.0f)
                    : 1.0f;
            rag.locomotionTargetVel =
                moveDir * driveSpeed * comp._driveWeight * tiltScale * phaseDriveScale;

            if (comp.debug) {
                _debugTimer += dt;
                if (_debugTimer >= 0.25f) {
                    _debugTimer = 0.0f;
                    const glm::vec3 v = rag._locomotionRootVel;
                    spdlog::info("[Loco] phase={} input={:.2f} requested={:.2f} target={:.2f} drive={:.2f} actual={:.2f} along={:+.2f} gait={:.2f} tilt={:.1f} steps={} poles=({:+.0f},{:+.0f}) limit=({:.1f},{:.1f}) clamped=({},{})",
                                 LocomotionGaitPhaseName(comp._gaitPhase),
                                 magnitude, speed,
                                 glm::length(glm::vec2(rag.locomotionTargetVel.x,
                                                       rag.locomotionTargetVel.z)),
                                 comp._driveWeight,
                                 glm::length(glm::vec2(v.x, v.z)), glm::dot(v, moveDir),
                                 comp._gaitWeight,
                                 tiltDeg, comp._landedStepCount,
                                 comp._legL.selectedPoleDeg, comp._legR.selectedPoleDeg,
                                 comp._legL.hipLimitViolationDeg,
                                 comp._legR.hipLimitViolationDeg,
                                 comp._legL.hipTargetClamped,
                                 comp._legR.hipTargetClamped);
                }
            }
        }
    }

private:
    enum class SwingResult { None, Target, Settled };

    float _debugTimer = 0.0f;

    static float Approach(float value, float target, float amount)
    {
        if (value < target) return glm::min(value + amount, target);
        return glm::max(value - amount, target);
    }

    static float SmootherStep(float t)
    {
        t = glm::clamp(t, 0.0f, 1.0f);
        return t * t * t * (t * (t * 6.0f - 15.0f) + 10.0f);
    }

    static void ResetGait(LocamotionControllerComponent& c)
    {
        c._gaitWeight = 0.0f;
        c._driveWeight = 0.0f;
        c._wasWalking = false;
        c._landedStepCount = 0;
        c._phaseTime = 0.0f;
        c._weightShiftStableTime = 0.0f;
        c._firstStep = true;
        c._weightShiftTargetCaptured = false;
        c._gaitPhase = LocamotionControllerComponent::GaitPhase::Idle;
        c._legL = {};
        c._legR = {};
    }

    static void CameraRelativeBasis(Scene& scene, glm::vec3& right, glm::vec3& forward)
    {
        right = glm::vec3(1, 0, 0);
        forward = glm::vec3(0, 0, -1);
        const entt::entity cam = scene.GetPrimaryCamera();
        if (cam == entt::null) return;

        if (scene.Has<CameraDirectorComponent>(cam)) {
            const float yaw = glm::radians(scene.Get<CameraDirectorComponent>(cam).yawDeg);
            forward = glm::vec3(std::sin(yaw), 0.0f, -std::cos(yaw));
            right = glm::vec3(std::cos(yaw), 0.0f, std::sin(yaw));
        } else if (scene.Has<TransformComponent>(cam)) {
            const auto& transform = scene.Get<TransformComponent>(cam);
            forward = transform.rotation * glm::vec3(0, 0, -1);
            right = transform.rotation * glm::vec3(1, 0, 0);
            forward.y = right.y = 0.0f;
            if (glm::length(forward) > 1e-4f) forward = glm::normalize(forward);
            if (glm::length(right) > 1e-4f) right = glm::normalize(right);
        }
    }

    static glm::mat4 BoneWorldMatrix(const Diamond::Skeleton& skeleton,
                                     const AnimatorComponent& animator,
                                     const glm::mat4& entityWorld, int bone)
    {
        return entityWorld * animator.palette[bone] *
               glm::inverse(skeleton.bones[bone].inverseBind);
    }

    static glm::vec3 BoneWorldPos(const Diamond::Skeleton& skeleton,
                                  const AnimatorComponent& animator,
                                  const glm::mat4& entityWorld, int bone)
    {
        return glm::vec3(BoneWorldMatrix(skeleton, animator, entityWorld, bone)[3]);
    }

    static glm::quat OrientationOf(const glm::mat4& matrix)
    {
        const glm::vec3 x(matrix[0]), y(matrix[1]), z(matrix[2]);
        if (glm::dot(x, x) < 1e-12f || glm::dot(y, y) < 1e-12f || glm::dot(z, z) < 1e-12f)
            return glm::quat(1, 0, 0, 0);
        return glm::normalize(glm::quat_cast(
            glm::mat3(glm::normalize(x), glm::normalize(y), glm::normalize(z))));
    }

    static glm::quat BoneWorldRot(const Diamond::Skeleton& skeleton,
                                  const AnimatorComponent& animator,
                                  const glm::mat4& entityWorld, int bone)
    {
        return OrientationOf(BoneWorldMatrix(skeleton, animator, entityWorld, bone));
    }

    static glm::vec3 PerpendicularTo(const glm::vec3& direction)
    {
        glm::vec3 perpendicular = glm::cross(direction, glm::vec3(0, 1, 0));
        if (glm::dot(perpendicular, perpendicular) < 1e-8f)
            perpendicular = glm::cross(direction, glm::vec3(1, 0, 0));
        return glm::normalize(perpendicular);
    }

    static glm::vec3 ConstraintPlaneAxis(const glm::vec3& axis)
    {
        const glm::vec3 normalized = glm::normalize(axis);
        const glm::vec3 reference =
            std::abs(normalized.y) < 0.99f ? glm::vec3(0, 1, 0) : glm::vec3(1, 0, 0);
        return glm::normalize(glm::cross(normalized, reference));
    }

    static void ResolveLeg(LocamotionControllerComponent::LegState& leg,
                           const std::string& footName,
                           const Diamond::Skeleton& skeleton,
                           const RagdollComponent& rag)
    {
        if (leg.footIdx >= 0 && leg.footIdx < static_cast<int>(skeleton.bones.size()) &&
            skeleton.bones[leg.footIdx].name == footName) return;

        leg.footIdx = skeleton.Find(footName);
        if (leg.footIdx < 0 || footName.empty() || footName.back() != '5') return;
        const std::string base = footName.substr(0, footName.size() - 1);
        leg.hipIdx = skeleton.Find(base + "1");
        leg.kneeIdx = skeleton.Find(base + "2");
        leg.ankleIdx = skeleton.Find(base + "3");

        if (!rag.config || leg.kneeIdx < 0 || leg.hipIdx < 0) return;
        for (const auto& body : rag.config->bodies) {
            if (body.boneName == skeleton.bones[leg.kneeIdx].name) {
                leg.kneeHingeAxis = body.twistAxisLocal;
            } else if (body.boneName == skeleton.bones[leg.hipIdx].name) {
                leg.hipTwistAxis = body.twistAxisLocal;
                leg.hipSwingNormalDeg = body.swingNormalDeg;
                leg.hipSwingPlaneDeg = body.swingPlaneDeg;
                leg.hipTwistMinDeg = body.twistMinDeg;
                leg.hipTwistMaxDeg = body.twistMaxDeg;
            }
        }
    }

    static glm::vec3 GroundedSoleTarget(Scene& scene, entt::entity entity,
                                        const glm::vec3& hips, const glm::vec3& horizontal,
                                        float rayLength, float fallbackY)
    {
        const glm::vec3 probe(hips.x + horizontal.x, hips.y, hips.z + horizontal.z);
        const HitResult hit = Physics::Raycast(probe, glm::vec3(0, -1, 0), rayLength, entity);
        return hit.hit ? hit.point : glm::vec3(probe.x, fallbackY, probe.z);
    }

    struct ReachProjection
    {
        glm::vec3 target { 0.0f };
        float fraction = 1.0f;
        float liftScale = 1.0f;
        float poleDeg = 0.0f;
        float duration = 0.40f;
        float requestedViolationDeg = 0.0f;
        float projectedViolationDeg = 0.0f;
        float requestedDuration = 0.40f;
    };

    struct TrajectoryEvaluation
    {
        float poleDeg = 0.0f;
        float violationDeg = std::numeric_limits<float>::infinity();
        float requiredDuration = 0.0f;
        float score = std::numeric_limits<float>::infinity();
    };

    static glm::vec3 SwingSoleTarget(
        const glm::vec3& start, const glm::vec3& target, float progress,
        float toeOffFraction, float toeOffHeight, float stepLift, float liftScale)
    {
        const float t = glm::clamp(progress, 0.0f, 1.0f);
        const float toeEnd = glm::clamp(toeOffFraction, 0.05f, 0.45f);
        const float toeT = glm::clamp(t / toeEnd, 0.0f, 1.0f);
        if (t < toeEnd) {
            glm::vec3 result = start;
            result.y += toeOffHeight * liftScale * SmootherStep(toeT);
            return result;
        }

        const float travelT = (t - toeEnd) / (1.0f - toeEnd);
        const float travelBlend = SmootherStep(travelT);
        glm::vec3 result = glm::mix(start, target, travelBlend);
        const float liftWave = std::sin(glm::pi<float>() * travelT);
        result.y += liftScale * (
            toeOffHeight * (1.0f - travelBlend) +
            stepLift * liftWave * liftWave);
        return result;
    }

    static ReachProjection ProjectSwingTarget(
        LocamotionControllerComponent::LegState& leg,
        const Diamond::Skeleton& skeleton, AnimatorComponent& animator,
        const glm::mat4& entityWorld, const glm::vec3& moveDir,
        const glm::vec3& desiredTarget, float reachFraction, float hipLimitMarginDeg,
        float toeOffFraction, float toeOffHeight, float stepLift,
        float legLength, float baseDuration, float maximumDuration,
        float maximumHipSpeedDeg, float maximumKneeSpeedDeg,
        float maximumFootSpeedLegsPerSec)
    {
        ReachProjection result;
        result.target = desiredTarget;
        result.duration = baseDuration;

        const glm::vec3 savedSoleTarget = leg.soleTarget;
        const float savedPoleDeg = leg.selectedPoleDeg;
        const float savedViolationDeg = leg.hipLimitViolationDeg;
        const bool savedClamped = leg.hipTargetClamped;
        auto quaternionAngleDeg = [](const glm::quat& from, const glm::quat& to) {
            const glm::quat delta =
                glm::normalize(glm::conjugate(from) * to);
            return glm::degrees(2.0f * std::acos(
                glm::clamp(std::abs(delta.w), 0.0f, 1.0f)));
        };
        auto evaluatePole = [&](float fraction, float liftScale, float poleDeg) {
            TrajectoryEvaluation evaluation;
            evaluation.poleDeg = poleDeg;
            evaluation.violationDeg = 0.0f;

            const int hipParent = skeleton.bones[leg.hipIdx].parent;
            const glm::quat hipParentWorld = hipParent >= 0
                ? BoneWorldRot(skeleton, animator, entityWorld, hipParent)
                : OrientationOf(entityWorld);
            const glm::quat hipWorld =
                BoneWorldRot(skeleton, animator, entityWorld, leg.hipIdx);
            const glm::quat kneeWorld =
                BoneWorldRot(skeleton, animator, entityWorld, leg.kneeIdx);
            glm::quat previousHip = glm::normalize(
                glm::conjugate(hipParentWorld) * hipWorld);
            glm::quat previousKnee = glm::normalize(
                glm::conjugate(hipWorld) * kneeWorld);
            glm::vec3 previousSole = leg.swingStartSole;
            float maximumViolationDeg = 0.0f;
            const glm::vec3 target =
                glm::mix(leg.swingStartSole, desiredTarget, fraction);
            constexpr int trajectorySamples = 8;
            for (int sample = 0; sample <= trajectorySamples; ++sample) {
                const float progress =
                    static_cast<float>(sample) / static_cast<float>(trajectorySamples);
                leg.soleTarget = SwingSoleTarget(
                    leg.swingStartSole, target, progress, toeOffFraction,
                    toeOffHeight, stepLift, liftScale);
                glm::quat hipTarget, kneeTarget;
                SolveLeg(leg, skeleton, animator, entityWorld, moveDir, reachFraction,
                         hipLimitMarginDeg, poleDeg, 0.0f, false, false,
                         &hipTarget, &kneeTarget);
                maximumViolationDeg =
                    glm::max(maximumViolationDeg, leg.hipLimitViolationDeg);

                const float sampleScale = static_cast<float>(trajectorySamples);
                evaluation.requiredDuration = glm::max(
                    evaluation.requiredDuration,
                    quaternionAngleDeg(previousHip, hipTarget) * sampleScale /
                        glm::max(maximumHipSpeedDeg, 1.0f));
                evaluation.requiredDuration = glm::max(
                    evaluation.requiredDuration,
                    quaternionAngleDeg(previousKnee, kneeTarget) * sampleScale /
                        glm::max(maximumKneeSpeedDeg, 1.0f));
                evaluation.requiredDuration = glm::max(
                    evaluation.requiredDuration,
                    glm::length(leg.soleTarget - previousSole) * sampleScale /
                        glm::max(legLength * maximumFootSpeedLegsPerSec, 0.01f));
                previousHip = hipTarget;
                previousKnee = kneeTarget;
                previousSole = leg.soleTarget;
            }
            evaluation.violationDeg = maximumViolationDeg;
            evaluation.score =
                maximumViolationDeg * 1000.0f +
                evaluation.requiredDuration +
                std::abs(poleDeg) * 0.002f +
                std::abs(poleDeg - savedPoleDeg) * 0.001f;
            return evaluation;
        };
        auto bestTrajectory = [&](float fraction, float liftScale) {
            TrajectoryEvaluation best;
            for (int poleDeg = -90; poleDeg <= 90; poleDeg += 10) {
                const TrajectoryEvaluation candidate =
                    evaluatePole(fraction, liftScale, static_cast<float>(poleDeg));
                if (candidate.score < best.score) best = candidate;
            }
            return best;
        };

        constexpr float feasibleToleranceDeg = 0.05f;
        const float safeMaximumDuration =
            glm::max(maximumDuration, baseDuration);
        const TrajectoryEvaluation requested = bestTrajectory(1.0f, 1.0f);
        result.requestedViolationDeg = requested.violationDeg;
        result.requestedDuration =
            glm::max(baseDuration, requested.requiredDuration);
        auto feasible = [&](const TrajectoryEvaluation& evaluation) {
            return evaluation.violationDeg <= feasibleToleranceDeg;
        };
        auto furthestFeasibleFraction = [&](float liftScale) {
            // Walk backward along the requested stride until a legal foothold is
            // found, then refine the boundary. Keep the already selected pole so
            // refinement is a bounded one-dimensional search.
            constexpr int coarseSteps = 8;
            float feasibleFraction = -1.0f;
            float infeasibleFraction = 1.0f;
            for (int step = coarseSteps - 1; step >= 0; --step) {
                const float fraction =
                    static_cast<float>(step) / static_cast<float>(coarseSteps);
                if (feasible(evaluatePole(
                        fraction, liftScale, requested.poleDeg))) {
                    feasibleFraction = fraction;
                    infeasibleFraction =
                        static_cast<float>(step + 1) / static_cast<float>(coarseSteps);
                    break;
                }
            }

            if (feasibleFraction >= 0.0f) {
                for (int iteration = 0; iteration < 5; ++iteration) {
                    const float middle = (feasibleFraction + infeasibleFraction) * 0.5f;
                    if (feasible(evaluatePole(
                            middle, liftScale, requested.poleDeg)))
                        feasibleFraction = middle;
                    else
                        infeasibleFraction = middle;
                }
            }
            return feasibleFraction;
        };

        if (!feasible(requested)) {
            result.fraction = furthestFeasibleFraction(1.0f);

            // Prefer the authored lift when it still permits a useful stride. If
            // the vertical arc itself blocks the leg, find the highest lift that
            // maximizes legal forward travel.
            constexpr float minimumUsefulFraction = 0.35f;
            if (result.fraction < minimumUsefulFraction) {
                float bestFraction = glm::max(result.fraction, 0.0f);
                float bestLiftScale = result.fraction >= 0.0f ? 1.0f : 0.4f;
                // Preserve enough clearance to avoid replacing a constraint-safe
                // swing with a foot that simply scrapes along the floor.
                for (int liftStep = 9; liftStep >= 4; --liftStep) {
                    const float liftScale = static_cast<float>(liftStep) / 10.0f;
                    const float fraction = furthestFeasibleFraction(liftScale);
                    if (fraction > bestFraction + 1e-4f ||
                        (std::abs(fraction - bestFraction) <= 1e-4f &&
                         liftScale > bestLiftScale)) {
                        bestFraction = glm::max(fraction, 0.0f);
                        bestLiftScale = liftScale;
                    }
                }
                result.fraction = bestFraction;
                result.liftScale = bestLiftScale;
            }

            if (result.fraction < 0.0f) result.fraction = 0.0f;
            result.target = glm::mix(
                leg.swingStartSole, desiredTarget, result.fraction);
        }

        const TrajectoryEvaluation projected = evaluatePole(
            result.fraction, result.liftScale, requested.poleDeg);
        result.poleDeg = projected.poleDeg;
        result.projectedViolationDeg = projected.violationDeg;
        result.duration = glm::clamp(
            glm::max(baseDuration, projected.requiredDuration),
            baseDuration, safeMaximumDuration);
        leg.soleTarget = savedSoleTarget;
        leg.selectedPoleDeg = savedPoleDeg;
        leg.hipLimitViolationDeg = savedViolationDeg;
        leg.hipTargetClamped = savedClamped;
        return result;
    }

    void UpdateGait(Scene& scene, entt::entity entity, LocamotionControllerComponent& comp,
                    RagdollComponent& rag, const glm::vec3& moveDir,
                    bool walking, float dt)
    {
        if (!scene.Has<SkinnedMeshComponent>(entity) || !scene.Has<AnimatorComponent>(entity)) return;
        auto& mesh = scene.Get<SkinnedMeshComponent>(entity);
        auto& animator = scene.Get<AnimatorComponent>(entity);
        const auto& skeleton = mesh.skeleton;
        const int count = static_cast<int>(skeleton.bones.size());
        if (count == 0 || static_cast<int>(animator.palette.size()) != count ||
            static_cast<int>(animator.pose.size()) != count) return;

        const int rootBone = Physics::GetRagdollRootBone(rag);
        if (rootBone < 0) return;
        const glm::mat4 entityWorld = scene.GetTransformSystem().GetWorldMatrix(entity);

        ResolveLeg(comp._legL, comp.leftFootBone, skeleton, rag);
        ResolveLeg(comp._legR, comp.rightFootBone, skeleton, rag);
        if (!ValidLeg(comp._legL) || !ValidLeg(comp._legR)) return;

        InitializeLeg(scene, entity, comp._legL, skeleton, animator, entityWorld);
        InitializeLeg(scene, entity, comp._legR, skeleton, animator, entityWorld);

        const glm::vec3 leftFootNow = BoneWorldPos(skeleton, animator, entityWorld, comp._legL.footIdx);
        const glm::vec3 rightFootNow = BoneWorldPos(skeleton, animator, entityWorld, comp._legR.footIdx);
        const SwingResult leftResult = AdvanceSwing(
            entity, comp._legL, comp, leftFootNow, dt, comp.debug, "left");
        const SwingResult rightResult = AdvanceSwing(
            entity, comp._legR, comp, rightFootNow, dt, comp.debug, "right");
        const bool completed = leftResult != SwingResult::None || rightResult != SwingResult::None;
        const bool reachedTarget = leftResult == SwingResult::Target ||
                                   rightResult == SwingResult::Target;
        const bool settled = leftResult == SwingResult::Settled ||
                             rightResult == SwingResult::Settled;
        if (completed) {
            if (settled) comp._landedStepCount = 0;
            else if (reachedTarget) ++comp._landedStepCount;
            comp._timeSinceLanding = 0.0f;
        } else {
            comp._timeSinceLanding += dt;
        }

        auto startSwing = [&]() {
            auto& leg = comp._nextLeft ? comp._legL : comp._legR;
            const auto& stanceLeg = comp._nextLeft ? comp._legR : comp._legL;
            const float side = comp._nextLeft ? -1.0f : 1.0f;
            const glm::vec3 stepDirection = moveDir;
            const glm::vec3 right = glm::normalize(glm::cross(moveDir, glm::vec3(0, 1, 0)));
            const glm::vec3 footNow = BoneWorldPos(skeleton, animator, entityWorld, leg.footIdx);
            leg.swingStartSole = footNow - glm::vec3(0, leg.soleHeight, 0);
            leg.swingDirection = stepDirection;
            const glm::vec3 rootPosition = BoneWorldPos(skeleton, animator, entityWorld, rootBone);
            glm::vec3 placement = rootPosition;
            glm::vec3 horizontalTarget = stanceLeg.plantSole + stepDirection * comp.stepLength;
            const float desiredSide = glm::dot(
                stanceLeg.plantSole + right * (side * comp.stanceHalfWidth * 2.0f), right);
            horizontalTarget += right * (desiredSide - glm::dot(horizontalTarget, right));
            placement.x = horizontalTarget.x;
            placement.z = horizontalTarget.z;
            glm::vec3 desiredTarget = GroundedSoleTarget(
                scene, entity, placement, glm::vec3(0.0f), comp.groundRayLength,
                leg.swingStartSole.y);
            const float legLength =
                glm::length(skeleton.bones[leg.kneeIdx].localT) +
                glm::length(skeleton.bones[leg.ankleIdx].localT);
            const glm::vec2 requestedTravel(
                desiredTarget.x - leg.swingStartSole.x,
                desiredTarget.z - leg.swingStartSole.z);
            const float requestedTravelLength = glm::length(requestedTravel);
            const float maximumTravel =
                legLength * glm::max(comp.maxSwingTravelFraction, 0.0f);
            float travelCap = 1.0f;
            if (requestedTravelLength > maximumTravel &&
                requestedTravelLength > 1e-4f) {
                travelCap = maximumTravel / requestedTravelLength;
                desiredTarget.x =
                    leg.swingStartSole.x + requestedTravel.x * travelCap;
                desiredTarget.z =
                    leg.swingStartSole.z + requestedTravel.y * travelCap;
            }
            const ReachProjection projection = ProjectSwingTarget(
                leg, skeleton, animator, entityWorld, stepDirection, desiredTarget,
                comp.maxReachFraction, comp.hipLimitMarginDeg, comp.toeOffFraction,
                comp.toeOffHeight, comp.stepLift, legLength, comp.stepDuration,
                comp.maxStepDuration, comp.maxHipSpeedDeg, comp.maxKneeSpeedDeg,
                comp.maxFootSpeedLegsPerSec);
            leg.swingTargetSole = projection.target;
            leg.swingLiftScale = projection.liftScale;
            leg.swingPoleDeg = projection.poleDeg;
            leg.swingDuration = projection.duration;
            leg.swingProgress = 0.0f;
            leg.swingPoseBlend = 0.0f;
            leg.landingWait = 0.0f;
            leg.landingWarning = false;
            leg.phase = LocamotionControllerComponent::LegPhase::Swinging;
            comp._nextLeft = !comp._nextLeft;
            comp._firstStep = false;
            comp._phaseTime = 0.0f;
            comp._weightShiftStableTime = 0.0f;
            comp._gaitPhase = LocamotionControllerComponent::GaitPhase::Swing;
            if (comp.debug)
                spdlog::info("[Loco] {} step to ({:+.2f}, {:+.2f}, {:+.2f}) swingTravel={:+.2f} aheadOfStance={:+.2f} travelCap={:.2f} projection={:.2f} lift={:.2f} pole={:+.0f} duration={:.2f} requestedDuration={:.2f} requestedLimit={:.1f} projectedLimit={:.1f} move=({:+.2f},{:+.2f})",
                             side < 0.0f ? "left" : "right", leg.swingTargetSole.x,
                             leg.swingTargetSole.y, leg.swingTargetSole.z,
                             glm::dot(leg.swingTargetSole - leg.swingStartSole, stepDirection),
                             glm::dot(leg.swingTargetSole - stanceLeg.plantSole, stepDirection),
                             travelCap, projection.fraction, projection.liftScale,
                             projection.poleDeg, projection.duration,
                             projection.requestedDuration,
                             projection.requestedViolationDeg,
                             projection.projectedViolationDeg,
                             moveDir.x, moveDir.z);
        };

        const bool anySwing = comp._legL.phase == LocamotionControllerComponent::LegPhase::Swinging ||
                              comp._legR.phase == LocamotionControllerComponent::LegPhase::Swinging;
        if (!walking) {
            comp._phaseTime = 0.0f;
            comp._weightShiftStableTime = 0.0f;
            comp._gaitPhase = anySwing
                ? LocamotionControllerComponent::GaitPhase::Swing
                : LocamotionControllerComponent::GaitPhase::Idle;
        } else if (anySwing) {
            comp._gaitPhase = LocamotionControllerComponent::GaitPhase::Swing;
            const auto& swingLeg =
                comp._legL.phase == LocamotionControllerComponent::LegPhase::Swinging
                    ? comp._legL : comp._legR;
            const float fadeStart = glm::clamp(comp.toeOffFraction, 0.05f, 0.45f);
            const float fadeEnd = glm::max(comp.swingSupportFadeEnd, fadeStart + 0.01f);
            const float fadeT = (swingLeg.swingProgress - fadeStart) / (fadeEnd - fadeStart);
            rag.locomotionSupportTarget = comp._swingSupportTarget;
            rag.locomotionSupportTargetVel = glm::vec3(0.0f);
            rag.locomotionSupportTargetWeight = 1.0f - SmootherStep(fadeT);
        } else {
            using GaitPhase = LocamotionControllerComponent::GaitPhase;
            if (completed) {
                comp._phaseTime = 0.0f;
                comp._weightShiftStableTime = 0.0f;
                comp._gaitPhase = GaitPhase::DoubleSupport;
            }

            if (comp._gaitPhase == GaitPhase::Idle) {
                comp._phaseTime = 0.0f;
                comp._weightShiftStableTime = 0.0f;
                comp._weightShiftTargetCaptured = false;
                comp._gaitPhase = GaitPhase::WeightShift;
                if (comp.debug) spdlog::info("[Loco] weight shift started");
            } else if (comp._gaitPhase == GaitPhase::DoubleSupport) {
                comp._phaseTime += dt;
                if (comp._phaseTime >= comp.doubleSupportTime) {
                    comp._phaseTime = 0.0f;
                    comp._weightShiftStableTime = 0.0f;
                    comp._weightShiftTargetCaptured = false;
                    comp._gaitPhase = GaitPhase::WeightShift;
                    if (comp.debug) spdlog::info("[Loco] weight shift started");
                }
            }

            if (comp._gaitPhase == GaitPhase::WeightShift) {
                const auto& stanceLeg = comp._nextLeft ? comp._legR : comp._legL;
                const glm::vec3 midpoint = (comp._legL.plantSole + comp._legR.plantSole) * 0.5f;
                const glm::vec3 finalSupportTarget = glm::mix(
                    stanceLeg.plantSole, midpoint,
                    glm::clamp(comp.weightShiftInboard, 0.0f, 0.5f));
                comp._swingSupportTarget = finalSupportTarget;
                if (!comp._weightShiftTargetCaptured) {
                    comp._weightShiftStartTarget = rag._locomotionCOMValid
                        ? glm::vec3(rag._locomotionCOM.x, finalSupportTarget.y,
                                    rag._locomotionCOM.z)
                        : midpoint;
                    comp._weightShiftTargetCaptured = true;
                }

                const float targetTime = glm::max(comp.weightShiftTargetTime, 0.01f);
                const float targetT = glm::clamp(comp._phaseTime / targetTime, 0.0f, 1.0f);
                const float targetBlend = SmootherStep(targetT);
                const float targetBlendRate =
                    targetT < 1.0f
                        ? 30.0f * targetT * targetT *
                          (targetT - 1.0f) * (targetT - 1.0f) / targetTime
                        : 0.0f;
                const glm::vec3 targetTravel =
                    finalSupportTarget - comp._weightShiftStartTarget;
                rag.locomotionSupportTarget =
                    comp._weightShiftStartTarget + targetTravel * targetBlend;
                rag.locomotionSupportTargetVel = targetTravel * targetBlendRate;
                rag.locomotionSupportTargetWeight = 1.0f;

                bool stanceGrounded = false;
                for (int i = 0; i < 2; ++i) {
                    if (rag._locomotionFootBones[i] == stanceLeg.footIdx) {
                        stanceGrounded = rag._locomotionFootGrounded[i];
                        break;
                    }
                }

                const glm::vec2 comDelta(
                    rag._locomotionCOM.x - finalSupportTarget.x,
                    rag._locomotionCOM.z - finalSupportTarget.z);
                const float comError = rag._locomotionCOMValid
                    ? glm::length(comDelta)
                    : std::numeric_limits<float>::infinity();
                const bool supportReady =
                    stanceGrounded && comError <= glm::max(comp.weightShiftRadius, 0.0f);
                comp._weightShiftStableTime = supportReady
                    ? comp._weightShiftStableTime + dt
                    : 0.0f;
                comp._phaseTime += dt;

                const float minimumTime =
                    comp._firstStep ? comp.firstWeightShiftTime : comp.weightShiftTime;
                const bool stable =
                    comp._weightShiftStableTime >= glm::max(comp.weightShiftStableTime, 0.0f);
                if (comp._phaseTime >= glm::max(minimumTime, 0.0f) && stable) {
                    if (comp.debug)
                        spdlog::info("[Loco] weight shift ready stance={} comError={:.3f} time={:.2f}",
                                     comp._nextLeft ? "right" : "left",
                                     comError, comp._phaseTime);
                    startSwing();
                } else if (comp._phaseTime >=
                           glm::max(comp.weightShiftTimeout, minimumTime)) {
                    if (comp.debug)
                        spdlog::warn("[Loco] weight shift timed out stance={} grounded={} comError={:.3f}",
                                     comp._nextLeft ? "right" : "left",
                                     stanceGrounded, comError);
                    comp._phaseTime = 0.0f;
                    comp._weightShiftStableTime = 0.0f;
                    comp._weightShiftTargetCaptured = false;
                    comp._gaitPhase = GaitPhase::DoubleSupport;
                    rag.locomotionSupportTargetWeight = 0.0f;
                }
            }
        }

        if (comp._gaitWeight > 0.001f) {
            const float leftWeight = comp._legL.phase == LocamotionControllerComponent::LegPhase::Swinging
                ? glm::mix(comp.stancePoseWeight, comp.swingPoseWeight,
                           comp._legL.swingPoseBlend)
                : comp.stancePoseWeight;
            SolveLeg(comp._legL, skeleton, animator, entityWorld, moveDir,
                     comp.maxReachFraction, comp.hipLimitMarginDeg,
                     comp._legL.phase == LocamotionControllerComponent::LegPhase::Swinging
                         ? comp._legL.swingPoleDeg
                         : std::numeric_limits<float>::quiet_NaN(),
                     comp._gaitWeight * leftWeight,
                     comp.ikWriteEnabled, comp.debug);
            const float rightWeight = comp._legR.phase == LocamotionControllerComponent::LegPhase::Swinging
                ? glm::mix(comp.stancePoseWeight, comp.swingPoseWeight,
                           comp._legR.swingPoseBlend)
                : comp.stancePoseWeight;
            SolveLeg(comp._legR, skeleton, animator, entityWorld, moveDir,
                     comp.maxReachFraction, comp.hipLimitMarginDeg,
                     comp._legR.phase == LocamotionControllerComponent::LegPhase::Swinging
                         ? comp._legR.swingPoleDeg
                         : std::numeric_limits<float>::quiet_NaN(),
                     comp._gaitWeight * rightWeight,
                     comp.ikWriteEnabled, comp.debug);
        }

        if (comp.debug) {
            if (rag._locomotionCOMValid) {
                const glm::vec3 comMarker(rag._locomotionCOM.x,
                                          rag.locomotionSupportTarget.y + 0.03f,
                                          rag._locomotionCOM.z);
                DebugDraw::Sphere(comMarker, 0.035f, {1.0f, 0.85f, 0.1f});
            }
            if (rag.locomotionSupportTargetWeight > 0.0f)
                DebugDraw::Sphere(rag.locomotionSupportTarget + glm::vec3(0, 0.03f, 0),
                                  0.04f, {0.1f, 0.9f, 0.9f});
            DrawLeg(comp._legL);
            DrawLeg(comp._legR);
        }
    }

    static bool ValidLeg(const LocamotionControllerComponent::LegState& leg)
    {
        return leg.footIdx >= 0 && leg.ankleIdx >= 0 && leg.kneeIdx >= 0 && leg.hipIdx >= 0;
    }

    static void InitializeLeg(Scene& scene, entt::entity entity,
                              LocamotionControllerComponent::LegState& leg,
                              const Diamond::Skeleton& skeleton,
                              const AnimatorComponent& animator,
                              const glm::mat4& entityWorld)
    {
        if (leg.init) return;
        const glm::vec3 foot = BoneWorldPos(skeleton, animator, entityWorld, leg.footIdx);
        const HitResult hit = Physics::Raycast(foot + glm::vec3(0, 0.2f, 0),
                                               glm::vec3(0, -1, 0), 0.9f, entity);
        if (hit.hit) {
            leg.soleHeight = glm::clamp(foot.y - hit.point.y, 0.01f, 0.45f);
            leg.plantSole = hit.point;
        } else {
            leg.plantSole = foot - glm::vec3(0, leg.soleHeight, 0);
        }
        leg.soleTarget = leg.plantSole;
        leg.init = true;
    }

    static SwingResult AdvanceSwing(entt::entity entity,
                                    LocamotionControllerComponent::LegState& leg,
                                    const LocamotionControllerComponent& comp,
                                    const glm::vec3& footPosition, float dt,
                                    bool debug, const char* side)
    {
        if (leg.phase != LocamotionControllerComponent::LegPhase::Swinging) {
            leg.soleTarget = leg.plantSole;
            leg.swingPoseBlend = 0.0f;
            return SwingResult::None;
        }

        leg.swingProgress = glm::min(
            leg.swingProgress + dt / glm::max(leg.swingDuration, 0.01f), 1.0f);
        const float t = leg.swingProgress;
        const float toeEnd = glm::clamp(comp.toeOffFraction, 0.05f, 0.45f);
        const float toeT = glm::clamp(t / toeEnd, 0.0f, 1.0f);
        leg.swingPoseBlend = SmootherStep(toeT);

        leg.soleTarget = SwingSoleTarget(
            leg.swingStartSole, leg.swingTargetSole, t, comp.toeOffFraction,
            comp.toeOffHeight, comp.stepLift, leg.swingLiftScale);
        if (t < 1.0f) return SwingResult::None;

        const glm::vec3 actualSole = footPosition - glm::vec3(0, leg.soleHeight, 0);
        const float actualTravel = glm::dot(actualSole - leg.swingStartSole, leg.swingDirection);
        const glm::vec2 horizontalDelta(footPosition.x - leg.swingTargetSole.x,
                                        footPosition.z - leg.swingTargetSole.z);
        const float horizontalError = glm::length(horizontalDelta);
        const HitResult ground = Physics::Raycast(footPosition + glm::vec3(0, 0.05f, 0),
                                                  glm::vec3(0, -1, 0),
                                                  leg.soleHeight + 0.12f, entity);
        const float groundError = ground.hit
            ? std::abs(ground.point.y - leg.swingTargetSole.y)
            : std::numeric_limits<float>::infinity();
        const bool landed = horizontalError <= comp.landingTolerance && groundError <= 0.08f;
        if (!landed) {
            leg.landingWait += dt;
            if (debug && !leg.landingWarning && leg.landingWait >= 0.20f) {
                leg.landingWarning = true;
                spdlog::warn("[Loco] {} foot has not landed: horizontal={:.3f} actualAlongMove={:+.3f} ground={} groundError={:.3f}",
                             side, horizontalError, actualTravel, ground.hit, groundError);
            }
            if (!ground.hit || groundError > 0.08f || leg.landingWait < 0.35f)
                return SwingResult::None;

            leg.swingTargetSole = glm::vec3(footPosition.x, ground.point.y, footPosition.z);
        }

        leg.plantSole = leg.swingTargetSole;
        leg.soleTarget = leg.plantSole;
        leg.phase = LocamotionControllerComponent::LegPhase::Planted;
        leg.swingPoseBlend = 0.0f;
        if (debug)
            spdlog::info("[Loco] {} {} actualAlongMove={:+.3f}", side,
                         landed ? "landed" : "settled", actualTravel);
        return landed ? SwingResult::Target : SwingResult::Settled;
    }

    static void SolveLeg(LocamotionControllerComponent::LegState& leg,
                         const Diamond::Skeleton& skeleton, AnimatorComponent& animator,
                         const glm::mat4& entityWorld, const glm::vec3& moveDir,
                         float reachFraction, float hipLimitMarginDeg, float forcedPoleDeg,
                         float weight, bool writePose, bool debug,
                         glm::quat* solvedHipTarget = nullptr,
                         glm::quat* solvedKneeTarget = nullptr)
    {
        const glm::vec3 hipPos = BoneWorldPos(skeleton, animator, entityWorld, leg.hipIdx);
        const glm::vec3 anklePos = BoneWorldPos(skeleton, animator, entityWorld, leg.ankleIdx);
        const glm::vec3 footPos = BoneWorldPos(skeleton, animator, entityWorld, leg.footIdx);

        const float upperLength = glm::length(skeleton.bones[leg.kneeIdx].localT);
        const float lowerLength = glm::length(skeleton.bones[leg.ankleIdx].localT);
        if (upperLength < 1e-4f || lowerLength < 1e-4f) return;

        const glm::vec3 desiredFootCenter = leg.soleTarget + glm::vec3(0, leg.soleHeight, 0);
        glm::vec3 ankleTarget = desiredFootCenter + (anklePos - footPos);
        glm::vec3 toTarget = ankleTarget - hipPos;
        const float maxReach = (upperLength + lowerLength) * glm::clamp(reachFraction, 0.5f, 0.99f);
        if (const float reach = glm::length(toTarget); reach > maxReach && reach > 1e-4f)
            ankleTarget = hipPos + toTarget * (maxReach / reach);

        const int hipParent = skeleton.bones[leg.hipIdx].parent;
        const glm::quat hipParentRot = hipParent >= 0
            ? BoneWorldRot(skeleton, animator, entityWorld, hipParent)
            : OrientationOf(entityWorld);
        const float distance = glm::clamp(glm::length(ankleTarget - hipPos),
                                          std::abs(upperLength - lowerLength) + 1e-4f,
                                          upperLength + lowerLength - 1e-4f);
        const float includedCos = glm::clamp(
            (upperLength * upperLength + lowerLength * lowerLength - distance * distance) /
            (2.0f * upperLength * lowerLength), -1.0f, 1.0f);
        const float kneeBend = glm::pi<float>() - std::acos(includedCos);
        const glm::quat kneeTarget = glm::normalize(
            skeleton.bones[leg.kneeIdx].localR *
            glm::angleAxis(kneeBend, glm::normalize(leg.kneeHingeAxis)));

        const glm::vec3 upperLocal = skeleton.bones[leg.kneeIdx].localT;
        const glm::vec3 lowerLocal = kneeTarget * skeleton.bones[leg.ankleIdx].localT;
        const glm::vec3 chainLocal = upperLocal + lowerLocal;
        const glm::vec3 localForward = glm::normalize(chainLocal);
        glm::vec3 localBend = upperLocal - localForward * glm::dot(upperLocal, localForward);
        if (glm::dot(localBend, localBend) < 1e-8f)
            localBend = PerpendicularTo(localForward);
        else
            localBend = glm::normalize(localBend);
        const glm::vec3 localSide = glm::normalize(glm::cross(localForward, localBend));

        const glm::vec3 worldForward = glm::normalize(ankleTarget - hipPos);
        glm::vec3 poleDir = moveDir;
        if (glm::dot(poleDir, poleDir) < 1e-8f) poleDir = glm::vec3(0, 0, -1);
        glm::vec3 worldBend = poleDir - worldForward * glm::dot(poleDir, worldForward);
        if (glm::dot(worldBend, worldBend) < 1e-8f)
            worldBend = PerpendicularTo(worldForward);
        else
            worldBend = glm::normalize(worldBend);

        const glm::mat3 localBasis(localForward, localBend, localSide);

        // The ankle target admits many valid knee bend planes around worldForward.
        // Search those planes and prefer one whose hip rotation stays inside the
        // authored Jolt swing/twist cone. This matters for mirrored rigs: the same
        // movement-facing pole can map into a permissive cone region on one hip and
        // an unreachable region on the other.
        const glm::vec3 hipAxisLocal = glm::normalize(leg.hipTwistAxis);
        const glm::mat4 hipBindWorld =
            entityWorld * glm::inverse(skeleton.bones[leg.hipIdx].inverseBind);
        const glm::quat hipBindRot = OrientationOf(hipBindWorld);
        const glm::vec3 hipAxisWorld = hipBindRot * hipAxisLocal;
        const glm::vec3 planeAxisWorld = ConstraintPlaneAxis(hipAxisWorld);
        const glm::vec3 planeAxisLocal =
            glm::normalize(glm::conjugate(hipBindRot) * planeAxisWorld);
        const glm::vec3 normalAxisLocal =
            glm::normalize(glm::cross(hipAxisLocal, planeAxisLocal));
        const glm::quat constraintBasis = glm::normalize(glm::quat_cast(
            glm::mat3(hipAxisLocal, planeAxisLocal, normalAxisLocal)));

        const float marginDeg = glm::max(hipLimitMarginDeg, 0.0f);
        const float safePlaneDeg =
            glm::clamp(leg.hipSwingPlaneDeg - marginDeg, 1.0f, 179.0f);
        const float safeNormalDeg =
            glm::clamp(leg.hipSwingNormalDeg - marginDeg, 1.0f, 179.0f);
        const float twistMiddleDeg =
            (leg.hipTwistMinDeg + leg.hipTwistMaxDeg) * 0.5f;
        const float safeTwistMinDeg =
            glm::min(leg.hipTwistMinDeg + marginDeg, twistMiddleDeg);
        const float safeTwistMaxDeg =
            glm::max(leg.hipTwistMaxDeg - marginDeg, twistMiddleDeg);
        const float planeRadius = glm::max(
            std::sin(glm::radians(safePlaneDeg) * 0.5f),
            1e-4f);
        const float normalRadius = glm::max(
            std::sin(glm::radians(safeNormalDeg) * 0.5f),
            1e-4f);
        const float twistMin = glm::radians(safeTwistMinDeg);
        const float twistMax = glm::radians(safeTwistMaxDeg);

        glm::quat hipTarget(1, 0, 0, 0);
        glm::quat bestConstraintDelta(1, 0, 0, 0);
        glm::vec3 chosenWorldBend = worldBend;
        float bestScore = std::numeric_limits<float>::infinity();
        float bestPoleDeg = 0.0f;
        float bestViolationDeg = 0.0f;
        const float previousPole = glm::radians(leg.selectedPoleDeg);

        for (int poleDeg = -90; poleDeg <= 90; poleDeg += 5) {
            if (std::isfinite(forcedPoleDeg) &&
                poleDeg != static_cast<int>(std::round(forcedPoleDeg)))
                continue;
            const float poleAngle = glm::radians(static_cast<float>(poleDeg));
            const glm::vec3 candidateBend =
                glm::angleAxis(poleAngle, worldForward) * worldBend;
            const glm::vec3 candidateSide =
                glm::normalize(glm::cross(worldForward, candidateBend));
            const glm::mat3 worldBasis(worldForward, candidateBend, candidateSide);
            const glm::quat hipWorldTarget = glm::normalize(
                glm::quat_cast(worldBasis * glm::transpose(localBasis)));
            const glm::quat candidateHipTarget =
                glm::normalize(glm::conjugate(hipParentRot) * hipWorldTarget);

            glm::quat delta = glm::normalize(
                glm::conjugate(skeleton.bones[leg.hipIdx].localR) *
                candidateHipTarget);
            glm::quat constraintDelta = glm::normalize(
                glm::conjugate(constraintBasis) * delta * constraintBasis);
            if (constraintDelta.w < 0.0f) constraintDelta = -constraintDelta;

            glm::quat twist(
                constraintDelta.w, constraintDelta.x, 0.0f, 0.0f);
            const float twistLength = glm::length(twist);
            twist = twistLength > 1e-6f
                ? twist / twistLength
                : glm::quat(1, 0, 0, 0);
            glm::quat swing = glm::normalize(
                constraintDelta * glm::conjugate(twist));
            if (swing.w < 0.0f) swing = -swing;

            const float ellipseDistance = std::sqrt(
                (swing.y * swing.y) / (planeRadius * planeRadius) +
                (swing.z * swing.z) / (normalRadius * normalRadius));
            const float swingViolation = glm::max(ellipseDistance - 1.0f, 0.0f);
            const float twistAngle = 2.0f * std::atan2(twist.x, twist.w);
            const float twistViolation =
                twistAngle < twistMin ? twistMin - twistAngle :
                twistAngle > twistMax ? twistAngle - twistMax : 0.0f;
            const float violation =
                swingViolation + twistViolation / glm::pi<float>();
            const float hipRotation =
                2.0f * std::acos(glm::clamp(std::abs(delta.w), 0.0f, 1.0f));
            const float score =
                violation * 1000.0f +
                std::abs(poleAngle) * 0.50f +
                hipRotation * 0.10f +
                std::abs(poleAngle - previousPole) * 0.05f;
            if (score < bestScore) {
                bestScore = score;
                bestPoleDeg = static_cast<float>(poleDeg);
                bestViolationDeg = glm::degrees(
                    swingViolation * glm::max(
                        glm::radians(safePlaneDeg),
                        glm::radians(safeNormalDeg)) +
                    twistViolation);
                chosenWorldBend = candidateBend;
                hipTarget = candidateHipTarget;
                bestConstraintDelta = constraintDelta;
            }
        }

        leg.selectedPoleDeg = bestPoleDeg;
        leg.hipLimitViolationDeg = bestViolationDeg;
        leg.hipTargetClamped = bestViolationDeg > 0.05f;

        if (leg.hipTargetClamped) {
            // The planner preflights the nominal swing, but live pelvis motion can
            // still move the target outside the safe envelope. Clamp the selected
            // motor target in constraint space so physics never chases an
            // impossible orientation.
            glm::quat twist(
                bestConstraintDelta.w, bestConstraintDelta.x, 0.0f, 0.0f);
            const float twistLength = glm::length(twist);
            twist = twistLength > 1e-6f
                ? twist / twistLength
                : glm::quat(1, 0, 0, 0);
            glm::quat swing = glm::normalize(
                bestConstraintDelta * glm::conjugate(twist));
            if (swing.w < 0.0f) swing = -swing;

            const float twistAngle = glm::clamp(
                2.0f * std::atan2(twist.x, twist.w), twistMin, twistMax);
            const glm::quat clampedTwist =
                glm::angleAxis(twistAngle, glm::vec3(1, 0, 0));

            const float ellipseDistance = std::sqrt(
                (swing.y * swing.y) / (planeRadius * planeRadius) +
                (swing.z * swing.z) / (normalRadius * normalRadius));
            const float swingScale =
                ellipseDistance > 1.0f ? 1.0f / ellipseDistance : 1.0f;
            const float swingY = swing.y * swingScale;
            const float swingZ = swing.z * swingScale;
            const float swingW = std::sqrt(glm::max(
                1.0f - swingY * swingY - swingZ * swingZ, 0.0f));
            const glm::quat clampedSwing(swingW, 0.0f, swingY, swingZ);

            const glm::quat clampedConstraintDelta = glm::normalize(
                clampedSwing * clampedTwist);
            const glm::quat clampedDelta = glm::normalize(
                constraintBasis * clampedConstraintDelta *
                glm::conjugate(constraintBasis));
            hipTarget = glm::normalize(
                skeleton.bones[leg.hipIdx].localR * clampedDelta);
        }

        if (solvedHipTarget) *solvedHipTarget = hipTarget;
        if (solvedKneeTarget) *solvedKneeTarget = kneeTarget;

        if (writePose) {
            animator.pose[leg.hipIdx].rotation = glm::normalize(glm::slerp(
                animator.pose[leg.hipIdx].rotation, hipTarget, weight));
            animator.pose[leg.kneeIdx].rotation = glm::normalize(glm::slerp(
                animator.pose[leg.kneeIdx].rotation, kneeTarget, weight));
        }

        if (debug) {
            DebugDraw::Sphere(ankleTarget, 0.025f, {0.3f, 0.7f, 1.0f});
            DebugDraw::Sphere(anklePos, 0.025f, {1.0f, 0.15f, 0.15f});
            DebugDraw::Line(anklePos, ankleTarget, {1.0f, 0.15f, 0.15f});
            DebugDraw::Line(hipPos, hipPos + chosenWorldBend * 0.25f,
                            {0.8f, 0.3f, 1.0f});
        }
    }

    static void DrawLeg(const LocamotionControllerComponent::LegState& leg)
    {
        DebugDraw::Sphere(leg.plantSole, 0.04f, {0.2f, 1.0f, 0.2f});
        DebugDraw::Sphere(leg.soleTarget, 0.025f, {1.0f, 1.0f, 1.0f});
        if (leg.phase == LocamotionControllerComponent::LegPhase::Swinging)
            DebugDraw::Sphere(leg.swingTargetSole, 0.04f, {1.0f, 0.6f, 0.1f});
    }
};
