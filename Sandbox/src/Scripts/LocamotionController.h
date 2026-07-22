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
    float facingOffsetDeg = 90.0f;
    float groundRayLength = 1.2f;
    float consciousness   = 1.0f;

    std::string leftFootBone  = "leg_joint_L_5";
    std::string rightFootBone = "leg_joint_R_5";

    float minWalkSpeed    = 0.15f;
    float stepLength      = 0.25f;
    float stanceHalfWidth = 0.10f;
    float stepDuration    = 0.40f;
    float stepLift        = 0.10f;
    float doubleSupportTime = 0.08f;
    float postPoweredGrace  = 0.4f;
    float gaitBlendTime     = 0.2f;
    float kneePoleDist      = 0.5f;
    float maxReachFraction  = 0.95f;
    float swingPoseWeight   = 0.35f;
    float landingTolerance  = 0.10f;

    bool ikWriteEnabled = true;
    bool debug = false;

    enum class LegPhase { Planted, Swinging };
    struct LegState {
        int footIdx  = -1;
        int ankleIdx = -1;
        int kneeIdx  = -1;
        int hipIdx   = -1;
        glm::vec3 hipTwistAxis  { 0.0f, 1.0f, 0.0f };
        glm::vec3 kneeHingeAxis { 1.0f, 0.0f, 0.0f };
        LegPhase phase = LegPhase::Planted;
        glm::vec3 plantSole       { 0.0f };
        glm::vec3 swingStartSole  { 0.0f };
        glm::vec3 swingTargetSole { 0.0f };
        glm::vec3 soleTarget      { 0.0f };
        float soleHeight = 0.03f;
        float swingProgress = 0.0f;
        float landingWait = 0.0f;
        bool landingWarning = false;
        bool init = false;
    };

    float _yaw = 0.0f;
    float _timeSincePowered = -1.0f;
    float _gaitWeight = 0.0f;
    float _driveWeight = 0.0f;
    float _timeSinceLanding = 0.0f;
    bool _grounded = true;
    bool _wasWalking = false;
    int _landedStepCount = 0;
    bool _nextLeft = true;
    LegState _legL, _legR;
};

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
    ImGui::DragFloat("Post-Powered Grace", &c.postPoweredGrace, 0.01f, 0.0f, 2.0f);
    ImGui::DragFloat("Gait Blend Time", &c.gaitBlendTime, 0.01f, 0.01f, 1.0f);
    ImGui::DragFloat("Knee Pole Distance", &c.kneePoleDist, 0.01f, 0.0f, 2.0f);
    ImGui::DragFloat("Max Reach Fraction", &c.maxReachFraction, 0.01f, 0.5f, 0.99f);
    ImGui::DragFloat("Swing Pose Weight", &c.swingPoseWeight, 0.01f, 0.0f, 1.0f);
    ImGui::DragFloat("Landing Tolerance", &c.landingTolerance, 0.01f, 0.02f, 0.3f);
    ImGui::Checkbox("IK Write Enabled", &c.ikWriteEnabled);
    ImGui::Checkbox("Debug", &c.debug);
    ImGui::TextDisabled(c._grounded ? "Grounded" : "Airborne");
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
    j["postPoweredGrace"] = c.postPoweredGrace;
    j["gaitBlendTime"] = c.gaitBlendTime;
    j["kneePoleDist"] = c.kneePoleDist;
    j["maxReachFraction"] = c.maxReachFraction;
    j["swingPoseWeight"] = c.swingPoseWeight;
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
    c.facingOffsetDeg = j.value("facingOffsetDeg", 90.0f);
    c.groundRayLength = j.value("groundRayLength", 1.2f);
    c.consciousness = j.value("consciousness", 1.0f);
    c.leftFootBone = j.value("leftFootBone", std::string("leg_joint_L_5"));
    c.rightFootBone = j.value("rightFootBone", std::string("leg_joint_R_5"));
    c.minWalkSpeed = j.value("minWalkSpeed", 0.15f);
    c.stepLength = j.value("stepLength", j.value("strideLength", 0.25f));
    c.stanceHalfWidth = j.value("stanceHalfWidth", 0.10f);
    c.stepDuration = j.value("stepDuration", 0.40f);
    c.stepLift = j.value("stepLift", 0.10f);
    c.doubleSupportTime = j.value("doubleSupportTime", 0.08f);
    c.postPoweredGrace = j.value("postPoweredGrace", 0.4f);
    c.gaitBlendTime = j.value("gaitBlendTime", 0.2f);
    c.kneePoleDist = j.value("kneePoleDist", 0.5f);
    c.maxReachFraction = j.value("maxReachFraction", 0.95f);
    c.swingPoseWeight = j.value("swingPoseWeight", 0.35f);
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
        Input::BindAxis("MoveX", GamepadAxis::LeftX);
        Input::BindAxis("MoveY", GamepadAxis::LeftY);
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
            const glm::vec3 requestedVel = moveDir * speed;
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
            }
            if (stoppedWalking) {
                comp._legL = {};
                comp._legR = {};
                comp._nextLeft = true;
            }
            if (!wantsToWalk || !canGait) comp._landedStepCount = 0;
            comp._wasWalking = wantsToWalk;

            const float driveTarget = gaitRequested && comp._landedStepCount >= 2 ? 1.0f : 0.0f;
            const float driveRate = 1.0f / glm::max(comp.moveRampTime, 0.01f);
            comp._driveWeight = Approach(comp._driveWeight, driveTarget, driveRate * dt);
            rag.locomotionTargetVel = requestedVel * comp._driveWeight;

            UpdateGait(scene, entity, comp, rag, moveDir, gaitRequested, dt);

            if (comp.debug) {
                _debugTimer += dt;
                if (_debugTimer >= 0.25f) {
                    _debugTimer = 0.0f;
                    const glm::vec3 v = rag._locomotionRootVel;
                    spdlog::info("[Loco] input={:.2f} requested={:.2f} drive={:.2f} actual={:.2f} gait={:.2f} tilt={:.1f} steps={}",
                                 magnitude, speed, comp._driveWeight,
                                 glm::length(glm::vec2(v.x, v.z)), comp._gaitWeight,
                                 tiltDeg, comp._landedStepCount);
                }
            }
        }
    }

private:
    float _debugTimer = 0.0f;

    static float Approach(float value, float target, float amount)
    {
        if (value < target) return glm::min(value + amount, target);
        return glm::max(value - amount, target);
    }

    static void ResetGait(LocamotionControllerComponent& c)
    {
        c._gaitWeight = 0.0f;
        c._driveWeight = 0.0f;
        c._wasWalking = false;
        c._landedStepCount = 0;
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

        if (!rag.config) return;
        for (const auto& body : rag.config->bodies) {
            if (leg.hipIdx >= 0 && body.boneName == skeleton.bones[leg.hipIdx].name)
                leg.hipTwistAxis = body.twistAxisLocal;
            if (leg.kneeIdx >= 0 && body.boneName == skeleton.bones[leg.kneeIdx].name)
                leg.kneeHingeAxis = body.twistAxisLocal;
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

    void UpdateGait(Scene& scene, entt::entity entity, LocamotionControllerComponent& comp,
                    const RagdollComponent& rag, const glm::vec3& moveDir,
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
        bool landed = AdvanceSwing(entity, comp._legL, comp, leftFootNow, dt, comp.debug, "left");
        landed |= AdvanceSwing(entity, comp._legR, comp, rightFootNow, dt, comp.debug, "right");
        if (landed) {
            ++comp._landedStepCount;
            comp._timeSinceLanding = 0.0f;
        } else {
            comp._timeSinceLanding += dt;
        }

        const bool anySwing = comp._legL.phase == LocamotionControllerComponent::LegPhase::Swinging ||
                              comp._legR.phase == LocamotionControllerComponent::LegPhase::Swinging;
        if (walking && !anySwing && comp._timeSinceLanding >= comp.doubleSupportTime) {
            auto& leg = comp._nextLeft ? comp._legL : comp._legR;
            const float side = comp._nextLeft ? -1.0f : 1.0f;
            const glm::vec3 right = glm::normalize(glm::cross(moveDir, glm::vec3(0, 1, 0)));
            const glm::vec3 horizontal = moveDir * comp.stepLength + right * (side * comp.stanceHalfWidth);
            const glm::vec3 footNow = BoneWorldPos(skeleton, animator, entityWorld, leg.footIdx);
            leg.swingStartSole = footNow - glm::vec3(0, leg.soleHeight, 0);
            const glm::vec3 rootPosition = BoneWorldPos(skeleton, animator, entityWorld, rootBone);
            leg.swingTargetSole = GroundedSoleTarget(scene, entity, rootPosition,
                                                     horizontal, comp.groundRayLength,
                                                     leg.swingStartSole.y);
            leg.swingProgress = 0.0f;
            leg.landingWait = 0.0f;
            leg.landingWarning = false;
            leg.phase = LocamotionControllerComponent::LegPhase::Swinging;
            comp._nextLeft = !comp._nextLeft;
            if (comp.debug)
                spdlog::info("[Loco] {} step to ({:+.2f}, {:+.2f}, {:+.2f})",
                             side < 0.0f ? "left" : "right", leg.swingTargetSole.x,
                             leg.swingTargetSole.y, leg.swingTargetSole.z);
        }

        if (comp._gaitWeight > 0.001f &&
            comp._legL.phase == LocamotionControllerComponent::LegPhase::Swinging) {
            SolveLeg(comp._legL, skeleton, animator, entityWorld, rootBone, moveDir,
                     comp.kneePoleDist, comp.maxReachFraction,
                     comp._gaitWeight * comp.swingPoseWeight,
                     comp.ikWriteEnabled, comp.debug);
        }
        if (comp._gaitWeight > 0.001f &&
            comp._legR.phase == LocamotionControllerComponent::LegPhase::Swinging) {
            SolveLeg(comp._legR, skeleton, animator, entityWorld, rootBone, moveDir,
                     comp.kneePoleDist, comp.maxReachFraction,
                     comp._gaitWeight * comp.swingPoseWeight,
                     comp.ikWriteEnabled, comp.debug);
        }

        if (comp.debug) {
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

    static bool AdvanceSwing(entt::entity entity,
                             LocamotionControllerComponent::LegState& leg,
                             const LocamotionControllerComponent& comp,
                             const glm::vec3& footPosition, float dt,
                             bool debug, const char* side)
    {
        if (leg.phase != LocamotionControllerComponent::LegPhase::Swinging) {
            leg.soleTarget = leg.plantSole;
            return false;
        }

        leg.swingProgress = glm::min(leg.swingProgress + dt / glm::max(comp.stepDuration, 0.01f), 1.0f);
        const float t = leg.swingProgress;
        const float smooth = t * t * (3.0f - 2.0f * t);
        leg.soleTarget = glm::mix(leg.swingStartSole, leg.swingTargetSole, smooth);
        leg.soleTarget.y += comp.stepLift * std::sin(glm::pi<float>() * t);
        if (t < 1.0f) return false;

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
            if (debug && !leg.landingWarning && leg.landingWait >= 0.25f) {
                leg.landingWarning = true;
                spdlog::warn("[Loco] {} foot has not landed: horizontal={:.3f} ground={} groundError={:.3f} clearance={:.3f}",
                             side, horizontalError, ground.hit, groundError, leg.soleHeight);
            }
            return false;
        }

        leg.plantSole = leg.swingTargetSole;
        leg.soleTarget = leg.plantSole;
        leg.phase = LocamotionControllerComponent::LegPhase::Planted;
        return true;
    }

    static void SolveLeg(const LocamotionControllerComponent::LegState& leg,
                         const Diamond::Skeleton& skeleton, AnimatorComponent& animator,
                         const glm::mat4& entityWorld, int rootBone, const glm::vec3& moveDir,
                         float poleDistance, float reachFraction, float weight,
                         bool writePose, bool debug)
    {
        const glm::vec3 hipPos = BoneWorldPos(skeleton, animator, entityWorld, leg.hipIdx);
        const glm::vec3 kneePos = BoneWorldPos(skeleton, animator, entityWorld, leg.kneeIdx);
        const glm::vec3 anklePos = BoneWorldPos(skeleton, animator, entityWorld, leg.ankleIdx);
        const glm::vec3 footPos = BoneWorldPos(skeleton, animator, entityWorld, leg.footIdx);
        const glm::quat hipRot = BoneWorldRot(skeleton, animator, entityWorld, leg.hipIdx);
        const glm::quat kneeRot = BoneWorldRot(skeleton, animator, entityWorld, leg.kneeIdx);

        const float upperLength = glm::length(skeleton.bones[leg.kneeIdx].localT);
        const float lowerLength = glm::length(skeleton.bones[leg.ankleIdx].localT);
        if (upperLength < 1e-4f || lowerLength < 1e-4f) return;

        const glm::vec3 desiredFootCenter = leg.soleTarget + glm::vec3(0, leg.soleHeight, 0);
        glm::vec3 ankleTarget = desiredFootCenter + (anklePos - footPos);
        glm::vec3 toTarget = ankleTarget - hipPos;
        const float maxReach = (upperLength + lowerLength) * glm::clamp(reachFraction, 0.5f, 0.99f);
        if (const float reach = glm::length(toTarget); reach > maxReach && reach > 1e-4f)
            ankleTarget = hipPos + toTarget * (maxReach / reach);

        glm::vec3 poleDir = moveDir;
        if (glm::length(poleDir) < 1e-4f) poleDir = glm::vec3(0, 0, -1);
        const glm::vec3 pole = kneePos + poleDir * poleDistance;

        glm::quat solvedHip, ignoredKnee;
        Diamond::SolveTwoBone(hipPos, kneePos, anklePos, hipRot, kneeRot,
                              ankleTarget, pole, solvedHip, ignoredKnee);

        const glm::quat rootRot = BoneWorldRot(skeleton, animator, entityWorld, rootBone);
        const glm::quat rawHipLocal = glm::normalize(glm::conjugate(rootRot) * solvedHip);
        const glm::quat hipBind = skeleton.bones[leg.hipIdx].localR;
        const glm::quat bindDelta = glm::normalize(glm::conjugate(hipBind) * rawHipLocal);
        const glm::vec3 twistAxis = glm::normalize(leg.hipTwistAxis);
        const glm::vec3 projected = twistAxis * glm::dot(glm::vec3(bindDelta.x, bindDelta.y, bindDelta.z), twistAxis);
        glm::quat twist(bindDelta.w, projected.x, projected.y, projected.z);
        if (glm::length(twist) < 1e-5f) twist = glm::quat(1, 0, 0, 0);
        else twist = glm::normalize(twist);
        const glm::quat hipTarget = glm::normalize(hipBind * bindDelta * glm::conjugate(twist));

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

        if (writePose) {
            animator.pose[leg.hipIdx].rotation = glm::normalize(glm::slerp(
                animator.pose[leg.hipIdx].rotation, hipTarget, weight));
            animator.pose[leg.kneeIdx].rotation = glm::normalize(glm::slerp(
                animator.pose[leg.kneeIdx].rotation, kneeTarget, weight));
        }

        if (debug) {
            DebugDraw::Sphere(ankleTarget, 0.025f, {0.3f, 0.7f, 1.0f});
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
