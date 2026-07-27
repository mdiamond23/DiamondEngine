#pragma once

#include "Scene/Scripting.h"
#include "Scene/Scene.h"
#include "Scene/ComponentRegistry.h"
#include "Scene/Components.h"
#include "Core/Input.h"
#include "Scene/Physics/PhysicsAPI.h"
#include "Scene/Physics/RagdollComponent.h"
#include "Animation/AnimationComponents.h"
#include "DebugDraw.h"
#include "CameraDirector.h"

#include <imgui.h>
#include <nlohmann/json.hpp>
#include <glm/glm.hpp>
#include <glm/gtc/constants.hpp>
#include <glm/gtc/quaternion.hpp>
#include <spdlog/spdlog.h>
#include <cmath>
#include <string>

// SIMBICON (Yin et al. 2007) section 3, on a powered ragdoll. Each FSM state is a target
// pose; the swing hip and torso track in the WORLD frame while everything else is
// parent-relative; balance is the feedback law theta_d = theta_d0 + cd*d + cv*v applied to
// the swing hip in both planes. There is no foothold planner and no IK for the base gait --
// where the foot lands IS the balance decision. IK returns only as a bounded terrain
// displacement on top of the target angles (paper section 8), zero on flat ground.
struct LocamotionControllerComponent
{
    float maxSpeed        = 1.0f;
    float turnRate        = 10.0f;
    float deadzone        = 0.2f;
    float minWalkSpeed    = 0.15f;
    float facingOffsetDeg = 90.0f;
    float groundRayLength = 1.2f;
    float consciousness   = 1.0f;
    float postPoweredGrace = 0.4f;
    float gaitBlendTime   = 0.6f;

    // Assisted stepping is the bring-up controller: keep the stance joints and engine
    // balance active, prove an airborne swing and a real landing, then transfer support.
    // The torque-only SIMBICON path remains below it for later reintroduction.
    bool  assistedStepping = true;
    float weightShiftTime = 0.30f;
    float assistedLiftTime = 0.35f;
    float assistedLiftHeight = 0.12f;
    float assistedTakeoffClearance = 0.06f;
    float assistedLiftFrequency = 3.5f;
    float assistedLiftMaxForce = 325.0f;
    float assistedTransferTime = 0.20f;
    float assistedPlantTimeout = 0.65f;
    float assistedPlantHipDeg = 18.0f;
    float airborneConfirmTime = 0.05f;
    float minPlantForward = 0.04f;
    float maxPlantVerticalSpeed = 2.0f;

    std::string leftFootBone  = "leg_joint_L_5";
    std::string rightFootBone = "leg_joint_R_5";
    std::string torsoBone     = "torso_joint_3";

    // Table 1, "3D walk". State 0 lifts the swing leg for a fixed time; state 1 drives it
    // down and exits on contact. Angles in degrees here, radians in the paper.
    float state0Time     = 0.30f;
    float minSwingTime   = 0.08f;
    float maxSwingTime   = 0.60f;

    float s0TorsoDeg       = 0.0f;
    float s0SwingHipDeg    = 28.6f;   // 0.5 rad
    float s0SwingKneeDeg   = 63.0f;   // 1.1 rad of flexion
    float s0SwingAnkleDeg  = 34.4f;   // 0.6 rad
    float s0StanceKneeDeg  = 2.9f;    // 0.05 rad
    float s0StanceAnkleDeg = 0.0f;

    float s1TorsoDeg       = 0.0f;
    float s1SwingHipDeg    = -5.7f;   // -0.1 rad
    float s1SwingKneeDeg   = 2.9f;
    float s1SwingAnkleDeg  = 8.6f;    // 0.15 rad
    float s1StanceKneeDeg  = 5.7f;
    float s1StanceAnkleDeg = 0.0f;

    // Balance feedback. Paper's stable ranges: sagittal cd [-0.71,1.4] cv [0.03,0.59],
    // coronal cd [-1.29,1.13] cv [-0.06,0.48]. Nominal 0.5/0.2 in both planes.
    float cd    = 0.5f;
    float cv    = 0.2f;
    float cdLat = 0.5f;
    float cvLat = 0.2f;
    float stanceWidthDeg = 6.0f;
    float stanceHipDeg   = 0.0f;      // stand-in for the free stance-hip torque
    bool  useHipCOMProxy = true;

    // Virtual PD gains (paper 3.2). tau_torso holds pelvis attitude in the world frame;
    // tau_B drives the swing femur to its world target; the stance hip takes the residual
    // tau_A = -tau_torso - tau_B, which is what propels the body using only internal torque.
    float torsoKp = 100.0f;
    float torsoKd = 10.0f;
    float hipKp   = 100.0f;
    float hipKd   = 10.0f;
    float maxVirtualTorque = 250.0f;
    float supportScale = 1.0f;   // engine vertical hip assist; dial toward 0

    float swingHipLimitDeg = 55.0f;
    float hipLimitMarginDeg = 3.0f;
    float poseWeight = 1.0f;
    float uprightScale = 1.0f;        // 1 = engine rights the pelvis, 0 = torso joint alone

    // Terrain displacement layer (paper section 8). Off until the flat-ground limit cycle holds.
    bool  ikTerrainEnabled = false;
    float ikMaxOffsetDeg   = 12.0f;
    float ikGroundProbe    = 0.45f;

    bool ikWriteEnabled = true;
    bool debug = false;

    struct LegState {
        int footIdx = -1, ankleIdx = -1, kneeIdx = -1, hipIdx = -1;
        glm::vec3 kneeHingeAxis { 1.0f, 0.0f, 0.0f };
        glm::vec3 ankleAxis { 1.0f, 0.0f, 0.0f };
        glm::vec3 hipTwistAxis { 1.0f, 0.0f, 0.0f };
        float hipSwingNormalDeg = 60.0f, hipSwingPlaneDeg = 60.0f;
        float hipTwistMinDeg = -45.0f, hipTwistMaxDeg = 45.0f;
        float ankleSwingNormalDeg = 60.0f, ankleSwingPlaneDeg = 45.0f;
        float ankleTwistMinDeg = -45.0f, ankleTwistMaxDeg = 45.0f;
    };

    float _yaw = 0.0f;
    float _timeSincePowered = -1.0f;
    float _gaitWeight = 0.0f;
    float _poseBlend = 0.0f;
    float _stateTime = 0.0f;
    int   _stateIndex = 0;
    bool  _swingIsLeft = true;
    bool  _grounded = true;
    bool  _wasWalking = false;
    bool  _swingWasAirborne = false;
    float _airborneTime = 0.0f;
    int   _steps = 0;
    float _dSag = 0.0f, _dLat = 0.0f, _vSag = 0.0f, _vLat = 0.0f;
    float _swingHipCmdDeg = 0.0f, _swingHipLatCmdDeg = 0.0f;
    float _ikOffsetDeg = 0.0f;
    float _torsoTorque = 0.0f, _swingTorque = 0.0f, _stanceTorque = 0.0f;
    // Frame diagnostics. _myTiltDeg vs _engineTiltDeg is the falsifiable test of whether
    // entityRot * BindModelRot(bone) really is the body's bind rotation -- every world-frame
    // target assumes it does, and nothing has ever checked.
    float _myTiltDeg = 0.0f, _engineTiltDeg = 0.0f;
    glm::vec3 _right { 0.0f }, _fwd { 0.0f };
    glm::vec3 _femurCmd { 0.0f }, _femurActual { 0.0f };
    float _femurErrDeg = 0.0f, _torsoErrDeg = 0.0f;
    bool _swingContact = false;
    float _swingForward = 0.0f, _swingFootY = 0.0f, _swingFootVy = 0.0f;
    float _swingClearance = 0.0f;
    float _liftTargetY = 0.0f;
    float _torsoP = 0.0f, _torsoD = 0.0f, _swingP = 0.0f, _swingD = 0.0f;
    bool  _saturated = false;
    LegState _legL, _legR;
};

template<>
inline void DrawComponentInspector<LocamotionControllerComponent>(LocamotionControllerComponent& c)
{
    ImGui::DragFloat("Max Speed", &c.maxSpeed, 0.05f, 0.0f, 10.0f);
    ImGui::DragFloat("Turn Rate", &c.turnRate, 0.1f, 0.0f, 30.0f);
    ImGui::DragFloat("Deadzone", &c.deadzone, 0.01f, 0.0f, 0.9f);
    ImGui::DragFloat("Min Walk Speed", &c.minWalkSpeed, 0.01f, 0.0f, 2.0f);
    ImGui::DragFloat("Facing Offset", &c.facingOffsetDeg, 1.0f, -180.0f, 180.0f, "%.0f deg");
    ImGui::DragFloat("Ground Ray", &c.groundRayLength, 0.05f, 0.1f, 10.0f);
    ImGui::DragFloat("Consciousness", &c.consciousness, 0.01f, 0.0f, 1.0f);
    ImGui::DragFloat("Post-Powered Grace", &c.postPoweredGrace, 0.01f, 0.0f, 2.0f);
    ImGui::DragFloat("Gait Blend Time", &c.gaitBlendTime, 0.01f, 0.01f, 1.0f);

    auto boneField = [](const char* label, std::string& name) {
        char buf[128];
        std::snprintf(buf, sizeof(buf), "%s", name.c_str());
        if (ImGui::InputText(label, buf, sizeof(buf))) name = buf;
    };
    ImGui::Separator();
    boneField("Left Foot Bone", c.leftFootBone);
    boneField("Right Foot Bone", c.rightFootBone);
    boneField("Torso Bone", c.torsoBone);

    if (ImGui::CollapsingHeader("Assisted Step Bring-up", ImGuiTreeNodeFlags_DefaultOpen)) {
        ImGui::Checkbox("Assisted Stepping", &c.assistedStepping);
        ImGui::DragFloat("Weight Shift Time", &c.weightShiftTime, 0.01f, 0.05f, 1.0f);
        ImGui::DragFloat("Lift / Reach Time", &c.assistedLiftTime, 0.01f, 0.1f, 1.5f);
        ImGui::DragFloat("Lift Height", &c.assistedLiftHeight, 0.005f, 0.02f, 0.4f, "%.3f m");
        ImGui::DragFloat("Takeoff Clearance", &c.assistedTakeoffClearance, 0.005f,
                         0.01f, 0.2f, "%.3f m");
        ImGui::DragFloat("Lift Frequency", &c.assistedLiftFrequency, 0.25f, 0.0f, 20.0f);
        ImGui::DragFloat("Lift Max Force", &c.assistedLiftMaxForce, 10.0f, 0.0f, 3000.0f);
        ImGui::DragFloat("Transfer Time", &c.assistedTransferTime, 0.01f, 0.05f, 1.0f);
        ImGui::DragFloat("Plant Timeout", &c.assistedPlantTimeout, 0.01f, 0.1f, 2.0f);
        ImGui::DragFloat("Plant Hip", &c.assistedPlantHipDeg, 0.5f, 0.0f, 45.0f, "%.1f deg");
        ImGui::DragFloat("Airborne Confirm", &c.airborneConfirmTime, 0.01f, 0.0f, 0.25f);
        ImGui::DragFloat("Min Plant Forward", &c.minPlantForward, 0.005f, 0.0f, 0.4f, "%.3f m");
        ImGui::DragFloat("Max Plant Y Speed", &c.maxPlantVerticalSpeed, 0.1f, 0.1f, 10.0f);
        ImGui::TextDisabled("contact=%s airborne=%s forward=%.3f clear=%.3f y=%.3f/%.3f vy=%+.2f",
                            c._swingContact ? "yes" : "no",
                            c._swingWasAirborne ? "yes" : "no",
                            c._swingForward, c._swingClearance,
                            c._swingFootY, c._liftTargetY, c._swingFootVy);
    }

    if (ImGui::CollapsingHeader("FSM", ImGuiTreeNodeFlags_DefaultOpen)) {
        ImGui::DragFloat("State 0 Time", &c.state0Time, 0.01f, 0.05f, 1.0f);
        ImGui::DragFloat("Min Swing Time", &c.minSwingTime, 0.01f, 0.0f, 0.5f);
        ImGui::DragFloat("Max Swing Time", &c.maxSwingTime, 0.01f, 0.1f, 2.0f);
        ImGui::TextDisabled("State 0 - lift");
        ImGui::DragFloat("S0 Torso", &c.s0TorsoDeg, 0.5f, -45.0f, 45.0f, "%.1f deg");
        ImGui::DragFloat("S0 Swing Hip", &c.s0SwingHipDeg, 0.5f, -60.0f, 90.0f, "%.1f deg");
        ImGui::DragFloat("S0 Swing Knee", &c.s0SwingKneeDeg, 0.5f, 0.0f, 140.0f, "%.1f deg");
        ImGui::DragFloat("S0 Swing Ankle", &c.s0SwingAnkleDeg, 0.5f, -60.0f, 60.0f, "%.1f deg");
        ImGui::DragFloat("S0 Stance Knee", &c.s0StanceKneeDeg, 0.5f, 0.0f, 140.0f, "%.1f deg");
        ImGui::DragFloat("S0 Stance Ankle", &c.s0StanceAnkleDeg, 0.5f, -60.0f, 60.0f, "%.1f deg");
        ImGui::TextDisabled("State 1 - plant, exits on contact");
        ImGui::DragFloat("S1 Torso", &c.s1TorsoDeg, 0.5f, -45.0f, 45.0f, "%.1f deg");
        ImGui::DragFloat("S1 Swing Hip", &c.s1SwingHipDeg, 0.5f, -60.0f, 90.0f, "%.1f deg");
        ImGui::DragFloat("S1 Swing Knee", &c.s1SwingKneeDeg, 0.5f, 0.0f, 140.0f, "%.1f deg");
        ImGui::DragFloat("S1 Swing Ankle", &c.s1SwingAnkleDeg, 0.5f, -60.0f, 60.0f, "%.1f deg");
        ImGui::DragFloat("S1 Stance Knee", &c.s1StanceKneeDeg, 0.5f, 0.0f, 140.0f, "%.1f deg");
        ImGui::DragFloat("S1 Stance Ankle", &c.s1StanceAnkleDeg, 0.5f, -60.0f, 60.0f, "%.1f deg");
    }

    if (ImGui::CollapsingHeader("Balance Feedback", ImGuiTreeNodeFlags_DefaultOpen)) {
        ImGui::SliderFloat("cd (sagittal)", &c.cd, -0.71f, 1.4f);
        ImGui::SliderFloat("cv (sagittal)", &c.cv, 0.03f, 0.59f);
        ImGui::SliderFloat("cd (coronal)", &c.cdLat, -1.29f, 1.13f);
        ImGui::SliderFloat("cv (coronal)", &c.cvLat, -0.06f, 0.48f);
        ImGui::DragFloat("Stance Width", &c.stanceWidthDeg, 0.5f, 0.0f, 30.0f, "%.1f deg");
        ImGui::DragFloat("Stance Hip", &c.stanceHipDeg, 0.5f, -30.0f, 30.0f, "%.1f deg");
        ImGui::Checkbox("Hip COM Proxy", &c.useHipCOMProxy);
        ImGui::TextDisabled("Virtual PD (tau_torso / tau_B / tau_A)");
        ImGui::DragFloat("Torso Kp", &c.torsoKp, 10.0f, 0.0f, 2000.0f);
        ImGui::DragFloat("Torso Kd", &c.torsoKd, 1.0f, 0.0f, 200.0f);
        ImGui::DragFloat("Hip Kp", &c.hipKp, 10.0f, 0.0f, 2000.0f);
        ImGui::DragFloat("Hip Kd", &c.hipKd, 1.0f, 0.0f, 200.0f);
        ImGui::DragFloat("Max Virtual Torque", &c.maxVirtualTorque, 10.0f, 0.0f, 2000.0f);
        ImGui::SliderFloat("Support Scale", &c.supportScale, 0.0f, 1.0f);
        ImGui::TextDisabled("d=(%.3f, %.3f)  v=(%.3f, %.3f)", c._dSag, c._dLat, c._vSag, c._vLat);
        ImGui::TextDisabled("swing hip cmd = %.1f / %.1f deg",
                            c._swingHipCmdDeg, c._swingHipLatCmdDeg);
    }

    ImGui::Separator();
    ImGui::DragFloat("Swing Hip Limit", &c.swingHipLimitDeg, 1.0f, 10.0f, 90.0f, "%.0f deg");
    ImGui::DragFloat("Hip Limit Margin", &c.hipLimitMarginDeg, 0.5f, 0.0f, 20.0f, "%.1f deg");
    ImGui::SliderFloat("Pose Weight", &c.poseWeight, 0.0f, 1.0f);
    ImGui::SliderFloat("Upright Scale", &c.uprightScale, 0.0f, 1.0f);
    ImGui::Checkbox("IK Terrain Layer", &c.ikTerrainEnabled);
    ImGui::DragFloat("IK Max Offset", &c.ikMaxOffsetDeg, 0.5f, 0.0f, 45.0f, "%.1f deg");
    ImGui::DragFloat("IK Ground Probe", &c.ikGroundProbe, 0.01f, 0.05f, 1.0f);
    ImGui::Checkbox("IK Write Enabled", &c.ikWriteEnabled);
    ImGui::Checkbox("Debug", &c.debug);
    ImGui::TextDisabled(c._grounded ? "Grounded" : "Airborne");
    ImGui::TextDisabled("state %d  swing=%s  steps=%d  gait=%.2f  pose=%.2f", c._stateIndex,
                        c._swingIsLeft ? "left" : "right", c._steps, c._gaitWeight, c._poseBlend);
    ImGui::Separator();
    ImGui::TextDisabled("Frame check");
    const float tiltDelta = c._myTiltDeg - c._engineTiltDeg;
    ImGui::TextColored(std::abs(tiltDelta) > 1.0f ? ImVec4(1, 0.4f, 0.3f, 1) : ImVec4(0.4f, 1, 0.5f, 1),
                       "tilt mine=%.1f engine=%.1f delta=%+.1f%s", c._myTiltDeg, c._engineTiltDeg,
                       tiltDelta, std::abs(tiltDelta) > 1.0f ? "  <- BIND FRAME WRONG" : "  ok");
    ImGui::TextDisabled("right   (%+.2f, %+.2f, %+.2f)", c._right.x, c._right.y, c._right.z);
    ImGui::TextDisabled("fwd     (%+.2f, %+.2f, %+.2f)", c._fwd.x, c._fwd.y, c._fwd.z);
    ImGui::TextDisabled("femur cmd    (%+.2f, %+.2f, %+.2f)", c._femurCmd.x, c._femurCmd.y, c._femurCmd.z);
    ImGui::TextDisabled("femur actual (%+.2f, %+.2f, %+.2f)", c._femurActual.x, c._femurActual.y, c._femurActual.z);
    ImGui::TextDisabled("femur error  %.1f deg", c._femurErrDeg);
}

template<>
inline std::string SerializeComponent<LocamotionControllerComponent>(const LocamotionControllerComponent& c)
{
    nlohmann::json j;
    j["maxSpeed"] = c.maxSpeed;
    j["turnRate"] = c.turnRate;
    j["deadzone"] = c.deadzone;
    j["minWalkSpeed"] = c.minWalkSpeed;
    j["facingOffsetDeg"] = c.facingOffsetDeg;
    j["groundRayLength"] = c.groundRayLength;
    j["consciousness"] = c.consciousness;
    j["postPoweredGrace"] = c.postPoweredGrace;
    j["gaitBlendTime"] = c.gaitBlendTime;
    j["assistedStepping"] = c.assistedStepping;
    j["weightShiftTime"] = c.weightShiftTime;
    j["assistedLiftTime"] = c.assistedLiftTime;
    j["assistedLiftHeight"] = c.assistedLiftHeight;
    j["assistedTakeoffClearance"] = c.assistedTakeoffClearance;
    j["assistedLiftFrequency"] = c.assistedLiftFrequency;
    j["assistedLiftMaxForce"] = c.assistedLiftMaxForce;
    j["assistedTransferTime"] = c.assistedTransferTime;
    j["assistedPlantTimeout"] = c.assistedPlantTimeout;
    j["assistedPlantHipDeg"] = c.assistedPlantHipDeg;
    j["airborneConfirmTime"] = c.airborneConfirmTime;
    j["minPlantForward"] = c.minPlantForward;
    j["maxPlantVerticalSpeed"] = c.maxPlantVerticalSpeed;
    j["leftFootBone"] = c.leftFootBone;
    j["rightFootBone"] = c.rightFootBone;
    j["torsoBone"] = c.torsoBone;
    j["state0Time"] = c.state0Time;
    j["minSwingTime"] = c.minSwingTime;
    j["maxSwingTime"] = c.maxSwingTime;
    j["s0TorsoDeg"] = c.s0TorsoDeg;
    j["s0SwingHipDeg"] = c.s0SwingHipDeg;
    j["s0SwingKneeDeg"] = c.s0SwingKneeDeg;
    j["s0SwingAnkleDeg"] = c.s0SwingAnkleDeg;
    j["s0StanceKneeDeg"] = c.s0StanceKneeDeg;
    j["s0StanceAnkleDeg"] = c.s0StanceAnkleDeg;
    j["s1TorsoDeg"] = c.s1TorsoDeg;
    j["s1SwingHipDeg"] = c.s1SwingHipDeg;
    j["s1SwingKneeDeg"] = c.s1SwingKneeDeg;
    j["s1SwingAnkleDeg"] = c.s1SwingAnkleDeg;
    j["s1StanceKneeDeg"] = c.s1StanceKneeDeg;
    j["s1StanceAnkleDeg"] = c.s1StanceAnkleDeg;
    j["cd"] = c.cd;
    j["cv"] = c.cv;
    j["cdLat"] = c.cdLat;
    j["cvLat"] = c.cvLat;
    j["stanceWidthDeg"] = c.stanceWidthDeg;
    j["stanceHipDeg"] = c.stanceHipDeg;
    j["useHipCOMProxy"] = c.useHipCOMProxy;
    j["torsoKp"] = c.torsoKp;
    j["torsoKd"] = c.torsoKd;
    j["hipKp"] = c.hipKp;
    j["hipKd"] = c.hipKd;
    j["maxVirtualTorque"] = c.maxVirtualTorque;
    j["supportScale"] = c.supportScale;
    j["swingHipLimitDeg"] = c.swingHipLimitDeg;
    j["hipLimitMarginDeg"] = c.hipLimitMarginDeg;
    j["poseWeight"] = c.poseWeight;
    j["uprightScale"] = c.uprightScale;
    j["ikTerrainEnabled"] = c.ikTerrainEnabled;
    j["ikMaxOffsetDeg"] = c.ikMaxOffsetDeg;
    j["ikGroundProbe"] = c.ikGroundProbe;
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
    c.turnRate = j.value("turnRate", 10.0f);
    c.deadzone = j.value("deadzone", 0.2f);
    c.minWalkSpeed = j.value("minWalkSpeed", 0.15f);
    c.facingOffsetDeg = j.value("facingOffsetDeg", 90.0f);
    c.groundRayLength = j.value("groundRayLength", 1.2f);
    c.consciousness = j.value("consciousness", 1.0f);
    c.postPoweredGrace = j.value("postPoweredGrace", 0.4f);
    c.gaitBlendTime = j.value("gaitBlendTime", 0.6f);
    c.assistedStepping = j.value("assistedStepping", true);
    c.weightShiftTime = j.value("weightShiftTime", 0.30f);
    c.assistedLiftTime = j.value("assistedLiftTime", 0.35f);
    c.assistedLiftHeight = j.value("assistedLiftHeight", 0.12f);
    c.assistedTakeoffClearance = j.value("assistedTakeoffClearance", 0.06f);
    c.assistedLiftFrequency = j.value("assistedLiftFrequency", 3.5f);
    c.assistedLiftMaxForce = j.value("assistedLiftMaxForce", 325.0f);
    c.assistedTransferTime = j.value("assistedTransferTime", 0.20f);
    c.assistedPlantTimeout = j.value("assistedPlantTimeout", 0.65f);
    c.assistedPlantHipDeg = j.value("assistedPlantHipDeg", 18.0f);
    c.airborneConfirmTime = j.value("airborneConfirmTime", 0.05f);
    c.minPlantForward = j.value("minPlantForward", 0.04f);
    c.maxPlantVerticalSpeed = j.value("maxPlantVerticalSpeed", 2.0f);
    c.leftFootBone = j.value("leftFootBone", std::string("leg_joint_L_5"));
    c.rightFootBone = j.value("rightFootBone", std::string("leg_joint_R_5"));
    c.torsoBone = j.value("torsoBone", std::string("torso_joint_3"));
    c.state0Time = j.value("state0Time", 0.30f);
    c.minSwingTime = j.value("minSwingTime", 0.08f);
    c.maxSwingTime = j.value("maxSwingTime", 0.60f);
    c.s0TorsoDeg = j.value("s0TorsoDeg", 0.0f);
    c.s0SwingHipDeg = j.value("s0SwingHipDeg", 28.6f);
    c.s0SwingKneeDeg = j.value("s0SwingKneeDeg", 63.0f);
    c.s0SwingAnkleDeg = j.value("s0SwingAnkleDeg", 34.4f);
    c.s0StanceKneeDeg = j.value("s0StanceKneeDeg", 2.9f);
    c.s0StanceAnkleDeg = j.value("s0StanceAnkleDeg", 0.0f);
    c.s1TorsoDeg = j.value("s1TorsoDeg", 0.0f);
    c.s1SwingHipDeg = j.value("s1SwingHipDeg", -5.7f);
    c.s1SwingKneeDeg = j.value("s1SwingKneeDeg", 2.9f);
    c.s1SwingAnkleDeg = j.value("s1SwingAnkleDeg", 8.6f);
    c.s1StanceKneeDeg = j.value("s1StanceKneeDeg", 5.7f);
    c.s1StanceAnkleDeg = j.value("s1StanceAnkleDeg", 0.0f);
    c.cd = j.value("cd", 0.5f);
    c.cv = j.value("cv", 0.2f);
    c.cdLat = j.value("cdLat", 0.5f);
    c.cvLat = j.value("cvLat", 0.2f);
    c.stanceWidthDeg = j.value("stanceWidthDeg", 6.0f);
    c.stanceHipDeg = j.value("stanceHipDeg", 0.0f);
    c.useHipCOMProxy = j.value("useHipCOMProxy", true);
    c.torsoKp = j.value("torsoKp", 100.0f);
    c.torsoKd = j.value("torsoKd", 10.0f);
    c.hipKp = j.value("hipKp", 100.0f);
    c.hipKd = j.value("hipKd", 10.0f);
    c.maxVirtualTorque = j.value("maxVirtualTorque", 250.0f);
    c.supportScale = j.value("supportScale", 1.0f);
    c.swingHipLimitDeg = j.value("swingHipLimitDeg", 55.0f);
    c.hipLimitMarginDeg = j.value("hipLimitMarginDeg", 3.0f);
    c.poseWeight = j.value("poseWeight", 1.0f);
    c.uprightScale = j.value("uprightScale", 1.0f);
    c.ikTerrainEnabled = j.value("ikTerrainEnabled", false);
    c.ikMaxOffsetDeg = j.value("ikMaxOffsetDeg", 12.0f);
    c.ikGroundProbe = j.value("ikGroundProbe", 0.45f);
    c.ikWriteEnabled = j.value("ikWriteEnabled", true);
    c.debug = j.value("debug", false);
}

DECLARE_COMPONENT(LocamotionControllerComponent, "LocamotionController")

class LocamotionControllerSystem : public GameSystem
{
    DECLARE_SYSTEM(LocamotionControllerSystem, 100)

    using Comp = LocamotionControllerComponent;
    using Leg  = LocamotionControllerComponent::LegState;

public:
    void OnStart(Scene&) override
    {
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

        for (auto [entity, comp] : scene.View<Comp>().each()) {
            if (!scene.Has<RagdollComponent>(entity) || !scene.Has<TransformComponent>(entity)) continue;
            auto& rag = scene.Get<RagdollComponent>(entity);
            auto& xf = scene.Get<TransformComponent>(entity);

            if (rag.mode == RagdollMode::Animated) {
                Physics::SetRagdollMode(rag, RagdollMode::Powered);
                comp._timeSincePowered = 0.0f;
            }
            if (rag.mode != RagdollMode::Powered) {
                rag.locomotionActive = false;
                rag.locomotionSimbicon = false;
                rag.locomotionSimbiconBlend = 0.0f;
                rag.locomotionHipTorque[0] = rag.locomotionHipTorque[1] = glm::vec3(0.0f);
                rag.locomotionHipBones[0] = rag.locomotionHipBones[1] = -1;
                rag.locomotionLiftBone = -1;
                ResetGait(comp);
                continue;
            }

            // A scene can author the ragdoll as Powered, in which case the Animated->Powered
            // flip never runs and the -1 sentinel would double the grace period.
            if (comp._timeSincePowered < 0.0f) comp._timeSincePowered = 0.0f;
            comp._timeSincePowered += dt;
            rag.strength = glm::clamp(comp.consciousness, 0.0f, 1.0f);

            const float speed = magnitude > comp.deadzone
                ? (magnitude - comp.deadzone) / (1.0f - comp.deadzone) * comp.maxSpeed
                : 0.0f;
            glm::vec3 moveDir = camRight * h + camForward * f;
            moveDir.y = 0.0f;
            moveDir = glm::length(moveDir) > 1e-4f ? glm::normalize(moveDir) : glm::vec3(0.0f);
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
            rag.locomotionUprightScale = comp.uprightScale;

            const float tiltDeg = Physics::GetRagdollTiltDeg(rag);
            const bool ready = comp._grounded && comp._timeSincePowered >= comp.postPoweredGrace;
            // Deliberately NOT gated on fallen tilt. Freezing the FSM mid-stumble latched a
            // stale one-legged pose at exactly the moment the balance law wanted its biggest
            // catch step; the engine's root drive still bails at locomotionFallenTilt.
            const bool gaitRequested = wantsToWalk && ready;
            const float gaitRate = 1.0f / glm::max(comp.gaitBlendTime, 0.01f);
            comp._gaitWeight = Approach(comp._gaitWeight, gaitRequested ? 1.0f : 0.0f, gaitRate * dt);

            // The controller holds a standing pose whenever it is upright, so gait engagement
            // differs from standing by one leg rather than by the whole body.
            const float poseTarget = ready && tiltDeg < rag.locomotionFallenTilt ? 1.0f : 0.0f;
            comp._poseBlend = Approach(comp._poseBlend, poseTarget, gaitRate * dt);

            if (wantsToWalk && !comp._wasWalking) {
                comp._stateIndex = 0;
                comp._stateTime = 0.0f;
                comp._steps = 0;
                comp._swingWasAirborne = false;
                comp._airborneTime = 0.0f;
            }
            comp._wasWalking = wantsToWalk;

            // The gait owns balance while stepping; the engine's COM-over-base forces are
            // only correct when there is no swing leg to place.
            // Crossfade, matched to the torque ramp: engine assists fade out exactly as the
            // virtual torques fade in, so total balance authority never dips.
            const bool torqueGait = gaitRequested && !comp.assistedStepping;
            rag.locomotionSimbicon = torqueGait;
            rag.locomotionSimbiconBlend = torqueGait ? comp._gaitWeight : 0.0f;
            rag.locomotionSupportTargetWeight = 0.0f;
            rag.locomotionTargetVel = glm::vec3(0.0f);
            rag.locomotionHipTorque[0] = rag.locomotionHipTorque[1] = glm::vec3(0.0f);
            rag.locomotionHipBones[0] = rag.locomotionHipBones[1] = -1;
            rag.locomotionLiftBone = -1;

            UpdateGait(scene, entity, comp, rag, gaitRequested, dt);

            if (comp.debug) {
                _debugTimer += dt;
                if (_debugTimer >= 0.25f) {
                    _debugTimer = 0.0f;
                    const glm::vec3 v = rag._locomotionRootVel;
                    spdlog::info("[Loco] s{} swing={} t={:.2f} gait={:.2f} pose={:.2f} d=({:+.3f},{:+.3f}) v=({:+.3f},{:+.3f}) hip=({:+.1f},{:+.1f}) tau=(T{:.0f},B{:.0f},A{:.0f}) spd={:.2f} tilt={:.1f} steps={}",
                                 comp._stateIndex, comp._swingIsLeft ? "L" : "R",
                                 comp._stateTime, comp._gaitWeight, comp._poseBlend,
                                 comp._dSag, comp._dLat, comp._vSag, comp._vLat,
                                 comp._swingHipCmdDeg, comp._swingHipLatCmdDeg,
                                 comp._torsoTorque, comp._swingTorque, comp._stanceTorque,
                                 glm::length(glm::vec2(v.x, v.z)), tiltDeg, comp._steps);
                    spdlog::info("[LocoFrame] tilt(mine={:.1f} engine={:.1f} delta={:+.1f}) right=({:+.2f},{:+.2f},{:+.2f}) fwd=({:+.2f},{:+.2f},{:+.2f}) femurCmd=({:+.2f},{:+.2f},{:+.2f}) femurActual=({:+.2f},{:+.2f},{:+.2f}) femurErr={:.1f}",
                                 comp._myTiltDeg, comp._engineTiltDeg,
                                 comp._myTiltDeg - comp._engineTiltDeg,
                                 comp._right.x, comp._right.y, comp._right.z,
                                 comp._fwd.x, comp._fwd.y, comp._fwd.z,
                                 comp._femurCmd.x, comp._femurCmd.y, comp._femurCmd.z,
                                 comp._femurActual.x, comp._femurActual.y, comp._femurActual.z,
                                 comp._femurErrDeg);
                    spdlog::info("[LocoPD] torso(P={:.0f} D={:.0f}) swing(P={:.0f} D={:.0f}) sat={}",
                                 comp._torsoP, comp._torsoD, comp._swingP, comp._swingD,
                                 comp._saturated ? "YES" : "no");
                    if (comp.assistedStepping)
                        spdlog::info("[LocoStep] phase={} contact={} airborne={} footForward={:+.3f} clearance={:+.3f} footY={:.3f}/{:.3f} footVy={:+.2f}",
                                     comp._stateIndex, comp._swingContact ? "yes" : "no",
                                     comp._swingWasAirborne ? "yes" : "no",
                                     comp._swingForward, comp._swingClearance, comp._swingFootY,
                                     comp._liftTargetY, comp._swingFootVy);
                }
            }
        }
    }

private:
    float _debugTimer = 0.0f;

    static float Approach(float value, float target, float amount)
    {
        return value < target ? glm::min(value + amount, target)
                              : glm::max(value - amount, target);
    }

    static void ResetGait(Comp& c)
    {
        c._gaitWeight = 0.0f;
        c._poseBlend = 0.0f;
        c._stateIndex = 0;
        c._stateTime = 0.0f;
        c._steps = 0;
        c._wasWalking = false;
        c._swingWasAirborne = false;
        c._airborneTime = 0.0f;
        c._swingContact = false;
        c._swingForward = c._swingFootY = c._swingFootVy = c._liftTargetY = 0.0f;
        c._swingClearance = 0.0f;
        c._torsoTorque = c._swingTorque = c._stanceTorque = 0.0f;
        c._torsoP = c._torsoD = c._swingP = c._swingD = 0.0f;
        c._saturated = false;
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

    static glm::quat OrientationOf(const glm::mat4& matrix)
    {
        const glm::vec3 x(matrix[0]), y(matrix[1]), z(matrix[2]);
        if (glm::dot(x, x) < 1e-12f || glm::dot(y, y) < 1e-12f || glm::dot(z, z) < 1e-12f)
            return glm::quat(1, 0, 0, 0);
        return glm::normalize(glm::quat_cast(
            glm::mat3(glm::normalize(x), glm::normalize(y), glm::normalize(z))));
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

    static glm::quat BindModelRot(const Diamond::Skeleton& skeleton, int bone)
    {
        return OrientationOf(glm::inverse(skeleton.bones[bone].inverseBind));
    }

    static glm::quat RotationBetween(const glm::vec3& from, const glm::vec3& to)
    {
        const glm::vec3 a = glm::normalize(from), b = glm::normalize(to);
        const float d = glm::clamp(glm::dot(a, b), -1.0f, 1.0f);
        if (d > 0.99999f) return glm::quat(1, 0, 0, 0);
        if (d < -0.99999f) {
            glm::vec3 axis = glm::cross(glm::vec3(1, 0, 0), a);
            if (glm::dot(axis, axis) < 1e-8f) axis = glm::cross(glm::vec3(0, 1, 0), a);
            return glm::angleAxis(glm::pi<float>(), glm::normalize(axis));
        }
        return glm::angleAxis(std::acos(d), glm::normalize(glm::cross(a, b)));
    }

    // Physical body rotation when the bone has one, else the animated pose. World-frame
    // targets must compose against the physical parent -- the two diverge exactly when
    // balance feedback matters.
    static glm::quat ParentWorldRot(const RagdollComponent& rag,
                                    const Diamond::Skeleton& skeleton,
                                    const AnimatorComponent& animator,
                                    const glm::mat4& entityWorld, int bone)
    {
        const int parent = skeleton.bones[bone].parent;
        if (parent < 0) return OrientationOf(entityWorld);
        bool ok = false;
        const glm::quat physical = Physics::GetRagdollBoneRotation(rag, parent, &ok);
        if (ok) return physical;
        return OrientationOf(BoneWorldMatrix(skeleton, animator, entityWorld, parent));
    }

    struct Envelope {
        glm::vec3 twistAxis { 1.0f, 0.0f, 0.0f };
        float swingNormalDeg = 60.0f, swingPlaneDeg = 60.0f;
        float twistMinDeg = -45.0f, twistMaxDeg = 45.0f;
    };

    // Salvaged from the old leg solver: express a local target in Jolt's authored
    // swing/twist basis and pull it back to the nearest legal orientation. Kept because it
    // is useful for ANY target source -- Table 1's angles plus a large cd*d term can leave
    // the authored cone on a rig the paper never saw.
    static glm::quat ClampToEnvelope(const Envelope& env, const glm::quat& restLocal,
                                     const glm::quat& target, float marginDeg)
    {
        const glm::vec3 axis = glm::normalize(env.twistAxis);
        const glm::vec3 reference = std::abs(axis.y) < 0.99f ? glm::vec3(0, 1, 0) : glm::vec3(1, 0, 0);
        const glm::vec3 planeAxis = glm::normalize(glm::cross(axis, reference));
        const glm::vec3 normalAxis = glm::normalize(glm::cross(axis, planeAxis));
        const glm::quat basis = glm::normalize(glm::quat_cast(glm::mat3(axis, planeAxis, normalAxis)));

        const float margin = glm::max(marginDeg, 0.0f);
        const float planeRadius = glm::max(std::sin(
            glm::radians(glm::clamp(env.swingPlaneDeg - margin, 1.0f, 179.0f)) * 0.5f), 1e-4f);
        const float normalRadius = glm::max(std::sin(
            glm::radians(glm::clamp(env.swingNormalDeg - margin, 1.0f, 179.0f)) * 0.5f), 1e-4f);
        const float middle = (env.twistMinDeg + env.twistMaxDeg) * 0.5f;
        const float twistMin = glm::radians(glm::min(env.twistMinDeg + margin, middle));
        const float twistMax = glm::radians(glm::max(env.twistMaxDeg - margin, middle));

        glm::quat delta = glm::normalize(glm::conjugate(restLocal) * target);
        glm::quat inBasis = glm::normalize(glm::conjugate(basis) * delta * basis);
        if (inBasis.w < 0.0f) inBasis = -inBasis;

        glm::quat twist(inBasis.w, inBasis.x, 0.0f, 0.0f);
        const float twistLength = glm::length(twist);
        twist = twistLength > 1e-6f ? twist / twistLength : glm::quat(1, 0, 0, 0);
        glm::quat swing = glm::normalize(inBasis * glm::conjugate(twist));
        if (swing.w < 0.0f) swing = -swing;

        const float ellipse = std::sqrt(
            (swing.y * swing.y) / (planeRadius * planeRadius) +
            (swing.z * swing.z) / (normalRadius * normalRadius));
        const float twistAngle = 2.0f * std::atan2(twist.x, twist.w);
        if (ellipse <= 1.0f && twistAngle >= twistMin && twistAngle <= twistMax) return target;

        const float scale = ellipse > 1.0f ? 1.0f / ellipse : 1.0f;
        const float sy = swing.y * scale, sz = swing.z * scale;
        const glm::quat clampedSwing(std::sqrt(glm::max(1.0f - sy * sy - sz * sz, 0.0f)), 0.0f, sy, sz);
        const glm::quat clampedTwist = glm::angleAxis(
            glm::clamp(twistAngle, twistMin, twistMax), glm::vec3(1, 0, 0));
        const glm::quat clamped = glm::normalize(
            basis * glm::normalize(clampedSwing * clampedTwist) * glm::conjugate(basis));
        return glm::normalize(restLocal * clamped);
    }

    static void ResolveLeg(Leg& leg, const std::string& footName,
                           const Diamond::Skeleton& skeleton, const RagdollComponent& rag)
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
            } else if (leg.ankleIdx >= 0 && body.boneName == skeleton.bones[leg.ankleIdx].name) {
                leg.ankleAxis = body.twistAxisLocal;
                leg.ankleSwingNormalDeg = body.swingNormalDeg;
                leg.ankleSwingPlaneDeg = body.swingPlaneDeg;
                leg.ankleTwistMinDeg = body.twistMinDeg;
                leg.ankleTwistMaxDeg = body.twistMaxDeg;
            } else if (body.boneName == skeleton.bones[leg.hipIdx].name) {
                leg.hipTwistAxis = body.twistAxisLocal;
                leg.hipSwingNormalDeg = body.swingNormalDeg;
                leg.hipSwingPlaneDeg = body.swingPlaneDeg;
                leg.hipTwistMinDeg = body.twistMinDeg;
                leg.hipTwistMaxDeg = body.twistMaxDeg;
            }
        }
    }

    static bool ValidLeg(const Leg& leg)
    {
        return leg.footIdx >= 0 && leg.ankleIdx >= 0 && leg.kneeIdx >= 0 && leg.hipIdx >= 0;
    }

    static bool FootGrounded(const RagdollComponent& rag, int footBone)
    {
        for (int i = 0; i < 2; ++i)
            if (rag._locomotionFootBones[i] == footBone) return rag._locomotionFootGrounded[i];
        return false;
    }

    static bool FootContact(const RagdollComponent& rag, int footBone, glm::vec3* normal = nullptr)
    {
        for (int i = 0; i < 2; ++i) {
            if (rag._locomotionFootBones[i] != footBone) continue;
            if (normal) *normal = rag._locomotionFootContactNormal[i];
            return rag._locomotionFootContact[i];
        }
        if (normal) *normal = glm::vec3(0.0f);
        return false;
    }

    static void BlendPose(AnimatorComponent& animator, int bone, const glm::quat& target, float weight)
    {
        animator.pose[bone].rotation = glm::normalize(glm::slerp(
            glm::normalize(animator.pose[bone].rotation), glm::normalize(target),
            glm::clamp(weight, 0.0f, 1.0f)));
    }

    // Terrain displacement (paper section 8): a difference of two implicit leg solves, so
    // it is identically zero on flat ground and cannot destabilise the base gait.
    static float TerrainKneeOffsetDeg(Scene& scene, entt::entity entity, const Comp& comp,
                                      const Leg& swing, const Leg& stance,
                                      const Diamond::Skeleton& skeleton,
                                      const AnimatorComponent& animator,
                                      const glm::mat4& entityWorld)
    {
        const glm::vec3 swingFoot = BoneWorldPos(skeleton, animator, entityWorld, swing.footIdx);
        const glm::vec3 stanceFoot = BoneWorldPos(skeleton, animator, entityWorld, stance.footIdx);
        const HitResult under = Physics::Raycast(swingFoot + glm::vec3(0, 0.1f, 0),
                                                 glm::vec3(0, -1, 0),
                                                 comp.ikGroundProbe + 0.1f, entity);
        const HitResult beneathStance = Physics::Raycast(stanceFoot + glm::vec3(0, 0.1f, 0),
                                                         glm::vec3(0, -1, 0),
                                                         comp.ikGroundProbe + 0.1f, entity);
        if (!under.hit || !beneathStance.hit) return 0.0f;

        const float step = under.point.y - beneathStance.point.y;
        if (std::abs(step) < 1e-3f) return 0.0f;

        const glm::vec3 hip = BoneWorldPos(skeleton, animator, entityWorld, swing.hipIdx);
        const float upper = glm::length(skeleton.bones[swing.kneeIdx].localT);
        const float lower = glm::length(skeleton.bones[swing.ankleIdx].localT);
        if (upper < 1e-4f || lower < 1e-4f) return 0.0f;

        auto kneeFor = [&](float reach) {
            const float d = glm::clamp(reach, std::abs(upper - lower) + 1e-4f,
                                       upper + lower - 1e-4f);
            return std::acos(glm::clamp(
                (upper * upper + lower * lower - d * d) / (2.0f * upper * lower), -1.0f, 1.0f));
        };
        const float nominal = glm::length(hip - swingFoot);
        const float adjusted = glm::max(nominal - step, 0.05f);
        const float deltaDeg = glm::degrees(kneeFor(adjusted) - kneeFor(nominal));
        return glm::clamp(deltaDeg, -comp.ikMaxOffsetDeg, comp.ikMaxOffsetDeg);
    }

    void UpdateAssistedGait(Scene& scene, entt::entity entity, Comp& comp,
                            RagdollComponent& rag, bool walking, float dt)
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
        ResolveLeg(comp._legL, comp.leftFootBone, skeleton, rag);
        ResolveLeg(comp._legR, comp.rightFootBone, skeleton, rag);
        if (!ValidLeg(comp._legL) || !ValidLeg(comp._legR)) return;

        const glm::mat4 entityWorld = scene.GetTransformSystem().GetWorldMatrix(entity);
        const glm::quat entityRot = OrientationOf(entityWorld);
        const glm::quat heading = glm::normalize(rag.locomotionTargetRot);
        const glm::vec3 leftHipBind = glm::vec3(
            (entityWorld * glm::inverse(skeleton.bones[comp._legL.hipIdx].inverseBind))[3]);
        const glm::vec3 rightHipBind = glm::vec3(
            (entityWorld * glm::inverse(skeleton.bones[comp._legR.hipIdx].inverseBind))[3]);
        glm::vec3 right = rightHipBind - leftHipBind;
        right.y = 0.0f;
        right = glm::length(right) > 1e-4f ? glm::normalize(right) : glm::vec3(1, 0, 0);
        right = heading * right;
        right.y = 0.0f;
        right = glm::length(right) > 1e-4f ? glm::normalize(right) : glm::vec3(1, 0, 0);
        const glm::vec3 fwd = glm::normalize(glm::cross(glm::vec3(0, 1, 0), right));

        auto physicalPosition = [&](int bone) {
            bool ok = false;
            glm::vec3 p = Physics::GetRagdollBonePosition(rag, bone, &ok);
            return ok ? p : BoneWorldPos(skeleton, animator, entityWorld, bone);
        };

        Leg* swing = comp._swingIsLeft ? &comp._legL : &comp._legR;
        Leg* stance = comp._swingIsLeft ? &comp._legR : &comp._legL;
        glm::vec3 swingFoot = physicalPosition(swing->footIdx);
        glm::vec3 stanceFoot = physicalPosition(stance->footIdx);
        glm::vec3 contactNormal(0.0f);
        bool contact = FootContact(rag, swing->footIdx, &contactNormal);
        bool velocityOk = false;
        glm::vec3 swingVelocity = Physics::GetRagdollBoneLinearVelocity(
            rag, swing->footIdx, &velocityOk);
        if (!velocityOk) swingVelocity = glm::vec3(0.0f);
        float swingClearance = swingFoot.y - stanceFoot.y;
        const float tiltDeg = Physics::GetRagdollTiltDeg(rag);
        const bool canStep = walking && tiltDeg < rag.locomotionFallenTilt;

        comp._swingContact = contact;
        comp._swingForward = glm::dot(swingFoot - stanceFoot, fwd);
        comp._swingFootY = swingFoot.y;
        comp._swingClearance = swingClearance;
        comp._liftTargetY = stanceFoot.y + glm::max(comp.assistedLiftHeight, 0.0f);
        comp._swingFootVy = swingVelocity.y;
        comp._right = right;
        comp._fwd = fwd;

        // This bring-up controller intentionally leaves every engine assist and both hip
        // position motors active. No virtual hip torque is allowed until a planted stance
        // leg and trustworthy touchdown transitions have been demonstrated.
        rag.locomotionSimbicon = false;
        rag.locomotionSimbiconBlend = 0.0f;
        rag.locomotionHipTorque[0] = rag.locomotionHipTorque[1] = glm::vec3(0.0f);
        rag.locomotionHipBones[0] = rag.locomotionHipBones[1] = -1;
        rag.locomotionSupportScale = 1.0f;
        comp._torsoTorque = comp._swingTorque = comp._stanceTorque = 0.0f;
        comp._torsoP = comp._torsoD = comp._swingP = comp._swingD = 0.0f;
        comp._saturated = false;

        if (!canStep) {
            comp._stateIndex = 0;
            comp._stateTime = 0.0f;
            comp._swingWasAirborne = false;
            comp._airborneTime = 0.0f;
            rag.locomotionSupportTargetWeight = 0.0f;
        } else {
            comp._stateTime += dt;
            switch (comp._stateIndex) {
                case 0: // double support: shift COM over the future stance foot
                    if (comp._stateTime >= glm::max(comp.weightShiftTime, 0.05f)) {
                        comp._stateIndex = 1;
                        comp._stateTime = 0.0f;
                        comp._swingWasAirborne = false;
                        comp._airborneTime = 0.0f;
                    }
                    break;
                case 1: { // lift + reach
                    // Contact manifolds can persist while a rotating toe peels away from
                    // the floor. Accept either a clean solver separation or enough physical
                    // foot-center clearance as takeoff evidence; require it to persist so a
                    // single noisy frame cannot advance the gait.
                    const bool takeoffEvidence = !contact ||
                        swingClearance >= glm::max(comp.assistedTakeoffClearance, 0.0f);
                    if (takeoffEvidence) comp._airborneTime += dt;
                    else if (!comp._swingWasAirborne) comp._airborneTime = 0.0f;
                    if (comp._airborneTime >= glm::max(comp.airborneConfirmTime, 0.0f))
                        comp._swingWasAirborne = true;
                    if (comp._swingWasAirborne &&
                        comp._stateTime >= glm::max(comp.assistedLiftTime, 0.1f)) {
                        comp._stateIndex = 2;
                        comp._stateTime = 0.0f;
                    }
                    break;
                }
                case 2: { // extend down; accept only a NEW, forward, low-speed support contact
                    const bool planted = comp._swingWasAirborne && contact &&
                        contactNormal.y >= 0.5f &&
                        std::abs(swingVelocity.y) <= glm::max(comp.maxPlantVerticalSpeed, 0.1f) &&
                        comp._swingForward >= glm::max(comp.minPlantForward, 0.0f);
                    if (planted) {
                        comp._stateIndex = 3;
                        comp._stateTime = 0.0f;
                        ++comp._steps;
                    } else if (comp._stateTime >= glm::max(comp.assistedPlantTimeout, 0.1f)) {
                        // The foot did not establish a valid support. Re-lift the same leg;
                        // never report a step or hand support to the other leg on a timeout.
                        comp._stateIndex = 1;
                        comp._stateTime = 0.0f;
                        comp._swingWasAirborne = false;
                        comp._airborneTime = 0.0f;
                    }
                    break;
                }
                case 3: // confirmed double support: move COM to the new foot, then alternate
                    if (comp._stateTime >= glm::max(comp.assistedTransferTime, 0.05f)) {
                        comp._swingIsLeft = !comp._swingIsLeft;
                        comp._stateIndex = 0;
                        comp._stateTime = 0.0f;
                        comp._swingWasAirborne = false;
                        comp._airborneTime = 0.0f;
                    }
                    break;
                default:
                    comp._stateIndex = 0;
                    comp._stateTime = 0.0f;
                    break;
            }
        }

        // A transition may have swapped the leg roles; refresh all physical measurements.
        swing = comp._swingIsLeft ? &comp._legL : &comp._legR;
        stance = comp._swingIsLeft ? &comp._legR : &comp._legL;
        swingFoot = physicalPosition(swing->footIdx);
        stanceFoot = physicalPosition(stance->footIdx);
        contact = FootContact(rag, swing->footIdx, &contactNormal);
        velocityOk = false;
        swingVelocity = Physics::GetRagdollBoneLinearVelocity(
            rag, swing->footIdx, &velocityOk);
        if (!velocityOk) swingVelocity = glm::vec3(0.0f);
        swingClearance = swingFoot.y - stanceFoot.y;
        const glm::vec3 rootPos = physicalPosition(rootBone);
        const glm::vec3 com = rag._locomotionCOMValid ? rag._locomotionCOM : rootPos;
        const glm::vec3 comVel = rag._locomotionCOMValid ? rag._locomotionCOMVel
                                                         : rag._locomotionRootVel;
        const glm::vec3 d = com - stanceFoot;
        comp._dSag = glm::dot(d, fwd);
        comp._dLat = glm::dot(d, right);
        comp._vSag = glm::dot(comVel, fwd);
        comp._vLat = glm::dot(comVel, right);

        comp._swingFootY = swingFoot.y;
        comp._swingContact = contact;
        comp._swingForward = glm::dot(swingFoot - stanceFoot, fwd);
        comp._swingFootVy = swingVelocity.y;
        comp._swingClearance = swingClearance;
        comp._liftTargetY = stanceFoot.y + glm::max(comp.assistedLiftHeight, 0.0f);
        if (canStep && comp._stateIndex == 1) {
            rag.locomotionLiftBone = swing->footIdx;
            rag.locomotionLiftTargetY = comp._liftTargetY;
            rag.locomotionLiftFrequency = glm::max(comp.assistedLiftFrequency, 0.0f);
            rag.locomotionLiftMaxForce = glm::max(comp.assistedLiftMaxForce, 0.0f);
        } else {
            rag.locomotionLiftBone = -1;
        }

        if (canStep) {
            // During transfer, the just-landed swing foot becomes the support target. In all
            // earlier phases the original stance foot stays authoritative.
            // Weight shift should be lateral only. Pulling COM to the stance foot in both
            // horizontal axes pulled the body backward while the swing sole was reaching
            // forward, which looked exactly like reversed input. Only the confirmed
            // transfer phase is allowed to advance COM to the newly planted foot.
            const glm::vec3 support = comp._stateIndex == 3
                ? swingFoot
                : com + right * glm::dot(stanceFoot - com, right);
            rag.locomotionSupportTarget = glm::vec3(support.x, com.y, support.z);
            rag.locomotionSupportTargetVel = glm::vec3(0.0f);
            rag.locomotionSupportTargetWeight = 1.0f;
        }

        if (!comp.ikWriteEnabled) return;
        const float weight = glm::clamp(comp._poseBlend * comp.poseWeight, 0.0f, 1.0f);
        auto writeBind = [&](int bone) {
            BlendPose(animator, bone, skeleton.bones[bone].localR, weight);
        };
        auto writeAxis = [&](int bone, const glm::vec3& axis, float degrees) {
            const glm::quat target = glm::normalize(
                skeleton.bones[bone].localR *
                glm::angleAxis(glm::radians(degrees), glm::normalize(axis)));
            BlendPose(animator, bone, target, weight);
        };
        auto worldHipTarget = [&](const Leg& leg, float sagittalDeg) {
            const glm::quat bindWorld = glm::normalize(
                heading * entityRot * BindModelRot(skeleton, leg.hipIdx));
            const glm::vec3 localDirection = glm::normalize(skeleton.bones[leg.kneeIdx].localT);
            const glm::vec3 bindDirection = bindWorld * localDirection;
            const glm::vec3 desiredDirection =
                glm::angleAxis(glm::radians(sagittalDeg), right) * glm::vec3(0, -1, 0);
            return glm::normalize(RotationBetween(bindDirection, desiredDirection) * bindWorld);
        };
        auto writeWorldMotor = [&](int bone, const glm::quat& worldTarget) {
            const int parent = skeleton.bones[bone].parent;
            const glm::quat parentWorld = parent >= 0
                ? glm::normalize(heading * entityRot * BindModelRot(skeleton, parent))
                : glm::normalize(heading * entityRot);
            BlendPose(animator, bone, glm::conjugate(parentWorld) * worldTarget, weight);
        };

        // Always motor-lock the stance chain. Shift holds both legs at bind; Lift flexes and
        // reaches; Plant/Transfer keep the new foot ahead while the knee extends.
        writeBind(stance->hipIdx);
        writeBind(stance->kneeIdx);
        writeBind(stance->ankleIdx);
        float swingHipDeg = 0.0f, swingKneeDeg = 0.0f, swingAnkleDeg = 0.0f;
        if (canStep && comp._stateIndex == 1) {
            swingHipDeg = comp.s0SwingHipDeg;
            swingKneeDeg = comp.s0SwingKneeDeg;
            swingAnkleDeg = comp.s0SwingAnkleDeg;
        } else if (canStep && (comp._stateIndex == 2 || comp._stateIndex == 3)) {
            swingHipDeg = comp.assistedPlantHipDeg;
            swingKneeDeg = comp.s1SwingKneeDeg;
            swingAnkleDeg = comp.s1SwingAnkleDeg;
        }
        if (std::abs(swingHipDeg) < 1e-4f) writeBind(swing->hipIdx);
        else writeWorldMotor(swing->hipIdx, worldHipTarget(*swing, swingHipDeg));
        if (std::abs(swingKneeDeg) < 1e-4f) writeBind(swing->kneeIdx);
        else writeAxis(swing->kneeIdx, swing->kneeHingeAxis, swingKneeDeg);
        if (std::abs(swingAnkleDeg) < 1e-4f) writeBind(swing->ankleIdx);
        else writeAxis(swing->ankleIdx, swing->ankleAxis, swingAnkleDeg);

        comp._swingHipCmdDeg = swingHipDeg;
        comp._swingHipLatCmdDeg = 0.0f;
        comp._femurCmd = glm::angleAxis(glm::radians(swingHipDeg), right) * glm::vec3(0, -1, 0);
        bool hipOk = false;
        const glm::quat hipRotation = Physics::GetRagdollBoneRotation(rag, swing->hipIdx, &hipOk);
        comp._femurActual = hipOk
            ? glm::normalize(hipRotation * glm::normalize(skeleton.bones[swing->kneeIdx].localT))
            : glm::vec3(0, -1, 0);
        comp._femurErrDeg = glm::degrees(std::acos(glm::clamp(
            glm::dot(comp._femurActual, glm::normalize(comp._femurCmd)), -1.0f, 1.0f)));
        comp._engineTiltDeg = Physics::GetRagdollTiltDeg(rag);
        comp._myTiltDeg = comp._engineTiltDeg;

        if (comp.debug) {
            DebugDraw::Sphere(stanceFoot, 0.045f, {0.2f, 1.0f, 0.2f});
            DebugDraw::Sphere(swingFoot, 0.04f, contact ? glm::vec3(0.2f, 0.8f, 1.0f)
                                                        : glm::vec3(1.0f, 0.5f, 0.1f));
            DebugDraw::Line(stanceFoot, stanceFoot + fwd * 0.25f, {0.3f, 1.0f, 0.3f});
        }
    }

    void UpdateGait(Scene& scene, entt::entity entity, Comp& comp, RagdollComponent& rag,
                    bool walking, float dt)
    {
        if (comp.assistedStepping) {
            UpdateAssistedGait(scene, entity, comp, rag, walking, dt);
            return;
        }
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

        Leg& swing = comp._swingIsLeft ? comp._legL : comp._legR;

        // --- FSM (paper 3.1): fixed time, then contact. ---
        if (walking) {
            comp._stateTime += dt;
            if (comp._stateIndex == 0) {
                if (comp._stateTime >= glm::max(comp.state0Time, 0.01f)) {
                    comp._stateIndex = 1;
                    comp._stateTime = 0.0f;
                }
            } else {
                const bool contact = comp._stateTime >= comp.minSwingTime &&
                                     FootGrounded(rag, swing.footIdx);
                if (contact || comp._stateTime >= comp.maxSwingTime) {
                    comp._swingIsLeft = !comp._swingIsLeft;
                    comp._stateIndex = 0;
                    comp._stateTime = 0.0f;
                    if (contact) ++comp._steps;
                }
            }
        } else {
            comp._stateIndex = 0;
            comp._stateTime = 0.0f;
        }
        Leg& swingLeg  = comp._swingIsLeft ? comp._legL : comp._legR;
        Leg& stanceLeg = comp._swingIsLeft ? comp._legR : comp._legL;

        const bool s0 = comp._stateIndex == 0;
        const float torsoDeg       = s0 ? comp.s0TorsoDeg : comp.s1TorsoDeg;
        const float swingHipDeg    = s0 ? comp.s0SwingHipDeg : comp.s1SwingHipDeg;
        const float swingKneeDeg   = s0 ? comp.s0SwingKneeDeg : comp.s1SwingKneeDeg;
        const float swingAnkleDeg  = s0 ? comp.s0SwingAnkleDeg : comp.s1SwingAnkleDeg;
        const float stanceKneeDeg  = s0 ? comp.s0StanceKneeDeg : comp.s1StanceKneeDeg;
        const float stanceAnkleDeg = s0 ? comp.s0StanceAnkleDeg : comp.s1StanceAnkleDeg;

        // Build the control frame from the authored bind hips, then rotate it by the desired
        // heading. Measuring it from the live hips made the target frame follow pelvis yaw:
        // once a stumble began, "forward" rotated with the fall and later steps went sideways
        // or backwards. SIMBICON's torso and swing-hip targets are WORLD-frame targets, so
        // their sagittal/coronal axes must remain tied to the requested heading.
        const glm::quat entityRot = OrientationOf(entityWorld);
        const glm::vec3 leftHipBind = glm::vec3(
            (entityWorld * glm::inverse(skeleton.bones[comp._legL.hipIdx].inverseBind))[3]);
        const glm::vec3 rightHipBind = glm::vec3(
            (entityWorld * glm::inverse(skeleton.bones[comp._legR.hipIdx].inverseBind))[3]);
        glm::vec3 right = rightHipBind - leftHipBind;
        right.y = 0.0f;
        right = glm::length(right) > 1e-4f ? glm::normalize(right) : glm::vec3(1, 0, 0);
        right = glm::normalize(glm::normalize(rag.locomotionTargetRot) * right);
        right.y = 0.0f;
        right = glm::length(right) > 1e-4f ? glm::normalize(right) : glm::vec3(1, 0, 0);
        const glm::vec3 fwd = glm::normalize(glm::cross(glm::vec3(0, 1, 0), right));

        // --- Balance feedback (paper 3.3): d, v measured from the stance ankle. ---

        const glm::vec3 stanceAnkle = BoneWorldPos(skeleton, animator, entityWorld, stanceLeg.ankleIdx);
        const bool useHipProxy = comp.useHipCOMProxy || !rag._locomotionCOMValid;
        const glm::vec3 com = useHipProxy
            ? BoneWorldPos(skeleton, animator, entityWorld, rootBone)
            : rag._locomotionCOM;
        const glm::vec3 comVel = useHipProxy ? rag._locomotionRootVel : rag._locomotionCOMVel;
        const glm::vec3 d = com - stanceAnkle;
        comp._dSag = glm::dot(d, fwd);
        comp._dLat = glm::dot(d, right);
        comp._vSag = glm::dot(comVel, fwd);
        comp._vLat = glm::dot(comVel, right);

        const float limit = glm::max(comp.swingHipLimitDeg, 1.0f);
        const float sagDeg = glm::clamp(
            swingHipDeg + glm::degrees(comp.cd * comp._dSag + comp.cv * comp._vSag), -limit, limit);
        const float widthBias = comp._swingIsLeft ? -comp.stanceWidthDeg : comp.stanceWidthDeg;
        const float latDeg = glm::clamp(
            widthBias + glm::degrees(comp.cdLat * comp._dLat + comp.cvLat * comp._vLat), -limit, limit);
        comp._swingHipCmdDeg = sagDeg;
        comp._swingHipLatCmdDeg = latDeg;

        // --- Frame diagnostics (no behaviour, measurement only) ---
        {
            bool ok = false;
            const glm::quat pelvisNow = Physics::GetRagdollBoneRotation(rag, rootBone, &ok);
            const glm::quat pelvisBind = entityRot * BindModelRot(skeleton, rootBone);
            const glm::vec3 myUp = ok
                ? pelvisNow * (glm::conjugate(pelvisBind) * glm::vec3(0, 1, 0))
                : glm::vec3(0, 1, 0);
            comp._myTiltDeg = glm::degrees(std::acos(glm::clamp(myUp.y, -1.0f, 1.0f)));
            comp._engineTiltDeg = Physics::GetRagdollTiltDeg(rag);
            comp._torsoErrDeg = glm::degrees(std::acos(glm::clamp(
                glm::dot(glm::normalize(myUp), glm::vec3(0, 1, 0)), -1.0f, 1.0f)));

            comp._right = right;
            comp._fwd = fwd;
            comp._femurCmd = glm::angleAxis(glm::radians(-latDeg), fwd) *
                             glm::angleAxis(glm::radians(sagDeg), right) * glm::vec3(0, -1, 0);
            const glm::vec3 hipPos = BoneWorldPos(skeleton, animator, entityWorld, swingLeg.hipIdx);
            const glm::vec3 kneePos = BoneWorldPos(skeleton, animator, entityWorld, swingLeg.kneeIdx);
            const glm::vec3 limb = kneePos - hipPos;
            comp._femurActual = glm::length(limb) > 1e-5f ? glm::normalize(limb) : glm::vec3(0, -1, 0);
            comp._femurErrDeg = glm::degrees(std::acos(glm::clamp(
                glm::dot(comp._femurActual, glm::normalize(comp._femurCmd)), -1.0f, 1.0f)));
        }

        comp._ikOffsetDeg = comp.ikTerrainEnabled && walking && !s0
            ? TerrainKneeOffsetDeg(scene, entity, comp, swingLeg, stanceLeg, skeleton,
                                   animator, entityWorld)
            : 0.0f;

        // _poseBlend only fades the pose motors. The virtual hip torques must keep ownership
        // while the FSM attempts a catch step; returning here used to re-enable both 250 N-m
        // bind motors as soon as tilt crossed locomotionFallenTilt.
        if (!comp.ikWriteEnabled) return;

        // Standing is the BIND pose, which is what held stable for 21 s. Folding the gait
        // blend into the write weight means g=0 writes nothing and the motors hold bind --
        // an explicit standing pose here regressed standing into a topple.
        const float g = glm::clamp(comp._gaitWeight, 0.0f, 1.0f);
        const float weight = glm::clamp(comp._poseBlend * comp.poseWeight * g, 0.0f, 1.0f);
        const float stanceSideWidth = comp._swingIsLeft ? comp.stanceWidthDeg : -comp.stanceWidthDeg;
        const float swingKneeCmd  = swingKneeDeg + comp._ikOffsetDeg;
        const float swingAnkleCmd = swingAnkleDeg - comp._ikOffsetDeg * 0.5f;
        const float stanceKneeCmd = stanceKneeDeg;
        const float stanceAnkleCmd = stanceAnkleDeg;

        // --- Write targets (paper 3.2): swing hip and torso in the world frame, the rest
        // parent-relative. A world target becomes a joint target by composing against the
        // PHYSICAL parent, which is what decouples the swing leg from torso pitch. ---
        auto writeWorld = [&](int bone, const glm::quat& rotation) {
            const glm::quat parent = ParentWorldRot(rag, skeleton, animator, entityWorld, bone);
            BlendPose(animator, bone, glm::conjugate(parent) * rotation, weight);
        };
        auto writeAxis = [&](int bone, const glm::vec3& axis, float degrees, const Envelope* env) {
            glm::quat target = glm::normalize(
                skeleton.bones[bone].localR *
                glm::angleAxis(glm::radians(degrees), glm::normalize(axis)));
            if (env)
                target = ClampToEnvelope(*env, skeleton.bones[bone].localR, target,
                                         comp.hipLimitMarginDeg);
            BlendPose(animator, bone, target, weight);
        };

        // The gait target is a thigh direction, not a full femur orientation. Keeping that
        // distinction here is important: a ball-joint femur can twist around its own length,
        // and spending swing-hip torque correcting that invisible twist starves the forward
        // step of torque.
        const glm::vec3 desiredSwingDir = glm::normalize(
            glm::angleAxis(glm::radians(-latDeg), fwd) *
            glm::angleAxis(glm::radians(sagDeg), right) * glm::vec3(0, -1, 0));

        // Hips are NOT written as motor targets -- they are driven by the virtual torques
        // below, and a live motor on the same joint would fight them.
        writeAxis(swingLeg.kneeIdx, swingLeg.kneeHingeAxis, swingKneeCmd, nullptr);
        writeAxis(stanceLeg.kneeIdx, stanceLeg.kneeHingeAxis, stanceKneeCmd, nullptr);

        auto ankleEnvelope = [](const Leg& leg) {
            Envelope env;
            env.twistAxis = leg.ankleAxis;
            env.swingNormalDeg = leg.ankleSwingNormalDeg;
            env.swingPlaneDeg = leg.ankleSwingPlaneDeg;
            env.twistMinDeg = leg.ankleTwistMinDeg;
            env.twistMaxDeg = leg.ankleTwistMaxDeg;
            return env;
        };
        const Envelope swingAnkleEnv = ankleEnvelope(swingLeg);
        const Envelope stanceAnkleEnv = ankleEnvelope(stanceLeg);
        writeAxis(swingLeg.ankleIdx, swingLeg.ankleAxis, swingAnkleCmd, &swingAnkleEnv);
        writeAxis(stanceLeg.ankleIdx, stanceLeg.ankleAxis, stanceAnkleCmd, &stanceAnkleEnv);

        // Full world-frame body target: authored bind orientation, desired heading, then the
        // state's pitch about the desired right axis. Omitting heading left yaw uncontrolled;
        // the stance-hip residual then spun the pelvis and carried the step frame with it.
        auto worldBodyTarget = [&](int bone, float pitchDeg) {
            const glm::quat bind = entityRot * BindModelRot(skeleton, bone);
            return glm::normalize(glm::angleAxis(glm::radians(pitchDeg), right) *
                                  glm::normalize(rag.locomotionTargetRot) * bind);
        };

        const int torsoIdx = skeleton.Find(comp.torsoBone);
        if (torsoIdx >= 0) writeWorld(torsoIdx, worldBodyTarget(torsoIdx, torsoDeg));

        // --- Virtual PD torques (paper 3.2). tau_A = -tau_torso - tau_B is the whole point:
        // the stance hip is a free variable, so commanding torso attitude and swing-leg
        // placement leaves a residual that pushes the body forward using internal torque. ---
        auto orientationPD = [&](int bone, const glm::quat& target, float kp, float kd,
                                 float* proportionalOut, float* dampingOut) {
            bool ok = false;
            const glm::quat current = Physics::GetRagdollBoneRotation(rag, bone, &ok);
            if (!ok) return glm::vec3(0.0f);
            const glm::vec3 omega = Physics::GetRagdollBoneAngularVelocity(rag, bone);
            glm::quat error = glm::normalize(glm::normalize(target) * glm::conjugate(current));
            if (error.w < 0.0f) error = -error;
            const glm::vec3 axisRaw(error.x, error.y, error.z);
            const float length = glm::length(axisRaw);
            const glm::vec3 axis = length > 1e-6f ? axisRaw / length : glm::vec3(0.0f);
            const float angle = 2.0f * std::atan2(length, glm::clamp(error.w, -1.0f, 1.0f));
            const glm::vec3 proportional = axis * (angle * kp);
            const glm::vec3 damping = -omega * kd;
            if (proportionalOut) *proportionalOut = glm::length(proportional);
            if (dampingOut) *dampingOut = glm::length(damping);
            return proportional + damping;
        };

        // Swing-hip PD in direction space. The old full-orientation controller damped the
        // femur's twist and every other component of its angular velocity. In the failing
        // run that damping reached 257 Nm while the useful proportional term was only 53 Nm,
        // so the knee folded correctly but the thigh never came forward. This controller
        // removes spin around the thigh and applies the shortest rotation that actually moves
        // the knee toward the commanded direction.
        auto directionPD = [&](int bone, const glm::vec3& localDirection,
                               const glm::vec3& targetDirection, float kp, float kd,
                               float* proportionalOut, float* dampingOut) {
            bool ok = false;
            const glm::quat current = Physics::GetRagdollBoneRotation(rag, bone, &ok);
            if (!ok) return glm::vec3(0.0f);

            const glm::vec3 actual = glm::normalize(current * glm::normalize(localDirection));
            const glm::vec3 target = glm::normalize(targetDirection);
            const glm::vec3 cross = glm::cross(actual, target);
            const float sinAngle = glm::length(cross);
            const float cosAngle = glm::clamp(glm::dot(actual, target), -1.0f, 1.0f);
            const glm::vec3 axis = sinAngle > 1e-6f ? cross / sinAngle : glm::vec3(0.0f);
            const float angle = std::atan2(sinAngle, cosAngle);
            const glm::vec3 proportional = axis * (angle * kp);

            const glm::vec3 omega = Physics::GetRagdollBoneAngularVelocity(rag, bone);
            const glm::vec3 directionOmega = omega - actual * glm::dot(omega, actual);
            const glm::vec3 damping = -directionOmega * kd;
            if (proportionalOut) *proportionalOut = glm::length(proportional);
            if (dampingOut) *dampingOut = glm::length(damping);
            return proportional + damping;
        };

        const glm::vec3 torsoTorque =
            orientationPD(rootBone, worldBodyTarget(rootBone, torsoDeg),
                          comp.torsoKp, comp.torsoKd, &comp._torsoP, &comp._torsoD) * g;
        const glm::vec3 swingLocalDirection =
            glm::normalize(skeleton.bones[swingLeg.kneeIdx].localT);
        const glm::vec3 swingTorque =
            directionPD(swingLeg.hipIdx, swingLocalDirection, desiredSwingDir,
                        comp.hipKp, comp.hipKd, &comp._swingP, &comp._swingD) * g;
        glm::vec3 stanceTorque = -torsoTorque - swingTorque;

        // Scale all three TOGETHER. Clamping them independently breaks
        // tau_A = -tau_torso - tau_B, so the pelvis stops netting tau_torso and the paper's
        // central identity silently stops holding -- which it did, ~100% of the time.
        {
            const float limit = glm::max(comp.maxVirtualTorque, 0.0f);
            const float worst = glm::max(glm::length(torsoTorque),
                                         glm::max(glm::length(swingTorque),
                                                  glm::length(stanceTorque)));
            if (worst > limit && worst > 1e-6f) {
                const float scale = limit / worst;
                stanceTorque *= scale;
                comp._torsoTorque = glm::length(torsoTorque) * scale;
                comp._swingTorque = glm::length(swingTorque) * scale;
                rag.locomotionHipTorque[0] = swingTorque * scale;
                comp._saturated = true;
            } else {
                comp._torsoTorque = glm::length(torsoTorque);
                comp._swingTorque = glm::length(swingTorque);
                rag.locomotionHipTorque[0] = swingTorque;
                comp._saturated = false;
            }
        }
        rag.locomotionHipBones[0] = swingLeg.hipIdx;
        rag.locomotionHipBones[1] = stanceLeg.hipIdx;
        rag.locomotionHipTorque[1] = stanceTorque;
        rag.locomotionSupportScale = comp.supportScale;
        comp._stanceTorque = glm::length(stanceTorque);

        if (comp.debug) {
            DebugDraw::Sphere(stanceAnkle, 0.04f, {0.2f, 1.0f, 0.2f});
            DebugDraw::Sphere(glm::vec3(com.x, stanceAnkle.y + 0.03f, com.z), 0.035f,
                              {1.0f, 0.85f, 0.1f});
            DebugDraw::Line(stanceAnkle, stanceAnkle + fwd * comp._dSag, {1.0f, 0.4f, 0.1f});
            DebugDraw::Line(stanceAnkle, stanceAnkle + right * comp._dLat, {0.1f, 0.6f, 1.0f});
            const glm::vec3 hip = BoneWorldPos(skeleton, animator, entityWorld, swingLeg.hipIdx);
            DebugDraw::Line(hip, hip + desiredSwingDir * 0.35f, {1.0f, 0.2f, 0.8f});
        }
    }
};
