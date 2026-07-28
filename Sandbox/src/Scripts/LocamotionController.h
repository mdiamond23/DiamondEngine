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

    // Bring-up validation: 0 = normal locomotion, 1 = standing/shove, 2 = weight shift,
    // 3 = single-leg lift, 4 = grounded motor isolation. Validation modes retain physical
    // standing but categorically bypass the gait path.
    int   validationTest = 3;
    float test1ShoveImpulse = 25.0f;
    float test1ShoveCooldown = 3.0f;
    float test2ShiftFraction = 0.65f;
    float test2ShiftTime = 1.50f;
    float test2SupportFrequency = 1.50f;
    float test2SupportMaxAccel = 3.0f;
    float test3SupportFraction = 0.92f;
    float test3TakeoffHeight = 0.060f;
    float test3TakeoffTime = 0.50f;
    float test3TakeoffFrequency = 3.5f;
    float test3TakeoffMaxForce = 180.0f;
    float test3LiftHeight = 0.10f;
    float test3LiftTime = 0.65f;
    float test3HoldTime = 0.50f;
    float test3LowerTime = 0.65f;
    float test3ContactSettleTime = 0.30f;

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
    float proceduralStepLength = 0.30f;
    float proceduralStepWidth = 0.18f;
    float proceduralPlantTime = 0.20f;
    float proceduralMaxReach = 0.94f;
    float proceduralPoseResponse = 0.10f;
    float proceduralSupportShift = 0.70f;
    float proceduralSupportFrequency = 2.0f;
    float proceduralSupportMaxAccel = 4.0f;
    float proceduralPelvisBob = 0.018f;
    float proceduralTorsoRollDeg = 3.0f;
    float proceduralTorsoPitchDeg = 2.0f;
    float proceduralFootLockFrequency = 2.5f;
    float proceduralFootLockEffectiveMass = 12.0f;
    float proceduralFootLockMaxForce = 600.0f;
    float proceduralFootLockTolerance = 0.04f;
    float proceduralWeightShiftTolerance = 0.07f;
    float proceduralStanceSupportScale = 0.55f;
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
        bool planted = false;
        glm::vec3 plantFoot { 0.0f };
        glm::vec3 swingStartFoot { 0.0f };
        glm::vec3 swingTargetFoot { 0.0f };
        glm::vec3 desiredFoot { 0.0f };
        glm::vec3 ankleFromFootWorld { 0.0f };
        glm::vec3 kneePoleWorld { 0.0f, 0.0f, -1.0f };
        glm::quat plantedFootWorldRotation { 1, 0, 0, 0 };
        glm::vec3 referenceUpperWorld { 0.0f, -1.0f, 0.0f };
        glm::quat referenceHipWorld { 1, 0, 0, 0 };
        glm::quat referenceKneeLocal { 1, 0, 0, 0 };
        glm::quat referenceAnkleLocal { 1, 0, 0, 0 };
        glm::quat referenceFootLocal { 1, 0, 0, 0 };
        float referenceKneeBend = 0.0f;
        float groundOffset = 0.06f;
        float lockWeight = 0.0f;
        bool commandValid = false;
        glm::quat hipCommand { 1, 0, 0, 0 };
        glm::quat kneeCommand { 1, 0, 0, 0 };
        glm::quat ankleCommand { 1, 0, 0, 0 };
        glm::quat footCommand { 1, 0, 0, 0 };
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
    float _stepPhase = 0.0f;
    float _stanceLockError = 0.0f, _swingTargetError = 0.0f;
    float _maxStanceLockError = 0.0f;
    float _plantAcquireTime = 0.0f;
    float _touchdownHeightError = 0.0f;
    glm::vec3 _desiredVelocity { 0.0f };
    float _torsoP = 0.0f, _torsoD = 0.0f, _swingP = 0.0f, _swingD = 0.0f;
    bool  _saturated = false;
    int   _validationActiveTest = -1;
    bool  _test1BaselineValid = false;
    bool  _test1ContactL = false, _test1ContactR = false;
    float _test1Time = 0.0f;
    float _test1SettleTime = 0.0f;
    float _test1SinceShove = -1.0f;
    float _test1RecoveryTime = -1.0f;
    float _test1PeakTilt = 0.0f;
    float _test1HorizontalSpeed = 0.0f;
    float _test1FootDriftL = 0.0f, _test1FootDriftR = 0.0f;
    int   _test1Shoves = 0;
    glm::vec3 _test1FootBaselineL { 0.0f };
    glm::vec3 _test1FootBaselineR { 0.0f };
    bool  _test2BaselineValid = false;
    bool  _test2ContactL = false, _test2ContactR = false;
    float _test2Time = 0.0f;
    float _test2SettleTime = 0.0f;
    float _test2Command = 0.0f;
    int   _test2TargetSide = 0;
    float _test2ComLateral = 0.0f;
    float _test2TargetLateral = 0.0f;
    float _test2FootDriftL = 0.0f, _test2FootDriftR = 0.0f;
    float _test2MaxFootDrift = 0.0f;
    float _test2PeakTilt = 0.0f;
    glm::vec3 _test2FootBaselineL { 0.0f };
    glm::vec3 _test2FootBaselineR { 0.0f };
    glm::vec3 _test2ComBaseline { 0.0f };
    glm::vec3 _test2Right { 1.0f, 0.0f, 0.0f };
    glm::vec3 _test2SupportTarget { 0.0f };
    bool  _test3BaselineValid = false;
    bool  _test3ContactL = false, _test3ContactR = false;
    float _test3Time = 0.0f;
    float _test3SettleTime = 0.0f;
    int   _test3Phase = 0;
    float _test3PhaseTime = 0.0f;
    int   _test3SupportSide = 0;
    float _test3ComCommand = 0.0f;
    float _test3ComLateral = 0.0f, _test3TargetLateral = 0.0f;
    float _test3LiftBlend = 0.0f;
    float _test3LiftStartBlend = 0.0f;
    float _test3AirborneTime = 0.0f;
    float _test3Clearance = 0.0f;
    float _test3TargetError = 0.0f;
    float _test3KneePoleDot = 1.0f;
    float _test3MaxStanceDrift = 0.0f;
    float _test3PeakTilt = 0.0f;
    bool  _test3Aborted = false;
    glm::vec3 _test3FootBaselineL { 0.0f };
    glm::vec3 _test3FootBaselineR { 0.0f };
    glm::vec3 _test3ComBaseline { 0.0f };
    glm::vec3 _test3Right { 1.0f, 0.0f, 0.0f };
    glm::vec3 _test3SupportTarget { 0.0f };
    glm::vec3 _test3SwingStart { 0.0f };
    bool _groundTestBaselineValid = false;
    float _groundTestTime = 0.0f;
    float _groundTestSettleTime = 0.0f;
    int _groundTestStage = 0;
    float _groundTestStageTime = 0.0f;
    glm::quat _groundTestPose[8] {
        glm::quat(1, 0, 0, 0), glm::quat(1, 0, 0, 0),
        glm::quat(1, 0, 0, 0), glm::quat(1, 0, 0, 0),
        glm::quat(1, 0, 0, 0), glm::quat(1, 0, 0, 0),
        glm::quat(1, 0, 0, 0), glm::quat(1, 0, 0, 0)
    };
    glm::vec3 _groundTestComTarget { 0.0f };
    glm::vec3 _groundTestFootStart[2] { glm::vec3(0.0f), glm::vec3(0.0f) };
    glm::quat _groundTestFootRotStart[2] { glm::quat(1, 0, 0, 0), glm::quat(1, 0, 0, 0) };
    bool _groundTestContact[2] { false, false };
    bool _groundTestPrevContact[2] { false, false };
    int _groundTestContactTransitions[2] { 0, 0 };
    float _groundTestFootSpeed[2] { 0.0f, 0.0f };
    float _groundTestFootAngularSpeed[2] { 0.0f, 0.0f };
    float _groundTestFootDisplacement[2] { 0.0f, 0.0f };
    float _groundTestFootRotation[2] { 0.0f, 0.0f };
    float _groundTestMaxSpeed[2] { 0.0f, 0.0f };
    float _groundTestMaxAngularSpeed[2] { 0.0f, 0.0f };
    float _groundTestMaxDisplacement[2] { 0.0f, 0.0f };
    float _groundTestMaxRotation[2] { 0.0f, 0.0f };
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

    if (ImGui::CollapsingHeader("Validation", ImGuiTreeNodeFlags_DefaultOpen)) {
        const char* tests[] = { "Off - locomotion", "Test 1 - standing/shove",
                                "Test 2 - weight shift", "Test 3 - single-leg lift",
                                "Diagnostic - grounded motor isolation" };
        ImGui::Combo("Active Test", &c.validationTest, tests, IM_ARRAYSIZE(tests));
        c.validationTest = glm::clamp(c.validationTest, 0, 4);

        ImGui::TextDisabled("Test 1: F6/F7 = impulse left/right");
        ImGui::DragFloat("Shove Impulse", &c.test1ShoveImpulse,
                         1.0f, 0.0f, 200.0f, "%.0f N*s");
        ImGui::DragFloat("Shove Cooldown", &c.test1ShoveCooldown,
                         0.1f, 0.0f, 10.0f, "%.1f s");
        ImGui::TextDisabled("baseline=%s settle=%.2f/1.00 s shoves=%d",
                            c._test1BaselineValid ? "ready" : "waiting",
                            c._test1SettleTime, c._test1Shoves);
        ImGui::TextDisabled("contact=(%s,%s) drift=(%.3f,%.3f) m",
                            c._test1ContactL ? "L" : "-",
                            c._test1ContactR ? "R" : "-",
                            c._test1FootDriftL, c._test1FootDriftR);
        ImGui::TextDisabled("speed=%.3f m/s peakTilt=%.1f deg recovery=%.2f s",
                            c._test1HorizontalSpeed, c._test1PeakTilt,
                            c._test1RecoveryTime);

        ImGui::Separator();
        ImGui::TextDisabled("Test 2: F6=left, F8=center, F7=right");
        ImGui::SliderFloat("Shift Fraction", &c.test2ShiftFraction, 0.1f, 0.9f);
        ImGui::DragFloat("Shift Time", &c.test2ShiftTime,
                         0.05f, 0.25f, 5.0f, "%.2f s");
        ImGui::DragFloat("Shift Frequency", &c.test2SupportFrequency,
                         0.1f, 0.25f, 5.0f, "%.2f Hz");
        ImGui::DragFloat("Shift Max Accel", &c.test2SupportMaxAccel,
                         0.1f, 0.1f, 10.0f, "%.1f m/s^2");
        ImGui::TextDisabled("baseline=%s settle=%.2f/1.00 s side=%d blend=%+.2f",
                            c._test2BaselineValid ? "ready" : "waiting",
                            c._test2SettleTime, c._test2TargetSide, c._test2Command);
        ImGui::TextDisabled("contact=(%s,%s) COM=%+.3f target=%+.3f err=%+.3f m",
                            c._test2ContactL ? "L" : "-",
                            c._test2ContactR ? "R" : "-",
                            c._test2ComLateral, c._test2TargetLateral,
                            c._test2TargetLateral - c._test2ComLateral);
        ImGui::TextDisabled("drift=(%.3f,%.3f) max=%.3f m peakTilt=%.1f deg",
                            c._test2FootDriftL, c._test2FootDriftR,
                            c._test2MaxFootDrift, c._test2PeakTilt);

        ImGui::Separator();
        ImGui::TextDisabled("Test 3: F6=left support/right lift, F7=mirror");
        ImGui::SliderFloat("Test 3 Support Fraction", &c.test3SupportFraction,
                           0.65f, 1.0f);
        ImGui::DragFloat("Takeoff Height", &c.test3TakeoffHeight,
                         0.0025f, 0.02f, 0.10f, "%.3f m");
        ImGui::DragFloat("Takeoff Timeout", &c.test3TakeoffTime,
                         0.025f, 0.10f, 1.0f, "%.2f s");
        ImGui::DragFloat("Takeoff Frequency", &c.test3TakeoffFrequency,
                         0.1f, 0.5f, 8.0f, "%.1f Hz");
        ImGui::DragFloat("Takeoff Max Force", &c.test3TakeoffMaxForce,
                         5.0f, 10.0f, 500.0f, "%.0f N");
        ImGui::DragFloat("Lift Height", &c.test3LiftHeight,
                         0.005f, 0.03f, 0.25f, "%.3f m");
        ImGui::DragFloat("Lift Time", &c.test3LiftTime,
                         0.05f, 0.20f, 2.0f, "%.2f s");
        ImGui::DragFloat("Hold Time", &c.test3HoldTime,
                         0.05f, 0.10f, 2.0f, "%.2f s");
        ImGui::DragFloat("Lower Time", &c.test3LowerTime,
                         0.05f, 0.20f, 2.0f, "%.2f s");
        ImGui::TextDisabled("baseline=%s phase=%d side=%d t=%.2f",
                            c._test3BaselineValid ? "ready" : "waiting",
                            c._test3Phase, c._test3SupportSide, c._test3PhaseTime);
        ImGui::TextDisabled("contact=(%s,%s) COM=%+.3f/%+.3f lift=%.2f clear=%.3f air=%.2f",
                            c._test3ContactL ? "L" : "-",
                            c._test3ContactR ? "R" : "-",
                            c._test3ComLateral, c._test3TargetLateral,
                            c._test3LiftBlend, c._test3Clearance,
                            c._test3AirborneTime);
        ImGui::TextDisabled("swingErr=%.3f poleDot=%+.2f stanceDrift=%.3f tilt=%.1f",
                            c._test3TargetError, c._test3KneePoleDot,
                            c._test3MaxStanceDrift, c._test3PeakTilt);

        ImGui::Separator();
        ImGui::TextDisabled("Ground diagnostic: F6=next, F7=previous, F8=recapture");
        ImGui::TextDisabled("stage=%d baseline=%s stageTime=%.2f s",
                            c._groundTestStage,
                            c._groundTestBaselineValid ? "ready" : "waiting",
                            c._groundTestStageTime);
        ImGui::TextDisabled("L speed=%.3f/%.3f m/s ang=%.1f/%.1f deg/s",
                            c._groundTestFootSpeed[0], c._groundTestMaxSpeed[0],
                            c._groundTestFootAngularSpeed[0], c._groundTestMaxAngularSpeed[0]);
        ImGui::TextDisabled("R speed=%.3f/%.3f m/s ang=%.1f/%.1f deg/s",
                            c._groundTestFootSpeed[1], c._groundTestMaxSpeed[1],
                            c._groundTestFootAngularSpeed[1], c._groundTestMaxAngularSpeed[1]);
    }

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
        ImGui::DragFloat("Step Length", &c.proceduralStepLength, 0.01f, 0.05f, 0.7f, "%.2f m");
        ImGui::DragFloat("Step Width", &c.proceduralStepWidth, 0.01f, 0.05f, 0.4f, "%.2f m");
        ImGui::DragFloat("Plant Lower Time", &c.proceduralPlantTime, 0.01f, 0.05f, 0.6f, "%.2f s");
        ImGui::DragFloat("IK Max Reach", &c.proceduralMaxReach, 0.005f, 0.75f, 0.99f);
        ImGui::DragFloat("Pose Response", &c.proceduralPoseResponse, 0.01f, 0.02f, 0.5f, "%.2f s");
        ImGui::DragFloat("Support Shift", &c.proceduralSupportShift, 0.01f, 0.0f, 1.0f);
        ImGui::DragFloat("Support Frequency", &c.proceduralSupportFrequency, 0.1f, 0.5f, 6.0f, "%.1f Hz");
        ImGui::DragFloat("Support Max Accel", &c.proceduralSupportMaxAccel, 0.1f, 0.5f, 12.0f);
        ImGui::DragFloat("Pelvis Bob", &c.proceduralPelvisBob, 0.002f, 0.0f, 0.08f, "%.3f m");
        ImGui::DragFloat("Torso Roll", &c.proceduralTorsoRollDeg, 0.25f, 0.0f, 12.0f, "%.1f deg");
        ImGui::DragFloat("Torso Pitch", &c.proceduralTorsoPitchDeg, 0.25f, -10.0f, 15.0f, "%.1f deg");
        ImGui::DragFloat("Foot Lock Frequency", &c.proceduralFootLockFrequency, 0.1f, 0.5f, 12.0f, "%.1f Hz");
        ImGui::DragFloat("Foot Lock Mass", &c.proceduralFootLockEffectiveMass, 1.0f, 1.0f, 100.0f, "%.1f kg");
        ImGui::DragFloat("Foot Lock Max Force", &c.proceduralFootLockMaxForce, 25.0f, 0.0f, 5000.0f, "%.0f N");
        ImGui::DragFloat("Foot Lock Tolerance", &c.proceduralFootLockTolerance, 0.002f, 0.005f, 0.20f, "%.3f m");
        ImGui::DragFloat("Weight Shift Tolerance", &c.proceduralWeightShiftTolerance, 0.005f, 0.01f, 0.25f, "%.3f m");
        ImGui::SliderFloat("Stance Support Scale", &c.proceduralStanceSupportScale, 0.0f, 1.0f);
        ImGui::DragFloat("Airborne Confirm", &c.airborneConfirmTime, 0.01f, 0.0f, 0.25f);
        ImGui::DragFloat("Min Plant Forward", &c.minPlantForward, 0.005f, 0.0f, 0.4f, "%.3f m");
        ImGui::DragFloat("Max Plant Y Speed", &c.maxPlantVerticalSpeed, 0.1f, 0.1f, 10.0f);
        ImGui::TextDisabled("contact=%s airborne=%s forward=%.3f clear=%.3f y=%.3f/%.3f vy=%+.2f",
                            c._swingContact ? "yes" : "no",
                            c._swingWasAirborne ? "yes" : "no",
                            c._swingForward, c._swingClearance,
                            c._swingFootY, c._liftTargetY, c._swingFootVy);
        ImGui::TextDisabled("lock=%.3f m  max=%.3f m  acquire=%.2f/0.20 s",
                            c._stanceLockError, c._maxStanceLockError,
                            c._plantAcquireTime);
        ImGui::TextDisabled("touchdown height error=%.3f m", c._touchdownHeightError);
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
    j["validationTest"] = c.validationTest;
    j["test1ShoveImpulse"] = c.test1ShoveImpulse;
    j["test1ShoveCooldown"] = c.test1ShoveCooldown;
    j["test2ShiftFraction"] = c.test2ShiftFraction;
    j["test2ShiftTime"] = c.test2ShiftTime;
    j["test2SupportFrequency"] = c.test2SupportFrequency;
    j["test2SupportMaxAccel"] = c.test2SupportMaxAccel;
    j["test3SupportFraction"] = c.test3SupportFraction;
    j["test3TakeoffHeight"] = c.test3TakeoffHeight;
    j["test3TakeoffTime"] = c.test3TakeoffTime;
    j["test3TakeoffFrequency"] = c.test3TakeoffFrequency;
    j["test3TakeoffMaxForce"] = c.test3TakeoffMaxForce;
    j["test3LiftHeight"] = c.test3LiftHeight;
    j["test3LiftTime"] = c.test3LiftTime;
    j["test3HoldTime"] = c.test3HoldTime;
    j["test3LowerTime"] = c.test3LowerTime;
    j["test3ContactSettleTime"] = c.test3ContactSettleTime;
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
    j["proceduralStepLength"] = c.proceduralStepLength;
    j["proceduralStepWidth"] = c.proceduralStepWidth;
    j["proceduralPlantTime"] = c.proceduralPlantTime;
    j["proceduralMaxReach"] = c.proceduralMaxReach;
    j["proceduralPoseResponse"] = c.proceduralPoseResponse;
    j["proceduralSupportShift"] = c.proceduralSupportShift;
    j["proceduralSupportFrequency"] = c.proceduralSupportFrequency;
    j["proceduralSupportMaxAccel"] = c.proceduralSupportMaxAccel;
    j["proceduralPelvisBob"] = c.proceduralPelvisBob;
    j["proceduralTorsoRollDeg"] = c.proceduralTorsoRollDeg;
    j["proceduralTorsoPitchDeg"] = c.proceduralTorsoPitchDeg;
    j["proceduralFootLockFrequency"] = c.proceduralFootLockFrequency;
    j["proceduralFootLockEffectiveMass"] = c.proceduralFootLockEffectiveMass;
    j["proceduralFootLockMaxForce"] = c.proceduralFootLockMaxForce;
    j["proceduralFootLockTolerance"] = c.proceduralFootLockTolerance;
    j["proceduralWeightShiftTolerance"] = c.proceduralWeightShiftTolerance;
    j["proceduralStanceSupportScale"] = c.proceduralStanceSupportScale;
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
    c.validationTest = j.value("validationTest", 3);
    c.test1ShoveImpulse = j.value("test1ShoveImpulse", 25.0f);
    c.test1ShoveCooldown = j.value("test1ShoveCooldown", 3.0f);
    c.test2ShiftFraction = j.value("test2ShiftFraction", 0.65f);
    c.test2ShiftTime = j.value("test2ShiftTime", 1.50f);
    c.test2SupportFrequency = j.value("test2SupportFrequency", 1.50f);
    c.test2SupportMaxAccel = j.value("test2SupportMaxAccel", 3.0f);
    c.test3SupportFraction = j.value("test3SupportFraction", 0.92f);
    c.test3TakeoffHeight = j.value("test3TakeoffHeight", 0.060f);
    c.test3TakeoffTime = j.value("test3TakeoffTime", 0.50f);
    c.test3TakeoffFrequency = j.value("test3TakeoffFrequency", 3.5f);
    c.test3TakeoffMaxForce = j.value("test3TakeoffMaxForce", 180.0f);
    c.test3LiftHeight = j.value("test3LiftHeight", 0.10f);
    c.test3LiftTime = j.value("test3LiftTime", 0.65f);
    c.test3HoldTime = j.value("test3HoldTime", 0.50f);
    c.test3LowerTime = j.value("test3LowerTime", 0.65f);
    c.test3ContactSettleTime = j.value("test3ContactSettleTime", 0.30f);
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
    c.proceduralStepLength = j.value("proceduralStepLength", 0.30f);
    c.proceduralStepWidth = j.value("proceduralStepWidth", 0.18f);
    c.proceduralPlantTime = j.value("proceduralPlantTime", 0.20f);
    c.proceduralMaxReach = j.value("proceduralMaxReach", 0.94f);
    c.proceduralPoseResponse = j.value("proceduralPoseResponse", 0.10f);
    c.proceduralSupportShift = j.value("proceduralSupportShift", 0.70f);
    c.proceduralSupportFrequency = j.value("proceduralSupportFrequency", 2.0f);
    c.proceduralSupportMaxAccel = j.value("proceduralSupportMaxAccel", 4.0f);
    c.proceduralPelvisBob = j.value("proceduralPelvisBob", 0.018f);
    c.proceduralTorsoRollDeg = j.value("proceduralTorsoRollDeg", 3.0f);
    c.proceduralTorsoPitchDeg = j.value("proceduralTorsoPitchDeg", 2.0f);
    c.proceduralFootLockFrequency = j.value("proceduralFootLockFrequency", 2.5f);
    c.proceduralFootLockEffectiveMass = j.value("proceduralFootLockEffectiveMass", 12.0f);
    c.proceduralFootLockMaxForce = j.value("proceduralFootLockMaxForce", 600.0f);
    c.proceduralFootLockTolerance = j.value("proceduralFootLockTolerance", 0.04f);
    c.proceduralWeightShiftTolerance = j.value("proceduralWeightShiftTolerance", 0.07f);
    c.proceduralStanceSupportScale = j.value("proceduralStanceSupportScale", 0.55f);
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
                rag.locomotionFootLockBones[0] = rag.locomotionFootLockBones[1] = -1;
                rag.locomotionFootLockWeights[0] = rag.locomotionFootLockWeights[1] = 0.0f;
                rag.locomotionHeightOffset = 0.0f;
                ResetGait(comp);
                ResetTest1(comp);
                ResetTest2(comp);
                ResetTest3(comp);
                ResetGroundTest(comp);
                comp._validationActiveTest = -1;
                continue;
            }

            // A scene can author the ragdoll as Powered, in which case the Animated->Powered
            // flip never runs and the -1 sentinel would double the grace period.
            if (comp._timeSincePowered < 0.0f) comp._timeSincePowered = 0.0f;
            comp._timeSincePowered += dt;
            rag.strength = glm::clamp(comp.consciousness, 0.0f, 1.0f);

            comp.validationTest = glm::clamp(comp.validationTest, 0, 4);
            if (comp.validationTest != comp._validationActiveTest) {
                ResetGait(comp);
                ResetTest1(comp);
                ResetTest2(comp);
                ResetTest3(comp);
                ResetGroundTest(comp);
                comp._validationActiveTest = comp.validationTest;
            }

            const float speed = magnitude > comp.deadzone
                ? (magnitude - comp.deadzone) / (1.0f - comp.deadzone) * comp.maxSpeed
                : 0.0f;
            glm::vec3 moveDir = camRight * h + camForward * f;
            moveDir.y = 0.0f;
            moveDir = glm::length(moveDir) > 1e-4f ? glm::normalize(moveDir) : glm::vec3(0.0f);
            // Validation is an isolation mode. WASD is deliberately inert so an accidental
            // input cannot enable a gait controller during a foundation measurement.
            const bool wantsToWalk = comp.validationTest == 0 && speed > comp.minWalkSpeed;

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

            // Log input edges next to the controller state that receives them. This makes a
            // test reproducible from loco_debug.log without requiring a video or relying on
            // a description of when a key was pressed. Axis values use the controller's
            // forward-positive convention (f), while moveDir is the camera-relative world
            // command after normalization. Re-querying an edge is safe: Input does not
            // consume pressed/released state.
            auto logInputEdge = [&](Key key, const char* name) {
                const bool pressed = Input::IsKeyPressed(key);
                const bool released = Input::IsKeyReleased(key);
                if (!pressed && !released) return;
                spdlog::info(
                    "[LocoInput] t={:.2f} key={} event={} axis=({:+.2f},{:+.2f}) "
                    "move=({:+.2f},{:+.2f}) speed={:.2f} walk={} test={} "
                    "phase={} baseline={} grounded={} ready={}",
                    comp._timeSincePowered, name, pressed ? "PRESS" : "RELEASE",
                    h, f, moveDir.x, moveDir.z, speed,
                    wantsToWalk ? "yes" : "no", comp.validationTest,
                    comp._test3Phase, comp._test3BaselineValid ? "ready" : "waiting",
                    comp._grounded ? "yes" : "no", ready ? "yes" : "no");
            };
            logInputEdge(Key::W, "W");
            logInputEdge(Key::A, "A");
            logInputEdge(Key::S, "S");
            logInputEdge(Key::D, "D");
            logInputEdge(Key::F6, "F6");
            logInputEdge(Key::F7, "F7");
            logInputEdge(Key::F8, "F8");

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
            if (wantsToWalk != comp._wasWalking) {
                // Re-anchor once at a start/stop boundary. Never chase the simulated foot
                // every frame: that was the old false "lock" that reported zero error while
                // both feet slid away with the pelvis.
                comp._legL.planted = false;
                comp._legR.planted = false;
                comp._legL.lockWeight = 0.0f;
                comp._legR.lockWeight = 0.0f;
                comp._plantAcquireTime = 0.0f;
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
            // Input velocity is planning data only. The pelvis is never pushed toward it;
            // travel can happen only after a forward foot plant and support transfer.
            comp._desiredVelocity = moveDir * speed * comp._gaitWeight;
            rag.locomotionTargetVel = glm::vec3(0.0f);
            rag.locomotionHeightOffset = 0.0f;
            rag.locomotionHipTorque[0] = rag.locomotionHipTorque[1] = glm::vec3(0.0f);
            rag.locomotionHipBones[0] = rag.locomotionHipBones[1] = -1;
            rag.locomotionLiftBone = -1;
            for (int& bone : rag.locomotionDisabledMotorBones) bone = -1;
            rag.locomotionFootLockBones[0] = rag.locomotionFootLockBones[1] = -1;
            rag.locomotionFootLockWeights[0] = rag.locomotionFootLockWeights[1] = 0.0f;
            rag.locomotionFootLockFrequency = glm::max(comp.proceduralFootLockFrequency, 0.0f);
            rag.locomotionFootLockEffectiveMass = glm::max(
                comp.proceduralFootLockEffectiveMass, 0.0f);
            rag.locomotionFootLockMaxForce = glm::max(comp.proceduralFootLockMaxForce, 0.0f);

            if (comp.validationTest > 0) {
                // Keep the validated bind/standing motor target, but bypass the gait
                // dispatcher entirely. With walking=false the procedural pose path cannot
                // acquire plants, solve swing IK, command support transfer, or arm locks.
                UpdateProceduralGait(scene, entity, comp, rag, false, dt);
                if (comp.validationTest == 1) {
                    UpdateTest1(scene, entity, comp, rag, ready, tiltDeg, dt);
                } else if (comp.validationTest == 2) {
                    UpdateTest2(scene, entity, comp, rag, ready, tiltDeg, dt);
                } else if (comp.validationTest == 3) {
                    UpdateTest3(scene, entity, comp, rag, ready, tiltDeg, dt);
                } else {
                    UpdateGroundTest(scene, entity, comp, rag, ready, tiltDeg, dt);
                }
            } else {
                UpdateGait(scene, entity, comp, rag, gaitRequested, dt);
            }

            if (comp.debug) {
                _debugTimer += dt;
                if (_debugTimer >= 0.25f) {
                    _debugTimer = 0.0f;
                    const glm::vec3 v = rag._locomotionRootVel;
                    spdlog::info("[Loco] s{} swing={} t={:.2f} gait={:.2f} pose={:.2f} d=({:+.3f},{:+.3f}) v=({:+.3f},{:+.3f}) hip=({:+.1f},{:+.1f}) tau=(T{:.0f},B{:.0f},A{:.0f}) target={:.2f} spd={:.2f} tilt={:.1f} steps={}",
                                 comp._stateIndex, comp._swingIsLeft ? "L" : "R",
                                 comp._stateTime, comp._gaitWeight, comp._poseBlend,
                                 comp._dSag, comp._dLat, comp._vSag, comp._vLat,
                                 comp._swingHipCmdDeg, comp._swingHipLatCmdDeg,
                                 comp._torsoTorque, comp._swingTorque, comp._stanceTorque,
                                 glm::length(glm::vec2(comp._desiredVelocity.x,
                                                       comp._desiredVelocity.z)),
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
                    if (comp.validationTest == 1) {
                        spdlog::info("[LocoTest1] t={:.2f} baseline={} settle={:.2f} contact=({},{}) drift=({:.3f},{:.3f}) speed={:.3f} tilt={:.1f} peak={:.1f} sinceShove={:.2f} recovery={:.2f} shoves={} locks=({:.2f},{:.2f})",
                                     comp._test1Time,
                                     comp._test1BaselineValid ? "READY" : "wait",
                                     comp._test1SettleTime,
                                     comp._test1ContactL ? "L" : "-",
                                     comp._test1ContactR ? "R" : "-",
                                     comp._test1FootDriftL, comp._test1FootDriftR,
                                     comp._test1HorizontalSpeed, tiltDeg,
                                     comp._test1PeakTilt, comp._test1SinceShove,
                                     comp._test1RecoveryTime, comp._test1Shoves,
                                     rag.locomotionFootLockWeights[0],
                                     rag.locomotionFootLockWeights[1]);
                    } else if (comp.validationTest == 2) {
                        spdlog::info("[LocoTest2] t={:.2f} baseline={} settle={:.2f} side={} blend={:+.2f} contact=({},{}) com={:+.3f} target={:+.3f} err={:+.3f} drift=({:.3f},{:.3f}) maxDrift={:.3f} tilt={:.1f} peak={:.1f} force={:.0f} sat={} locks=({:.2f},{:.2f})",
                                     comp._test2Time,
                                     comp._test2BaselineValid ? "READY" : "wait",
                                     comp._test2SettleTime,
                                     comp._test2TargetSide, comp._test2Command,
                                     comp._test2ContactL ? "L" : "-",
                                     comp._test2ContactR ? "R" : "-",
                                     comp._test2ComLateral, comp._test2TargetLateral,
                                     comp._test2TargetLateral - comp._test2ComLateral,
                                     comp._test2FootDriftL, comp._test2FootDriftR,
                                     comp._test2MaxFootDrift, tiltDeg,
                                     comp._test2PeakTilt,
                                     glm::length(rag._locomotionSupportForce),
                                     rag._locomotionSupportSaturated ? "YES" : "no",
                                     rag.locomotionFootLockWeights[0],
                                     rag.locomotionFootLockWeights[1]);
                    } else if (comp.validationTest == 3) {
                        spdlog::info("[LocoTest3] t={:.2f} baseline={} phase={} phaseT={:.2f} support={} com={:+.3f}/{:+.3f} contact=({},{}) lift={:.2f} clear={:.3f} air={:.2f} swingErr={:.3f} poleDot={:+.2f} stanceDrift={:.3f} tilt={:.1f} peak={:.1f} force={:.0f} sat={} locks=({:.2f},{:.2f}) liftAssist=(bone={} targetY={:.3f} force={:.0f}/{:.0f})",
                                     comp._test3Time,
                                     comp._test3BaselineValid ? "READY" : "wait",
                                     comp._test3Phase, comp._test3PhaseTime,
                                     comp._test3SupportSide,
                                     comp._test3ComLateral, comp._test3TargetLateral,
                                     comp._test3ContactL ? "L" : "-",
                                     comp._test3ContactR ? "R" : "-",
                                     comp._test3LiftBlend, comp._test3Clearance,
                                     comp._test3AirborneTime,
                                     comp._test3TargetError, comp._test3KneePoleDot,
                                     comp._test3MaxStanceDrift, tiltDeg,
                                     comp._test3PeakTilt,
                                     glm::length(rag._locomotionSupportForce),
                                     rag._locomotionSupportSaturated ? "YES" : "no",
                                     rag.locomotionFootLockWeights[0],
                                     rag.locomotionFootLockWeights[1],
                                     rag.locomotionLiftBone,
                                     rag.locomotionLiftTargetY,
                                     rag._locomotionLiftForce,
                                     rag.locomotionLiftMaxForce);
                        LogKneeDiagnostics(scene, entity, comp, rag);
                    } else if (comp.validationTest == 4) {
                        auto footSlot = [&](int bone) {
                            for (int i = 0; i < 2; ++i)
                                if (rag._locomotionFootBones[i] == bone) return i;
                            return -1;
                        };
                        const int slotL = footSlot(comp._legL.footIdx);
                        const int slotR = footSlot(comp._legR.footIdx);
                        const float penL = slotL >= 0 ? rag._locomotionFootPenetration[slotL] : 0.0f;
                        const float penR = slotR >= 0 ? rag._locomotionFootPenetration[slotR] : 0.0f;
                        const float soleL = slotL >= 0 ? rag._locomotionFootSoleMinY[slotL] : 0.0f;
                        const float soleR = slotR >= 0 ? rag._locomotionFootSoleMinY[slotR] : 0.0f;
                        const float contactYL = slotL >= 0 ? rag._locomotionFootContactPoint[slotL].y : 0.0f;
                        const float contactYR = slotR >= 0 ? rag._locomotionFootContactPoint[slotR].y : 0.0f;
                        spdlog::info("[LocoGround] t={:.2f} baseline={} stage={} ({}) stageT={:.2f} contact=({},{}) transitions=({},{}) speed=({:.3f}/{:.3f},{:.3f}/{:.3f}) ang=({:.1f}/{:.1f},{:.1f}/{:.1f}) disp=({:.4f}/{:.4f},{:.4f}/{:.4f}) rot=({:.2f}/{:.2f},{:.2f}/{:.2f}) soleY=({:.4f},{:.4f}) contactY=({:.4f},{:.4f}) penetration=({:+.5f},{:+.5f}) force={:.0f}",
                                     comp._groundTestTime,
                                     comp._groundTestBaselineValid ? "READY" : "wait",
                                     comp._groundTestStage,
                                     GroundTestStageName(comp._groundTestStage),
                                     comp._groundTestStageTime,
                                     comp._groundTestContact[0] ? "L" : "-",
                                     comp._groundTestContact[1] ? "R" : "-",
                                     comp._groundTestContactTransitions[0],
                                     comp._groundTestContactTransitions[1],
                                     comp._groundTestFootSpeed[0], comp._groundTestMaxSpeed[0],
                                     comp._groundTestFootSpeed[1], comp._groundTestMaxSpeed[1],
                                     comp._groundTestFootAngularSpeed[0], comp._groundTestMaxAngularSpeed[0],
                                     comp._groundTestFootAngularSpeed[1], comp._groundTestMaxAngularSpeed[1],
                                     comp._groundTestFootDisplacement[0], comp._groundTestMaxDisplacement[0],
                                     comp._groundTestFootDisplacement[1], comp._groundTestMaxDisplacement[1],
                                     comp._groundTestFootRotation[0], comp._groundTestMaxRotation[0],
                                     comp._groundTestFootRotation[1], comp._groundTestMaxRotation[1],
                                     soleL, soleR, contactYL, contactYR, penL, penR,
                                     glm::length(rag._locomotionSupportForce));
                    } else if (comp.assistedStepping) {
                        spdlog::info("[LocoStep] phase={} cycle={:.2f} acquire={:.2f} contact={} airborne={} footForward={:+.3f} clearance={:+.3f} footY={:.3f}/{:.3f} footVy={:+.2f} groundErr={:.3f} lockErr={:.3f} maxLock={:.3f} lockW=({:.2f},{:.2f}) lockF=({:.0f},{:.0f}) swingErr={:.3f}",
                                     comp._stateIndex, comp._stepPhase,
                                     comp._plantAcquireTime,
                                     comp._swingContact ? "yes" : "no",
                                     comp._swingWasAirborne ? "yes" : "no",
                                     comp._swingForward, comp._swingClearance, comp._swingFootY,
                                     comp._liftTargetY, comp._swingFootVy,
                                     comp._touchdownHeightError,
                                     comp._stanceLockError, comp._maxStanceLockError,
                                     rag.locomotionFootLockWeights[0],
                                     rag.locomotionFootLockWeights[1],
                                     rag._locomotionFootLockForce[0],
                                     rag._locomotionFootLockForce[1],
                                     comp._swingTargetError);
                    }
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
        c._stepPhase = 0.0f;
        c._stanceLockError = c._swingTargetError = 0.0f;
        c._maxStanceLockError = 0.0f;
        c._plantAcquireTime = 0.0f;
        c._touchdownHeightError = 0.0f;
        c._desiredVelocity = glm::vec3(0.0f);
        c._torsoTorque = c._swingTorque = c._stanceTorque = 0.0f;
        c._torsoP = c._torsoD = c._swingP = c._swingD = 0.0f;
        c._saturated = false;
        c._legL = {};
        c._legR = {};
    }

    static void ResetTest1(Comp& c)
    {
        c._test1BaselineValid = false;
        c._test1ContactL = c._test1ContactR = false;
        c._test1Time = 0.0f;
        c._test1SettleTime = 0.0f;
        c._test1SinceShove = -1.0f;
        c._test1RecoveryTime = -1.0f;
        c._test1PeakTilt = 0.0f;
        c._test1HorizontalSpeed = 0.0f;
        c._test1FootDriftL = c._test1FootDriftR = 0.0f;
        c._test1Shoves = 0;
        c._test1FootBaselineL = c._test1FootBaselineR = glm::vec3(0.0f);
    }

    static void ResetTest2(Comp& c)
    {
        c._test2BaselineValid = false;
        c._test2ContactL = c._test2ContactR = false;
        c._test2Time = 0.0f;
        c._test2SettleTime = 0.0f;
        c._test2Command = 0.0f;
        c._test2TargetSide = 0;
        c._test2ComLateral = c._test2TargetLateral = 0.0f;
        c._test2FootDriftL = c._test2FootDriftR = 0.0f;
        c._test2MaxFootDrift = 0.0f;
        c._test2PeakTilt = 0.0f;
        c._test2FootBaselineL = c._test2FootBaselineR = glm::vec3(0.0f);
        c._test2ComBaseline = glm::vec3(0.0f);
        c._test2Right = glm::vec3(1.0f, 0.0f, 0.0f);
        c._test2SupportTarget = glm::vec3(0.0f);
    }

    static void ResetTest3(Comp& c)
    {
        c._test3BaselineValid = false;
        c._test3ContactL = c._test3ContactR = false;
        c._test3Time = 0.0f;
        c._test3SettleTime = 0.0f;
        c._test3Phase = 0;
        c._test3PhaseTime = 0.0f;
        c._test3SupportSide = 0;
        c._test3ComCommand = 0.0f;
        c._test3ComLateral = c._test3TargetLateral = 0.0f;
        c._test3LiftBlend = 0.0f;
        c._test3LiftStartBlend = 0.0f;
        c._test3AirborneTime = 0.0f;
        c._test3Clearance = 0.0f;
        c._test3TargetError = 0.0f;
        c._test3KneePoleDot = 1.0f;
        c._test3MaxStanceDrift = 0.0f;
        c._test3PeakTilt = 0.0f;
        c._test3Aborted = false;
        c._test3FootBaselineL = c._test3FootBaselineR = glm::vec3(0.0f);
        c._test3ComBaseline = glm::vec3(0.0f);
        c._test3Right = glm::vec3(1.0f, 0.0f, 0.0f);
        c._test3SupportTarget = glm::vec3(0.0f);
        c._test3SwingStart = glm::vec3(0.0f);
    }

    static void ResetGroundTest(Comp& c)
    {
        c._groundTestBaselineValid = false;
        c._groundTestTime = 0.0f;
        c._groundTestSettleTime = 0.0f;
        c._groundTestStage = 0;
        c._groundTestStageTime = 0.0f;
        c._groundTestComTarget = glm::vec3(0.0f);
        for (int i = 0; i < 2; ++i) {
            c._groundTestFootStart[i] = glm::vec3(0.0f);
            c._groundTestFootRotStart[i] = glm::quat(1, 0, 0, 0);
            c._groundTestContact[i] = false;
            c._groundTestPrevContact[i] = false;
            c._groundTestContactTransitions[i] = 0;
            c._groundTestFootSpeed[i] = 0.0f;
            c._groundTestFootAngularSpeed[i] = 0.0f;
            c._groundTestFootDisplacement[i] = 0.0f;
            c._groundTestFootRotation[i] = 0.0f;
            c._groundTestMaxSpeed[i] = 0.0f;
            c._groundTestMaxAngularSpeed[i] = 0.0f;
            c._groundTestMaxDisplacement[i] = 0.0f;
            c._groundTestMaxRotation[i] = 0.0f;
        }
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
                leg.kneeHingeAxis = body.hingeAxisLocal;
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

    static void LogKneeDiagnostics(Scene& scene, entt::entity entity, const Comp& comp,
                                   const RagdollComponent& rag)
    {
        if (!scene.Has<SkinnedMeshComponent>(entity) || !rag.config
            || !ValidLeg(comp._legL) || !ValidLeg(comp._legR)) return;

        const auto& skeleton = scene.Get<SkinnedMeshComponent>(entity).skeleton;
        const glm::mat4 entityWorld = scene.GetTransformSystem().GetWorldMatrix(entity);
        const glm::vec3 c0(entityWorld[0]), c1(entityWorld[1]), c2(entityWorld[2]);
        const float scale = glm::max(
            (glm::length(c0) + glm::length(c1) + glm::length(c2)) / 3.0f, 1e-6f);

        auto bindPosition = [&](int bone) {
            return glm::vec3((entityWorld
                * glm::inverse(skeleton.bones[bone].inverseBind))[3]);
        };
        auto bodyDef = [&](int bone) -> const RagdollBodyDef* {
            if (bone < 0 || bone >= static_cast<int>(skeleton.bones.size())) return nullptr;
            const std::string& name = skeleton.bones[bone].name;
            for (const auto& def : rag.config->bodies)
                if (def.boneName == name) return &def;
            return nullptr;
        };

        struct Measurement {
            float upper = 0.0f, upperBind = 0.0f;
            float lower = 0.0f, lowerBind = 0.0f;
            float kneeColliderGap = -1.0f, ankleColliderGap = -1.0f;
            bool valid = false;
        };
        auto measure = [&](const Leg& leg) {
            Measurement m;
            bool hipOk = false, kneeOk = false, ankleOk = false;
            const glm::vec3 hip = Physics::GetRagdollBonePosition(rag, leg.hipIdx, &hipOk);
            const glm::vec3 knee = Physics::GetRagdollBonePosition(rag, leg.kneeIdx, &kneeOk);
            const glm::vec3 ankle = Physics::GetRagdollBonePosition(rag, leg.ankleIdx, &ankleOk);
            if (!hipOk || !kneeOk || !ankleOk) return m;

            m.upper = glm::length(knee - hip);
            m.lower = glm::length(ankle - knee);
            m.upperBind = glm::length(bindPosition(leg.kneeIdx) - bindPosition(leg.hipIdx));
            m.lowerBind = glm::length(bindPosition(leg.ankleIdx) - bindPosition(leg.kneeIdx));

            auto colliderTipGap = [&](int bodyBone, const glm::vec3& bodyPosition,
                                      const glm::vec3& expectedTip) {
                const RagdollBodyDef* def = bodyDef(bodyBone);
                bool rotationOk = false;
                const glm::quat bodyRotation = Physics::GetRagdollBoneRotation(
                    rag, bodyBone, &rotationOk);
                if (!def || !rotationOk || def->shape != RagdollBodyDef::Shape::Capsule
                    || glm::dot(def->twistAxisLocal, def->twistAxisLocal) < 1e-8f)
                    return -1.0f;
                const float fullLength = 2.0f * (def->halfHeight + def->radius) * scale;
                const glm::vec3 tip = bodyPosition
                    + bodyRotation * glm::normalize(def->twistAxisLocal) * fullLength;
                return glm::length(tip - expectedTip);
            };
            m.kneeColliderGap = colliderTipGap(leg.hipIdx, hip, knee);
            m.ankleColliderGap = colliderTipGap(leg.kneeIdx, knee, ankle);
            m.valid = true;
            return m;
        };

        const Measurement left = measure(comp._legL);
        const Measurement right = measure(comp._legR);
        if (!left.valid || !right.valid) return;
        spdlog::info(
            "[LocoKnee] L stretchMM=({:+.2f},{:+.2f}) colliderGapMM=({:.2f},{:.2f}) "
            "segmentM=({:.4f}/{:.4f},{:.4f}/{:.4f}) R stretchMM=({:+.2f},{:+.2f}) "
            "colliderGapMM=({:.2f},{:.2f}) segmentM=({:.4f}/{:.4f},{:.4f}/{:.4f})",
            (left.upper - left.upperBind) * 1000.0f,
            (left.lower - left.lowerBind) * 1000.0f,
            left.kneeColliderGap * 1000.0f, left.ankleColliderGap * 1000.0f,
            left.upper, left.upperBind, left.lower, left.lowerBind,
            (right.upper - right.upperBind) * 1000.0f,
            (right.lower - right.lowerBind) * 1000.0f,
            right.kneeColliderGap * 1000.0f, right.ankleColliderGap * 1000.0f,
            right.upper, right.upperBind, right.lower, right.lowerBind);
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

    static void UpdateTest1(Scene& scene, entt::entity entity, Comp& comp,
                            RagdollComponent& rag, bool ready, float tiltDeg, float dt)
    {
        if (!scene.Has<SkinnedMeshComponent>(entity) || !ValidLeg(comp._legL)
            || !ValidLeg(comp._legR)) return;

        bool leftPositionOk = false, rightPositionOk = false;
        const glm::vec3 leftFoot = Physics::GetRagdollBonePosition(
            rag, comp._legL.footIdx, &leftPositionOk);
        const glm::vec3 rightFoot = Physics::GetRagdollBonePosition(
            rag, comp._legR.footIdx, &rightPositionOk);
        if (!leftPositionOk || !rightPositionOk) return;

        comp._test1ContactL = FootContact(rag, comp._legL.footIdx);
        comp._test1ContactR = FootContact(rag, comp._legR.footIdx);
        comp._test1HorizontalSpeed = glm::length(glm::vec2(
            rag._locomotionRootVel.x, rag._locomotionRootVel.z));

        if (!ready) {
            comp._test1Time = 0.0f;
            comp._test1SettleTime = 0.0f;
            return;
        }
        comp._test1Time += dt;

        const glm::vec3 leftVelocity = Physics::GetRagdollBoneLinearVelocity(
            rag, comp._legL.footIdx);
        const glm::vec3 rightVelocity = Physics::GetRagdollBoneLinearVelocity(
            rag, comp._legR.footIdx);
        const bool settled = comp._test1ContactL && comp._test1ContactR
            && glm::length(leftVelocity) < 0.15f
            && glm::length(rightVelocity) < 0.15f
            && comp._test1HorizontalSpeed < 0.15f
            && tiltDeg < 15.0f;

        if (!comp._test1BaselineValid) {
            comp._test1SettleTime = settled ? comp._test1SettleTime + dt : 0.0f;
            if (comp._test1SettleTime >= 1.0f) {
                comp._test1BaselineValid = true;
                comp._test1FootBaselineL = leftFoot;
                comp._test1FootBaselineR = rightFoot;
                comp._test1PeakTilt = tiltDeg;
                spdlog::info("[LocoTest1] baseline acquired at t={:.2f} footL=({:+.3f},{:+.3f},{:+.3f}) footR=({:+.3f},{:+.3f},{:+.3f}) tilt={:.1f}",
                             comp._test1Time,
                             leftFoot.x, leftFoot.y, leftFoot.z,
                             rightFoot.x, rightFoot.y, rightFoot.z, tiltDeg);
            }
        }

        if (comp._test1BaselineValid) {
            comp._test1FootDriftL = glm::length(glm::vec2(
                leftFoot.x - comp._test1FootBaselineL.x,
                leftFoot.z - comp._test1FootBaselineL.z));
            comp._test1FootDriftR = glm::length(glm::vec2(
                rightFoot.x - comp._test1FootBaselineR.x,
                rightFoot.z - comp._test1FootBaselineR.z));
        }

        if (comp._test1SinceShove >= 0.0f) {
            comp._test1SinceShove += dt;
            comp._test1PeakTilt = glm::max(comp._test1PeakTilt, tiltDeg);
            if (comp._test1RecoveryTime < 0.0f && comp._test1SinceShove >= 0.10f
                && settled) {
                comp._test1RecoveryTime = comp._test1SinceShove;
                spdlog::info("[LocoTest1] recovered in {:.2f}s peakTilt={:.1f} drift=({:.3f},{:.3f})",
                             comp._test1RecoveryTime, comp._test1PeakTilt,
                             comp._test1FootDriftL, comp._test1FootDriftR);
            }
        }

        const bool shoveLeft = Input::IsKeyPressed(Key::F6);
        const bool shoveRight = Input::IsKeyPressed(Key::F7);
        if (!shoveLeft && !shoveRight) return;

        const bool cooldownComplete = comp._test1SinceShove < 0.0f
            || comp._test1SinceShove >= glm::max(comp.test1ShoveCooldown, 0.0f);
        if (!comp._test1BaselineValid || !cooldownComplete) {
            spdlog::warn("[LocoTest1] shove ignored: baseline={} cooldown={:.2f}/{:.2f}",
                         comp._test1BaselineValid ? "ready" : "waiting",
                         comp._test1SinceShove, comp.test1ShoveCooldown);
            return;
        }

        glm::vec3 right = rightFoot - leftFoot;
        right.y = 0.0f;
        if (glm::dot(right, right) < 1e-8f) right = comp._right;
        if (glm::dot(right, right) < 1e-8f) right = glm::vec3(1, 0, 0);
        right = glm::normalize(right);
        const glm::vec3 direction = shoveLeft ? -right : right;

        const auto& skeleton = scene.Get<SkinnedMeshComponent>(entity).skeleton;
        int shoveBone = skeleton.Find(comp.torsoBone);
        if (shoveBone < 0) shoveBone = Physics::GetRagdollRootBone(rag);
        const float magnitude = glm::max(comp.test1ShoveImpulse, 0.0f);
        if (!Physics::AddRagdollBoneImpulse(rag, shoveBone, direction * magnitude)) {
            spdlog::error("[LocoTest1] failed to apply shove: no ragdoll body for bone {}",
                          shoveBone);
            return;
        }

        comp._test1SinceShove = 0.0f;
        comp._test1RecoveryTime = -1.0f;
        comp._test1PeakTilt = tiltDeg;
        ++comp._test1Shoves;
        spdlog::info("[LocoTest1] shove {} impulse={:.1f}N*s dir=({:+.2f},{:+.2f},{:+.2f}) bone={}",
                     shoveLeft ? "LEFT" : "RIGHT", magnitude,
                     direction.x, direction.y, direction.z, shoveBone);
    }

    static void UpdateTest2(Scene& scene, entt::entity entity, Comp& comp,
                            RagdollComponent& rag, bool ready, float tiltDeg, float dt)
    {
        if (!scene.Has<SkinnedMeshComponent>(entity) || !ValidLeg(comp._legL)
            || !ValidLeg(comp._legR) || !rag._locomotionCOMValid) return;

        bool leftPositionOk = false, rightPositionOk = false;
        const glm::vec3 leftFoot = Physics::GetRagdollBonePosition(
            rag, comp._legL.footIdx, &leftPositionOk);
        const glm::vec3 rightFoot = Physics::GetRagdollBonePosition(
            rag, comp._legR.footIdx, &rightPositionOk);
        if (!leftPositionOk || !rightPositionOk) return;

        comp._test2ContactL = FootContact(rag, comp._legL.footIdx);
        comp._test2ContactR = FootContact(rag, comp._legR.footIdx);
        if (!ready) {
            comp._test2Time = 0.0f;
            comp._test2SettleTime = 0.0f;
            return;
        }
        comp._test2Time += dt;

        const glm::vec3 leftVelocity = Physics::GetRagdollBoneLinearVelocity(
            rag, comp._legL.footIdx);
        const glm::vec3 rightVelocity = Physics::GetRagdollBoneLinearVelocity(
            rag, comp._legR.footIdx);
        const float horizontalSpeed = glm::length(glm::vec2(
            rag._locomotionRootVel.x, rag._locomotionRootVel.z));
        const bool settled = comp._test2ContactL && comp._test2ContactR
            && glm::length(leftVelocity) < 0.15f
            && glm::length(rightVelocity) < 0.15f
            && horizontalSpeed < 0.15f
            && tiltDeg < 15.0f;

        if (!comp._test2BaselineValid) {
            comp._test2SettleTime = settled ? comp._test2SettleTime + dt : 0.0f;
            if (comp._test2SettleTime >= 1.0f) {
                glm::vec3 right = rightFoot - leftFoot;
                right.y = 0.0f;
                if (glm::dot(right, right) < 1e-8f) right = comp._right;
                if (glm::dot(right, right) < 1e-8f) right = glm::vec3(1, 0, 0);

                comp._test2BaselineValid = true;
                comp._test2FootBaselineL = leftFoot;
                comp._test2FootBaselineR = rightFoot;
                comp._test2ComBaseline = rag._locomotionCOM;
                comp._test2Right = glm::normalize(right);
                comp._test2SupportTarget = comp._test2ComBaseline;
                comp._test2PeakTilt = tiltDeg;
                spdlog::info("[LocoTest2] baseline acquired at t={:.2f} span={:.3f}m tilt={:.1f}",
                             comp._test2Time,
                             glm::dot(comp._test2FootBaselineR
                                      - comp._test2FootBaselineL,
                                      comp._test2Right), tiltDeg);
            }
        }

        const bool chooseLeft = Input::IsKeyPressed(Key::F6);
        const bool chooseRight = Input::IsKeyPressed(Key::F7);
        const bool chooseCenter = Input::IsKeyPressed(Key::F8);
        if ((chooseLeft || chooseRight || chooseCenter) && !comp._test2BaselineValid) {
            spdlog::warn("[LocoTest2] target ignored until baseline=ready");
        } else if (comp._test2BaselineValid && (chooseLeft || chooseRight || chooseCenter)) {
            comp._test2TargetSide = chooseLeft ? -1 : (chooseRight ? 1 : 0);
            spdlog::info("[LocoTest2] target={}",
                         comp._test2TargetSide < 0 ? "LEFT"
                         : (comp._test2TargetSide > 0 ? "RIGHT" : "CENTER"));
        }

        if (!comp._test2BaselineValid) return;

        comp._test2Command = Approach(
            comp._test2Command, static_cast<float>(comp._test2TargetSide),
            dt / glm::max(comp.test2ShiftTime, 0.01f));

        const float fraction = glm::clamp(comp.test2ShiftFraction, 0.0f, 1.0f);
        const float leftAvailable = glm::dot(
            comp._test2FootBaselineL - comp._test2ComBaseline, comp._test2Right);
        const float rightAvailable = glm::dot(
            comp._test2FootBaselineR - comp._test2ComBaseline, comp._test2Right);
        const float lateralOffset = comp._test2Command < 0.0f
            ? -comp._test2Command * glm::min(leftAvailable, 0.0f) * fraction
            :  comp._test2Command * glm::max(rightAvailable, 0.0f) * fraction;
        const glm::vec3 target = comp._test2ComBaseline
                               + comp._test2Right * lateralOffset;
        glm::vec3 targetVelocity = dt > 1e-6f
            ? (target - comp._test2SupportTarget) / dt : glm::vec3(0.0f);
        targetVelocity.y = 0.0f;
        comp._test2SupportTarget = target;

        rag.locomotionSupportTarget = target;
        rag.locomotionSupportTargetVel = targetVelocity;
        rag.locomotionSupportTargetWeight = 1.0f;
        rag.locomotionCOMSupportFreq = glm::max(comp.test2SupportFrequency, 0.0f);
        rag.locomotionCOMSupportMaxAccel = glm::max(comp.test2SupportMaxAccel, 0.0f);

        comp._test2ComLateral = glm::dot(
            rag._locomotionCOM - comp._test2ComBaseline, comp._test2Right);
        comp._test2TargetLateral = lateralOffset;
        comp._test2FootDriftL = glm::length(glm::vec2(
            leftFoot.x - comp._test2FootBaselineL.x,
            leftFoot.z - comp._test2FootBaselineL.z));
        comp._test2FootDriftR = glm::length(glm::vec2(
            rightFoot.x - comp._test2FootBaselineR.x,
            rightFoot.z - comp._test2FootBaselineR.z));
        comp._test2MaxFootDrift = glm::max(comp._test2MaxFootDrift,
            glm::max(comp._test2FootDriftL, comp._test2FootDriftR));
        comp._test2PeakTilt = glm::max(comp._test2PeakTilt, tiltDeg);

        if (comp.debug) {
            DebugDraw::Sphere(comp._test2ComBaseline, 0.025f, {0.2f, 0.7f, 1.0f});
            DebugDraw::Sphere(target, 0.035f, {1.0f, 0.7f, 0.1f});
            DebugDraw::Line(comp._test2ComBaseline, target, {1.0f, 0.7f, 0.1f});
        }
    }

    static void UpdateTest3(Scene& scene, entt::entity entity, Comp& comp,
                            RagdollComponent& rag, bool ready, float tiltDeg, float dt)
    {
        if (!scene.Has<SkinnedMeshComponent>(entity)
            || !scene.Has<AnimatorComponent>(entity)
            || !ValidLeg(comp._legL) || !ValidLeg(comp._legR)
            || !rag._locomotionCOMValid) return;

        auto& mesh = scene.Get<SkinnedMeshComponent>(entity);
        auto& animator = scene.Get<AnimatorComponent>(entity);
        const auto& skeleton = mesh.skeleton;
        const int count = static_cast<int>(skeleton.bones.size());
        if (count == 0 || static_cast<int>(animator.pose.size()) != count) return;

        auto physicalPosition = [&](int bone, bool* okOut = nullptr) {
            bool ok = false;
            const glm::vec3 position = Physics::GetRagdollBonePosition(rag, bone, &ok);
            if (okOut) *okOut = ok;
            return position;
        };

        bool leftPositionOk = false, rightPositionOk = false;
        const glm::vec3 leftFoot = physicalPosition(comp._legL.footIdx, &leftPositionOk);
        const glm::vec3 rightFoot = physicalPosition(comp._legR.footIdx, &rightPositionOk);
        if (!leftPositionOk || !rightPositionOk) return;

        comp._test3ContactL = FootContact(rag, comp._legL.footIdx);
        comp._test3ContactR = FootContact(rag, comp._legR.footIdx);
        if (!ready) {
            comp._test3Time = 0.0f;
            comp._test3SettleTime = 0.0f;
            return;
        }
        comp._test3Time += dt;

        const glm::vec3 leftVelocity = Physics::GetRagdollBoneLinearVelocity(
            rag, comp._legL.footIdx);
        const glm::vec3 rightVelocity = Physics::GetRagdollBoneLinearVelocity(
            rag, comp._legR.footIdx);
        const float horizontalSpeed = glm::length(glm::vec2(
            rag._locomotionRootVel.x, rag._locomotionRootVel.z));
        const bool settled = comp._test3ContactL && comp._test3ContactR
            && glm::length(leftVelocity) < 0.15f
            && glm::length(rightVelocity) < 0.15f
            && horizontalSpeed < 0.15f
            && tiltDeg < 15.0f;

        if (!comp._test3BaselineValid) {
            comp._test3SettleTime = settled ? comp._test3SettleTime + dt : 0.0f;
            if (comp._test3SettleTime >= 1.0f) {
                glm::vec3 right = rightFoot - leftFoot;
                right.y = 0.0f;
                if (glm::dot(right, right) < 1e-8f) right = comp._right;
                if (glm::dot(right, right) < 1e-8f) right = glm::vec3(1, 0, 0);

                comp._test3BaselineValid = true;
                comp._test3FootBaselineL = leftFoot;
                comp._test3FootBaselineR = rightFoot;
                comp._test3ComBaseline = rag._locomotionCOM;
                comp._test3Right = glm::normalize(right);
                comp._test3SupportTarget = comp._test3ComBaseline;
                comp._test3PeakTilt = tiltDeg;
                spdlog::info("[LocoTest3] baseline acquired at t={:.2f} span={:.3f}m tilt={:.1f}",
                             comp._test3Time,
                             glm::dot(comp._test3FootBaselineR
                                      - comp._test3FootBaselineL,
                                      comp._test3Right), tiltDeg);
            }
        }
        if (!comp._test3BaselineValid) return;

        if (comp._test3Phase == 0) {
            const bool startLeftSupport = Input::IsKeyPressed(Key::F6);
            const bool startRightSupport = Input::IsKeyPressed(Key::F7);
            if (startLeftSupport || startRightSupport) {
                // Every isolated run starts from the feet that are actually under the
                // character now. Reusing the first-ever baseline made F7 inherit the
                // harmless landing offset from a preceding F6 run and abort before takeoff.
                glm::vec3 currentRight = rightFoot - leftFoot;
                currentRight.y = 0.0f;
                if (glm::dot(currentRight, currentRight) < 1e-8f)
                    currentRight = comp._test3Right;
                comp._test3FootBaselineL = leftFoot;
                comp._test3FootBaselineR = rightFoot;
                comp._test3ComBaseline = rag._locomotionCOM;
                comp._test3Right = glm::normalize(currentRight);
                comp._test3SupportTarget = comp._test3ComBaseline;
                comp._test3SupportSide = startLeftSupport ? -1 : 1;
                comp._test3Phase = 1;
                comp._test3PhaseTime = 0.0f;
                comp._test3PeakTilt = tiltDeg;
                comp._test3MaxStanceDrift = 0.0f;
                comp._test3LiftStartBlend = 0.0f;
                comp._test3AirborneTime = 0.0f;
                comp._test3Aborted = false;
                spdlog::info("[LocoTest3] sequence start support={} swing={}",
                             startLeftSupport ? "LEFT" : "RIGHT",
                             startLeftSupport ? "RIGHT" : "LEFT");
            }
        }

        if (comp._test3Phase > 0) comp._test3PhaseTime += dt;
        const float desiredComCommand = comp._test3Phase >= 1 && comp._test3Phase <= 6
            ? static_cast<float>(comp._test3SupportSide) : 0.0f;
        comp._test3ComCommand = Approach(
            comp._test3ComCommand, desiredComCommand,
            dt / glm::max(comp.test2ShiftTime, 0.01f));

        // Test 2 deliberately stopped short of either foot. Single support cannot: place
        // the COM projection nearly over the actual stance sole before unloading the other
        // leg, or the nominal swing foot still carries enough normal force to stay sticky.
        const float fraction = glm::clamp(comp.test3SupportFraction, 0.0f, 1.0f);
        const float leftAvailable = glm::dot(
            comp._test3FootBaselineL - comp._test3ComBaseline, comp._test3Right);
        const float rightAvailable = glm::dot(
            comp._test3FootBaselineR - comp._test3ComBaseline, comp._test3Right);
        const float lateralOffset = comp._test3ComCommand < 0.0f
            ? -comp._test3ComCommand * glm::min(leftAvailable, 0.0f) * fraction
            :  comp._test3ComCommand * glm::max(rightAvailable, 0.0f) * fraction;
        const glm::vec3 supportTarget = comp._test3ComBaseline
                                     + comp._test3Right * lateralOffset;
        glm::vec3 supportVelocity = dt > 1e-6f
            ? (supportTarget - comp._test3SupportTarget) / dt : glm::vec3(0.0f);
        supportVelocity.y = 0.0f;
        comp._test3SupportTarget = supportTarget;

        rag.locomotionSupportTarget = supportTarget;
        rag.locomotionSupportTargetVel = supportVelocity;
        rag.locomotionSupportTargetWeight = 1.0f;
        rag.locomotionCOMSupportFreq = glm::max(comp.test2SupportFrequency, 0.0f);
        rag.locomotionCOMSupportMaxAccel = glm::max(comp.test2SupportMaxAccel, 0.0f);

        comp._test3ComLateral = glm::dot(
            rag._locomotionCOM - comp._test3ComBaseline, comp._test3Right);
        comp._test3TargetLateral = lateralOffset;
        comp._test3PeakTilt = glm::max(comp._test3PeakTilt, tiltDeg);

        Leg* swing = comp._test3SupportSide < 0 ? &comp._legR : &comp._legL;
        Leg* stance = comp._test3SupportSide < 0 ? &comp._legL : &comp._legR;
        const glm::vec3 swingFoot = physicalPosition(swing->footIdx);
        const glm::vec3 stanceFoot = physicalPosition(stance->footIdx);
        const glm::vec3 stanceBaseline = comp._test3SupportSide < 0
            ? comp._test3FootBaselineL : comp._test3FootBaselineR;
        const float stanceDrift = glm::length(glm::vec2(
            stanceFoot.x - stanceBaseline.x, stanceFoot.z - stanceBaseline.z));
        comp._test3MaxStanceDrift = glm::max(comp._test3MaxStanceDrift, stanceDrift);

        auto captureSwing = [&]() {
            const glm::vec3 hip = physicalPosition(swing->hipIdx);
            const glm::vec3 knee = physicalPosition(swing->kneeIdx);
            const glm::vec3 ankle = physicalPosition(swing->ankleIdx);
            comp._test3SwingStart = swingFoot;
            swing->desiredFoot = swingFoot;
            swing->ankleFromFootWorld = ankle - swingFoot;
            const glm::vec3 upper = knee - hip;
            const glm::vec3 lower = ankle - knee;
            if (glm::dot(upper, upper) > 1e-8f && glm::dot(lower, lower) > 1e-8f) {
                swing->referenceUpperWorld = glm::normalize(upper);
                swing->referenceKneeBend = std::acos(glm::clamp(
                    glm::dot(glm::normalize(upper), glm::normalize(lower)),
                    -1.0f, 1.0f));
            }
            const glm::vec3 chain = ankle - hip;
            if (glm::dot(chain, chain) > 1e-8f) {
                const glm::vec3 axis = glm::normalize(chain);
                glm::vec3 pole = upper - axis * glm::dot(upper, axis);
                if (glm::dot(pole, pole) > 1e-8f)
                    swing->kneePoleWorld = glm::normalize(pole);
            }

            bool hipRotationOk = false, kneeRotationOk = false;
            bool ankleRotationOk = false, footRotationOk = false;
            swing->referenceHipWorld = Physics::GetRagdollBoneRotation(
                rag, swing->hipIdx, &hipRotationOk);
            const glm::quat kneeWorld = Physics::GetRagdollBoneRotation(
                rag, swing->kneeIdx, &kneeRotationOk);
            const glm::quat ankleWorld = Physics::GetRagdollBoneRotation(
                rag, swing->ankleIdx, &ankleRotationOk);
            const glm::quat footWorld = Physics::GetRagdollBoneRotation(
                rag, swing->footIdx, &footRotationOk);
            if (hipRotationOk && kneeRotationOk)
                swing->referenceKneeLocal = glm::normalize(
                    glm::conjugate(swing->referenceHipWorld) * kneeWorld);
            else
                swing->referenceKneeLocal = skeleton.bones[swing->kneeIdx].localR;
            if (kneeRotationOk && ankleRotationOk)
                swing->referenceAnkleLocal = glm::normalize(
                    glm::conjugate(kneeWorld) * ankleWorld);
            else
                swing->referenceAnkleLocal = skeleton.bones[swing->ankleIdx].localR;
            if (ankleRotationOk && footRotationOk)
                swing->referenceFootLocal = glm::normalize(
                    glm::conjugate(ankleWorld) * footWorld);
            else
                swing->referenceFootLocal = skeleton.bones[swing->footIdx].localR;
            if (!hipRotationOk)
                swing->referenceHipWorld = glm::normalize(
                    ParentWorldRot(rag, skeleton, animator,
                                   scene.GetTransformSystem().GetWorldMatrix(entity),
                                   swing->hipIdx)
                    * animator.pose[swing->hipIdx].rotation);

            swing->hipCommand = animator.pose[swing->hipIdx].rotation;
            swing->kneeCommand = swing->referenceKneeLocal;
            swing->ankleCommand = swing->referenceAnkleLocal;
            swing->footCommand = swing->referenceFootLocal;
            swing->commandValid = true;
        };

        const float comError = comp._test3TargetLateral - comp._test3ComLateral;
        if (comp._test3Phase == 1
            && std::abs(comp._test3ComCommand - comp._test3SupportSide) < 0.01f
            && std::abs(comError) < 0.01f
            && comp._test3ContactL && comp._test3ContactR
            && comp._test3PhaseTime >= glm::max(comp.test2ShiftTime, 0.01f)) {
            captureSwing();
            comp._test3Phase = 2;
            comp._test3PhaseTime = 0.0f;
            spdlog::info("[LocoTest3] phase=TAKEOFF foot=({:+.3f},{:+.3f},{:+.3f})",
                         comp._test3SwingStart.x, comp._test3SwingStart.y,
                         comp._test3SwingStart.z);
        }

        auto smoothstep = [](float t) {
            t = glm::clamp(t, 0.0f, 1.0f);
            return t * t * (3.0f - 2.0f * t);
        };
        const float liftHeight = glm::max(comp.test3LiftHeight, 0.001f);
        const float takeoffHeight = glm::clamp(comp.test3TakeoffHeight,
                                                0.005f, liftHeight);
        const float takeoffBlend = takeoffHeight / liftHeight;
        const float currentClearance = swingFoot.y - comp._test3SwingStart.y;
        const bool swingContactNow = FootContact(rag, swing->footIdx);
        auto abortSequence = [&](const char* reason) {
            comp._test3Aborted = true;
            comp._test3Phase = 7;
            comp._test3PhaseTime = 0.0f;
            comp._test3LiftBlend = 0.0f;
            comp._test3AirborneTime = 0.0f;
            spdlog::warn("[LocoTest3] ABORT {} clear={:.3f} contact={} stanceDrift={:.3f} tilt={:.1f}; recentering",
                         reason, currentClearance, swingContactNow ? "yes" : "no",
                         stanceDrift, tiltDeg);
        };
        if (comp._test3Phase == 2) {
            // A powered ragdoll motor only controls joint angles. With both high-friction
            // soles in contact, folding those joints does not determine which end of the
            // chain moves. Briefly unload the selected foot with an internal toe-off force;
            // only after the contact manifold has actually disappeared for several frames
            // does IK become the sole swing controller. Foot-center motion by one sole half-
            // thickness is not clearance: the previous test released at 2.5 cm while the
            // collision was still active, immediately recreating the closed chain.
            // TAKEOFF is a contact-release phase, not the visible lift trajectory.
            // Command its full clearance immediately so the one-sided spring has time
            // to open the manifold and hold the foot airborne for the required 50 ms.
            // Easing this target over the same interval used as the timeout made the
            // foot reach release clearance only as TAKEOFF expired. Once released,
            // phase 3 continues smoothly from takeoffBlend to the authored lift height.
            comp._test3LiftBlend = takeoffBlend;
            const float releaseClearance = glm::max(0.040f, takeoffHeight * 0.75f);
            const bool airborneEvidence = !swingContactNow
                && currentClearance >= releaseClearance;
            comp._test3AirborneTime = airborneEvidence
                ? comp._test3AirborneTime + dt : 0.0f;
            if (comp._test3AirborneTime >= 0.05f) {
                comp._test3LiftStartBlend = glm::clamp(
                    glm::max(comp._test3LiftBlend, currentClearance / liftHeight),
                    0.0f, 1.0f);
                comp._test3Phase = 3;
                comp._test3PhaseTime = 0.0f;
                spdlog::info("[LocoTest3] phase=LIFT released clearance={:.3f} airborneFor={:.3f}",
                             currentClearance, comp._test3AirborneTime);
            } else if (comp._test3PhaseTime >= glm::max(comp.test3TakeoffTime, 0.01f)) {
                abortSequence("takeoff did not open swing contact");
            }
        } else if (comp._test3Phase == 3) {
            comp._test3LiftBlend = glm::mix(
                comp._test3LiftStartBlend, 1.0f,
                smoothstep(comp._test3PhaseTime / glm::max(comp.test3LiftTime, 0.01f)));
            if (comp._test3PhaseTime >= glm::max(comp.test3LiftTime, 0.01f)) {
                comp._test3Phase = 4;
                comp._test3PhaseTime = 0.0f;
                comp._test3LiftBlend = 1.0f;
                spdlog::info("[LocoTest3] phase=HOLD clearance={:.3f}",
                             currentClearance);
            }
        } else if (comp._test3Phase == 4) {
            comp._test3LiftBlend = 1.0f;
            if (comp._test3PhaseTime >= glm::max(comp.test3HoldTime, 0.01f)) {
                comp._test3Phase = 5;
                comp._test3PhaseTime = 0.0f;
                spdlog::info("[LocoTest3] phase=LOWER");
            }
        } else if (comp._test3Phase == 5) {
            comp._test3LiftBlend = 1.0f - smoothstep(
                comp._test3PhaseTime / glm::max(comp.test3LowerTime, 0.01f));
            if (comp._test3PhaseTime >= glm::max(comp.test3LowerTime, 0.01f)) {
                comp._test3Phase = 6;
                comp._test3PhaseTime = 0.0f;
                comp._test3SettleTime = 0.0f;
                comp._test3LiftBlend = 0.0f;
                spdlog::info("[LocoTest3] phase=CONTACT");
            }
        } else if (comp._test3Phase == 6) {
            comp._test3LiftBlend = 0.0f;
            const bool swingContact = FootContact(rag, swing->footIdx);
            const float swingSpeed = glm::length(
                Physics::GetRagdollBoneLinearVelocity(rag, swing->footIdx));
            const bool contactSettled = swingContact && swingSpeed < 0.15f
                && std::abs(swingFoot.y - comp._test3SwingStart.y) < 0.03f;
            comp._test3SettleTime = contactSettled
                ? comp._test3SettleTime + dt : 0.0f;
            if (comp._test3SettleTime >= glm::max(comp.test3ContactSettleTime, 0.01f)) {
                comp._test3Phase = 7;
                comp._test3PhaseTime = 0.0f;
                spdlog::info("[LocoTest3] phase=RECENTER");
            }
        } else if (comp._test3Phase == 7
                   && std::abs(comp._test3ComCommand) < 0.01f
                   && std::abs(comp._test3ComLateral) < 0.01f
                   && comp._test3PhaseTime >= glm::max(comp.test2ShiftTime, 0.01f)) {
            if (comp._test3Aborted) {
                spdlog::warn("[LocoTest3] reset after aborted sequence support={} maxDrift={:.3f} peakTilt={:.1f}",
                             comp._test3SupportSide < 0 ? "LEFT" : "RIGHT",
                             comp._test3MaxStanceDrift, comp._test3PeakTilt);
            } else {
                spdlog::info("[LocoTest3] COMPLETE support={} maxDrift={:.3f} peakTilt={:.1f} poleDot={:+.2f}",
                             comp._test3SupportSide < 0 ? "LEFT" : "RIGHT",
                             comp._test3MaxStanceDrift, comp._test3PeakTilt,
                             comp._test3KneePoleDot);
            }
            comp._test3Phase = 0;
            comp._test3PhaseTime = 0.0f;
            comp._test3SupportSide = 0;
        }

        // Test 3 is a diagnostic, not a fall demonstration. Once airborne, any renewed
        // swing contact means the closed chain has returned. Likewise, a sliding stance
        // sole or excessive tilt has already failed the test, so stop commanding the swing
        // pose immediately while the validated standing controller can still recover.
        const bool airbornePhase = comp._test3Phase == 3 || comp._test3Phase == 4;
        if (airbornePhase && comp._test3PhaseTime >= 0.10f
            && (swingContactNow || currentClearance < 0.030f)) {
            abortSequence("airborne swing lost clearance");
        } else if (comp._test3Phase >= 2 && comp._test3Phase <= 5
                   && stanceDrift > 0.040f) {
            abortSequence("stance foot exceeded 4 cm drift");
        } else if (comp._test3Phase >= 2 && comp._test3Phase <= 5
                   && tiltDeg >= 30.0f) {
            abortSequence("tilt reached 30 degrees");
        }

        const bool applySwingIK = comp._test3Phase >= 2 && comp._test3Phase <= 6;
        // Keep the same one-sided lift spring through takeoff, lift, and hold. The
        // previous phase-2-only command disappeared on the exact frame that contact
        // opened, so the foot fell straight back into the manifold before IK could take
        // up the load. During LOWER the cap fades with the trajectory; the target itself
        // remains continuous and the spring naturally supplies zero force at/above it.
        const bool liftAssistActive = comp._test3Phase >= 2 && comp._test3Phase <= 5;
        if (liftAssistActive) {
            const float lowerFade = comp._test3Phase == 5
                ? glm::clamp(comp._test3LiftBlend, 0.0f, 1.0f) : 1.0f;
            rag.locomotionLiftBone = swing->footIdx;
            rag.locomotionLiftTargetY = comp._test3SwingStart.y
                                      + liftHeight * comp._test3LiftBlend;
            rag.locomotionLiftFrequency = glm::max(comp.test3TakeoffFrequency, 0.0f);
            rag.locomotionLiftMaxForce = glm::max(comp.test3TakeoffMaxForce, 0.0f)
                                       * lowerFade;
        } else {
            rag.locomotionLiftBone = -1;
            rag.locomotionLiftMaxForce = 0.0f;
        }
        if (applySwingIK) {
            const glm::vec3 desiredFoot = comp._test3SwingStart
                + glm::vec3(0.0f, liftHeight * comp._test3LiftBlend, 0.0f);
            swing->desiredFoot = desiredFoot;
            const glm::vec3 hipPosition = physicalPosition(swing->hipIdx);
            glm::vec3 desiredAnkle = desiredFoot + swing->ankleFromFootWorld;
            glm::vec3 toTarget = desiredAnkle - hipPosition;
            const float upperLength = glm::length(skeleton.bones[swing->kneeIdx].localT);
            const float lowerLength = glm::length(skeleton.bones[swing->ankleIdx].localT);
            const float maxReach = (upperLength + lowerLength)
                * glm::clamp(comp.proceduralMaxReach, 0.70f, 0.99f);
            if (const float reach = glm::length(toTarget);
                reach > maxReach && reach > 1e-5f) {
                toTarget *= maxReach / reach;
                desiredAnkle = hipPosition + toTarget;
            }
            const float distance = glm::clamp(glm::length(toTarget),
                std::abs(upperLength - lowerLength) + 1e-4f,
                upperLength + lowerLength - 1e-4f);
            if (upperLength > 1e-4f && lowerLength > 1e-4f && distance > 1e-4f) {
                const float includedCos = glm::clamp(
                    (upperLength * upperLength + lowerLength * lowerLength
                     - distance * distance) / (2.0f * upperLength * lowerLength),
                    -1.0f, 1.0f);
                const float kneeBend = glm::pi<float>() - std::acos(includedCos);
                const float kneeDelta = kneeBend - swing->referenceKneeBend;
                const glm::quat kneeTarget = glm::normalize(
                    swing->referenceKneeLocal
                    * glm::angleAxis(kneeDelta, glm::normalize(swing->kneeHingeAxis)));

                const glm::vec3 worldForward = glm::normalize(toTarget);
                glm::vec3 worldBend = swing->kneePoleWorld
                    - worldForward * glm::dot(swing->kneePoleWorld, worldForward);
                if (glm::dot(worldBend, worldBend) < 1e-8f)
                    worldBend = comp._fwd;
                worldBend -= worldForward * glm::dot(worldBend, worldForward);
                if (glm::dot(worldBend, worldBend) < 1e-8f)
                    worldBend = comp._test3Right;
                else
                    worldBend = glm::normalize(worldBend);
                const float hipCos = glm::clamp(
                    (upperLength * upperLength + distance * distance
                     - lowerLength * lowerLength)
                    / (2.0f * upperLength * distance), -1.0f, 1.0f);
                const float hipSin = std::sqrt(glm::max(1.0f - hipCos * hipCos, 0.0f));
                const glm::vec3 desiredUpper = glm::normalize(
                    worldForward * hipCos + worldBend * hipSin);
                const glm::quat hipWorld = glm::normalize(
                    RotationBetween(swing->referenceUpperWorld, desiredUpper)
                    * swing->referenceHipWorld);
                const glm::mat4 entityWorld = scene.GetTransformSystem().GetWorldMatrix(entity);
                const glm::quat parentWorld = ParentWorldRot(
                    rag, skeleton, animator, entityWorld, swing->hipIdx);
                glm::quat hipTarget = glm::normalize(glm::conjugate(parentWorld) * hipWorld);
                Envelope hipEnvelope;
                hipEnvelope.twistAxis = swing->hipTwistAxis;
                hipEnvelope.swingNormalDeg = swing->hipSwingNormalDeg;
                hipEnvelope.swingPlaneDeg = swing->hipSwingPlaneDeg;
                hipEnvelope.twistMinDeg = swing->hipTwistMinDeg;
                hipEnvelope.twistMaxDeg = swing->hipTwistMaxDeg;
                hipTarget = ClampToEnvelope(hipEnvelope,
                    skeleton.bones[swing->hipIdx].localR, hipTarget,
                    comp.hipLimitMarginDeg);

                const float alpha = 1.0f - std::exp(
                    -dt / glm::max(comp.proceduralPoseResponse, 0.01f));
                swing->hipCommand = glm::normalize(
                    glm::slerp(swing->hipCommand, hipTarget, alpha));
                swing->kneeCommand = glm::normalize(
                    glm::slerp(swing->kneeCommand, kneeTarget, alpha));
                swing->ankleCommand = swing->referenceAnkleLocal;
                swing->footCommand = swing->referenceFootLocal;
                const float poseWeight = glm::clamp(
                    comp._poseBlend * comp.poseWeight, 0.0f, 1.0f);
                BlendPose(animator, swing->hipIdx, swing->hipCommand, poseWeight);
                BlendPose(animator, swing->kneeIdx, swing->kneeCommand, poseWeight);
                BlendPose(animator, swing->ankleIdx, swing->ankleCommand, poseWeight);
                BlendPose(animator, swing->footIdx, swing->footCommand, poseWeight);
            }

            comp._test3Clearance = swingFoot.y - comp._test3SwingStart.y;
            comp._test3TargetError = glm::length(swingFoot - desiredFoot);
            const glm::vec3 hip = physicalPosition(swing->hipIdx);
            const glm::vec3 knee = physicalPosition(swing->kneeIdx);
            const glm::vec3 ankle = physicalPosition(swing->ankleIdx);
            const glm::vec3 chain = ankle - hip;
            if (glm::dot(chain, chain) > 1e-8f) {
                const glm::vec3 axis = glm::normalize(chain);
                glm::vec3 pole = (knee - hip) - axis * glm::dot(knee - hip, axis);
                if (glm::dot(pole, pole) > 1e-8f)
                    comp._test3KneePoleDot = glm::dot(
                        glm::normalize(pole), swing->kneePoleWorld);
            }
            if (comp.debug) {
                DebugDraw::Sphere(desiredFoot, 0.035f, {1.0f, 0.4f, 0.1f});
                DebugDraw::Line(swingFoot, desiredFoot, {1.0f, 0.4f, 0.1f});
            }
        } else {
            comp._test3LiftBlend = 0.0f;
            comp._test3Clearance = 0.0f;
            comp._test3TargetError = 0.0f;
        }

        if (comp.debug) {
            DebugDraw::Sphere(comp._test3ComBaseline, 0.025f, {0.2f, 0.7f, 1.0f});
            DebugDraw::Sphere(supportTarget, 0.035f, {1.0f, 0.7f, 0.1f});
            DebugDraw::Line(comp._test3ComBaseline, supportTarget, {1.0f, 0.7f, 0.1f});
        }
    }

    static const char* GroundTestStageName(int stage)
    {
        switch (stage) {
            case 0: return "fixed soles / ankle motors on / COM on";
            case 1: return "fixed soles / ankle motors off / COM on";
            case 2: return "fixed soles / ankle motors on / COM off";
            case 3: return "fixed soles / ankle motors off / COM off";
            default: return "unknown";
        }
    }

    static void UpdateGroundTest(Scene& scene, entt::entity entity, Comp& comp,
                                 RagdollComponent& rag, bool ready, float tiltDeg, float dt)
    {
        if (!scene.Has<SkinnedMeshComponent>(entity)
            || !scene.Has<AnimatorComponent>(entity)
            || !ValidLeg(comp._legL) || !ValidLeg(comp._legR)
            || !rag._locomotionCOMValid) return;

        auto& animator = scene.Get<AnimatorComponent>(entity);
        const int bones[8] = {
            comp._legL.hipIdx, comp._legL.kneeIdx, comp._legL.ankleIdx, comp._legL.footIdx,
            comp._legR.hipIdx, comp._legR.kneeIdx, comp._legR.ankleIdx, comp._legR.footIdx
        };
        for (int bone : bones)
            if (bone < 0 || bone >= static_cast<int>(animator.pose.size())) return;

        const int footBones[2] = { comp._legL.footIdx, comp._legR.footIdx };
        glm::vec3 footPosition[2];
        glm::quat footRotation[2];
        glm::vec3 footVelocity[2];
        glm::vec3 footAngularVelocity[2];
        bool valid = true;
        for (int i = 0; i < 2; ++i) {
            bool positionOk = false, rotationOk = false;
            bool velocityOk = false, angularOk = false;
            footPosition[i] = Physics::GetRagdollBonePosition(rag, footBones[i], &positionOk);
            footRotation[i] = Physics::GetRagdollBoneRotation(rag, footBones[i], &rotationOk);
            footVelocity[i] = Physics::GetRagdollBoneLinearVelocity(rag, footBones[i], &velocityOk);
            footAngularVelocity[i] = Physics::GetRagdollBoneAngularVelocity(
                rag, footBones[i], &angularOk);
            valid = valid && positionOk && rotationOk && velocityOk && angularOk;
            comp._groundTestContact[i] = FootContact(rag, footBones[i]);
        }
        if (!valid) return;

        if (!ready) {
            comp._groundTestTime = 0.0f;
            comp._groundTestSettleTime = 0.0f;
            return;
        }
        comp._groundTestTime += dt;

        const bool settled = comp._groundTestContact[0] && comp._groundTestContact[1]
            && glm::length(footVelocity[0]) < 0.15f
            && glm::length(footVelocity[1]) < 0.15f
            && glm::length(footAngularVelocity[0]) < glm::radians(20.0f)
            && glm::length(footAngularVelocity[1]) < glm::radians(20.0f)
            && tiltDeg < 15.0f;

        auto beginStage = [&](int stage) {
            comp._groundTestStage = glm::clamp(stage, 0, 3);
            comp._groundTestStageTime = 0.0f;
            for (int i = 0; i < 2; ++i) {
                comp._groundTestFootStart[i] = footPosition[i];
                comp._groundTestFootRotStart[i] = footRotation[i];
                comp._groundTestPrevContact[i] = comp._groundTestContact[i];
                comp._groundTestContactTransitions[i] = 0;
                comp._groundTestFootSpeed[i] = 0.0f;
                comp._groundTestFootAngularSpeed[i] = 0.0f;
                comp._groundTestFootDisplacement[i] = 0.0f;
                comp._groundTestFootRotation[i] = 0.0f;
                comp._groundTestMaxSpeed[i] = 0.0f;
                comp._groundTestMaxAngularSpeed[i] = 0.0f;
                comp._groundTestMaxDisplacement[i] = 0.0f;
                comp._groundTestMaxRotation[i] = 0.0f;
            }
            spdlog::info("[LocoGround] stage={} ({}) started contact=({},{}) tilt={:.1f}",
                         comp._groundTestStage,
                         GroundTestStageName(comp._groundTestStage),
                         comp._groundTestContact[0] ? "L" : "-",
                         comp._groundTestContact[1] ? "R" : "-", tiltDeg);
        };

        if (!comp._groundTestBaselineValid) {
            comp._groundTestSettleTime = settled ? comp._groundTestSettleTime + dt : 0.0f;
            if (comp._groundTestSettleTime < 1.0f) return;
            comp._groundTestBaselineValid = true;
            comp._groundTestComTarget = rag._locomotionCOM;
            for (int i = 0; i < 8; ++i)
                comp._groundTestPose[i] = glm::normalize(animator.pose[bones[i]].rotation);
            beginStage(0);
            spdlog::info("[LocoGround] baseline acquired at t={:.2f}; pose is now frozen",
                         comp._groundTestTime);
        }

        // Make motor targets invariant. Animation and the normal standing solver may run
        // earlier in this frame, but these final local rotations are what the physics motor
        // pass reads, eliminating a moving target as a variable.
        for (int i = 0; i < 8; ++i)
            animator.pose[bones[i]].rotation = comp._groundTestPose[i];

        if (Input::IsKeyPressed(Key::F8)) {
            spdlog::info("[LocoGround] recapture requested");
            ResetGroundTest(comp);
            rag.locomotionSupportTargetWeight = 0.0f;
            return;
        }
        if (Input::IsKeyPressed(Key::F6) && comp._groundTestStage < 3)
            beginStage(comp._groundTestStage + 1);
        else if (Input::IsKeyPressed(Key::F7) && comp._groundTestStage > 0)
            beginStage(comp._groundTestStage - 1);

        // Terminal soles are fixed joints and have no motor. Exercise the two remaining
        // possible feedback sources as a 2x2 comparison: ankle motors on/off and explicit
        // COM support on/off. The controller clears this array before every update.
        const bool anklesOff = comp._groundTestStage == 1 || comp._groundTestStage == 3;
        if (anklesOff) {
            rag.locomotionDisabledMotorBones[0] = comp._legL.ankleIdx;
            rag.locomotionDisabledMotorBones[1] = comp._legR.ankleIdx;
        }

        const bool comOn = comp._groundTestStage < 2;
        if (comOn) {
            rag.locomotionSupportTarget = comp._groundTestComTarget;
            rag.locomotionSupportTargetVel = glm::vec3(0.0f);
            rag.locomotionSupportTargetWeight = 1.0f;
            rag.locomotionCOMSupportFreq = glm::max(comp.test2SupportFrequency, 0.0f);
            rag.locomotionCOMSupportMaxAccel = glm::max(comp.test2SupportMaxAccel, 0.0f);
        } else {
            rag.locomotionSupportTargetWeight = 0.0f;
        }

        comp._groundTestStageTime += dt;
        auto rotationDifferenceDeg = [](glm::quat a, glm::quat b) {
            const glm::quat q = glm::normalize(glm::conjugate(glm::normalize(a))
                                              * glm::normalize(b));
            return glm::degrees(2.0f * std::acos(
                glm::clamp(std::abs(q.w), 0.0f, 1.0f)));
        };
        for (int i = 0; i < 2; ++i) {
            comp._groundTestFootSpeed[i] = glm::length(footVelocity[i]);
            comp._groundTestFootAngularSpeed[i] = glm::degrees(
                glm::length(footAngularVelocity[i]));
            comp._groundTestFootDisplacement[i] = glm::length(
                footPosition[i] - comp._groundTestFootStart[i]);
            comp._groundTestFootRotation[i] = rotationDifferenceDeg(
                comp._groundTestFootRotStart[i], footRotation[i]);
            comp._groundTestMaxSpeed[i] = glm::max(
                comp._groundTestMaxSpeed[i], comp._groundTestFootSpeed[i]);
            comp._groundTestMaxAngularSpeed[i] = glm::max(
                comp._groundTestMaxAngularSpeed[i], comp._groundTestFootAngularSpeed[i]);
            comp._groundTestMaxDisplacement[i] = glm::max(
                comp._groundTestMaxDisplacement[i], comp._groundTestFootDisplacement[i]);
            comp._groundTestMaxRotation[i] = glm::max(
                comp._groundTestMaxRotation[i], comp._groundTestFootRotation[i]);
            if (comp._groundTestContact[i] != comp._groundTestPrevContact[i]) {
                ++comp._groundTestContactTransitions[i];
                comp._groundTestPrevContact[i] = comp._groundTestContact[i];
            }
        }
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

    void UpdateProceduralGait(Scene& scene, entt::entity entity, Comp& comp,
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

        auto smoothstep = [](float t) {
            t = glm::clamp(t, 0.0f, 1.0f);
            return t * t * (3.0f - 2.0f * t);
        };
        auto physicalPosition = [&](int bone) {
            bool ok = false;
            const glm::vec3 p = Physics::GetRagdollBonePosition(rag, bone, &ok);
            return ok ? p : BoneWorldPos(skeleton, animator, entityWorld, bone);
        };
        auto initializeLeg = [&](Leg& leg) {
            if (leg.planted) return;
            const glm::vec3 foot = physicalPosition(leg.footIdx);
            const HitResult ground = Physics::Raycast(
                foot + glm::vec3(0, 0.25f, 0), glm::vec3(0, -1, 0),
                0.65f, entity, false);
            if (ground.hit)
                leg.groundOffset = glm::clamp(foot.y - ground.point.y, 0.015f, 0.20f);
            leg.plantFoot = foot;
            leg.swingStartFoot = foot;
            leg.swingTargetFoot = foot;
            leg.desiredFoot = foot;
            const glm::vec3 hip = physicalPosition(leg.hipIdx);
            const glm::vec3 knee = physicalPosition(leg.kneeIdx);
            const glm::vec3 ankle = physicalPosition(leg.ankleIdx);
            leg.ankleFromFootWorld = ankle - foot;
            const glm::vec3 upper = knee - hip;
            const glm::vec3 lower = ankle - knee;
            if (glm::dot(upper, upper) > 1e-8f && glm::dot(lower, lower) > 1e-8f) {
                leg.referenceKneeBend = std::acos(glm::clamp(
                    glm::dot(glm::normalize(upper), glm::normalize(lower)),
                    -1.0f, 1.0f));
            }
            const glm::vec3 chain = ankle - hip;
            if (glm::dot(chain, chain) > 1e-8f) {
                const glm::vec3 axis = glm::normalize(chain);
                glm::vec3 pole = upper - axis * glm::dot(upper, axis);
                if (glm::dot(pole, pole) > 1e-8f)
                    leg.kneePoleWorld = glm::normalize(pole);
                else
                    leg.kneePoleWorld = fwd;
            }
            bool rotationOk = false;
            leg.plantedFootWorldRotation = Physics::GetRagdollBoneRotation(
                rag, leg.footIdx, &rotationOk);
            if (!rotationOk)
                leg.plantedFootWorldRotation = OrientationOf(
                    BoneWorldMatrix(skeleton, animator, entityWorld, leg.footIdx));
            leg.planted = true;
        };

        // Acquire plant anchors only from a settled double contact. The previous version
        // captured them on the first powered frame while the rig was still dropping roughly
        // 2.3 m/s, then asked two 1800 N springs to drag the feet back to those stale points.
        // That produced the reported jitter before the procedural IK was even enabled.
        const glm::vec3 leftAcquireVelocity = Physics::GetRagdollBoneLinearVelocity(
            rag, comp._legL.footIdx);
        const glm::vec3 rightAcquireVelocity = Physics::GetRagdollBoneLinearVelocity(
            rag, comp._legR.footIdx);
        const float tiltDeg = Physics::GetRagdollTiltDeg(rag);
        const bool plantsReady = comp._legL.planted && comp._legR.planted;
        const bool settledDoubleContact = FootContact(rag, comp._legL.footIdx)
            && FootContact(rag, comp._legR.footIdx)
            && glm::length(leftAcquireVelocity) <= 0.35f
            && glm::length(rightAcquireVelocity) <= 0.35f
            && tiltDeg <= 15.0f;
        if (!walking) {
            comp._plantAcquireTime = 0.0f;
        } else if (!plantsReady) {
            comp._plantAcquireTime = settledDoubleContact
                ? comp._plantAcquireTime + dt : 0.0f;
            if (comp._plantAcquireTime >= 0.20f) {
                initializeLeg(comp._legL);
                initializeLeg(comp._legR);
            }
        }

        Leg* swing = comp._swingIsLeft ? &comp._legL : &comp._legR;
        Leg* stance = comp._swingIsLeft ? &comp._legR : &comp._legL;
        glm::vec3 swingFoot = physicalPosition(swing->footIdx);
        glm::vec3 stanceFoot = physicalPosition(stance->footIdx);
        const glm::vec3 rootPos = physicalPosition(rootBone);
        const glm::vec3 com = rag._locomotionCOMValid ? rag._locomotionCOM : rootPos;
        const glm::vec3 comVel = rag._locomotionCOMValid ? rag._locomotionCOMVel
                                                         : rag._locomotionRootVel;
        glm::vec3 contactNormal(0.0f);
        bool contact = FootContact(rag, swing->footIdx, &contactNormal);
        bool velocityOk = false;
        glm::vec3 swingVelocity = Physics::GetRagdollBoneLinearVelocity(
            rag, swing->footIdx, &velocityOk);
        if (!velocityOk) swingVelocity = glm::vec3(0.0f);
        float swingClearance = swingFoot.y - stanceFoot.y;
        const bool canStep = walking && comp._legL.planted && comp._legR.planted
                          && tiltDeg < rag.locomotionFallenTilt;

        comp._right = right;
        comp._fwd = fwd;
        comp._dSag = glm::dot(com - stanceFoot, fwd);
        comp._dLat = glm::dot(com - stanceFoot, right);
        comp._vSag = glm::dot(comVel, fwd);
        comp._vLat = glm::dot(comVel, right);
        comp._swingContact = contact;
        comp._swingForward = glm::dot(swingFoot - stanceFoot, fwd);
        comp._swingFootY = swingFoot.y;
        comp._swingFootVy = swingVelocity.y;
        comp._swingClearance = swingClearance;

        // Assisted locomotion owns foot placement and COM tracking, but not virtual
        // SIMBICON hip torques. This keeps the known-stable powered joint motors active.
        rag.locomotionSimbicon = false;
        rag.locomotionSimbiconBlend = 0.0f;
        rag.locomotionHipTorque[0] = rag.locomotionHipTorque[1] = glm::vec3(0.0f);
        rag.locomotionHipBones[0] = rag.locomotionHipBones[1] = -1;
        rag.locomotionSupportScale = 1.0f;
        comp._torsoTorque = comp._swingTorque = comp._stanceTorque = 0.0f;
        comp._torsoP = comp._torsoD = comp._swingP = comp._swingD = 0.0f;
        comp._saturated = false;

        auto groundFootTarget = [&](const Leg& leg, glm::vec3 target,
                                    float fallbackY) {
            const glm::vec3 probe(target.x, rootPos.y + 0.25f, target.z);
            const HitResult hit = Physics::Raycast(
                probe, glm::vec3(0, -1, 0),
                glm::max(comp.groundRayLength, 0.2f) + 0.75f, entity, false);
            target.y = hit.hit ? hit.point.y + leg.groundOffset : fallbackY;
            return target;
        };
        auto planSwing = [&]() {
            swing->swingStartFoot = swingFoot;
            const float desiredSpeed = glm::length(glm::vec2(
                comp._desiredVelocity.x, comp._desiredVelocity.z));
            const float speedT = glm::clamp(desiredSpeed / glm::max(comp.maxSpeed, 0.01f), 0.0f, 1.0f);
            const float lead = glm::clamp(
                comp.proceduralStepLength * glm::mix(0.65f, 1.0f, speedT)
                    + comp._dSag * 0.20f + comp._vSag * 0.06f,
                0.10f, glm::max(comp.proceduralStepLength * 1.35f, 0.12f));
            const float side = comp._swingIsLeft ? -1.0f : 1.0f;
            glm::vec3 target = com + fwd * lead
                             + right * (side * glm::max(comp.proceduralStepWidth, 0.02f) * 0.5f);
            target = groundFootTarget(*swing, target, stance->plantFoot.y);

            // Keep the requested foothold inside the physical two-link reach envelope.
            const glm::vec3 hip = physicalPosition(swing->hipIdx);
            const float legLength = glm::length(skeleton.bones[swing->kneeIdx].localT)
                                  + glm::length(skeleton.bones[swing->ankleIdx].localT);
            const float maxReach = legLength * glm::clamp(comp.proceduralMaxReach, 0.70f, 0.99f);
            glm::vec2 horizontal(target.x - hip.x, target.z - hip.z);
            const float vertical = target.y - hip.y;
            const float maxHorizontal = std::sqrt(glm::max(maxReach * maxReach
                                                            - vertical * vertical, 0.01f));
            if (const float h = glm::length(horizontal); h > maxHorizontal && h > 1e-5f) {
                horizontal *= maxHorizontal / h;
                target.x = hip.x + horizontal.x;
                target.z = hip.z + horizontal.y;
            }
            swing->swingTargetFoot = target;
        };

        // A tilted sole can touch on one corner while its bone origin is still far above
        // the terrain. Solver contact alone accepted that as a plant in the failing run
        // (roughly 9 cm high), so validate the end-effector height against the ground probe.
        const HitResult swingGround = Physics::Raycast(
            swingFoot + glm::vec3(0, 0.25f, 0), glm::vec3(0, -1, 0),
            0.75f, entity, false);
        const float touchdownExpectedY = swingGround.hit
            ? swingGround.point.y + swing->groundOffset : swingFoot.y;
        comp._touchdownHeightError = swingGround.hit
            ? std::abs(swingFoot.y - touchdownExpectedY) : -1.0f;

        const glm::vec3 leftFootNow = physicalPosition(comp._legL.footIdx);
        const glm::vec3 rightFootNow = physicalPosition(comp._legR.footIdx);
        const float leftLockError = comp._legL.planted ? glm::length(glm::vec2(
            leftFootNow.x - comp._legL.plantFoot.x,
            leftFootNow.z - comp._legL.plantFoot.z)) : 0.0f;
        const float rightLockError = comp._legR.planted ? glm::length(glm::vec2(
            rightFootNow.x - comp._legR.plantFoot.x,
            rightFootNow.z - comp._legR.plantFoot.z)) : 0.0f;
        comp._maxStanceLockError = glm::max(
            comp._maxStanceLockError, glm::max(leftLockError, rightLockError));
        const bool leftContact = FootContact(rag, comp._legL.footIdx);
        const bool rightContact = FootContact(rag, comp._legR.footIdx);
        const float lockTolerance = glm::max(comp.proceduralFootLockTolerance, 0.005f);
        const bool bothLocked = leftContact && rightContact
            && leftLockError <= lockTolerance && rightLockError <= lockTolerance;
        const bool stanceContact = comp._swingIsLeft ? rightContact : leftContact;
        const float stanceError = comp._swingIsLeft ? rightLockError : leftLockError;

        if (!canStep) {
            comp._stateIndex = 0;
            comp._stateTime = 0.0f;
            comp._swingWasAirborne = false;
            comp._airborneTime = 0.0f;
            rag.locomotionSupportTargetWeight = 0.0f;
            rag.locomotionLiftBone = -1;
            rag.locomotionHeightOffset = 0.0f;
        } else {
            comp._stateTime += dt;
            switch (comp._stateIndex) {
                case 0:
                    // Prove the foundation before lifting: both planted feet must remain
                    // within tolerance and the COM must actually settle over the stance side.
                    if (comp._stateTime >= glm::max(comp.weightShiftTime, 0.05f)
                        && bothLocked
                        && std::abs(comp._dLat)
                           <= glm::max(comp.proceduralWeightShiftTolerance, 0.01f)) {
                        planSwing();
                        comp._stateIndex = 1;
                        comp._stateTime = 0.0f;
                        comp._swingWasAirborne = false;
                        comp._airborneTime = 0.0f;
                    }
                    break;
                case 1: {
                    if (!stanceContact || stanceError > lockTolerance * 2.0f) {
                        // Losing the only support invalidates the step. Put both feet back
                        // under double-support control instead of continuing in mid-air.
                        comp._stateIndex = 0;
                        comp._stateTime = 0.0f;
                        comp._swingWasAirborne = false;
                        comp._airborneTime = 0.0f;
                        break;
                    }
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
                case 2: {
                    if (!stanceContact || stanceError > lockTolerance * 2.0f) {
                        comp._stateIndex = 0;
                        comp._stateTime = 0.0f;
                        comp._swingWasAirborne = false;
                        comp._airborneTime = 0.0f;
                        break;
                    }
                    const bool planted = comp._swingWasAirborne && contact &&
                        swingGround.hit && comp._touchdownHeightError <= 0.04f &&
                        contactNormal.y >= 0.5f &&
                        std::abs(swingVelocity.y) <= glm::max(comp.maxPlantVerticalSpeed, 0.1f) &&
                        comp._swingForward >= glm::max(comp.minPlantForward, 0.0f);
                    if (planted) {
                        swing->plantFoot = glm::vec3(
                            swingFoot.x, touchdownExpectedY, swingFoot.z);
                        swing->swingTargetFoot = swing->plantFoot;
                        comp._stateIndex = 3;
                        comp._stateTime = 0.0f;
                        ++comp._steps;
                    } else if (comp._stateTime >= glm::max(comp.assistedPlantTimeout, 0.1f)) {
                        swing->swingStartFoot = swingFoot;
                        comp._stateIndex = 1;
                        comp._stateTime = 0.0f;
                        comp._swingWasAirborne = false;
                        comp._airborneTime = 0.0f;
                    }
                    break;
                }
                case 3:
                    // Do not hand support to the next leg until the new plant has survived
                    // a real double-support interval without sliding out of tolerance.
                    if (comp._stateTime >= glm::max(comp.assistedTransferTime, 0.05f)
                        && bothLocked) {
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

        // Refresh roles after a transition and generate a continuous foot target. Lift/reach
        // rises monotonically to an apex; plant lowers smoothly from that apex to the chosen
        // foothold. No joint target changes discontinuously at a state boundary.
        swing = comp._swingIsLeft ? &comp._legL : &comp._legR;
        stance = comp._swingIsLeft ? &comp._legR : &comp._legL;
        swingFoot = physicalPosition(swing->footIdx);
        stanceFoot = physicalPosition(stance->footIdx);
        contact = FootContact(rag, swing->footIdx, &contactNormal);
        velocityOk = false;
        swingVelocity = Physics::GetRagdollBoneLinearVelocity(rag, swing->footIdx, &velocityOk);
        if (!velocityOk) swingVelocity = glm::vec3(0.0f);
        swingClearance = swingFoot.y - stanceFoot.y;

        const float liftTime = glm::max(comp.assistedLiftTime, 0.1f);
        const float plantTime = glm::max(comp.proceduralPlantTime, 0.05f);
        const float transferTime = glm::max(comp.assistedTransferTime, 0.05f);
        float phaseT = 0.0f;
        if (!canStep) {
            comp._legL.desiredFoot = comp._legL.planted
                ? comp._legL.plantFoot : physicalPosition(comp._legL.footIdx);
            comp._legR.desiredFoot = comp._legR.planted
                ? comp._legR.plantFoot : physicalPosition(comp._legR.footIdx);
            comp._stepPhase = 0.0f;
        } else if (comp._stateIndex == 0) {
            phaseT = glm::clamp(comp._stateTime / glm::max(comp.weightShiftTime, 0.05f), 0.0f, 1.0f);
            comp._stepPhase = 0.15f * phaseT;
            swing->desiredFoot = swing->plantFoot;
            stance->desiredFoot = stance->plantFoot;
        } else if (comp._stateIndex == 1) {
            phaseT = glm::clamp(comp._stateTime / liftTime, 0.0f, 1.0f);
            const float e = smoothstep(phaseT);
            swing->desiredFoot = glm::mix(swing->swingStartFoot, swing->swingTargetFoot, e);
            swing->desiredFoot.y = glm::mix(swing->swingStartFoot.y,
                swing->swingTargetFoot.y + glm::max(comp.assistedLiftHeight, 0.0f), e);
            stance->desiredFoot = stance->plantFoot;
            comp._stepPhase = 0.15f + 0.45f * phaseT;
        } else if (comp._stateIndex == 2) {
            phaseT = glm::clamp(comp._stateTime / plantTime, 0.0f, 1.0f);
            swing->desiredFoot = swing->swingTargetFoot;
            swing->desiredFoot.y += glm::max(comp.assistedLiftHeight, 0.0f)
                                  * (1.0f - smoothstep(phaseT));
            stance->desiredFoot = stance->plantFoot;
            comp._stepPhase = 0.60f + 0.25f * phaseT;
        } else {
            phaseT = glm::clamp(comp._stateTime / transferTime, 0.0f, 1.0f);
            swing->desiredFoot = swing->plantFoot;
            stance->desiredFoot = stance->plantFoot;
            comp._stepPhase = 0.85f + 0.15f * phaseT;
        }

        comp._swingContact = contact;
        comp._swingForward = glm::dot(swingFoot - stanceFoot, fwd);
        comp._swingFootY = swingFoot.y;
        comp._swingFootVy = swingVelocity.y;
        comp._swingClearance = swingClearance;
        comp._liftTargetY = swing->desiredFoot.y;
        comp._stanceLockError = stance->planted ? glm::length(glm::vec2(
            stanceFoot.x - stance->plantFoot.x,
            stanceFoot.z - stance->plantFoot.z)) : 0.0f;
        if (stance->planted)
            comp._maxStanceLockError = glm::max(comp._maxStanceLockError,
                                                comp._stanceLockError);
        comp._swingTargetError = glm::length(swingFoot - swing->desiredFoot);

        auto commandLock = [&](int slot, Leg& leg, bool enabled) {
            const float blendStep = dt / 0.15f;
            leg.lockWeight = Approach(leg.lockWeight, enabled ? 1.0f : 0.0f,
                                      blendStep);
            rag.locomotionFootLockBones[slot] = leg.lockWeight > 0.001f
                ? leg.footIdx : -1;
            rag.locomotionFootLockTargets[slot] = leg.plantFoot;
            rag.locomotionFootLockWeights[slot] = leg.lockWeight;
        };
        const bool upright = tiltDeg < rag.locomotionFallenTilt;
        const bool doubleSupport = !canStep || comp._stateIndex == 0 || comp._stateIndex == 3;
        commandLock(0, comp._legL, walking && comp._legL.planted && upright
            && (doubleSupport || !comp._swingIsLeft));
        commandLock(1, comp._legR, walking && comp._legR.planted && upright
            && (doubleSupport || comp._swingIsLeft));

        // Standing uses the full safety net. During a valid single-support phase, let the
        // stance leg carry most of the weight and retain only a reduced, force-based catch.
        rag.locomotionSupportScale = 1.0f;
        if (canStep) {
            const bool singleSupport = comp._stateIndex == 1 || comp._stateIndex == 2;
            const bool trustedStance = stanceContact
                && comp._stanceLockError <= lockTolerance * 2.0f;
            if (singleSupport && trustedStance)
                rag.locomotionSupportScale = glm::clamp(
                    comp.proceduralStanceSupportScale, 0.0f, 1.0f);
            else
                rag.locomotionSupportScale = 0.80f;
        }

        if (canStep && comp._stateIndex == 1) {
            rag.locomotionLiftBone = swing->footIdx;
            rag.locomotionLiftTargetY = swing->desiredFoot.y;
            rag.locomotionLiftFrequency = glm::max(comp.assistedLiftFrequency, 0.0f);
            rag.locomotionLiftMaxForce = glm::max(comp.assistedLiftMaxForce, 0.0f);
        } else {
            rag.locomotionLiftBone = -1;
        }

        if (canStep) {
            // Input never pushes the pelvis. State 0 shifts laterally over the stance foot;
            // swing holds that support; only a confirmed plant lets state 3 transfer the COM
            // from the old plant to the new one, which is the sole source of forward travel.
            const float shift = glm::clamp(comp.proceduralSupportShift, 0.0f, 1.0f);
            glm::vec3 support = stance->plantFoot;
            if (comp._stateIndex == 0) {
                const float lateralError = glm::dot(stance->plantFoot - com, right) * shift;
                support = com + right * lateralError;
            } else if (comp._stateIndex == 3) {
                support = glm::mix(stance->plantFoot, swing->plantFoot,
                                   smoothstep(phaseT));
            } else {
                support = glm::mix(com, stance->plantFoot, shift);
            }
            rag.locomotionSupportTarget = glm::vec3(support.x, com.y, support.z);
            rag.locomotionSupportTargetVel = glm::vec3(0.0f);
            rag.locomotionSupportTargetWeight = 1.0f;
            rag.locomotionCOMSupportFreq = glm::max(comp.proceduralSupportFrequency, 0.1f);
            rag.locomotionCOMSupportMaxAccel = glm::max(comp.proceduralSupportMaxAccel, 0.1f);
            rag.locomotionHeightOffset = -glm::max(comp.proceduralPelvisBob, 0.0f)
                * std::cos(glm::two_pi<float>() * comp._stepPhase);
        }

        if (!comp.ikWriteEnabled) return;
        const float poseWeight = glm::clamp(comp._poseBlend * comp.poseWeight, 0.0f, 1.0f);
        const float commandAlpha = 1.0f - std::exp(
            -dt / glm::max(comp.proceduralPoseResponse, 0.01f));

        auto solveLeg = [&](Leg& leg, float side, bool procedural) {
            glm::quat hipTarget = skeleton.bones[leg.hipIdx].localR;
            glm::quat kneeTarget = skeleton.bones[leg.kneeIdx].localR;
            glm::quat ankleTarget = skeleton.bones[leg.ankleIdx].localR;
            glm::quat footTarget = skeleton.bones[leg.footIdx].localR;

            if (procedural) {
                const glm::vec3 hipPos = physicalPosition(leg.hipIdx);
                const float upperLength = glm::length(skeleton.bones[leg.kneeIdx].localT);
                const float lowerLength = glm::length(skeleton.bones[leg.ankleIdx].localT);
                if (upperLength > 1e-4f && lowerLength > 1e-4f) {
                    // The foot->ankle offset is captured once at plant acquisition. Using
                    // the live offset fed foot-body rotation back into the IK target; the
                    // ankle motor would rotate, move its own target, and reverse on the next
                    // frame. That loop was the large command oscillation in LocoMotor.
                    glm::vec3 desiredAnkle = leg.desiredFoot + leg.ankleFromFootWorld;
                    glm::vec3 toTarget = desiredAnkle - hipPos;
                    const float maxReach = (upperLength + lowerLength)
                        * glm::clamp(comp.proceduralMaxReach, 0.70f, 0.99f);
                    if (const float reach = glm::length(toTarget); reach > maxReach && reach > 1e-5f) {
                        toTarget *= maxReach / reach;
                        desiredAnkle = hipPos + toTarget;
                    }
                    const float distance = glm::clamp(glm::length(toTarget),
                        std::abs(upperLength - lowerLength) + 1e-4f,
                        upperLength + lowerLength - 1e-4f);
                    const float includedCos = glm::clamp(
                        (upperLength * upperLength + lowerLength * lowerLength
                         - distance * distance) / (2.0f * upperLength * lowerLength),
                        -1.0f, 1.0f);
                    const float kneeBend = glm::pi<float>() - std::acos(includedCos);
                    const float kneeDelta = kneeBend - leg.referenceKneeBend;
                    kneeTarget = glm::normalize(skeleton.bones[leg.kneeIdx].localR
                        * glm::angleAxis(kneeDelta, glm::normalize(leg.kneeHingeAxis)));

                    // Stable analytic two-bone solve. Compute the desired KNEE point and
                    // align only the bind femur direction to it. Reconstructing a complete
                    // three-axis basis from nearly collinear vectors introduced quaternion
                    // branch flips, visible as 10 -> 54 -> 27 degree hip commands.
                    const glm::vec3 worldForward = glm::normalize(toTarget);
                    glm::vec3 pole = leg.kneePoleWorld;
                    glm::vec3 worldBend = pole - worldForward * glm::dot(pole, worldForward);
                    if (glm::dot(worldBend, worldBend) < 1e-8f)
                        worldBend = fwd + right * (side * 0.08f);
                    worldBend -= worldForward * glm::dot(worldBend, worldForward);
                    if (glm::dot(worldBend, worldBend) < 1e-8f)
                        worldBend = right;
                    else worldBend = glm::normalize(worldBend);
                    const float hipCos = glm::clamp(
                        (upperLength * upperLength + distance * distance
                         - lowerLength * lowerLength)
                        / (2.0f * upperLength * distance), -1.0f, 1.0f);
                    const float hipSin = std::sqrt(glm::max(1.0f - hipCos * hipCos, 0.0f));
                    const glm::vec3 desiredUpper = glm::normalize(
                        worldForward * hipCos + worldBend * hipSin);
                    const glm::quat parentWorld = ParentWorldRot(
                        rag, skeleton, animator, entityWorld, leg.hipIdx);
                    const glm::quat bindHipWorld = glm::normalize(
                        parentWorld * skeleton.bones[leg.hipIdx].localR);
                    const glm::vec3 bindUpper = glm::normalize(
                        bindHipWorld * glm::normalize(skeleton.bones[leg.kneeIdx].localT));
                    const glm::quat hipWorld = glm::normalize(
                        RotationBetween(bindUpper, desiredUpper) * bindHipWorld);
                    hipTarget = glm::normalize(glm::conjugate(parentWorld) * hipWorld);
                    Envelope hipEnvelope;
                    hipEnvelope.twistAxis = leg.hipTwistAxis;
                    hipEnvelope.swingNormalDeg = leg.hipSwingNormalDeg;
                    hipEnvelope.swingPlaneDeg = leg.hipSwingPlaneDeg;
                    hipEnvelope.twistMinDeg = leg.hipTwistMinDeg;
                    hipEnvelope.twistMaxDeg = leg.hipTwistMaxDeg;
                    hipTarget = ClampToEnvelope(hipEnvelope,
                        skeleton.bones[leg.hipIdx].localR, hipTarget,
                        comp.hipLimitMarginDeg);

                    // Hold the authored ankle relation for this foundation pass. The former
                    // world-leveling calculation was coupled to the moving hip/knee solve and
                    // injected visible twist into the feet. Foot orientation can be layered
                    // back later from the measured ground normal without changing leg reach.
                    ankleTarget = skeleton.bones[leg.ankleIdx].localR;
                    footTarget = glm::normalize(glm::conjugate(ParentWorldRot(
                        rag, skeleton, animator, entityWorld, leg.footIdx))
                        * leg.plantedFootWorldRotation);

                    if (&leg == swing) {
                        comp._femurCmd = glm::normalize(hipWorld
                            * glm::normalize(skeleton.bones[leg.kneeIdx].localT));
                        const glm::vec3 actualUpper = physicalPosition(leg.kneeIdx) - hipPos;
                        if (glm::dot(actualUpper, actualUpper) > 1e-8f)
                            comp._femurActual = glm::normalize(actualUpper);
                        comp._femurErrDeg = glm::degrees(std::acos(glm::clamp(
                            glm::dot(comp._femurCmd, comp._femurActual), -1.0f, 1.0f)));
                    }

                    if (comp.debug) {
                        DebugDraw::Sphere(leg.desiredFoot, 0.03f,
                                          &leg == swing ? glm::vec3(1.0f, 0.55f, 0.1f)
                                                       : glm::vec3(0.2f, 1.0f, 0.2f));
                        DebugDraw::Line(hipPos, desiredAnkle, {0.7f, 0.3f, 1.0f});
                    }
                }
            }

            if (!leg.commandValid) {
                leg.hipCommand = glm::normalize(animator.pose[leg.hipIdx].rotation);
                leg.kneeCommand = glm::normalize(animator.pose[leg.kneeIdx].rotation);
                leg.ankleCommand = glm::normalize(animator.pose[leg.ankleIdx].rotation);
                leg.footCommand = glm::normalize(animator.pose[leg.footIdx].rotation);
                leg.commandValid = true;
            }
            leg.hipCommand = glm::normalize(glm::slerp(leg.hipCommand, hipTarget, commandAlpha));
            leg.kneeCommand = glm::normalize(glm::slerp(leg.kneeCommand, kneeTarget, commandAlpha));
            leg.ankleCommand = glm::normalize(glm::slerp(leg.ankleCommand, ankleTarget, commandAlpha));
            leg.footCommand = glm::normalize(glm::slerp(leg.footCommand, footTarget, commandAlpha));
            BlendPose(animator, leg.hipIdx, leg.hipCommand, poseWeight);
            BlendPose(animator, leg.kneeIdx, leg.kneeCommand, poseWeight);
            BlendPose(animator, leg.ankleIdx, leg.ankleCommand, poseWeight);
            BlendPose(animator, leg.footIdx, leg.footCommand, poseWeight);
        };

        // Test 3 owns exactly one swing leg during takeoff/lift/hold/lower/contact. Do not even
        // update the standing command for that leg here: blending two pose writers was one
        // of the feedback conflicts this validation ladder is intended to eliminate.
        const bool test3OwnsSwing = comp.validationTest == 3
            && comp._test3Phase >= 2 && comp._test3Phase <= 6;
        const bool test3OwnsLeft = test3OwnsSwing && comp._test3SupportSide > 0;
        const bool test3OwnsRight = test3OwnsSwing && comp._test3SupportSide < 0;
        if (!test3OwnsLeft) solveLeg(comp._legL, -1.0f, canStep);
        if (!test3OwnsRight) solveLeg(comp._legR, 1.0f, canStep);

        // Small phase-continuous torso counter-motion supplies life without an authored clip.
        // It is world-referenced like the paper's torso target and converted through the live
        // physical parent, so it cannot inherit a twisted pelvis frame.
        const int torsoIdx = skeleton.Find(comp.torsoBone);
        if (torsoIdx >= 0) {
            const float speedT = glm::clamp(glm::length(glm::vec2(
                comp._desiredVelocity.x, comp._desiredVelocity.z))
                / glm::max(comp.maxSpeed, 0.01f), 0.0f, 1.0f);
            const float side = comp._swingIsLeft ? -1.0f : 1.0f;
            const float roll = canStep ? side * comp.proceduralTorsoRollDeg
                * std::sin(glm::pi<float>() * comp._stepPhase) : 0.0f;
            const float pitch = canStep ? -comp.proceduralTorsoPitchDeg * speedT : 0.0f;
            const glm::quat torsoWorld = glm::normalize(
                glm::angleAxis(glm::radians(roll), fwd)
                * glm::angleAxis(glm::radians(pitch), right)
                * heading * entityRot * BindModelRot(skeleton, torsoIdx));
            const glm::quat torsoLocal = glm::normalize(glm::conjugate(ParentWorldRot(
                rag, skeleton, animator, entityWorld, torsoIdx)) * torsoWorld);
            BlendPose(animator, torsoIdx, torsoLocal, poseWeight);
        }

        comp._swingHipCmdDeg = 0.0f;
        comp._swingHipLatCmdDeg = 0.0f;
        comp._engineTiltDeg = tiltDeg;
        comp._myTiltDeg = tiltDeg;
    }

    // Kept temporarily as a diagnostic fallback while the continuous procedural solver is
    // brought up. The normal assisted path calls UpdateProceduralGait below.
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
            UpdateProceduralGait(scene, entity, comp, rag, walking, dt);
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
