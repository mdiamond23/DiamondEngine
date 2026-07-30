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
#include "LocomotionGaitRuntime.h"

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
    float runtimeTurnSpeedDeg = 540.0f;
    float deadzone        = 0.2f;
    float minWalkSpeed    = 0.15f;
    float facingOffsetDeg = 90.0f;
    float groundRayLength = 1.2f;
    float consciousness   = 1.0f;
    float postPoweredGrace = 0.4f;
    float gaitBlendTime   = 0.6f;

    // Bring-up validation: 0 = normal locomotion, 1 = standing/shove, 2 = weight shift,
    // 3 = single-leg lift, 4 = one forward step, 5 = support transfer,
    // 6 = two-step handoff, 7 = continuous gait, 8 = grounded motor isolation. Validation
    // modes retain physical standing but categorically bypass the gait path.
    int   validationTest = 0;
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
    float test4StepLength = 0.225f;
    float test4SwingHeight = 0.10f;
    float test4SwingTime = 0.90f;
    float test4ArrivalHeight = 0.05f;
    float test4ArrivalTolerance = 0.02f;
    float test4ArrivalSettleTime = 0.10f;
    float test4ArrivalTimeout = 0.60f;
    float test4DescentTime = 0.35f;
    float test4PlantTimeout = 0.35f;
    float test4PlantAcquireTime = 0.10f;
    float test4PlantAcquireMaxSpeed = 0.05f;
    float test4PlantAcquireTimeout = 0.60f;
    float test4ContactSettleTime = 0.30f;
    float test4TouchdownMaxVerticalSpeed = 0.25f;
    float test4TouchdownMinNormalY = 0.70f;
    float test4TargetTolerance = 0.04f;
    float test4SafeReachFraction = 0.99f;
    float test5TransferTime = 1.00f;
    float test5SupportFraction = 0.92f;
    float test5ComTolerance = 0.04f;
    float test5HoldTime = 0.50f;
    float test5HoldTimeout = 1.50f;
    float test6InterStepTime = 0.25f;
    float test6DriftGrowthTolerance = 0.015f;
    float test7DesiredSpeed = 0.020f;
    float test7PlacementGain = 0.75f;
    float test7NominalAdvance = 0.10f;
    float test7MinStepLength = 0.06f;
    float test7MaxStepLength = 0.14f;
    float test7ReachCrouch = 0.018f;
    float test7CrouchTime = 0.35f;
    float test7UsableReachFraction = 0.975f;
    float test7SoleLevelTime = 0.35f;
    float test7FootPositionGain = 0.45f;
    float test7FootVelocityLeadTime = 0.08f;
    float test7MaxFootCorrection = 0.045f;
    float test7InterStepRecenterTime = 0.35f;
    float test7UprightStiffness = 700.0f;
    float test7UprightDamping = 100.0f;
    float test7UprightMaxTorque = 250.0f;
    float test7HeadingStiffness = 180.0f;
    float test7HeadingDamping = 50.0f;
    float test7HeadingMaxTorque = 80.0f;
    float test7InterStepTiltLimit = 15.0f;
    float test7InterStepHeadingLimit = 8.0f;
    int   test7TargetSteps = 10;
    bool  test7EnduranceRun = false;
    float test7EnduranceTime = 60.0f;
    float test7StopTime = 1.0f;
    float test7StopHoldTime = 0.50f;

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
        glm::vec3 ankleFromFootLocal { 0.0f };
        glm::vec3 kneePoleWorld { 0.0f, 0.0f, -1.0f };
        glm::vec3 groundReferenceKneePoleHeadingLocal { 0.0f, 0.0f, -1.0f };
        glm::quat plantedFootWorldRotation { 1, 0, 0, 0 };
        // Captured once from the settled stance relative to the heading that produced it.
        // Reapplying the active gait heading preserves the flat sole while allowing its yaw
        // and the anatomical knee-bend plane to follow a turn.
        glm::quat groundReferenceFootHeadingLocalRotation { 1, 0, 0, 0 };
        bool groundReferenceFootRotationValid = false;
        bool groundReferenceKneePoleValid = false;
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
    bool  _test4BaselineValid = false;
    bool  _test4ContactL = false, _test4ContactR = false;
    bool  _test4PrevSwingContact = false;
    bool  _test4TouchdownAccepted = false;
    bool  _test4Aborted = false;
    float _test4Time = 0.0f;
    float _test4SettleTime = 0.0f;
    int   _test4Phase = 0;
    float _test4PhaseTime = 0.0f;
    int   _test4SupportSide = 0;
    float _test4ComCommand = 0.0f;
    float _test4ComLateral = 0.0f, _test4TargetLateral = 0.0f;
    float _test4AirborneTime = 0.0f;
    float _test4ArrivalStableTime = 0.0f;
    float _test4ReachLimit = 0.0f;
    float _test4PlantAcquireStableTime = 0.0f;
    float _test4TrajectoryT = 0.0f;
    float _test4Clearance = 0.0f;
    float _test4ForwardTravel = 0.0f;
    float _test4TargetError = 0.0f;
    float _test4HorizontalTargetError = 0.0f;
    float _test4ForwardTargetError = 0.0f;
    float _test4LateralTargetError = 0.0f;
    float _test4VerticalTargetError = 0.0f;
    float _test4TouchdownVy = 0.0f;
    float _test4TouchdownNormalY = 0.0f;
    float _test4StanceDrift = 0.0f, _test4PlantDrift = 0.0f;
    float _test4MaxStanceDrift = 0.0f, _test4MaxPlantDrift = 0.0f;
    float _test4InitialTilt = 0.0f, _test4PeakTilt = 0.0f, _test4FinalTilt = 0.0f;
    float _test4MaxMotorRatio = 0.0f;
    bool  _test4MotorSaturated = false;
    bool  _test4PlantPoseCaptured = false;
    glm::vec3 _test4FootBaselineL { 0.0f };
    glm::vec3 _test4FootBaselineR { 0.0f };
    glm::vec3 _test4ComBaseline { 0.0f };
    glm::vec3 _test4Right { 1.0f, 0.0f, 0.0f };
    glm::vec3 _test4Forward { 0.0f, 0.0f, -1.0f };
    glm::vec3 _test4SupportTarget { 0.0f };
    glm::vec3 _test4SwingStart { 0.0f };
    glm::vec3 _test4ArcStart { 0.0f };
    glm::vec3 _test4Foothold { 0.0f };
    glm::vec3 _test4DesiredFoot { 0.0f };
    glm::vec3 _test4TouchdownPlant { 0.0f };
    glm::vec3 _test4ApiVelocity { 0.0f };
    glm::vec3 _test4MeasuredVelocity { 0.0f };
    glm::vec3 _test4PreviousSwingFoot { 0.0f };
    bool _test4PreviousSwingFootValid = false;
    float _test4FootUpY = 1.0f;
    glm::vec3 _test4ContactPoint { 0.0f };
    glm::vec3 _test4ContactLocal { 0.0f };
    float _test5TransferT = 0.0f;
    float _test5HoldStableTime = 0.0f;
    float _test5ContactLossTime = 0.0f;
    float _test5ComError = 0.0f;
    float _test5ComToOldSupport = 0.0f;
    float _test5ComToNewSupport = 0.0f;
    float _test5ComHorizontalSpeed = 0.0f;
    glm::vec3 _test5TransferStartTarget { 0.0f };
    glm::vec3 _test5TransferEndTarget { 0.0f };
    int _test6StepIndex = 0;
    int _test6StepsCompleted = 0;
    float _test6InterStepStableTime = 0.0f;
    float _test6InitialTilt = 0.0f;
    float _test6StepForward[2] { 0.0f, 0.0f };
    float _test6StepMaxDrift[2] { 0.0f, 0.0f };
    float _test6StepPeakTilt[2] { 0.0f, 0.0f };
    float _test6StepMotorRatio[2] { 0.0f, 0.0f };
    bool _test6PreviousContactsValid = false;
    bool _test6PreviousContactL = false;
    bool _test6PreviousContactR = false;
    int _test6ContactTransitionsL = 0;
    int _test6ContactTransitionsR = 0;
    float _test7ContactChangeTimeL = 0.0f;
    float _test7ContactChangeTimeR = 0.0f;
    bool _test7Running = false;
    bool _test7StopRequested = false;
    float _test7RunTime = 0.0f;
    float _test7StepStartTime = 0.0f;
    float _test7LastStepPeriod = 0.0f;
    float _test7PreviousStepPeriod = 0.0f;
    float _test7MeasuredSpeed = 0.0f;
    float _test7CommandedStepLength = 0.10f;
    float _test7ReachCommandCeiling = 0.14f;
    float _test7TakeoffContactRecoveryTime = 0.0f;
    float _test7SwingRecontactTime = 0.0f;
    float _test7SettledTrackingLoss = 0.0f;
    float _test7ForwardPreShift = 0.0f;
    int _test7ReachClearSteps = 0;
    float _test7PlannedSupportAdvance = 0.0f;
    float _test7AchievedSupportAdvance = 0.0f;
    float _test7LastStepLength = 0.0f;
    float _test7PreviousStepLength = 0.0f;
    float _test7LastSupportAdvance = 0.0f;
    float _test7PreviousSupportAdvance = 0.0f;
    float _test7StepMaxRelevantDrift = 0.0f;
    float _test7MaxDrift = 0.0f;
    float _test7PeakTilt = 0.0f;
    float _test7MaxMotorRatio = 0.0f;
    float _test7StopStableTime = 0.0f;
    float _test7StopFootDriftL = 0.0f;
    float _test7StopFootDriftR = 0.0f;
    float _test7StopMaxFootDrift = 0.0f;
    float _test7StopSettleFootDriftL = 0.0f;
    float _test7StopSettleFootDriftR = 0.0f;
    float _test7StopMaxSettleFootDrift = 0.0f;
    float _test7CrouchBlend = 0.0f;
    float _test7IkRequestedReach = 0.0f;
    float _test7IkClampedReach = 0.0f;
    float _test7IkMaxReach = 0.0f;
    float _test7IkPhysicalReach = 0.0f;
    float _test7IkReachShortfall = 0.0f;
    float _test7IkReachShortfallForward = 0.0f;
    float _test7IkHipEnvelopeClampDeg = 0.0f;
    float _test7IkHipCommandLagDeg = 0.0f;
    float _test7IkKneeCommandLagDeg = 0.0f;
    float _test7IkKneeBendDeg = 0.0f;
    float _test7IkHipTravelForward = 0.0f;
    float _test7IkHipTravelLateral = 0.0f;
    float _test7IkHipTravelVertical = 0.0f;
    float _test7FootCorrection = 0.0f;
    float _test7FootCorrectionForward = 0.0f;
    float _test7FootTargetSpeed = 0.0f;
    float _test7SoleLevelBlend = 0.0f;
    float _test7SoleAngularErrorDeg = 0.0f;
    float _test7InterStepRecenterT = 0.0f;
    float _test7InterStepCenterError = 0.0f;
    float _test7RootPitchRate = 0.0f;
    float _test7RootRollRate = 0.0f;
    float _test7RootYawRate = 0.0f;
    float _test7RootTiltRate = 0.0f;
    float _test7HeadingErrorDeg = 0.0f;
    float _test7PeakHeadingErrorDeg = 0.0f;
    bool _test7TakeoffContactRecoveryActive = false;
    bool _test7IkPlanHipValid = false;
    bool _test7ReachClampedStep = false;
    bool _test7OldSupportDriftAllowanceLogged = false;
    bool _test7StopSettleReferenceValid = false;
    // True when the reusable continuous-gait core owns the walking pose. Test 7 is the
    // first command producer; gameplay input will become another without changing the core.
    bool _continuousGaitEnabled = false;
    bool _runtimeWalkIntent = false;
    bool _runtimeRestartBlocked = false;
    bool _runtimeTurnActive = false;
    int _runtimeNextSupportSide = -1;
    float _runtimeTurnElapsed = 0.0f;
    float _runtimeTurnDuration = 0.0f;
    float _runtimeTurnTotalYaw = 0.0f;
    float _runtimeTurnAppliedYaw = 0.0f;
    glm::vec3 _runtimeDesiredForward { 0.0f, 0.0f, -1.0f };
    glm::vec3 _runtimeTurnTargetForward { 0.0f, 0.0f, -1.0f };
    glm::vec3 _test7StartCom { 0.0f };
    glm::vec3 _test7StepStartCom { 0.0f };
    glm::vec3 _test7IkPlanHip { 0.0f };
    glm::vec3 _test7InterStepRecenterStart { 0.0f };
    glm::vec3 _test7InterStepRecenterTarget { 0.0f };
    glm::vec3 _test7StopStartTarget { 0.0f };
    glm::vec3 _test7StopEndTarget { 0.0f };
    glm::vec3 _test7StopFootTargetL { 0.0f };
    glm::vec3 _test7StopFootTargetR { 0.0f };
    glm::vec3 _test7StopSettleFootTargetL { 0.0f };
    glm::vec3 _test7StopSettleFootTargetR { 0.0f };
    glm::quat _test7HeadingTargetRot { 1.0f, 0.0f, 0.0f, 0.0f };
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
    ImGui::DragFloat("Runtime Turn Speed", &c.runtimeTurnSpeedDeg,
                     10.0f, 90.0f, 1080.0f, "%.0f deg/s");
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
                                "Test 4 - one forward step",
                                "Test 5 - support transfer",
                                "Test 6 - second step",
                                "Test 7 - continuous gait",
                                "Diagnostic - grounded motor isolation" };
        ImGui::Combo("Active Test", &c.validationTest, tests, IM_ARRAYSIZE(tests));
        c.validationTest = glm::clamp(c.validationTest, 0, 8);

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
        ImGui::TextDisabled("Test 4: F6=left support/right step, F7=mirror, F8=recapture");
        ImGui::DragFloat("Test 4 Step Length", &c.test4StepLength,
                         0.005f, 0.15f, 0.25f, "%.3f m");
        ImGui::DragFloat("Test 4 Swing Height", &c.test4SwingHeight,
                         0.005f, 0.06f, 0.20f, "%.3f m");
        ImGui::DragFloat("Test 4 Swing Time", &c.test4SwingTime,
                         0.025f, 0.30f, 2.0f, "%.2f s");
        ImGui::DragFloat("Test 4 Arrival Height", &c.test4ArrivalHeight,
                         0.0025f, 0.03f, 0.10f, "%.3f m");
        ImGui::DragFloat("Test 4 Arrival Tolerance", &c.test4ArrivalTolerance,
                         0.0025f, 0.01f, 0.06f, "%.3f m");
        ImGui::DragFloat("Test 4 Arrival Settle Time", &c.test4ArrivalSettleTime,
                         0.025f, 0.05f, 0.50f, "%.2f s");
        ImGui::DragFloat("Test 4 Arrival Timeout", &c.test4ArrivalTimeout,
                         0.025f, 0.10f, 0.75f, "%.2f s");
        ImGui::DragFloat("Test 4 Descent Time", &c.test4DescentTime,
                         0.025f, 0.15f, 1.0f, "%.2f s");
        ImGui::DragFloat("Test 4 Plant Timeout", &c.test4PlantTimeout,
                         0.025f, 0.10f, 1.0f, "%.2f s");
        ImGui::DragFloat("Test 4 Plant Stable Time", &c.test4PlantAcquireTime,
                         0.025f, 0.05f, 0.50f, "%.2f s");
        ImGui::DragFloat("Test 4 Plant Max Speed", &c.test4PlantAcquireMaxSpeed,
                         0.005f, 0.01f, 0.20f, "%.3f m/s");
        ImGui::DragFloat("Test 4 Plant Acquire Timeout", &c.test4PlantAcquireTimeout,
                         0.025f, 0.20f, 1.50f, "%.2f s");
        ImGui::DragFloat("Test 4 Settle Time", &c.test4ContactSettleTime,
                         0.025f, 0.30f, 2.0f, "%.2f s");
        ImGui::DragFloat("Touchdown Max Vy", &c.test4TouchdownMaxVerticalSpeed,
                         0.01f, 0.05f, 1.0f, "%.2f m/s");
        ImGui::SliderFloat("Touchdown Min Normal Y", &c.test4TouchdownMinNormalY,
                           0.35f, 1.0f);
        ImGui::DragFloat("Test 4 Target Tolerance", &c.test4TargetTolerance,
                         0.0025f, 0.01f, 0.08f, "%.3f m");
        ImGui::SliderFloat("Test 4 Safe Reach Fraction", &c.test4SafeReachFraction,
                           0.94f, 0.995f, "%.3f");
        static const char* test4Phases[] = {
            "IDLE", "WEIGHT_SHIFT", "TAKEOFF", "SWING", "ARRIVAL",
            "DESCENT", "TOUCHDOWN_WAIT", "SETTLE", "TRANSFER", "HOLD",
            "INTERSTEP", "COMPLETE", "ABORT", "STOPPING", "RETURN_STAND"
        };
        const int test4PhaseIndex = glm::clamp(c._test4Phase, 0, 14);
        ImGui::TextDisabled("baseline=%s phase=%s side=%d t=%.2f path=%.2f",
                            c._test4BaselineValid ? "ready" : "waiting",
                            test4Phases[test4PhaseIndex], c._test4SupportSide,
                            c._test4PhaseTime, c._test4TrajectoryT);
        ImGui::TextDisabled("contact=(%s,%s) clear=%.3f forward=%.3f err=%.3f m",
                            c._test4ContactL ? "L" : "-",
                            c._test4ContactR ? "R" : "-",
                            c._test4Clearance, c._test4ForwardTravel,
                            c._test4TargetError);
        ImGui::TextDisabled("target h/fwd/lat/y=(%.3f,%+.3f,%+.3f,%.3f) m",
                            c._test4HorizontalTargetError,
                            c._test4ForwardTargetError,
                            c._test4LateralTargetError,
                            c._test4VerticalTargetError);
        ImGui::TextDisabled("velocity api/fd y=(%+.3f,%+.3f) upY=%.2f",
                            c._test4ApiVelocity.y,
                            c._test4MeasuredVelocity.y, c._test4FootUpY);
        ImGui::TextDisabled("touchdown=%s vy=%+.3f normalY=%.2f drift=(%.3f,%.3f) m",
                            c._test4TouchdownAccepted ? "accepted" : "waiting",
                            c._test4TouchdownVy, c._test4TouchdownNormalY,
                            c._test4StanceDrift, c._test4PlantDrift);
        ImGui::TextDisabled("tilt=%.1f/%.1f/%.1f motorRatio=%.2f sat=%s",
                            c._test4InitialTilt, c._test4PeakTilt, c._test4FinalTilt,
                            c._test4MaxMotorRatio,
                            c._test4MotorSaturated ? "YES" : "no");

        ImGui::Separator();
        ImGui::TextDisabled("Test 5: Test 4 step, then isolated COM transfer and hold");
        ImGui::DragFloat("Test 5 Transfer Time", &c.test5TransferTime,
                         0.05f, 0.25f, 3.0f, "%.2f s");
        ImGui::SliderFloat("Test 5 Support Fraction", &c.test5SupportFraction,
                           0.70f, 1.0f);
        ImGui::DragFloat("Test 5 COM Tolerance", &c.test5ComTolerance,
                         0.0025f, 0.01f, 0.10f, "%.3f m");
        ImGui::DragFloat("Test 5 Hold Time", &c.test5HoldTime,
                         0.05f, 0.25f, 2.0f, "%.2f s");
        ImGui::DragFloat("Test 5 Hold Timeout", &c.test5HoldTimeout,
                         0.05f, 0.50f, 4.0f, "%.2f s");
        ImGui::TextDisabled("transfer=%.2f hold=%.2f COM err=%.3f speed=%.3f m/s",
                            c._test5TransferT, c._test5HoldStableTime,
                            c._test5ComError, c._test5ComHorizontalSpeed);
        ImGui::TextDisabled("COM distance old/new=(%.3f,%.3f) m",
                            c._test5ComToOldSupport,
                            c._test5ComToNewSupport);

        ImGui::Separator();
        ImGui::TextDisabled("Test 6: step, transfer, role swap, step, transfer");
        ImGui::DragFloat("Test 6 Inter-Step Hold", &c.test6InterStepTime,
                         0.025f, 0.10f, 1.0f, "%.2f s");
        ImGui::DragFloat("Test 6 Drift Growth Tolerance", &c.test6DriftGrowthTolerance,
                         0.0025f, 0.0f, 0.04f, "%.3f m");
        ImGui::TextDisabled("step=%d completed=%d interStable=%.2f transitions=(%d,%d)",
                            c._test6StepIndex, c._test6StepsCompleted,
                            c._test6InterStepStableTime,
                            c._test6ContactTransitionsL,
                            c._test6ContactTransitionsR);
        ImGui::TextDisabled("forward=(%.3f,%.3f) drift=(%.3f,%.3f) m",
                            c._test6StepForward[0], c._test6StepForward[1],
                            c._test6StepMaxDrift[0], c._test6StepMaxDrift[1]);
        ImGui::TextDisabled("peakTilt=(%.1f,%.1f) motorRatio=(%.2f,%.2f)",
                            c._test6StepPeakTilt[0], c._test6StepPeakTilt[1],
                            c._test6StepMotorRatio[0], c._test6StepMotorRatio[1]);

        ImGui::Separator();
        ImGui::TextDisabled("Test 7: F6/F7=start, F8=finish current step and stop");
        ImGui::DragFloat("Test 7 Desired Speed", &c.test7DesiredSpeed,
                         0.0025f, 0.005f, 0.08f, "%.3f m/s");
        ImGui::DragFloat("Test 7 Placement Gain", &c.test7PlacementGain,
                         0.05f, 0.0f, 3.0f, "%.2f s");
        ImGui::DragFloat("Test 7 Nominal Support Advance", &c.test7NominalAdvance,
                         0.005f, 0.04f, 0.18f, "%.3f m");
        ImGui::DragFloat("Test 7 Min Support Advance", &c.test7MinStepLength,
                         0.005f, 0.03f, 0.12f, "%.3f m");
        ImGui::DragFloat("Test 7 Max Support Advance", &c.test7MaxStepLength,
                         0.005f, 0.08f, 0.20f, "%.3f m");
        ImGui::DragFloat("Test 7 Reach Crouch", &c.test7ReachCrouch,
                         0.002f, 0.0f, 0.04f, "%.3f m");
        ImGui::DragFloat("Test 7 Crouch Ramp", &c.test7CrouchTime,
                         0.025f, 0.10f, 1.0f, "%.2f s");
        ImGui::DragFloat("Test 7 Usable Reach", &c.test7UsableReachFraction,
                         0.0025f, 0.94f, 0.99f, "%.3f");
        ImGui::DragFloat("Test 7 Sole Level Time", &c.test7SoleLevelTime,
                         0.025f, 0.10f, 0.90f, "%.2f s");
        ImGui::DragFloat("Test 7 Foot Position Gain", &c.test7FootPositionGain,
                         0.025f, 0.0f, 1.0f, "%.2f");
        ImGui::DragFloat("Test 7 Foot Velocity Lead", &c.test7FootVelocityLeadTime,
                         0.005f, 0.0f, 0.20f, "%.3f s");
        ImGui::DragFloat("Test 7 Max Foot Correction", &c.test7MaxFootCorrection,
                         0.0025f, 0.0f, 0.08f, "%.3f m");
        ImGui::DragFloat("Test 7 Inter-Step Recenter", &c.test7InterStepRecenterTime,
                         0.025f, 0.20f, 1.0f, "%.2f s");
        ImGui::DragFloat("Test 7 Upright Stiffness", &c.test7UprightStiffness,
                         25.0f, 0.0f, 2000.0f, "%.0f N*m/rad");
        ImGui::DragFloat("Test 7 Upright Damping", &c.test7UprightDamping,
                         5.0f, 0.0f, 500.0f, "%.0f N*m*s/rad");
        ImGui::DragFloat("Test 7 Upright Max Torque", &c.test7UprightMaxTorque,
                         5.0f, 0.0f, 500.0f, "%.0f N*m/axis");
        ImGui::DragFloat("Test 7 Heading Stiffness", &c.test7HeadingStiffness,
                         10.0f, 0.0f, 1000.0f, "%.0f N*m/rad");
        ImGui::DragFloat("Test 7 Heading Damping", &c.test7HeadingDamping,
                         2.5f, 0.0f, 250.0f, "%.0f N*m*s/rad");
        ImGui::DragFloat("Test 7 Heading Max Torque", &c.test7HeadingMaxTorque,
                         5.0f, 0.0f, 250.0f, "%.0f N*m");
        ImGui::DragFloat("Test 7 Inter-Step Tilt Gate", &c.test7InterStepTiltLimit,
                         0.5f, 12.0f, 25.0f, "%.1f deg");
        ImGui::DragFloat("Test 7 Inter-Step Heading Gate", &c.test7InterStepHeadingLimit,
                         0.5f, 2.0f, 20.0f, "%.1f deg");
        ImGui::DragInt("Test 7 Target Steps", &c.test7TargetSteps, 1.0f, 2, 100);
        ImGui::Checkbox("Test 7 60-Second Run", &c.test7EnduranceRun);
        ImGui::DragFloat("Test 7 Endurance Time", &c.test7EnduranceTime,
                         1.0f, 10.0f, 180.0f, "%.0f s");
        ImGui::DragFloat("Test 7 Stop Time", &c.test7StopTime,
                         0.05f, 0.25f, 3.0f, "%.2f s");
        ImGui::DragFloat("Test 7 Stop Hold", &c.test7StopHoldTime,
                         0.05f, 0.25f, 2.0f, "%.2f s");
        ImGui::TextDisabled("running=%s stop=%s t=%.1f steps=%d advanceCmd=%.3f m",
                            c._test7Running ? "yes" : "no",
                            c._test7StopRequested ? "requested" : "no",
                            c._test7RunTime, c._test6StepsCompleted,
                            c._test7CommandedStepLength);
        ImGui::TextDisabled("speed=%.3f m/s period=%.2f s foot=%.3f support=%.3f m",
                            c._test7MeasuredSpeed, c._test7LastStepPeriod,
                            c._test7LastStepLength,
                            c._test7AchievedSupportAdvance);
        ImGui::TextDisabled("max drift=%.3f m tilt=%.1f motorRatio=%.2f stopStable=%.2f",
                            c._test7MaxDrift, c._test7PeakTilt,
                            c._test7MaxMotorRatio, c._test7StopStableTime);
        ImGui::TextDisabled(
            "recenter=%.2f centerErr=%.3f m rootRate=(p=%+.2f,r=%+.2f,y=%+.2f) tilt=%.2f rad/s",
            c._test7InterStepRecenterT, c._test7InterStepCenterError,
            c._test7RootPitchRate, c._test7RootRollRate,
            c._test7RootYawRate, c._test7RootTiltRate);

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
    j["runtimeTurnSpeedDeg"] = c.runtimeTurnSpeedDeg;
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
    j["test4StepLength"] = c.test4StepLength;
    j["test4SwingHeight"] = c.test4SwingHeight;
    j["test4SwingTime"] = c.test4SwingTime;
    j["test4ArrivalHeight"] = c.test4ArrivalHeight;
    j["test4ArrivalTolerance"] = c.test4ArrivalTolerance;
    j["test4ArrivalSettleTime"] = c.test4ArrivalSettleTime;
    j["test4ArrivalTimeout"] = c.test4ArrivalTimeout;
    j["test4DescentTime"] = c.test4DescentTime;
    j["test4PlantTimeout"] = c.test4PlantTimeout;
    j["test4PlantAcquireTime"] = c.test4PlantAcquireTime;
    j["test4PlantAcquireMaxSpeed"] = c.test4PlantAcquireMaxSpeed;
    j["test4PlantAcquireTimeout"] = c.test4PlantAcquireTimeout;
    j["test4ContactSettleTime"] = c.test4ContactSettleTime;
    j["test4TouchdownMaxVerticalSpeed"] = c.test4TouchdownMaxVerticalSpeed;
    j["test4TouchdownMinNormalY"] = c.test4TouchdownMinNormalY;
    j["test4TargetTolerance"] = c.test4TargetTolerance;
    j["test4SafeReachFraction"] = c.test4SafeReachFraction;
    j["test5TransferTime"] = c.test5TransferTime;
    j["test5SupportFraction"] = c.test5SupportFraction;
    j["test5ComTolerance"] = c.test5ComTolerance;
    j["test5HoldTime"] = c.test5HoldTime;
    j["test5HoldTimeout"] = c.test5HoldTimeout;
    j["test6InterStepTime"] = c.test6InterStepTime;
    j["test6DriftGrowthTolerance"] = c.test6DriftGrowthTolerance;
    j["test7DesiredSpeed"] = c.test7DesiredSpeed;
    j["test7PlacementGain"] = c.test7PlacementGain;
    j["test7NominalAdvance"] = c.test7NominalAdvance;
    j["test7MinStepLength"] = c.test7MinStepLength;
    j["test7MaxStepLength"] = c.test7MaxStepLength;
    j["test7ReachCrouch"] = c.test7ReachCrouch;
    j["test7CrouchTime"] = c.test7CrouchTime;
    j["test7UsableReachFraction"] = c.test7UsableReachFraction;
    j["test7SoleLevelTime"] = c.test7SoleLevelTime;
    j["test7FootPositionGain"] = c.test7FootPositionGain;
    j["test7FootVelocityLeadTime"] = c.test7FootVelocityLeadTime;
    j["test7MaxFootCorrection"] = c.test7MaxFootCorrection;
    j["test7InterStepRecenterTime"] = c.test7InterStepRecenterTime;
    j["test7UprightStiffness"] = c.test7UprightStiffness;
    j["test7UprightDamping"] = c.test7UprightDamping;
    j["test7UprightMaxTorque"] = c.test7UprightMaxTorque;
    j["test7HeadingStiffness"] = c.test7HeadingStiffness;
    j["test7HeadingDamping"] = c.test7HeadingDamping;
    j["test7HeadingMaxTorque"] = c.test7HeadingMaxTorque;
    j["test7InterStepTiltLimit"] = c.test7InterStepTiltLimit;
    j["test7InterStepHeadingLimit"] = c.test7InterStepHeadingLimit;
    j["test7TargetSteps"] = c.test7TargetSteps;
    j["test7EnduranceRun"] = c.test7EnduranceRun;
    j["test7EnduranceTime"] = c.test7EnduranceTime;
    j["test7StopTime"] = c.test7StopTime;
    j["test7StopHoldTime"] = c.test7StopHoldTime;
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
    c.runtimeTurnSpeedDeg = j.value("runtimeTurnSpeedDeg", 540.0f);
    c.deadzone = j.value("deadzone", 0.2f);
    c.minWalkSpeed = j.value("minWalkSpeed", 0.15f);
    c.facingOffsetDeg = j.value("facingOffsetDeg", 90.0f);
    c.groundRayLength = j.value("groundRayLength", 1.2f);
    c.consciousness = j.value("consciousness", 1.0f);
    c.postPoweredGrace = j.value("postPoweredGrace", 0.4f);
    c.gaitBlendTime = j.value("gaitBlendTime", 0.6f);
    c.validationTest = j.value("validationTest", 0);
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
    c.test4StepLength = j.value("test4StepLength", 0.225f);
    c.test4SwingHeight = j.value("test4SwingHeight", 0.10f);
    c.test4SwingTime = j.value("test4SwingTime", 0.90f);
    c.test4ArrivalHeight = j.value("test4ArrivalHeight", 0.05f);
    c.test4ArrivalTolerance = j.value("test4ArrivalTolerance", 0.02f);
    c.test4ArrivalSettleTime = j.value("test4ArrivalSettleTime", 0.10f);
    c.test4ArrivalTimeout = j.value("test4ArrivalTimeout", 0.60f);
    c.test4DescentTime = j.value("test4DescentTime", 0.35f);
    c.test4PlantTimeout = j.value("test4PlantTimeout", 0.35f);
    c.test4PlantAcquireTime = j.value("test4PlantAcquireTime", 0.10f);
    c.test4PlantAcquireMaxSpeed = j.value("test4PlantAcquireMaxSpeed", 0.05f);
    c.test4PlantAcquireTimeout = j.value("test4PlantAcquireTimeout", 0.60f);
    c.test4ContactSettleTime = j.value("test4ContactSettleTime", 0.30f);
    c.test4TouchdownMaxVerticalSpeed = j.value(
        "test4TouchdownMaxVerticalSpeed", 0.25f);
    c.test4TouchdownMinNormalY = j.value("test4TouchdownMinNormalY", 0.70f);
    c.test4TargetTolerance = j.value("test4TargetTolerance", 0.04f);
    c.test4SafeReachFraction = j.value("test4SafeReachFraction", 0.99f);
    c.test5TransferTime = j.value("test5TransferTime", 1.00f);
    c.test5SupportFraction = j.value("test5SupportFraction", 0.92f);
    c.test5ComTolerance = j.value("test5ComTolerance", 0.04f);
    c.test5HoldTime = j.value("test5HoldTime", 0.50f);
    c.test5HoldTimeout = j.value("test5HoldTimeout", 1.50f);
    c.test6InterStepTime = j.value("test6InterStepTime", 0.25f);
    c.test6DriftGrowthTolerance = j.value(
        "test6DriftGrowthTolerance", 0.015f);
    c.test7DesiredSpeed = j.value("test7DesiredSpeed", 0.020f);
    c.test7PlacementGain = j.value("test7PlacementGain", 0.75f);
    c.test7NominalAdvance = j.value("test7NominalAdvance", 0.10f);
    c.test7MinStepLength = j.value("test7MinStepLength", 0.06f);
    c.test7MaxStepLength = j.value("test7MaxStepLength", 0.14f);
    c.test7ReachCrouch = j.value("test7ReachCrouch", 0.018f);
    c.test7CrouchTime = j.value("test7CrouchTime", 0.35f);
    c.test7UsableReachFraction = j.value(
        "test7UsableReachFraction", 0.975f);
    c.test7SoleLevelTime = j.value("test7SoleLevelTime", 0.35f);
    c.test7FootPositionGain = j.value("test7FootPositionGain", 0.45f);
    c.test7FootVelocityLeadTime = j.value(
        "test7FootVelocityLeadTime", 0.08f);
    c.test7MaxFootCorrection = j.value("test7MaxFootCorrection", 0.045f);
    c.test7InterStepRecenterTime = j.value(
        "test7InterStepRecenterTime", 0.35f);
    c.test7UprightStiffness = j.value(
        "test7UprightStiffness", 700.0f);
    c.test7UprightDamping = j.value(
        "test7UprightDamping", 100.0f);
    c.test7UprightMaxTorque = j.value(
        "test7UprightMaxTorque", 250.0f);
    c.test7HeadingStiffness = j.value(
        "test7HeadingStiffness", 180.0f);
    c.test7HeadingDamping = j.value(
        "test7HeadingDamping", 50.0f);
    c.test7HeadingMaxTorque = j.value(
        "test7HeadingMaxTorque", 80.0f);
    c.test7InterStepTiltLimit = j.value(
        "test7InterStepTiltLimit", 15.0f);
    c.test7InterStepHeadingLimit = j.value(
        "test7InterStepHeadingLimit", 8.0f);
    c.test7TargetSteps = j.value("test7TargetSteps", 10);
    c.test7EnduranceRun = j.value("test7EnduranceRun", false);
    c.test7EnduranceTime = j.value("test7EnduranceTime", 60.0f);
    c.test7StopTime = j.value("test7StopTime", 1.0f);
    c.test7StopHoldTime = j.value("test7StopHoldTime", 0.50f);
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
                rag.locomotionTorqueUpright = false;
                rag.locomotionSimbicon = false;
                rag.locomotionSimbiconBlend = 0.0f;
                rag.locomotionHipTorque[0] = rag.locomotionHipTorque[1] = glm::vec3(0.0f);
                rag.locomotionHipBones[0] = rag.locomotionHipBones[1] = -1;
                rag.locomotionLiftBone = -1;
                rag.locomotionFootLockBones[0] = rag.locomotionFootLockBones[1] = -1;
                rag.locomotionFootLockWeights[0] = rag.locomotionFootLockWeights[1] = 0.0f;
                rag.locomotionHeightOffset = 0.0f;
                comp._continuousGaitEnabled = false;
                ResetGait(comp);
                ResetTest1(comp);
                ResetTest2(comp);
                ResetTest3(comp);
                ResetTest4(comp);
                ResetGroundTest(comp);
                comp._validationActiveTest = -1;
                continue;
            }

            // A scene can author the ragdoll as Powered, in which case the Animated->Powered
            // flip never runs and the -1 sentinel would double the grace period.
            if (comp._timeSincePowered < 0.0f) comp._timeSincePowered = 0.0f;
            comp._timeSincePowered += dt;
            rag.strength = glm::clamp(comp.consciousness, 0.0f, 1.0f);

            comp.validationTest = glm::clamp(comp.validationTest, 0, 8);
            if (comp.validationTest != comp._validationActiveTest) {
                ResetGait(comp);
                ResetTest1(comp);
                ResetTest2(comp);
                ResetTest3(comp);
                ResetTest4(comp);
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
            // input cannot enable a gait controller during a foundation measurement. Runtime
            // uses a lower stop threshold than start threshold so analog noise cannot chatter
            // the controller between walking and its controlled stop.
            if (comp.validationTest == 0) {
                const float startSpeed = glm::max(comp.minWalkSpeed, 0.0f);
                const float stopSpeed = startSpeed * 0.5f;
                comp._runtimeWalkIntent = comp._runtimeWalkIntent
                    ? speed > stopSpeed : speed > startSpeed;
            } else {
                comp._runtimeWalkIntent = false;
            }
            const bool wantsToWalk = comp.validationTest == 0
                && comp._runtimeWalkIntent;
            if (!wantsToWalk)
                comp._runtimeRestartBlocked = false;

            const Diamond::Locomotion::GaitCommand validationGaitCommand =
                BuildValidationGaitCommand(comp);
            const Diamond::Locomotion::GaitCommand runtimeGaitCommand =
                BuildRuntimeGaitCommand(comp, moveDir, speed, wantsToWalk);
            const Diamond::Locomotion::GaitCommand& continuousGaitCommand =
                runtimeGaitCommand.enabled
                    ? runtimeGaitCommand : validationGaitCommand;
            comp._continuousGaitEnabled = continuousGaitCommand.enabled;

            if (wantsToWalk && !continuousGaitCommand.enabled) {
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
                    comp._continuousGaitEnabled
                        || (comp.validationTest >= 4 && comp.validationTest <= 7)
                        ? comp._test4Phase : comp._test3Phase,
                    (comp._continuousGaitEnabled
                        || (comp.validationTest >= 4 && comp.validationTest <= 7)
                        ? comp._test4BaselineValid
                                              : comp._test3BaselineValid)
                        ? "ready" : "waiting",
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

            if (!continuousGaitCommand.enabled
                && wantsToWalk && !comp._wasWalking) {
                comp._stateIndex = 0;
                comp._stateTime = 0.0f;
                comp._steps = 0;
                comp._swingWasAirborne = false;
                comp._airborneTime = 0.0f;
            }
            if (!continuousGaitCommand.enabled
                && wantsToWalk != comp._wasWalking) {
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
            const bool torqueGait = gaitRequested
                && !continuousGaitCommand.enabled && !comp.assistedStepping;
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
            rag.locomotionTorqueUpright = continuousGaitCommand.enabled;
            if (rag.locomotionTorqueUpright) {
                rag.locomotionTorqueUprightStiffness = glm::max(
                    comp.test7UprightStiffness, 0.0f);
                rag.locomotionTorqueUprightDamping = glm::max(
                    comp.test7UprightDamping, 0.0f);
                rag.locomotionTorqueUprightMaxTorque = glm::max(
                    comp.test7UprightMaxTorque, 0.0f);
                rag.locomotionTorqueHeadingStiffness = glm::max(
                    comp.test7HeadingStiffness, 0.0f);
                rag.locomotionTorqueHeadingDamping = glm::max(
                    comp.test7HeadingDamping, 0.0f);
                rag.locomotionTorqueHeadingMaxTorque = glm::max(
                    comp.test7HeadingMaxTorque, 0.0f);
            }
            rag.locomotionLiftBone = -1;
            for (int& bone : rag.locomotionDisabledMotorBones) bone = -1;
            rag.locomotionFootLockBones[0] = rag.locomotionFootLockBones[1] = -1;
            rag.locomotionFootLockWeights[0] = rag.locomotionFootLockWeights[1] = 0.0f;
            rag.locomotionFootLockFrequency = glm::max(comp.proceduralFootLockFrequency, 0.0f);
            rag.locomotionFootLockEffectiveMass = glm::max(
                comp.proceduralFootLockEffectiveMass, 0.0f);
            rag.locomotionFootLockMaxForce = glm::max(comp.proceduralFootLockMaxForce, 0.0f);

            if (continuousGaitCommand.enabled) {
                // Both Test 7 and gameplay use the same physical gait core. This standing
                // pass resolves the rig and supplies the validated bind pose; the core owns
                // swing IK, support transfer, heading, and stopping while active.
                UpdateProceduralGait(scene, entity, comp, rag, false, dt);
                UpdateTest4(scene, entity, comp, rag, ready, tiltDeg, dt,
                            continuousGaitCommand);
            } else if (comp.validationTest > 0) {
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
                } else if (comp.validationTest >= 4 && comp.validationTest <= 6) {
                    UpdateTest4(scene, entity, comp, rag, ready, tiltDeg, dt,
                                continuousGaitCommand);
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
                    spdlog::info("[LocoFrame] tilt(mine={:.1f} engine={:.1f} delta={:+.1f}) entityRight=({:+.2f},{:+.2f},{:+.2f}) entityFwd=({:+.2f},{:+.2f},{:+.2f}) gaitRight=({:+.2f},{:+.2f},{:+.2f}) gaitFwd=({:+.2f},{:+.2f},{:+.2f}) femurCmd=({:+.2f},{:+.2f},{:+.2f}) femurActual=({:+.2f},{:+.2f},{:+.2f}) femurErr={:.1f}",
                                 comp._myTiltDeg, comp._engineTiltDeg,
                                 comp._myTiltDeg - comp._engineTiltDeg,
                                 comp._right.x, comp._right.y, comp._right.z,
                                 comp._fwd.x, comp._fwd.y, comp._fwd.z,
                                 comp._test4Right.x, comp._test4Right.y,
                                 comp._test4Right.z,
                                 comp._test4Forward.x, comp._test4Forward.y,
                                 comp._test4Forward.z,
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
                    } else if (comp.validationTest >= 4 && comp.validationTest <= 7) {
                        const int swingBone = comp._test4SupportSide < 0
                            ? comp._legR.footIdx : comp._legL.footIdx;
                        bool swingPositionOk = false;
                        const glm::vec3 swingPosition = Physics::GetRagdollBonePosition(
                            rag, swingBone, &swingPositionOk);
                        spdlog::info(
                            "[LocoTest{}] t={:.2f} baseline={} phase={} phaseT={:.2f} support={} "
                            "com={:+.3f}/{:+.3f} contact=({},{}) air={:.2f} path={:.2f} "
                            "clear={:.3f} forward={:.3f} desired=({:+.3f},{:+.3f},{:+.3f}) "
                            "foot=({:+.3f},{:+.3f},{:+.3f}) target=({:+.3f},{:+.3f},{:+.3f}) "
                            "err={:.3f} target=(h={:.3f},fwd={:+.3f},lat={:+.3f},y={:.3f}) "
                            "touchdown={} vy=(api={:+.3f},fd={:+.3f}) normalY={:.2f} upY={:+.2f} "
                            "contactLocal=({:+.3f},{:+.3f},{:+.3f}) "
                            "drift=({:.3f}/{:.3f},{:.3f}/{:.3f}) tilt=({:.1f}/{:.1f}/{:.1f}) "
                            "supportForce={:.0f} supportSat={} lift={:.0f}/{:.0f} "
                            "locks=({:.2f},{:.2f}) lockF=({:.0f},{:.0f}) motorRatio={:.2f} motorSat={} "
                            "transfer=(t={:.2f},hold={:.2f},err={:.3f},old={:.3f},new={:.3f},speed={:.3f}) "
                            "sequence=(step={},done={},inter={:.2f},edges=({},{}))",
                            comp.validationTest,
                            comp._test4Time,
                            comp._test4BaselineValid ? "READY" : "wait",
                            comp._test4Phase, comp._test4PhaseTime,
                            comp._test4SupportSide,
                            comp._test4ComLateral, comp._test4TargetLateral,
                            comp._test4ContactL ? "L" : "-",
                            comp._test4ContactR ? "R" : "-",
                            comp._test4AirborneTime, comp._test4TrajectoryT,
                            comp._test4Clearance, comp._test4ForwardTravel,
                            comp._test4DesiredFoot.x, comp._test4DesiredFoot.y,
                            comp._test4DesiredFoot.z,
                            swingPositionOk ? swingPosition.x : 0.0f,
                            swingPositionOk ? swingPosition.y : 0.0f,
                            swingPositionOk ? swingPosition.z : 0.0f,
                            comp._test4Foothold.x, comp._test4Foothold.y,
                            comp._test4Foothold.z,
                            comp._test4TargetError,
                            comp._test4HorizontalTargetError,
                            comp._test4ForwardTargetError,
                            comp._test4LateralTargetError,
                            comp._test4VerticalTargetError,
                            comp._test4TouchdownAccepted ? "accepted" : "waiting",
                            comp._test4ApiVelocity.y,
                            comp._test4MeasuredVelocity.y,
                            comp._test4TouchdownNormalY, comp._test4FootUpY,
                            comp._test4ContactLocal.x,
                            comp._test4ContactLocal.y,
                            comp._test4ContactLocal.z,
                            comp._test4StanceDrift, comp._test4MaxStanceDrift,
                            comp._test4PlantDrift, comp._test4MaxPlantDrift,
                            comp._test4InitialTilt, comp._test4PeakTilt,
                            comp._test4FinalTilt,
                            glm::length(rag._locomotionSupportForce),
                            rag._locomotionSupportSaturated ? "YES" : "no",
                            rag._locomotionLiftForce, rag.locomotionLiftMaxForce,
                            rag.locomotionFootLockWeights[0],
                            rag.locomotionFootLockWeights[1],
                            rag._locomotionFootLockForce[0],
                            rag._locomotionFootLockForce[1],
                            comp._test4MaxMotorRatio,
                            comp._test4MotorSaturated ? "YES" : "no",
                            comp._test5TransferT, comp._test5HoldStableTime,
                            comp._test5ComError, comp._test5ComToOldSupport,
                            comp._test5ComToNewSupport,
                            comp._test5ComHorizontalSpeed,
                            comp._test6StepIndex, comp._test6StepsCompleted,
                            comp._test6InterStepStableTime,
                            comp._test6ContactTransitionsL,
                            comp._test6ContactTransitionsR);
                        if (comp.validationTest == 7) {
                            spdlog::info(
                                "[LocoTest7] STATUS running={} stop={} time={:.2f}s "
                                "steps={} advanceCommand={:.3f}m "
                                "measuredSpeed={:.3f}/{:.3f}m/s "
                                "period={:.2f}s footTravel={:.3f}m "
                                 "supportAdvance=({:.3f},{:.3f})m maxDrift={:.3f} "
                                 "peakTilt={:.1f} maxMotorRatio={:.2f} "
                                 "crouch=(blend={:.2f},height={:+.3f}m) "
                                 "recenter=(t={:.2f},centerErr={:.3f}m) "
                                 "angular=(tiltRate={:.3f}rad/s,yawRate={:+.3f}rad/s) "
                                 "heading=(error={:+.1f}deg,peak={:.1f}deg)",
                                comp._test7Running ? "yes" : "no",
                                comp._test7StopRequested ? "yes" : "no",
                                comp._test7RunTime, comp._test6StepsCompleted,
                                comp._test7CommandedStepLength,
                                comp._test7MeasuredSpeed, comp.test7DesiredSpeed,
                                comp._test7LastStepPeriod,
                                comp._test7LastStepLength,
                                comp._test7PlannedSupportAdvance,
                                comp._test7AchievedSupportAdvance,
                                comp._test7MaxDrift, comp._test7PeakTilt,
                                 comp._test7MaxMotorRatio,
                                 comp._test7CrouchBlend,
                                 rag.locomotionHeightOffset,
                                 comp._test7InterStepRecenterT,
                                 comp._test7InterStepCenterError,
                                 comp._test7RootTiltRate,
                                 comp._test7RootYawRate,
                                 comp._test7HeadingErrorDeg,
                                 comp._test7PeakHeadingErrorDeg);
                            spdlog::info(
                                "[LocoTest7] CONTROL_STATUS phase={} "
                                "rootW=(pitch={:+.3f},roll={:+.3f},yaw={:+.3f},"
                                "tilt={:.3f})rad/s "
                                "upright=(mode={},active={},dw=(pitch={:+.3f},roll={:+.3f},"
                                "yaw={:+.3f})rad/s,motorTorque=(pitch={:+.1f},"
                                "roll={:+.1f},yaw={:+.1f},total={:.1f},"
                                "tiltCap={:.1f},yawCap={:.1f})Nm,sat={}) "
                                "heading=(error={:+.1f}deg,sat={}) "
                                "supportTargetVel=(lat={:+.3f},fwd={:+.3f})m/s "
                                "supportForce=(P=({:+.0f},{:+.0f}),D=({:+.0f},{:+.0f}),"
                                "total={:.0f})N supportSat={}",
                                comp._test4Phase,
                                comp._test7RootPitchRate,
                                comp._test7RootRollRate,
                                comp._test7RootYawRate,
                                comp._test7RootTiltRate,
                                rag.locomotionTorqueUpright
                                    ? "SOLVER_SPRING" : "VELOCITY",
                                rag._locomotionUprightTorqueActive ? "YES" : "no",
                                glm::dot(rag._locomotionUprightDeltaAngularVelocity,
                                         comp._test4Right),
                                glm::dot(rag._locomotionUprightDeltaAngularVelocity,
                                         comp._test4Forward),
                                rag._locomotionUprightDeltaAngularVelocity.y,
                                glm::dot(rag._locomotionUprightTorque,
                                         comp._test4Right),
                                glm::dot(rag._locomotionUprightTorque,
                                         comp._test4Forward),
                                rag._locomotionUprightTorque.y,
                                glm::length(rag._locomotionUprightTorque),
                                rag.locomotionTorqueUprightMaxTorque,
                                rag.locomotionTorqueHeadingMaxTorque,
                                rag._locomotionUprightSaturated ? "YES" : "no",
                                rag._locomotionHeadingErrorDeg,
                                rag._locomotionHeadingSaturated ? "YES" : "no",
                                glm::dot(rag.locomotionSupportTargetVel,
                                         comp._test4Right),
                                glm::dot(rag.locomotionSupportTargetVel,
                                         comp._test4Forward),
                                glm::dot(rag._locomotionSupportPositionForce,
                                         comp._test4Right),
                                glm::dot(rag._locomotionSupportPositionForce,
                                         comp._test4Forward),
                                glm::dot(rag._locomotionSupportDampingForce,
                                         comp._test4Right),
                                glm::dot(rag._locomotionSupportDampingForce,
                                         comp._test4Forward),
                                glm::length(rag._locomotionSupportForce),
                                rag._locomotionSupportSaturated ? "YES" : "no");
                            if (comp._test4Phase >= 2 && comp._test4Phase <= 7
                                && comp._test7IkPlanHipValid) {
                                spdlog::info(
                                    "[LocoTest7] IK_STATUS phase={} "
                                    "reach=(requested={:.3f},clamped={:.3f},"
                                    "max={:.3f},physical={:.3f}) "
                                    "reachShortfall=({:.3f}m,fwd={:+.3f}m) "
                                    "hipMove=(fwd={:+.3f},lat={:+.3f},y={:+.3f})m "
                                    "hipEnvelopeClamp={:.1f}deg "
                                    "commandLag=(hip={:.1f},knee={:.1f})deg "
                                    "kneeBend={:.1f}deg footError=(h={:.3f},fwd={:+.3f})m "
                                    "footControl=(corr={:.3f}m,fwd={:+.3f}m,"
                                    "targetSpeed={:.3f}m/s,level={:.2f})",
                                    comp._test4Phase,
                                    comp._test7IkRequestedReach,
                                    comp._test7IkClampedReach,
                                    comp._test7IkMaxReach,
                                    comp._test7IkPhysicalReach,
                                    comp._test7IkReachShortfall,
                                    comp._test7IkReachShortfallForward,
                                    comp._test7IkHipTravelForward,
                                    comp._test7IkHipTravelLateral,
                                    comp._test7IkHipTravelVertical,
                                    comp._test7IkHipEnvelopeClampDeg,
                                    comp._test7IkHipCommandLagDeg,
                                    comp._test7IkKneeCommandLagDeg,
                                    comp._test7IkKneeBendDeg,
                                    comp._test4HorizontalTargetError,
                                    comp._test4ForwardTargetError,
                                    comp._test7FootCorrection,
                                    comp._test7FootCorrectionForward,
                                    comp._test7FootTargetSpeed,
                                    comp._test7SoleLevelBlend);
                            }
                        }
                        LogKneeDiagnostics(scene, entity, comp, rag);
                    } else if (comp.validationTest == 8) {
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

    static Diamond::Locomotion::GaitCommand BuildValidationGaitCommand(const Comp& comp)
    {
        using Diamond::Locomotion::GaitCommand;
        using Diamond::Locomotion::RunLimit;

        GaitCommand command;
        command.enabled = comp.validationTest == 7;
        if (!command.enabled) return command;

        const bool startLeftSupport = Input::IsKeyPressed(Key::F6);
        const bool startRightSupport = Input::IsKeyPressed(Key::F7);
        command.startRequested = startLeftSupport || startRightSupport;
        command.initialSupportSide = startLeftSupport ? -1
            : (startRightSupport ? 1 : 0);
        command.stopRequested = Input::IsKeyPressed(Key::F8);
        command.resetRequested = command.stopRequested;
        command.desiredSpeed = comp.test7DesiredSpeed;
        command.runLimit = comp.test7EnduranceRun
            ? RunLimit::Duration : RunLimit::StepCount;
        command.stepLimit = comp.test7TargetSteps;
        command.durationLimit = comp.test7EnduranceTime;
        return command;
    }

    static Diamond::Locomotion::GaitCommand BuildRuntimeGaitCommand(
        const Comp& comp, const glm::vec3& moveDirection,
        float inputSpeed, bool wantsToWalk)
    {
        using Diamond::Locomotion::GaitCommand;

        GaitCommand command;
        command.enabled = comp.validationTest == 0;
        if (!command.enabled) return command;

        const bool directionChanged = wantsToWalk && comp._test7Running
            && glm::dot(moveDirection, moveDirection) > 1e-8f
            && glm::dot(comp._runtimeDesiredForward,
                        comp._runtimeDesiredForward) > 1e-8f
            && glm::dot(glm::normalize(moveDirection),
                        glm::normalize(comp._runtimeDesiredForward))
                < std::cos(glm::radians(5.0f));
        command.startRequested = wantsToWalk && !comp._test7Running
            && !comp._runtimeRestartBlocked;
        command.stopRequested = comp._test7Running
            && (!wantsToWalk || directionChanged);
        command.initialSupportSide = comp._runtimeNextSupportSide;
        command.desiredForward = moveDirection;

        // Keep Step 2 inside Test 7's validated speed envelope. Input magnitude is still
        // represented, while the later responsiveness pass can raise cadence and stride
        // together instead of asking this low-speed gait to jump directly to maxSpeed.
        const float inputFraction = comp.maxSpeed > 1e-4f
            ? glm::clamp(inputSpeed / comp.maxSpeed, 0.0f, 1.0f) : 0.0f;
        command.desiredSpeed = glm::max(comp.test7DesiredSpeed, 0.0f)
            * inputFraction;
        return command;
    }

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
        c._runtimeWalkIntent = false;
        c._runtimeRestartBlocked = false;
        c._runtimeTurnActive = false;
        c._runtimeNextSupportSide = -1;
        c._runtimeTurnElapsed = 0.0f;
        c._runtimeTurnDuration = 0.0f;
        c._runtimeTurnTotalYaw = 0.0f;
        c._runtimeTurnAppliedYaw = 0.0f;
        c._runtimeTurnTargetForward = glm::vec3(0.0f, 0.0f, -1.0f);
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

    static void ResetTest4(Comp& c)
    {
        c._test4BaselineValid = false;
        c._test4ContactL = c._test4ContactR = false;
        c._test4PrevSwingContact = false;
        c._test4TouchdownAccepted = false;
        c._test4Aborted = false;
        c._test4Time = 0.0f;
        c._test4SettleTime = 0.0f;
        c._test4Phase = 0;
        c._test4PhaseTime = 0.0f;
        c._test4SupportSide = 0;
        c._test4ComCommand = 0.0f;
        c._test4ComLateral = c._test4TargetLateral = 0.0f;
        c._test4AirborneTime = 0.0f;
        c._test4ArrivalStableTime = 0.0f;
        c._test4ReachLimit = 0.0f;
        c._test4PlantAcquireStableTime = 0.0f;
        c._test4TrajectoryT = 0.0f;
        c._test4Clearance = 0.0f;
        c._test4ForwardTravel = 0.0f;
        c._test4TargetError = 0.0f;
        c._test4HorizontalTargetError = 0.0f;
        c._test4ForwardTargetError = 0.0f;
        c._test4LateralTargetError = 0.0f;
        c._test4VerticalTargetError = 0.0f;
        c._test4TouchdownVy = 0.0f;
        c._test4TouchdownNormalY = 0.0f;
        c._test4StanceDrift = c._test4PlantDrift = 0.0f;
        c._test4MaxStanceDrift = c._test4MaxPlantDrift = 0.0f;
        c._test4InitialTilt = c._test4PeakTilt = c._test4FinalTilt = 0.0f;
        c._test4MaxMotorRatio = 0.0f;
        c._test4MotorSaturated = false;
        c._test4PlantPoseCaptured = false;
        c._test4FootBaselineL = c._test4FootBaselineR = glm::vec3(0.0f);
        c._test4ComBaseline = glm::vec3(0.0f);
        c._test4Right = glm::vec3(1.0f, 0.0f, 0.0f);
        c._test4Forward = glm::vec3(0.0f, 0.0f, -1.0f);
        c._test4SupportTarget = glm::vec3(0.0f);
        c._test4SwingStart = glm::vec3(0.0f);
        c._test4ArcStart = glm::vec3(0.0f);
        c._test4Foothold = glm::vec3(0.0f);
        c._test4DesiredFoot = glm::vec3(0.0f);
        c._test4TouchdownPlant = glm::vec3(0.0f);
        c._test4ApiVelocity = glm::vec3(0.0f);
        c._test4MeasuredVelocity = glm::vec3(0.0f);
        c._test4PreviousSwingFoot = glm::vec3(0.0f);
        c._test4PreviousSwingFootValid = false;
        c._test4FootUpY = 1.0f;
        c._test4ContactPoint = glm::vec3(0.0f);
        c._test4ContactLocal = glm::vec3(0.0f);
        c._test5TransferT = 0.0f;
        c._test5HoldStableTime = 0.0f;
        c._test5ContactLossTime = 0.0f;
        c._test5ComError = 0.0f;
        c._test5ComToOldSupport = 0.0f;
        c._test5ComToNewSupport = 0.0f;
        c._test5ComHorizontalSpeed = 0.0f;
        c._test5TransferStartTarget = glm::vec3(0.0f);
        c._test5TransferEndTarget = glm::vec3(0.0f);
        c._test6StepIndex = 0;
        c._test6StepsCompleted = 0;
        c._test6InterStepStableTime = 0.0f;
        c._test6InitialTilt = 0.0f;
        for (int i = 0; i < 2; ++i) {
            c._test6StepForward[i] = 0.0f;
            c._test6StepMaxDrift[i] = 0.0f;
            c._test6StepPeakTilt[i] = 0.0f;
            c._test6StepMotorRatio[i] = 0.0f;
        }
        c._test6PreviousContactsValid = false;
        c._test6PreviousContactL = false;
        c._test6PreviousContactR = false;
        c._test6ContactTransitionsL = 0;
        c._test6ContactTransitionsR = 0;
        c._test7ContactChangeTimeL = 0.0f;
        c._test7ContactChangeTimeR = 0.0f;
        c._test7Running = false;
        c._test7StopRequested = false;
        c._test7RunTime = 0.0f;
        c._test7StepStartTime = 0.0f;
        c._test7LastStepPeriod = 0.0f;
        c._test7PreviousStepPeriod = 0.0f;
        c._test7MeasuredSpeed = 0.0f;
        const float test7MinimumAdvance = glm::min(
            c.test7MinStepLength, c.test7MaxStepLength);
        const float test7MaximumAdvance = glm::max(
            c.test7MinStepLength, c.test7MaxStepLength);
        const float test7TrackingReserve = glm::min(glm::max(
            c.test4TargetTolerance * 0.5f, 0.010f), 0.025f);
        c._test7CommandedStepLength = glm::clamp(
            c.test7NominalAdvance,
            glm::min(test7MaximumAdvance,
                     test7MinimumAdvance + test7TrackingReserve),
            test7MaximumAdvance);
        c._test7ReachCommandCeiling = test7MaximumAdvance;
        c._test7TakeoffContactRecoveryTime = 0.0f;
        c._test7SwingRecontactTime = 0.0f;
        c._test7SettledTrackingLoss = 0.0f;
        c._test7ForwardPreShift = 0.0f;
        c._test7ReachClearSteps = 0;
        c._test7PlannedSupportAdvance = 0.0f;
        c._test7AchievedSupportAdvance = 0.0f;
        c._test7LastStepLength = 0.0f;
        c._test7PreviousStepLength = 0.0f;
        c._test7LastSupportAdvance = 0.0f;
        c._test7PreviousSupportAdvance = 0.0f;
        c._test7StepMaxRelevantDrift = 0.0f;
        c._test7MaxDrift = 0.0f;
        c._test7PeakTilt = 0.0f;
        c._test7MaxMotorRatio = 0.0f;
        c._test7StopStableTime = 0.0f;
        c._test7StopFootDriftL = 0.0f;
        c._test7StopFootDriftR = 0.0f;
        c._test7StopMaxFootDrift = 0.0f;
        c._test7StopSettleFootDriftL = 0.0f;
        c._test7StopSettleFootDriftR = 0.0f;
        c._test7StopMaxSettleFootDrift = 0.0f;
        c._test7CrouchBlend = 0.0f;
        c._test7IkRequestedReach = 0.0f;
        c._test7IkClampedReach = 0.0f;
        c._test7IkMaxReach = 0.0f;
        c._test7IkPhysicalReach = 0.0f;
        c._test7IkReachShortfall = 0.0f;
        c._test7IkReachShortfallForward = 0.0f;
        c._test7IkHipEnvelopeClampDeg = 0.0f;
        c._test7IkHipCommandLagDeg = 0.0f;
        c._test7IkKneeCommandLagDeg = 0.0f;
        c._test7IkKneeBendDeg = 0.0f;
        c._test7IkHipTravelForward = 0.0f;
        c._test7IkHipTravelLateral = 0.0f;
        c._test7IkHipTravelVertical = 0.0f;
        c._test7FootCorrection = 0.0f;
        c._test7FootCorrectionForward = 0.0f;
        c._test7FootTargetSpeed = 0.0f;
        c._test7SoleLevelBlend = 0.0f;
        c._test7SoleAngularErrorDeg = 0.0f;
        c._test7InterStepRecenterT = 0.0f;
        c._test7InterStepCenterError = 0.0f;
        c._test7RootPitchRate = 0.0f;
        c._test7RootRollRate = 0.0f;
        c._test7RootYawRate = 0.0f;
        c._test7RootTiltRate = 0.0f;
        c._test7HeadingErrorDeg = 0.0f;
        c._test7PeakHeadingErrorDeg = 0.0f;
        c._test7TakeoffContactRecoveryActive = false;
        c._test7IkPlanHipValid = false;
        c._test7ReachClampedStep = false;
        c._test7OldSupportDriftAllowanceLogged = false;
        c._test7StopSettleReferenceValid = false;
        c._runtimeTurnActive = false;
        c._runtimeTurnElapsed = 0.0f;
        c._runtimeTurnDuration = 0.0f;
        c._runtimeTurnTotalYaw = 0.0f;
        c._runtimeTurnAppliedYaw = 0.0f;
        c._runtimeDesiredForward = glm::vec3(0.0f, 0.0f, -1.0f);
        c._runtimeTurnTargetForward = glm::vec3(0.0f, 0.0f, -1.0f);
        c._test7StartCom = glm::vec3(0.0f);
        c._test7StepStartCom = glm::vec3(0.0f);
        c._test7IkPlanHip = glm::vec3(0.0f);
        c._test7InterStepRecenterStart = glm::vec3(0.0f);
        c._test7InterStepRecenterTarget = glm::vec3(0.0f);
        c._test7StopStartTarget = glm::vec3(0.0f);
        c._test7StopEndTarget = glm::vec3(0.0f);
        c._test7StopFootTargetL = glm::vec3(0.0f);
        c._test7StopFootTargetR = glm::vec3(0.0f);
        c._test7StopSettleFootTargetL = glm::vec3(0.0f);
        c._test7StopSettleFootTargetR = glm::vec3(0.0f);
        c._test7HeadingTargetRot = glm::quat(1.0f, 0.0f, 0.0f, 0.0f);
        c._legL.groundReferenceFootRotationValid = false;
        c._legR.groundReferenceFootRotationValid = false;
        c._legL.groundReferenceKneePoleValid = false;
        c._legR.groundReferenceKneePoleValid = false;
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

    static bool FootContact(const RagdollComponent& rag, int footBone,
                            glm::vec3* normal = nullptr, glm::vec3* point = nullptr)
    {
        for (int i = 0; i < 2; ++i) {
            if (rag._locomotionFootBones[i] != footBone) continue;
            if (normal) *normal = rag._locomotionFootContactNormal[i];
            if (point) *point = rag._locomotionFootContactPoint[i];
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
            if (footRotationOk) {
                swing->plantedFootWorldRotation = footWorld;
                swing->ankleFromFootLocal = glm::conjugate(footWorld)
                    * (ankle - swingFoot);
            } else {
                swing->plantedFootWorldRotation = OrientationOf(BoneWorldMatrix(
                    skeleton, animator,
                    scene.GetTransformSystem().GetWorldMatrix(entity),
                    swing->footIdx));
                swing->ankleFromFootLocal =
                    glm::conjugate(swing->plantedFootWorldRotation)
                    * (ankle - swingFoot);
            }
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

    static void UpdateTest4(
        Scene& scene, entt::entity entity, Comp& comp, RagdollComponent& rag,
        bool ready, float tiltDeg, float dt,
        const Diamond::Locomotion::GaitCommand& continuousCommand)
    {
        constexpr int kIdle = 0;
        constexpr int kWeightShift = 1;
        constexpr int kTakeoff = 2;
        constexpr int kSwing = 3;
        constexpr int kArrival = 4;
        constexpr int kDescent = 5;
        constexpr int kTouchdownWait = 6;
        constexpr int kSettle = 7;
        constexpr int kTransfer = 8;
        constexpr int kHold = 9;
        constexpr int kInterStep = 10;
        constexpr int kComplete = 11;
        constexpr int kAbort = 12;
        constexpr int kStopping = 13;
        constexpr int kReturnStand = 14;
        const bool transferEnabled = comp.validationTest >= 5
            || continuousCommand.enabled;
        const bool twoStepEnabled = comp.validationTest == 6;
        const bool continuousEnabled = continuousCommand.enabled;
        const bool multiStepEnabled = twoStepEnabled || continuousEnabled;
        const bool gameplayCommand = continuousEnabled
            && continuousCommand.runLimit == Diamond::Locomotion::RunLimit::None;

        if (!scene.Has<SkinnedMeshComponent>(entity)
            || !scene.Has<AnimatorComponent>(entity)
            || !ValidLeg(comp._legL) || !ValidLeg(comp._legR)
            || !rag._locomotionCOMValid) return;

        auto& mesh = scene.Get<SkinnedMeshComponent>(entity);
        auto& animator = scene.Get<AnimatorComponent>(entity);
        const auto& skeleton = mesh.skeleton;
        const int count = static_cast<int>(skeleton.bones.size());
        if (count == 0 || static_cast<int>(animator.pose.size()) != count) return;

        const glm::mat4 entityWorld = scene.GetTransformSystem().GetWorldMatrix(entity);
        auto physicalPosition = [&](int bone, bool* okOut = nullptr) {
            bool ok = false;
            const glm::vec3 position = Physics::GetRagdollBonePosition(rag, bone, &ok);
            if (okOut) *okOut = ok;
            return position;
        };
        auto smoothstep = [](float t) {
            t = glm::clamp(t, 0.0f, 1.0f);
            return t * t * (3.0f - 2.0f * t);
        };

        bool leftPositionOk = false, rightPositionOk = false;
        const glm::vec3 leftFoot = physicalPosition(comp._legL.footIdx, &leftPositionOk);
        const glm::vec3 rightFoot = physicalPosition(comp._legR.footIdx, &rightPositionOk);
        if (!leftPositionOk || !rightPositionOk) return;

        comp._test4ContactL = FootContact(rag, comp._legL.footIdx);
        comp._test4ContactR = FootContact(rag, comp._legR.footIdx);
        if (multiStepEnabled && comp._test6PreviousContactsValid
            && comp._test4Phase > kIdle) {
            if (continuousEnabled) {
                // Test 7 validates gait events, not short-lived sole-manifold chatter.
                // The observed takeoff recovery contacts can persist for 50-180 ms, so
                // require a state to survive beyond that recovery window before counting it.
                constexpr float kTest7ContactDebounceTime = 0.20f;
                auto updateDebouncedContact = [&](bool rawContact,
                                                   bool& debouncedContact,
                                                   float& changeTime,
                                                   int& transitionCount,
                                                   const char* footName) {
                    if (rawContact == debouncedContact) {
                        changeTime = 0.0f;
                        return;
                    }
                    changeTime += dt;
                    if (changeTime < kTest7ContactDebounceTime) return;
                    debouncedContact = rawContact;
                    changeTime = 0.0f;
                    ++transitionCount;
                    spdlog::info(
                        "[LocoTest7] CONTACT_EDGE foot={} state={} "
                        "debounce={:.3f}s transitions=({},{})",
                        footName, rawContact ? "CONTACT" : "AIRBORNE",
                        kTest7ContactDebounceTime,
                        comp._test6ContactTransitionsL,
                        comp._test6ContactTransitionsR);
                };
                updateDebouncedContact(
                    comp._test4ContactL, comp._test6PreviousContactL,
                    comp._test7ContactChangeTimeL,
                    comp._test6ContactTransitionsL, "LEFT");
                updateDebouncedContact(
                    comp._test4ContactR, comp._test6PreviousContactR,
                    comp._test7ContactChangeTimeR,
                    comp._test6ContactTransitionsR, "RIGHT");
            } else {
                if (comp._test4ContactL != comp._test6PreviousContactL)
                    ++comp._test6ContactTransitionsL;
                if (comp._test4ContactR != comp._test6PreviousContactR)
                    ++comp._test6ContactTransitionsR;
                comp._test6PreviousContactL = comp._test4ContactL;
                comp._test6PreviousContactR = comp._test4ContactR;
            }
        }
        if (!ready) {
            comp._test4Time = 0.0f;
            comp._test4SettleTime = 0.0f;
            return;
        }
        comp._test4Time += dt;

        const glm::vec3 leftVelocity = Physics::GetRagdollBoneLinearVelocity(
            rag, comp._legL.footIdx);
        const glm::vec3 rightVelocity = Physics::GetRagdollBoneLinearVelocity(
            rag, comp._legR.footIdx);
        const float horizontalSpeed = glm::length(glm::vec2(
            rag._locomotionRootVel.x, rag._locomotionRootVel.z));
        const bool settledStanding = comp._test4ContactL && comp._test4ContactR
            && glm::length(leftVelocity) < 0.15f
            && glm::length(rightVelocity) < 0.15f
            && horizontalSpeed < 0.15f
            && tiltDeg < 15.0f;

        auto makeHorizontalBasis = [](glm::vec3& right, glm::vec3& forward) {
            constexpr float basisEpsilon = 1e-8f;
            const glm::vec3 worldUp(0.0f, 1.0f, 0.0f);
            right.y = 0.0f;
            forward.y = 0.0f;
            if (glm::dot(forward, forward) < basisEpsilon) {
                if (glm::dot(right, right) < basisEpsilon)
                    right = glm::vec3(1.0f, 0.0f, 0.0f);
                else
                    right = glm::normalize(right);
                forward = glm::cross(worldUp, right);
            }
            forward = glm::normalize(forward);

            // Foot separation includes fore/aft stance offset. Remove that component
            // before using it as a lateral lane axis, otherwise lane preservation can
            // silently shorten (or lengthen) the commanded support advance.
            right -= forward * glm::dot(right, forward);
            if (glm::dot(right, right) < basisEpsilon)
                right = glm::cross(forward, worldUp);
            right = glm::normalize(right);
        };
        auto horizontalForward = [](const glm::quat& heading) {
            glm::vec3 forward = glm::normalize(heading)
                * glm::vec3(0.0f, 0.0f, -1.0f);
            forward.y = 0.0f;
            return glm::dot(forward, forward) > 1e-8f
                ? glm::normalize(forward) : glm::vec3(0.0f, 0.0f, -1.0f);
        };
        auto signedHeadingDelta = [](glm::vec3 from, glm::vec3 to) {
            from.y = to.y = 0.0f;
            if (glm::dot(from, from) < 1e-8f
                || glm::dot(to, to) < 1e-8f) return 0.0f;
            from = glm::normalize(from);
            to = glm::normalize(to);
            return std::atan2(glm::cross(from, to).y,
                              glm::clamp(glm::dot(from, to), -1.0f, 1.0f));
        };
        auto setGaitHeading = [&](glm::vec3 forward) {
            forward.y = 0.0f;
            if (glm::dot(forward, forward) < 1e-8f)
                forward = glm::vec3(0.0f, 0.0f, -1.0f);
            forward = glm::normalize(forward);
            glm::vec3 right = glm::cross(
                forward, glm::vec3(0.0f, 1.0f, 0.0f));
            makeHorizontalBasis(right, forward);
            comp._test4Right = right;
            comp._test4Forward = forward;
            const float targetYaw = std::atan2(-forward.x, -forward.z);
            comp._test7HeadingTargetRot = glm::angleAxis(
                targetYaw, glm::vec3(0.0f, 1.0f, 0.0f));
            rag.locomotionTargetRot = comp._test7HeadingTargetRot;
        };
        if (!comp._test4BaselineValid) {
            comp._test4SettleTime = settledStanding
                ? comp._test4SettleTime + dt : 0.0f;
            if (comp._test4SettleTime >= 1.0f) {
                glm::vec3 right = rightFoot - leftFoot;
                right.y = 0.0f;
                if (glm::dot(right, right) < 1e-8f) right = comp._right;
                if (glm::dot(right, right) < 1e-8f) right = glm::vec3(1, 0, 0);
                glm::vec3 forward = comp._fwd;
                makeHorizontalBasis(right, forward);

                comp._test4BaselineValid = true;
                comp._test4FootBaselineL = leftFoot;
                comp._test4FootBaselineR = rightFoot;
                comp._test4ComBaseline = rag._locomotionCOM;
                comp._test4Right = glm::normalize(right);
                comp._test4Forward = glm::normalize(forward);
                comp._test4SupportTarget = comp._test4ComBaseline;
                comp._test4InitialTilt = tiltDeg;
                comp._test4PeakTilt = tiltDeg;
                comp._test4FinalTilt = tiltDeg;
                spdlog::info(
                    "[LocoTest4] baseline acquired at t={:.2f} span={:.3f}m "
                    "forward=({:+.2f},{:+.2f},{:+.2f}) tilt={:.1f}",
                    comp._test4Time,
                    glm::dot(rightFoot - leftFoot, comp._test4Right),
                    comp._test4Forward.x, comp._test4Forward.y,
                    comp._test4Forward.z, tiltDeg);
            }
        }
        if (!comp._test4BaselineValid) return;

        if (continuousEnabled) {
            if (comp._test7Running)
                rag.locomotionTargetRot = comp._test7HeadingTargetRot;
            const glm::vec3 rootAngularVelocity =
                rag._locomotionRootAngularVelocity;
            comp._test7RootPitchRate = glm::dot(
                rootAngularVelocity, comp._test4Right);
            comp._test7RootRollRate = glm::dot(
                rootAngularVelocity, comp._test4Forward);
            comp._test7RootYawRate = rootAngularVelocity.y;
            comp._test7RootTiltRate = glm::length(glm::vec2(
                rootAngularVelocity.x, rootAngularVelocity.z));
            comp._test7HeadingErrorDeg = rag._locomotionHeadingErrorDeg;
            if (comp._test7Running) {
                comp._test7PeakHeadingErrorDeg = glm::max(
                    comp._test7PeakHeadingErrorDeg,
                    std::abs(comp._test7HeadingErrorDeg));
            }
        }

        if (continuousCommand.stopRequested && continuousEnabled
            && comp._test7Running && comp._test4Phase != kStopping
            && comp._test4Phase != kReturnStand
            && !comp._test7StopRequested) {
            comp._test7StopRequested = true;
            spdlog::info(
                "[LocoTest7] STOP_REQUEST accepted phase={} step={} "
                "action=finish-current-step-then-recenter",
                comp._test4Phase, comp._test6StepIndex);
        } else if (gameplayCommand && comp._test7Running
                   && comp._test7StopRequested
                   && !continuousCommand.stopRequested
                   && comp._test4Phase != kStopping
                   && comp._test4Phase != kReturnStand) {
            // The player returned to the active heading before the committed stop.
            // Keep walking instead of carrying a stale stop request into the next step.
            comp._test7StopRequested = false;
            spdlog::info(
                "[LocoTest7] STOP_REQUEST canceled phase={} step={}",
                comp._test4Phase, comp._test6StepIndex);
        } else if ((continuousEnabled && continuousCommand.resetRequested)
                   || (!continuousEnabled && Input::IsKeyPressed(Key::F8))) {
            spdlog::info("[LocoTest4] baseline recapture requested");
            ResetTest4(comp);
            rag.locomotionSupportTargetWeight = 0.0f;
            rag.locomotionLiftBone = -1;
            rag.locomotionLiftMaxForce = 0.0f;
            return;
        }

        auto commitRuntimeStandingHeading = [&](glm::vec3 forward) {
            setGaitHeading(forward);
            const float targetYaw = std::atan2(
                -comp._test4Forward.x, -comp._test4Forward.z);
            // This is the source used by the general standing controller at the start
            // of the next update, so the eased physical turn and its target cannot fight.
            comp._yaw = targetYaw - glm::radians(comp.facingOffsetDeg);
        };
        auto rotateRuntimeReferences = [&](const glm::vec3& pivot,
                                           const glm::quat& turn) {
            auto rotatePoint = [&](glm::vec3 point) {
                return pivot + turn * (point - pivot);
            };
            comp._test4FootBaselineL = rotatePoint(comp._test4FootBaselineL);
            comp._test4FootBaselineR = rotatePoint(comp._test4FootBaselineR);
            comp._test4ComBaseline = rotatePoint(comp._test4ComBaseline);
            comp._test4SupportTarget = rotatePoint(comp._test4SupportTarget);
        };
        auto beginRuntimeTurn = [&](glm::vec3 from, glm::vec3 to,
                                    const char* action) {
            from.y = to.y = 0.0f;
            from = glm::normalize(from);
            to = glm::normalize(to);
            comp._runtimeTurnActive = true;
            comp._runtimeTurnElapsed = 0.0f;
            comp._runtimeTurnAppliedYaw = 0.0f;
            comp._runtimeTurnTotalYaw = signedHeadingDelta(from, to);
            comp._runtimeTurnTargetForward = to;
            comp._runtimeDesiredForward = to;
            const float turnDegrees = std::abs(
                glm::degrees(comp._runtimeTurnTotalYaw));
            const float turnSpeed = glm::max(
                comp.runtimeTurnSpeedDeg, 90.0f);
            comp._runtimeTurnDuration = glm::clamp(
                turnDegrees / turnSpeed, 0.12f, 0.40f);
            spdlog::info(
                "[LocoRuntime] TURN_BLEND {} yaw={:+.1f}deg duration={:.3f}s "
                "speed={:.0f}deg/s from=({:+.2f},{:+.2f}) "
                "to=({:+.2f},{:+.2f})",
                action, glm::degrees(comp._runtimeTurnTotalYaw),
                comp._runtimeTurnDuration, turnSpeed,
                from.x, from.z, to.x, to.z);
        };
        auto advanceRuntimeTurn = [&](glm::vec3 desiredForward) {
            constexpr float kTurnThresholdDeg = 0.5f;
            desiredForward.y = 0.0f;
            if (glm::dot(desiredForward, desiredForward) > 1e-8f)
                desiredForward = glm::normalize(desiredForward);
            else
                desiredForward = comp._runtimeTurnTargetForward;

            if (std::abs(glm::degrees(signedHeadingDelta(
                    comp._runtimeTurnTargetForward, desiredForward)))
                    > kTurnThresholdDeg) {
                beginRuntimeTurn(comp._test4Forward, desiredForward, "RETARGET");
            }

            comp._runtimeTurnElapsed = glm::min(
                comp._runtimeTurnElapsed + dt, comp._runtimeTurnDuration);
            const float linearT = comp._runtimeTurnDuration > 1e-6f
                ? comp._runtimeTurnElapsed / comp._runtimeTurnDuration : 1.0f;
            const float easedT = linearT * linearT * (3.0f - 2.0f * linearT);
            const float desiredAppliedYaw =
                comp._runtimeTurnTotalYaw * easedT;
            const float deltaYaw = desiredAppliedYaw
                - comp._runtimeTurnAppliedYaw;

            if (std::abs(deltaYaw) > 1e-7f) {
                const glm::vec3 pivot = 0.5f * (leftFoot + rightFoot);
                const glm::quat turn = glm::angleAxis(
                    deltaYaw, glm::vec3(0.0f, 1.0f, 0.0f));
                if (!Physics::RotateRagdollYaw(rag, pivot, deltaYaw)) {
                    comp._runtimeTurnActive = false;
                    comp._runtimeRestartBlocked = true;
                    spdlog::error(
                        "[LocoRuntime] TURN_BLEND FAIL applied={:+.1f}deg "
                        "action=block-restart-until-input-release",
                        glm::degrees(comp._runtimeTurnAppliedYaw));
                    return;
                }
                rotateRuntimeReferences(pivot, turn);
                commitRuntimeStandingHeading(
                    turn * comp._test4Forward);
                comp._runtimeTurnAppliedYaw = desiredAppliedYaw;
            }

            comp._test4PreviousSwingFootValid = false;
            if (linearT >= 1.0f) {
                comp._runtimeTurnActive = false;
                commitRuntimeStandingHeading(
                    comp._runtimeTurnTargetForward);
                spdlog::info(
                    "[LocoRuntime] TURN_BLEND COMPLETE yaw={:+.1f}deg "
                    "duration={:.3f}s heading=({:+.2f},{:+.2f}) "
                    "action=refresh-one-physics-step",
                    glm::degrees(comp._runtimeTurnTotalYaw),
                    comp._runtimeTurnElapsed,
                    comp._test4Forward.x, comp._test4Forward.z);
            }
        };

        if (comp._test4Phase == kIdle) {
            if (gameplayCommand && comp._runtimeTurnActive) {
                if (!continuousCommand.startRequested) {
                    comp._runtimeTurnActive = false;
                    comp._runtimeDesiredForward = comp._test4Forward;
                    spdlog::info(
                        "[LocoRuntime] TURN_BLEND CANCELED applied={:+.1f}deg "
                        "heading=({:+.2f},{:+.2f}) reason=input-release",
                        glm::degrees(comp._runtimeTurnAppliedYaw),
                        comp._test4Forward.x, comp._test4Forward.z);
                } else {
                    advanceRuntimeTurn(continuousCommand.desiredForward);
                }
                return;
            }
            const bool startLeftSupport = continuousEnabled
                ? continuousCommand.startRequested
                    && continuousCommand.initialSupportSide < 0
                : Input::IsKeyPressed(Key::F6);
            const bool startRightSupport = continuousEnabled
                ? continuousCommand.startRequested
                    && continuousCommand.initialSupportSide > 0
                : Input::IsKeyPressed(Key::F7);
            if (startLeftSupport || startRightSupport) {
                const glm::quat startHeading = glm::normalize(
                    rag.locomotionTargetRot);
                const glm::vec3 standingForward = horizontalForward(startHeading);
                glm::vec3 desiredForward = continuousEnabled
                    && glm::dot(continuousCommand.desiredForward,
                                continuousCommand.desiredForward) > 1e-8f
                    ? continuousCommand.desiredForward : comp._fwd;
                desiredForward.y = 0.0f;
                if (glm::dot(desiredForward, desiredForward) < 1e-8f)
                    desiredForward = standingForward;
                desiredForward = glm::normalize(desiredForward);

                const float requestedTurn = signedHeadingDelta(
                    standingForward, desiredForward);
                comp._runtimeDesiredForward = desiredForward;

                // Temporary gameplay turn: ease the complete physical ragdoll around
                // the settled feet, then leave the gait idle until the next physics step
                // has refreshed contacts. The ordinary straight gait starts afterward.
                constexpr float kTurnThresholdDeg = 0.5f;
                if (gameplayCommand
                    && std::abs(glm::degrees(requestedTurn))
                        > kTurnThresholdDeg) {
                    beginRuntimeTurn(
                        standingForward, desiredForward, "BEGIN");
                    advanceRuntimeTurn(desiredForward);
                    return;
                }

                comp._test4FootBaselineL = leftFoot;
                comp._test4FootBaselineR = rightFoot;
                comp._test4ComBaseline = rag._locomotionCOM;
                setGaitHeading(desiredForward);
                comp._test4SupportTarget = comp._test4ComBaseline;
                comp._test4SupportSide = startLeftSupport ? -1 : 1;
                if (continuousCommand.runLimit
                        == Diamond::Locomotion::RunLimit::None) {
                    comp._runtimeNextSupportSide = -comp._test4SupportSide;
                }
                comp._test4Phase = kWeightShift;
                comp._test4PhaseTime = 0.0f;
                comp._test4SettleTime = 0.0f;
                comp._test4AirborneTime = 0.0f;
                comp._test4ArrivalStableTime = 0.0f;
                comp._test4ReachLimit = 0.0f;
                comp._test7PlannedSupportAdvance = 0.0f;
                comp._test7AchievedSupportAdvance = 0.0f;
                comp._test4PlantAcquireStableTime = 0.0f;
                comp._test4TrajectoryT = 0.0f;
                comp._test4TouchdownAccepted = false;
                comp._test4Aborted = false;
                comp._test4MaxStanceDrift = 0.0f;
                comp._test4MaxPlantDrift = 0.0f;
                comp._test4InitialTilt = tiltDeg;
                comp._test4PeakTilt = tiltDeg;
                comp._test4FinalTilt = tiltDeg;
                comp._test4MaxMotorRatio = 0.0f;
                comp._test4MotorSaturated = false;
                comp._test4PlantPoseCaptured = false;
                comp._test4PreviousSwingFootValid = false;
                comp._test5TransferT = 0.0f;
                comp._test5HoldStableTime = 0.0f;
                comp._test5ContactLossTime = 0.0f;
                comp._test5ComError = 0.0f;
                comp._test5ComToOldSupport = 0.0f;
                comp._test5ComToNewSupport = 0.0f;
                comp._test5ComHorizontalSpeed = 0.0f;
                comp._test5TransferStartTarget = comp._test4ComBaseline;
                comp._test5TransferEndTarget = comp._test4ComBaseline;
                comp._test6StepIndex = multiStepEnabled ? 1 : 0;
                comp._test6StepsCompleted = 0;
                comp._test6InterStepStableTime = 0.0f;
                comp._test6InitialTilt = tiltDeg;
                for (int i = 0; i < 2; ++i) {
                    comp._test6StepForward[i] = 0.0f;
                    comp._test6StepMaxDrift[i] = 0.0f;
                    comp._test6StepPeakTilt[i] = 0.0f;
                    comp._test6StepMotorRatio[i] = 0.0f;
                }
                comp._test6PreviousContactsValid = multiStepEnabled;
                comp._test6PreviousContactL = comp._test4ContactL;
                comp._test6PreviousContactR = comp._test4ContactR;
                comp._test6ContactTransitionsL = 0;
                comp._test6ContactTransitionsR = 0;
                comp._test7ContactChangeTimeL = 0.0f;
                comp._test7ContactChangeTimeR = 0.0f;
                if (continuousEnabled) {
                    auto captureGroundFootReference = [&](auto& leg) {
                        bool rotationOk = false;
                        const glm::quat footWorld = Physics::GetRagdollBoneRotation(
                            rag, leg.footIdx, &rotationOk);
                        leg.groundReferenceFootRotationValid = rotationOk;
                        if (rotationOk) {
                            leg.groundReferenceFootHeadingLocalRotation = glm::normalize(
                                glm::conjugate(startHeading) * footWorld);
                        }

                        const glm::vec3 hip = physicalPosition(leg.hipIdx);
                        const glm::vec3 knee = physicalPosition(leg.kneeIdx);
                        const glm::vec3 ankle = physicalPosition(leg.ankleIdx);
                        const glm::vec3 chain = ankle - hip;
                        const glm::vec3 upper = knee - hip;
                        leg.groundReferenceKneePoleValid = false;
                        if (glm::dot(chain, chain) > 1e-8f) {
                            const glm::vec3 axis = glm::normalize(chain);
                            glm::vec3 pole = upper - axis * glm::dot(upper, axis);
                            if (glm::dot(pole, pole) > 1e-8f) {
                                pole = glm::normalize(pole);
                                leg.groundReferenceKneePoleHeadingLocal =
                                    glm::conjugate(startHeading) * pole;
                                leg.groundReferenceKneePoleValid = true;
                            }
                        }
                    };
                    // Store the settled feet and anatomical bend planes in the heading-local
                    // frame. Each touchdown can then adopt the active gait yaw without
                    // ratcheting measured landing error into the next step.
                    captureGroundFootReference(comp._legL);
                    captureGroundFootReference(comp._legR);
                    spdlog::info(
                        "[LocoDirection] START requested=({:+.2f},{:+.2f}) "
                        "active=({:+.2f},{:+.2f}) right=({:+.2f},{:+.2f}) "
                        "turn={:+.1f}deg support={} swing={}",
                        comp._runtimeDesiredForward.x,
                        comp._runtimeDesiredForward.z,
                        comp._test4Forward.x, comp._test4Forward.z,
                        comp._test4Right.x, comp._test4Right.z,
                        glm::degrees(requestedTurn),
                        comp._test4SupportSide < 0 ? "LEFT" : "RIGHT",
                        comp._test4SupportSide < 0 ? "RIGHT" : "LEFT");
                    comp._test7Running = true;
                    comp._test7StopRequested = false;
                    comp._test7RunTime = 0.0f;
                    comp._test7StepStartTime = 0.0f;
                    comp._test7LastStepPeriod = 0.0f;
                    comp._test7PreviousStepPeriod = 0.0f;
                    comp._test7MeasuredSpeed = 0.0f;
                    const float minimumAdvance = glm::min(
                        comp.test7MinStepLength, comp.test7MaxStepLength);
                    const float maximumAdvance = glm::max(
                        comp.test7MinStepLength, comp.test7MaxStepLength);
                    const float trackingReserve = glm::min(glm::max(
                        comp.test4TargetTolerance * 0.5f, 0.010f), 0.025f);
                    comp._test7CommandedStepLength = glm::clamp(
                        comp.test7NominalAdvance,
                        glm::min(maximumAdvance,
                                 minimumAdvance + trackingReserve),
                        maximumAdvance);
                    comp._test7ReachCommandCeiling = maximumAdvance;
                    comp._test7TakeoffContactRecoveryTime = 0.0f;
                    comp._test7SwingRecontactTime = 0.0f;
                    comp._test7SettledTrackingLoss = 0.0f;
                    comp._test7ForwardPreShift = 0.0f;
                    comp._test7ReachClearSteps = 0;
                    comp._test7PlannedSupportAdvance = 0.0f;
                    comp._test7AchievedSupportAdvance = 0.0f;
                    comp._test7LastStepLength = 0.0f;
                    comp._test7PreviousStepLength = 0.0f;
                    comp._test7LastSupportAdvance = 0.0f;
                    comp._test7PreviousSupportAdvance = 0.0f;
                    comp._test7StepMaxRelevantDrift = 0.0f;
                    comp._test7MaxDrift = 0.0f;
                    comp._test7PeakTilt = tiltDeg;
                    comp._test7HeadingErrorDeg = rag._locomotionHeadingErrorDeg;
                    comp._test7PeakHeadingErrorDeg = std::abs(
                        comp._test7HeadingErrorDeg);
                    comp._test7MaxMotorRatio = 0.0f;
                    comp._test7StopStableTime = 0.0f;
                    comp._test7StopFootDriftL = 0.0f;
                    comp._test7StopFootDriftR = 0.0f;
                    comp._test7StopMaxFootDrift = 0.0f;
                    comp._test7StopSettleFootDriftL = 0.0f;
                    comp._test7StopSettleFootDriftR = 0.0f;
                    comp._test7StopMaxSettleFootDrift = 0.0f;
                    comp._test7StopSettleReferenceValid = false;
                    comp._test7CrouchBlend = 0.0f;
                    comp._test7FootCorrection = 0.0f;
                    comp._test7FootCorrectionForward = 0.0f;
                    comp._test7FootTargetSpeed = 0.0f;
                    comp._test7SoleLevelBlend = 0.0f;
                    comp._test7IkPlanHipValid = false;
                    comp._test7TakeoffContactRecoveryActive = false;
                    comp._test7ReachClampedStep = false;
                    comp._test7OldSupportDriftAllowanceLogged = false;
                    comp._test7StartCom = rag._locomotionCOM;
                    comp._test7StepStartCom = rag._locomotionCOM;
                    spdlog::info(
                        "[LocoTest7] START mode={} target={} desiredSpeed={:.3f}m/s "
                        "supportAdvanceBounds=({:.3f},{:.3f})m "
                        "initialAdvance={:.3f}m crouch={:.3f}m usableReach={:.3f} "
                        "footControl=(kp={:.2f},lead={:.3f}s,max={:.3f}m,level={:.2f}s) "
                        "upright=(mode=SOLVER_SPRING,stiffness={:.0f}Nm/rad,"
                        "damping={:.0f}Nm*s/rad,"
                        "maxTorque={:.0f}Nm/axis) "
                        "heading=(stiffness={:.0f}Nm/rad,damping={:.0f}Nm*s/rad,"
                        "maxTorque={:.0f}Nm) gates=(tilt={:.1f}deg,heading={:.1f}deg)",
                        Diamond::Locomotion::RunLimitName(
                            continuousCommand.runLimit),
                        continuousCommand.runLimit
                                == Diamond::Locomotion::RunLimit::Duration
                            ? glm::max(continuousCommand.durationLimit, 10.0f)
                            : static_cast<float>(glm::max(
                                continuousCommand.stepLimit, 2)),
                        continuousCommand.desiredSpeed,
                        glm::min(comp.test7MinStepLength, comp.test7MaxStepLength),
                        glm::max(comp.test7MinStepLength, comp.test7MaxStepLength),
                        comp._test7CommandedStepLength,
                        glm::max(comp.test7ReachCrouch, 0.0f),
                        glm::clamp(comp.test7UsableReachFraction, 0.94f, 0.99f),
                        glm::clamp(comp.test7FootPositionGain, 0.0f, 1.0f),
                        glm::max(comp.test7FootVelocityLeadTime, 0.0f),
                        glm::max(comp.test7MaxFootCorrection, 0.0f),
                        glm::max(comp.test7SoleLevelTime, 0.10f),
                        glm::max(comp.test7UprightStiffness, 0.0f),
                        glm::max(comp.test7UprightDamping, 0.0f),
                        glm::max(comp.test7UprightMaxTorque, 0.0f),
                        glm::max(comp.test7HeadingStiffness, 0.0f),
                        glm::max(comp.test7HeadingDamping, 0.0f),
                        glm::max(comp.test7HeadingMaxTorque, 0.0f),
                        glm::clamp(comp.test7InterStepTiltLimit, 12.0f, 25.0f),
                        glm::clamp(comp.test7InterStepHeadingLimit, 2.0f, 20.0f));
                }
                spdlog::info("[LocoTest{}] sequence start support={} swing={} step={:.3f}m",
                             comp.validationTest,
                             comp._test4SupportSide < 0 ? "LEFT" : "RIGHT",
                             comp._test4SupportSide < 0 ? "RIGHT" : "LEFT",
                             comp.test4StepLength);
            }
        }

        if (comp._test4Phase > 0) comp._test4PhaseTime += dt;
        if (continuousEnabled && comp._test7Running)
            comp._test7RunTime += dt;
        if (continuousEnabled) {
            // Keep a small crouch for the whole walking sequence. Planning a grounded
            // foothold at nearly full anatomical extension amplified an 8 mm radial motor
            // lag into a 3-4 cm horizontal landing miss. The existing dynamic-root height
            // spring supplies this offset without teleporting any body. Ramp it out during
            // STOPPING / RETURN_STAND (and after an abort) instead of changing height at a
            // phase boundary.
            const bool crouchRequested = comp._test7Running
                && comp._test4Phase >= kWeightShift
                && comp._test4Phase <= kInterStep;
            comp._test7CrouchBlend = Approach(
                comp._test7CrouchBlend, crouchRequested ? 1.0f : 0.0f,
                dt / glm::max(comp.test7CrouchTime, 0.10f));
            rag.locomotionHeightOffset = -glm::max(
                comp.test7ReachCrouch, 0.0f) * smoothstep(comp._test7CrouchBlend);

        }
        const float desiredComCommand = comp._test4Phase >= kWeightShift
            && comp._test4Phase <= kComplete
            ? static_cast<float>(comp._test4SupportSide) : 0.0f;
        comp._test4ComCommand = Approach(
            comp._test4ComCommand, desiredComCommand,
            dt / glm::max(comp.test2ShiftTime, 0.01f));

        const float fraction = glm::clamp(comp.test3SupportFraction, 0.0f, 1.0f);
        const float leftAvailable = glm::dot(
            comp._test4FootBaselineL - comp._test4ComBaseline, comp._test4Right);
        const float rightAvailable = glm::dot(
            comp._test4FootBaselineR - comp._test4ComBaseline, comp._test4Right);
        const float lateralOffset = comp._test4ComCommand < 0.0f
            ? -comp._test4ComCommand * glm::min(leftAvailable, 0.0f) * fraction
            :  comp._test4ComCommand * glm::max(rightAvailable, 0.0f) * fraction;
        glm::vec3 supportTarget = comp._test4ComBaseline
                                + comp._test4Right * lateralOffset;
        glm::vec3 supportVelocity(0.0f);
        bool supportVelocityExplicit = false;
        float test7StanceForwardTarget = 0.0f;
        if (continuousEnabled && comp._test4Phase >= kWeightShift
            && comp._test4Phase <= kSettle) {
            // TRANSFER leaves the COM at the forward midpoint of the two planted feet.
            // Before the next single-support phase, deliberately move it toward the new
            // stance foot in both axes. The old controller shifted only laterally, leaving
            // the hip progressively farther behind every new foothold.
            const glm::vec3 stanceSupport = comp._test4SupportSide < 0
                ? leftFoot : rightFoot;
            constexpr float kTest7ForwardStanceFraction = 0.90f;
            test7StanceForwardTarget = glm::dot(
                stanceSupport - comp._test4ComBaseline,
                comp._test4Forward) * kTest7ForwardStanceFraction;
            supportTarget += comp._test4Forward * test7StanceForwardTarget;
        }
        if (transferEnabled && comp._test4Phase >= kTransfer
            && comp._test4Phase <= kComplete) {
            const float transferT = comp._test4Phase == kTransfer
                ? glm::clamp(comp._test4PhaseTime
                    / glm::max(comp.test5TransferTime, 0.05f), 0.0f, 1.0f)
                : 1.0f;
            comp._test5TransferT = transferT;
            // Unlike the old walking path, transfer is phase-continuous and supplies the
            // support controller with the target velocity. This avoids an instantaneous
            // forward pull when the newly planted foot becomes authoritative.
            supportTarget = glm::mix(comp._test5TransferStartTarget,
                                     comp._test5TransferEndTarget,
                                     smoothstep(transferT));
        }
        if (continuousEnabled && comp._test4Phase == kInterStep) {
            // Both endpoints were captured once at phase entry. Never rebuild the target
            // from vibrating physical feet: differentiating that moving midpoint turned
            // contact noise into a commanded COM velocity and fed it back into the body.
            const float duration = glm::max(
                comp.test7InterStepRecenterTime, 0.20f);
            const float recenterT = glm::clamp(
                comp._test4PhaseTime / duration, 0.0f, 1.0f);
            const float recenterBlend = smoothstep(recenterT);
            supportTarget = glm::mix(
                comp._test7InterStepRecenterStart,
                comp._test7InterStepRecenterTarget,
                recenterBlend);
            const float blendRate = recenterT > 0.0f && recenterT < 1.0f
                ? 6.0f * recenterT * (1.0f - recenterT) / duration
                : 0.0f;
            supportVelocity = (comp._test7InterStepRecenterTarget
                             - comp._test7InterStepRecenterStart) * blendRate;
            supportVelocityExplicit = true;
            comp._test7InterStepRecenterT = recenterT;
            comp._test7InterStepCenterError = glm::length(glm::vec2(
                rag._locomotionCOM.x - comp._test7InterStepRecenterTarget.x,
                rag._locomotionCOM.z - comp._test7InterStepRecenterTarget.z));
        } else if (continuousEnabled) {
            comp._test7InterStepRecenterT = 0.0f;
            comp._test7InterStepCenterError = 0.0f;
        }
        if (continuousEnabled && comp._test4Phase == kStopping) {
            // The last completed transfer already produced a stable support point. Hold it
            // throughout shutdown instead of pulling the COM toward the sole midpoint while
            // the legs are changing pose. A fixed target also makes the feed-forward term
            // exactly zero rather than deriving a velocity from numerical differencing.
            supportTarget = comp._test7StopStartTarget;
            supportVelocity = glm::vec3(0.0f);
            supportVelocityExplicit = true;
        }
        if (!supportVelocityExplicit) {
            supportVelocity = dt > 1e-6f
                ? (supportTarget - comp._test4SupportTarget) / dt
                : glm::vec3(0.0f);
        }
        supportVelocity.y = 0.0f;
        comp._test4SupportTarget = supportTarget;

        rag.locomotionSupportTarget = supportTarget;
        rag.locomotionSupportTargetVel = supportVelocity;
        rag.locomotionSupportTargetWeight =
            continuousEnabled && (comp._test4Phase == kReturnStand
                                  || comp._test4Phase == kComplete)
                ? 0.0f : 1.0f;
        rag.locomotionCOMSupportFreq = glm::max(comp.test2SupportFrequency, 0.0f);
        rag.locomotionCOMSupportMaxAccel = glm::max(comp.test2SupportMaxAccel, 0.0f);

        // Shutdown sole captures are diagnostic references only. The per-frame controller
        // reset above leaves both lock bones disabled and both weights at zero; do not add a
        // second world-space controller while support and joint motors settle the character.
        if (continuousEnabled && comp._test4Phase == kStopping
            && !comp._test7StopSettleReferenceValid) {
            // This is commanded displacement while the captured walking pose becomes the
            // standing pose. Keep it visible, but do not treat it as continued plant slide.
            comp._test7StopFootDriftL = glm::length(glm::vec2(
                leftFoot.x - comp._test7StopFootTargetL.x,
                leftFoot.z - comp._test7StopFootTargetL.z));
            comp._test7StopFootDriftR = glm::length(glm::vec2(
                rightFoot.x - comp._test7StopFootTargetR.x,
                rightFoot.z - comp._test7StopFootTargetR.z));
            comp._test7StopMaxFootDrift = glm::max(
                comp._test7StopMaxFootDrift,
                glm::max(comp._test7StopFootDriftL,
                         comp._test7StopFootDriftR));
        }
        if (continuousEnabled && comp._test7StopSettleReferenceValid
            && (comp._test4Phase == kStopping
                || comp._test4Phase == kReturnStand)) {
            comp._test7StopSettleFootDriftL = glm::length(glm::vec2(
                leftFoot.x - comp._test7StopSettleFootTargetL.x,
                leftFoot.z - comp._test7StopSettleFootTargetL.z));
            comp._test7StopSettleFootDriftR = glm::length(glm::vec2(
                rightFoot.x - comp._test7StopSettleFootTargetR.x,
                rightFoot.z - comp._test7StopSettleFootTargetR.z));
            comp._test7StopMaxSettleFootDrift = glm::max(
                comp._test7StopMaxSettleFootDrift,
                glm::max(comp._test7StopSettleFootDriftL,
                         comp._test7StopSettleFootDriftR));
        }

        comp._test4ComLateral = glm::dot(
            rag._locomotionCOM - comp._test4ComBaseline, comp._test4Right);
        comp._test4TargetLateral = glm::dot(
            supportTarget - comp._test4ComBaseline, comp._test4Right);
        comp._test4PeakTilt = glm::max(comp._test4PeakTilt, tiltDeg);
        comp._test4FinalTilt = tiltDeg;

        Leg* swing = comp._test4SupportSide < 0 ? &comp._legR : &comp._legL;
        Leg* stance = comp._test4SupportSide < 0 ? &comp._legL : &comp._legR;
        const glm::vec3 swingFoot = physicalPosition(swing->footIdx);
        const glm::vec3 stanceFoot = physicalPosition(stance->footIdx);
        const glm::vec3 swingVelocity = Physics::GetRagdollBoneLinearVelocity(
            rag, swing->footIdx);
        auto horizontalDistance = [](const glm::vec3& a, const glm::vec3& b) {
            return glm::length(glm::vec2(a.x - b.x, a.z - b.z));
        };
        comp._test5ComHorizontalSpeed = glm::length(glm::vec2(
            rag._locomotionCOMVel.x, rag._locomotionCOMVel.z));
        if (transferEnabled && comp._test4Phase >= kTransfer
            && comp._test4Phase <= kComplete) {
            comp._test5ComError = horizontalDistance(
                rag._locomotionCOM, comp._test5TransferEndTarget);
            comp._test5ComToOldSupport = horizontalDistance(
                rag._locomotionCOM, stanceFoot);
            comp._test5ComToNewSupport = horizontalDistance(
                rag._locomotionCOM, swingFoot);
        }
        const bool transferOrHold = comp._test4Phase >= kTransfer
                                 && comp._test4Phase <= kHold;
        const bool test7OldSupportUnloaded = continuousEnabled && transferOrHold
            && comp._test5ComToNewSupport + 0.020f
                < comp._test5ComToOldSupport;
        comp._test4ApiVelocity = swingVelocity;
        glm::vec3 contactNormal(0.0f);
        glm::vec3 contactPoint(0.0f);
        const bool swingContactNow = FootContact(
            rag, swing->footIdx, &contactNormal, &contactPoint);
        const bool stanceContactNow = FootContact(rag, stance->footIdx);
        if (comp._test4PreviousSwingFootValid && dt > 1e-6f)
            comp._test4MeasuredVelocity =
                (swingFoot - comp._test4PreviousSwingFoot) / dt;
        else
            comp._test4MeasuredVelocity = glm::vec3(0.0f);
        comp._test4PreviousSwingFoot = swingFoot;
        comp._test4PreviousSwingFootValid = true;

        bool swingRotationOk = false;
        const glm::quat swingRotation = Physics::GetRagdollBoneRotation(
            rag, swing->footIdx, &swingRotationOk);
        comp._test4FootUpY = swingRotationOk
            ? (swingRotation * glm::vec3(0.0f, 1.0f, 0.0f)).y : 0.0f;
        comp._test4ContactPoint = contactPoint;
        comp._test4ContactLocal = swingContactNow && swingRotationOk
            ? glm::conjugate(swingRotation) * (contactPoint - swingFoot)
            : glm::vec3(0.0f);
        const glm::vec3 stanceBaseline = comp._test4SupportSide < 0
            ? comp._test4FootBaselineL : comp._test4FootBaselineR;
        comp._test4StanceDrift = glm::length(glm::vec2(
            stanceFoot.x - stanceBaseline.x, stanceFoot.z - stanceBaseline.z));
        comp._test4MaxStanceDrift = glm::max(
            comp._test4MaxStanceDrift, comp._test4StanceDrift);
        if (comp._test4TouchdownAccepted) {
            comp._test4PlantDrift = glm::length(glm::vec2(
                swingFoot.x - comp._test4TouchdownPlant.x,
                swingFoot.z - comp._test4TouchdownPlant.z));
            comp._test4MaxPlantDrift = glm::max(
                comp._test4MaxPlantDrift, comp._test4PlantDrift);
        } else {
            comp._test4PlantDrift = 0.0f;
        }
        comp._test4Clearance = swingFoot.y - comp._test4SwingStart.y;
        comp._test4ForwardTravel = glm::dot(
            swingFoot - comp._test4SwingStart, comp._test4Forward);
        comp._test7AchievedSupportAdvance = continuousEnabled
            ? glm::dot(swingFoot - stanceFoot, comp._test4Forward) : 0.0f;
        if (comp._test4Phase >= kTakeoff) {
            const glm::vec3 targetDelta = swingFoot - comp._test4Foothold;
            comp._test4ForwardTargetError = glm::dot(
                comp._test4Foothold - swingFoot, comp._test4Forward);
            comp._test4LateralTargetError = glm::dot(
                targetDelta, comp._test4Right);
            comp._test4HorizontalTargetError = glm::length(glm::vec2(
                targetDelta.x, targetDelta.z));
            comp._test4VerticalTargetError = std::abs(targetDelta.y);
        } else {
            comp._test4ForwardTargetError = 0.0f;
            comp._test4LateralTargetError = 0.0f;
            comp._test4HorizontalTargetError = 0.0f;
            comp._test4VerticalTargetError = 0.0f;
        }

        comp._test4MotorSaturated = false;
        for (int i = 0; i < 6; ++i) {
            comp._test4MaxMotorRatio = glm::max(
                comp._test4MaxMotorRatio, rag._locomotionMotorSaturationRatio[i]);
            comp._test4MotorSaturated = comp._test4MotorSaturated
                || rag._locomotionMotorSaturated[i];
        }
        if (continuousEnabled && comp._test7Running) {
            const bool activeGaitPhase = comp._test4Phase >= kWeightShift
                                      && comp._test4Phase <= kInterStep;
            if (activeGaitPhase) {
                // Once transfer has unloaded the old support, its small release slide is
                // preparation for the next swing rather than loss of the loaded plant.
                // STOPPING / RETURN_STAND own separate immutable foot anchors and drift
                // metrics, so their deliberate posture transition cannot contaminate the
                // active-gait measurement.
                const float relevantDrift = test7OldSupportUnloaded
                    ? comp._test4PlantDrift
                    : glm::max(comp._test4StanceDrift, comp._test4PlantDrift);
                comp._test7StepMaxRelevantDrift = glm::max(
                    comp._test7StepMaxRelevantDrift, relevantDrift);
                comp._test7MaxDrift = glm::max(
                    comp._test7MaxDrift, relevantDrift);
            }
            comp._test7PeakTilt = glm::max(comp._test7PeakTilt, tiltDeg);
            comp._test7MaxMotorRatio = glm::max(
                comp._test7MaxMotorRatio, comp._test4MaxMotorRatio);
        }

        auto captureSwing = [&]() {
            const glm::vec3 hip = physicalPosition(swing->hipIdx);
            const glm::vec3 knee = physicalPosition(swing->kneeIdx);
            const glm::vec3 ankle = physicalPosition(swing->ankleIdx);
            comp._test4SwingStart = swingFoot;
            comp._test4ArcStart = swingFoot;
            comp._test4DesiredFoot = swingFoot;
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

            bool hipOk = false, kneeOk = false, ankleOk = false, footOk = false;
            swing->referenceHipWorld = Physics::GetRagdollBoneRotation(
                rag, swing->hipIdx, &hipOk);
            const glm::quat kneeWorld = Physics::GetRagdollBoneRotation(
                rag, swing->kneeIdx, &kneeOk);
            const glm::quat ankleWorld = Physics::GetRagdollBoneRotation(
                rag, swing->ankleIdx, &ankleOk);
            const glm::quat footWorld = Physics::GetRagdollBoneRotation(
                rag, swing->footIdx, &footOk);
            if (footOk) {
                swing->plantedFootWorldRotation = footWorld;
                swing->ankleFromFootLocal = glm::conjugate(footWorld)
                    * (ankle - swingFoot);
            } else {
                swing->plantedFootWorldRotation = OrientationOf(BoneWorldMatrix(
                    skeleton, animator, entityWorld, swing->footIdx));
                swing->ankleFromFootLocal =
                    glm::conjugate(swing->plantedFootWorldRotation)
                    * (ankle - swingFoot);
            }
            if (hipOk && kneeOk)
                swing->referenceKneeLocal = glm::normalize(
                    glm::conjugate(swing->referenceHipWorld) * kneeWorld);
            else
                swing->referenceKneeLocal = skeleton.bones[swing->kneeIdx].localR;
            if (kneeOk && ankleOk)
                swing->referenceAnkleLocal = glm::normalize(
                    glm::conjugate(kneeWorld) * ankleWorld);
            else
                swing->referenceAnkleLocal = skeleton.bones[swing->ankleIdx].localR;
            if (ankleOk && footOk)
                swing->referenceFootLocal = glm::normalize(
                    glm::conjugate(ankleWorld) * footWorld);
            else
                swing->referenceFootLocal = skeleton.bones[swing->footIdx].localR;
            if (!hipOk)
                swing->referenceHipWorld = glm::normalize(
                    ParentWorldRot(rag, skeleton, animator, entityWorld, swing->hipIdx)
                    * animator.pose[swing->hipIdx].rotation);

            swing->hipCommand = animator.pose[swing->hipIdx].rotation;
            swing->kneeCommand = swing->referenceKneeLocal;
            swing->ankleCommand = swing->referenceAnkleLocal;
            swing->footCommand = swing->referenceFootLocal;
            swing->commandValid = true;
        };

        auto capturePhysicalLocalPose = [&](Leg& leg) {
            bool hipOk = false, kneeOk = false, ankleOk = false, footOk = false;
            const glm::quat hipWorld = Physics::GetRagdollBoneRotation(
                rag, leg.hipIdx, &hipOk);
            const glm::quat kneeWorld = Physics::GetRagdollBoneRotation(
                rag, leg.kneeIdx, &kneeOk);
            const glm::quat ankleWorld = Physics::GetRagdollBoneRotation(
                rag, leg.ankleIdx, &ankleOk);
            const glm::quat footWorld = Physics::GetRagdollBoneRotation(
                rag, leg.footIdx, &footOk);
            if (hipOk)
                leg.hipCommand = glm::normalize(glm::conjugate(ParentWorldRot(
                    rag, skeleton, animator, entityWorld, leg.hipIdx)) * hipWorld);
            if (hipOk && kneeOk)
                leg.kneeCommand = glm::normalize(glm::conjugate(hipWorld) * kneeWorld);
            if (kneeOk && ankleOk)
                leg.ankleCommand = glm::normalize(glm::conjugate(kneeWorld) * ankleWorld);
            if (ankleOk && footOk)
                leg.footCommand = glm::normalize(glm::conjugate(ankleWorld) * footWorld);
            leg.commandValid = true;
        };

        // The foothold is a target for the sole body, while the two-bone solve ends at
        // the ankle. Keep their separation in sole-local space. The settled sole reference
        // is heading-local: pitch/roll remain ground aligned while yaw follows the latched
        // gait frame. The per-step planted rotation is only the continuity endpoint.
        auto nominalFootWorldRotation = [&]() {
            return continuousEnabled && swing->groundReferenceFootRotationValid
                ? glm::normalize(comp._test7HeadingTargetRot
                    * swing->groundReferenceFootHeadingLocalRotation)
                : glm::normalize(swing->plantedFootWorldRotation);
        };
        auto nominalKneePoleWorld = [&]() {
            glm::vec3 pole = continuousEnabled
                && swing->groundReferenceKneePoleValid
                ? comp._test7HeadingTargetRot
                    * swing->groundReferenceKneePoleHeadingLocal
                : swing->kneePoleWorld;
            return glm::dot(pole, pole) > 1e-8f
                ? glm::normalize(pole) : comp._test4Forward;
        };
        auto ankleFromFootWorld = [&](const glm::quat& footWorldRotation) {
            return glm::normalize(footWorldRotation) * swing->ankleFromFootLocal;
        };
        auto nominalAnkleFromFootWorld = [&]() {
            return ankleFromFootWorld(nominalFootWorldRotation());
        };
        auto rotationDifferenceDeg = [](glm::quat a, glm::quat b) {
            const glm::quat difference = glm::normalize(
                glm::conjugate(glm::normalize(a)) * glm::normalize(b));
            return glm::degrees(2.0f * std::acos(
                glm::clamp(std::abs(difference.w), 0.0f, 1.0f)));
        };
        auto horizontalYawDeg = [](const glm::quat& rotation) {
            glm::vec3 forward = glm::normalize(rotation)
                * glm::vec3(0.0f, 0.0f, -1.0f);
            forward.y = 0.0f;
            if (glm::dot(forward, forward) < 1e-8f) return 0.0f;
            forward = glm::normalize(forward);
            return glm::degrees(std::atan2(-forward.x, -forward.z));
        };
        bool physicalSwingFootRotationOk = false;
        const glm::quat physicalSwingFootWorld =
            Physics::GetRagdollBoneRotation(
                rag, swing->footIdx, &physicalSwingFootRotationOk);
        comp._test7SoleAngularErrorDeg = continuousEnabled
            && physicalSwingFootRotationOk
            ? rotationDifferenceDeg(
                physicalSwingFootWorld, nominalFootWorldRotation())
            : 0.0f;

        auto planFoothold = [&]() {
            const float minimumAdvance = glm::min(
                comp.test7MinStepLength, comp.test7MaxStepLength);
            const float maximumAdvance = glm::max(
                comp.test7MinStepLength, comp.test7MaxStepLength);
            // Test 7 only needs a modest admission margin here. The completed step is
            // still required to satisfy the independent minimum support advance below.
            const float baseTrackingReserve = 0.010f;
            const float trackingReserve = continuousEnabled
                ? glm::min(glm::max(
                    baseTrackingReserve,
                    comp._test7SettledTrackingLoss + 0.003f), 0.040f)
                : 0.0f;
            const float minimumCommand = glm::min(
                maximumAdvance, minimumAdvance + trackingReserve);
            const float placementDistance = continuousEnabled
                ? glm::clamp(comp._test7CommandedStepLength,
                    minimumCommand, maximumAdvance)
                : glm::clamp(comp.test4StepLength, 0.15f, 0.25f);
            glm::vec3 requestedTarget;
            if (continuousEnabled) {
                // A limit cycle places the next support relative to the current support,
                // not relative to where this swing foot happened to land one cycle ago.
                // Gameplay turns give each anatomical foot a signed lane in the new frame;
                // a raw projection can change sign during a 90/180-degree reframe and ask
                // the swing leg to cross through the planted leg.
                float lateralLane = glm::dot(
                    comp._test4SwingStart - stanceFoot, comp._test4Right);
                if (gameplayCommand) {
                    constexpr float kMinimumRuntimeLane = 0.10f;
                    const float laneMagnitude = glm::clamp(
                        std::abs(lateralLane), kMinimumRuntimeLane, 0.24f);
                    const float swingSide = swing == &comp._legL ? -1.0f : 1.0f;
                    lateralLane = swingSide * laneMagnitude;
                }
                requestedTarget = stanceFoot
                    + comp._test4Forward * placementDistance
                    + comp._test4Right * lateralLane;
            } else {
                requestedTarget = comp._test4SwingStart
                    + comp._test4Forward * placementDistance;
            }
            const HitResult startGround = Physics::Raycast(
                comp._test4SwingStart + glm::vec3(0, 0.25f, 0),
                glm::vec3(0, -1, 0), 0.75f, entity, false);
            const HitResult targetGround = Physics::Raycast(
                requestedTarget + glm::vec3(0, 0.35f, 0),
                glm::vec3(0, -1, 0), 1.0f, entity, false);
            const float soleCenterOffset = startGround.hit
                ? comp._test4SwingStart.y - startGround.point.y : 0.0f;
            requestedTarget.y = targetGround.hit
                ? targetGround.point.y + soleCenterOffset : comp._test4SwingStart.y;

            const glm::vec3 hip = physicalPosition(swing->hipIdx);
            glm::vec3 predictedLandingHip = hip;
            if (continuousEnabled) {
                // Weight shift waits until the COM is within 1 cm of its stance target.
                // Include that small remaining horizontal correction when testing the
                // grounded foothold, so planning and landing use the same expected pelvis
                // position instead of the pre-takeoff snapshot.
                glm::vec3 remainingComShift =
                    comp._test4SupportTarget - rag._locomotionCOM;
                remainingComShift.y = 0.0f;
                if (const float remaining = glm::length(remainingComShift);
                    remaining > 0.015f && remaining > 1e-5f)
                    remainingComShift *= 0.015f / remaining;
                predictedLandingHip += remainingComShift;
            }
            if (continuousEnabled) {
                comp._test7IkPlanHip = predictedLandingHip;
                comp._test7IkPlanHipValid = true;
            }
            const float legLength = glm::length(skeleton.bones[swing->kneeIdx].localT)
                                  + glm::length(skeleton.bones[swing->ankleIdx].localT);
            const float configuredReach = legLength
                * glm::clamp(comp.proceduralMaxReach, 0.70f, 0.99f);
            const float currentReach = glm::length(
                comp._test4SwingStart
                    + ankleFromFootWorld(swing->plantedFootWorldRotation) - hip);
            const float safeReachFraction = continuousEnabled
                ? glm::min(comp.test4SafeReachFraction,
                           comp.test7UsableReachFraction)
                : comp.test4SafeReachFraction;
            const float safeAnatomicalReach = legLength * glm::clamp(
                safeReachFraction, 0.94f, 0.995f);
            const float antiSingularityCeiling = legLength * 0.995f;
            // The current physical stance is necessarily reachable. Tests 4-6 may begin
            // with an almost straight knee, so their captured reach can legitimately exceed
            // the generic anti-singularity fraction. Continuous gait instead couples its
            // small pelvis crouch with a conservative usable fraction; retaining knee bend
            // avoids magnifying millimetres of radial motor lag into centimetres of forward
            // landing error. Never shorten a captured physical pose just to satisfy the cap.
            comp._test4ReachLimit = glm::min(
                glm::max(configuredReach,
                         glm::max(currentReach, safeAnatomicalReach)),
                glm::max(currentReach, antiSingularityCeiling));

            auto groundedCandidate = [&](float horizontalScale) {
                glm::vec3 candidate = glm::mix(
                    comp._test4SwingStart, requestedTarget,
                    glm::clamp(horizontalScale, 0.0f, 1.0f));
                const HitResult ground = Physics::Raycast(
                    candidate + glm::vec3(0, 0.35f, 0),
                    glm::vec3(0, -1, 0), 1.0f, entity, false);
                candidate.y = ground.hit
                    ? ground.point.y + soleCenterOffset
                    : glm::mix(comp._test4SwingStart.y, requestedTarget.y,
                               glm::clamp(horizontalScale, 0.0f, 1.0f));
                return candidate;
            };
            auto targetReach = [&](const glm::vec3& candidate) {
                return glm::length(candidate + nominalAnkleFromFootWorld()
                    - predictedLandingHip);
            };

            glm::vec3 target = requestedTarget;
            const float requestedReach = targetReach(requestedTarget);
            bool reachClamped = requestedReach > comp._test4ReachLimit;
            if (reachClamped) {
                float reachableScale = 0.0f;
                float unreachableScale = 1.0f;
                for (int iteration = 0; iteration < 12; ++iteration) {
                    const float candidateScale =
                        0.5f * (reachableScale + unreachableScale);
                    const glm::vec3 candidate = groundedCandidate(candidateScale);
                    if (targetReach(candidate) <= comp._test4ReachLimit)
                        reachableScale = candidateScale;
                    else
                        unreachableScale = candidateScale;
                }
                target = groundedCandidate(reachableScale);
            }
            comp._test4Foothold = target;
            const float requestedForward = glm::dot(
                requestedTarget - comp._test4SwingStart, comp._test4Forward);
            const float plannedForward = glm::dot(
                target - comp._test4SwingStart, comp._test4Forward);
            const float requestedSupportAdvance = glm::dot(
                requestedTarget - stanceFoot, comp._test4Forward);
            const float plannedSupportAdvance = glm::dot(
                target - stanceFoot, comp._test4Forward);
            comp._test7PlannedSupportAdvance = continuousEnabled
                ? plannedSupportAdvance : 0.0f;
            if (continuousEnabled)
                comp._test7ReachClampedStep = reachClamped;
            spdlog::info(
                "[LocoTest4] foothold latched start=({:+.3f},{:+.3f},{:+.3f}) "
                "requested=({:+.3f},{:+.3f},{:+.3f}) "
                "target=({:+.3f},{:+.3f},{:+.3f}) forward={:.3f}->{:.3f}m "
                "supportAdvance={:.3f}->{:.3f}m "
                "reach={:.3f}->{:.3f}/{:.3f}m "
                "reachBasis=(captured={:.3f},safe={:.3f},anatomy={:.3f}) "
                "hipPlanShift={:.3f}m "
                "clamp={} ground={}",
                comp._test4SwingStart.x, comp._test4SwingStart.y,
                comp._test4SwingStart.z,
                requestedTarget.x, requestedTarget.y, requestedTarget.z,
                target.x, target.y, target.z,
                requestedForward, plannedForward,
                requestedSupportAdvance, plannedSupportAdvance,
                requestedReach, targetReach(target), comp._test4ReachLimit,
                currentReach, safeAnatomicalReach, legLength,
                glm::length(predictedLandingHip - hip),
                reachClamped ? "HORIZONTAL" : "none",
                targetGround.hit ? "hit" : "fallback");
            if (continuousEnabled) {
                const glm::vec3 footholdDelta = target - comp._test4SwingStart;
                spdlog::info(
                    "[LocoDirection] FOOTHOLD step={} swing={} "
                    "command=({:+.2f},{:+.2f}) gait=({:+.2f},{:+.2f}) "
                    "right=({:+.2f},{:+.2f}) delta=({:+.3f},{:+.3f}) "
                    "projected=(forward={:+.3f},lateral={:+.3f}) "
                    "soleYaw=(actual={:+.1f},target={:+.1f})deg",
                    comp._test6StepIndex,
                    swing == &comp._legL ? "LEFT" : "RIGHT",
                    comp._runtimeDesiredForward.x,
                    comp._runtimeDesiredForward.z,
                    comp._test4Forward.x, comp._test4Forward.z,
                    comp._test4Right.x, comp._test4Right.z,
                    footholdDelta.x, footholdDelta.z,
                    glm::dot(footholdDelta, comp._test4Forward),
                    glm::dot(footholdDelta, comp._test4Right),
                    physicalSwingFootRotationOk
                        ? horizontalYawDeg(physicalSwingFootWorld) : 0.0f,
                    horizontalYawDeg(nominalFootWorldRotation()));
                // The settle gate validates achieved support-to-support advance, so admit
                // the foothold in that same space. The filtered loss is measured from prior
                // physical landings; ignoring it here admitted plans that were reachable
                // analytically but already predicted to finish below the 6 cm invariant.
                const float predictedAchievedAdvance =
                    plannedSupportAdvance - trackingReserve;
                const float requiredPlannedAdvance =
                    minimumAdvance + trackingReserve;
                const bool footholdAccepted = predictedAchievedAdvance + 0.0005f
                    >= minimumAdvance;
                spdlog::info(
                    "[LocoTest7] FOOTHOLD_GATE result={} planned={:.3f}m "
                    "predicted={:.3f}m required={:.3f}m minimum={:.3f}m "
                    "baseReserve={:.3f}m trackingReserve={:.3f}m",
                    footholdAccepted ? "PASS" : "FAIL",
                    plannedSupportAdvance, predictedAchievedAdvance,
                    requiredPlannedAdvance,
                    minimumAdvance, baseTrackingReserve, trackingReserve);
                return footholdAccepted;
            }
            return plannedForward >= 0.149f;
        };

        auto abortSequence = [&](const char* reason) {
            if (comp._test4Phase == kAbort || comp._test4Phase == kIdle) return;
            const int abortedPhase = comp._test4Phase;
            comp._test4Aborted = true;
            comp._test4Phase = kAbort;
            comp._test4PhaseTime = 0.0f;
            comp._test4SettleTime = 0.0f;
            comp._test4AirborneTime = 0.0f;
            if (continuousEnabled) comp._test7Running = false;
            if (gameplayCommand) {
                comp._runtimeRestartBlocked = true;
                spdlog::warn(
                    "[LocoRuntime] RESTART_BLOCKED reason=abort "
                    "action=release-movement-before-retry");
            }
            spdlog::warn(
                "[LocoTest4] ABORT {} clear={:.3f} forward={:.3f} contact={} "
                "target=(h={:.3f},fwd={:+.3f},lat={:+.3f},y={:.3f}) "
                "normalY={:.2f} vy=(api={:+.3f},fd={:+.3f}) upY={:+.2f} "
                "contactLocal=({:+.3f},{:+.3f},{:+.3f}) "
                "drift=({:.3f},{:.3f}) tilt={:.1f}",
                reason, comp._test4Clearance, comp._test4ForwardTravel,
                swingContactNow ? "yes" : "no",
                comp._test4HorizontalTargetError,
                comp._test4ForwardTargetError,
                comp._test4LateralTargetError,
                comp._test4VerticalTargetError,
                contactNormal.y, swingVelocity.y,
                comp._test4MeasuredVelocity.y, comp._test4FootUpY,
                comp._test4ContactLocal.x, comp._test4ContactLocal.y,
                comp._test4ContactLocal.z, comp._test4StanceDrift,
                comp._test4PlantDrift, tiltDeg);
            if (continuousEnabled && comp._test7IkPlanHipValid
                && abortedPhase >= kTakeoff && abortedPhase <= kSettle) {
                spdlog::warn(
                    "[LocoTest7] IK_ABORT phase={} "
                    "reach=(requested={:.3f},clamped={:.3f},max={:.3f},physical={:.3f}) "
                    "reachShortfall=({:.3f}m,fwd={:+.3f}m) "
                    "hipMove=(fwd={:+.3f},lat={:+.3f},y={:+.3f})m "
                    "hipEnvelopeClamp={:.1f}deg commandLag=(hip={:.1f},knee={:.1f})deg "
                    "kneeBend={:.1f}deg footError=(h={:.3f},fwd={:+.3f},lat={:+.3f})m "
                    "footControl=(corr={:.3f}m,fwd={:+.3f}m,"
                    "targetSpeed={:.3f}m/s,level={:.2f})",
                    abortedPhase,
                    comp._test7IkRequestedReach,
                    comp._test7IkClampedReach,
                    comp._test7IkMaxReach,
                    comp._test7IkPhysicalReach,
                    comp._test7IkReachShortfall,
                    comp._test7IkReachShortfallForward,
                    comp._test7IkHipTravelForward,
                    comp._test7IkHipTravelLateral,
                    comp._test7IkHipTravelVertical,
                    comp._test7IkHipEnvelopeClampDeg,
                    comp._test7IkHipCommandLagDeg,
                    comp._test7IkKneeCommandLagDeg,
                    comp._test7IkKneeBendDeg,
                    comp._test4HorizontalTargetError,
                    comp._test4ForwardTargetError,
                    comp._test4LateralTargetError,
                    comp._test7FootCorrection,
                    comp._test7FootCorrectionForward,
                    comp._test7FootTargetSpeed,
                    comp._test7SoleLevelBlend);
            }
        };

        const float comError = comp._test4TargetLateral - comp._test4ComLateral;
        const float forwardComError = glm::dot(
            comp._test4SupportTarget - rag._locomotionCOM,
            comp._test4Forward);
        const float forwardComSpeed = glm::dot(
            rag._locomotionCOMVel, comp._test4Forward);
        constexpr float kTest7WeightShiftForwardTolerance = 0.015f;
        constexpr float kTest7WeightShiftForwardSpeedTolerance = 0.010f;
        const float weightShiftMinimumTime = glm::max(
            comp.test2ShiftTime, 0.01f);
        // Valid shifts in the current 1.5 Hz setup can need a little over three
        // seconds to settle. Leave margin above that observed range, but never wait
        // forever on an unreachable predicate again.
        const float test7WeightShiftTimeout = glm::max(
            weightShiftMinimumTime * 3.0f, weightShiftMinimumTime + 3.0f);
        const bool weightShiftCommandReady =
            std::abs(comp._test4ComCommand - comp._test4SupportSide) < 0.01f;
        const bool weightShiftLateralReady = std::abs(comError) < 0.01f;
        const bool weightShiftForwardReady = !continuousEnabled
            || std::abs(forwardComError) < kTest7WeightShiftForwardTolerance;
        const bool weightShiftForwardSpeedReady = !continuousEnabled
            || std::abs(forwardComSpeed)
                < kTest7WeightShiftForwardSpeedTolerance;
        const bool weightShiftContactsReady =
            comp._test4ContactL && comp._test4ContactR;
        const bool weightShiftTimeReady =
            comp._test4PhaseTime >= weightShiftMinimumTime;
        if (comp._test4Phase == kWeightShift
            && weightShiftCommandReady
            && weightShiftLateralReady
            && weightShiftForwardReady
            && weightShiftForwardSpeedReady
            && weightShiftContactsReady
            && weightShiftTimeReady) {
            captureSwing();
            if (planFoothold()) {
                comp._test4Phase = kTakeoff;
                comp._test4PhaseTime = 0.0f;
                comp._test4PrevSwingContact = true;
                comp._test7TakeoffContactRecoveryTime = 0.0f;
                comp._test7TakeoffContactRecoveryActive = false;
                spdlog::info("[LocoTest4] phase=TAKEOFF");
                if (continuousEnabled) {
                    spdlog::info(
                        "[LocoTest7] STANCE_COM_ALIGNMENT target={:+.3f}m "
                        "achieved={:+.3f}m error={:+.3f}m "
                        "forwardSpeed={:+.3f}m/s trackingLoss={:.3f}m",
                        test7StanceForwardTarget,
                        glm::dot(rag._locomotionCOM - comp._test4ComBaseline,
                                 comp._test4Forward),
                        forwardComError,
                        forwardComSpeed,
                        comp._test7SettledTrackingLoss);
                }
            } else {
                abortSequence(continuousEnabled
                    ? "latched foothold lacked support-advance tracking reserve"
                    : "latched foothold fell below 15 cm after reach clamp");
            }
        } else if (continuousEnabled
                   && comp._test4Phase == kWeightShift
                   && comp._test4PhaseTime >= test7WeightShiftTimeout) {
            spdlog::warn(
                "[LocoTest7] WEIGHT_SHIFT_TIMEOUT step={} time={:.2f}/{:.2f}s "
                "predicates=(command={} error={:+.3f}/0.010,"
                "lateral={} error={:+.3f}/0.010m,"
                "forward={} error={:+.3f}/0.015m,"
                "forwardSpeed={} speed={:+.3f}/0.010m/s,"
                "contacts={} leftContact={} rightContact={},"
                "minimumTime={} elapsed={:.2f}/{:.2f}s)",
                comp._test6StepIndex,
                comp._test4PhaseTime, test7WeightShiftTimeout,
                weightShiftCommandReady ? "PASS" : "FAIL",
                comp._test4ComCommand - comp._test4SupportSide,
                weightShiftLateralReady ? "PASS" : "FAIL", comError,
                weightShiftForwardReady ? "PASS" : "FAIL", forwardComError,
                weightShiftForwardSpeedReady ? "PASS" : "FAIL",
                forwardComSpeed,
                weightShiftContactsReady ? "PASS" : "FAIL",
                comp._test4ContactL ? "PASS" : "FAIL",
                comp._test4ContactR ? "PASS" : "FAIL",
                weightShiftTimeReady ? "PASS" : "FAIL",
                comp._test4PhaseTime, weightShiftMinimumTime);
            abortSequence("weight-shift readiness timed out");
        }

        const float takeoffHeight = glm::clamp(
            comp.test3TakeoffHeight, 0.040f,
            glm::max(comp.test4SwingHeight, 0.041f));
        glm::vec3 desiredFoot = comp._test4SwingStart;
        if (comp._test4Phase == kTakeoff) {
            desiredFoot.y += takeoffHeight;
            const float releaseClearance = glm::max(0.040f, takeoffHeight * 0.75f);
            const bool recoverableTakeoffContact = continuousEnabled
                && swingContactNow && comp._test4Clearance >= 0.040f;
            if (recoverableTakeoffContact) {
                if (!comp._test7TakeoffContactRecoveryActive) {
                    spdlog::info(
                        "[LocoTest7] TAKEOFF_CONTACT recovery=BEGIN clear={:.3f}m "
                        "forward={:+.3f}m tilt={:.1f} upY={:+.2f} "
                        "contactLocal=({:+.3f},{:+.3f},{:+.3f})",
                        comp._test4Clearance, comp._test4ForwardTravel,
                        tiltDeg, comp._test4FootUpY,
                        comp._test4ContactLocal.x, comp._test4ContactLocal.y,
                        comp._test4ContactLocal.z);
                }
                comp._test7TakeoffContactRecoveryActive = true;
            } else if (comp._test7TakeoffContactRecoveryActive
                       && !swingContactNow) {
                spdlog::info(
                    "[LocoTest7] TAKEOFF_CONTACT recovery=PASS duration={:.3f}s "
                    "clear={:.3f}m forward={:+.3f}m",
                    comp._test7TakeoffContactRecoveryTime,
                    comp._test4Clearance, comp._test4ForwardTravel);
                comp._test7TakeoffContactRecoveryActive = false;
            }
            const bool takeoffRecoveryStarted =
                comp._test7TakeoffContactRecoveryTime > 0.0f
                || recoverableTakeoffContact;
            if (takeoffRecoveryStarted) {
                comp._test7TakeoffContactRecoveryTime += dt;
                // A high foot center with a live ground manifold means a toe or heel edge
                // is still down. Keep the command vertical until that edge opens instead of
                // feeding the contact into the forward swing trajectory.
                desiredFoot.y += 0.025f;
            }
            const bool airborneEvidence = !swingContactNow
                && comp._test4Clearance >= releaseClearance;
            comp._test4AirborneTime = airborneEvidence
                ? comp._test4AirborneTime + dt : 0.0f;
            if (comp._test4AirborneTime >= 0.05f) {
                comp._test4ArcStart = desiredFoot;
                comp._test4Phase = kSwing;
                comp._test4PhaseTime = 0.0f;
                comp._test4TrajectoryT = 0.0f;
                comp._test7TakeoffContactRecoveryTime = 0.0f;
                comp._test7TakeoffContactRecoveryActive = false;
                comp._test7SwingRecontactTime = 0.0f;
                spdlog::info(
                    "[LocoTest4] phase=SWING released clearance={:.3f} airborneFor={:.3f}",
                    comp._test4Clearance, comp._test4AirborneTime);
            } else {
                const float takeoffDeadline =
                    glm::max(comp.test3TakeoffTime, 0.01f)
                    + (takeoffRecoveryStarted ? 0.15f : 0.0f);
                if (comp._test4PhaseTime >= takeoffDeadline) {
                    abortSequence(takeoffRecoveryStarted
                        ? "takeoff contact persisted through recovery window"
                        : "takeoff did not open swing contact");
                }
            }
        }

        const glm::vec3 hoverTarget = comp._test4Foothold
            + glm::vec3(0.0f, glm::max(comp.test4ArrivalHeight, 0.03f), 0.0f);
        auto trajectoryPoint = [&](float t) {
            t = glm::clamp(t, 0.0f, 1.0f);
            glm::vec3 point = glm::mix(
                comp._test4ArcStart, hoverTarget, smoothstep(t));
            const float apexY = glm::max(comp._test4ArcStart.y, hoverTarget.y)
                + glm::max(comp.test4SwingHeight - comp.test4ArrivalHeight, 0.02f);
            point.y = t < 0.5f
                ? glm::mix(comp._test4ArcStart.y, apexY, smoothstep(t * 2.0f))
                : glm::mix(apexY, hoverTarget.y,
                           smoothstep((t - 0.5f) * 2.0f));
            return point;
        };

        auto acceptTouchdown = [&]() {
            comp._test4TouchdownAccepted = true;
            comp._test4TouchdownPlant = swingFoot;
            comp._test4TouchdownVy = swingVelocity.y;
            comp._test4TouchdownNormalY = contactNormal.y;
            comp._test4PlantDrift = 0.0f;
            comp._test4MaxPlantDrift = 0.0f;
            // Retain the final landing IK command briefly while contact settles. Capturing
            // the first-impact joint pose immediately allowed the sole to rock backward.
            comp._test4PlantPoseCaptured = false;
            comp._test4PlantAcquireStableTime = 0.0f;
            comp._test4Phase = kSettle;
            comp._test4PhaseTime = 0.0f;
            comp._test4SettleTime = 0.0f;
            spdlog::info(
                "[LocoTest4] TOUCHDOWN accepted foot=({:+.3f},{:+.3f},{:+.3f}) "
                "target=(h={:.3f},fwd={:+.3f},lat={:+.3f},y={:.3f}) "
                "vy=(api={:+.3f},fd={:+.3f}) normal=({:+.2f},{:+.2f},{:+.2f}) "
                "upY={:+.2f} contactWorld=({:+.3f},{:+.3f},{:+.3f}) "
                "contactLocal=({:+.3f},{:+.3f},{:+.3f})",
                swingFoot.x, swingFoot.y, swingFoot.z,
                comp._test4HorizontalTargetError,
                comp._test4ForwardTargetError,
                comp._test4LateralTargetError,
                comp._test4VerticalTargetError,
                swingVelocity.y, comp._test4MeasuredVelocity.y,
                contactNormal.x, contactNormal.y, contactNormal.z,
                comp._test4FootUpY,
                comp._test4ContactPoint.x, comp._test4ContactPoint.y,
                comp._test4ContactPoint.z,
                comp._test4ContactLocal.x, comp._test4ContactLocal.y,
                comp._test4ContactLocal.z);
        };

        auto evaluateTouchdown = [&](const char* stage, float descentProgress) {
            const float minNormalY = glm::clamp(
                comp.test4TouchdownMinNormalY, 0.35f, 1.0f);
            const float maxVerticalSpeed = continuousEnabled
                ? glm::max(comp.test4TouchdownMaxVerticalSpeed, 0.30f)
                : glm::max(comp.test4TouchdownMaxVerticalSpeed, 0.05f);
            const float horizontalTolerance = glm::max(
                comp.test4TargetTolerance, 0.01f);
            // Test 7 can touch a few frames early as the support-relative target settles.
            // Once descent has genuinely begun, a near-ground, accurately tracked, slow,
            // upward-facing contact is stronger evidence than a brittle time boundary.
            const float minimumProgress = continuousEnabled ? 0.45f : 0.70f;
            const float verticalTolerance = continuousEnabled ? 0.035f : 0.030f;
            const bool progressOk = descentProgress >= minimumProgress;
            const bool normalOk = contactNormal.y >= minNormalY;
            const bool velocityOk = std::abs(swingVelocity.y) <= maxVerticalSpeed;
            const bool horizontalOk =
                comp._test4HorizontalTargetError <= horizontalTolerance;
            const bool verticalOk =
                comp._test4VerticalTargetError <= verticalTolerance;
            spdlog::info(
                "[LocoTest4] TOUCHDOWN_CHECK stage={} result={} "
                "progress={:.2f}/{:.2f}[{}] normalY={:.2f}/{:.2f}[{}] "
                "absVy={:.3f}/{:.3f}[{}] fdVy={:+.3f} "
                "horizontal={:.3f}/{:.3f}[{}] fwd={:+.3f} lat={:+.3f} "
                "vertical={:.3f}/{:.3f}[{}] upY={:+.2f} "
                "contactLocal=({:+.3f},{:+.3f},{:+.3f})",
                stage,
                progressOk && normalOk && velocityOk && horizontalOk && verticalOk
                    ? "PASS" : "FAIL",
                descentProgress, minimumProgress, progressOk ? "ok" : "FAIL",
                contactNormal.y, minNormalY, normalOk ? "ok" : "FAIL",
                std::abs(swingVelocity.y), maxVerticalSpeed,
                velocityOk ? "ok" : "FAIL", comp._test4MeasuredVelocity.y,
                comp._test4HorizontalTargetError, horizontalTolerance,
                horizontalOk ? "ok" : "FAIL",
                comp._test4ForwardTargetError, comp._test4LateralTargetError,
                comp._test4VerticalTargetError, verticalTolerance,
                verticalOk ? "ok" : "FAIL",
                comp._test4FootUpY,
                comp._test4ContactLocal.x, comp._test4ContactLocal.y,
                comp._test4ContactLocal.z);
            if (progressOk && normalOk && velocityOk && horizontalOk && verticalOk) {
                acceptTouchdown();
            } else if (!progressOk) {
                abortSequence("touchdown occurred before safe descent window");
            } else if (!normalOk) {
                abortSequence("touchdown normal was below threshold");
            } else if (!velocityOk) {
                abortSequence("touchdown vertical speed exceeded threshold");
            } else if (!horizontalOk) {
                abortSequence("touchdown horizontal target error exceeded tolerance");
            } else {
                abortSequence("touchdown vertical target error exceeded 3 cm");
            }
        };

        const bool touchdownEdge = swingContactNow && !comp._test4PrevSwingContact;
        if (comp._test4Phase == kSwing) {
            const float swingProgress = glm::clamp(
                comp._test4PhaseTime / glm::max(comp.test4SwingTime, 0.05f),
                0.0f, 1.0f);
            comp._test4TrajectoryT = 0.70f * swingProgress;
            desiredFoot = trajectoryPoint(swingProgress);
            const bool earlySwingContact = swingContactNow
                && comp._test4PhaseTime >= 0.10f;
            const bool recoverableRecontact = continuousEnabled
                && earlySwingContact && comp._test4Clearance >= 0.040f;
            if (recoverableRecontact) {
                if (comp._test7SwingRecontactTime <= 0.0f) {
                    spdlog::info(
                        "[LocoTest7] SWING_RECONTACT recovery=BEGIN clear={:.3f}m "
                        "forward={:+.3f}m vy={:+.3f}m/s contactLocal=({:+.3f},{:+.3f},{:+.3f})",
                        comp._test4Clearance, comp._test4ForwardTravel,
                        swingVelocity.y, comp._test4ContactLocal.x,
                        comp._test4ContactLocal.y, comp._test4ContactLocal.z);
                }
                comp._test7SwingRecontactTime += dt;
                // Keep opening vertical clearance while a toe/heel edge releases.
                desiredFoot.y = glm::max(
                    desiredFoot.y,
                    comp._test4SwingStart.y
                        + glm::max(comp.test4SwingHeight, takeoffHeight));
                if (comp._test7SwingRecontactTime >= 0.10f)
                    abortSequence("early swing contact persisted through recovery window");
            } else {
                if (continuousEnabled && comp._test7SwingRecontactTime > 0.0f
                    && !swingContactNow) {
                    spdlog::info(
                        "[LocoTest7] SWING_RECONTACT recovery=PASS duration={:.3f}s "
                        "clear={:.3f}m forward={:+.3f}m",
                        comp._test7SwingRecontactTime,
                        comp._test4Clearance, comp._test4ForwardTravel);
                }
                comp._test7SwingRecontactTime = 0.0f;
                if (earlySwingContact)
                    abortSequence("swing contacted before hover arrival");
            }
            if (comp._test4Phase == kSwing && swingProgress >= 1.0f) {
                comp._test4Phase = kArrival;
                comp._test4PhaseTime = 0.0f;
                comp._test4ArrivalStableTime = 0.0f;
                desiredFoot = hoverTarget;
                spdlog::info(
                    "[LocoTest4] phase=ARRIVAL target=({:+.3f},{:+.3f},{:+.3f}) "
                    "horizontal={:.3f} fwd={:+.3f} lat={:+.3f}",
                    hoverTarget.x, hoverTarget.y, hoverTarget.z,
                    comp._test4HorizontalTargetError,
                    comp._test4ForwardTargetError,
                    comp._test4LateralTargetError);
            }
        } else if (comp._test4Phase == kArrival) {
            comp._test4TrajectoryT = 0.70f;
            desiredFoot = hoverTarget;
            const float arrivalTolerance = glm::max(
                comp.test4ArrivalTolerance,
                continuousEnabled ? 0.025f : 0.01f);
            constexpr float kTest7SoleArrivalToleranceDeg = 10.0f;
            const float arrivalVerticalError = std::abs(
                swingFoot.y - hoverTarget.y);
            const bool soleAligned = !continuousEnabled
                || comp._test7SoleAngularErrorDeg
                    <= kTest7SoleArrivalToleranceDeg;
            const bool arrivalWithinTolerance =
                comp._test4HorizontalTargetError <= arrivalTolerance
                && arrivalVerticalError <= 0.025f
                && soleAligned;
            comp._test4ArrivalStableTime = arrivalWithinTolerance
                ? comp._test4ArrivalStableTime + dt : 0.0f;
            const bool recoverableArrivalContact = continuousEnabled
                && swingContactNow && comp._test4Clearance >= 0.040f;
            if (recoverableArrivalContact) {
                // The normal arrival target may already be slightly below the center of a
                // pitched sole whose long edge touched first. Re-open vertical clearance
                // while the horizontal/leveling feedback catches up instead of continuing
                // to pull that edge into the ground for the duration of the grace window.
                desiredFoot.y = glm::max(
                    hoverTarget.y, swingFoot.y + 0.030f);
                if (comp._test7SwingRecontactTime <= 0.0f) {
                    spdlog::info(
                        "[LocoTest7] ARRIVAL_RECONTACT recovery=BEGIN "
                        "clear={:.3f}m error=(h={:.3f},fwd={:+.3f})m "
                        "soleError={:.1f}/{:.1f}deg "
                        "footControl=(corr={:.3f}m,level={:.2f})",
                        comp._test4Clearance,
                        comp._test4HorizontalTargetError,
                        comp._test4ForwardTargetError,
                        comp._test7SoleAngularErrorDeg,
                        kTest7SoleArrivalToleranceDeg,
                        comp._test7FootCorrection,
                        comp._test7SoleLevelBlend);
                }
                comp._test7SwingRecontactTime += dt;
                comp._test4ArrivalStableTime = 0.0f;
                // The measured ankle needs roughly a quarter second to unwind its final
                // 20-25 degrees. Keep this bounded, but do not declare failure before the
                // powered joint has had one physically plausible convergence interval.
                if (comp._test7SwingRecontactTime >= 0.35f)
                    abortSequence("arrival contact persisted through recovery window");
            } else if (swingContactNow) {
                abortSequence("arrival hold contacted before descent");
            } else {
                if (continuousEnabled && comp._test7SwingRecontactTime > 0.0f) {
                    spdlog::info(
                        "[LocoTest7] ARRIVAL_RECONTACT recovery=PASS "
                        "duration={:.3f}s clear={:.3f}m "
                        "error=(h={:.3f},fwd={:+.3f})m soleError={:.1f}deg",
                        comp._test7SwingRecontactTime,
                        comp._test4Clearance,
                        comp._test4HorizontalTargetError,
                        comp._test4ForwardTargetError,
                        comp._test7SoleAngularErrorDeg);
                }
                comp._test7SwingRecontactTime = 0.0f;
            }
            if (comp._test4Phase == kArrival && !swingContactNow
                && comp._test4ArrivalStableTime >= glm::max(
                           comp.test4ArrivalSettleTime, 0.05f)) {
                comp._test4Phase = kDescent;
                comp._test4PhaseTime = 0.0f;
                spdlog::info(
                    "[LocoTest4] phase=DESCENT arrival h={:.3f}/{:.3f} "
                    "fwd={:+.3f} lat={:+.3f} y={:.3f} "
                    "soleError={:.1f}/{:.1f}deg stable={:.3f}s",
                    comp._test4HorizontalTargetError, arrivalTolerance,
                    comp._test4ForwardTargetError,
                    comp._test4LateralTargetError,
                    arrivalVerticalError,
                    comp._test7SoleAngularErrorDeg,
                    kTest7SoleArrivalToleranceDeg,
                    comp._test4ArrivalStableTime);
            } else if (comp._test4Phase == kArrival && !swingContactNow
                       && comp._test4PhaseTime >= glm::max(
                           comp.test4ArrivalTimeout, 0.10f)) {
                spdlog::warn(
                    "[LocoTest4] ARRIVAL_CHECK result=FAIL horizontal={:.3f}/{:.3f}[{}] "
                    "hoverY={:.3f}/0.025[{}] stable={:.3f}/{:.3f}s[{}] "
                    "soleError={:.1f}/{:.1f}[{}] fwd={:+.3f} lat={:+.3f}",
                    comp._test4HorizontalTargetError, arrivalTolerance,
                    comp._test4HorizontalTargetError <= arrivalTolerance ? "ok" : "FAIL",
                    arrivalVerticalError,
                    arrivalVerticalError <= 0.025f ? "ok" : "FAIL",
                    comp._test4ArrivalStableTime,
                    glm::max(comp.test4ArrivalSettleTime, 0.05f),
                    comp._test4ArrivalStableTime >= glm::max(
                        comp.test4ArrivalSettleTime, 0.05f) ? "ok" : "FAIL",
                    comp._test7SoleAngularErrorDeg,
                    kTest7SoleArrivalToleranceDeg,
                    soleAligned ? "ok" : "FAIL",
                    comp._test4ForwardTargetError,
                    comp._test4LateralTargetError);
                if (comp._test4HorizontalTargetError > arrivalTolerance)
                    abortSequence("hover horizontal arrival did not converge before timeout");
                else if (arrivalVerticalError > 0.025f)
                    abortSequence("hover vertical arrival did not converge before timeout");
                else if (!soleAligned)
                    abortSequence("hover sole orientation did not converge before timeout");
                else
                    abortSequence("hover arrival did not remain within tolerance long enough");
            }
        } else if (comp._test4Phase == kDescent) {
            const float descentProgress = glm::clamp(
                comp._test4PhaseTime / glm::max(comp.test4DescentTime, 0.05f),
                0.0f, 1.0f);
            comp._test4TrajectoryT = 0.70f + 0.30f * descentProgress;
            desiredFoot = glm::mix(
                hoverTarget, comp._test4Foothold, smoothstep(descentProgress));
            if (touchdownEdge) {
                evaluateTouchdown("DESCENT", descentProgress);
            } else if (descentProgress >= 1.0f) {
                comp._test4Phase = kTouchdownWait;
                comp._test4PhaseTime = 0.0f;
                desiredFoot = comp._test4Foothold;
                spdlog::info("[LocoTest4] phase=TOUCHDOWN_WAIT");
            }
        } else if (comp._test4Phase == kTouchdownWait) {
            comp._test4TrajectoryT = 1.0f;
            desiredFoot = comp._test4Foothold;
            if (touchdownEdge) {
                evaluateTouchdown("TOUCHDOWN_WAIT", 1.0f);
            } else if (comp._test4PhaseTime >= glm::max(comp.test4PlantTimeout, 0.10f)) {
                abortSequence("touchdown contact timed out");
            }
        } else if (comp._test4Phase >= kSettle
                   && comp._test4Phase <= kComplete) {
            comp._test4TrajectoryT = 1.0f;
            desiredFoot = comp._test4Foothold;
        }
        comp._test4TargetError = glm::length(swingFoot - desiredFoot);

        if (transferEnabled && comp._test4Phase >= kTransfer
            && comp._test4Phase <= kHold) {
            comp._test5ContactLossTime = swingContactNow && stanceContactNow
                ? 0.0f : comp._test5ContactLossTime + dt;
        } else {
            comp._test5ContactLossTime = 0.0f;
        }

        if (comp._test4Phase >= kTakeoff && comp._test4Phase <= kSettle
            && !stanceContactNow) {
            abortSequence("stance contact was lost");
        } else if (comp._test4Phase >= kTakeoff && comp._test4Phase <= kSettle
                   && comp._test4StanceDrift > 0.040f) {
            abortSequence("stance foot exceeded 4 cm drift");
        } else if (comp._test4Phase >= kTakeoff && comp._test4Phase <= kSettle
                   && tiltDeg >= 30.0f) {
            abortSequence("tilt reached 30 degrees");
        } else if (comp._test4Phase == kSwing
                   && comp._test4TrajectoryT < 0.65f
                   && comp._test4PhaseTime >= 0.10f
                   && comp._test4Clearance < 0.030f) {
            abortSequence("airborne swing lost clearance");
        } else if (comp._test4Phase == kSettle
                   && comp._test4PlantPoseCaptured
                   && comp._test4PlantDrift > 0.040f) {
            abortSequence("new plant exceeded 4 cm drift");
        } else if (transferEnabled && comp._test4Phase >= kTransfer
                   && comp._test4Phase <= kHold
                   && comp._test5ContactLossTime > 0.05f) {
            abortSequence("foot contact was lost during support transfer");
        } else if (transferEnabled && transferOrHold
                   && (comp._test4PlantDrift > 0.040f
                       || (comp._test4StanceDrift > 0.040f
                           && !test7OldSupportUnloaded))) {
            if (continuousEnabled) {
                spdlog::warn(
                    "[LocoTest7] TRANSFER_DRIFT_ABORT oldSupport={} oldDrift={:.3f}m "
                    "newSupport={} newDrift={:.3f}m oldUnloaded={}",
                    comp._test4SupportSide < 0 ? "LEFT" : "RIGHT",
                    comp._test4StanceDrift,
                    comp._test4SupportSide < 0 ? "RIGHT" : "LEFT",
                    comp._test4PlantDrift,
                    test7OldSupportUnloaded ? "yes" : "no");
            }
            abortSequence("plant drift exceeded 4 cm during support transfer");
        } else if (transferEnabled && comp._test4Phase >= kTransfer
                   && comp._test4Phase <= kHold && tiltDeg >= 30.0f) {
            abortSequence("tilt reached 30 degrees during support transfer");
        }

        const bool liftAssistActive = comp._test4Phase >= kTakeoff
                                   && comp._test4Phase <= kDescent;
        if (liftAssistActive) {
            float liftFade = 1.0f;
            if (comp._test4Phase == kSwing)
                liftFade = 1.0f - 0.5f * smoothstep(
                    (comp._test4TrajectoryT - 0.40f) / 0.30f);
            else if (comp._test4Phase == kArrival)
                liftFade = 0.5f;
            else if (comp._test4Phase == kDescent) {
                const float descentProgress = glm::clamp(
                    (comp._test4TrajectoryT - 0.70f) / 0.30f, 0.0f, 1.0f);
                liftFade = 0.5f * (1.0f - smoothstep(descentProgress / 0.70f));
            }
            rag.locomotionLiftBone = swing->footIdx;
            rag.locomotionLiftTargetY = desiredFoot.y;
            rag.locomotionLiftFrequency = glm::max(comp.test3TakeoffFrequency, 0.0f);
            rag.locomotionLiftMaxForce = glm::max(comp.test3TakeoffMaxForce, 0.0f)
                                       * glm::clamp(liftFade, 0.0f, 1.0f);
        } else {
            rag.locomotionLiftBone = -1;
            rag.locomotionLiftMaxForce = 0.0f;
        }

        auto solveSwingIK = [&](const glm::vec3& targetFoot) {
            glm::vec3 controlledFoot = targetFoot;
            comp._test7FootCorrection = 0.0f;
            comp._test7FootCorrectionForward = 0.0f;
            comp._test7FootTargetSpeed = 0.0f;
            if (continuousEnabled
                && comp._test4Phase >= kSwing
                && comp._test4Phase <= kArrival) {
                // Joint-space motors are closed-loop, but the sole previously had no
                // horizontal task-space feedback. _test4DesiredFoot still contains the
                // previous frame here, so lead the moving target by its actual velocity
                // and add a bounded correction from the measured sole-center error. The
                // correction collapses to zero when the physical foot catches the nominal
                // path, so the admitted foothold remains the equilibrium rather than being
                // permanently displaced.
                glm::vec3 positionError = targetFoot - swingFoot;
                positionError.y = 0.0f;
                glm::vec3 targetVelocity(0.0f);
                if (dt > 1e-6f) {
                    targetVelocity = (targetFoot - comp._test4DesiredFoot) / dt;
                    targetVelocity.y = 0.0f;
                }
                glm::vec3 footCorrection = positionError
                    * glm::clamp(comp.test7FootPositionGain, 0.0f, 1.0f)
                    + targetVelocity
                    * glm::max(comp.test7FootVelocityLeadTime, 0.0f);
                const float maximumCorrection = glm::max(
                    comp.test7MaxFootCorrection, 0.0f);
                const float correctionLength = glm::length(footCorrection);
                if (correctionLength > maximumCorrection
                    && correctionLength > 1e-6f) {
                    footCorrection *= maximumCorrection / correctionLength;
                }
                controlledFoot += footCorrection;
                comp._test7FootCorrection = glm::length(footCorrection);
                comp._test7FootCorrectionForward = glm::dot(
                    footCorrection, comp._test4Forward);
                comp._test7FootTargetSpeed = glm::length(targetVelocity);
            }
            swing->desiredFoot = controlledFoot;
            const glm::vec3 hipPosition = physicalPosition(swing->hipIdx);
            float soleLevelBlend = 0.0f;
            if (continuousEnabled && swing->groundReferenceFootRotationValid) {
                if (comp._test4Phase == kSwing) {
                    soleLevelBlend = smoothstep(glm::clamp(
                        comp._test4PhaseTime
                            / glm::max(comp.test7SoleLevelTime, 0.10f),
                        0.0f, 1.0f));
                } else if (comp._test4Phase >= kArrival) {
                    soleLevelBlend = 1.0f;
                }
            }
            comp._test7SoleLevelBlend = soleLevelBlend;
            const glm::quat desiredFootWorld = glm::normalize(glm::slerp(
                glm::normalize(swing->plantedFootWorldRotation),
                nominalFootWorldRotation(), soleLevelBlend));
            glm::vec3 desiredAnkle = controlledFoot
                + ankleFromFootWorld(desiredFootWorld);
            const glm::vec3 requestedAnkle = desiredAnkle;
            glm::vec3 toTarget = desiredAnkle - hipPosition;
            const float requestedReach = glm::length(toTarget);
            const float upperLength = glm::length(skeleton.bones[swing->kneeIdx].localT);
            const float lowerLength = glm::length(skeleton.bones[swing->ankleIdx].localT);
            const float configuredReach = (upperLength + lowerLength)
                * glm::clamp(comp.proceduralMaxReach, 0.70f, 0.99f);
            const float maxReach = comp._test4ReachLimit > 1e-4f
                ? glm::max(configuredReach, comp._test4ReachLimit)
                : configuredReach;
            if (const float reach = glm::length(toTarget);
                reach > maxReach && reach > 1e-5f) {
                toTarget *= maxReach / reach;
                desiredAnkle = hipPosition + toTarget;
            }
            if (continuousEnabled) {
                const glm::vec3 physicalAnkle = physicalPosition(swing->ankleIdx);
                const glm::vec3 reachCorrection = requestedAnkle - desiredAnkle;
                const glm::vec3 hipTravel = comp._test7IkPlanHipValid
                    ? hipPosition - comp._test7IkPlanHip : glm::vec3(0.0f);
                comp._test7IkRequestedReach = requestedReach;
                comp._test7IkClampedReach = glm::length(toTarget);
                comp._test7IkMaxReach = maxReach;
                comp._test7IkPhysicalReach = glm::length(
                    physicalAnkle - hipPosition);
                comp._test7IkReachShortfall = glm::length(reachCorrection);
                comp._test7IkReachShortfallForward = glm::dot(
                    reachCorrection, comp._test4Forward);
                comp._test7IkHipTravelForward = glm::dot(
                    hipTravel, comp._test4Forward);
                comp._test7IkHipTravelLateral = glm::dot(
                    hipTravel, comp._test4Right);
                comp._test7IkHipTravelVertical = hipTravel.y;
            }
            const float distance = glm::clamp(glm::length(toTarget),
                std::abs(upperLength - lowerLength) + 1e-4f,
                upperLength + lowerLength - 1e-4f);
            if (upperLength <= 1e-4f || lowerLength <= 1e-4f || distance <= 1e-4f)
                return;

            const float includedCos = glm::clamp(
                (upperLength * upperLength + lowerLength * lowerLength
                 - distance * distance) / (2.0f * upperLength * lowerLength),
                -1.0f, 1.0f);
            const float kneeBend = glm::pi<float>() - std::acos(includedCos);
            if (continuousEnabled)
                comp._test7IkKneeBendDeg = glm::degrees(kneeBend);
            const float kneeDelta = kneeBend - swing->referenceKneeBend;
            const glm::quat kneeTarget = glm::normalize(
                swing->referenceKneeLocal
                * glm::angleAxis(kneeDelta, glm::normalize(swing->kneeHingeAxis)));

            const glm::vec3 worldForward = glm::normalize(toTarget);
            const glm::vec3 kneePole = nominalKneePoleWorld();
            glm::vec3 worldBend = kneePole
                - worldForward * glm::dot(kneePole, worldForward);
            if (glm::dot(worldBend, worldBend) < 1e-8f)
                worldBend = comp._test4Forward;
            worldBend -= worldForward * glm::dot(worldBend, worldForward);
            if (glm::dot(worldBend, worldBend) < 1e-8f)
                worldBend = comp._test4Right;
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
            const glm::quat parentWorld = ParentWorldRot(
                rag, skeleton, animator, entityWorld, swing->hipIdx);
            glm::quat hipTarget = glm::normalize(glm::conjugate(parentWorld) * hipWorld);
            const glm::quat unconstrainedHipTarget = hipTarget;
            Envelope hipEnvelope;
            hipEnvelope.twistAxis = swing->hipTwistAxis;
            hipEnvelope.swingNormalDeg = swing->hipSwingNormalDeg;
            hipEnvelope.swingPlaneDeg = swing->hipSwingPlaneDeg;
            hipEnvelope.twistMinDeg = swing->hipTwistMinDeg;
            hipEnvelope.twistMaxDeg = swing->hipTwistMaxDeg;
            hipTarget = ClampToEnvelope(hipEnvelope,
                skeleton.bones[swing->hipIdx].localR, hipTarget,
                comp.hipLimitMarginDeg);

            // The sole is terminal; preserving its old local pose lets pelvis/knee motion
            // rotate the entire foot away from the world-space foothold. Counter-rotate at
            // the powered ankle instead. The reference terminal relation remains fixed,
            // while the ankle target makes that relation produce the captured sole world
            // orientation after the commanded hip/knee solve.
            const glm::quat commandedHipWorld = glm::normalize(
                parentWorld * hipTarget);
            const glm::quat commandedKneeWorld = glm::normalize(
                commandedHipWorld * kneeTarget);
            bool physicalKneeWorldOk = false;
            const glm::quat measuredKneeWorld =
                Physics::GetRagdollBoneRotation(
                    rag, swing->kneeIdx, &physicalKneeWorldOk);
            // The ankle is a local motor beneath the physical knee. Building its target
            // under the commanded knee assumes upstream tracking is perfect and turns
            // knee lag into world-space sole pitch. Close that chain against the measured
            // parent so the local target still asks for the intended world-space sole.
            const glm::quat ankleParentWorld = continuousEnabled
                && physicalKneeWorldOk
                ? glm::normalize(measuredKneeWorld)
                : commandedKneeWorld;
            glm::quat ankleTarget = glm::normalize(
                glm::conjugate(ankleParentWorld)
                * desiredFootWorld
                * glm::conjugate(swing->referenceFootLocal));
            Envelope ankleEnvelope;
            ankleEnvelope.twistAxis = swing->ankleAxis;
            ankleEnvelope.swingNormalDeg = swing->ankleSwingNormalDeg;
            ankleEnvelope.swingPlaneDeg = swing->ankleSwingPlaneDeg;
            ankleEnvelope.twistMinDeg = swing->ankleTwistMinDeg;
            ankleEnvelope.twistMaxDeg = swing->ankleTwistMaxDeg;
            ankleTarget = ClampToEnvelope(ankleEnvelope,
                skeleton.bones[swing->ankleIdx].localR, ankleTarget,
                comp.hipLimitMarginDeg);

            if (continuousEnabled)
                comp._test7IkHipEnvelopeClampDeg = rotationDifferenceDeg(
                    unconstrainedHipTarget, hipTarget);

            const float alpha = 1.0f - std::exp(
                -dt / glm::max(comp.proceduralPoseResponse, 0.01f));
            swing->hipCommand = glm::normalize(
                glm::slerp(swing->hipCommand, hipTarget, alpha));
            swing->kneeCommand = glm::normalize(
                glm::slerp(swing->kneeCommand, kneeTarget, alpha));
            swing->ankleCommand = glm::normalize(
                glm::slerp(swing->ankleCommand, ankleTarget, alpha));
            if (continuousEnabled) {
                comp._test7IkHipCommandLagDeg = rotationDifferenceDeg(
                    swing->hipCommand, hipTarget);
                comp._test7IkKneeCommandLagDeg = rotationDifferenceDeg(
                    swing->kneeCommand, kneeTarget);
            }
            swing->footCommand = swing->referenceFootLocal;
        };

        const bool acquiringPlant = comp._test4Phase == kSettle
                                 && !comp._test4PlantPoseCaptured;
        const bool applySwingIK = (comp._test4Phase >= kTakeoff
                                && comp._test4Phase <= kTouchdownWait)
                               || acquiringPlant;
        if (applySwingIK) solveSwingIK(desiredFoot);
        // Preserve the nominal target as history only after IK has consumed the previous
        // frame. Writing it before solve made targetVelocity identically zero and silently
        // disabled the configured 80 ms feed-forward term.
        comp._test4DesiredFoot = desiredFoot;
        const bool holdTouchdownPose = comp._test4Phase >= kSettle
            && (comp._test4Phase <= kComplete
                || (continuousEnabled && comp._test4Phase == kStopping));
        const bool holdMultiStepBaseline = multiStepEnabled
            && comp._test6StepIndex >= 2 && comp._test4Phase == kWeightShift;
        const float poseWeight = glm::clamp(
            comp._poseBlend * comp.poseWeight, 0.0f, 1.0f);
        const float shutdownPoseBlend = continuousEnabled
            && comp._test4Phase == kStopping
            ? 1.0f - smoothstep(comp._test4PhaseTime
                / glm::max(comp.test7StopTime, 0.25f))
            : 1.0f;
        auto writeLegPose = [&](const Leg& leg) {
            auto shutdownTarget = [&](int bone, const glm::quat& walkingTarget) {
                if (shutdownPoseBlend >= 1.0f) return walkingTarget;
                return glm::normalize(glm::slerp(
                    skeleton.bones[bone].localR, walkingTarget,
                    shutdownPoseBlend));
            };
            BlendPose(animator, leg.hipIdx,
                      shutdownTarget(leg.hipIdx, leg.hipCommand), poseWeight);
            BlendPose(animator, leg.kneeIdx,
                      shutdownTarget(leg.kneeIdx, leg.kneeCommand), poseWeight);
            BlendPose(animator, leg.ankleIdx,
                      shutdownTarget(leg.ankleIdx, leg.ankleCommand), poseWeight);
            BlendPose(animator, leg.footIdx,
                      shutdownTarget(leg.footIdx, leg.footCommand), poseWeight);
        };
        if (applySwingIK || holdTouchdownPose || holdMultiStepBaseline)
            writeLegPose(*swing);
        if (multiStepEnabled && comp._test6StepIndex >= 2
            && comp._test4Phase >= kWeightShift
            && (comp._test4Phase <= kComplete
                || (continuousEnabled && comp._test4Phase == kStopping)))
            writeLegPose(*stance);

        if (comp._test4Phase == kSettle) {
            const float minimumSupportAdvance = glm::min(
                comp.test7MinStepLength, comp.test7MaxStepLength);
            if (!comp._test4PlantPoseCaptured) {
                const float plantSpeed = glm::length(swingVelocity);
                const float maxAcquireSpeed = glm::max(
                    comp.test4PlantAcquireMaxSpeed, 0.01f);
                const bool acquisitionKinematicallyStable =
                    swingContactNow && stanceContactNow
                    && plantSpeed <= maxAcquireSpeed;
                comp._test4PlantAcquireStableTime = acquisitionKinematicallyStable
                    ? comp._test4PlantAcquireStableTime + dt : 0.0f;
                if (comp._test4PlantAcquireStableTime >= glm::max(
                        comp.test4PlantAcquireTime, 0.05f)) {
                    const float retainedFromTarget =
                        comp._test7PlannedSupportAdvance
                        - glm::max(comp._test4ForwardTargetError, 0.0f);
                    const bool retainsMinimumAdvance = !continuousEnabled
                        || comp._test7AchievedSupportAdvance + 0.0005f
                            >= minimumSupportAdvance;
                    if (!retainsMinimumAdvance) {
                        spdlog::warn(
                            "[LocoTest7] LANDING_ADVANCE_CHECK result=FAIL "
                            "planned={:.3f} retainedFromTarget={:.3f} "
                            "achieved={:.3f}/{:.3f} forwardError={:+.3f} "
                            "stanceDrift={:.3f}",
                            comp._test7PlannedSupportAdvance,
                            retainedFromTarget,
                            comp._test7AchievedSupportAdvance,
                            minimumSupportAdvance,
                            comp._test4ForwardTargetError,
                            comp._test4StanceDrift);
                        abortSequence(
                            "stable landing did not retain minimum support advance");
                    } else {
                        // Test 7 keeps the converged analytic landing command. Copying the
                        // measured pose here promoted residual hip/ankle motor lag into the
                        // next equilibrium and made forward lean and toe pitch ratchet per
                        // step. Tests 4-6 retain their already-validated physical capture.
                        if (!continuousEnabled)
                            capturePhysicalLocalPose(*swing);
                        comp._test4PlantPoseCaptured = true;
                        comp._test4SettleTime = 0.0f;
                        spdlog::info(
                            "[LocoTest4] phase=SETTLE_HOLD acquire={:.3f}s "
                            "stable={:.3f}s forward={:.3f} plantDrift={:.3f} "
                            "speed={:.3f}/{:.3f}",
                            comp._test4PhaseTime, comp._test4PlantAcquireStableTime,
                            comp._test4ForwardTravel, comp._test4PlantDrift,
                            plantSpeed, maxAcquireSpeed);
                        if (continuousEnabled) {
                            spdlog::info(
                                "[LocoTest7] LANDING_ADVANCE_CHECK result=PASS "
                                "planned={:.3f} retainedFromTarget={:.3f} "
                                "achieved={:.3f}/{:.3f} forwardError={:+.3f} "
                                "stanceDrift={:.3f}",
                                comp._test7PlannedSupportAdvance,
                                retainedFromTarget,
                                comp._test7AchievedSupportAdvance,
                                minimumSupportAdvance,
                                comp._test4ForwardTargetError,
                                comp._test4StanceDrift);
                        }
                    }
                } else if (comp._test4PhaseTime >= glm::max(
                               comp.test4PlantAcquireTimeout, 0.20f)) {
                    spdlog::warn(
                        "[LocoTest4] PLANT_ACQUIRE result=FAIL contact={} "
                        "stance={} speed={:.3f}/{:.3f} stable={:.3f}/{:.3f}s "
                        "forward={:.3f} drift={:.3f}",
                        swingContactNow ? "yes" : "no",
                        stanceContactNow ? "yes" : "no",
                        plantSpeed, maxAcquireSpeed,
                        comp._test4PlantAcquireStableTime,
                        glm::max(comp.test4PlantAcquireTime, 0.05f),
                        comp._test4ForwardTravel, comp._test4PlantDrift);
                    abortSequence("new plant did not settle before acquisition timeout");
                }
            }
            const bool locksOff = rag.locomotionFootLockWeights[0] <= 0.001f
                               && rag.locomotionFootLockWeights[1] <= 0.001f
                               && rag._locomotionFootLockForce[0] <= 0.5f
                               && rag._locomotionFootLockForce[1] <= 0.5f;
            const bool stablePlant = comp._test4PlantPoseCaptured
                && comp._test4ContactL && comp._test4ContactR
                && glm::length(leftVelocity) < 0.15f
                && glm::length(rightVelocity) < 0.15f
                && horizontalSpeed < 0.15f
                && comp._test4StanceDrift <= 0.040f
                && comp._test4PlantDrift <= 0.040f
                && tiltDeg < 30.0f
                && std::abs(tiltDeg - comp._test4InitialTilt) <= 10.0f
                && !rag._locomotionSupportSaturated
                && rag.locomotionLiftBone < 0
                && rag._locomotionLiftForce <= 0.5f
                && locksOff
                && !comp._test4MotorSaturated;
            const bool stepDistanceValid = continuousEnabled
                ? comp._test7AchievedSupportAdvance + 0.0005f
                    >= minimumSupportAdvance
                : comp._test4ForwardTravel >= 0.149f;
            comp._test4SettleTime = stablePlant
                ? comp._test4SettleTime + dt : 0.0f;
            if (comp._test4SettleTime >= glm::max(
                    comp.test4ContactSettleTime, 0.30f)) {
                if (!stepDistanceValid) {
                    if (continuousEnabled) {
                        spdlog::warn(
                            "[LocoTest7] SUPPORT_ADVANCE_CHECK result=FAIL "
                            "planned={:.3f} achieved={:.3f}/{:.3f} "
                            "footTravel={:.3f} targetError={:.3f}",
                            comp._test7PlannedSupportAdvance,
                            comp._test7AchievedSupportAdvance,
                            minimumSupportAdvance,
                            comp._test4ForwardTravel,
                            comp._test4HorizontalTargetError);
                        abortSequence(
                            "settled step finished below minimum support advance");
                    } else {
                        abortSequence(
                            "settled step finished below 15 cm forward travel");
                    }
                } else if (transferEnabled) {
                    if (continuousEnabled) {
                        const float settledLoss = glm::max(
                            comp._test7PlannedSupportAdvance
                                - comp._test7AchievedSupportAdvance,
                            0.0f);
                        comp._test7SettledTrackingLoss = settledLoss
                            > comp._test7SettledTrackingLoss
                            ? settledLoss
                            : glm::mix(comp._test7SettledTrackingLoss,
                                       settledLoss, 0.25f);
                        const float nextReserve = glm::min(glm::max(
                            glm::min(glm::max(
                                comp.test4TargetTolerance * 0.5f,
                                0.010f), 0.025f),
                            comp._test7SettledTrackingLoss + 0.003f),
                            0.040f);
                        spdlog::info(
                            "[LocoTest7] TRACKING_FEEDBACK planned={:.3f}m "
                            "settled={:.3f}m loss={:.3f}m filtered={:.3f}m "
                            "nextReserve={:.3f}m",
                            comp._test7PlannedSupportAdvance,
                            comp._test7AchievedSupportAdvance,
                            settledLoss,
                            comp._test7SettledTrackingLoss,
                            nextReserve);
                    }
                    const float transferFraction = glm::clamp(
                        comp.test5SupportFraction, 0.70f, 1.0f);
                    glm::vec3 comStart = rag._locomotionCOM;
                    glm::vec3 newPlant = swingFoot;
                    comStart.y = comp._test4SupportTarget.y;
                    newPlant.y = comp._test4SupportTarget.y;
                    comp._test5TransferStartTarget = comp._test4SupportTarget;
                    const float forwardTransferFraction = continuousEnabled
                        ? 0.50f : transferFraction;
                    if (continuousEnabled) {
                        glm::vec3 forwardSupportPoint = glm::mix(
                            stanceFoot, swingFoot, forwardTransferFraction);
                        forwardSupportPoint.y = comp._test4SupportTarget.y;
                        // Double support should finish with the COM centered fore/aft
                        // between the planted feet. The following WEIGHT_SHIFT owns the
                        // deliberate move toward the new stance foot before it unloads the
                        // rear leg. Keeping these jobs separate prevents forward lean from
                        // accumulating at every role swap.
                        comp._test5TransferEndTarget = comStart
                            + comp._test4Right * glm::dot(
                                newPlant - comStart, comp._test4Right)
                                * transferFraction
                            + comp._test4Forward * glm::dot(
                                forwardSupportPoint - comStart,
                                comp._test4Forward);
                    } else {
                        comp._test5TransferEndTarget = glm::mix(
                            comStart, newPlant, transferFraction);
                    }
                    comp._test5TransferT = 0.0f;
                    comp._test5HoldStableTime = 0.0f;
                    comp._test5ContactLossTime = 0.0f;
                    comp._test4Phase = kTransfer;
                    comp._test4PhaseTime = 0.0f;
                    spdlog::info(
                        "[LocoTest{}] phase=TRANSFER oldSupport={} newSupport={} "
                        "start=({:+.3f},{:+.3f},{:+.3f}) "
                        "target=({:+.3f},{:+.3f},{:+.3f}) "
                        "fraction=(lateral={:.2f},forward={:.2f}) "
                        "axes={} shift=(lateral={:+.3f},forward={:+.3f}) time={:.2f}s",
                        comp.validationTest,
                        comp._test4SupportSide < 0 ? "LEFT" : "RIGHT",
                        comp._test4SupportSide < 0 ? "RIGHT" : "LEFT",
                        comp._test5TransferStartTarget.x,
                        comp._test5TransferStartTarget.y,
                        comp._test5TransferStartTarget.z,
                        comp._test5TransferEndTarget.x,
                        comp._test5TransferEndTarget.y,
                        comp._test5TransferEndTarget.z,
                        transferFraction,
                        forwardTransferFraction,
                        continuousEnabled
                            ? "LATERAL_SUPPORT_FORWARD_65_PERCENT"
                            : "COUPLED_TO_SUPPORT",
                        glm::dot(comp._test5TransferEndTarget - comStart,
                                 comp._test4Right),
                        glm::dot(comp._test5TransferEndTarget - comStart,
                                 comp._test4Forward),
                        glm::max(comp.test5TransferTime, 0.05f));
                } else {
                    comp._test4Phase = kComplete;
                    comp._test4PhaseTime = 0.0f;
                    spdlog::info(
                        "[LocoTest4] COMPLETE support={} forward={:.3f} "
                        "drift=({:.3f},{:.3f}) tilt=({:.1f}->{:.1f}, peak={:.1f}) "
                        "touchdownVy={:+.3f} normalY={:.2f} maxMotorRatio={:.2f}",
                        comp._test4SupportSide < 0 ? "LEFT" : "RIGHT",
                        comp._test4ForwardTravel,
                        comp._test4MaxStanceDrift, comp._test4MaxPlantDrift,
                        comp._test4InitialTilt, tiltDeg, comp._test4PeakTilt,
                        comp._test4TouchdownVy, comp._test4TouchdownNormalY,
                        comp._test4MaxMotorRatio);
                }
            }
        } else if (comp._test4Phase == kTransfer) {
            if (comp._test4PhaseTime >= glm::max(comp.test5TransferTime, 0.05f)) {
                comp._test4Phase = kHold;
                comp._test4PhaseTime = 0.0f;
                comp._test5HoldStableTime = 0.0f;
                spdlog::info(
                    "[LocoTest{}] phase=HOLD transfer=1.00 COMerr={:.3f} "
                    "distance=(old={:.3f},new={:.3f}) speed={:.3f} "
                    "contact=({},{}) drift=({:.3f},{:.3f}) tilt={:.1f}",
                    comp.validationTest, comp._test5ComError,
                    comp._test5ComToOldSupport,
                    comp._test5ComToNewSupport,
                    comp._test5ComHorizontalSpeed,
                    comp._test4ContactL ? "L" : "-",
                    comp._test4ContactR ? "R" : "-",
                    comp._test4StanceDrift, comp._test4PlantDrift, tiltDeg);
            }
        } else if (comp._test4Phase == kHold) {
            const float comTolerance = glm::max(comp.test5ComTolerance, 0.01f);
            const float newSupportRadius = glm::max(comTolerance, 0.065f);
            const bool comAtTarget = comp._test5ComError <= comTolerance;
            const bool insideNewSupport =
                comp._test5ComToNewSupport <= newSupportRadius;
            // We do not yet expose a per-foot normal impulse. Being decisively closer to
            // the new sole while both contacts remain is the geometric unload predicate;
            // Test 6 will then prove the old foot can become the next swing foot.
            const bool oldLegUnloaded = comp._test5ComToNewSupport + 0.020f
                                      < comp._test5ComToOldSupport;
            if (continuousEnabled && oldLegUnloaded
                && comp._test4StanceDrift > 0.040f
                && !comp._test7OldSupportDriftAllowanceLogged) {
                spdlog::info(
                    "[LocoTest7] OLD_SUPPORT_RELEASE allowed foot={} drift={:.3f}m "
                    "newSupport={} drift={:.3f}m distance=(old={:.3f},new={:.3f})",
                    comp._test4SupportSide < 0 ? "LEFT" : "RIGHT",
                    comp._test4StanceDrift,
                    comp._test4SupportSide < 0 ? "RIGHT" : "LEFT",
                    comp._test4PlantDrift,
                    comp._test5ComToOldSupport,
                    comp._test5ComToNewSupport);
                comp._test7OldSupportDriftAllowanceLogged = true;
            }
            const bool locksOff = rag.locomotionFootLockWeights[0] <= 0.001f
                               && rag.locomotionFootLockWeights[1] <= 0.001f
                               && rag._locomotionFootLockForce[0] <= 0.5f
                               && rag._locomotionFootLockForce[1] <= 0.5f;
            const bool stableTransfer = comAtTarget && insideNewSupport
                && oldLegUnloaded
                && swingContactNow && stanceContactNow
                && glm::length(leftVelocity) < 0.15f
                && glm::length(rightVelocity) < 0.15f
                && comp._test5ComHorizontalSpeed < 0.15f
                && (continuousEnabled || comp._test4StanceDrift <= 0.040f)
                && comp._test4PlantDrift <= 0.040f
                && tiltDeg < 30.0f
                && !rag._locomotionSupportSaturated
                && rag.locomotionLiftBone < 0
                && rag._locomotionLiftForce <= 0.5f
                && locksOff
                && !comp._test4MotorSaturated;
            comp._test5HoldStableTime = stableTransfer
                ? comp._test5HoldStableTime + dt : 0.0f;

            if (comp._test5HoldStableTime >= glm::max(
                    comp.test5HoldTime, 0.50f)) {
                if (multiStepEnabled) {
                    if (continuousEnabled) {
                        comp._test6StepsCompleted = comp._test6StepIndex;
                        comp._test7PreviousStepPeriod = comp._test7LastStepPeriod;
                        comp._test7LastStepPeriod = glm::max(
                            comp._test7RunTime - comp._test7StepStartTime, dt);
                        comp._test7PreviousStepLength = comp._test7LastStepLength;
                        comp._test7LastStepLength = comp._test4ForwardTravel;
                        comp._test7PreviousSupportAdvance =
                            comp._test7LastSupportAdvance;
                        comp._test7LastSupportAdvance =
                            comp._test7AchievedSupportAdvance;
                        const float comAdvance = glm::dot(
                            rag._locomotionCOM - comp._test7StepStartCom,
                            comp._test4Forward);
                        comp._test7MeasuredSpeed = comAdvance
                            / comp._test7LastStepPeriod;
                        const float stepDrift =
                            comp._test7StepMaxRelevantDrift;
                        comp._test7MaxDrift = glm::max(
                            comp._test7MaxDrift, stepDrift);
                        comp._test7PeakTilt = glm::max(
                            comp._test7PeakTilt, comp._test4PeakTilt);
                        comp._test7MaxMotorRatio = glm::max(
                            comp._test7MaxMotorRatio, comp._test4MaxMotorRatio);

                        const bool targetReached =
                            Diamond::Locomotion::HasReachedRunLimit(
                                continuousCommand, comp._test6StepsCompleted,
                                comp._test7RunTime);
                        const bool shouldStop = targetReached
                                             || comp._test7StopRequested;
                        spdlog::info(
                            "[LocoTest7] STEP_COMPLETE index={} support={} "
                            "footTravel={:.3f}m supportAdvance={:.3f}m "
                            "COMadvance={:+.3f}m "
                            "period={:.3f}s speed={:.3f}/{:.3f}m/s "
                            "drift={:.3f} tilt={:.1f} "
                            "heading=(error={:+.1f},peak={:.1f})deg motorRatio={:.2f} "
                            "edges=({},{}) next={}",
                            comp._test6StepsCompleted,
                            comp._test4SupportSide < 0 ? "RIGHT" : "LEFT",
                            comp._test7LastStepLength,
                            comp._test7AchievedSupportAdvance, comAdvance,
                            comp._test7LastStepPeriod,
                            comp._test7MeasuredSpeed,
                            continuousCommand.desiredSpeed,
                            stepDrift, comp._test4PeakTilt,
                            comp._test7HeadingErrorDeg,
                            comp._test7PeakHeadingErrorDeg,
                            comp._test4MaxMotorRatio,
                            comp._test6ContactTransitionsL,
                            comp._test6ContactTransitionsR,
                            shouldStop ? "STOP" : "ROLE_SWAP");

                        if (shouldStop) {
                            capturePhysicalLocalPose(*swing);
                            capturePhysicalLocalPose(*stance);
                            comp._test7StopStartTarget = comp._test4SupportTarget;
                            comp._test7StopEndTarget = comp._test7StopStartTarget;
                            // Capture immutable sole references for shutdown drift reporting.
                            // They are measurements only and never become force targets.
                            comp._test7StopFootTargetL = leftFoot;
                            comp._test7StopFootTargetR = rightFoot;
                            comp._test7StopFootDriftL = 0.0f;
                            comp._test7StopFootDriftR = 0.0f;
                            comp._test7StopMaxFootDrift = 0.0f;
                            comp._test7StopSettleFootDriftL = 0.0f;
                            comp._test7StopSettleFootDriftR = 0.0f;
                            comp._test7StopMaxSettleFootDrift = 0.0f;
                            comp._test7StopSettleReferenceValid = false;
                            comp._test7StopStableTime = 0.0f;
                            comp._test4Phase = kStopping;
                            comp._test4PhaseTime = 0.0f;
                            spdlog::info(
                                "[LocoTest7] phase=STOPPING reason={} "
                                "supportHold=({:+.3f},{:+.3f},{:+.3f}) "
                                "footReferences=(L=({:+.3f},{:+.3f},{:+.3f}),"
                                "R=({:+.3f},{:+.3f},{:+.3f})) "
                                "poseRelease={:.2f}s locks=DISABLED",
                                targetReached ? "target-reached" : "requested",
                                comp._test7StopStartTarget.x,
                                comp._test7StopStartTarget.y,
                                comp._test7StopStartTarget.z,
                                comp._test7StopFootTargetL.x,
                                comp._test7StopFootTargetL.y,
                                comp._test7StopFootTargetL.z,
                                comp._test7StopFootTargetR.x,
                                comp._test7StopFootTargetR.y,
                                comp._test7StopFootTargetR.z,
                                glm::max(comp.test7StopTime, 0.25f));
                        } else {
                            comp._test4Phase = kInterStep;
                            comp._test4PhaseTime = 0.0f;
                            comp._test6InterStepStableTime = 0.0f;
                            comp._test7InterStepRecenterStart =
                                comp._test4SupportTarget;
                            comp._test7InterStepRecenterTarget =
                                0.5f * (leftFoot + rightFoot);
                            comp._test7InterStepRecenterTarget.y =
                                comp._test4SupportTarget.y;
                            comp._test7InterStepRecenterT = 0.0f;
                            comp._test7InterStepCenterError =
                                horizontalDistance(
                                    rag._locomotionCOM,
                                    comp._test7InterStepRecenterTarget);
                            spdlog::info(
                                "[LocoTest7] phase=INTERSTEP recenter=ARMED "
                                "supportStart=({:+.3f},{:+.3f},{:+.3f}) "
                                "fixedCenter=({:+.3f},{:+.3f},{:+.3f}) "
                                "time={:.2f}s centerErr={:.3f}m "
                                "gates=(tilt={:.1f}deg,heading={:.1f}deg)",
                                comp._test7InterStepRecenterStart.x,
                                comp._test7InterStepRecenterStart.y,
                                comp._test7InterStepRecenterStart.z,
                                comp._test7InterStepRecenterTarget.x,
                                comp._test7InterStepRecenterTarget.y,
                                comp._test7InterStepRecenterTarget.z,
                                glm::max(comp.test7InterStepRecenterTime, 0.20f),
                                comp._test7InterStepCenterError,
                                glm::clamp(comp.test7InterStepTiltLimit,
                                           12.0f, 25.0f),
                                glm::clamp(comp.test7InterStepHeadingLimit,
                                           2.0f, 20.0f));
                        }
                    } else {
                    const int stepSlot = glm::clamp(comp._test6StepIndex - 1, 0, 1);
                    comp._test6StepForward[stepSlot] = comp._test4ForwardTravel;
                    comp._test6StepMaxDrift[stepSlot] = glm::max(
                        comp._test4MaxStanceDrift, comp._test4MaxPlantDrift);
                    comp._test6StepPeakTilt[stepSlot] = comp._test4PeakTilt;
                    comp._test6StepMotorRatio[stepSlot] = comp._test4MaxMotorRatio;
                    comp._test6StepsCompleted = comp._test6StepIndex;

                    if (comp._test6StepIndex == 1) {
                        comp._test4Phase = kInterStep;
                        comp._test4PhaseTime = 0.0f;
                        comp._test6InterStepStableTime = 0.0f;
                        spdlog::info(
                            "[LocoTest6] STEP1_COMPLETE support={} planted={} "
                            "forward={:.3f} COMerr={:.3f} drift={:.3f} "
                            "peakTilt={:.1f} motorRatio={:.2f} "
                            "edges=({},{}) phase=INTERSTEP",
                            comp._test4SupportSide < 0 ? "LEFT" : "RIGHT",
                            comp._test4SupportSide < 0 ? "RIGHT" : "LEFT",
                            comp._test6StepForward[0], comp._test5ComError,
                            comp._test6StepMaxDrift[0],
                            comp._test6StepPeakTilt[0],
                            comp._test6StepMotorRatio[0],
                            comp._test6ContactTransitionsL,
                            comp._test6ContactTransitionsR);
                    } else {
                        const float driftTolerance = glm::max(
                            comp.test6DriftGrowthTolerance, 0.0f);
                        const bool driftGrowthOk = comp._test6StepMaxDrift[1]
                            <= comp._test6StepMaxDrift[0] + driftTolerance;
                        const bool motorGrowthOk = comp._test6StepMotorRatio[1]
                            <= comp._test6StepMotorRatio[0] + 0.05f;
                        const bool finalTiltOk = std::abs(
                            tiltDeg - comp._test6InitialTilt) <= 10.0f;
                        const bool contactEdgesOk = comp._test6ContactTransitionsL == 2
                                                 && comp._test6ContactTransitionsR == 2;
                        const bool sequencePass = driftGrowthOk && motorGrowthOk
                            && finalTiltOk && contactEdgesOk
                            && comp._test6StepsCompleted == 2;
                        spdlog::info(
                            "[LocoTest6] COMPLETE_CHECK result={} steps={}/2 "
                            "forward=({:.3f},{:.3f}) "
                            "drift=({:.3f},{:.3f}) growth={:+.3f}/{:.3f}[{}] "
                            "peakTilt=({:.1f},{:.1f}) finalTilt={:.1f}/{:.1f}[{}] "
                            "motorRatio=({:.2f},{:.2f}) growth={:+.2f}/0.05[{}] "
                            "edges=({},{})/2[{}]",
                            sequencePass ? "PASS" : "FAIL",
                            comp._test6StepsCompleted,
                            comp._test6StepForward[0], comp._test6StepForward[1],
                            comp._test6StepMaxDrift[0], comp._test6StepMaxDrift[1],
                            comp._test6StepMaxDrift[1] - comp._test6StepMaxDrift[0],
                            driftTolerance, driftGrowthOk ? "ok" : "FAIL",
                            comp._test6StepPeakTilt[0], comp._test6StepPeakTilt[1],
                            tiltDeg, comp._test6InitialTilt,
                            finalTiltOk ? "ok" : "FAIL",
                            comp._test6StepMotorRatio[0],
                            comp._test6StepMotorRatio[1],
                            comp._test6StepMotorRatio[1]
                                - comp._test6StepMotorRatio[0],
                            motorGrowthOk ? "ok" : "FAIL",
                            comp._test6ContactTransitionsL,
                            comp._test6ContactTransitionsR,
                            contactEdgesOk ? "ok" : "FAIL");
                        if (sequencePass) {
                            comp._test4Phase = kComplete;
                            comp._test4PhaseTime = 0.0f;
                            spdlog::info(
                                "[LocoTest6] COMPLETE steps=2 support={} "
                                "COMerr={:.3f} hold={:.3f}s drift=({:.3f},{:.3f}) "
                                "tilt=({:.1f}->{:.1f}) edges=({},{})",
                                comp._test4SupportSide < 0 ? "RIGHT" : "LEFT",
                                comp._test5ComError, comp._test5HoldStableTime,
                                comp._test6StepMaxDrift[0],
                                comp._test6StepMaxDrift[1],
                                comp._test6InitialTilt, tiltDeg,
                                comp._test6ContactTransitionsL,
                                comp._test6ContactTransitionsR);
                        } else {
                            abortSequence("two-step accumulation check failed");
                        }
                    }
                    }
                } else {
                    comp._test4Phase = kComplete;
                    comp._test4PhaseTime = 0.0f;
                    spdlog::info(
                        "[LocoTest5] COMPLETE oldSupport={} newSupport={} "
                        "COMerr={:.3f} distance=(old={:.3f},new={:.3f}) "
                        "hold={:.3f}s drift=({:.3f},{:.3f}) "
                        "tilt=({:.1f}->{:.1f},peak={:.1f}) supportForce={:.0f} "
                        "supportSat={} maxMotorRatio={:.2f}",
                        comp._test4SupportSide < 0 ? "LEFT" : "RIGHT",
                        comp._test4SupportSide < 0 ? "RIGHT" : "LEFT",
                        comp._test5ComError,
                        comp._test5ComToOldSupport,
                        comp._test5ComToNewSupport,
                        comp._test5HoldStableTime,
                        comp._test4MaxStanceDrift, comp._test4MaxPlantDrift,
                        comp._test4InitialTilt, tiltDeg, comp._test4PeakTilt,
                        glm::length(rag._locomotionSupportForce),
                        rag._locomotionSupportSaturated ? "YES" : "no",
                        comp._test4MaxMotorRatio);
                }
            } else if (comp._test4PhaseTime >= glm::max(
                           comp.test5HoldTimeout,
                           glm::max(comp.test5HoldTime, 0.50f))) {
                spdlog::warn(
                    "[LocoTest{}] TRANSFER_CHECK result=FAIL "
                    "COMerr={:.3f}/{:.3f}[{}] newRegion={:.3f}/{:.3f}[{}] "
                    "oldUnload=({:.3f}+0.020<{:.3f})[{}] "
                    "contact=({},{}) speed={:.3f}/0.150 drift=({:.3f},{:.3f}) "
                    "tilt={:.1f}/30 supportSat={} motorSat={} stable={:.3f}/{:.3f}s",
                    comp.validationTest, comp._test5ComError, comTolerance,
                    comAtTarget ? "ok" : "FAIL",
                    comp._test5ComToNewSupport, newSupportRadius,
                    insideNewSupport ? "ok" : "FAIL",
                    comp._test5ComToNewSupport,
                    comp._test5ComToOldSupport,
                    oldLegUnloaded ? "ok" : "FAIL",
                    comp._test4ContactL ? "L" : "-",
                    comp._test4ContactR ? "R" : "-",
                    comp._test5ComHorizontalSpeed,
                    comp._test4StanceDrift, comp._test4PlantDrift,
                    tiltDeg,
                    rag._locomotionSupportSaturated ? "YES" : "no",
                    comp._test4MotorSaturated ? "YES" : "no",
                    comp._test5HoldStableTime,
                    glm::max(comp.test5HoldTime, 0.50f));
                abortSequence("support transfer did not settle before hold timeout");
            }
        } else if (comp._test4Phase == kInterStep) {
            const float interStepTiltLimit = continuousEnabled
                ? glm::clamp(comp.test7InterStepTiltLimit, 12.0f, 25.0f)
                : 30.0f;
            const float interStepHeadingLimit = continuousEnabled
                ? glm::clamp(comp.test7InterStepHeadingLimit, 2.0f, 20.0f)
                : 180.0f;
            const bool recenterReady = !continuousEnabled
                || (comp._test7InterStepRecenterT >= 0.999f
                    && comp._test7InterStepCenterError
                        <= glm::max(comp.test5ComTolerance, 0.04f));
            const bool uprightReady = !continuousEnabled
                || !rag._locomotionUprightSaturated;
            const bool headingReady = !continuousEnabled
                || std::abs(comp._test7HeadingErrorDeg) <= interStepHeadingLimit;
            const bool interStepReady = swingContactNow && stanceContactNow
                && glm::length(leftVelocity) < 0.15f
                && glm::length(rightVelocity) < 0.15f
                && comp._test5ComHorizontalSpeed < 0.15f
                && tiltDeg <= interStepTiltLimit
                && headingReady
                && recenterReady
                && uprightReady
                && !rag._locomotionSupportSaturated
                && !comp._test4MotorSaturated;
            comp._test6InterStepStableTime = interStepReady
                ? comp._test6InterStepStableTime + dt : 0.0f;
            if (comp._test6InterStepStableTime >= glm::max(
                    comp.test6InterStepTime, 0.10f)) {
                const float completedPlannedAdvance =
                    comp._test7PlannedSupportAdvance;
                const float completedAchievedAdvance =
                    comp._test7AchievedSupportAdvance;
                const bool completedReachClamped =
                    comp._test7ReachClampedStep;
                // Test 7's legs already hold their last analytic support commands.
                // Recapturing their measured rotations here made whatever tracking error
                // remained after this step the commanded starting pose of the next one.
                // Tests 4-6 retain their already-validated physical handoff behavior.
                if (!continuousEnabled) {
                    capturePhysicalLocalPose(*swing);
                    capturePhysicalLocalPose(*stance);
                }
                const int oldSupportSide = comp._test4SupportSide;
                comp._test4FootBaselineL = leftFoot;
                comp._test4FootBaselineR = rightFoot;
                comp._test4ComBaseline = rag._locomotionCOM;
                comp._test4SupportTarget = comp._test4ComBaseline;
                comp._test4SupportSide = -oldSupportSide;
                comp._test4ComCommand = 0.0f;
                comp._test4ComLateral = 0.0f;
                comp._test4TargetLateral = 0.0f;
                comp._test4Phase = kWeightShift;
                comp._test4PhaseTime = 0.0f;
                comp._test4SettleTime = 0.0f;
                comp._test4AirborneTime = 0.0f;
                comp._test4ArrivalStableTime = 0.0f;
                comp._test4ReachLimit = 0.0f;
                comp._test7PlannedSupportAdvance = 0.0f;
                comp._test7AchievedSupportAdvance = 0.0f;
                comp._test4PlantAcquireStableTime = 0.0f;
                comp._test4TrajectoryT = 0.0f;
                comp._test4TouchdownAccepted = false;
                comp._test4MaxStanceDrift = 0.0f;
                comp._test4MaxPlantDrift = 0.0f;
                comp._test7StepMaxRelevantDrift = 0.0f;
                comp._test7TakeoffContactRecoveryTime = 0.0f;
                comp._test7TakeoffContactRecoveryActive = false;
                comp._test7SwingRecontactTime = 0.0f;
                comp._test7IkPlanHipValid = false;
                comp._test7ReachClampedStep = false;
                comp._test7OldSupportDriftAllowanceLogged = false;
                comp._test4InitialTilt = tiltDeg;
                comp._test4PeakTilt = tiltDeg;
                comp._test4FinalTilt = tiltDeg;
                comp._test4MaxMotorRatio = 0.0f;
                comp._test4MotorSaturated = false;
                comp._test4PlantPoseCaptured = false;
                comp._test4PreviousSwingFootValid = false;
                comp._test4PrevSwingContact = true;
                comp._test5TransferT = 0.0f;
                comp._test5HoldStableTime = 0.0f;
                comp._test5ContactLossTime = 0.0f;
                comp._test5ComError = 0.0f;
                comp._test5ComToOldSupport = 0.0f;
                comp._test5ComToNewSupport = 0.0f;
                comp._test5TransferStartTarget = comp._test4ComBaseline;
                comp._test5TransferEndTarget = comp._test4ComBaseline;
                if (continuousEnabled) {
                    const float minimumStep = glm::min(
                        comp.test7MinStepLength, comp.test7MaxStepLength);
                    const float maximumStep = glm::max(
                        comp.test7MinStepLength, comp.test7MaxStepLength);
                    const float baseTrackingReserve = 0.010f;
                    const float trackingReserve = glm::min(glm::max(
                        baseTrackingReserve,
                        comp._test7SettledTrackingLoss + 0.003f), 0.040f);
                    const float minimumCommand = glm::min(
                        maximumStep, minimumStep + trackingReserve);
                    comp._test7ReachCommandCeiling = glm::clamp(
                        comp._test7ReachCommandCeiling,
                        minimumCommand, maximumStep);
                    const float speedFeedbackTarget = glm::clamp(
                        comp.test7NominalAdvance + comp.test7PlacementGain
                            * (continuousCommand.desiredSpeed
                               - comp._test7MeasuredSpeed),
                        minimumCommand, maximumStep);
                    const float previousCommand =
                        comp._test7CommandedStepLength;
                    float nextCommand = glm::mix(
                        previousCommand, speedFeedbackTarget, 0.50f);
                    if (completedReachClamped) {
                        // A reach-limited gait must not respond to low measured speed by
                        // asking for an even longer next step. Use the completed step's
                        // reachable plan and measured tracking reserve as the next cap.
                        const float reachLimitedTarget = glm::clamp(
                            glm::min(completedPlannedAdvance,
                                     completedAchievedAdvance + trackingReserve),
                            minimumCommand, maximumStep);
                        comp._test7ReachCommandCeiling = glm::min(
                            comp._test7ReachCommandCeiling,
                            reachLimitedTarget);
                        comp._test7ReachClearSteps = 0;
                        nextCommand = glm::min(
                            nextCommand, comp._test7ReachCommandCeiling);
                        spdlog::info(
                            "[LocoTest7] REACH_FEEDBACK applied planned={:.3f}m "
                            "achieved={:.3f}m reserve={:.3f}m speedTarget={:.3f}m "
                            "ceiling={:.3f}m command={:.3f}->{:.3f}m",
                            completedPlannedAdvance, completedAchievedAdvance,
                            trackingReserve, speedFeedbackTarget,
                            comp._test7ReachCommandCeiling,
                            previousCommand, nextCommand);
                    } else if (comp._test7ReachCommandCeiling
                               < maximumStep - 1e-4f) {
                        ++comp._test7ReachClearSteps;
                        const bool releaseCeiling =
                            comp._test7ReachClearSteps >= 3;
                        if (releaseCeiling) {
                            comp._test7ReachCommandCeiling = glm::min(
                                maximumStep,
                                comp._test7ReachCommandCeiling + 0.005f);
                            comp._test7ReachClearSteps = 0;
                        }
                        nextCommand = glm::min(
                            nextCommand, comp._test7ReachCommandCeiling);
                        spdlog::info(
                            "[LocoTest7] REACH_CEILING state={} ceiling={:.3f}m "
                            "clearSteps={}/3 speedTarget={:.3f}m command={:.3f}->{:.3f}m",
                            releaseCeiling ? "RELAX" : "HOLD",
                            comp._test7ReachCommandCeiling,
                            comp._test7ReachClearSteps,
                            speedFeedbackTarget, previousCommand, nextCommand);
                    } else {
                        comp._test7ReachCommandCeiling = maximumStep;
                        comp._test7ReachClearSteps = 0;
                    }
                    comp._test7CommandedStepLength = glm::clamp(
                        nextCommand, minimumCommand, maximumStep);
                    ++comp._test6StepIndex;
                    comp._test7StepStartTime = comp._test7RunTime;
                    comp._test7StepStartCom = rag._locomotionCOM;
                    spdlog::info(
                        "[LocoTest7] ROLE_SWAP result=READY completed={} next={} "
                        "oldSupport={} newSupport={} nextSwing={} "
                        "speed={:.3f}/{:.3f}m/s advanceCommand={:.3f}m "
                        "stable={:.3f}s contact=({},{}) tilt={:.1f}/{:.1f} "
                        "heading={:+.1f}/{:.1f}deg "
                        "recenter=(t={:.2f},centerErr={:.3f}m) "
                        "angular=(tiltRate={:.3f}rad/s,uprightSat={}) "
                        "edges=({},{})",
                        comp._test6StepsCompleted, comp._test6StepIndex,
                        oldSupportSide < 0 ? "LEFT" : "RIGHT",
                        oldSupportSide < 0 ? "RIGHT" : "LEFT",
                        oldSupportSide < 0 ? "LEFT" : "RIGHT",
                        comp._test7MeasuredSpeed,
                        continuousCommand.desiredSpeed,
                        comp._test7CommandedStepLength,
                        comp._test6InterStepStableTime,
                        comp._test4ContactL ? "L" : "-",
                        comp._test4ContactR ? "R" : "-",
                        tiltDeg, interStepTiltLimit,
                        comp._test7HeadingErrorDeg, interStepHeadingLimit,
                        comp._test7InterStepRecenterT,
                        comp._test7InterStepCenterError,
                        comp._test7RootTiltRate,
                        rag._locomotionUprightSaturated ? "YES" : "no",
                        comp._test6ContactTransitionsL,
                        comp._test6ContactTransitionsR);
                } else {
                    comp._test6StepIndex = 2;
                    spdlog::info(
                        "[LocoTest6] ROLE_SWAP result=READY oldSupport={} newSupport={} "
                        "nextSwing={} stable={:.3f}s COMspeed={:.3f} "
                        "contact=({},{}) tilt={:.1f} edges=({},{})",
                        oldSupportSide < 0 ? "LEFT" : "RIGHT",
                        oldSupportSide < 0 ? "RIGHT" : "LEFT",
                        oldSupportSide < 0 ? "LEFT" : "RIGHT",
                        comp._test6InterStepStableTime,
                        comp._test5ComHorizontalSpeed,
                        comp._test4ContactL ? "L" : "-",
                        comp._test4ContactR ? "R" : "-",
                        tiltDeg,
                        comp._test6ContactTransitionsL,
                        comp._test6ContactTransitionsR);
                }
            } else if (comp._test4PhaseTime >= glm::max(
                           comp.test5HoldTimeout, 0.50f)) {
                if (continuousEnabled) {
                    spdlog::warn(
                        "[LocoTest7] INTERSTEP_CHECK result=FAIL "
                        "contact=({},{}) speed=(L={:.3f},R={:.3f},COM={:.3f}) "
                        "tilt={:.1f}/{:.1f} tiltRate={:.3f}rad/s "
                        "heading={:+.1f}/{:.1f}deg ready={} "
                        "saturation=(support={},upright={},heading={},motor={}) "
                        "stable={:.3f}/{:.3f}s recenter=(t={:.2f},err={:.3f}m,ready={})",
                        swingContactNow ? "S" : "-",
                        stanceContactNow ? "T" : "-",
                        glm::length(leftVelocity),
                        glm::length(rightVelocity),
                        comp._test5ComHorizontalSpeed,
                        tiltDeg, interStepTiltLimit,
                        comp._test7RootTiltRate,
                        comp._test7HeadingErrorDeg, interStepHeadingLimit,
                        headingReady ? "yes" : "NO",
                        rag._locomotionSupportSaturated ? "YES" : "no",
                        rag._locomotionUprightSaturated ? "YES" : "no",
                        rag._locomotionHeadingSaturated ? "YES" : "no",
                        comp._test4MotorSaturated ? "YES" : "no",
                        comp._test6InterStepStableTime,
                        glm::max(comp.test6InterStepTime, 0.10f),
                        comp._test7InterStepRecenterT,
                        comp._test7InterStepCenterError,
                        recenterReady ? "yes" : "NO");
                }
                abortSequence("inter-step handoff did not remain ready");
            }
        } else if (continuousEnabled && comp._test4Phase == kStopping) {
            const float supportError = horizontalDistance(
                rag._locomotionCOM, comp._test7StopEndTarget);
            const bool poseReleased = comp._test4PhaseTime >= glm::max(
                    comp.test7StopTime, 0.25f)
                && comp._test7CrouchBlend <= 0.001f;
            if (poseReleased && !comp._test7StopSettleReferenceValid) {
                // The walking-to-standing pose blend deliberately changes the staggered
                // stance geometry. Capture the physical soles once that command is complete,
                // then measure only continued motion from this settled-pose reference.
                comp._test7StopSettleFootTargetL = leftFoot;
                comp._test7StopSettleFootTargetR = rightFoot;
                comp._test7StopSettleFootDriftL = 0.0f;
                comp._test7StopSettleFootDriftR = 0.0f;
                comp._test7StopMaxSettleFootDrift = 0.0f;
                comp._test7StopSettleReferenceValid = true;
                comp._test7StopStableTime = 0.0f;
                spdlog::info(
                    "[LocoTest7] STOP_SETTLE_REFERENCE "
                    "transitionDisplacement=(L={:.3f},R={:.3f},max={:.3f})m "
                    "settleReferences=(L=({:+.3f},{:+.3f},{:+.3f}),"
                    "R=({:+.3f},{:+.3f},{:+.3f}))",
                    comp._test7StopFootDriftL,
                    comp._test7StopFootDriftR,
                    comp._test7StopMaxFootDrift,
                    comp._test7StopSettleFootTargetL.x,
                    comp._test7StopSettleFootTargetL.y,
                    comp._test7StopSettleFootTargetL.z,
                    comp._test7StopSettleFootTargetR.x,
                    comp._test7StopSettleFootTargetR.y,
                    comp._test7StopSettleFootTargetR.z);
            }
            const bool stopFeetSettled = comp._test7StopSettleReferenceValid
                && comp._test7StopSettleFootDriftL <= 0.040f
                && comp._test7StopSettleFootDriftR <= 0.040f;
            const bool settled = poseReleased
                && supportError <= glm::max(comp.test5ComTolerance, 0.04f)
                && comp._test4ContactL && comp._test4ContactR
                && glm::length(leftVelocity) < 0.15f
                && glm::length(rightVelocity) < 0.15f
                && comp._test5ComHorizontalSpeed < 0.15f
                && stopFeetSettled
                && tiltDeg < 30.0f
                && !rag._locomotionSupportSaturated
                && !comp._test4MotorSaturated;
            comp._test7StopStableTime = settled
                ? comp._test7StopStableTime + dt : 0.0f;
            if (comp._test7StopStableTime >= glm::max(
                    comp.test7StopHoldTime, 0.25f)) {
                comp._test4Phase = kReturnStand;
                comp._test4PhaseTime = 0.0f;
                comp._test7StopStableTime = 0.0f;
                rag.locomotionSupportTargetWeight = 0.0f;
                spdlog::info(
                    "[LocoTest7] phase=RETURN_STAND supportError={:.3f} "
                    "contact=({},{}) speed={:.3f} tilt={:.1f} "
                    "shutdownFeet=(transition=({:.3f},{:.3f}),"
                    "settleDrift=({:.3f},{:.3f}),locks=({:.2f},{:.2f}),"
                    "force=({:.0f},{:.0f})N) pose=STANDING",
                    supportError,
                    comp._test4ContactL ? "L" : "-",
                    comp._test4ContactR ? "R" : "-",
                    comp._test5ComHorizontalSpeed, tiltDeg,
                    comp._test7StopFootDriftL,
                    comp._test7StopFootDriftR,
                    comp._test7StopSettleFootDriftL,
                    comp._test7StopSettleFootDriftR,
                    rag.locomotionFootLockWeights[0],
                    rag.locomotionFootLockWeights[1],
                    rag._locomotionFootLockForce[0],
                    rag._locomotionFootLockForce[1]);
            } else if (comp._test4PhaseTime >= glm::max(
                           comp.test7StopTime + comp.test5HoldTimeout, 1.0f)) {
                spdlog::warn(
                    "[LocoTest7] STOP_CHECK result=FAIL supportError={:.3f}/{:.3f}m "
                    "contact=({},{}) speed=(L={:.3f},R={:.3f},COM={:.3f}) "
                    "shutdownFeet=(transition=({:.3f},{:.3f},max={:.3f})m,"
                    "settleDrift=({:.3f},{:.3f},max={:.3f})/0.040m,"
                    "locks=({:.2f},{:.2f}),force=({:.0f},{:.0f})N) "
                    "poseReleased={} settleReference={} crouch={:.3f} tilt={:.1f} "
                    "saturation=(support={},motor={})",
                    supportError, glm::max(comp.test5ComTolerance, 0.04f),
                    comp._test4ContactL ? "L" : "-",
                    comp._test4ContactR ? "R" : "-",
                    glm::length(leftVelocity),
                    glm::length(rightVelocity),
                    comp._test5ComHorizontalSpeed,
                    comp._test7StopFootDriftL,
                    comp._test7StopFootDriftR,
                    comp._test7StopMaxFootDrift,
                    comp._test7StopSettleFootDriftL,
                    comp._test7StopSettleFootDriftR,
                    comp._test7StopMaxSettleFootDrift,
                    rag.locomotionFootLockWeights[0],
                    rag.locomotionFootLockWeights[1],
                    rag._locomotionFootLockForce[0],
                    rag._locomotionFootLockForce[1],
                    poseReleased ? "yes" : "NO",
                    comp._test7StopSettleReferenceValid ? "yes" : "NO",
                    comp._test7CrouchBlend,
                    tiltDeg,
                    rag._locomotionSupportSaturated ? "YES" : "no",
                    comp._test4MotorSaturated ? "YES" : "no");
                abortSequence("continuous-gait stop did not settle");
            }
        } else if (continuousEnabled && comp._test4Phase == kReturnStand) {
            const bool locksOff = rag.locomotionFootLockWeights[0] <= 0.001f
                               && rag.locomotionFootLockWeights[1] <= 0.001f
                               && rag._locomotionFootLockForce[0] <= 0.5f
                               && rag._locomotionFootLockForce[1] <= 0.5f;
            const bool test1Standing = comp._test4ContactL && comp._test4ContactR
                && glm::length(leftVelocity) < 0.15f
                && glm::length(rightVelocity) < 0.15f
                && comp._test5ComHorizontalSpeed < 0.15f
                && tiltDeg < 15.0f
                && std::abs(comp._test7HeadingErrorDeg)
                    <= glm::clamp(comp.test7InterStepHeadingLimit, 2.0f, 20.0f)
                && !rag._locomotionSupportSaturated
                && !rag._locomotionUprightSaturated
                && rag.locomotionLiftBone < 0
                && rag._locomotionLiftForce <= 0.5f
                && locksOff
                && !comp._test4MotorSaturated;
            comp._test7StopStableTime = test1Standing
                ? comp._test7StopStableTime + dt : 0.0f;
            if (comp._test7StopStableTime >= 0.50f) {
                const int completed = comp._test6StepsCompleted;
                const int edgeTotal = comp._test6ContactTransitionsL
                                    + comp._test6ContactTransitionsR;
                const bool stepCountOk =
                    Diamond::Locomotion::RunLimitSatisfied(
                        continuousCommand, completed, comp._test7RunTime);
                const bool edgeCountOk = edgeTotal == 2 * completed
                    && std::abs(comp._test6ContactTransitionsL
                              - comp._test6ContactTransitionsR) <= 2;
                const bool lengthConverged = completed < 3
                    || std::abs(comp._test7LastSupportAdvance
                              - comp._test7PreviousSupportAdvance) <= 0.030f;
                const bool periodConverged = completed < 3
                    || std::abs(comp._test7LastStepPeriod
                              - comp._test7PreviousStepPeriod) <= 0.75f;
                const bool runtimeCommand = continuousCommand.runLimit
                    == Diamond::Locomotion::RunLimit::None;
                const float totalForward = glm::dot(
                    rag._locomotionCOM - comp._test7StartCom,
                    comp._test4Forward);
                const bool forwardOk = totalForward >= 0.05f * completed;
                const bool boundsOk = comp._test7MaxDrift <= 0.040f
                    && comp._test7StopMaxSettleFootDrift <= 0.040f
                    && comp._test7PeakTilt < 30.0f
                    && comp._test7MaxMotorRatio <= 1.0f;
                const bool pass = stepCountOk && edgeCountOk
                    && lengthConverged && periodConverged
                    && (runtimeCommand || forwardOk)
                    && boundsOk && test1Standing;
                spdlog::info(
                    "[LocoTest7] COMPLETE_CHECK result={} mode={} "
                    "steps={} time={:.2f}s forward={:.3f}[{}] "
                    "lastSupportAdvance=({:.3f},{:.3f}) "
                    "delta={:.3f}/0.030[{}] "
                    "lastPeriod=({:.3f},{:.3f}) delta={:.3f}/0.750[{}] "
                    "maxDrift={:.3f}/0.040 "
                    "stopTransition=(L={:.3f},R={:.3f},max={:.3f})m "
                    "stopDrift=(L={:.3f},R={:.3f},max={:.3f})/0.040 "
                    "peakTilt={:.1f}/30 "
                    "heading=(final={:+.1f},peak={:.1f})deg "
                    "maxMotorRatio={:.2f}/1.00 edges=({},{}) total={}/{}[{}] "
                    "standing={}",
                    pass ? "PASS" : "FAIL",
                    Diamond::Locomotion::RunLimitName(
                        continuousCommand.runLimit),
                    completed, comp._test7RunTime,
                    totalForward, runtimeCommand
                        ? "runtime" : (forwardOk ? "ok" : "FAIL"),
                    comp._test7PreviousSupportAdvance,
                    comp._test7LastSupportAdvance,
                    std::abs(comp._test7LastSupportAdvance
                           - comp._test7PreviousSupportAdvance),
                    lengthConverged ? "ok" : "FAIL",
                    comp._test7PreviousStepPeriod,
                    comp._test7LastStepPeriod,
                    std::abs(comp._test7LastStepPeriod
                           - comp._test7PreviousStepPeriod),
                    periodConverged ? "ok" : "FAIL",
                    comp._test7MaxDrift,
                    comp._test7StopFootDriftL,
                    comp._test7StopFootDriftR,
                    comp._test7StopMaxFootDrift,
                    comp._test7StopSettleFootDriftL,
                    comp._test7StopSettleFootDriftR,
                    comp._test7StopMaxSettleFootDrift,
                    comp._test7PeakTilt,
                    comp._test7HeadingErrorDeg,
                    comp._test7PeakHeadingErrorDeg,
                    comp._test7MaxMotorRatio,
                    comp._test6ContactTransitionsL,
                    comp._test6ContactTransitionsR,
                    edgeTotal, 2 * completed,
                    edgeCountOk ? "ok" : "FAIL",
                    test1Standing ? "ok" : "FAIL");
                comp._test7Running = false;
                if (pass) {
                    comp._test4Phase = runtimeCommand ? kIdle : kComplete;
                    comp._test4PhaseTime = 0.0f;
                    comp._test4SupportSide = 0;
                    comp._test7StopRequested = false;
                    if (runtimeCommand) {
                        spdlog::info(
                            "[LocoRuntime] STOP_COMPLETE steps={} time={:.2f}s "
                            "forward={:.3f} finalTilt={:.1f} ready=IDLE",
                            completed, comp._test7RunTime, totalForward, tiltDeg);
                    } else {
                        spdlog::info(
                            "[LocoTest7] COMPLETE steps={} time={:.2f}s "
                            "forward={:.3f} finalTilt={:.1f} support=OFF",
                            completed, comp._test7RunTime, totalForward, tiltDeg);
                    }
                } else {
                    abortSequence("continuous-gait completion check failed");
                }
            } else if (comp._test4PhaseTime >= 3.0f) {
                abortSequence("return to standing did not settle");
            }
        } else if (comp._test4Phase == kAbort
                   && std::abs(comp._test4ComCommand) < 0.01f
                   && std::abs(comp._test4ComLateral) < 0.01f
                   && (!transferEnabled || horizontalDistance(
                           rag._locomotionCOM, comp._test4ComBaseline) < 0.05f)
                   && comp._test4PhaseTime >= glm::max(comp.test2ShiftTime, 0.01f)) {
            spdlog::warn(
                "[LocoTest4] reset after aborted sequence support={} "
                "maxDrift=({:.3f},{:.3f}) peakTilt={:.1f}",
                comp._test4SupportSide < 0 ? "LEFT" : "RIGHT",
                comp._test4MaxStanceDrift, comp._test4MaxPlantDrift,
                comp._test4PeakTilt);
            comp._test4Phase = kIdle;
            comp._test4PhaseTime = 0.0f;
            comp._test4SupportSide = 0;
            comp._test4TouchdownAccepted = false;
        }

        comp._test4PrevSwingContact = swingContactNow;

        if (comp.debug) {
            DebugDraw::Sphere(comp._test4ComBaseline, 0.025f, {0.2f, 0.7f, 1.0f});
            DebugDraw::Sphere(supportTarget, 0.035f, {1.0f, 0.7f, 0.1f});
            DebugDraw::Line(comp._test4ComBaseline, supportTarget, {1.0f, 0.7f, 0.1f});
            DebugDraw::Sphere(comp._test4Foothold, 0.045f, {0.2f, 1.0f, 0.2f});
            DebugDraw::Sphere(desiredFoot, 0.035f, {1.0f, 0.4f, 0.1f});
            DebugDraw::Line(swingFoot, desiredFoot, {1.0f, 0.4f, 0.1f});
            if (comp._test4Phase >= kSwing && comp._test4Phase <= kComplete) {
                glm::vec3 previous = trajectoryPoint(0.0f);
                for (int i = 1; i <= 12; ++i) {
                    const glm::vec3 next = trajectoryPoint(static_cast<float>(i) / 12.0f);
                    DebugDraw::Line(previous, next, {0.8f, 0.3f, 1.0f});
                    previous = next;
                }
                DebugDraw::Line(hoverTarget, comp._test4Foothold,
                                {0.2f, 0.9f, 1.0f});
            }
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
            leg.ankleFromFootLocal =
                glm::conjugate(leg.plantedFootWorldRotation) * (ankle - foot);
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

        // Tests 3-7 own their isolated IK/local touchdown pose while active. Multi-step
        // tests own both legs after the first role swap so their staggered analytic support
        // commands remain authoritative instead of being replaced by the standing solver.
        // Test 7 releases both during RETURN_STAND so the validated standing motor target,
        // rather than the final walking pose, owns the stopped character.
        const bool test3OwnsSwing = comp.validationTest == 3
            && comp._test3Phase >= 2 && comp._test3Phase <= 6;
        const bool test4OwnsSwing =
            ((comp.validationTest >= 4 && comp.validationTest <= 7)
             || comp._continuousGaitEnabled)
            && comp._test4Phase >= 2 && comp._test4Phase <= 11
            && !(comp._continuousGaitEnabled && comp._test4Phase == 11);
        const bool multiStepOwnsBoth =
            (comp.validationTest == 6 && comp._test6StepIndex == 2
                && comp._test4Phase >= 1 && comp._test4Phase <= 11)
            || (comp._continuousGaitEnabled && comp._test6StepIndex >= 2
                && comp._test4Phase >= 1 && comp._test4Phase <= 13);
        const bool validationOwnsLeft =
            (test3OwnsSwing && comp._test3SupportSide > 0)
            || (test4OwnsSwing && comp._test4SupportSide > 0)
            || multiStepOwnsBoth;
        const bool validationOwnsRight =
            (test3OwnsSwing && comp._test3SupportSide < 0)
            || (test4OwnsSwing && comp._test4SupportSide < 0)
            || multiStepOwnsBoth;
        if (!validationOwnsLeft) solveLeg(comp._legL, -1.0f, canStep);
        if (!validationOwnsRight) solveLeg(comp._legR, 1.0f, canStep);

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
