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

// Gameplay-facing settings and runtime state for the physical ragdoll gait. The system
// below translates movement intent into a small gait command; the detailed step planner
// and constrained leg solve are isolated in LocomotionPhysicalGait.inl.
struct LocamotionControllerComponent
{
    float maxSpeed        = 1.0f;
    float deadzone        = 0.2f;
    float minWalkSpeed    = 0.15f;
    float facingOffsetDeg = 90.0f;
    float groundRayLength = 1.2f;
    float consciousness   = 1.0f;
    float postPoweredGrace = 0.4f;
    float gaitBlendTime   = 0.6f;

    float weightShiftDuration = 1.50f;
    float supportFrequency = 1.50f;
    float supportMaxAcceleration = 3.0f;
    float supportBias = 0.92f;
    float takeoffHeight = 0.060f;
    float takeoffTimeout = 0.50f;
    float liftFrequency = 3.5f;
    float liftMaxForce = 180.0f;
    float stepLength = 0.225f;
    float swingHeight = 0.10f;
    float swingDuration = 0.90f;
    float arrivalHeight = 0.05f;
    float arrivalTolerance = 0.02f;
    float arrivalSettleDuration = 0.10f;
    float arrivalTimeout = 0.60f;
    float descentDuration = 0.35f;
    float plantTimeout = 0.35f;
    float plantAcquireDuration = 0.10f;
    float plantAcquireMaxSpeed = 0.05f;
    float plantAcquireTimeout = 0.60f;
    float contactSettleDuration = 0.30f;
    float touchdownMaxVerticalSpeed = 0.25f;
    float touchdownMaxHorizontalSpeed = 0.65f;
    float touchdownMaxAngularSpeed = 3.00f;
    float touchdownMaxCommandSpeed = 0.25f;
    float touchdownMinTrajectoryProgress = 0.92f;
    float touchdownMinNormalY = 0.70f;
    float footTargetTolerance = 0.04f;
    float safeReachFraction = 0.99f;
    float transferDuration = 1.00f;
    float transferSupportBias = 0.92f;
    float transferComTolerance = 0.04f;
    float transferHoldDuration = 0.50f;
    float transferTimeout = 1.50f;
    float interStepDuration = 0.25f;
    float driftGrowthTolerance = 0.015f;
    float gaitDesiredSpeed = 0.020f;
    // Desired time from one planted step to the next. Runtime phase durations are
    // proportionally compressed to this budget while retaining small physical safety
    // floors. Set to zero to use the individual phase durations verbatim.
    float gaitTargetStepPeriod = 1.20f;
    // Multiplies the physical safety floor of every cadence-bearing phase. Values below
    // one permit faster limb/support motion while preserving their relative envelope.
    float gaitCadenceFloorScale = 1.0f;
    // Validation phases can overlap motion and therefore use a smaller independent floor
    // without demanding that the physical foot swing or COM transfer unrealistically fast.
    float gaitValidationFloorScale = 1.0f;
    float gaitSupportMaxSpeed = 0.30f;
    // Forward support transport is intentionally separate from the general support cap:
    // the latter must still have authority for lateral load transfer and recovery.
    float gaitSupportTransportSpeed = 0.10f;
    bool gaitAdaptationEnabled = true;
    float gaitAdaptationResponse = 0.25f;
    float gaitMaxStrideCorrection = 0.020f;
    float gaitMaxPeriodSlowdown = 0.40f;
    float gaitStressStopThreshold = 3.0f;
    float gaitPlacementGain = 0.75f;
    float gaitNominalAdvance = 0.10f;
    float gaitMinStepLength = 0.06f;
    float gaitMaxStepLength = 0.14f;
    float gaitReachCrouch = 0.018f;
    float gaitCrouchTime = 0.35f;
    float gaitUsableReachFraction = 0.975f;
    float gaitSoleLevelTime = 0.35f;
    float gaitFootPositionGain = 0.45f;
    float gaitFootVelocityLeadTime = 0.08f;
    float gaitMaxFootCorrection = 0.045f;
    // Straight walking uses the same measured-foot command governor as turning.
    // These limits bound the Cartesian sole command rather than the joint motors.
    float gaitSwingSpeedLimit = 2.00f;
    float gaitSwingAcceleration = 18.0f;
    float gaitSoleAngularSpeedLimit = 3.25f;
    float gaitSoleAngularAcceleration = 18.0f;
    float gaitLandingLinearDampingTime = 0.08f;
    float gaitInterStepRecenterTime = 0.35f;
    float gaitUprightStiffness = 700.0f;
    float gaitUprightDamping = 100.0f;
    float gaitUprightMaxTorque = 250.0f;
    float gaitHeadingStiffness = 180.0f;
    float gaitHeadingDamping = 50.0f;
    float gaitHeadingMaxTorque = 80.0f;
    float gaitMaxTurnStepDeg = 5.0f;
    float gaitTurnFullAdvanceDeg = 15.0f;
    float gaitTurnZeroAdvanceDeg = 45.0f;
    float gaitTurnInsideSwingSpeedLimit = 0.78f;
    float gaitTurnOutsideSwingSpeedLimit = 0.62f;
    float gaitTurnAngularSpeedLimit = 2.25f;
    float gaitTurnLinearAcceleration = 2.50f;
    float gaitTurnAngularAcceleration = 12.0f;
    float gaitMaxFootSeparation = 0.18f;
    float gaitInterStepTiltLimit = 15.0f;
    float gaitInterStepHeadingLimit = 8.0f;
    float gaitStopTime = 1.0f;
    float gaitStopHoldTime = 0.50f;

    float maxLegReachFraction = 0.94f;
    float standingPoseResponse = 0.10f;

    std::string leftFootBone  = "leg_joint_L_5";
    std::string rightFootBone = "leg_joint_R_5";
    std::string torsoBone     = "torso_joint_3";

    float hipLimitMarginDeg = 3.0f;
    float poseWeight = 1.0f;
    float uprightScale = 1.0f;        // 1 = engine rights the pelvis, 0 = torso joint alone

    bool ikWriteEnabled = true;
    bool debug = false;

    // Bounded turn-plan state. Desired heading remains live, committed heading advances
    // only after support transfer, and an admitted active plan stays immutable until
    // that transfer succeeds or the step is cancelled.
    enum class TurnConstraintResult {
        NotEvaluated,
        None,
        ControllerYawLimit,
        TurnPairBudget,
        TurnInitiationRole,
        SwingLinearSpeed,
        SwingTrackingReserve,
        SwingAngularSpeed,
        PhysicalGeometry,
        SwingReach,
        StanceReach,
        SwingHipEnvelope,
        StanceHipEnvelope,
        KneeEnvelope,
        AnkleEnvelope,
        CommandClosure,
        FootSeparation,
        LaneCrossing,
        Terrain,
        MinimumAdvance
    };

    enum class TurnStepObjective {
        None,
        Translation,
        Angular,
        Combined
    };

    struct TurnLegFeasibilityDiagnostics {
        float reach = 0.0f;
        float reachLimit = 0.0f;
        float reachMargin = 0.0f;
        float kneeBendDeg = 0.0f;
        float kneeMarginDeg = 0.0f;
        float hipClampDeg = 0.0f;
        float hipSwingReserve = 0.0f;
        float hipTwistMarginDeg = 0.0f;
        float ankleClampDeg = 0.0f;
        float ankleHardClampDeg = 0.0f;
        float ankleSwingReserve = 0.0f;
        float ankleTwistMarginDeg = 0.0f;
        float positionClosure = 0.0f;
        bool physicalGeometryValid = false;
        bool safe = false;
    };

    struct TurnPlanDiagnostics {
        glm::vec3 desiredForward { 0.0f, 0.0f, -1.0f };
        glm::vec3 committedForward { 0.0f, 0.0f, -1.0f };
        glm::vec3 activeStartForward { 0.0f, 0.0f, -1.0f };
        glm::vec3 activeStartRight { 1.0f, 0.0f, 0.0f };
        glm::vec3 activeMidForward { 0.0f, 0.0f, -1.0f };
        glm::vec3 activeMidRight { 1.0f, 0.0f, 0.0f };
        glm::vec3 activeEndForward { 0.0f, 0.0f, -1.0f };
        glm::vec3 activeEndRight { 1.0f, 0.0f, 0.0f };
        glm::quat activeStartRotation { 1.0f, 0.0f, 0.0f, 0.0f };
        glm::quat activeEndRotation { 1.0f, 0.0f, 0.0f, 0.0f };

        // Angles are stored in radians and converted to degrees only for telemetry.
        float desiredHeadingError = 0.0f;
        float requestedYaw = 0.0f;
        float admittedYaw = 0.0f;
        float achievedYaw = 0.0f;
        float plannedTurnProgress = 0.0f;
        float achievedTurnProgress = 0.0f;
        float nominalAdvance = 0.0f;
        float headingAdvanceScale = 1.0f;
        float requestedAdvance = 0.0f;
        float admittedAdvance = 0.0f;
        float achievedAdvance = 0.0f;
        TurnStepObjective objective = TurnStepObjective::None;
        bool translationObjectiveSatisfied = false;
        bool angularObjectiveSatisfied = false;

        // Dynamic swing admission is expressed in the same target-space motion that
        // the trajectory generator will command.  Achieved values are measured from
        // the physical sole and accumulated over the admitted step.
        float requestedSwingDistance = 0.0f;
        float admittedSwingDistance = 0.0f;
        float requiredSwingSpeed = 0.0f;
        float admittedSwingSpeed = 0.0f;
        float swingSpeedLimit = 0.0f;
        float minimumLaneSwingSpeed = 0.0f;
        float swingSpeedClosureTolerance = 0.0f;
        float requiredAngularSpeed = 0.0f;
        float admittedAngularSpeed = 0.0f;
        float angularSpeedLimit = 0.0f;
        float achievedSwingSpeed = 0.0f;
        float achievedAngularSpeed = 0.0f;
        // A speed-limited outside step publishes one coupled budget for the
        // immediately following inside step.  These copies make the admission
        // decision and its support/arrival conditioning visible in one record.
        float pairAdvanceScale = 1.0f;
        float pairYawScale = 1.0f;
        float supportTransportSpeed = 0.0f;
        float arrivalTrajectoryT = 0.70f;

        glm::vec3 planOrigin { 0.0f };
        glm::vec3 candidateFootPosition { 0.0f };
        glm::vec3 admittedFootPosition { 0.0f };
        glm::quat candidateFootRotation { 1.0f, 0.0f, 0.0f, 0.0f };
        glm::quat admittedFootRotation { 1.0f, 0.0f, 0.0f, 0.0f };
        glm::quat stanceFootRotationAtAdmission { 1.0f, 0.0f, 0.0f, 0.0f };
        float requestedReach = 0.0f;
        float admittedReach = 0.0f;
        float reachLimit = 0.0f;
        float reachMargin = 0.0f;
        float candidateFootSeparation = 0.0f;
        float admittedFootSeparation = 0.0f;
        float rawFootSeparation = 0.0f;
        float maximumFootSeparation = 0.0f;
        // Counterfactual Slice 2j prediction. Retained for comparison only; it no
        // longer gates or modifies an admitted plan.
        float sampledAnkleClampDeg = 0.0f;
        float admittedKneeSwivelDeg = 0.0f;
        float sampledSwivelClosure = 0.0f;
        bool sampledSwivelAccepted = false;
        float stanceReferenceRotationDeltaDeg = 0.0f;
        float plantReferenceRotationDeltaDeg = 0.0f;
        float touchdownSoleErrorDeg = 0.0f;
        float touchdownAngularSpeed = 0.0f;
        float touchdownHorizontalSpeed = 0.0f;
        glm::vec3 predictedContactHip { 0.0f };
        glm::vec3 actualContactHip { 0.0f };
        glm::vec3 contactHipError { 0.0f };
        glm::vec3 predictedContactSupportVelocity { 0.0f };
        TurnConstraintResult limitingConstraint =
            TurnConstraintResult::NotEvaluated;
        // Slice 3 begins in shadow mode. This proof uses the current physical-chain,
        // zero-swivel execution math but cannot yet reject or alter a foothold. It must
        // first demonstrate that accepted 5-degree turns are not false rejections.
        TurnLegFeasibilityDiagnostics shadowSwing;
        TurnLegFeasibilityDiagnostics shadowStance;
        TurnConstraintResult shadowLimitingConstraint =
            TurnConstraintResult::NotEvaluated;
        float shadowRequestedSwingSpeed = 0.0f;
        float shadowAdmittedSwingSpeed = 0.0f;
        float shadowSwingSpeedLimit = 0.0f;
        float shadowSwingSpeedClampLoss = 0.0f;
        float shadowSwingSpeedClampReference = 0.0f;
        bool swingFootLeft = false;
        bool outsideFoot = false;
        bool pairBudgetApplied = false;
        bool pairBudgetLatched = false;
        bool swingSpeedLaneFloorApplied = false;
        bool swingSpeedLaneFloorExceeded = false;
        bool turnInitiationDeferred = false;
        bool turnExitBlendApplied = false;
        bool footSeparationLimited = false;
        bool candidateGrounded = false;
        bool candidateEvaluated = false;
        bool candidateAccepted = false;
        bool shadowFeasibilityEvaluated = false;
        bool shadowFeasibilitySafe = false;
        bool shadowRuntimeAccepted = false;
        bool activeHeadingPlan = false;
        bool touchdownReadinessBlockedLogged = false;
    };

    TurnPlanDiagnostics _gaitTurnPlan;
    bool _gaitTurnPairPendingInside = false;
    float _gaitTurnPairAdvanceScale = 1.0f;
    float _gaitTurnPairYawScale = 1.0f;
    float _gaitTurnPairYawSign = 0.0f;
    bool _gaitTurnExitBlendPending = false;
    // A large physical direction change remains a step-planner objective until it
    // converges. Exact opposites have no unique atan2 sign, so latch the direction
    // selected by the first available outside foot instead of allowing frame noise to
    // alternate between clockwise and counter-clockwise plans.
    bool _gaitPhysicalReversalActive = false;
    bool _gaitReversalSideLatched = false;
    float _gaitReversalYawSign = 0.0f;
    glm::vec3 _gaitReversalTargetForward { 0.0f, 0.0f, -1.0f };
    int _gaitReversalStepCount = 0;
    // Live intent is independent of the immutable admitted step. Retarget telemetry
    // compares against this accumulated reference so slowly moving analog input still
    // produces one meaningful record after crossing the angular notice threshold.
    glm::vec3 _gaitRetargetReferenceForward { 0.0f, 0.0f, -1.0f };
    bool _gaitRetargetReferenceValid = false;
    int _gaitRetargetSequence = 0;
    // An early released swing returns to its old plant while the body-heading target
    // smoothly unwinds from the command already issued this frame to the step-entry
    // heading. The immutable admitted plan remains intact for diagnostics.
    bool _gaitTurnCancellationUnwindActive = false;
    glm::quat _gaitTurnCancellationStartRotation { 1.0f, 0.0f, 0.0f, 0.0f };
    glm::quat _gaitTurnCancellationEndRotation { 1.0f, 0.0f, 0.0f, 0.0f };
    glm::vec3 _gaitTurnCancellationStartForward { 0.0f, 0.0f, -1.0f };
    glm::vec3 _gaitTurnCancellationEndForward { 0.0f, 0.0f, -1.0f };
    float _gaitTurnCancellationProgress = 0.0f;

    struct LegState {
        int footIdx = -1, ankleIdx = -1, kneeIdx = -1, hipIdx = -1;
        glm::vec3 kneeHingeAxis { 1.0f, 0.0f, 0.0f };
        float kneeMinDeg = 0.0f, kneeMaxDeg = 150.0f;
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
        // Physical constraint-chain geometry captured in each parent body's local
        // frame. Runtime IK must use the same link vectors the ragdoll actually
        // enforces; imported skeleton localT remains animation/skin geometry.
        glm::vec3 upperSegmentLocal { 0.0f };
        glm::vec3 lowerSegmentLocal { 0.0f };
        bool segmentGeometryValid = false;
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
        bool plantSolveValid = false;
        glm::quat hipCommand { 1, 0, 0, 0 };
        glm::quat kneeCommand { 1, 0, 0, 0 };
        glm::quat ankleCommand { 1, 0, 0, 0 };
        glm::quat footCommand { 1, 0, 0, 0 };
        // Continuous knee-plane redundancy state. Slice 2m admits changes at a bounded
        // rate so turn orientation can move through the whole leg without branch flips.
        float kneePlaneSwivelRad = 0.0f;
    };

    float _yaw = 0.0f;
    float _timeSincePowered = -1.0f;
    float _gaitWeight = 0.0f;
    float _poseBlend = 0.0f;
    bool  _grounded = true;
    glm::vec3 _right { 0.0f }, _fwd { 0.0f };
    glm::vec3 _desiredVelocity { 0.0f };
    bool  _physicalStepBaselineValid = false;
    bool  _physicalStepContactL = false, _physicalStepContactR = false;
    bool  _physicalStepPrevSwingContact = false;
    bool  _physicalStepTouchdownAccepted = false;
    bool  _physicalStepTouchdownContactValid = false;
    bool  _physicalStepAborted = false;
    float _physicalStepTime = 0.0f;
    float _physicalStepSettleTime = 0.0f;
    int   _physicalStepPhase = 0;
    float _physicalStepPhaseTime = 0.0f;
    int   _physicalStepSupportSide = 0;
    float _physicalStepComCommand = 0.0f;
    float _physicalStepComLateral = 0.0f, _physicalStepTargetLateral = 0.0f;
    float _physicalStepAirborneTime = 0.0f;
    float _physicalStepArrivalStableTime = 0.0f;
    float _physicalStepReachLimit = 0.0f;
    float _physicalStepPlantAcquireStableTime = 0.0f;
    float _physicalStepPlantSettledOffsetTime = 0.0f;
    float _physicalStepPlantUnsafeTime = 0.0f;
    bool _physicalStepPlantAnchorRebased = false;
    // Landing owns the first material contact only while the sole is still rocking.
    // Once it is level and quiet, ownership blends to a frozen sole-center anchor.
    bool _physicalStepPlantCenterAnchorActive = false;
    bool _physicalStepPlantContactMigrationLogged = false;
    bool _physicalStepPlantPivotReleaseLatched = false;
    float _physicalStepPlantPivotStableTime = 0.0f;
    float _physicalStepPlantPivotMaxStableTime = 0.0f;
    float _physicalStepPlantPivotReleaseTriggerTime = 0.0f;
    float _physicalStepPlantPivotReleaseTime = 0.0f;
    float _physicalStepPlantPivotReleaseWeight = 0.0f;
    float _physicalStepPlantCenterBlendTime = 0.0f;
    float _physicalStepPlantAnchorTelemetryTime = 0.0f;
    float _physicalStepPlantAnchorHandoffPhaseTime = -1.0f;
    float _physicalStepPlantPivotContactBlockedTime = 0.0f;
    float _physicalStepPlantPivotSoleBlockedTime = 0.0f;
    float _physicalStepPlantPivotAngularBlockedTime = 0.0f;
    float _physicalStepPlantPivotLinearBlockedTime = 0.0f;
    float _physicalStepPlantContactMigration = 0.0f;
    float _physicalStepPlantAngularSpeed = 0.0f;
    glm::vec3 _physicalStepPlantCenterAnchorStart { 0.0f };
    glm::vec3 _physicalStepPlantCenterAnchorTarget { 0.0f };
    float _physicalStepTrajectoryT = 0.0f;
    // The landing brake is a one-way state: readiness must remain true briefly
    // before descent is released, and can never pull an admitted trajectory back.
    float _gaitLandingBrakeReadyTime = 0.0f;
    bool _gaitLandingBrakeReleased = false;
    // Slice 2g: the inside turn owns a measured-state trajectory governor.
    // Position and world-space sole commands retain their rates across phase
    // boundaries so ARRIVAL and DESCENT cannot reintroduce an impulse.
    float _gaitSwingCommandSpeed = 0.0f;
    float _gaitSwingCommandAngularSpeed = 0.0f;
    float _gaitSwingCommandTrackingError = 0.0f;
    float _gaitSwingSoleCommandErrorDeg = 0.0f;
    // Slice 2h: measured-parent cancellation is bounded before the final local
    // ankle motor command is acceleration/rate limited. The path fields retain
    // the sampled 3D timing proof used by the swing safety deadline.
    float _gaitAnkleCommandAngularSpeed = 0.0f;
    float _gaitAnkleCommandDeltaDeg = 0.0f;
    float _gaitAnkleParentCompensationDeg = 0.0f;
    float _gaitAnkleParentCompensationAppliedDeg = 0.0f;
    float _gaitAnkleAchievableResidualDeg = 0.0f;
    float _gaitAnkleDiagnosticCooldown = 0.0f;
    float _gaitIkAnkleEnvelopeClampDeg = 0.0f;
    float _gaitIkExactAnkleClampDeg = 0.0f;
    float _gaitIkKneeSwivelRequestedDeg = 0.0f;
    float _gaitIkKneeSwivelAppliedDeg = 0.0f;
    float _gaitIkKneeSwivelCandidateClosure = 0.0f;
    bool _gaitIkKneeSwivelCandidateAccepted = false;
    float _gaitIkOrientationPriority = 1.0f;
    float _gaitIkOrientationRelaxedDeg = 0.0f;
    float _gaitIkAnkleSwingUsage = 0.0f;
    float _gaitIkAnkleSwingReserve = 0.0f;
    float _gaitIkAnkleTwistDeg = 0.0f;
    float _gaitIkAnkleTwistMarginDeg = 0.0f;
    // Slice 2m: one shared rate admission for the complete swing-leg command.
    float _gaitCoherentJointScale = 1.0f;
    float _gaitCoherentHipDeltaDeg = 0.0f;
    float _gaitCoherentKneeDeltaDeg = 0.0f;
    float _gaitCoherentAnkleDeltaDeg = 0.0f;
    // Slice 2k diagnostic: split swing-foot error into IK/FK closure error and
    // powered-ragdoll tracking error. No field participates in control routing.
    glm::vec3 _gaitFkDesiredPosition { 0.0f };
    glm::vec3 _gaitFkCommandPosition { 0.0f };
    glm::vec3 _gaitFkPhysicalPosition { 0.0f };
    glm::vec3 _gaitFkDesiredToCommand { 0.0f };
    glm::vec3 _gaitFkCommandToPhysical { 0.0f };
    glm::vec3 _gaitFkDesiredToPhysical { 0.0f };
    float _gaitFkDesiredToCommandOrientationDeg = 0.0f;
    float _gaitFkCommandToPhysicalOrientationDeg = 0.0f;
    float _gaitFkDesiredToPhysicalOrientationDeg = 0.0f;
    float _gaitFkDiagnosticCooldown = 0.0f;
    bool _gaitFkValid = false;
    float _gaitSwingPathLength = 0.0f;
    float _gaitSwingMinimumDuration = 0.0f;
    float _gaitSwingDeadline = 0.0f;
    float _gaitSwingWatchdogProgress = 0.0f;
    float _gaitSwingNoProgressTime = 0.0f;
    glm::quat _gaitSwingSoleCommandWorld { 1.0f, 0.0f, 0.0f, 0.0f };
    bool _gaitSwingSoleCommandValid = false;
    // Center rise is useful trajectory telemetry, but it is not ground clearance when
    // the box-shaped sole pitches. Capture the physics-computed lowest sole point at
    // release and track its actual rise independently.
    float _physicalStepSoleStartMinY = 0.0f;
    float _physicalStepSoleClearance = 0.0f;
    float _physicalStepContactPenetration = 0.0f;
    bool _physicalStepSoleReferenceValid = false;
    float _physicalStepClearance = 0.0f;
    float _physicalStepForwardTravel = 0.0f;
    float _physicalStepTargetError = 0.0f;
    float _physicalStepHorizontalTargetError = 0.0f;
    float _physicalStepForwardTargetError = 0.0f;
    float _physicalStepLateralTargetError = 0.0f;
    float _physicalStepVerticalTargetError = 0.0f;
    float _physicalStepTouchdownVy = 0.0f;
    float _physicalStepTouchdownNormalY = 0.0f;
    float _physicalStepPlantCenterTravel = 0.0f;
    float _physicalStepStanceDrift = 0.0f, _physicalStepPlantDrift = 0.0f;
    float _physicalStepMaxStanceDrift = 0.0f, _physicalStepMaxPlantDrift = 0.0f;
    float _physicalStepInitialTilt = 0.0f, _physicalStepPeakTilt = 0.0f, _physicalStepFinalTilt = 0.0f;
    float _physicalStepMaxMotorRatio = 0.0f;
    bool  _physicalStepMotorSaturated = false;
    bool  _physicalStepPlantPoseCaptured = false;
    glm::vec3 _physicalStepFootBaselineL { 0.0f };
    glm::vec3 _physicalStepFootBaselineR { 0.0f };
    glm::vec3 _physicalStepComBaseline { 0.0f };
    glm::vec3 _physicalStepRight { 1.0f, 0.0f, 0.0f };
    glm::vec3 _physicalStepForward { 0.0f, 0.0f, -1.0f };
    glm::vec3 _physicalStepSupportTarget { 0.0f };
    glm::vec3 _physicalStepSwingStart { 0.0f };
    glm::vec3 _physicalStepArcStart { 0.0f };
    glm::vec3 _physicalStepFoothold { 0.0f };
    glm::vec3 _physicalStepDesiredFoot { 0.0f };
    glm::vec3 _physicalStepTouchdownPlant { 0.0f };
    glm::vec3 _physicalStepTouchdownContactWorld { 0.0f };
    glm::vec3 _physicalStepTouchdownContactLocal { 0.0f };
    glm::vec3 _physicalStepApiVelocity { 0.0f };
    glm::vec3 _physicalStepMeasuredVelocity { 0.0f };
    glm::vec3 _physicalStepPreviousSwingFoot { 0.0f };
    bool _physicalStepPreviousSwingFootValid = false;
    float _physicalStepFootUpY = 1.0f;
    glm::vec3 _physicalStepContactPoint { 0.0f };
    glm::vec3 _physicalStepContactLocal { 0.0f };
    float _supportTransferTransferT = 0.0f;
    float _supportTransferHoldStableTime = 0.0f;
    float _supportTransferContactLossTime = 0.0f;
    float _supportTransferComError = 0.0f;
    float _supportTransferComToOldSupport = 0.0f;
    float _supportTransferComToNewSupport = 0.0f;
    float _supportTransferComHorizontalSpeed = 0.0f;
    glm::vec3 _supportTransferTransferStartTarget { 0.0f };
    glm::vec3 _supportTransferTransferEndTarget { 0.0f };
    int _stepSequenceStepIndex = 0;
    int _stepSequenceStepsCompleted = 0;
    float _stepSequenceInterStepStableTime = 0.0f;
    float _stepSequenceInitialTilt = 0.0f;
    float _stepSequenceStepForward[2] { 0.0f, 0.0f };
    float _stepSequenceStepMaxDrift[2] { 0.0f, 0.0f };
    float _stepSequenceStepPeakTilt[2] { 0.0f, 0.0f };
    float _stepSequenceStepMotorRatio[2] { 0.0f, 0.0f };
    bool _stepSequencePreviousContactsValid = false;
    bool _stepSequencePreviousContactL = false;
    bool _stepSequencePreviousContactR = false;
    int _stepSequenceContactTransitionsL = 0;
    int _stepSequenceContactTransitionsR = 0;
    float _gaitContactChangeTimeL = 0.0f;
    float _gaitContactChangeTimeR = 0.0f;
    bool _gaitRunning = false;
    bool _gaitStopRequested = false;
    bool _gaitLandingObjectiveStopRequested = false;
    float _gaitRunTime = 0.0f;
    float _gaitStepStartTime = 0.0f;
    float _gaitLastLaunchTime = 0.0f;
    float _gaitLastLaunchPeriod = 0.0f;
    float _gaitStepRecontactPauseTime = 0.0f;
    float _gaitStepMaxSoleErrorDeg = 0.0f;
    float _gaitStepMaxSwingAngularSpeed = 0.0f;
    int _gaitStepSpeculativeContacts = 0;
    bool _gaitLastLaunchValid = false;
    bool _gaitSpeculativeContactActive = false;
    float _gaitLastStepPeriod = 0.0f;
    float _gaitPreviousStepPeriod = 0.0f;
    float _gaitMeasuredSpeed = 0.0f;
    float _gaitFilteredForwardError = 0.0f;
    float _gaitFilteredLateralError = 0.0f;
    float _gaitFilteredTouchdownSpeed = 0.0f;
    float _gaitFilteredDrift = 0.0f;
    float _gaitFilteredMotorRatio = 0.0f;
    float _gaitFilteredUnloadDeficit = 0.0f;
    float _gaitAdaptiveStrideOffset = 0.0f;
    float _gaitAdaptiveLateralOffset = 0.0f;
    float _gaitAdaptivePeriodOffset = 0.0f;
    float _gaitAdaptiveTransferBiasOffset = 0.0f;
    float _gaitStress = 0.0f;
    int _gaitRecoveryFailureSteps = 0;
    bool _gaitAdaptiveStopRequested = false;
    float _gaitPhaseTimeScale = 1.0f;
    float _gaitCommandedStepLength = 0.10f;
    float _gaitReachCommandCeiling = 0.14f;
    float _gaitTakeoffContactRecoveryTime = 0.0f;
    float _gaitSwingRecontactTime = 0.0f;
    float _gaitSettledTrackingLoss = 0.0f;
    float _gaitForwardPreShift = 0.0f;
    int _gaitReachClearSteps = 0;
    float _gaitPlannedSupportAdvance = 0.0f;
    float _gaitAchievedSupportAdvance = 0.0f;
    float _gaitLastStepLength = 0.0f;
    float _gaitPreviousStepLength = 0.0f;
    float _gaitLastSupportAdvance = 0.0f;
    float _gaitPreviousSupportAdvance = 0.0f;
    float _gaitStepMaxRelevantDrift = 0.0f;
    float _gaitMaxDrift = 0.0f;
    float _gaitPeakTilt = 0.0f;
    float _gaitMaxMotorRatio = 0.0f;
    float _gaitStopStableTime = 0.0f;
    float _gaitStopFootDriftL = 0.0f;
    float _gaitStopFootDriftR = 0.0f;
    float _gaitStopMaxFootDrift = 0.0f;
    float _gaitStopSettleFootDriftL = 0.0f;
    float _gaitStopSettleFootDriftR = 0.0f;
    float _gaitStopMaxSettleFootDrift = 0.0f;
    float _gaitCrouchBlend = 0.0f;
    float _gaitIkRequestedReach = 0.0f;
    float _gaitIkClampedReach = 0.0f;
    float _gaitIkMaxReach = 0.0f;
    float _gaitIkPhysicalReach = 0.0f;
    float _gaitIkReachShortfall = 0.0f;
    float _gaitIkReachShortfallForward = 0.0f;
    float _gaitIkHipEnvelopeClampDeg = 0.0f;
    float _gaitIkHipCommandLagDeg = 0.0f;
    float _gaitIkKneeCommandLagDeg = 0.0f;
    float _gaitIkKneeBendDeg = 0.0f;
    float _gaitIkHipTravelForward = 0.0f;
    float _gaitIkHipTravelLateral = 0.0f;
    float _gaitIkHipTravelVertical = 0.0f;
    float _gaitFootCorrection = 0.0f;
    float _gaitFootCorrectionForward = 0.0f;
    float _gaitFootTargetSpeed = 0.0f;
    float _gaitSoleLevelBlend = 0.0f;
    float _gaitSoleAngularErrorDeg = 0.0f;
    float _gaitPlantPreviousDrift = 0.0f;
    float _gaitPlantDriftRate = 0.0f;
    bool _gaitPlantRecoveryLogged = false;
    float _gaitPlantCorrectionPeakRequested = 0.0f;
    float _gaitPlantCorrectionPeakApplied = 0.0f;
    bool _gaitPlantCorrectionSaturated = false;
    float _gaitPlantCorrectionRequested = 0.0f;
    float _gaitPlantCorrectionApplied = 0.0f;
    bool _gaitPlantCorrectionAtLimit = false;
    float _gaitInterStepRecenterT = 0.0f;
    float _gaitInterStepCenterError = 0.0f;
    float _gaitRootPitchRate = 0.0f;
    float _gaitRootRollRate = 0.0f;
    float _gaitRootYawRate = 0.0f;
    float _gaitRootTiltRate = 0.0f;
    float _gaitHeadingErrorDeg = 0.0f;
    float _gaitPeakHeadingErrorDeg = 0.0f;
    bool _gaitTakeoffContactRecoveryActive = false;
    bool _gaitIkPlanHipValid = false;
    bool _gaitReachClampedStep = false;
    bool _gaitOldSupportDriftAllowanceLogged = false;
    // 0 = ordinary step, 1 = return an early swing to its original plant,
    // 2 = finish a late swing at a shortened safe foothold.
    int _gaitCancelMode = 0;
    bool _gaitLandingVerificationPending = false;
    float _gaitLandingStableTime = 0.0f;
    bool _gaitStopSettleReferenceValid = false;
    // Once the first transfer has established a real support foot, subsequent steps keep
    // one continuous support target and bypass the stop/recenter/weight-shift reset.
    bool _gaitContinuousCycle = false;
    bool _gaitBypassWeightShift = false;
    float _gaitNewSupportLoad = 0.0f;
    bool _gaitNewSupportLoadLatched = false;
    glm::vec3 _gaitCycleSupportTarget { 0.0f };
    // One velocity-continuous pelvis/support command spans late swing, touchdown,
    // transfer, and the following takeoff. FSM phases only validate that motion.
    bool _gaitSupportCurveActive = false;
    int _gaitSupportCurveStep = -1;
    float _gaitSupportCurveTime = 0.0f;
    float _gaitSupportCurveDuration = 0.0f;
    glm::vec3 _gaitSupportCurveStart { 0.0f };
    glm::vec3 _gaitSupportCurveEnd { 0.0f };
    glm::vec3 _gaitSupportCurveStartVelocity { 0.0f };
    glm::vec3 _gaitSupportCurveEndVelocity { 0.0f };
    glm::vec3 _gaitSupportCommandVelocity { 0.0f };
    // True while the physical gait owns the walking pose.
    bool _gaitEnabled = false;
    bool _runtimeWalkIntent = false;
    bool _runtimeRestartBlocked = false;
    // A rejected physical reversal completes the validated stop but cannot restart
    // against the identical command forever. Release or a meaningful direction change
    // clears this target-specific block.
    bool _runtimeRestartBlockForwardValid = false;
    glm::vec3 _runtimeRestartBlockForward { 0.0f, 0.0f, -1.0f };
    int _runtimeAutoRetryCount = 0;
    float _runtimeRecoveryCooldown = 0.0f;
    float _runtimeRecoveryStableTime = 0.0f;
    int _runtimeNextSupportSide = -1;
    glm::vec3 _gaitStartCom { 0.0f };
    glm::vec3 _gaitStepStartCom { 0.0f };
    glm::vec3 _gaitIkPlanHip { 0.0f };
    glm::vec3 _gaitInterStepRecenterStart { 0.0f };
    glm::vec3 _gaitInterStepRecenterTarget { 0.0f };
    glm::vec3 _gaitStopStartTarget { 0.0f };
    glm::vec3 _gaitStopEndTarget { 0.0f };
    glm::vec3 _gaitStopFootTargetL { 0.0f };
    glm::vec3 _gaitStopFootTargetR { 0.0f };
    glm::vec3 _gaitStopSettleFootTargetL { 0.0f };
    glm::vec3 _gaitStopSettleFootTargetR { 0.0f };
    glm::quat _gaitHeadingTargetRot { 1.0f, 0.0f, 0.0f, 0.0f };
    LegState _legL, _legR;
};
template<>
inline void DrawComponentInspector<LocamotionControllerComponent>(LocamotionControllerComponent& c)
{
#define LOCO_DRAG(label, name, speed, low, high, format) ImGui::DragFloat(label, &c.name, speed, low, high, format)
    if (ImGui::CollapsingHeader("Movement", ImGuiTreeNodeFlags_DefaultOpen)) {
        LOCO_DRAG("Max Speed", maxSpeed, 0.05f, 0.0f, 10.0f, "%.2f m/s");
        LOCO_DRAG("Input Deadzone", deadzone, 0.01f, 0.0f, 0.9f, "%.2f");
        LOCO_DRAG("Minimum Walk Speed", minWalkSpeed, 0.01f, 0.0f, 2.0f, "%.2f m/s");
        LOCO_DRAG("Facing Offset", facingOffsetDeg, 1.0f, -180.0f, 180.0f, "%.0f deg");
        LOCO_DRAG("Ground Probe", groundRayLength, 0.05f, 0.1f, 10.0f, "%.2f m");
        ImGui::SliderFloat("Consciousness", &c.consciousness, 0.0f, 1.0f);
        LOCO_DRAG("Powered Grace", postPoweredGrace, 0.01f, 0.0f, 2.0f, "%.2f s");
        LOCO_DRAG("Gait Blend", gaitBlendTime, 0.01f, 0.01f, 1.0f, "%.2f s");
    }
    if (ImGui::CollapsingHeader("Step Planning", ImGuiTreeNodeFlags_DefaultOpen)) {
        LOCO_DRAG("Weight Shift", weightShiftDuration, 0.01f, 0.05f, 3.0f, "%.2f s");
        LOCO_DRAG("Support Frequency", supportFrequency, 0.1f, 0.1f, 10.0f, "%.1f Hz");
        LOCO_DRAG("Support Max Acceleration", supportMaxAcceleration, 0.1f, 0.1f, 20.0f, "%.2f");
        ImGui::SliderFloat("Support Bias", &c.supportBias, 0.0f, 1.0f);
        LOCO_DRAG("Takeoff Height", takeoffHeight, 0.005f, 0.0f, 0.3f, "%.3f m");
        LOCO_DRAG("Takeoff Timeout", takeoffTimeout, 0.01f, 0.05f, 2.0f, "%.2f s");
        LOCO_DRAG("Lift Frequency", liftFrequency, 0.1f, 0.1f, 20.0f, "%.1f Hz");
        LOCO_DRAG("Lift Max Force", liftMaxForce, 5.0f, 0.0f, 2000.0f, "%.0f N");
        LOCO_DRAG("Step Length", stepLength, 0.005f, 0.02f, 0.8f, "%.3f m");
        LOCO_DRAG("Swing Height", swingHeight, 0.005f, 0.01f, 0.4f, "%.3f m");
        LOCO_DRAG("Swing Duration", swingDuration, 0.01f, 0.1f, 2.0f, "%.2f s");
        LOCO_DRAG("Arrival Height", arrivalHeight, 0.005f, 0.0f, 0.2f, "%.3f m");
        LOCO_DRAG("Arrival Tolerance", arrivalTolerance, 0.002f, 0.002f, 0.15f, "%.3f m");
        LOCO_DRAG("Arrival Settle", arrivalSettleDuration, 0.01f, 0.0f, 1.0f, "%.2f s");
        LOCO_DRAG("Arrival Timeout", arrivalTimeout, 0.01f, 0.05f, 2.0f, "%.2f s");
        LOCO_DRAG("Descent Duration", descentDuration, 0.01f, 0.05f, 1.5f, "%.2f s");
        LOCO_DRAG("Plant Timeout", plantTimeout, 0.01f, 0.05f, 2.0f, "%.2f s");
        LOCO_DRAG("Plant Acquire", plantAcquireDuration, 0.01f, 0.0f, 1.0f, "%.2f s");
        LOCO_DRAG("Plant Max Speed", plantAcquireMaxSpeed, 0.005f, 0.0f, 1.0f, "%.3f m/s");
        LOCO_DRAG("Plant Acquire Timeout", plantAcquireTimeout, 0.01f, 0.05f, 2.0f, "%.2f s");
        LOCO_DRAG("Contact Settle", contactSettleDuration, 0.01f, 0.0f, 1.5f, "%.2f s");
        LOCO_DRAG("Touchdown Max Y Speed", touchdownMaxVerticalSpeed, 0.01f, 0.0f, 2.0f, "%.2f m/s");
        LOCO_DRAG("Touchdown Max Horizontal", touchdownMaxHorizontalSpeed, 0.01f, 0.05f, 2.0f, "%.2f m/s");
        LOCO_DRAG("Touchdown Max Angular", touchdownMaxAngularSpeed, 0.05f, 0.25f, 10.0f, "%.2f rad/s");
        LOCO_DRAG("Touchdown Max Command", touchdownMaxCommandSpeed, 0.01f, 0.05f, 1.0f, "%.2f m/s");
        LOCO_DRAG("Touchdown Min Trajectory", touchdownMinTrajectoryProgress, 0.01f, 0.45f, 1.0f, "%.2f");
        ImGui::SliderFloat("Touchdown Min Normal Y", &c.touchdownMinNormalY, 0.0f, 1.0f);
        LOCO_DRAG("Foot Target Tolerance", footTargetTolerance, 0.002f, 0.002f, 0.2f, "%.3f m");
        ImGui::SliderFloat("Safe Reach Fraction", &c.safeReachFraction, 0.5f, 1.0f);
    }
    if (ImGui::CollapsingHeader("Gait", ImGuiTreeNodeFlags_DefaultOpen)) {
        LOCO_DRAG("Target Gait Speed", gaitDesiredSpeed, 0.005f, 0.0f, 2.0f, "%.3f m/s");
        LOCO_DRAG("Target Step Period", gaitTargetStepPeriod, 0.01f, 0.0f, 4.0f, "%.2f s");
        LOCO_DRAG("Motion Safety Floor Scale", gaitCadenceFloorScale, 0.01f, 0.20f, 1.50f, "%.2fx");
        LOCO_DRAG("Validation Floor Scale", gaitValidationFloorScale, 0.01f, 0.20f, 1.50f, "%.2fx");
        LOCO_DRAG("Support Motion Speed Cap", gaitSupportMaxSpeed, 0.01f, 0.01f, 1.0f, "%.2f m/s");
        LOCO_DRAG("Support Transport Speed", gaitSupportTransportSpeed, 0.01f, 0.03f, 1.0f, "%.2f m/s");
        ImGui::Checkbox("Online Gait Adaptation", &c.gaitAdaptationEnabled);
        LOCO_DRAG("Adaptation Response", gaitAdaptationResponse, 0.01f, 0.01f, 1.0f, "%.2f");
        LOCO_DRAG("Max Stride Correction", gaitMaxStrideCorrection, 0.002f, 0.0f, 0.10f, "%.3f m");
        LOCO_DRAG("Max Period Slowdown", gaitMaxPeriodSlowdown, 0.01f, 0.0f, 2.0f, "%.2f s");
        LOCO_DRAG("Stress Stop Threshold", gaitStressStopThreshold, 0.1f, 0.5f, 10.0f, "%.1f");
        LOCO_DRAG("Transfer Duration", transferDuration, 0.01f, 0.05f, 3.0f, "%.2f s");
        ImGui::SliderFloat("Transfer Support Bias", &c.transferSupportBias, 0.0f, 1.0f);
        LOCO_DRAG("Transfer COM Tolerance", transferComTolerance, 0.002f, 0.002f, 0.2f, "%.3f m");
        LOCO_DRAG("Transfer Hold", transferHoldDuration, 0.01f, 0.0f, 2.0f, "%.2f s");
        LOCO_DRAG("Transfer Timeout", transferTimeout, 0.01f, 0.1f, 5.0f, "%.2f s");
        LOCO_DRAG("Inter-Step Duration", interStepDuration, 0.01f, 0.0f, 2.0f, "%.2f s");
        LOCO_DRAG("Drift Growth Tolerance", driftGrowthTolerance, 0.001f, 0.0f, 0.2f, "%.3f m");
        LOCO_DRAG("Placement Gain", gaitPlacementGain, 0.01f, 0.0f, 4.0f, "%.2f");
        LOCO_DRAG("Nominal Advance", gaitNominalAdvance, 0.005f, 0.0f, 0.5f, "%.3f m");
        LOCO_DRAG("Minimum Advance", gaitMinStepLength, 0.005f, 0.0f, 0.5f, "%.3f m");
        LOCO_DRAG("Maximum Advance", gaitMaxStepLength, 0.005f, 0.0f, 0.8f, "%.3f m");
        LOCO_DRAG("Reach Crouch", gaitReachCrouch, 0.002f, 0.0f, 0.2f, "%.3f m");
        LOCO_DRAG("Crouch Ramp", gaitCrouchTime, 0.01f, 0.01f, 2.0f, "%.2f s");
        ImGui::SliderFloat("Usable Reach Fraction", &c.gaitUsableReachFraction, 0.5f, 1.0f);
        LOCO_DRAG("Sole Level Time", gaitSoleLevelTime, 0.01f, 0.01f, 2.0f, "%.2f s");
        LOCO_DRAG("Foot Position Gain", gaitFootPositionGain, 0.01f, 0.0f, 4.0f, "%.2f");
        LOCO_DRAG("Foot Velocity Lead", gaitFootVelocityLeadTime, 0.01f, 0.0f, 0.5f, "%.2f s");
        LOCO_DRAG("Max Foot Correction", gaitMaxFootCorrection, 0.002f, 0.0f, 0.2f, "%.3f m");
        LOCO_DRAG("Walk Foot Speed", gaitSwingSpeedLimit, 0.05f, 0.25f, 4.0f, "%.2f m/s");
        LOCO_DRAG("Walk Foot Accel", gaitSwingAcceleration, 0.5f, 1.0f, 30.0f, "%.1f m/s^2");
        LOCO_DRAG("Walk Sole Angular Speed", gaitSoleAngularSpeedLimit, 0.05f, 0.25f, 8.0f, "%.2f rad/s");
        LOCO_DRAG("Walk Sole Angular Accel", gaitSoleAngularAcceleration, 0.5f, 1.0f, 40.0f, "%.1f rad/s^2");
        LOCO_DRAG("Landing Linear Damping", gaitLandingLinearDampingTime, 0.005f, 0.0f, 0.25f, "%.3f s");
        LOCO_DRAG("Inter-Step Recenter", gaitInterStepRecenterTime, 0.01f, 0.01f, 2.0f, "%.2f s");
        LOCO_DRAG("Upright Stiffness", gaitUprightStiffness, 10.0f, 0.0f, 3000.0f, "%.0f");
        LOCO_DRAG("Upright Damping", gaitUprightDamping, 5.0f, 0.0f, 500.0f, "%.0f");
        LOCO_DRAG("Upright Max Torque", gaitUprightMaxTorque, 10.0f, 0.0f, 2000.0f, "%.0f");
        LOCO_DRAG("Heading Stiffness", gaitHeadingStiffness, 5.0f, 0.0f, 1000.0f, "%.0f");
        LOCO_DRAG("Heading Damping", gaitHeadingDamping, 2.0f, 0.0f, 300.0f, "%.0f");
        LOCO_DRAG("Heading Max Torque", gaitHeadingMaxTorque, 5.0f, 0.0f, 1000.0f, "%.0f");
        LOCO_DRAG("Maximum Turn Per Step", gaitMaxTurnStepDeg, 0.25f, 0.0f, 45.0f, "%.2f deg");
        LOCO_DRAG("Turn Full Advance Through", gaitTurnFullAdvanceDeg, 1.0f, 0.0f, 90.0f, "%.1f deg");
        LOCO_DRAG("Turn Zero Advance At", gaitTurnZeroAdvanceDeg, 1.0f, 0.0f, 180.0f, "%.1f deg");
        LOCO_DRAG("Inside Turn Foot Speed", gaitTurnInsideSwingSpeedLimit, 0.01f, 0.10f, 2.0f, "%.2f m/s");
        LOCO_DRAG("Outside Turn Foot Speed", gaitTurnOutsideSwingSpeedLimit, 0.01f, 0.10f, 2.0f, "%.2f m/s");
        LOCO_DRAG("Turn Foot Angular Speed", gaitTurnAngularSpeedLimit, 0.05f, 0.25f, 8.0f, "%.2f rad/s");
        LOCO_DRAG("Turn Foot Linear Accel", gaitTurnLinearAcceleration, 0.10f, 0.25f, 10.0f, "%.2f m/s^2");
        LOCO_DRAG("Turn Foot Angular Accel", gaitTurnAngularAcceleration, 0.50f, 1.0f, 40.0f, "%.1f rad/s^2");
        LOCO_DRAG("Maximum Foot Separation", gaitMaxFootSeparation, 0.005f, 0.10f, 0.30f, "%.3f m");
        LOCO_DRAG("Inter-Step Tilt Gate", gaitInterStepTiltLimit, 0.5f, 0.0f, 45.0f, "%.1f deg");
        LOCO_DRAG("Inter-Step Heading Gate", gaitInterStepHeadingLimit, 0.5f, 0.0f, 45.0f, "%.1f deg");
        LOCO_DRAG("Stop Duration", gaitStopTime, 0.01f, 0.05f, 3.0f, "%.2f s");
        LOCO_DRAG("Stop Hold", gaitStopHoldTime, 0.01f, 0.0f, 2.0f, "%.2f s");
        if (c.gaitAdaptationEnabled) {
            ImGui::Text("Adaptive: stride %+.3fm  period +%.2fs  transfer +%.3f  stress %.2f  failSteps %d",
                        c._gaitAdaptiveStrideOffset,
                        c._gaitAdaptivePeriodOffset,
                        c._gaitAdaptiveTransferBiasOffset,
                        c._gaitStress,
                        c._gaitRecoveryFailureSteps);
        }
    }
    if (ImGui::CollapsingHeader("Pose and IK", ImGuiTreeNodeFlags_DefaultOpen)) {
        ImGui::SliderFloat("Maximum Leg Reach", &c.maxLegReachFraction, 0.5f, 1.0f);
        LOCO_DRAG("Standing Pose Response", standingPoseResponse, 0.01f, 0.01f, 1.0f, "%.2f s");
        LOCO_DRAG("Hip Limit Margin", hipLimitMarginDeg, 0.5f, 0.0f, 20.0f, "%.1f deg");
        ImGui::SliderFloat("Pose Weight", &c.poseWeight, 0.0f, 1.0f);
        ImGui::SliderFloat("Upright Scale", &c.uprightScale, 0.0f, 1.0f);
        ImGui::Checkbox("IK Write Enabled", &c.ikWriteEnabled);
        auto boneField = [](const char* label, std::string& name) {
            char buffer[128]; std::snprintf(buffer, sizeof(buffer), "%s", name.c_str());
            if (ImGui::InputText(label, buffer, sizeof(buffer))) name = buffer;
        };
        boneField("Left Foot Bone", c.leftFootBone);
        boneField("Right Foot Bone", c.rightFootBone);
        boneField("Torso Bone", c.torsoBone);
    }
    ImGui::Checkbox("Debug Locomotion", &c.debug);
#undef LOCO_DRAG
}

template<>
inline std::string SerializeComponent<LocamotionControllerComponent>(const LocamotionControllerComponent& c)
{
    nlohmann::json j;
#define LOCO_SAVE(name) j[#name] = c.name
    LOCO_SAVE(maxSpeed); LOCO_SAVE(deadzone); LOCO_SAVE(minWalkSpeed);
    LOCO_SAVE(facingOffsetDeg); LOCO_SAVE(groundRayLength); LOCO_SAVE(consciousness);
    LOCO_SAVE(postPoweredGrace); LOCO_SAVE(gaitBlendTime); LOCO_SAVE(weightShiftDuration);
    LOCO_SAVE(supportFrequency); LOCO_SAVE(supportMaxAcceleration); LOCO_SAVE(supportBias);
    LOCO_SAVE(takeoffHeight); LOCO_SAVE(takeoffTimeout); LOCO_SAVE(liftFrequency); LOCO_SAVE(liftMaxForce);
    LOCO_SAVE(stepLength); LOCO_SAVE(swingHeight); LOCO_SAVE(swingDuration); LOCO_SAVE(arrivalHeight);
    LOCO_SAVE(arrivalTolerance); LOCO_SAVE(arrivalSettleDuration); LOCO_SAVE(arrivalTimeout);
    LOCO_SAVE(descentDuration); LOCO_SAVE(plantTimeout); LOCO_SAVE(plantAcquireDuration);
    LOCO_SAVE(plantAcquireMaxSpeed); LOCO_SAVE(plantAcquireTimeout); LOCO_SAVE(contactSettleDuration);
    LOCO_SAVE(touchdownMaxVerticalSpeed); LOCO_SAVE(touchdownMaxHorizontalSpeed);
    LOCO_SAVE(touchdownMaxAngularSpeed); LOCO_SAVE(touchdownMaxCommandSpeed);
    LOCO_SAVE(touchdownMinTrajectoryProgress); LOCO_SAVE(touchdownMinNormalY);
    LOCO_SAVE(footTargetTolerance); LOCO_SAVE(safeReachFraction); LOCO_SAVE(transferDuration);
    LOCO_SAVE(transferSupportBias); LOCO_SAVE(transferComTolerance); LOCO_SAVE(transferHoldDuration);
    LOCO_SAVE(transferTimeout); LOCO_SAVE(interStepDuration); LOCO_SAVE(driftGrowthTolerance);
    LOCO_SAVE(gaitDesiredSpeed); LOCO_SAVE(gaitTargetStepPeriod); LOCO_SAVE(gaitCadenceFloorScale);
    LOCO_SAVE(gaitValidationFloorScale);
    LOCO_SAVE(gaitSupportMaxSpeed); LOCO_SAVE(gaitSupportTransportSpeed);
    LOCO_SAVE(gaitAdaptationEnabled); LOCO_SAVE(gaitAdaptationResponse);
    LOCO_SAVE(gaitMaxStrideCorrection); LOCO_SAVE(gaitMaxPeriodSlowdown);
    LOCO_SAVE(gaitStressStopThreshold);
    LOCO_SAVE(gaitPlacementGain); LOCO_SAVE(gaitNominalAdvance); LOCO_SAVE(gaitMinStepLength);
    LOCO_SAVE(gaitMaxStepLength); LOCO_SAVE(gaitReachCrouch); LOCO_SAVE(gaitCrouchTime);
    LOCO_SAVE(gaitUsableReachFraction); LOCO_SAVE(gaitSoleLevelTime); LOCO_SAVE(gaitFootPositionGain);
    LOCO_SAVE(gaitFootVelocityLeadTime); LOCO_SAVE(gaitMaxFootCorrection);
    LOCO_SAVE(gaitSwingSpeedLimit); LOCO_SAVE(gaitSwingAcceleration);
    LOCO_SAVE(gaitSoleAngularSpeedLimit); LOCO_SAVE(gaitSoleAngularAcceleration);
    LOCO_SAVE(gaitLandingLinearDampingTime);
    LOCO_SAVE(gaitInterStepRecenterTime); LOCO_SAVE(gaitUprightStiffness);
    LOCO_SAVE(gaitUprightDamping); LOCO_SAVE(gaitUprightMaxTorque); LOCO_SAVE(gaitHeadingStiffness);
    LOCO_SAVE(gaitHeadingDamping); LOCO_SAVE(gaitHeadingMaxTorque);
    LOCO_SAVE(gaitMaxTurnStepDeg);
    LOCO_SAVE(gaitTurnFullAdvanceDeg); LOCO_SAVE(gaitTurnZeroAdvanceDeg);
    LOCO_SAVE(gaitTurnInsideSwingSpeedLimit); LOCO_SAVE(gaitTurnOutsideSwingSpeedLimit);
    LOCO_SAVE(gaitTurnAngularSpeedLimit); LOCO_SAVE(gaitTurnLinearAcceleration);
    LOCO_SAVE(gaitTurnAngularAcceleration);
    LOCO_SAVE(gaitMaxFootSeparation); LOCO_SAVE(gaitInterStepTiltLimit);
    LOCO_SAVE(gaitInterStepHeadingLimit); LOCO_SAVE(gaitStopTime); LOCO_SAVE(gaitStopHoldTime);
    LOCO_SAVE(maxLegReachFraction); LOCO_SAVE(standingPoseResponse); LOCO_SAVE(hipLimitMarginDeg);
    LOCO_SAVE(poseWeight); LOCO_SAVE(uprightScale); LOCO_SAVE(ikWriteEnabled);
    LOCO_SAVE(leftFootBone); LOCO_SAVE(rightFootBone); LOCO_SAVE(torsoBone); LOCO_SAVE(debug);
#undef LOCO_SAVE
    return j.dump();
}

template<>
inline void DeserializeComponent<LocamotionControllerComponent>(LocamotionControllerComponent& c,
                                                                 const std::string& data)
{
    const auto j = nlohmann::json::parse(data);
#define LOCO_LOAD(name) c.name = j.value(#name, c.name)
#define LOCO_MIGRATE(name, legacy) c.name = j.value(#name, j.value(legacy, c.name))
    LOCO_LOAD(maxSpeed); LOCO_LOAD(deadzone); LOCO_LOAD(minWalkSpeed);
    LOCO_LOAD(facingOffsetDeg); LOCO_LOAD(groundRayLength); LOCO_LOAD(consciousness);
    LOCO_LOAD(postPoweredGrace); LOCO_LOAD(gaitBlendTime);
    LOCO_MIGRATE(weightShiftDuration, "test2ShiftTime");
    LOCO_MIGRATE(supportFrequency, "test2SupportFrequency");
    LOCO_MIGRATE(supportMaxAcceleration, "test2SupportMaxAccel");
    LOCO_MIGRATE(supportBias, "test3SupportFraction");
    LOCO_MIGRATE(takeoffHeight, "test3TakeoffHeight"); LOCO_MIGRATE(takeoffTimeout, "test3TakeoffTime");
    LOCO_MIGRATE(liftFrequency, "test3TakeoffFrequency"); LOCO_MIGRATE(liftMaxForce, "test3TakeoffMaxForce");
    LOCO_MIGRATE(stepLength, "test4StepLength"); LOCO_MIGRATE(swingHeight, "test4SwingHeight");
    LOCO_MIGRATE(swingDuration, "test4SwingTime"); LOCO_MIGRATE(arrivalHeight, "test4ArrivalHeight");
    LOCO_MIGRATE(arrivalTolerance, "test4ArrivalTolerance");
    LOCO_MIGRATE(arrivalSettleDuration, "test4ArrivalSettleTime");
    LOCO_MIGRATE(arrivalTimeout, "test4ArrivalTimeout"); LOCO_MIGRATE(descentDuration, "test4DescentTime");
    LOCO_MIGRATE(plantTimeout, "test4PlantTimeout"); LOCO_MIGRATE(plantAcquireDuration, "test4PlantAcquireTime");
    LOCO_MIGRATE(plantAcquireMaxSpeed, "test4PlantAcquireMaxSpeed");
    LOCO_MIGRATE(plantAcquireTimeout, "test4PlantAcquireTimeout");
    LOCO_MIGRATE(contactSettleDuration, "test4ContactSettleTime");
    LOCO_MIGRATE(touchdownMaxVerticalSpeed, "test4TouchdownMaxVerticalSpeed");
    LOCO_LOAD(touchdownMaxHorizontalSpeed); LOCO_LOAD(touchdownMaxAngularSpeed);
    LOCO_LOAD(touchdownMaxCommandSpeed); LOCO_LOAD(touchdownMinTrajectoryProgress);
    LOCO_MIGRATE(touchdownMinNormalY, "test4TouchdownMinNormalY");
    LOCO_MIGRATE(footTargetTolerance, "test4TargetTolerance");
    LOCO_MIGRATE(safeReachFraction, "test4SafeReachFraction");
    LOCO_MIGRATE(transferDuration, "test5TransferTime"); LOCO_MIGRATE(transferSupportBias, "test5SupportFraction");
    LOCO_MIGRATE(transferComTolerance, "test5ComTolerance"); LOCO_MIGRATE(transferHoldDuration, "test5HoldTime");
    LOCO_MIGRATE(transferTimeout, "test5HoldTimeout"); LOCO_MIGRATE(interStepDuration, "test6InterStepTime");
    LOCO_MIGRATE(driftGrowthTolerance, "test6DriftGrowthTolerance");
    LOCO_MIGRATE(gaitDesiredSpeed, "test7DesiredSpeed"); LOCO_LOAD(gaitTargetStepPeriod);
    LOCO_LOAD(gaitCadenceFloorScale); LOCO_LOAD(gaitValidationFloorScale);
    LOCO_LOAD(gaitSupportMaxSpeed);
    LOCO_LOAD(gaitSupportTransportSpeed);
    LOCO_LOAD(gaitAdaptationEnabled); LOCO_LOAD(gaitAdaptationResponse);
    LOCO_LOAD(gaitMaxStrideCorrection); LOCO_LOAD(gaitMaxPeriodSlowdown);
    LOCO_LOAD(gaitStressStopThreshold);
    LOCO_MIGRATE(gaitPlacementGain, "test7PlacementGain"); LOCO_MIGRATE(gaitNominalAdvance, "test7NominalAdvance");
    LOCO_MIGRATE(gaitMinStepLength, "test7MinStepLength"); LOCO_MIGRATE(gaitMaxStepLength, "test7MaxStepLength");
    LOCO_MIGRATE(gaitReachCrouch, "test7ReachCrouch"); LOCO_MIGRATE(gaitCrouchTime, "test7CrouchTime");
    LOCO_MIGRATE(gaitUsableReachFraction, "test7UsableReachFraction");
    LOCO_MIGRATE(gaitSoleLevelTime, "test7SoleLevelTime"); LOCO_MIGRATE(gaitFootPositionGain, "test7FootPositionGain");
    LOCO_MIGRATE(gaitFootVelocityLeadTime, "test7FootVelocityLeadTime");
    LOCO_MIGRATE(gaitMaxFootCorrection, "test7MaxFootCorrection");
    LOCO_LOAD(gaitSwingSpeedLimit); LOCO_LOAD(gaitSwingAcceleration);
    LOCO_LOAD(gaitSoleAngularSpeedLimit); LOCO_LOAD(gaitSoleAngularAcceleration);
    LOCO_LOAD(gaitLandingLinearDampingTime);
    LOCO_MIGRATE(gaitInterStepRecenterTime, "test7InterStepRecenterTime");
    LOCO_MIGRATE(gaitUprightStiffness, "test7UprightStiffness");
    LOCO_MIGRATE(gaitUprightDamping, "test7UprightDamping");
    LOCO_MIGRATE(gaitUprightMaxTorque, "test7UprightMaxTorque");
    LOCO_MIGRATE(gaitHeadingStiffness, "test7HeadingStiffness");
    LOCO_MIGRATE(gaitHeadingDamping, "test7HeadingDamping");
    LOCO_MIGRATE(gaitHeadingMaxTorque, "test7HeadingMaxTorque");
    LOCO_LOAD(gaitMaxTurnStepDeg);
    LOCO_LOAD(gaitTurnFullAdvanceDeg); LOCO_LOAD(gaitTurnZeroAdvanceDeg);
    LOCO_LOAD(gaitTurnInsideSwingSpeedLimit); LOCO_LOAD(gaitTurnOutsideSwingSpeedLimit);
    LOCO_LOAD(gaitTurnAngularSpeedLimit); LOCO_LOAD(gaitTurnLinearAcceleration);
    LOCO_LOAD(gaitTurnAngularAcceleration);
    LOCO_LOAD(gaitMaxFootSeparation);
    LOCO_MIGRATE(gaitInterStepTiltLimit, "test7InterStepTiltLimit");
    LOCO_MIGRATE(gaitInterStepHeadingLimit, "test7InterStepHeadingLimit");
    LOCO_MIGRATE(gaitStopTime, "test7StopTime"); LOCO_MIGRATE(gaitStopHoldTime, "test7StopHoldTime");
    LOCO_MIGRATE(maxLegReachFraction, "proceduralMaxReach");
    LOCO_MIGRATE(standingPoseResponse, "proceduralPoseResponse");
    LOCO_LOAD(hipLimitMarginDeg); LOCO_LOAD(poseWeight); LOCO_LOAD(uprightScale);
    LOCO_LOAD(ikWriteEnabled); LOCO_LOAD(leftFootBone); LOCO_LOAD(rightFootBone);
    LOCO_LOAD(torsoBone); LOCO_LOAD(debug);
#undef LOCO_MIGRATE
#undef LOCO_LOAD
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
        const float inputX = Input::GetAxis("MoveX");
        const float inputY = -Input::GetAxis("MoveY");
        const float magnitude = glm::min(
            glm::length(glm::vec2(inputX, inputY)), 1.0f);

        glm::vec3 cameraRight, cameraForward;
        CameraRelativeBasis(scene, cameraRight, cameraForward);

        for (auto [entity, comp] : scene.View<Comp>().each()) {
            if (!scene.Has<RagdollComponent>(entity)
                || !scene.Has<TransformComponent>(entity)) continue;
            auto& rag = scene.Get<RagdollComponent>(entity);
            auto& transform = scene.Get<TransformComponent>(entity);

            if (rag.mode == RagdollMode::Animated) {
                Physics::SetRagdollMode(rag, RagdollMode::Powered);
                comp._timeSincePowered = 0.0f;
            }
            if (rag.mode != RagdollMode::Powered) {
                rag.locomotionActive = false;
                rag.locomotionTorqueUpright = false;
                rag.locomotionSimbicon = false;
                rag.locomotionSimbiconBlend = 0.0f;
                rag.locomotionHipTorque[0] =
                    rag.locomotionHipTorque[1] = glm::vec3(0.0f);
                rag.locomotionHipBones[0] = rag.locomotionHipBones[1] = -1;
                rag.locomotionLiftBone = -1;
                rag.locomotionFootLockBones[0] =
                    rag.locomotionFootLockBones[1] = -1;
                rag.locomotionFootLockWeights[0] =
                    rag.locomotionFootLockWeights[1] = 0.0f;
                rag.locomotionHeightOffset = 0.0f;
                comp._gaitEnabled = false;
                ResetGait(comp);
                ResetPhysicalGait(comp);
                continue;
            }

            if (comp._timeSincePowered < 0.0f)
                comp._timeSincePowered = 0.0f;
            comp._timeSincePowered += dt;
            rag.strength = glm::clamp(comp.consciousness, 0.0f, 1.0f);

            const float speed = magnitude > comp.deadzone
                ? (magnitude - comp.deadzone)
                    / (1.0f - comp.deadzone) * comp.maxSpeed
                : 0.0f;
            glm::vec3 moveDirection =
                cameraRight * inputX + cameraForward * inputY;
            moveDirection.y = 0.0f;
            moveDirection = glm::length(moveDirection) > 1e-4f
                ? glm::normalize(moveDirection) : glm::vec3(0.0f);

            const float startSpeed = glm::max(comp.minWalkSpeed, 0.0f);
            const float stopSpeed = startSpeed * 0.5f;
            comp._runtimeWalkIntent = comp._runtimeWalkIntent
                ? speed > stopSpeed : speed > startSpeed;
            const bool wantsToWalk = comp._runtimeWalkIntent;
            comp._runtimeRecoveryCooldown = glm::max(
                comp._runtimeRecoveryCooldown - dt, 0.0f);
            if (wantsToWalk && comp._runtimeRestartBlocked
                && comp._runtimeRestartBlockForwardValid
                && glm::dot(moveDirection, moveDirection) > 1e-8f) {
                glm::vec3 blockedForward = comp._runtimeRestartBlockForward;
                blockedForward.y = 0.0f;
                if (glm::dot(blockedForward, blockedForward) > 1e-8f) {
                    blockedForward = glm::normalize(blockedForward);
                    constexpr float kRestartDirectionChangeDeg = 0.5f;
                    const float directionDot = glm::clamp(
                        glm::dot(moveDirection, blockedForward), -1.0f, 1.0f);
                    if (directionDot < std::cos(glm::radians(
                            kRestartDirectionChangeDeg))) {
                        comp._runtimeRestartBlocked = false;
                        comp._runtimeRestartBlockForwardValid = false;
                        comp._runtimeAutoRetryCount = 0;
                        comp._runtimeRecoveryCooldown = 0.0f;
                        comp._runtimeRecoveryStableTime = 0.0f;
                        if (comp.debug) {
                            spdlog::info(
                                "[LocoRuntime] RESTART_UNBLOCKED "
                                "reason=direction-change delta>{:.1f}deg",
                                kRestartDirectionChangeDeg);
                        }
                    }
                }
            }
            if (!wantsToWalk) {
                if (comp.debug && comp._runtimeRestartBlocked
                    && comp._runtimeRestartBlockForwardValid) {
                    spdlog::info(
                        "[LocoRuntime] RESTART_UNBLOCKED "
                        "reason=input-release");
                }
                comp._runtimeRestartBlocked = false;
                comp._runtimeRestartBlockForwardValid = false;
                comp._runtimeAutoRetryCount = 0;
                comp._runtimeRecoveryCooldown = 0.0f;
                comp._runtimeRecoveryStableTime = 0.0f;
            }

            const Diamond::Locomotion::GaitCommand gaitCommand =
                BuildRuntimeGaitCommand(
                    comp, moveDirection, speed, wantsToWalk);
            comp._gaitEnabled = true;

            comp._grounded = Physics::Raycast(
                transform.position, glm::vec3(0, -1, 0),
                comp.groundRayLength, entity).hit;
            rag.locomotionActive =
                comp._grounded && rag.strength > 0.01f;
            rag.locomotionTargetRot = glm::angleAxis(
                comp._yaw + glm::radians(comp.facingOffsetDeg),
                glm::vec3(0, 1, 0));
            rag.locomotionUprightScale = comp.uprightScale;

            const float tiltDeg = Physics::GetRagdollTiltDeg(rag);
            const bool ready = comp._grounded
                && comp._timeSincePowered >= comp.postPoweredGrace;
            const float blendRate =
                1.0f / glm::max(comp.gaitBlendTime, 0.01f);
            comp._gaitWeight = Approach(
                comp._gaitWeight,
                wantsToWalk && ready ? 1.0f : 0.0f,
                blendRate * dt);
            comp._poseBlend = Approach(
                comp._poseBlend,
                ready && tiltDeg < rag.locomotionFallenTilt ? 1.0f : 0.0f,
                blendRate * dt);
            comp._desiredVelocity =
                moveDirection * speed * comp._gaitWeight;

            // The gait is the sole owner of balance, support transfer, and foot
            // placement. Legacy velocity-drive, virtual hip torque, and foot-lock
            // controllers stay disabled.
            rag.locomotionSimbicon = false;
            rag.locomotionSimbiconBlend = 0.0f;
            rag.locomotionTargetVel = glm::vec3(0.0f);
            rag.locomotionSupportTargetWeight = 0.0f;
            rag.locomotionHeightOffset = 0.0f;
            rag.locomotionHipTorque[0] =
                rag.locomotionHipTorque[1] = glm::vec3(0.0f);
            rag.locomotionHipBones[0] = rag.locomotionHipBones[1] = -1;
            rag.locomotionLiftBone = -1;
            for (int& bone : rag.locomotionDisabledMotorBones) bone = -1;
            rag.locomotionFootLockBones[0] =
                rag.locomotionFootLockBones[1] = -1;
            rag.locomotionFootLockWeights[0] =
                rag.locomotionFootLockWeights[1] = 0.0f;

            rag.locomotionTorqueUpright = true;
            rag.locomotionTorqueUprightStiffness =
                glm::max(comp.gaitUprightStiffness, 0.0f);
            rag.locomotionTorqueUprightDamping =
                glm::max(comp.gaitUprightDamping, 0.0f);
            rag.locomotionTorqueUprightMaxTorque =
                glm::max(comp.gaitUprightMaxTorque, 0.0f);
            rag.locomotionTorqueHeadingStiffness =
                glm::max(comp.gaitHeadingStiffness, 0.0f);
            rag.locomotionTorqueHeadingDamping =
                glm::max(comp.gaitHeadingDamping, 0.0f);
            rag.locomotionTorqueHeadingMaxTorque =
                glm::max(comp.gaitHeadingMaxTorque, 0.0f);

            // The standing pass supplies bind-pose targets for legs not currently
            // owned by the gait. The gait then applies the physical step phases.
            UpdateStandingPose(scene, entity, comp, rag, dt);
            UpdatePhysicalGait(
                scene, entity, comp, rag, ready, tiltDeg, dt, gaitCommand);

            if (comp.debug) {
                _debugTimer += dt;
                if (_debugTimer >= 0.5f) {
                    _debugTimer = 0.0f;
                    spdlog::debug(
                        "[Locomotion] phase={} phaseTime={:.2f}s trajectory={:.2f} "
                        "running={} stop={} steps={} stepPeriod={:.2f}s speed={:.3f}m/s "
                        "headingError={:+.1f}deg tilt={:.1f}deg",
                        comp._physicalStepPhase,
                        comp._physicalStepPhaseTime,
                        comp._physicalStepTrajectoryT,
                        comp._gaitRunning ? "yes" : "no",
                        comp._gaitStopRequested ? "yes" : "no",
                        comp._stepSequenceStepsCompleted,
                        comp._gaitLastStepPeriod,
                        comp._gaitMeasuredSpeed,
                        comp._gaitHeadingErrorDeg,
                        tiltDeg);
                }
            }
        }
    }

private:
    float _debugTimer = 0.0f;

    static const char* TurnConstraintName(Comp::TurnConstraintResult result)
    {
        using Result = Comp::TurnConstraintResult;
        switch (result) {
            case Result::NotEvaluated:      return "not-evaluated";
            case Result::None:              return "none";
            case Result::ControllerYawLimit:return "controller-yaw-limit";
            case Result::TurnPairBudget:    return "turn-pair-budget";
            case Result::TurnInitiationRole:
                return "turn-initiation-role";
            case Result::SwingLinearSpeed:  return "swing-linear-speed";
            case Result::SwingTrackingReserve:
                return "swing-tracking-reserve";
            case Result::SwingAngularSpeed: return "swing-angular-speed";
            case Result::PhysicalGeometry:  return "physical-geometry";
            case Result::SwingReach:        return "swing-reach";
            case Result::StanceReach:       return "stance-reach";
            case Result::SwingHipEnvelope:  return "swing-hip-envelope";
            case Result::StanceHipEnvelope: return "stance-hip-envelope";
            case Result::KneeEnvelope:      return "knee-envelope";
            case Result::AnkleEnvelope:     return "ankle-envelope";
            case Result::CommandClosure:    return "command-closure";
            case Result::FootSeparation:    return "foot-separation";
            case Result::LaneCrossing:      return "lane-crossing";
            case Result::Terrain:           return "terrain";
            case Result::MinimumAdvance:    return "minimum-advance";
        }
        return "unknown";
    }

    static const char* TurnObjectiveName(Comp::TurnStepObjective objective)
    {
        using Objective = Comp::TurnStepObjective;
        switch (objective) {
            case Objective::None:        return "none";
            case Objective::Translation: return "translation";
            case Objective::Angular:     return "angular";
            case Objective::Combined:    return "combined";
        }
        return "unknown";
    }

    static Diamond::Locomotion::GaitCommand BuildRuntimeGaitCommand(
        const Comp& comp, const glm::vec3& moveDirection,
        float inputSpeed, bool wantsToWalk)
    {
        using Diamond::Locomotion::GaitCommand;

        GaitCommand command;
        command.enabled = true;

        command.startRequested = wantsToWalk && !comp._gaitRunning
            && !comp._runtimeRestartBlocked
            && comp._runtimeRecoveryCooldown <= 0.0f
            && (comp._runtimeAutoRetryCount == 0
                || comp._runtimeRecoveryStableTime >= 0.25f);
        command.stopRequested = comp._gaitRunning && !wantsToWalk;
        command.initialSupportSide = comp._runtimeNextSupportSide;
        command.desiredForward = moveDirection;

        // Input magnitude scales the stable low-speed gait. Cadence and stride tuning can
        // raise responsiveness together without jumping directly to maxSpeed.
        const float inputFraction = comp.maxSpeed > 1e-4f
            ? glm::clamp(inputSpeed / comp.maxSpeed, 0.0f, 1.0f) : 0.0f;
        command.desiredSpeed = glm::max(comp.gaitDesiredSpeed, 0.0f)
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
        c._runtimeWalkIntent = false;
        c._runtimeRestartBlocked = false;
        c._runtimeRestartBlockForwardValid = false;
        c._runtimeRestartBlockForward = glm::vec3(0.0f, 0.0f, -1.0f);
        c._runtimeAutoRetryCount = 0;
        c._runtimeRecoveryCooldown = 0.0f;
        c._runtimeRecoveryStableTime = 0.0f;
        c._runtimeNextSupportSide = -1;
        c._desiredVelocity = glm::vec3(0.0f);
        c._legL = {};
        c._legR = {};
    }

#include "LocomotionPhysicalGait.inl"
    static void BlendPose(AnimatorComponent& animator, int bone,
                          const glm::quat& target, float weight)
    {
        animator.pose[bone].rotation = glm::normalize(glm::slerp(
            glm::normalize(animator.pose[bone].rotation),
            glm::normalize(target), glm::clamp(weight, 0.0f, 1.0f)));
    }

    // Supplies the authored standing pose only to limbs not currently owned by the
    // physical gait. Foot placement and swing IK remain single-owner operations.
    void UpdateStandingPose(Scene& scene, entt::entity entity, Comp& comp,
                            RagdollComponent& rag, float dt)
    {
        if (!scene.Has<SkinnedMeshComponent>(entity)
            || !scene.Has<AnimatorComponent>(entity)) return;
        auto& mesh = scene.Get<SkinnedMeshComponent>(entity);
        auto& animator = scene.Get<AnimatorComponent>(entity);
        const auto& skeleton = mesh.skeleton;
        const int count = static_cast<int>(skeleton.bones.size());
        if (count == 0 || static_cast<int>(animator.pose.size()) != count)
            return;

        ResolveLeg(comp._legL, comp.leftFootBone, skeleton, rag);
        ResolveLeg(comp._legR, comp.rightFootBone, skeleton, rag);
        if (!ValidLeg(comp._legL) || !ValidLeg(comp._legR)) return;

        const glm::mat4 entityWorld =
            scene.GetTransformSystem().GetWorldMatrix(entity);
        const glm::quat entityRotation = OrientationOf(entityWorld);
        const glm::quat heading = glm::normalize(rag.locomotionTargetRot);
        const glm::vec3 leftHip = glm::vec3(
            (entityWorld * glm::inverse(
                skeleton.bones[comp._legL.hipIdx].inverseBind))[3]);
        const glm::vec3 rightHip = glm::vec3(
            (entityWorld * glm::inverse(
                skeleton.bones[comp._legR.hipIdx].inverseBind))[3]);
        glm::vec3 right = rightHip - leftHip;
        right.y = 0.0f;
        right = glm::dot(right, right) > 1e-8f
            ? glm::normalize(right) : glm::vec3(1, 0, 0);
        right = heading * right;
        right.y = 0.0f;
        right = glm::dot(right, right) > 1e-8f
            ? glm::normalize(right) : glm::vec3(1, 0, 0);
        const glm::vec3 forward = glm::normalize(
            glm::cross(glm::vec3(0, 1, 0), right));
        comp._right = right;
        comp._fwd = forward;

        if (!comp.ikWriteEnabled) return;
        const float poseWeight = glm::clamp(
            comp._poseBlend * comp.poseWeight, 0.0f, 1.0f);
        const float commandAlpha = 1.0f - std::exp(
            -dt / glm::max(comp.standingPoseResponse, 0.01f));

        auto writeStandingLeg = [&](Leg& leg) {
            const glm::quat hipTarget =
                skeleton.bones[leg.hipIdx].localR;
            const glm::quat kneeTarget =
                skeleton.bones[leg.kneeIdx].localR;
            const glm::quat ankleTarget =
                skeleton.bones[leg.ankleIdx].localR;
            const glm::quat footTarget =
                skeleton.bones[leg.footIdx].localR;
            if (!leg.commandValid) {
                leg.hipCommand = glm::normalize(
                    animator.pose[leg.hipIdx].rotation);
                leg.kneeCommand = glm::normalize(
                    animator.pose[leg.kneeIdx].rotation);
                leg.ankleCommand = glm::normalize(
                    animator.pose[leg.ankleIdx].rotation);
                leg.footCommand = glm::normalize(
                    animator.pose[leg.footIdx].rotation);
                leg.commandValid = true;
            }
            leg.hipCommand = glm::normalize(glm::slerp(
                leg.hipCommand, hipTarget, commandAlpha));
            leg.kneeCommand = glm::normalize(glm::slerp(
                leg.kneeCommand, kneeTarget, commandAlpha));
            leg.ankleCommand = glm::normalize(glm::slerp(
                leg.ankleCommand, ankleTarget, commandAlpha));
            leg.footCommand = glm::normalize(glm::slerp(
                leg.footCommand, footTarget, commandAlpha));
            BlendPose(animator, leg.hipIdx, leg.hipCommand, poseWeight);
            BlendPose(animator, leg.kneeIdx, leg.kneeCommand, poseWeight);
            BlendPose(animator, leg.ankleIdx, leg.ankleCommand, poseWeight);
            BlendPose(animator, leg.footIdx, leg.footCommand, poseWeight);
        };

        // Physical gait phases are ordered through INTER_STEP (11); SUPPORT_READY (8)
        // is deliberately included so standing pose cannot steal the newly center-owned
        // leg before transfer begins.
        const bool gaitOwnsSwing = comp._physicalStepPhase >= 2
            && comp._physicalStepPhase <= 11;
        const bool gaitOwnsBoth = comp._stepSequenceStepIndex >= 2
            && comp._physicalStepPhase >= 1 && comp._physicalStepPhase <= 14;
        const bool gaitOwnsLeft = gaitOwnsBoth
            || (gaitOwnsSwing && comp._physicalStepSupportSide > 0);
        const bool gaitOwnsRight = gaitOwnsBoth
            || (gaitOwnsSwing && comp._physicalStepSupportSide < 0);
        if (!gaitOwnsLeft) writeStandingLeg(comp._legL);
        if (!gaitOwnsRight) writeStandingLeg(comp._legR);

        const int torsoIndex = skeleton.Find(comp.torsoBone);
        if (torsoIndex >= 0) {
            const glm::quat torsoWorld = glm::normalize(
                heading * entityRotation
                * BindModelRot(skeleton, torsoIndex));
            const glm::quat torsoLocal = glm::normalize(
                glm::conjugate(ParentWorldRot(
                    rag, skeleton, animator, entityWorld, torsoIndex))
                * torsoWorld);
            BlendPose(
                animator, torsoIndex, torsoLocal, poseWeight);
        }

    }


};
