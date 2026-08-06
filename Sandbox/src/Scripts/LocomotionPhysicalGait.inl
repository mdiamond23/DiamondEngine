// Included in LocamotionControllerSystem's private section. This file owns the
// physical step state machine, foot planning, and constrained ragdoll leg solve.
    static void ResetPhysicalGait(Comp& c)
    {
        c._gaitTurnPlan = {};
        c._gaitTurnPairPendingInside = false;
        c._gaitTurnPairAdvanceScale = 1.0f;
        c._gaitTurnPairYawScale = 1.0f;
        c._gaitTurnPairYawSign = 0.0f;
        c._gaitTurnExitBlendPending = false;
        c._gaitPhysicalReversalActive = false;
        c._gaitReversalSideLatched = false;
        c._gaitReversalYawSign = 0.0f;
        c._gaitReversalTargetForward = glm::vec3(0.0f, 0.0f, -1.0f);
        c._gaitReversalStepCount = 0;
        c._gaitRetargetReferenceForward = glm::vec3(0.0f, 0.0f, -1.0f);
        c._gaitRetargetReferenceValid = false;
        c._gaitRetargetSequence = 0;
        c._gaitTurnCancellationUnwindActive = false;
        c._gaitTurnCancellationStartRotation = glm::quat(1.0f, 0.0f, 0.0f, 0.0f);
        c._gaitTurnCancellationEndRotation = glm::quat(1.0f, 0.0f, 0.0f, 0.0f);
        c._gaitTurnCancellationStartForward = glm::vec3(0.0f, 0.0f, -1.0f);
        c._gaitTurnCancellationEndForward = glm::vec3(0.0f, 0.0f, -1.0f);
        c._gaitTurnCancellationProgress = 0.0f;
        c._physicalStepBaselineValid = false;
        c._physicalStepContactL = c._physicalStepContactR = false;
        c._physicalStepPrevSwingContact = false;
        c._physicalStepTouchdownAccepted = false;
        c._physicalStepTouchdownContactValid = false;
        c._physicalStepAborted = false;
        c._physicalStepTime = 0.0f;
        c._physicalStepSettleTime = 0.0f;
        c._physicalStepPhase = 0;
        c._physicalStepPhaseTime = 0.0f;
        c._physicalStepSupportSide = 0;
        c._physicalStepComCommand = 0.0f;
        c._physicalStepComLateral = c._physicalStepTargetLateral = 0.0f;
        c._physicalStepAirborneTime = 0.0f;
        c._physicalStepArrivalStableTime = 0.0f;
        c._physicalStepReachLimit = 0.0f;
        c._physicalStepPlantAcquireStableTime = 0.0f;
        c._physicalStepPlantSettledOffsetTime = 0.0f;
        c._physicalStepPlantUnsafeTime = 0.0f;
        c._physicalStepPlantAnchorRebased = false;
        c._physicalStepPlantCenterAnchorActive = false;
        c._physicalStepPlantContactMigrationLogged = false;
        c._physicalStepPlantPivotReleaseLatched = false;
        c._physicalStepPlantPivotStableTime = 0.0f;
        c._physicalStepPlantPivotMaxStableTime = 0.0f;
        c._physicalStepPlantPivotReleaseTriggerTime = 0.0f;
        c._physicalStepPlantPivotReleaseTime = 0.0f;
        c._physicalStepPlantPivotReleaseWeight = 0.0f;
        c._physicalStepPlantCenterBlendTime = 0.0f;
        c._physicalStepPlantAnchorTelemetryTime = 0.0f;
        c._physicalStepPlantAnchorHandoffPhaseTime = -1.0f;
        c._physicalStepPlantPivotContactBlockedTime = 0.0f;
        c._physicalStepPlantPivotSoleBlockedTime = 0.0f;
        c._physicalStepPlantPivotAngularBlockedTime = 0.0f;
        c._physicalStepPlantPivotLinearBlockedTime = 0.0f;
        c._physicalStepPlantContactMigration = 0.0f;
        c._physicalStepPlantAngularSpeed = 0.0f;
        c._physicalStepPlantCenterAnchorStart = glm::vec3(0.0f);
        c._physicalStepPlantCenterAnchorTarget = glm::vec3(0.0f);
        c._physicalStepTrajectoryT = 0.0f;
        c._gaitLandingBrakeReadyTime = 0.0f;
        c._gaitLandingBrakeReleased = false;
        c._gaitSwingCommandSpeed = 0.0f;
        c._gaitSwingCommandAngularSpeed = 0.0f;
        c._gaitSwingCommandTrackingError = 0.0f;
        c._gaitSwingSoleCommandErrorDeg = 0.0f;
        c._gaitAnkleCommandAngularSpeed = 0.0f;
        c._gaitAnkleCommandDeltaDeg = 0.0f;
        c._gaitAnkleParentCompensationDeg = 0.0f;
        c._gaitAnkleParentCompensationAppliedDeg = 0.0f;
        c._gaitAnkleAchievableResidualDeg = 0.0f;
        c._gaitAnkleDiagnosticCooldown = 0.0f;
        c._gaitIkAnkleEnvelopeClampDeg = 0.0f;
        c._gaitIkExactAnkleClampDeg = 0.0f;
        c._gaitIkKneeSwivelRequestedDeg = 0.0f;
        c._gaitIkKneeSwivelAppliedDeg = 0.0f;
        c._gaitIkOrientationPriority = 1.0f;
        c._gaitIkOrientationRelaxedDeg = 0.0f;
        c._gaitIkAnkleSwingUsage = 0.0f;
        c._gaitIkAnkleSwingReserve = 0.0f;
        c._gaitIkAnkleTwistDeg = 0.0f;
        c._gaitIkAnkleTwistMarginDeg = 0.0f;
        c._gaitFkDesiredPosition = glm::vec3(0.0f);
        c._gaitFkCommandPosition = glm::vec3(0.0f);
        c._gaitFkPhysicalPosition = glm::vec3(0.0f);
        c._gaitFkDesiredToCommand = glm::vec3(0.0f);
        c._gaitFkCommandToPhysical = glm::vec3(0.0f);
        c._gaitFkDesiredToPhysical = glm::vec3(0.0f);
        c._gaitFkDesiredToCommandOrientationDeg = 0.0f;
        c._gaitFkCommandToPhysicalOrientationDeg = 0.0f;
        c._gaitFkDesiredToPhysicalOrientationDeg = 0.0f;
        c._gaitFkDiagnosticCooldown = 0.0f;
        c._gaitFkValid = false;
        c._gaitSwingPathLength = 0.0f;
        c._gaitSwingMinimumDuration = 0.0f;
        c._gaitSwingDeadline = 0.0f;
        c._gaitSwingWatchdogProgress = 0.0f;
        c._gaitSwingNoProgressTime = 0.0f;
        c._gaitSwingSoleCommandWorld = glm::quat(
            1.0f, 0.0f, 0.0f, 0.0f);
        c._gaitSwingSoleCommandValid = false;
        c._physicalStepSoleStartMinY = 0.0f;
        c._physicalStepSoleClearance = 0.0f;
        c._physicalStepContactPenetration = 0.0f;
        c._physicalStepSoleReferenceValid = false;
        c._physicalStepClearance = 0.0f;
        c._physicalStepForwardTravel = 0.0f;
        c._physicalStepTargetError = 0.0f;
        c._physicalStepHorizontalTargetError = 0.0f;
        c._physicalStepForwardTargetError = 0.0f;
        c._physicalStepLateralTargetError = 0.0f;
        c._physicalStepVerticalTargetError = 0.0f;
        c._physicalStepTouchdownVy = 0.0f;
        c._physicalStepTouchdownNormalY = 0.0f;
        c._physicalStepPlantCenterTravel = 0.0f;
        c._physicalStepStanceDrift = c._physicalStepPlantDrift = 0.0f;
        c._physicalStepMaxStanceDrift = c._physicalStepMaxPlantDrift = 0.0f;
        c._physicalStepInitialTilt = c._physicalStepPeakTilt = c._physicalStepFinalTilt = 0.0f;
        c._physicalStepMaxMotorRatio = 0.0f;
        c._physicalStepMotorSaturated = false;
        c._physicalStepPlantPoseCaptured = false;
        c._physicalStepFootBaselineL = c._physicalStepFootBaselineR = glm::vec3(0.0f);
        c._physicalStepComBaseline = glm::vec3(0.0f);
        c._physicalStepRight = glm::vec3(1.0f, 0.0f, 0.0f);
        c._physicalStepForward = glm::vec3(0.0f, 0.0f, -1.0f);
        c._physicalStepSupportTarget = glm::vec3(0.0f);
        c._physicalStepSwingStart = glm::vec3(0.0f);
        c._physicalStepArcStart = glm::vec3(0.0f);
        c._physicalStepFoothold = glm::vec3(0.0f);
        c._physicalStepDesiredFoot = glm::vec3(0.0f);
        c._physicalStepTouchdownPlant = glm::vec3(0.0f);
        c._physicalStepTouchdownContactWorld = glm::vec3(0.0f);
        c._physicalStepTouchdownContactLocal = glm::vec3(0.0f);
        c._physicalStepApiVelocity = glm::vec3(0.0f);
        c._physicalStepMeasuredVelocity = glm::vec3(0.0f);
        c._physicalStepPreviousSwingFoot = glm::vec3(0.0f);
        c._physicalStepPreviousSwingFootValid = false;
        c._physicalStepFootUpY = 1.0f;
        c._physicalStepContactPoint = glm::vec3(0.0f);
        c._physicalStepContactLocal = glm::vec3(0.0f);
        c._supportTransferTransferT = 0.0f;
        c._supportTransferHoldStableTime = 0.0f;
        c._supportTransferContactLossTime = 0.0f;
        c._supportTransferComError = 0.0f;
        c._supportTransferComToOldSupport = 0.0f;
        c._supportTransferComToNewSupport = 0.0f;
        c._supportTransferComHorizontalSpeed = 0.0f;
        c._supportTransferTransferStartTarget = glm::vec3(0.0f);
        c._supportTransferTransferEndTarget = glm::vec3(0.0f);
        c._stepSequenceStepIndex = 0;
        c._stepSequenceStepsCompleted = 0;
        c._stepSequenceInterStepStableTime = 0.0f;
        c._stepSequenceInitialTilt = 0.0f;
        for (int i = 0; i < 2; ++i) {
            c._stepSequenceStepForward[i] = 0.0f;
            c._stepSequenceStepMaxDrift[i] = 0.0f;
            c._stepSequenceStepPeakTilt[i] = 0.0f;
            c._stepSequenceStepMotorRatio[i] = 0.0f;
        }
        c._stepSequencePreviousContactsValid = false;
        c._stepSequencePreviousContactL = false;
        c._stepSequencePreviousContactR = false;
        c._stepSequenceContactTransitionsL = 0;
        c._stepSequenceContactTransitionsR = 0;
        c._gaitContactChangeTimeL = 0.0f;
        c._gaitContactChangeTimeR = 0.0f;
        c._gaitRunning = false;
        c._gaitStopRequested = false;
        c._gaitLandingObjectiveStopRequested = false;
        c._gaitRunTime = 0.0f;
        c._gaitStepStartTime = 0.0f;
        c._gaitLastLaunchTime = 0.0f;
        c._gaitLastLaunchPeriod = 0.0f;
        c._gaitStepRecontactPauseTime = 0.0f;
        c._gaitStepMaxSoleErrorDeg = 0.0f;
        c._gaitStepMaxSwingAngularSpeed = 0.0f;
        c._gaitStepSpeculativeContacts = 0;
        c._gaitLastLaunchValid = false;
        c._gaitSpeculativeContactActive = false;
        c._gaitLastStepPeriod = 0.0f;
        c._gaitPreviousStepPeriod = 0.0f;
        c._gaitMeasuredSpeed = 0.0f;
        c._gaitFilteredForwardError = 0.0f;
        c._gaitFilteredLateralError = 0.0f;
        c._gaitFilteredTouchdownSpeed = 0.0f;
        c._gaitFilteredDrift = 0.0f;
        c._gaitFilteredMotorRatio = 0.0f;
        c._gaitFilteredUnloadDeficit = 0.0f;
        c._gaitAdaptiveStrideOffset = 0.0f;
        c._gaitAdaptiveLateralOffset = 0.0f;
        c._gaitAdaptivePeriodOffset = 0.0f;
        c._gaitAdaptiveTransferBiasOffset = 0.0f;
        c._gaitStress = 0.0f;
        c._gaitRecoveryFailureSteps = 0;
        c._gaitAdaptiveStopRequested = false;
        c._gaitPhaseTimeScale = 1.0f;
        const float gaitMinimumAdvance = glm::min(
            c.gaitMinStepLength, c.gaitMaxStepLength);
        const float gaitMaximumAdvance = glm::max(
            c.gaitMinStepLength, c.gaitMaxStepLength);
        const float gaitTrackingReserve = glm::min(glm::max(
            c.footTargetTolerance * 0.5f, 0.010f), 0.025f);
        c._gaitCommandedStepLength = glm::clamp(
            c.gaitNominalAdvance,
            glm::min(gaitMaximumAdvance,
                     gaitMinimumAdvance + gaitTrackingReserve),
            gaitMaximumAdvance);
        c._gaitReachCommandCeiling = gaitMaximumAdvance;
        c._gaitTakeoffContactRecoveryTime = 0.0f;
        c._gaitSwingRecontactTime = 0.0f;
        c._gaitSettledTrackingLoss = 0.0f;
        c._gaitForwardPreShift = 0.0f;
        c._gaitReachClearSteps = 0;
        c._gaitPlannedSupportAdvance = 0.0f;
        c._gaitAchievedSupportAdvance = 0.0f;
        c._gaitLastStepLength = 0.0f;
        c._gaitPreviousStepLength = 0.0f;
        c._gaitLastSupportAdvance = 0.0f;
        c._gaitPreviousSupportAdvance = 0.0f;
        c._gaitStepMaxRelevantDrift = 0.0f;
        c._gaitMaxDrift = 0.0f;
        c._gaitPeakTilt = 0.0f;
        c._gaitMaxMotorRatio = 0.0f;
        c._gaitStopStableTime = 0.0f;
        c._gaitStopFootDriftL = 0.0f;
        c._gaitStopFootDriftR = 0.0f;
        c._gaitStopMaxFootDrift = 0.0f;
        c._gaitStopSettleFootDriftL = 0.0f;
        c._gaitStopSettleFootDriftR = 0.0f;
        c._gaitStopMaxSettleFootDrift = 0.0f;
        c._gaitCrouchBlend = 0.0f;
        c._gaitIkRequestedReach = 0.0f;
        c._gaitIkClampedReach = 0.0f;
        c._gaitIkMaxReach = 0.0f;
        c._gaitIkPhysicalReach = 0.0f;
        c._gaitIkReachShortfall = 0.0f;
        c._gaitIkReachShortfallForward = 0.0f;
        c._gaitIkHipEnvelopeClampDeg = 0.0f;
        c._gaitIkHipCommandLagDeg = 0.0f;
        c._gaitIkKneeCommandLagDeg = 0.0f;
        c._gaitIkKneeBendDeg = 0.0f;
        c._gaitIkHipTravelForward = 0.0f;
        c._gaitIkHipTravelLateral = 0.0f;
        c._gaitIkHipTravelVertical = 0.0f;
        c._gaitFootCorrection = 0.0f;
        c._gaitFootCorrectionForward = 0.0f;
        c._gaitFootTargetSpeed = 0.0f;
        c._gaitSoleLevelBlend = 0.0f;
        c._gaitSoleAngularErrorDeg = 0.0f;
        c._gaitPlantPreviousDrift = 0.0f;
        c._gaitPlantDriftRate = 0.0f;
        c._gaitPlantRecoveryLogged = false;
        c._gaitPlantCorrectionPeakRequested = 0.0f;
        c._gaitPlantCorrectionPeakApplied = 0.0f;
        c._gaitPlantCorrectionSaturated = false;
        c._gaitPlantCorrectionRequested = 0.0f;
        c._gaitPlantCorrectionApplied = 0.0f;
        c._gaitPlantCorrectionAtLimit = false;
        c._gaitInterStepRecenterT = 0.0f;
        c._gaitInterStepCenterError = 0.0f;
        c._gaitRootPitchRate = 0.0f;
        c._gaitRootRollRate = 0.0f;
        c._gaitRootYawRate = 0.0f;
        c._gaitRootTiltRate = 0.0f;
        c._gaitHeadingErrorDeg = 0.0f;
        c._gaitPeakHeadingErrorDeg = 0.0f;
        c._gaitTakeoffContactRecoveryActive = false;
        c._gaitIkPlanHipValid = false;
        c._gaitReachClampedStep = false;
        c._gaitOldSupportDriftAllowanceLogged = false;
        c._gaitCancelMode = 0;
        c._gaitLandingVerificationPending = false;
        c._gaitLandingStableTime = 0.0f;
        c._gaitStopSettleReferenceValid = false;
        c._gaitContinuousCycle = false;
        c._gaitBypassWeightShift = false;
        c._gaitNewSupportLoad = 0.0f;
        c._gaitNewSupportLoadLatched = false;
        c._gaitCycleSupportTarget = glm::vec3(0.0f);
        c._gaitSupportCurveActive = false;
        c._gaitSupportCurveStep = -1;
        c._gaitSupportCurveTime = 0.0f;
        c._gaitSupportCurveDuration = 0.0f;
        c._gaitSupportCurveStart = glm::vec3(0.0f);
        c._gaitSupportCurveEnd = glm::vec3(0.0f);
        c._gaitSupportCurveStartVelocity = glm::vec3(0.0f);
        c._gaitSupportCurveEndVelocity = glm::vec3(0.0f);
        c._gaitSupportCommandVelocity = glm::vec3(0.0f);
        c._legL.planted = c._legR.planted = false;
        c._legL.plantSolveValid = c._legR.plantSolveValid = false;
        c._legL.plantFoot = c._legR.plantFoot = glm::vec3(0.0f);
        c._gaitStartCom = glm::vec3(0.0f);
        c._gaitStepStartCom = glm::vec3(0.0f);
        c._gaitIkPlanHip = glm::vec3(0.0f);
        c._gaitInterStepRecenterStart = glm::vec3(0.0f);
        c._gaitInterStepRecenterTarget = glm::vec3(0.0f);
        c._gaitStopStartTarget = glm::vec3(0.0f);
        c._gaitStopEndTarget = glm::vec3(0.0f);
        c._gaitStopFootTargetL = glm::vec3(0.0f);
        c._gaitStopFootTargetR = glm::vec3(0.0f);
        c._gaitStopSettleFootTargetL = glm::vec3(0.0f);
        c._gaitStopSettleFootTargetR = glm::vec3(0.0f);
        c._gaitHeadingTargetRot = glm::quat(1.0f, 0.0f, 0.0f, 0.0f);
        c._legL.groundReferenceFootRotationValid = false;
        c._legR.groundReferenceFootRotationValid = false;
        c._legL.groundReferenceKneePoleValid = false;
        c._legR.groundReferenceKneePoleValid = false;
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

    // Recover the complete rotation between two vector pairs. Aligning only the
    // upper-leg direction leaves the hip's twist about that segment undefined; the
    // knee hinge can then occupy a different plane and the lower link misses its
    // Cartesian endpoint. The secondary vector fixes that axial degree of freedom.
    static bool RotationMatchingVectorPair(
        const glm::vec3& sourcePrimary,
        const glm::vec3& sourceSecondary,
        const glm::vec3& targetPrimary,
        const glm::vec3& targetSecondary,
        glm::quat& rotation)
    {
        if (glm::dot(sourcePrimary, sourcePrimary) < 1e-8f
            || glm::dot(sourceSecondary, sourceSecondary) < 1e-8f
            || glm::dot(targetPrimary, targetPrimary) < 1e-8f
            || glm::dot(targetSecondary, targetSecondary) < 1e-8f) {
            return false;
        }
        const glm::vec3 sourceAxis = glm::normalize(sourcePrimary);
        const glm::vec3 targetAxis = glm::normalize(targetPrimary);
        glm::vec3 sourcePlane = sourceSecondary
            - sourceAxis * glm::dot(sourceSecondary, sourceAxis);
        glm::vec3 targetPlane = targetSecondary
            - targetAxis * glm::dot(targetSecondary, targetAxis);
        if (glm::dot(sourcePlane, sourcePlane) < 1e-8f
            || glm::dot(targetPlane, targetPlane) < 1e-8f) {
            return false;
        }
        sourcePlane = glm::normalize(sourcePlane);
        targetPlane = glm::normalize(targetPlane);
        const glm::vec3 sourceNormal = glm::normalize(
            glm::cross(sourceAxis, sourcePlane));
        const glm::vec3 targetNormal = glm::normalize(
            glm::cross(targetAxis, targetPlane));
        const glm::mat3 sourceBasis(
            sourceAxis, sourcePlane, sourceNormal);
        const glm::mat3 targetBasis(
            targetAxis, targetPlane, targetNormal);
        rotation = glm::normalize(glm::quat_cast(
            targetBasis * glm::transpose(sourceBasis)));
        return true;
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

    struct EnvelopeMeasurement {
        float swingUsage = 0.0f;
        float swingReserve = 1.0f;
        float twistDeg = 0.0f;
        float twistMarginDeg = 0.0f;
    };

    static EnvelopeMeasurement MeasureEnvelope(
        const Envelope& env, const glm::quat& restLocal,
        const glm::quat& target, float marginDeg)
    {
        const glm::vec3 axis = glm::normalize(env.twistAxis);
        const glm::vec3 reference = std::abs(axis.y) < 0.99f
            ? glm::vec3(0, 1, 0) : glm::vec3(1, 0, 0);
        const glm::vec3 planeAxis = glm::normalize(
            glm::cross(axis, reference));
        const glm::vec3 normalAxis = glm::normalize(
            glm::cross(axis, planeAxis));
        const glm::quat basis = glm::normalize(glm::quat_cast(
            glm::mat3(axis, planeAxis, normalAxis)));

        const float margin = glm::max(marginDeg, 0.0f);
        const float planeRadius = glm::max(std::sin(glm::radians(glm::clamp(
            env.swingPlaneDeg - margin, 1.0f, 179.0f)) * 0.5f), 1e-4f);
        const float normalRadius = glm::max(std::sin(glm::radians(glm::clamp(
            env.swingNormalDeg - margin, 1.0f, 179.0f)) * 0.5f), 1e-4f);
        const float middle = (env.twistMinDeg + env.twistMaxDeg) * 0.5f;
        const float twistMinDeg = glm::min(
            env.twistMinDeg + margin, middle);
        const float twistMaxDeg = glm::max(
            env.twistMaxDeg - margin, middle);

        glm::quat delta = glm::normalize(
            glm::conjugate(restLocal) * target);
        glm::quat inBasis = glm::normalize(
            glm::conjugate(basis) * delta * basis);
        if (inBasis.w < 0.0f) inBasis = -inBasis;
        glm::quat twist(inBasis.w, inBasis.x, 0.0f, 0.0f);
        const float twistLength = glm::length(twist);
        twist = twistLength > 1e-6f
            ? twist / twistLength : glm::quat(1, 0, 0, 0);
        const glm::quat swing = glm::normalize(
            inBasis * glm::conjugate(twist));

        EnvelopeMeasurement result;
        result.swingUsage = std::sqrt(
            (swing.y * swing.y) / (planeRadius * planeRadius)
            + (swing.z * swing.z) / (normalRadius * normalRadius));
        result.swingReserve = 1.0f - result.swingUsage;
        result.twistDeg = glm::degrees(
            2.0f * std::atan2(twist.x, twist.w));
        result.twistMarginDeg = glm::min(
            result.twistDeg - twistMinDeg,
            twistMaxDeg - result.twistDeg);
        return result;
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
                leg.kneeMinDeg = body.hingeMinDeg;
                leg.kneeMaxDeg = body.hingeMaxDeg;
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

    static int FootContactSlot(const RagdollComponent& rag, int footBone)
    {
        for (int i = 0; i < 2; ++i)
            if (rag._locomotionFootBones[i] == footBone) return i;
        return -1;
    }

    static void UpdatePhysicalGait(
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
        constexpr int kSupportReady = 8;
        constexpr int kTransfer = 9;
        constexpr int kHold = 10;
        constexpr int kInterStep = 11;
        constexpr int kComplete = 12;
        constexpr int kAbort = 13;
        constexpr int kStopping = 14;
        constexpr int kReturnStand = 15;
        constexpr bool transferEnabled = true;
        constexpr bool continuousEnabled = true;
        constexpr bool multiStepEnabled = true;
        constexpr bool gameplayCommand = true;

        if (!scene.Has<SkinnedMeshComponent>(entity)
            || !scene.Has<AnimatorComponent>(entity)
            || !ValidLeg(comp._legL) || !ValidLeg(comp._legR)
            || !rag._locomotionCOMValid) return;

        // Capture the inputs from the previous update before any state transition or
        // target generation occurs. A compact transition record at the end of the update
        // makes cadence stalls and command discontinuities visible without requiring a
        // long endurance run.
        const int phaseAtFrameStart = comp._physicalStepPhase;
        const float phaseTimeAtFrameStart = comp._physicalStepPhaseTime;
        const glm::vec3 previousDesiredFoot = comp._physicalStepDesiredFoot;
        const glm::vec3 previousSupportTarget = comp._physicalStepSupportTarget;

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
        // Complete nearly all translation while the leg has the full governed swing
        // window. The final three percent preserves a small converging approach without
        // making the short descent finish a meaningful part of the stride.
        constexpr float kWalkingSwingHorizontalAtArrival = 0.97f;
        // Brake earlier in the trajectory so horizontal and angular motion can settle
        // before the sole begins its final vertical placement.
        constexpr float kWalkingSwingLandingBrakeT = 0.83f;
        // Turning retains the recovered smoothstep trajectory. Unlike fast straight
        // walking, it continues horizontal travel and begins vertical descent at t=.70.
        constexpr float kTurnSwingLandingBrakeT = 0.90f;

        // Cadence is a whole-cycle property. When a target period is configured, allocate
        // it explicitly instead of proportionally scaling a collection of legacy phase
        // durations. Those durations sum to several seconds and their independent floors
        // previously produced a nominal budget larger than the requested gait period.
        //
        // Straight walking uses this launch-to-launch contract:
        //   1.00 = takeoff .16 + governed-swing .44 + arrival .06
        //        + descent .14 + overlapping support/landing reserve .20
        // At a 0.50 s target this is:
        //   .50 = .08 + .22 + .03 + .07 + .10 overlap reserve
        // Plant validation, role swap, and load transfer consume that reserve in
        // parallel. The measured-foot governor may extend a physically lagging swing,
        // but no validation timer is serialized into the nominal launch interval.
        // Safety timeouts remain unscaled.
        const float configuredStepBudget =
            glm::max(comp.weightShiftDuration, 0.01f)
            + glm::max(comp.swingDuration, 0.05f)
            + glm::max(comp.arrivalSettleDuration, 0.0f)
            + glm::max(comp.descentDuration, 0.05f)
            + glm::max(comp.plantAcquireDuration, 0.0f)
            + glm::max(comp.contactSettleDuration, 0.0f)
            + glm::max(comp.transferDuration, 0.05f)
            + glm::max(comp.transferHoldDuration, 0.0f)
            + glm::max(comp.interStepDuration, 0.0f);
        const float effectiveTargetStepPeriod = glm::max(
            comp.gaitTargetStepPeriod + comp._gaitAdaptivePeriodOffset, 0.0f);
        const float cadenceScale = continuousEnabled
            && comp.gaitTargetStepPeriod > 0.0f
            ? glm::clamp(effectiveTargetStepPeriod
                         / glm::max(configuredStepBudget, 0.01f), 0.10f, 1.0f)
            : 1.0f;
        comp._gaitPhaseTimeScale = cadenceScale;
        const float motionFloorScale = glm::clamp(
            comp.gaitCadenceFloorScale, 0.20f, 1.50f);
        const float validationFloorScale = glm::clamp(
            comp.gaitValidationFloorScale, 0.20f, 1.50f);
        auto motionCadenceTime = [&](float configured, float targetFraction,
                                     float floor) {
            const float requested = continuousEnabled
                && comp.gaitTargetStepPeriod > 0.0f
                ? effectiveTargetStepPeriod * targetFraction
                : configured;
            return glm::max(requested, floor * motionFloorScale);
        };
        auto validationCadenceTime = [&](float configured, float targetFraction,
                                         float floor) {
            const float requested = continuousEnabled
                && comp.gaitTargetStepPeriod > 0.0f
                ? effectiveTargetStepPeriod * targetFraction
                : configured;
            return glm::max(requested, floor * validationFloorScale);
        };
        // Cadence is the interval between alternating foot launches, not the sum of
        // every validation state. The commanded foot motion consumes 80% of that
        // interval; contact settling and load ownership use the remaining 20% while
        // the support controller continues moving.
        const float cadenceTakeoffTarget = effectiveTargetStepPeriod * 0.16f;
        const float cadenceTouchdownTarget = effectiveTargetStepPeriod * 0.04f;
        const float cadenceWalkingPlantQuietTime = glm::max(
            effectiveTargetStepPeriod * 0.08f, 0.04f);
        const float cadenceLoadOverlapTime = glm::max(
            effectiveTargetStepPeriod * 0.26f, 0.10f);
        const float cadenceWeightShiftTime = motionCadenceTime(
            comp.weightShiftDuration, 0.10f, 0.20f);
        const float cadenceSwingTime = motionCadenceTime(
            comp.swingDuration, 0.44f, 0.22f);
        const float cadenceArrivalSettleTime = validationCadenceTime(
            comp.arrivalSettleDuration, 0.06f, 0.03f);
        const float cadenceDescentTime = motionCadenceTime(
            comp.descentDuration, 0.14f, 0.14f);
        const float cadencePlantAcquireTime = validationCadenceTime(
            comp.plantAcquireDuration, 0.02f, 0.04f);
        const float cadenceLandingVerifyTime = validationCadenceTime(
            comp.contactSettleDuration, 0.06f, 0.08f);
        const float cadenceTransferTime = motionCadenceTime(
            comp.transferDuration, 0.10f, 0.14f);
        const float cadenceTransferHoldTime = validationCadenceTime(
            comp.transferHoldDuration, 0.10f, 0.06f);
        const float cadenceInterStepTime = validationCadenceTime(
            comp.interStepDuration, 0.02f, 0.04f);
        // The 0.50 s straight-walk cadence intentionally leaves only 70 ms for descent.
        // The recovered 12.5-degree turn path used roughly 120-130 ms here so the
        // physical sole could finish its angular and horizontal convergence. Keep that
        // reserve local to an admitted turn; ordinary walking retains the fast budget.
        constexpr float kTurnCadenceDescentFloor = 0.13f;
        auto yawBearingTurnCadenceActive = [&]() {
            return continuousEnabled && comp._gaitRunning
                && comp._gaitCancelMode == 0
                && comp._gaitTurnPlan.activeHeadingPlan
                && std::abs(comp._gaitTurnPlan.admittedYaw) > 1e-6f;
        };
        auto activeCadenceDescentTime = [&]() {
            return yawBearingTurnCadenceActive()
                ? glm::max(cadenceDescentTime, kTurnCadenceDescentFloor)
                : cadenceDescentTime;
        };
        const float cadencePlantBranchTime = cadenceWalkingPlantQuietTime
            + cadencePlantAcquireTime + cadenceLandingVerifyTime;
        const float cadenceMotionTime = cadenceTakeoffTarget
            + cadenceSwingTime + cadenceArrivalSettleTime
            + cadenceDescentTime;
        const float cadenceOverlapReserve = glm::max(
            effectiveTargetStepPeriod - cadenceMotionTime, 0.0f);
        const float cadenceEquationPeriod = cadenceMotionTime
            + cadenceOverlapReserve;

        bool leftPositionOk = false, rightPositionOk = false;
        const glm::vec3 leftFoot = physicalPosition(comp._legL.footIdx, &leftPositionOk);
        const glm::vec3 rightFoot = physicalPosition(comp._legR.footIdx, &rightPositionOk);
        if (!leftPositionOk || !rightPositionOk) return;

        comp._physicalStepContactL = FootContact(rag, comp._legL.footIdx);
        comp._physicalStepContactR = FootContact(rag, comp._legR.footIdx);
        if (multiStepEnabled && comp._stepSequencePreviousContactsValid
            && comp._physicalStepPhase > kIdle) {
            if (continuousEnabled) {
                // Ignore short-lived sole-manifold chatter when counting gait events.
                // The observed takeoff recovery contacts can persist for 50-180 ms, so
                // require a state to survive beyond that recovery window before counting it.
                constexpr float kContactDebounceTime = 0.20f;
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
                    if (changeTime < kContactDebounceTime) return;
                    debouncedContact = rawContact;
                    changeTime = 0.0f;
                    ++transitionCount;
                };
                updateDebouncedContact(
                    comp._physicalStepContactL, comp._stepSequencePreviousContactL,
                    comp._gaitContactChangeTimeL,
                    comp._stepSequenceContactTransitionsL, "LEFT");
                updateDebouncedContact(
                    comp._physicalStepContactR, comp._stepSequencePreviousContactR,
                    comp._gaitContactChangeTimeR,
                    comp._stepSequenceContactTransitionsR, "RIGHT");
            } else {
                if (comp._physicalStepContactL != comp._stepSequencePreviousContactL)
                    ++comp._stepSequenceContactTransitionsL;
                if (comp._physicalStepContactR != comp._stepSequencePreviousContactR)
                    ++comp._stepSequenceContactTransitionsR;
                comp._stepSequencePreviousContactL = comp._physicalStepContactL;
                comp._stepSequencePreviousContactR = comp._physicalStepContactR;
            }
        }
        if (!ready) {
            comp._physicalStepTime = 0.0f;
            comp._physicalStepSettleTime = 0.0f;
            return;
        }
        comp._physicalStepTime += dt;

        const glm::vec3 leftVelocity = Physics::GetRagdollBoneLinearVelocity(
            rag, comp._legL.footIdx);
        const glm::vec3 rightVelocity = Physics::GetRagdollBoneLinearVelocity(
            rag, comp._legR.footIdx);
        const float horizontalSpeed = glm::length(glm::vec2(
            rag._locomotionRootVel.x, rag._locomotionRootVel.z));
        const bool settledStanding = comp._physicalStepContactL && comp._physicalStepContactR
            && glm::length(leftVelocity) < 0.15f
            && glm::length(rightVelocity) < 0.15f
            && horizontalSpeed < 0.15f
            && tiltDeg < 15.0f;
        if (gameplayCommand && comp._physicalStepPhase == kIdle
            && !comp._gaitRunning
            && !comp._runtimeRestartBlocked
            && comp._runtimeAutoRetryCount > 0) {
            constexpr float kRetryStableTime = 0.25f;
            const float previousStableTime = comp._runtimeRecoveryStableTime;
            comp._runtimeRecoveryStableTime = settledStanding
                ? comp._runtimeRecoveryStableTime + dt : 0.0f;
            if (previousStableTime < kRetryStableTime
                && comp._runtimeRecoveryStableTime >= kRetryStableTime) {
                spdlog::info(
                    "[LocoRuntime] AUTO_RETRY_READY attempt={} stable={:.3f}s "
                    "contact=({},{}) speed=(L={:.3f},R={:.3f},root={:.3f}) "
                    "tilt={:.1f} phase=IDLE action=resume-held-intent",
                    comp._runtimeAutoRetryCount,
                    comp._runtimeRecoveryStableTime,
                    comp._physicalStepContactL ? "L" : "-",
                    comp._physicalStepContactR ? "R" : "-",
                    glm::length(leftVelocity),
                    glm::length(rightVelocity),
                    horizontalSpeed, tiltDeg);
            }
        } else {
            comp._runtimeRecoveryStableTime = 0.0f;
        }

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
        auto resolvedHeadingDelta = [&](glm::vec3 from, glm::vec3 to) {
            const float rawDelta = signedHeadingDelta(from, to);
            if (!comp._gaitReversalSideLatched)
                return rawDelta;

            const float targetDelta = signedHeadingDelta(
                comp._gaitReversalTargetForward, to);
            const bool sameTarget = std::abs(glm::degrees(targetDelta)) < 0.5f;
            const bool directionAmbiguous =
                std::abs(glm::degrees(rawDelta)) >= 179.5f;
            return sameTarget && directionAmbiguous
                ? comp._gaitReversalYawSign * std::abs(rawDelta)
                : rawDelta;
        };
        auto clearPhysicalReversal = [&](const char* reason) {
            if (comp.debug && (comp._gaitPhysicalReversalActive
                               || comp._gaitReversalSideLatched)) {
                spdlog::info(
                    "[LocomotionReversal] event=CLEAR reason={} steps={} "
                    "heading=({:+.3f},{:+.3f})",
                    reason, comp._gaitReversalStepCount,
                    comp._gaitTurnPlan.committedForward.x,
                    comp._gaitTurnPlan.committedForward.z);
            }
            comp._gaitPhysicalReversalActive = false;
            comp._gaitReversalSideLatched = false;
            comp._gaitReversalYawSign = 0.0f;
            comp._gaitReversalStepCount = 0;
        };
        auto updatePhysicalReversalIntent = [&](glm::vec3 from, glm::vec3 to,
                                                int supportSide,
                                                const char* source) {
            from.y = to.y = 0.0f;
            if (glm::dot(from, from) < 1e-8f
                || glm::dot(to, to) < 1e-8f)
                return signedHeadingDelta(from, to);
            from = glm::normalize(from);
            to = glm::normalize(to);

            const bool hadTarget = comp._gaitPhysicalReversalActive
                || comp._gaitReversalSideLatched;
            const bool targetChanged = hadTarget
                && std::abs(glm::degrees(signedHeadingDelta(
                    comp._gaitReversalTargetForward, to))) >= 0.5f;
            if (targetChanged)
                clearPhysicalReversal("live-retarget");

            const float rawDelta = signedHeadingDelta(from, to);
            const float errorDeg = std::abs(glm::degrees(rawDelta));
            constexpr float kPhysicalReversalNoticeDeg = 90.0f;
            const bool largeDirectionChange = errorDeg >= 179.5f
                || errorDeg > kPhysicalReversalNoticeDeg + 0.01f;
            if (!comp._gaitPhysicalReversalActive && largeDirectionChange) {
                comp._gaitPhysicalReversalActive = true;
                comp._gaitReversalTargetForward = to;
                comp._gaitReversalStepCount = 0;
                if (comp.debug) {
                    spdlog::info(
                        "[LocomotionReversal] event=BEGIN source={} "
                        "error={:.3f}deg target=({:+.3f},{:+.3f}) "
                        "route=physical-steps",
                        source, errorDeg, to.x, to.z);
                }
            }

            if (largeDirectionChange && errorDeg >= 179.5f
                && !comp._gaitReversalSideLatched) {
                // supportSide < 0 releases the right (+1) foot; supportSide > 0
                // releases the left (-1) foot. Choosing that sign makes the first
                // available swing the outside foot and removes the exact-180 tie.
                if (supportSide == 0)
                    supportSide = comp._runtimeNextSupportSide;
                const float firstSwingSide = supportSide < 0 ? 1.0f : -1.0f;
                comp._gaitReversalSideLatched = true;
                comp._gaitReversalYawSign = firstSwingSide;
                comp._gaitReversalTargetForward = to;
                if (comp.debug) {
                    spdlog::info(
                        "[LocomotionReversal] event=LATCH source={} "
                        "direction={} firstSwing={} target=({:+.3f},{:+.3f})",
                        source,
                        firstSwingSide > 0.0f ? "CCW" : "CW",
                        firstSwingSide > 0.0f ? "RIGHT" : "LEFT",
                        to.x, to.z);
                }
            }

            const float resolvedDelta = resolvedHeadingDelta(from, to);
            if (comp._gaitPhysicalReversalActive
                && !targetChanged && errorDeg < 0.5f)
                clearPhysicalReversal("heading-converged");
            return resolvedDelta;
        };
        auto gaitPhaseName = [&](int phase) {
            switch (phase) {
                case kIdle:          return "IDLE";
                case kWeightShift:   return "WEIGHT_SHIFT";
                case kTakeoff:       return "TAKEOFF";
                case kSwing:         return "SWING";
                case kArrival:       return "ARRIVAL";
                case kDescent:       return "DESCENT";
                case kTouchdownWait: return "TOUCHDOWN_WAIT";
                case kSettle:        return "SETTLE";
                case kSupportReady:  return "SUPPORT_READY";
                case kTransfer:      return "TRANSFER";
                case kHold:          return "HOLD";
                case kInterStep:     return "INTER_STEP";
                case kComplete:      return "COMPLETE";
                case kAbort:         return "ABORT";
                case kStopping:      return "STOPPING";
                case kReturnStand:   return "RETURN_STAND";
                default:             return "UNKNOWN";
            }
        };
        auto setGaitHeading = [&](glm::vec3 forward) {
            forward.y = 0.0f;
            if (glm::dot(forward, forward) < 1e-8f)
                forward = glm::vec3(0.0f, 0.0f, -1.0f);
            forward = glm::normalize(forward);
            glm::vec3 right = glm::cross(
                forward, glm::vec3(0.0f, 1.0f, 0.0f));
            makeHorizontalBasis(right, forward);
            comp._physicalStepRight = right;
            comp._physicalStepForward = forward;
            const float targetYaw = std::atan2(-forward.x, -forward.z);
            comp._gaitHeadingTargetRot = glm::angleAxis(
                targetYaw, glm::vec3(0.0f, 1.0f, 0.0f));
            rag.locomotionTargetRot = comp._gaitHeadingTargetRot;

            // Standing and the temporary large-angle fallback commit immediately. Live
            // gait turns instead latch their own start/end headings at foothold admission.
            auto& turnPlan = comp._gaitTurnPlan;
            turnPlan.committedForward = forward;
            turnPlan.activeStartForward = forward;
            turnPlan.activeStartRight = right;
            turnPlan.activeMidForward = forward;
            turnPlan.activeMidRight = right;
            turnPlan.activeEndForward = forward;
            turnPlan.activeEndRight = right;
            turnPlan.activeStartRotation = comp._gaitHeadingTargetRot;
            turnPlan.activeEndRotation = comp._gaitHeadingTargetRot;
            turnPlan.activeHeadingPlan = false;
        };
        if (!comp._physicalStepBaselineValid) {
            comp._physicalStepSettleTime = settledStanding
                ? comp._physicalStepSettleTime + dt : 0.0f;
            if (comp._physicalStepSettleTime >= 1.0f) {
                glm::vec3 right = rightFoot - leftFoot;
                right.y = 0.0f;
                if (glm::dot(right, right) < 1e-8f) right = comp._right;
                if (glm::dot(right, right) < 1e-8f) right = glm::vec3(1, 0, 0);
                glm::vec3 forward = comp._fwd;
                makeHorizontalBasis(right, forward);

                comp._physicalStepBaselineValid = true;
                comp._physicalStepFootBaselineL = leftFoot;
                comp._physicalStepFootBaselineR = rightFoot;
                comp._physicalStepComBaseline = rag._locomotionCOM;
                comp._physicalStepRight = glm::normalize(right);
                comp._physicalStepForward = glm::normalize(forward);
                comp._physicalStepSupportTarget = comp._physicalStepComBaseline;
                comp._physicalStepInitialTilt = tiltDeg;
                comp._physicalStepPeakTilt = tiltDeg;
                comp._physicalStepFinalTilt = tiltDeg;

                const float headingYaw = std::atan2(
                    -comp._physicalStepForward.x,
                    -comp._physicalStepForward.z);
                auto& turnPlan = comp._gaitTurnPlan;
                turnPlan.desiredForward = comp._physicalStepForward;
                turnPlan.committedForward = comp._physicalStepForward;
                turnPlan.activeStartForward = comp._physicalStepForward;
                turnPlan.activeStartRight = comp._physicalStepRight;
                turnPlan.activeMidForward = comp._physicalStepForward;
                turnPlan.activeMidRight = comp._physicalStepRight;
                turnPlan.activeEndForward = comp._physicalStepForward;
                turnPlan.activeEndRight = comp._physicalStepRight;
                turnPlan.activeStartRotation = glm::angleAxis(
                    headingYaw, glm::vec3(0.0f, 1.0f, 0.0f));
                turnPlan.activeEndRotation = turnPlan.activeStartRotation;
            }
        }
        if (!comp._physicalStepBaselineValid) return;

        // Desired heading remains live, but an admitted active step is immutable. The
        // next foothold consumes the newest error from the last committed heading.
        glm::vec3 diagnosticDesired = continuousCommand.desiredForward;
        diagnosticDesired.y = 0.0f;
        if (glm::dot(diagnosticDesired, diagnosticDesired) < 1e-8f)
            diagnosticDesired = comp._gaitTurnPlan.committedForward;
        diagnosticDesired = glm::normalize(diagnosticDesired);
        const bool liveDesiredHeading = glm::dot(
            continuousCommand.desiredForward,
            continuousCommand.desiredForward) > 1e-8f;
        if (continuousEnabled && gameplayCommand && liveDesiredHeading
            && comp._gaitRunning && !comp._gaitStopRequested
            && comp._physicalStepPhase < kStopping) {
            int nextAdmissionSupportSide = comp._physicalStepSupportSide;
            if (comp._gaitTurnPlan.activeHeadingPlan)
                nextAdmissionSupportSide = -nextAdmissionSupportSide;
            comp._gaitTurnPlan.desiredHeadingError =
                updatePhysicalReversalIntent(
                    comp._gaitTurnPlan.committedForward,
                    diagnosticDesired,
                    nextAdmissionSupportSide,
                    "live-command");
        } else {
            comp._gaitTurnPlan.desiredHeadingError = resolvedHeadingDelta(
                comp._gaitTurnPlan.committedForward, diagnosticDesired);
        }
        comp._gaitTurnPlan.desiredForward = diagnosticDesired;

        if (continuousEnabled && gameplayCommand && comp._gaitRunning
            && liveDesiredHeading) {
            if (!comp._gaitRetargetReferenceValid) {
                comp._gaitRetargetReferenceForward = diagnosticDesired;
                comp._gaitRetargetReferenceValid = true;
            } else {
                constexpr float kRetargetNoticeDeg = 0.5f;
                const float retargetDelta = signedHeadingDelta(
                    comp._gaitRetargetReferenceForward, diagnosticDesired);
                if (std::abs(glm::degrees(retargetDelta))
                    >= kRetargetNoticeDeg) {
                    const glm::vec3 previousDesired =
                        comp._gaitRetargetReferenceForward;
                    const float previousError = resolvedHeadingDelta(
                        comp._gaitTurnPlan.committedForward, previousDesired);
                    const float newError = resolvedHeadingDelta(
                        comp._gaitTurnPlan.committedForward, diagnosticDesired);
                    comp._gaitRetargetReferenceForward = diagnosticDesired;
                    ++comp._gaitRetargetSequence;
                    if (comp.debug) {
                        spdlog::info(
                            "[LocomotionRetarget] sequence={} event=RETARGET "
                            "phase={} step={} activeCommitted={} route={} "
                            "desired=(({:+.3f},{:+.3f})->({:+.3f},{:+.3f})) "
                            "delta={:+.3f}deg committedError=({:+.3f}->{:+.3f})deg "
                            "activeYaw={:+.3f}deg progress={:.3f}",
                            comp._gaitRetargetSequence,
                            gaitPhaseName(comp._physicalStepPhase),
                            comp._stepSequenceStepIndex,
                            comp._gaitTurnPlan.activeHeadingPlan ? "yes" : "no",
                            (continuousCommand.stopRequested
                             || comp._gaitStopRequested
                             || comp._physicalStepPhase >= kStopping)
                                ? "after-stop"
                                : (comp._gaitTurnPlan.activeHeadingPlan
                                    ? "next-step" : "current-admission"),
                            previousDesired.x, previousDesired.z,
                            diagnosticDesired.x, diagnosticDesired.z,
                            glm::degrees(retargetDelta),
                            glm::degrees(previousError),
                            glm::degrees(newError),
                            glm::degrees(comp._gaitTurnPlan.admittedYaw),
                            comp._gaitTurnPlan.plannedTurnProgress);
                    }
                }
            }
        }

        if (continuousEnabled) {
            if (comp._gaitRunning)
                rag.locomotionTargetRot = comp._gaitHeadingTargetRot;
            const glm::vec3 rootAngularVelocity =
                rag._locomotionRootAngularVelocity;
            comp._gaitRootPitchRate = glm::dot(
                rootAngularVelocity, comp._physicalStepRight);
            comp._gaitRootRollRate = glm::dot(
                rootAngularVelocity, comp._physicalStepForward);
            comp._gaitRootYawRate = rootAngularVelocity.y;
            comp._gaitRootTiltRate = glm::length(glm::vec2(
                rootAngularVelocity.x, rootAngularVelocity.z));
            comp._gaitHeadingErrorDeg = rag._locomotionHeadingErrorDeg;
            if (comp._gaitRunning) {
                comp._gaitPeakHeadingErrorDeg = glm::max(
                    comp._gaitPeakHeadingErrorDeg,
                    std::abs(comp._gaitHeadingErrorDeg));
            }
        }

        if (continuousCommand.stopRequested && continuousEnabled
            && comp._gaitRunning && comp._physicalStepPhase != kStopping
            && comp._physicalStepPhase != kReturnStand
            && !comp._gaitStopRequested) {
            comp._gaitStopRequested = true;
            clearPhysicalReversal("input-release");
            if (gameplayCommand && comp.debug) {
                spdlog::info(
                    "[LocomotionRetarget] sequence={} event=STOP_REQUEST "
                    "cause={} phase={} step={} activeCommitted={} "
                    "activeYaw={:+.3f}deg progress={:.3f} "
                    "committedError={:+.3f}deg",
                    comp._gaitRetargetSequence,
                    "input-release",
                    gaitPhaseName(comp._physicalStepPhase),
                    comp._stepSequenceStepIndex,
                    comp._gaitTurnPlan.activeHeadingPlan ? "yes" : "no",
                    glm::degrees(comp._gaitTurnPlan.admittedYaw),
                    comp._gaitTurnPlan.plannedTurnProgress,
                    glm::degrees(comp._gaitTurnPlan.desiredHeadingError));
            }
        } else if (gameplayCommand && comp._gaitRunning
                   && comp._gaitStopRequested
                   && !continuousCommand.stopRequested
                   && comp._gaitCancelMode == 0
                   && !comp._gaitAdaptiveStopRequested
                   && comp._physicalStepPhase != kStopping
                   && comp._physicalStepPhase != kReturnStand) {
            // The player returned to the active heading before the committed stop.
            // Keep walking instead of carrying a stale stop request into the next step.
            comp._gaitStopRequested = false;
            comp._gaitLandingObjectiveStopRequested = false;
            if (comp.debug) {
                spdlog::info(
                    "[LocomotionRetarget] sequence={} event=STOP_CANCEL "
                    "phase={} step={} action=continue-current-gait",
                    comp._gaitRetargetSequence,
                    gaitPhaseName(comp._physicalStepPhase),
                    comp._stepSequenceStepIndex);
            }
        }

        auto commitRuntimeStandingHeading = [&](glm::vec3 forward) {
            setGaitHeading(forward);
            const float targetYaw = std::atan2(
                -comp._physicalStepForward.x, -comp._physicalStepForward.z);
            // Publish the physical handoff heading to the general standing controller
            // so a completed stop cannot restart from a stale pre-cancellation frame.
            comp._yaw = targetYaw - glm::radians(comp.facingOffsetDeg);
        };

        if (comp._physicalStepPhase == kIdle) {
            const bool startLeftSupport = continuousCommand.startRequested
                && continuousCommand.initialSupportSide < 0;
            const bool startRightSupport = continuousCommand.startRequested
                && continuousCommand.initialSupportSide > 0;
            if (startLeftSupport || startRightSupport) {
                comp._gaitTurnPairPendingInside = false;
                comp._gaitTurnPairAdvanceScale = 1.0f;
                comp._gaitTurnPairYawScale = 1.0f;
                comp._gaitTurnPairYawSign = 0.0f;
                comp._gaitTurnCancellationUnwindActive = false;
                comp._gaitTurnCancellationProgress = 0.0f;
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

                comp._gaitRetargetReferenceForward = desiredForward;
                comp._gaitRetargetReferenceValid = true;
                comp._gaitRetargetSequence = 0;

                const int initialSupportSide = startLeftSupport ? -1 : 1;
                const float resolvedRequestedTurn =
                    updatePhysicalReversalIntent(
                        standingForward, desiredForward,
                        initialSupportSide, "gait-start");
                comp._gaitTurnPlan.desiredHeadingError = resolvedRequestedTurn;

                comp._physicalStepFootBaselineL = leftFoot;
                comp._physicalStepFootBaselineR = rightFoot;
                comp._physicalStepComBaseline = rag._locomotionCOM;
                // Do not snap the heading target to the command. The first admitted
                // footprint will consume at most gaitMaxTurnStepDeg from this physical
                // standing basis.
                setGaitHeading(standingForward);
                comp._physicalStepSupportTarget = comp._physicalStepComBaseline;
                comp._physicalStepSupportSide = initialSupportSide;
                comp._runtimeNextSupportSide = -comp._physicalStepSupportSide;
                comp._physicalStepPhase = kWeightShift;
                comp._physicalStepPhaseTime = 0.0f;
                comp._physicalStepSettleTime = 0.0f;
                comp._physicalStepAirborneTime = 0.0f;
                comp._physicalStepArrivalStableTime = 0.0f;
                comp._physicalStepReachLimit = 0.0f;
                comp._gaitPlannedSupportAdvance = 0.0f;
                comp._gaitAchievedSupportAdvance = 0.0f;
                comp._physicalStepPlantAcquireStableTime = 0.0f;
                comp._physicalStepPlantSettledOffsetTime = 0.0f;
                comp._physicalStepPlantUnsafeTime = 0.0f;
                comp._physicalStepPlantAnchorRebased = false;
                comp._physicalStepPlantCenterAnchorActive = false;
                comp._physicalStepPlantContactMigrationLogged = false;
                comp._physicalStepPlantPivotReleaseLatched = false;
                comp._physicalStepPlantPivotStableTime = 0.0f;
                comp._physicalStepPlantPivotMaxStableTime = 0.0f;
                comp._physicalStepPlantPivotReleaseTriggerTime = 0.0f;
                comp._physicalStepPlantPivotReleaseTime = 0.0f;
                comp._physicalStepPlantPivotReleaseWeight = 0.0f;
                comp._physicalStepPlantCenterBlendTime = 0.0f;
                comp._physicalStepPlantAnchorTelemetryTime = 0.0f;
                comp._physicalStepPlantAnchorHandoffPhaseTime = -1.0f;
                comp._physicalStepPlantPivotContactBlockedTime = 0.0f;
                comp._physicalStepPlantPivotSoleBlockedTime = 0.0f;
                comp._physicalStepPlantPivotAngularBlockedTime = 0.0f;
                comp._physicalStepPlantPivotLinearBlockedTime = 0.0f;
                comp._physicalStepPlantContactMigration = 0.0f;
                comp._physicalStepPlantAngularSpeed = 0.0f;
                comp._physicalStepPlantCenterAnchorStart = glm::vec3(0.0f);
                comp._physicalStepPlantCenterAnchorTarget = glm::vec3(0.0f);
                comp._gaitPlantPreviousDrift = 0.0f;
                comp._gaitPlantDriftRate = 0.0f;
                comp._gaitPlantRecoveryLogged = false;
                comp._gaitPlantCorrectionPeakRequested = 0.0f;
                comp._gaitPlantCorrectionPeakApplied = 0.0f;
                comp._gaitPlantCorrectionSaturated = false;
                comp._gaitPlantCorrectionRequested = 0.0f;
                comp._gaitPlantCorrectionApplied = 0.0f;
                comp._gaitPlantCorrectionAtLimit = false;
                comp._physicalStepTrajectoryT = 0.0f;
                comp._physicalStepSoleStartMinY = 0.0f;
                comp._physicalStepSoleClearance = 0.0f;
                comp._physicalStepContactPenetration = 0.0f;
                comp._physicalStepSoleReferenceValid = false;
                comp._physicalStepTouchdownAccepted = false;
                comp._physicalStepTouchdownContactValid = false;
                comp._physicalStepPlantCenterTravel = 0.0f;
                comp._physicalStepAborted = false;
                comp._physicalStepMaxStanceDrift = 0.0f;
                comp._physicalStepMaxPlantDrift = 0.0f;
                comp._physicalStepInitialTilt = tiltDeg;
                comp._physicalStepPeakTilt = tiltDeg;
                comp._physicalStepFinalTilt = tiltDeg;
                comp._physicalStepMaxMotorRatio = 0.0f;
                comp._physicalStepMotorSaturated = false;
                comp._physicalStepPlantPoseCaptured = false;
                comp._physicalStepPreviousSwingFootValid = false;
                comp._supportTransferTransferT = 0.0f;
                comp._supportTransferHoldStableTime = 0.0f;
                comp._supportTransferContactLossTime = 0.0f;
                comp._supportTransferComError = 0.0f;
                comp._supportTransferComToOldSupport = 0.0f;
                comp._supportTransferComToNewSupport = 0.0f;
                comp._supportTransferComHorizontalSpeed = 0.0f;
                comp._supportTransferTransferStartTarget = comp._physicalStepComBaseline;
                comp._supportTransferTransferEndTarget = comp._physicalStepComBaseline;
                    comp._stepSequenceStepIndex = 1;
                comp._stepSequenceStepsCompleted = 0;
                comp._stepSequenceInterStepStableTime = 0.0f;
                comp._stepSequenceInitialTilt = tiltDeg;
                for (int i = 0; i < 2; ++i) {
                    comp._stepSequenceStepForward[i] = 0.0f;
                    comp._stepSequenceStepMaxDrift[i] = 0.0f;
                    comp._stepSequenceStepPeakTilt[i] = 0.0f;
                    comp._stepSequenceStepMotorRatio[i] = 0.0f;
                }
                comp._stepSequencePreviousContactsValid = multiStepEnabled;
                comp._stepSequencePreviousContactL = comp._physicalStepContactL;
                comp._stepSequencePreviousContactR = comp._physicalStepContactR;
                comp._stepSequenceContactTransitionsL = 0;
                comp._stepSequenceContactTransitionsR = 0;
                comp._gaitContactChangeTimeL = 0.0f;
                comp._gaitContactChangeTimeR = 0.0f;
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
                    comp._gaitRunning = true;
                    comp._runtimeRecoveryStableTime = 0.0f;
                    comp._gaitStopRequested = false;
                    comp._gaitLandingObjectiveStopRequested = false;
                    comp._gaitRecoveryFailureSteps = 0;
                    comp._gaitRunTime = 0.0f;
                    comp._gaitStepStartTime = 0.0f;
                    comp._gaitLastLaunchTime = 0.0f;
                    comp._gaitLastLaunchPeriod = 0.0f;
                    comp._gaitStepRecontactPauseTime = 0.0f;
                    comp._gaitStepMaxSoleErrorDeg = 0.0f;
                    comp._gaitStepMaxSwingAngularSpeed = 0.0f;
                    comp._gaitStepSpeculativeContacts = 0;
                    comp._gaitLastLaunchValid = false;
                    comp._gaitSpeculativeContactActive = false;
                    comp._gaitLastStepPeriod = 0.0f;
                    comp._gaitPreviousStepPeriod = 0.0f;
                    comp._gaitMeasuredSpeed = 0.0f;
                    spdlog::info(
                        "[LocomotionCadence] PLAN target={:.3f}s equation={:.3f}s "
                        "motion=(takeoff={:.3f}+swing={:.3f}+arrival={:.3f}+"
                        "descent={:.3f}) overlapReserve={:.3f} "
                        "parallelSettle=(touchdown={:.3f},plant={:.3f}+"
                        "roleSwap={:.3f},load={:.3f})",
                        effectiveTargetStepPeriod,
                        cadenceEquationPeriod,
                        cadenceTakeoffTarget,
                        cadenceSwingTime,
                        cadenceArrivalSettleTime,
                        cadenceDescentTime,
                        cadenceOverlapReserve,
                        cadenceTouchdownTarget,
                        cadencePlantBranchTime,
                        cadenceInterStepTime,
                        cadenceLoadOverlapTime);
                    const float minimumAdvance = glm::min(
                        comp.gaitMinStepLength, comp.gaitMaxStepLength);
                    const float maximumAdvance = glm::max(
                        comp.gaitMinStepLength, comp.gaitMaxStepLength);
                    const float trackingReserve = glm::min(glm::max(
                        comp.footTargetTolerance * 0.5f, 0.010f), 0.025f);
                    comp._gaitCommandedStepLength = glm::clamp(
                        comp.gaitNominalAdvance + comp._gaitAdaptiveStrideOffset,
                        glm::min(maximumAdvance,
                                 minimumAdvance + trackingReserve),
                        maximumAdvance);
                    comp._gaitReachCommandCeiling = maximumAdvance;
                    comp._gaitTakeoffContactRecoveryTime = 0.0f;
                    comp._gaitSwingRecontactTime = 0.0f;
                    comp._gaitSettledTrackingLoss = 0.0f;
                    comp._gaitForwardPreShift = 0.0f;
                    comp._gaitReachClearSteps = 0;
                    comp._gaitPlannedSupportAdvance = 0.0f;
                    comp._gaitAchievedSupportAdvance = 0.0f;
                    comp._gaitLastStepLength = 0.0f;
                    comp._gaitPreviousStepLength = 0.0f;
                    comp._gaitLastSupportAdvance = 0.0f;
                    comp._gaitPreviousSupportAdvance = 0.0f;
                    comp._gaitStepMaxRelevantDrift = 0.0f;
                    comp._gaitMaxDrift = 0.0f;
                    comp._gaitPeakTilt = tiltDeg;
                    comp._gaitHeadingErrorDeg = rag._locomotionHeadingErrorDeg;
                    comp._gaitPeakHeadingErrorDeg = std::abs(
                        comp._gaitHeadingErrorDeg);
                    comp._gaitMaxMotorRatio = 0.0f;
                    comp._gaitStopStableTime = 0.0f;
                    comp._gaitStopFootDriftL = 0.0f;
                    comp._gaitStopFootDriftR = 0.0f;
                    comp._gaitStopMaxFootDrift = 0.0f;
                    comp._gaitStopSettleFootDriftL = 0.0f;
                    comp._gaitStopSettleFootDriftR = 0.0f;
                    comp._gaitStopMaxSettleFootDrift = 0.0f;
                    comp._gaitStopSettleReferenceValid = false;
                    comp._gaitCrouchBlend = 0.0f;
                    comp._gaitFootCorrection = 0.0f;
                    comp._gaitFootCorrectionForward = 0.0f;
                    comp._gaitFootTargetSpeed = 0.0f;
                    comp._gaitSoleLevelBlend = 0.0f;
                    comp._gaitPlantPreviousDrift = 0.0f;
                    comp._gaitPlantDriftRate = 0.0f;
                    comp._gaitPlantRecoveryLogged = false;
                    comp._gaitIkPlanHipValid = false;
                    comp._gaitTakeoffContactRecoveryActive = false;
                    comp._gaitReachClampedStep = false;
                    comp._gaitOldSupportDriftAllowanceLogged = false;
                    comp._gaitCancelMode = 0;
                    comp._gaitLandingVerificationPending = false;
                    comp._gaitLandingStableTime = 0.0f;
                    comp._gaitContinuousCycle = false;
                    comp._gaitBypassWeightShift = false;
                    comp._gaitNewSupportLoad = 0.0f;
                    comp._gaitNewSupportLoadLatched = false;
                    comp._gaitCycleSupportTarget = comp._physicalStepSupportTarget;
                    comp._gaitSupportCurveActive = false;
                    comp._gaitSupportCurveStep = -1;
                    comp._gaitSupportCurveTime = 0.0f;
                    comp._gaitSupportCurveDuration = 0.0f;
                    comp._gaitSupportCurveStart = comp._physicalStepSupportTarget;
                    comp._gaitSupportCurveEnd = comp._physicalStepSupportTarget;
                    comp._gaitSupportCurveStartVelocity = glm::vec3(0.0f);
                    comp._gaitSupportCurveEndVelocity = glm::vec3(0.0f);
                    comp._gaitSupportCommandVelocity = glm::vec3(0.0f);
                    comp._legL.planted = true;
                    comp._legR.planted = true;
                    comp._legL.plantSolveValid = false;
                    comp._legR.plantSolveValid = false;
                    comp._legL.plantFoot = leftFoot;
                    comp._legR.plantFoot = rightFoot;
                    comp._gaitStartCom = rag._locomotionCOM;
                    comp._gaitStepStartCom = rag._locomotionCOM;
                }
            }
        }

        if (comp._physicalStepPhase > 0) comp._physicalStepPhaseTime += dt;
        if (continuousEnabled && comp._gaitRunning)
            comp._gaitRunTime += dt;

        if (continuousEnabled && comp._gaitRunning
            && comp._gaitTurnCancellationUnwindActive) {
            constexpr float kCancellationSwingHeadingShare = 0.80f;
            float cancellationProgress =
                comp._gaitTurnCancellationProgress;
            if (comp._physicalStepPhase == kTakeoff) {
                cancellationProgress = 0.0f;
            } else if (comp._physicalStepPhase == kSwing) {
                const float cancellationSwingTime = glm::max(
                    0.18f, cadenceSwingTime * 0.65f);
                const float swingT = glm::clamp(
                    comp._physicalStepPhaseTime
                        / cancellationSwingTime,
                    0.0f, 1.0f);
                cancellationProgress = kCancellationSwingHeadingShare
                    * smoothstep(swingT);
            } else if (comp._physicalStepPhase == kArrival) {
                cancellationProgress = kCancellationSwingHeadingShare;
            } else if (comp._physicalStepPhase == kDescent) {
                const float descentT = glm::clamp(
                    comp._physicalStepPhaseTime
                        / glm::max(activeCadenceDescentTime(), 0.01f),
                    0.0f, 1.0f);
                cancellationProgress = kCancellationSwingHeadingShare
                    + (1.0f - kCancellationSwingHeadingShare)
                        * smoothstep(descentT);
            } else if (comp._physicalStepPhase >= kTouchdownWait) {
                cancellationProgress = 1.0f;
            }

            comp._gaitTurnCancellationProgress = glm::clamp(
                cancellationProgress, 0.0f, 1.0f);
            glm::quat cancellationEnd = glm::normalize(
                comp._gaitTurnCancellationEndRotation);
            const glm::quat cancellationStart = glm::normalize(
                comp._gaitTurnCancellationStartRotation);
            if (glm::dot(cancellationStart, cancellationEnd) < 0.0f)
                cancellationEnd = -cancellationEnd;
            comp._gaitHeadingTargetRot = glm::normalize(glm::slerp(
                cancellationStart, cancellationEnd,
                comp._gaitTurnCancellationProgress));
            rag.locomotionTargetRot = comp._gaitHeadingTargetRot;
        } else if (continuousEnabled && comp._gaitRunning
            && comp._gaitTurnPlan.activeHeadingPlan) {
            auto& turnPlan = comp._gaitTurnPlan;
            constexpr float kSwingArrivalT = 0.70f;
            // Unhandled recovery/stop phases retain the last commanded orientation.
            // This avoids snapping back to the step-entry heading if an active turn is
            // cancelled before the dedicated stop/retarget slice is implemented.
            float turnProgress = turnPlan.plannedTurnProgress;
            if (comp._physicalStepPhase == kWeightShift
                || comp._physicalStepPhase == kTakeoff) {
                turnProgress = 0.0f;
            } else if (comp._physicalStepPhase == kSwing) {
                // The measured foot governor may pause its trajectory when the physical
                // sole leaves the tracking tube. Advance body heading from that same
                // measured-state clock so the pelvis cannot rotate away from a lagging
                // turn foot.
                const float trajectoryT = glm::clamp(
                    comp._physicalStepTrajectoryT, 0.0f, kSwingArrivalT);
                turnProgress = 0.80f * smoothstep(trajectoryT);
            } else if (comp._physicalStepPhase == kArrival) {
                turnProgress = 0.80f * smoothstep(glm::clamp(
                    comp._physicalStepTrajectoryT, 0.0f, kSwingArrivalT));
            } else if (comp._physicalStepPhase == kDescent) {
                const float trajectoryT = glm::clamp(
                    comp._physicalStepTrajectoryT, kSwingArrivalT, 1.0f);
                turnProgress = 0.80f * smoothstep(trajectoryT);
            } else if (comp._physicalStepPhase == kTouchdownWait) {
                turnProgress = 0.80f;
            } else if (comp._physicalStepPhase == kSettle
                       || comp._physicalStepPhase == kSupportReady) {
                // Impact acceptance is not support ownership. Hold the body at its
                // touchdown heading while the planted sole proves and captures a quiet
                // center anchor; turning the pelvis here destabilizes that proof.
                turnProgress = 0.80f;
            } else if (comp._physicalStepPhase == kTransfer) {
                const float transferT = glm::clamp(
                    comp._physicalStepPhaseTime
                        / glm::max(cadenceTransferTime, 0.04f),
                    0.0f, 1.0f);
                turnProgress = 0.80f + 0.20f * smoothstep(transferT);
            } else if (comp._physicalStepPhase == kHold
                       || comp._physicalStepPhase == kInterStep) {
                turnProgress = 1.0f;
            }

            turnPlan.plannedTurnProgress = glm::clamp(
                turnProgress, 0.0f, 1.0f);
            glm::quat endRotation = glm::normalize(
                turnPlan.activeEndRotation);
            const glm::quat startRotation = glm::normalize(
                turnPlan.activeStartRotation);
            if (glm::dot(startRotation, endRotation) < 0.0f)
                endRotation = -endRotation;
            comp._gaitHeadingTargetRot = glm::normalize(glm::slerp(
                startRotation, endRotation,
                turnPlan.plannedTurnProgress));
            rag.locomotionTargetRot = comp._gaitHeadingTargetRot;

            if (std::abs(turnPlan.admittedYaw) > 1e-6f) {
                const float commandedYaw = turnPlan.admittedYaw
                    * turnPlan.plannedTurnProgress;
                turnPlan.achievedYaw = commandedYaw
                    - glm::radians(comp._gaitHeadingErrorDeg);
                turnPlan.achievedTurnProgress =
                    turnPlan.achievedYaw / turnPlan.admittedYaw;
            } else {
                turnPlan.achievedYaw = 0.0f;
                turnPlan.achievedTurnProgress = 0.0f;
            }
        }

        // A turning footprint uses an oblique pair: midpoint forward supplies its
        // translation coefficient while end right supplies its lane coefficient. Keep
        // that immutable objective frame separate from _physicalStepForward, which remains
        // the step-entry measurement frame until transfer commits. The zero-yaw exit
        // blend retains this conditioned geometry, while its dynamics use ordinary gait.
        auto hasTurnConditionedStep = [&]() {
            return continuousEnabled && comp._gaitRunning
                && comp._gaitCancelMode == 0
                && comp._gaitTurnPlan.activeHeadingPlan
                && (std::abs(comp._gaitTurnPlan.admittedYaw) > 1e-6f
                    || comp._gaitTurnPlan.turnExitBlendApplied);
        };
        auto hasYawBearingTurnStep = [&]() {
            return hasTurnConditionedStep()
                && std::abs(comp._gaitTurnPlan.admittedYaw) > 1e-6f;
        };
        auto stepObjectiveForward = [&]() {
            return hasTurnConditionedStep()
                ? comp._gaitTurnPlan.activeMidForward
                : comp._physicalStepForward;
        };
        auto stepObjectiveRight = [&]() {
            return hasTurnConditionedStep()
                ? comp._gaitTurnPlan.activeMidRight
                : comp._physicalStepRight;
        };
        auto stepObjectiveAdvance = [&](glm::vec3 delta) {
            delta.y = 0.0f;
            if (!hasTurnConditionedStep())
                return glm::dot(delta, comp._physicalStepForward);
            const auto& turnPlan = comp._gaitTurnPlan;
            const float denominator = glm::max(glm::dot(
                turnPlan.activeMidForward,
                turnPlan.activeEndForward), 1e-4f);
            return glm::dot(delta, turnPlan.activeEndForward)
                / denominator;
        };
        auto stepObjectiveLateral = [&](glm::vec3 delta) {
            delta.y = 0.0f;
            if (!hasTurnConditionedStep())
                return glm::dot(delta, comp._physicalStepRight);
            const auto& turnPlan = comp._gaitTurnPlan;
            const float denominator = glm::max(glm::dot(
                turnPlan.activeMidRight,
                turnPlan.activeEndRight), 1e-4f);
            return glm::dot(delta, turnPlan.activeMidRight)
                / denominator;
        };
        auto turnAdvanceScaleForHeadingError = [&](float headingError) {
            const float fullAdvanceDeg = glm::clamp(
                comp.gaitTurnFullAdvanceDeg, 0.0f, 90.0f);
            const float zeroAdvanceDeg = glm::max(
                glm::clamp(comp.gaitTurnZeroAdvanceDeg, 0.0f, 180.0f),
                fullAdvanceDeg + 0.01f);
            const float errorDeg = std::abs(glm::degrees(headingError));
            const float blend = glm::clamp(
                (errorDeg - fullAdvanceDeg)
                    / (zeroAdvanceDeg - fullAdvanceDeg),
                0.0f, 1.0f);
            return 1.0f - smoothstep(blend);
        };
        const bool walkingOverlapHandoff = continuousEnabled
            && !hasYawBearingTurnStep();
        const bool straightStartupSupport = walkingOverlapHandoff
            && comp._stepSequenceStepIndex <= 1;
        if (continuousEnabled) {
            // Keep a small crouch for the whole walking sequence. Planning a grounded
            // foothold at nearly full anatomical extension amplified an 8 mm radial motor
            // lag into a 3-4 cm horizontal landing miss. The existing dynamic-root height
            // spring supplies this offset without teleporting any body. Ramp it out during
            // STOPPING / RETURN_STAND (and after an abort) instead of changing height at a
            // phase boundary.
            const bool crouchRequested = comp._gaitRunning
                && comp._physicalStepPhase >= kWeightShift
                && comp._physicalStepPhase <= kInterStep;
            comp._gaitCrouchBlend = Approach(
                comp._gaitCrouchBlend, crouchRequested ? 1.0f : 0.0f,
                dt / glm::max(comp.gaitCrouchTime, 0.10f));
            rag.locomotionHeightOffset = -glm::max(
                comp.gaitReachCrouch, 0.0f) * smoothstep(comp._gaitCrouchBlend);

        }
        const float desiredComCommand = comp._physicalStepPhase >= kWeightShift
            && comp._physicalStepPhase <= kComplete
            ? static_cast<float>(comp._physicalStepSupportSide) : 0.0f;
        comp._physicalStepComCommand = Approach(
            comp._physicalStepComCommand, desiredComCommand,
            dt / cadenceWeightShiftTime);

        const float fraction = glm::clamp(comp.supportBias, 0.0f, 1.0f);
        const float leftAvailable = glm::dot(
            comp._physicalStepFootBaselineL - comp._physicalStepComBaseline, comp._physicalStepRight);
        const float rightAvailable = glm::dot(
            comp._physicalStepFootBaselineR - comp._physicalStepComBaseline, comp._physicalStepRight);
        const float lateralOffset = comp._physicalStepComCommand < 0.0f
            ? -comp._physicalStepComCommand * glm::min(leftAvailable, 0.0f) * fraction
            :  comp._physicalStepComCommand * glm::max(rightAvailable, 0.0f) * fraction;
        glm::vec3 supportTarget = comp._physicalStepComBaseline
                                + comp._physicalStepRight * lateralOffset;
        glm::vec3 supportVelocity(0.0f);
        bool supportVelocityExplicit = false;
        float gaitStanceForwardTarget = 0.0f;
        if (continuousEnabled && comp._physicalStepPhase >= kWeightShift
            && comp._physicalStepPhase <= kSettle) {
            // TRANSFER leaves the COM at the forward midpoint of the two planted feet.
            // Before the next single-support phase, deliberately move it toward the new
            // stance foot in both axes. The old controller shifted only laterally, leaving
            // the hip progressively farther behind every new foothold.
            const glm::vec3 stanceSupport = comp._physicalStepSupportSide < 0
                ? leftFoot : rightFoot;
            constexpr float kForwardStanceFraction = 0.90f;
            gaitStanceForwardTarget = glm::dot(
                stanceSupport - comp._physicalStepComBaseline,
                comp._physicalStepForward) * kForwardStanceFraction;
            supportTarget += comp._physicalStepForward * gaitStanceForwardTarget;
        }
        if (continuousEnabled && comp._gaitContinuousCycle
            && comp._physicalStepPhase >= kWeightShift
            && comp._physicalStepPhase <= kSettle) {
            // The completed transfer already placed the COM over the new support. Keep
            // that world-space target through the next swing instead of rebuilding a
            // baseline and visibly shifting the body a second time.
            supportTarget = comp._gaitCycleSupportTarget;
        }
        const bool awaitingSupportOwnership = continuousEnabled
            && !walkingOverlapHandoff
            && (comp._physicalStepPhase == kSettle
                || comp._physicalStepPhase == kSupportReady)
            && comp._physicalStepTouchdownAccepted;
        if (awaitingSupportOwnership) {
            // Touchdown accepts a collision; it does not authorize load transfer. Hold
            // the exact command present at impact until the material-point pivot has
            // handed ownership to a quiet sole-center anchor and that anchor has proved
            // stable in SUPPORT_READY.
            supportTarget = comp._supportTransferTransferStartTarget;
            supportVelocity = glm::vec3(0.0f);
            supportVelocityExplicit = true;
        }
        if (transferEnabled && comp._physicalStepPhase >= kTransfer
            && comp._physicalStepPhase <= kComplete) {
            const float transferT = comp._physicalStepPhase == kTransfer
                ? glm::clamp(comp._physicalStepPhaseTime
                    / cadenceTransferTime, 0.0f, 1.0f)
                : 1.0f;
            comp._supportTransferTransferT = transferT;
            // Unlike the old walking path, transfer is phase-continuous and supplies the
            // support controller with the target velocity. This avoids an instantaneous
            // forward pull when the newly planted foot becomes authoritative.
            supportTarget = glm::mix(comp._supportTransferTransferStartTarget,
                                     comp._supportTransferTransferEndTarget,
                                     smoothstep(transferT));
        }
        if (continuousEnabled && comp._physicalStepPhase == kInterStep) {
            // Both endpoints were captured once at phase entry. Never rebuild the target
            // from vibrating physical feet: differentiating that moving midpoint turned
            // contact noise into a commanded COM velocity and fed it back into the body.
            const float duration = cadenceInterStepTime;
            const float recenterT = glm::clamp(
                comp._physicalStepPhaseTime / duration, 0.0f, 1.0f);
            const float recenterBlend = smoothstep(recenterT);
            supportTarget = glm::mix(
                comp._gaitInterStepRecenterStart,
                comp._gaitInterStepRecenterTarget,
                recenterBlend);
            const float blendRate = recenterT > 0.0f && recenterT < 1.0f
                ? 6.0f * recenterT * (1.0f - recenterT) / duration
                : 0.0f;
            supportVelocity = (comp._gaitInterStepRecenterTarget
                             - comp._gaitInterStepRecenterStart) * blendRate;
            supportVelocityExplicit = true;
            comp._gaitInterStepRecenterT = recenterT;
            comp._gaitInterStepCenterError = glm::length(glm::vec2(
                rag._locomotionCOM.x - comp._gaitInterStepRecenterTarget.x,
                rag._locomotionCOM.z - comp._gaitInterStepRecenterTarget.z));
        } else if (continuousEnabled) {
            comp._gaitInterStepRecenterT = 0.0f;
            comp._gaitInterStepCenterError = 0.0f;
        }
        // Begin a small anticipatory preload before the foot reaches the floor. The old path
        // used a touchdown smoothstep followed by a second transfer smoothstep; both
        // segments imposed zero velocity at their boundary and produced the visible
        // stop-then-burst. Pre-contact motion is capped at 20% so the old support retains
        // clear ownership during the landing brake. Credible, velocity-gated contact then
        // advances the same C1 curve while plant validation runs in parallel.
        const float configuredSupportMaxSpeed = glm::clamp(
            walkingOverlapHandoff
                ? glm::max(comp.gaitSupportMaxSpeed, 0.40f)
                : glm::min(comp.gaitSupportMaxSpeed, 0.25f),
            0.01f, 1.0f);
        // Command enough geometric reserve that ordinary tracking error and a few
        // centimetres of live foot motion cannot leave the COM just below the load latch.
        // The latch remains the physical authority; this is only its command-space target.
        constexpr float kNewSupportLoadCommandTarget = 0.74f;
        // Straight walking deliberately commits the role swap shortly after the COM
        // crosses the support midpoint. The same continuous support curve remains active
        // after the swap, so the remaining load transfer overlaps the next takeoff/swing
        // instead of serializing behind HOLD. Turn-conditioned steps retain the more
        // conservative handoff until their separate cadence work is addressed.
        const float kNewSupportLoadAcquireThreshold =
            walkingOverlapHandoff ? 0.52f : 0.68f;
        const float kNewSupportLoadReleaseThreshold =
            walkingOverlapHandoff ? 0.48f : 0.64f;
        auto supportLoadFraction = [](const glm::vec3& point,
                                      const glm::vec3& oldSupport,
                                      const glm::vec3& newSupport) {
            glm::vec3 span = newSupport - oldSupport;
            span.y = 0.0f;
            const float spanLengthSq = glm::dot(span, span);
            if (spanLengthSq <= 1e-6f) return 0.5f;
            const glm::vec3 offset(
                point.x - oldSupport.x, 0.0f, point.z - oldSupport.z);
            return glm::clamp(
                glm::dot(offset, span) / spanLengthSq, 0.0f, 1.0f);
        };
        auto supportCurveEndVelocityForObjective = [&](glm::vec3 start,
                                                       glm::vec3 end,
                                                       float duration,
                                                       bool turning) {
            glm::vec3 span = end - start;
            span.y = 0.0f;
            glm::vec3 direction = comp._physicalStepForward;
            float distance = glm::max(glm::dot(
                span, comp._physicalStepForward), 0.0f);
            if (turning) {
                distance = glm::length(span);
                direction = distance > 1e-6f
                    ? span / distance : stepObjectiveForward();
            }
            if (distance <= 1e-6f)
                return glm::vec3(0.0f);
            const float speed = glm::clamp(
                0.25f * distance / glm::max(duration, 0.01f),
                0.03f, 0.10f);
            return direction * speed;
        };
        auto supportCurveEndVelocity = [&](glm::vec3 start,
                                           glm::vec3 end,
                                           float duration) {
            return supportCurveEndVelocityForObjective(
                start, end, duration, hasTurnConditionedStep());
        };
        auto evaluateSupportCurveSegment = [](
                const glm::vec3& start,
                const glm::vec3& startVelocity,
                const glm::vec3& end,
                const glm::vec3& endVelocity,
                float duration,
                float queryTime,
                glm::vec3& position,
                glm::vec3& velocity) {
            duration = glm::max(duration, 0.01f);
            const float curveT = glm::clamp(queryTime / duration, 0.0f, 1.0f);
            const float t2 = curveT * curveT;
            const float t3 = t2 * curveT;
            const float h00 = 2.0f * t3 - 3.0f * t2 + 1.0f;
            const float h10 = t3 - 2.0f * t2 + curveT;
            const float h01 = -2.0f * t3 + 3.0f * t2;
            const float h11 = t3 - t2;
            position = h00 * start + h10 * duration * startVelocity
                + h01 * end + h11 * duration * endVelocity;

            const float dh00 = 6.0f * t2 - 6.0f * curveT;
            const float dh10 = 3.0f * t2 - 4.0f * curveT + 1.0f;
            const float dh01 = -6.0f * t2 + 6.0f * curveT;
            const float dh11 = 3.0f * t2 - 2.0f * curveT;
            velocity = (dh00 * start + dh01 * end) / duration
                + dh10 * startVelocity + dh11 * endVelocity;

            if (queryTime > duration) {
                const float cruiseTime = queryTime - duration;
                position = end + endVelocity * cruiseTime;
                velocity = endVelocity;
            }
            position.y = start.y;
            velocity.y = 0.0f;
        };
        auto evaluateActiveSupportCurveAt = [&](float queryTime,
                                                 glm::vec3& position,
                                                 glm::vec3& velocity) {
            evaluateSupportCurveSegment(
                comp._gaitSupportCurveStart,
                comp._gaitSupportCurveStartVelocity,
                comp._gaitSupportCurveEnd,
                comp._gaitSupportCurveEndVelocity,
                comp._gaitSupportCurveDuration,
                queryTime,
                position,
                velocity);
        };
        bool startedSupportCurveThisFrame = false;
        if (continuousEnabled && comp._gaitRunning
            && comp._gaitCancelMode == 0
            && comp._physicalStepPhase >= kDescent
            && comp._physicalStepPhase <= kTouchdownWait
            && comp._gaitSupportCurveStep != comp._stepSequenceStepIndex) {
            const glm::vec3 oldSupport = comp._physicalStepSupportSide < 0
                ? leftFoot : rightFoot;
            const float transferFraction = glm::clamp(
                comp.transferSupportBias + comp._gaitAdaptiveTransferBiasOffset,
                kNewSupportLoadCommandTarget, 0.98f);
            comp._gaitSupportCurveActive = true;
            comp._gaitSupportCurveStep = comp._stepSequenceStepIndex;
            comp._gaitSupportCurveTime = 0.0f;
            comp._gaitSupportCurveDuration = glm::max(
                cadenceDescentTime, 0.18f);
            comp._gaitSupportCurveStart = comp._physicalStepSupportTarget;
            const glm::vec3 fullTransferTarget = glm::mix(
                oldSupport, comp._physicalStepFoothold, transferFraction);
            const float preContactSupportFraction =
                0.20f;
            comp._gaitSupportCurveEnd = glm::mix(
                comp._gaitSupportCurveStart, fullTransferTarget,
                preContactSupportFraction);
            comp._gaitSupportCurveEnd.y = comp._gaitSupportCurveStart.y;

            glm::vec3 incomingVelocity =
                comp._gaitSupportCommandVelocity;
            incomingVelocity.y = 0.0f;
            const float incomingSpeed = glm::length(incomingVelocity);
            if (incomingSpeed > configuredSupportMaxSpeed
                && incomingSpeed > 1e-6f) {
                incomingVelocity *= configuredSupportMaxSpeed / incomingSpeed;
            }
            comp._gaitSupportCurveStartVelocity = incomingVelocity;

            comp._gaitSupportCurveEndVelocity = supportCurveEndVelocity(
                comp._gaitSupportCurveStart,
                comp._gaitSupportCurveEnd,
                comp._gaitSupportCurveDuration);
            startedSupportCurveThisFrame = true;
            spdlog::info(
                "[LocomotionGait] SUPPORT_CURVE_BEGIN step={} duration={:.3f}s "
                "mode=preload fraction={:.2f} load={:.2f} "
                "limits=(speed={:.2f}mps,accel={:.2f}mps2) "
                "velocity=({:+.3f},{:+.3f})->({:+.3f},{:+.3f})",
                comp._stepSequenceStepIndex,
                comp._gaitSupportCurveDuration,
                preContactSupportFraction,
                comp._gaitNewSupportLoad,
                configuredSupportMaxSpeed,
                walkingOverlapHandoff ? 3.0f : 1.75f,
                comp._gaitSupportCurveStartVelocity.x,
                comp._gaitSupportCurveStartVelocity.z,
                comp._gaitSupportCurveEndVelocity.x,
                comp._gaitSupportCurveEndVelocity.z);
        }
        const bool supportCurveOwnsWalkingTarget = continuousEnabled
            && comp._gaitRunning && comp._gaitSupportCurveActive
            && comp._physicalStepPhase >= kWeightShift
            && comp._physicalStepPhase <= kInterStep
            && !awaitingSupportOwnership;
        if (supportCurveOwnsWalkingTarget) {
            if (!startedSupportCurveThisFrame)
                comp._gaitSupportCurveTime += dt;
            glm::vec3 curveTarget(0.0f), curveVelocity(0.0f);
            evaluateActiveSupportCurveAt(
                comp._gaitSupportCurveTime, curveTarget, curveVelocity);

            if ((walkingOverlapHandoff
                    && comp._physicalStepPhase >= kSettle)
                || comp._physicalStepPhase == kTransfer
                || comp._physicalStepPhase == kHold) {
                const glm::vec3 liveOldSupport =
                    comp._physicalStepSupportSide < 0 ? leftFoot : rightFoot;
                const glm::vec3 liveNewSupport =
                    comp._physicalStepSupportSide < 0 ? rightFoot : leftFoot;
                glm::vec3 liveSpan = liveNewSupport - liveOldSupport;
                liveSpan.y = 0.0f;
                if (glm::dot(liveSpan, liveSpan) > 1e-6f) {
                    // The Hermite endpoint is frozen when ownership is acquired, but both
                    // physical feet can still settle afterward. Correct only the component
                    // along their live span so the command remains consistent with the load
                    // fraction used by HOLD. Velocity and acceleration limits below keep this
                    // feedback C0/C1-safe at the support-controller boundary.
                    const float curveLoad = supportLoadFraction(
                        curveTarget, liveOldSupport, liveNewSupport);
                    if (curveLoad < kNewSupportLoadCommandTarget) {
                        curveTarget += liveSpan
                            * (kNewSupportLoadCommandTarget - curveLoad);
                    }
                }
            }

            // Hermite endpoint tangents do not bound the polynomial's middle velocity.
            // Track the curve through an explicit command-space speed and acceleration
            // limiter so a large support span cannot yank a planted rear foot.
            constexpr float kSupportPositionGain = 4.0f;
            glm::vec3 desiredVelocity = curveVelocity
                + (curveTarget - comp._physicalStepSupportTarget)
                    * kSupportPositionGain;
            desiredVelocity.y = 0.0f;
            const bool acquiringPlant = !walkingOverlapHandoff
                && (comp._physicalStepPhase == kSettle
                    || comp._physicalStepPhase == kSupportReady)
                && comp._physicalStepTouchdownAccepted
                && !comp._physicalStepPlantPoseCaptured;
            float maximumSupportSpeed = configuredSupportMaxSpeed;
            float maximumSupportAcceleration = walkingOverlapHandoff
                ? 3.0f : 1.75f;
            if (acquiringPlant) {
                const float driftPressure = glm::clamp(
                    (comp._physicalStepPlantDrift - 0.015f) / 0.015f,
                    0.0f, 1.0f);
                maximumSupportSpeed = glm::min(
                    configuredSupportMaxSpeed,
                    glm::mix(0.15f, 0.08f, driftPressure));
                maximumSupportAcceleration = glm::mix(
                    0.85f, 0.55f, driftPressure);

                // Do not turn a growing plant slide into a full-body stop. Preserve a
                // small forward flow while strongly reducing the lateral load ramp that
                // is pulling the sole away from its world anchor.
                const glm::vec3 objectiveForward = stepObjectiveForward();
                const glm::vec3 objectiveRight = stepObjectiveRight();
                const float forwardSpeed = glm::clamp(glm::dot(
                    desiredVelocity, objectiveForward),
                    -0.08f, 0.10f);
                const float lateralLimit = glm::mix(
                    0.10f, 0.03f, driftPressure);
                const float lateralSpeed = glm::clamp(glm::dot(
                    desiredVelocity, objectiveRight),
                    -lateralLimit, lateralLimit);
                desiredVelocity = objectiveForward * forwardSpeed
                    + objectiveRight * lateralSpeed;
            } else if (comp._physicalStepPhase == kTransfer) {
                // Once acquisition succeeds, ramp rather than jump from the conservative
                // plant speed back to the ordinary transfer ceiling.
                const float transferRamp = smoothstep(glm::clamp(
                    comp._physicalStepPhaseTime / 0.15f, 0.0f, 1.0f));
                const float transferStartSpeed = glm::min(
                    configuredSupportMaxSpeed, 0.15f);
                maximumSupportSpeed = glm::mix(
                    transferStartSpeed, configuredSupportMaxSpeed, transferRamp);
                maximumSupportAcceleration = glm::mix(
                    walkingOverlapHandoff ? 1.50f : 0.85f,
                    walkingOverlapHandoff ? 3.0f : 1.75f,
                    transferRamp);
            }
            const float desiredSpeed = glm::length(desiredVelocity);
            if (desiredSpeed > maximumSupportSpeed
                && desiredSpeed > 1e-6f) {
                desiredVelocity *= maximumSupportSpeed / desiredSpeed;
            }
            glm::vec3 accelerationDelta = desiredVelocity
                - comp._gaitSupportCommandVelocity;
            const bool decelerating = glm::length(desiredVelocity)
                < glm::length(comp._gaitSupportCommandVelocity);
            if (decelerating)
                maximumSupportAcceleration = glm::max(
                    maximumSupportAcceleration, 2.50f);
            const float maximumVelocityChange =
                maximumSupportAcceleration * dt;
            const float velocityChange = glm::length(accelerationDelta);
            if (velocityChange > maximumVelocityChange
                && velocityChange > 1e-6f) {
                accelerationDelta *= maximumVelocityChange / velocityChange;
            }
            comp._gaitSupportCommandVelocity += accelerationDelta;
            supportVelocity = comp._gaitSupportCommandVelocity;
            supportTarget = comp._physicalStepSupportTarget
                + supportVelocity * dt;
            supportTarget.y = comp._gaitSupportCurveStart.y;
            supportVelocityExplicit = true;
        }
        if (continuousEnabled && comp._physicalStepPhase == kStopping) {
            // The last completed transfer already produced a stable support point. Hold it
            // throughout shutdown instead of pulling the COM toward the sole midpoint while
            // the legs are changing pose. A fixed target also makes the feed-forward term
            // exactly zero rather than deriving a velocity from numerical differencing.
            supportTarget = comp._gaitStopStartTarget;
            supportVelocity = glm::vec3(0.0f);
            supportVelocityExplicit = true;
        }
        if (!supportVelocityExplicit) {
            supportVelocity = dt > 1e-6f
                ? (supportTarget - comp._physicalStepSupportTarget) / dt
                : glm::vec3(0.0f);
        }
        if (!supportCurveOwnsWalkingTarget)
            comp._gaitSupportCommandVelocity = supportVelocity;
        supportVelocity.y = 0.0f;
        comp._physicalStepSupportTarget = supportTarget;

        rag.locomotionSupportTarget = supportTarget;
        rag.locomotionSupportTargetVel = supportVelocity;
        rag.locomotionSupportTargetWeight =
            continuousEnabled && (comp._physicalStepPhase == kReturnStand
                                  || comp._physicalStepPhase == kComplete)
                ? 0.0f : 1.0f;
        rag.locomotionCOMSupportFreq = glm::max(comp.supportFrequency, 0.0f);
        rag.locomotionCOMSupportMaxAccel = glm::max(comp.supportMaxAcceleration, 0.0f);

        // Shutdown sole captures are diagnostic references only. The per-frame controller
        // reset above leaves both lock bones disabled and both weights at zero; do not add a
        // second world-space controller while support and joint motors settle the character.
        if (continuousEnabled && comp._physicalStepPhase == kStopping
            && !comp._gaitStopSettleReferenceValid) {
            // This is commanded displacement while the captured walking pose becomes the
            // standing pose. Keep it visible, but do not treat it as continued plant slide.
            comp._gaitStopFootDriftL = glm::length(glm::vec2(
                leftFoot.x - comp._gaitStopFootTargetL.x,
                leftFoot.z - comp._gaitStopFootTargetL.z));
            comp._gaitStopFootDriftR = glm::length(glm::vec2(
                rightFoot.x - comp._gaitStopFootTargetR.x,
                rightFoot.z - comp._gaitStopFootTargetR.z));
            comp._gaitStopMaxFootDrift = glm::max(
                comp._gaitStopMaxFootDrift,
                glm::max(comp._gaitStopFootDriftL,
                         comp._gaitStopFootDriftR));
        }
        if (continuousEnabled && comp._gaitStopSettleReferenceValid
            && (comp._physicalStepPhase == kStopping
                || comp._physicalStepPhase == kReturnStand)) {
            comp._gaitStopSettleFootDriftL = glm::length(glm::vec2(
                leftFoot.x - comp._gaitStopSettleFootTargetL.x,
                leftFoot.z - comp._gaitStopSettleFootTargetL.z));
            comp._gaitStopSettleFootDriftR = glm::length(glm::vec2(
                rightFoot.x - comp._gaitStopSettleFootTargetR.x,
                rightFoot.z - comp._gaitStopSettleFootTargetR.z));
            comp._gaitStopMaxSettleFootDrift = glm::max(
                comp._gaitStopMaxSettleFootDrift,
                glm::max(comp._gaitStopSettleFootDriftL,
                         comp._gaitStopSettleFootDriftR));
        }

        comp._physicalStepComLateral = glm::dot(
            rag._locomotionCOM - comp._physicalStepComBaseline, comp._physicalStepRight);
        comp._physicalStepTargetLateral = glm::dot(
            supportTarget - comp._physicalStepComBaseline, comp._physicalStepRight);
        comp._physicalStepPeakTilt = glm::max(comp._physicalStepPeakTilt, tiltDeg);
        comp._physicalStepFinalTilt = tiltDeg;

        Leg* swing = comp._physicalStepSupportSide < 0 ? &comp._legR : &comp._legL;
        Leg* stance = comp._physicalStepSupportSide < 0 ? &comp._legL : &comp._legR;
        const glm::vec3 swingFoot = physicalPosition(swing->footIdx);
        const glm::vec3 stanceFoot = physicalPosition(stance->footIdx);
        const glm::vec3 swingVelocity = Physics::GetRagdollBoneLinearVelocity(
            rag, swing->footIdx);
        bool swingAngularVelocityOk = false;
        const glm::vec3 swingAngularVelocity =
            Physics::GetRagdollBoneAngularVelocity(
                rag, swing->footIdx, &swingAngularVelocityOk);
        auto horizontalDistance = [](const glm::vec3& a, const glm::vec3& b) {
            return glm::length(glm::vec2(a.x - b.x, a.z - b.z));
        };
        comp._supportTransferComHorizontalSpeed = glm::length(glm::vec2(
            rag._locomotionCOMVel.x, rag._locomotionCOMVel.z));
        if (transferEnabled && comp._physicalStepPhase >= kTransfer
            && comp._physicalStepPhase <= kComplete) {
            comp._supportTransferComError = horizontalDistance(
                rag._locomotionCOM, comp._supportTransferTransferEndTarget);
            comp._supportTransferComToOldSupport = horizontalDistance(
                rag._locomotionCOM, stanceFoot);
            comp._supportTransferComToNewSupport = horizontalDistance(
                rag._locomotionCOM, swingFoot);
        }
        const bool transferOrHold = comp._physicalStepPhase >= kTransfer
                                 && comp._physicalStepPhase <= kHold;
        comp._physicalStepApiVelocity = swingVelocity;
        glm::vec3 contactNormal(0.0f);
        glm::vec3 contactPoint(0.0f);
        const bool rawSwingContact = FootContact(
            rag, swing->footIdx, &contactNormal, &contactPoint);
        const int swingContactSlot = FootContactSlot(rag, swing->footIdx);
        const float swingSoleMinY = swingContactSlot >= 0
            ? rag._locomotionFootSoleMinY[swingContactSlot]
            : swingFoot.y;
        const float rawContactPenetration = swingContactSlot >= 0
            ? rag._locomotionFootPenetration[swingContactSlot]
            : 0.0f;
        const float measuredSoleClearance =
            comp._physicalStepSoleReferenceValid
                ? swingSoleMinY - comp._physicalStepSoleStartMinY
                : swingFoot.y - comp._physicalStepSwingStart.y;
        comp._physicalStepSoleClearance = measuredSoleClearance;
        comp._physicalStepContactPenetration = rawContactPenetration;
        // Jolt reports speculative manifolds before shapes actually touch. Penetration is
        // the authority here: a sufficiently negative value is a separated pair even if
        // the foot's vertical velocity briefly turns downward. The old velocity condition
        // converted that predictive manifold back into a contact every oscillation.
        constexpr float kSpeculativeContactClearance = 0.0015f;
        const bool separatedByPenetration = rawSwingContact
            && rawContactPenetration < -0.001f;
        // An airborne foot needs strict evidence before a new touchdown can be accepted.
        // Keep the turn predicate unchanged until the turn controller is handled
        // separately; straight walking rejects every clearly separated manifold here.
        const bool speculativeTouchdownContact = separatedByPenetration
            && (walkingOverlapHandoff
                || measuredSoleClearance >= kSpeculativeContactClearance);
        // At the final turn landmark Jolt can retain a quiet, grounded sole as a
        // speculative manifold a few millimetres above its nominal surface. This is not
        // permission to create touchdown from proximity: require the raw manifold,
        // grounded state, final phase, foothold envelope, and quiet measured motion.
        constexpr float kTurnFinalContactClearance = 0.010f;
        const float finalContactHorizontalError = horizontalDistance(
            swingFoot, comp._physicalStepFoothold);
        const float finalContactVerticalError = std::abs(
            swingFoot.y - comp._physicalStepFoothold.y);
        const bool quietTurnFinalContact = hasYawBearingTurnStep()
            && comp._physicalStepPhase == kTouchdownWait
            && comp._physicalStepTrajectoryT >= 1.0f - 1e-4f
            && rawSwingContact
            && FootGrounded(rag, swing->footIdx)
            && rawContactPenetration >= -kTurnFinalContactClearance
            && measuredSoleClearance <= kTurnFinalContactClearance
            && finalContactHorizontalError <= 0.060f
            && finalContactVerticalError <= 0.040f
            && std::abs(swingVelocity.y) <= 0.08f
            && glm::length(glm::vec2(swingVelocity.x, swingVelocity.z))
                <= 0.15f;
        const bool swingTouchdownContactNow = rawSwingContact
            && (!speculativeTouchdownContact || quietTurnFinalContact);

        // Once a sole already owns support, the same speculative manifold has different
        // semantics: Jolt can keep a resting sole 2-8 mm inside its predictive contact
        // distance while still supplying the support impulse. Do not revoke that ownership
        // unless the sole also moves away from its captured surface. Ground proximity is a
        // fallback only for an already accepted, quiet plant; it can never create a new
        // touchdown by itself.
        constexpr float kMaintainedSupportClearance = 0.006f;
        const float maintainedSupportClearance = hasYawBearingTurnStep()
            ? kTurnFinalContactClearance
            : kMaintainedSupportClearance;
        const bool nearCapturedSupportSurface = measuredSoleClearance
            <= maintainedSupportClearance;
        const bool quietGroundProximity = comp._physicalStepTouchdownAccepted
            && FootGrounded(rag, swing->footIdx)
            && nearCapturedSupportSurface
            && std::abs(swingVelocity.y) <= 0.08f
            && glm::length(glm::vec2(swingVelocity.x, swingVelocity.z))
                <= 0.15f;
        const bool swingMaintainedSupportNow =
            (rawSwingContact
                && (!comp._physicalStepTouchdownAccepted
                    || rawContactPenetration >= -0.001f
                    || nearCapturedSupportSurface))
            || quietGroundProximity;
        const bool airborneContactSemantics =
            comp._physicalStepPhase >= kTakeoff
            && comp._physicalStepPhase <= kTouchdownWait;
        // All existing phase code consumes this dispatcher, but touchdown admission and
        // maintained support are now independent facts rather than one overloaded boolean.
        const bool swingContactNow = airborneContactSemantics
            ? swingTouchdownContactNow : swingMaintainedSupportNow;
        const bool diagnoseSpeculativeContact = continuousEnabled
            && comp._physicalStepPhase >= kTakeoff
            && comp._physicalStepPhase <= kSwing;
        if (diagnoseSpeculativeContact && speculativeTouchdownContact
            && !comp._gaitSpeculativeContactActive) {
            ++comp._gaitStepSpeculativeContacts;
            spdlog::info(
                "[LocomotionContact] SPECULATIVE_REJECT step={} phase={} "
                "soleClear={:.3f}m penetration={:+.4f}m vy={:+.3f}mps",
                comp._stepSequenceStepIndex,
                comp._physicalStepPhase,
                measuredSoleClearance,
                rawContactPenetration,
                swingVelocity.y);
        }
        comp._gaitSpeculativeContactActive = diagnoseSpeculativeContact
            && speculativeTouchdownContact;
        const bool stanceContactNow = FootContact(rag, stance->footIdx);
        const float loadHorizontalSpeed = glm::length(glm::vec2(
            swingVelocity.x, swingVelocity.z));
        const float configuredLoadHorizontalLimit = glm::clamp(
            comp.touchdownMaxHorizontalSpeed, 0.05f, 2.0f);
        const float configuredLoadAngularLimit = glm::clamp(
            comp.touchdownMaxAngularSpeed, 0.25f, 10.0f);
        const bool loadOwnershipKinematicsReady = !continuousEnabled
            || (comp._physicalStepTouchdownAccepted
                && loadHorizontalSpeed
                    <= (hasYawBearingTurnStep()
                        ? glm::min(configuredLoadHorizontalLimit, 0.12f)
                        : configuredLoadHorizontalLimit)
                && swingAngularVelocityOk
                && glm::length(swingAngularVelocity)
                    <= (hasYawBearingTurnStep()
                        ? glm::min(configuredLoadAngularLimit, 0.75f)
                        : configuredLoadAngularLimit)
                && comp._gaitSoleAngularErrorDeg
                    <= (hasYawBearingTurnStep() ? 10.0f : 15.0f));
        comp._gaitNewSupportLoad = supportLoadFraction(
            rag._locomotionCOM, stanceFoot, swingFoot);
        const float gaitSupportCommandLoad = supportLoadFraction(
            comp._physicalStepSupportTarget, stanceFoot, swingFoot);
        const bool loadHandoffPhase = comp._physicalStepPhase
                                        >= (walkingOverlapHandoff
                                            ? kSettle : kTransfer)
                                   && comp._physicalStepPhase <= kInterStep;
        if (!continuousEnabled || !loadHandoffPhase) {
            comp._gaitNewSupportLoadLatched = false;
        } else if (!comp._gaitNewSupportLoadLatched
                   && loadOwnershipKinematicsReady
                   && comp._gaitNewSupportLoad
                        >= kNewSupportLoadAcquireThreshold) {
            comp._gaitNewSupportLoadLatched = true;
            spdlog::info(
                "[LocomotionGait] SUPPORT_LOAD_LATCH step={} action=ACQUIRE "
                "load={:.3f} thresholds={:.2f}/{:.2f}",
                comp._stepSequenceStepIndex,
                comp._gaitNewSupportLoad,
                kNewSupportLoadAcquireThreshold,
                kNewSupportLoadReleaseThreshold);
        } else if (comp._gaitNewSupportLoadLatched
                   && (!loadOwnershipKinematicsReady
                       || comp._gaitNewSupportLoad
                            < kNewSupportLoadReleaseThreshold)) {
            comp._gaitNewSupportLoadLatched = false;
            spdlog::info(
                "[LocomotionGait] SUPPORT_LOAD_LATCH step={} action=RELEASE "
                "load={:.3f} thresholds={:.2f}/{:.2f}",
                comp._stepSequenceStepIndex,
                comp._gaitNewSupportLoad,
                kNewSupportLoadAcquireThreshold,
                kNewSupportLoadReleaseThreshold);
        }
        const bool gaitOldSupportUnloaded = continuousEnabled && transferOrHold
            && comp._gaitNewSupportLoadLatched;
        if (comp._physicalStepPreviousSwingFootValid && dt > 1e-6f)
            comp._physicalStepMeasuredVelocity =
                (swingFoot - comp._physicalStepPreviousSwingFoot) / dt;
        else
            comp._physicalStepMeasuredVelocity = glm::vec3(0.0f);
        comp._physicalStepPreviousSwingFoot = swingFoot;
        comp._physicalStepPreviousSwingFootValid = true;

        bool swingRotationOk = false;
        const glm::quat swingRotation = Physics::GetRagdollBoneRotation(
            rag, swing->footIdx, &swingRotationOk);
        comp._physicalStepPlantAngularSpeed = swingAngularVelocityOk
            ? glm::length(swingAngularVelocity) : 0.0f;
        if (continuousEnabled && comp._gaitTurnPlan.candidateEvaluated
            && comp._physicalStepPhase >= kTakeoff
            && comp._physicalStepPhase <= kTouchdownWait) {
            comp._gaitTurnPlan.achievedSwingSpeed = glm::max(
                comp._gaitTurnPlan.achievedSwingSpeed,
                glm::length(glm::vec2(swingVelocity.x, swingVelocity.z)));
            if (swingAngularVelocityOk) {
                comp._gaitTurnPlan.achievedAngularSpeed = glm::max(
                    comp._gaitTurnPlan.achievedAngularSpeed,
                    glm::length(swingAngularVelocity));
            }
        }
        comp._physicalStepFootUpY = swingRotationOk
            ? (swingRotation * glm::vec3(0.0f, 1.0f, 0.0f)).y : 0.0f;
        comp._physicalStepContactPoint = contactPoint;
        // Contact-point geometry is valid only when the solver supplied a manifold this
        // frame. The grounded-proximity fallback maintains ownership but carries no point.
        comp._physicalStepContactLocal = rawSwingContact && swingRotationOk
            ? glm::conjugate(swingRotation) * (contactPoint - swingFoot)
            : glm::vec3(0.0f);
        if (comp._physicalStepTouchdownAccepted
            && comp._physicalStepTouchdownContactValid
            && rawSwingContact && swingRotationOk) {
            // The deepest collision point is expected to walk across a rolling sole.
            // Keep the largest observed migration so a later contact-query flip cannot
            // silently re-authorize correction toward the obsolete impact edge.
            comp._physicalStepPlantContactMigration = glm::max(
                comp._physicalStepPlantContactMigration,
                glm::length(comp._physicalStepContactLocal
                    - comp._physicalStepTouchdownContactLocal));
        }
        const glm::vec3 stanceBaseline = comp._physicalStepSupportSide < 0
            ? comp._physicalStepFootBaselineL : comp._physicalStepFootBaselineR;
        comp._physicalStepStanceDrift = glm::length(glm::vec2(
            stanceFoot.x - stanceBaseline.x, stanceFoot.z - stanceBaseline.z));
        comp._physicalStepMaxStanceDrift = glm::max(
            comp._physicalStepMaxStanceDrift, comp._physicalStepStanceDrift);
        if (comp._physicalStepTouchdownAccepted) {
            comp._physicalStepPlantCenterTravel = glm::length(glm::vec2(
                swingFoot.x - comp._physicalStepTouchdownPlant.x,
                swingFoot.z - comp._physicalStepTouchdownPlant.z));
            if (comp._physicalStepTouchdownContactValid && swingRotationOk) {
                const glm::vec3 currentTouchdownMaterialPoint = swingFoot
                    + swingRotation
                        * comp._physicalStepTouchdownContactLocal;
                comp._physicalStepPlantDrift = glm::length(glm::vec2(
                    currentTouchdownMaterialPoint.x
                        - comp._physicalStepTouchdownContactWorld.x,
                    currentTouchdownMaterialPoint.z
                        - comp._physicalStepTouchdownContactWorld.z));
            } else {
                // Retain the old center metric only when the physics query could not
                // provide a usable touchdown material point.
                comp._physicalStepPlantDrift =
                    comp._physicalStepPlantCenterTravel;
            }
            const float driftRateSample = dt > 1e-6f
                ? (comp._physicalStepPlantDrift
                    - comp._gaitPlantPreviousDrift) / dt
                : 0.0f;
            comp._gaitPlantDriftRate = glm::mix(
                comp._gaitPlantDriftRate, driftRateSample, 0.25f);
            comp._gaitPlantPreviousDrift =
                comp._physicalStepPlantDrift;
            comp._physicalStepMaxPlantDrift = glm::max(
                comp._physicalStepMaxPlantDrift, comp._physicalStepPlantDrift);
        } else {
            comp._physicalStepPlantDrift = 0.0f;
            comp._physicalStepPlantCenterTravel = 0.0f;
            comp._gaitPlantPreviousDrift = 0.0f;
            comp._gaitPlantDriftRate = 0.0f;
        }
        comp._physicalStepClearance = swingFoot.y - comp._physicalStepSwingStart.y;
        comp._physicalStepForwardTravel = stepObjectiveAdvance(
            swingFoot - comp._physicalStepSwingStart);
        comp._gaitAchievedSupportAdvance = continuousEnabled
            ? stepObjectiveAdvance(swingFoot - stanceFoot) : 0.0f;
        if (continuousEnabled)
            comp._gaitTurnPlan.achievedAdvance =
                comp._gaitAchievedSupportAdvance;
        if (comp._physicalStepPhase >= kTakeoff) {
            const glm::vec3 targetDelta = swingFoot - comp._physicalStepFoothold;
            comp._physicalStepForwardTargetError = stepObjectiveAdvance(
                comp._physicalStepFoothold - swingFoot);
            comp._physicalStepLateralTargetError = stepObjectiveLateral(
                targetDelta);
            comp._physicalStepHorizontalTargetError = glm::length(glm::vec2(
                targetDelta.x, targetDelta.z));
            comp._physicalStepVerticalTargetError = std::abs(targetDelta.y);
        } else {
            comp._physicalStepForwardTargetError = 0.0f;
            comp._physicalStepLateralTargetError = 0.0f;
            comp._physicalStepHorizontalTargetError = 0.0f;
            comp._physicalStepVerticalTargetError = 0.0f;
        }

        comp._physicalStepMotorSaturated = false;
        for (int i = 0; i < 6; ++i) {
            comp._physicalStepMaxMotorRatio = glm::max(
                comp._physicalStepMaxMotorRatio, rag._locomotionMotorSaturationRatio[i]);
            comp._physicalStepMotorSaturated = comp._physicalStepMotorSaturated
                || rag._locomotionMotorSaturated[i];
        }
        if (continuousEnabled && comp._gaitRunning) {
            const bool activeGaitPhase = comp._physicalStepPhase >= kWeightShift
                                      && comp._physicalStepPhase <= kInterStep;
            if (activeGaitPhase) {
                // Drift follows support ownership, not the geometric unload estimate.
                // Before plant acquisition the stance foot is the loaded support. Once the
                // new plant is settled, that foot owns support stability and movement of
                // the old foot is release quality for the next swing, not support drift.
                const bool newSupportAcquired =
                    comp._physicalStepPlantPoseCaptured
                    && comp._physicalStepPhase >= kSettle;
                const float relevantDrift = newSupportAcquired
                    ? comp._physicalStepPlantDrift
                    : comp._physicalStepStanceDrift;
                comp._gaitStepMaxRelevantDrift = glm::max(
                    comp._gaitStepMaxRelevantDrift, relevantDrift);
                comp._gaitMaxDrift = glm::max(
                    comp._gaitMaxDrift, relevantDrift);
            }
            comp._gaitPeakTilt = glm::max(comp._gaitPeakTilt, tiltDeg);
            comp._gaitMaxMotorRatio = glm::max(
                comp._gaitMaxMotorRatio, comp._physicalStepMaxMotorRatio);
        }

        auto captureLegSolveReference = [&](Leg& leg,
                                            const glm::vec3& measuredFoot) {
            const glm::vec3 hip = physicalPosition(leg.hipIdx);
            const glm::vec3 knee = physicalPosition(leg.kneeIdx);
            const glm::vec3 ankle = physicalPosition(leg.ankleIdx);
            leg.desiredFoot = measuredFoot;
            leg.ankleFromFootWorld = ankle - measuredFoot;
            const glm::vec3 upper = knee - hip;
            const glm::vec3 lower = ankle - knee;
            if (glm::dot(upper, upper) > 1e-8f && glm::dot(lower, lower) > 1e-8f) {
                leg.referenceUpperWorld = glm::normalize(upper);
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
            }

            bool hipOk = false, kneeOk = false, ankleOk = false, footOk = false;
            leg.referenceHipWorld = Physics::GetRagdollBoneRotation(
                rag, leg.hipIdx, &hipOk);
            const glm::quat kneeWorld = Physics::GetRagdollBoneRotation(
                rag, leg.kneeIdx, &kneeOk);
            const glm::quat ankleWorld = Physics::GetRagdollBoneRotation(
                rag, leg.ankleIdx, &ankleOk);
            const glm::quat footWorld = Physics::GetRagdollBoneRotation(
                rag, leg.footIdx, &footOk);
            leg.segmentGeometryValid = hipOk && kneeOk && ankleOk
                && glm::dot(upper, upper) > 1e-8f
                && glm::dot(lower, lower) > 1e-8f;
            if (leg.segmentGeometryValid) {
                // Body positions are ragdoll bone origins. Express the constrained
                // parent-to-child offsets in the corresponding parent frames so both
                // analytical IK and commanded FK use the exact same physical links.
                leg.upperSegmentLocal = glm::conjugate(
                    glm::normalize(leg.referenceHipWorld)) * upper;
                leg.lowerSegmentLocal = glm::conjugate(
                    glm::normalize(kneeWorld)) * lower;
                leg.referenceUpperWorld = glm::normalize(
                    glm::normalize(leg.referenceHipWorld)
                        * leg.upperSegmentLocal);
            } else {
                leg.upperSegmentLocal =
                    skeleton.bones[leg.kneeIdx].localT;
                leg.lowerSegmentLocal =
                    skeleton.bones[leg.ankleIdx].localT;
            }
            if (footOk) {
                leg.plantedFootWorldRotation = footWorld;
                leg.ankleFromFootLocal = glm::conjugate(footWorld)
                    * (ankle - measuredFoot);
            } else {
                leg.plantedFootWorldRotation = OrientationOf(BoneWorldMatrix(
                    skeleton, animator, entityWorld, leg.footIdx));
                leg.ankleFromFootLocal =
                    glm::conjugate(leg.plantedFootWorldRotation)
                    * (ankle - measuredFoot);
            }
            if (hipOk && kneeOk)
                leg.referenceKneeLocal = glm::normalize(
                    glm::conjugate(leg.referenceHipWorld) * kneeWorld);
            else
                leg.referenceKneeLocal = skeleton.bones[leg.kneeIdx].localR;
            if (kneeOk && ankleOk)
                leg.referenceAnkleLocal = glm::normalize(
                    glm::conjugate(kneeWorld) * ankleWorld);
            else
                leg.referenceAnkleLocal = skeleton.bones[leg.ankleIdx].localR;
            if (ankleOk && footOk)
                leg.referenceFootLocal = glm::normalize(
                    glm::conjugate(ankleWorld) * footWorld);
            else
                leg.referenceFootLocal = skeleton.bones[leg.footIdx].localR;
            if (!hipOk)
                leg.referenceHipWorld = glm::normalize(
                    ParentWorldRot(rag, skeleton, animator, entityWorld, leg.hipIdx)
                    * animator.pose[leg.hipIdx].rotation);

            leg.hipCommand = animator.pose[leg.hipIdx].rotation;
            leg.kneeCommand = leg.referenceKneeLocal;
            leg.ankleCommand = leg.referenceAnkleLocal;
            leg.footCommand = leg.referenceFootLocal;
            leg.kneePlaneSwivelRad = 0.0f;
            leg.commandValid = true;
            leg.plantSolveValid = true;
        };

        auto captureSwing = [&]() {
            comp._physicalStepSwingStart = swingFoot;
            comp._physicalStepArcStart = swingFoot;
            comp._physicalStepDesiredFoot = swingFoot;
            if (swingContactSlot >= 0) {
                comp._physicalStepSoleStartMinY =
                    rag._locomotionFootSoleMinY[swingContactSlot];
                comp._physicalStepSoleClearance = 0.0f;
                comp._physicalStepContactPenetration =
                    rag._locomotionFootPenetration[swingContactSlot];
                comp._physicalStepSoleReferenceValid = true;
            } else {
                comp._physicalStepSoleStartMinY = swingFoot.y;
                comp._physicalStepSoleClearance = 0.0f;
                comp._physicalStepContactPenetration = 0.0f;
                comp._physicalStepSoleReferenceValid = false;
            }
            comp._gaitStepRecontactPauseTime = 0.0f;
            comp._gaitStepMaxSoleErrorDeg = 0.0f;
            comp._gaitStepMaxSwingAngularSpeed = 0.0f;
            comp._gaitStepSpeculativeContacts = 0;
            comp._gaitSpeculativeContactActive = false;
            captureLegSolveReference(*swing, swingFoot);
            comp._physicalStepTrajectoryT = 0.0f;
            comp._gaitLandingBrakeReadyTime = 0.0f;
            comp._gaitLandingBrakeReleased = false;
            comp._gaitSwingCommandSpeed = 0.0f;
            comp._gaitSwingCommandAngularSpeed = 0.0f;
            comp._gaitSwingCommandTrackingError = 0.0f;
            comp._gaitSwingSoleCommandErrorDeg = 0.0f;
            comp._gaitAnkleCommandAngularSpeed = 0.0f;
            comp._gaitAnkleCommandDeltaDeg = 0.0f;
            comp._gaitAnkleParentCompensationDeg = 0.0f;
            comp._gaitAnkleParentCompensationAppliedDeg = 0.0f;
            comp._gaitAnkleAchievableResidualDeg = 0.0f;
            comp._gaitAnkleDiagnosticCooldown = 0.0f;
            comp._gaitIkAnkleEnvelopeClampDeg = 0.0f;
            comp._gaitIkExactAnkleClampDeg = 0.0f;
            comp._gaitIkKneeSwivelRequestedDeg = 0.0f;
            comp._gaitIkKneeSwivelAppliedDeg = 0.0f;
            comp._gaitIkOrientationPriority = 1.0f;
            comp._gaitIkOrientationRelaxedDeg = 0.0f;
            comp._gaitIkAnkleSwingUsage = 0.0f;
            comp._gaitIkAnkleSwingReserve = 0.0f;
            comp._gaitIkAnkleTwistDeg = 0.0f;
            comp._gaitIkAnkleTwistMarginDeg = 0.0f;
            comp._gaitCoherentJointScale = 1.0f;
            comp._gaitCoherentHipDeltaDeg = 0.0f;
            comp._gaitCoherentKneeDeltaDeg = 0.0f;
            comp._gaitCoherentAnkleDeltaDeg = 0.0f;
            comp._gaitFkDesiredPosition = glm::vec3(0.0f);
            comp._gaitFkCommandPosition = glm::vec3(0.0f);
            comp._gaitFkPhysicalPosition = glm::vec3(0.0f);
            comp._gaitFkDesiredToCommand = glm::vec3(0.0f);
            comp._gaitFkCommandToPhysical = glm::vec3(0.0f);
            comp._gaitFkDesiredToPhysical = glm::vec3(0.0f);
            comp._gaitFkDesiredToCommandOrientationDeg = 0.0f;
            comp._gaitFkCommandToPhysicalOrientationDeg = 0.0f;
            comp._gaitFkDesiredToPhysicalOrientationDeg = 0.0f;
            comp._gaitFkDiagnosticCooldown = 0.0f;
            comp._gaitFkValid = false;
            comp._gaitSwingPathLength = 0.0f;
            comp._gaitSwingMinimumDuration = 0.0f;
            comp._gaitSwingDeadline = 0.0f;
            comp._gaitSwingWatchdogProgress = 0.0f;
            comp._gaitSwingNoProgressTime = 0.0f;
            comp._gaitSwingSoleCommandWorld = glm::normalize(
                swing->plantedFootWorldRotation);
            comp._gaitSwingSoleCommandValid = true;
            swing->planted = false;
        };
        if (continuousEnabled && comp._gaitRunning && stance->planted
            && !stance->plantSolveValid
            && comp._physicalStepPhase >= kWeightShift
            && comp._physicalStepPhase <= kInterStep) {
            // The initial support leg has not previously been a swing leg, so capture its
            // world-anchor solve explicitly. Waiting until step two left the first rear
            // foot as an unsolved local pose while the pelvis shifted over it.
            captureLegSolveReference(*stance, stanceFoot);
        }

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
            leg.kneePlaneSwivelRad = 0.0f;
            leg.commandValid = true;
        };

        // The foothold is a target for the sole body, while the two-bone solve ends at
        // the ankle. Keep their separation in sole-local space. The settled sole reference
        // is heading-local: pitch/roll remain ground aligned while yaw follows the latched
        // gait frame. The per-step planted rotation is only the continuity endpoint.
        auto footWorldRotationForHeading = [&](const Leg& leg,
                                                const glm::quat& heading) {
            return continuousEnabled && leg.groundReferenceFootRotationValid
                ? glm::normalize(glm::normalize(heading)
                    * leg.groundReferenceFootHeadingLocalRotation)
                : glm::normalize(leg.plantedFootWorldRotation);
        };
        auto nominalFootWorldRotation = [&](const Leg& leg) {
            // A planted sole is a world-space physical fact. Only the released swing
            // sole follows the admitted end heading; body heading may continue moving
            // through support transfer without rotating either planted reference.
            if (continuousEnabled && leg.planted)
                return glm::normalize(leg.plantedFootWorldRotation);
            if (continuousEnabled && &leg == swing
                && comp._gaitCancelMode == 1) {
                // An early cancellation returns to the exact old sole pose while the
                // separately owned heading target unwinds continuously above.
                return glm::normalize(leg.plantedFootWorldRotation);
            }
            const glm::quat heading = continuousEnabled
                && &leg == swing
                && comp._gaitTurnPlan.activeHeadingPlan
                ? comp._gaitTurnPlan.activeEndRotation
                : comp._gaitHeadingTargetRot;
            return footWorldRotationForHeading(leg, heading);
        };
        auto nominalKneePoleWorld = [&](const Leg& leg) {
            glm::vec3 pole = continuousEnabled
                && leg.groundReferenceKneePoleValid
                ? comp._gaitHeadingTargetRot
                    * leg.groundReferenceKneePoleHeadingLocal
                : leg.kneePoleWorld;
            return glm::dot(pole, pole) > 1e-8f
                ? glm::normalize(pole) : comp._physicalStepForward;
        };
        auto ankleFromFootWorld = [&](const Leg& leg,
                                      const glm::quat& footWorldRotation) {
            return glm::normalize(footWorldRotation) * leg.ankleFromFootLocal;
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
        comp._gaitSoleAngularErrorDeg = continuousEnabled
            && physicalSwingFootRotationOk
            ? rotationDifferenceDeg(
                physicalSwingFootWorld, nominalFootWorldRotation(*swing))
            : 0.0f;
        if (continuousEnabled
            && comp._physicalStepPhase >= kTakeoff
            && comp._physicalStepPhase <= kTouchdownWait) {
            comp._gaitStepMaxSoleErrorDeg = glm::max(
                comp._gaitStepMaxSoleErrorDeg,
                comp._gaitSoleAngularErrorDeg);
            if (swingAngularVelocityOk) {
                comp._gaitStepMaxSwingAngularSpeed = glm::max(
                    comp._gaitStepMaxSwingAngularSpeed,
                    glm::length(swingAngularVelocity));
            }
        }

        auto planFoothold = [&]() {
            auto& turnPlan = comp._gaitTurnPlan;
            const bool swingFootLeft = swing == &comp._legL;
            const float swingSide = swingFootLeft ? -1.0f : 1.0f;
            glm::vec3 committedForward = turnPlan.committedForward;
            committedForward.y = 0.0f;
            if (glm::dot(committedForward, committedForward) < 1e-8f)
                committedForward = comp._physicalStepForward;
            committedForward = glm::normalize(committedForward);

            glm::vec3 desiredForward = turnPlan.desiredForward;
            desiredForward.y = 0.0f;
            if (glm::dot(desiredForward, desiredForward) < 1e-8f)
                desiredForward = committedForward;
            desiredForward = glm::normalize(desiredForward);

            const float desiredHeadingError = resolvedHeadingDelta(
                committedForward, desiredForward);
            const float maximumStepYaw = glm::radians(glm::clamp(
                comp.gaitMaxTurnStepDeg, 0.0f, 45.0f));
            const float requestedStepYaw = glm::clamp(
                desiredHeadingError, -maximumStepYaw, maximumStepYaw);
            const bool turnExitBlendApplied = continuousEnabled
                && comp._gaitTurnExitBlendPending
                && !comp._gaitStopRequested
                && std::abs(requestedStepYaw) <= 1e-6f;
            const bool requestedOutsideFoot =
                swingSide * requestedStepYaw > 1e-6f;
            const bool turnInitiationDeferred = continuousEnabled
                && !comp._gaitTurnExitBlendPending
                && std::abs(requestedStepYaw) > 1e-6f
                && !requestedOutsideFoot;
            const bool pairDirectionMatches =
                comp._gaitTurnPairPendingInside
                && requestedStepYaw * comp._gaitTurnPairYawSign > 1e-6f;
            if (comp._gaitTurnPairPendingInside
                && (!pairDirectionMatches || requestedOutsideFoot
                    || std::abs(requestedStepYaw) <= 1e-6f)) {
                // A live retarget, stop, or unexpected role order invalidates the
                // outside-to-inside contract.  Never carry it into another turn.
                comp._gaitTurnPairPendingInside = false;
                comp._gaitTurnPairAdvanceScale = 1.0f;
                comp._gaitTurnPairYawScale = 1.0f;
                comp._gaitTurnPairYawSign = 0.0f;
            }
            const bool pairBudgetApplied =
                comp._gaitTurnPairPendingInside
                && pairDirectionMatches && !requestedOutsideFoot;
            const float pairAdvanceScale = pairBudgetApplied
                ? glm::clamp(comp._gaitTurnPairAdvanceScale, 0.25f, 1.0f)
                : 1.0f;
            const float pairYawScale = pairBudgetApplied
                ? glm::clamp(comp._gaitTurnPairYawScale, 0.55f, 1.0f)
                : 1.0f;
            // A fresh turn begins on the outside foot. The inside foot cannot become the
            // first yaw-bearing step: both observed cold turn starts in that role aborted
            // during ARRIVAL, while the outside-foot retry completed the same requested
            // arc. Do not turn this scheduling step into an ordinary forward stride,
            // though; it exists only to exchange support roles in place.
            const float pairRequestedStepYaw = turnInitiationDeferred
                ? 0.0f
                : requestedStepYaw * pairYawScale;
            const bool controllerYawLimited =
                std::abs(desiredHeadingError)
                    > maximumStepYaw + glm::radians(0.01f);
            const float startYaw = std::atan2(
                -committedForward.x, -committedForward.z);
            const glm::quat startRotation = glm::angleAxis(
                startYaw, glm::vec3(0.0f, 1.0f, 0.0f));
            // Sole leveling follows a smoothstep, whose peak derivative is 1.5 times
            // its average. Measure the complete released-sole rotation, not just this
            // step's heading delta: each anatomical foot normally carries two committed
            // yaw increments by the time it becomes the swing foot again.
            const float swingAngularSpeedLimit = glm::clamp(
                comp.gaitTurnAngularSpeedLimit, 0.25f, 8.0f);
            const float soleLevelTime = glm::max(
                comp.gaitSoleLevelTime, 0.10f);
            auto requiredSoleAngularSpeed = [&](float stepYaw) {
                const glm::quat candidateHeading = glm::normalize(
                    glm::angleAxis(stepYaw, glm::vec3(0.0f, 1.0f, 0.0f))
                    * startRotation);
                const glm::quat candidateFootRotation =
                    footWorldRotationForHeading(*swing, candidateHeading);
                return 1.5f * glm::radians(rotationDifferenceDeg(
                    swing->plantedFootWorldRotation,
                    candidateFootRotation)) / soleLevelTime;
            };
            const float requestedAngularSpeed =
                requiredSoleAngularSpeed(requestedStepYaw);
            const float pairRequestedAngularSpeed =
                requiredSoleAngularSpeed(pairRequestedStepYaw);
            float admittedStepYaw = pairRequestedStepYaw;
            const bool angularSpeedLimited =
                pairRequestedAngularSpeed
                    > swingAngularSpeedLimit + 0.001f;
            if (angularSpeedLimited) {
                float admittedScale = 0.0f;
                float rejectedScale = 1.0f;
                if (requiredSoleAngularSpeed(0.0f)
                    <= swingAngularSpeedLimit) {
                    for (int iteration = 0; iteration < 12; ++iteration) {
                        const float candidateScale =
                            0.5f * (admittedScale + rejectedScale);
                        if (requiredSoleAngularSpeed(
                                pairRequestedStepYaw * candidateScale)
                            <= swingAngularSpeedLimit) {
                            admittedScale = candidateScale;
                        } else {
                            rejectedScale = candidateScale;
                        }
                    }
                }
                admittedStepYaw = pairRequestedStepYaw * admittedScale;
            }
            const float headingAdvanceScale = turnInitiationDeferred
                ? 0.0f
                : (std::abs(admittedStepYaw) > 1e-6f
                    ? turnAdvanceScaleForHeadingError(desiredHeadingError)
                    : 1.0f);
            glm::quat midRotation = glm::normalize(
                glm::angleAxis(0.5f * admittedStepYaw,
                               glm::vec3(0.0f, 1.0f, 0.0f))
                * startRotation);
            glm::quat endRotation = glm::normalize(
                glm::angleAxis(admittedStepYaw,
                               glm::vec3(0.0f, 1.0f, 0.0f))
                * startRotation);
            auto basisFromRotation = [&](const glm::quat& rotation,
                                         glm::vec3& right,
                                         glm::vec3& forward) {
                forward = horizontalForward(rotation);
                right = glm::cross(
                    forward, glm::vec3(0.0f, 1.0f, 0.0f));
                makeHorizontalBasis(right, forward);
            };

            turnPlan.desiredHeadingError = desiredHeadingError;
            turnPlan.activeStartForward = committedForward;
            basisFromRotation(
                startRotation, turnPlan.activeStartRight,
                turnPlan.activeStartForward);
            basisFromRotation(
                midRotation, turnPlan.activeMidRight,
                turnPlan.activeMidForward);
            basisFromRotation(
                endRotation, turnPlan.activeEndRight,
                turnPlan.activeEndForward);
            auto plannedAdvanceCoefficient = [&](glm::vec3 delta) {
                delta.y = 0.0f;
                if (std::abs(admittedStepYaw) <= 1e-6f)
                    return glm::dot(delta, comp._physicalStepForward);
                const float denominator = glm::max(glm::dot(
                    turnPlan.activeMidForward,
                    turnPlan.activeEndForward), 1e-4f);
                return glm::dot(delta, turnPlan.activeEndForward)
                    / denominator;
            };
            auto plannedLaneCoefficient = [&](glm::vec3 delta) {
                delta.y = 0.0f;
                if (std::abs(admittedStepYaw) <= 1e-6f)
                    return glm::dot(delta, comp._physicalStepRight);
                const float denominator = glm::max(glm::dot(
                    turnPlan.activeMidRight,
                    turnPlan.activeEndRight), 1e-4f);
                return glm::dot(delta, turnPlan.activeMidRight)
                    / denominator;
            };
            turnPlan.activeStartRotation = startRotation;
            turnPlan.activeEndRotation = endRotation;
            turnPlan.requestedYaw = requestedStepYaw;
            turnPlan.admittedYaw = admittedStepYaw;
            turnPlan.achievedYaw = 0.0f;
            turnPlan.plannedTurnProgress = 0.0f;
            turnPlan.achievedTurnProgress = 0.0f;
            turnPlan.nominalAdvance = 0.0f;
            turnPlan.headingAdvanceScale = headingAdvanceScale;
            turnPlan.requestedAdvance = 0.0f;
            turnPlan.admittedAdvance = 0.0f;
            turnPlan.achievedAdvance = 0.0f;
            turnPlan.objective = Comp::TurnStepObjective::None;
            turnPlan.translationObjectiveSatisfied = false;
            turnPlan.angularObjectiveSatisfied = false;
            turnPlan.planOrigin = stanceFoot;
            turnPlan.candidateEvaluated = false;
            turnPlan.candidateAccepted = false;
            turnPlan.activeHeadingPlan = false;
            turnPlan.stanceFootRotationAtAdmission =
                glm::normalize(stance->plantedFootWorldRotation);
            turnPlan.stanceReferenceRotationDeltaDeg = 0.0f;
            turnPlan.plantReferenceRotationDeltaDeg = 0.0f;
            turnPlan.touchdownSoleErrorDeg = 0.0f;
            turnPlan.touchdownAngularSpeed = 0.0f;
            turnPlan.touchdownHorizontalSpeed = 0.0f;
            turnPlan.requestedSwingDistance = 0.0f;
            turnPlan.admittedSwingDistance = 0.0f;
            turnPlan.requiredSwingSpeed = 0.0f;
            turnPlan.admittedSwingSpeed = 0.0f;
            turnPlan.swingSpeedLimit = 0.0f;
            turnPlan.minimumLaneSwingSpeed = 0.0f;
            turnPlan.swingSpeedClosureTolerance = 0.0f;
            turnPlan.requiredAngularSpeed = requestedAngularSpeed;
            turnPlan.admittedAngularSpeed =
                requiredSoleAngularSpeed(admittedStepYaw);
            turnPlan.angularSpeedLimit = swingAngularSpeedLimit;
            turnPlan.achievedSwingSpeed = 0.0f;
            turnPlan.achievedAngularSpeed = 0.0f;
            turnPlan.predictedContactHip = glm::vec3(0.0f);
            turnPlan.actualContactHip = glm::vec3(0.0f);
            turnPlan.contactHipError = glm::vec3(0.0f);
            turnPlan.predictedContactSupportVelocity = glm::vec3(0.0f);
            turnPlan.sampledAnkleClampDeg = 0.0f;
            turnPlan.admittedKneeSwivelDeg = 0.0f;
            turnPlan.shadowSwing = {};
            turnPlan.shadowStance = {};
            turnPlan.shadowLimitingConstraint =
                Comp::TurnConstraintResult::NotEvaluated;
            turnPlan.shadowRequestedSwingSpeed = 0.0f;
            turnPlan.shadowAdmittedSwingSpeed = 0.0f;
            turnPlan.shadowSwingSpeedLimit = 0.0f;
            turnPlan.shadowSwingSpeedClampLoss = 0.0f;
            turnPlan.shadowSwingSpeedClampReference = 0.0f;
            turnPlan.shadowFeasibilityEvaluated = false;
            turnPlan.shadowFeasibilitySafe = false;
            turnPlan.shadowRuntimeAccepted = false;
            turnPlan.swingFootLeft = swingFootLeft;
            turnPlan.outsideFoot = swingSide * admittedStepYaw > 1e-6f;
            turnPlan.pairAdvanceScale = pairAdvanceScale;
            turnPlan.pairYawScale = pairYawScale;
            turnPlan.supportTransportSpeed = 0.0f;
            turnPlan.arrivalTrajectoryT = 0.70f;
            turnPlan.pairBudgetApplied = pairBudgetApplied;
            turnPlan.pairBudgetLatched = false;
            turnPlan.swingSpeedLaneFloorApplied = false;
            turnPlan.swingSpeedLaneFloorExceeded = false;
            turnPlan.turnInitiationDeferred = turnInitiationDeferred;
            turnPlan.turnExitBlendApplied = turnExitBlendApplied;
            turnPlan.touchdownReadinessBlockedLogged = false;
            turnPlan.limitingConstraint = pairBudgetApplied
                ? Comp::TurnConstraintResult::TurnPairBudget
                : (turnInitiationDeferred
                ? Comp::TurnConstraintResult::TurnInitiationRole
                : (angularSpeedLimited
                ? Comp::TurnConstraintResult::SwingAngularSpeed
                : (controllerYawLimited
                    ? Comp::TurnConstraintResult::ControllerYawLimit
                    : Comp::TurnConstraintResult::NotEvaluated)));

            if (continuousEnabled
                && ((pairBudgetApplied
                     && std::abs(admittedStepYaw) > 1e-6f)
                    || turnExitBlendApplied)) {
                // The outside step may have spent most of its translational budget
                // reaching the far lane.  Decelerate the following inside step's
                // incoming pelvis transport C1-continuously before predicting its hip.
                // The first zero-yaw step after a completed turn uses the same bounded
                // transport so it cannot jump directly back to straight-gait cadence.
                glm::vec3 incomingVelocity =
                    comp._gaitSupportCommandVelocity;
                incomingVelocity.y = 0.0f;
                const float incomingSpeed = glm::length(incomingVelocity);
                if (incomingSpeed > configuredSupportMaxSpeed
                    && incomingSpeed > 1e-6f) {
                    incomingVelocity *=
                        configuredSupportMaxSpeed / incomingSpeed;
                }
                constexpr float kInsideSupportTransportSpeed = 0.05f;
                const glm::vec3 outgoingVelocity =
                    turnPlan.activeMidForward
                    * glm::min(kInsideSupportTransportSpeed,
                               configuredSupportMaxSpeed)
                    * headingAdvanceScale;
                comp._gaitSupportCurveActive = true;
                comp._gaitSupportCurveStart =
                    comp._physicalStepSupportTarget;
                comp._gaitSupportCurveStartVelocity = incomingVelocity;
                comp._gaitSupportCurveTime = 0.0f;
                comp._gaitSupportCurveDuration = glm::max(
                    0.10f + cadenceSwingTime
                        + cadenceArrivalSettleTime,
                    0.40f);
                comp._gaitSupportCurveEnd =
                    comp._gaitSupportCurveStart
                    + outgoingVelocity * comp._gaitSupportCurveDuration;
                comp._gaitSupportCurveEnd.y =
                    comp._gaitSupportCurveStart.y;
                comp._gaitSupportCurveEndVelocity = outgoingVelocity;
                turnPlan.supportTransportSpeed =
                    glm::length(outgoingVelocity);
                if (turnExitBlendApplied) {
                    spdlog::info(
                        "[LocomotionGait] SUPPORT_TRANSPORT_TURN_EXIT step={} "
                        "swing={} duration={:.3f}s "
                        "velocity=({:+.3f},{:+.3f})->({:+.3f},{:+.3f})mps",
                        comp._stepSequenceStepIndex,
                        swingFootLeft ? "LEFT" : "RIGHT",
                        comp._gaitSupportCurveDuration,
                        incomingVelocity.x, incomingVelocity.z,
                        outgoingVelocity.x, outgoingVelocity.z);
                } else {
                    spdlog::info(
                        "[LocomotionGait] SUPPORT_TRANSPORT_PAIR step={} "
                        "swing={} role=inside duration={:.3f}s "
                        "pair=(advance={:.3f},yaw={:.3f}) "
                        "velocity=({:+.3f},{:+.3f})->({:+.3f},{:+.3f})mps",
                        comp._stepSequenceStepIndex,
                        swingFootLeft ? "LEFT" : "RIGHT",
                        comp._gaitSupportCurveDuration,
                        pairAdvanceScale, pairYawScale,
                        incomingVelocity.x, incomingVelocity.z,
                        outgoingVelocity.x, outgoingVelocity.z);
                }
            }

            const float minimumAdvance = glm::min(
                comp.gaitMinStepLength, comp.gaitMaxStepLength);
            const float maximumAdvance = glm::max(
                comp.gaitMinStepLength, comp.gaitMaxStepLength);
            // The runtime gait only needs a modest admission margin here. The completed step is
            // still required to satisfy the independent minimum support advance below.
            const float baseTrackingReserve = 0.010f;
            const float trackingReserve = continuousEnabled
                ? glm::min(glm::max(
                    baseTrackingReserve,
                    comp._gaitSettledTrackingLoss + 0.003f), 0.040f)
                : 0.0f;
            const float minimumCommand = glm::min(
                maximumAdvance, minimumAdvance + trackingReserve);
            const float nominalPlacementDistance = continuousEnabled
                ? glm::clamp(comp._gaitCommandedStepLength,
                    minimumCommand, maximumAdvance)
                : glm::clamp(comp.stepLength, 0.15f, 0.25f);
            const float placementDistance = continuousEnabled
                ? nominalPlacementDistance * headingAdvanceScale
                : nominalPlacementDistance;
            turnPlan.nominalAdvance = nominalPlacementDistance;
            glm::vec3 requestedTarget;
            float lateralLane = 0.0f;
            bool footSeparationLimited = false;
            if (continuousEnabled) {
                // A limit cycle places the next support relative to the current support,
                // not relative to where this swing foot happened to land one cycle ago.
                // Gameplay turns give each anatomical foot a signed lane in the new frame;
                // a raw projection can change sign during a 90/180-degree reframe and ask
                // the swing leg to cross through the planted leg.
                lateralLane = glm::dot(
                    comp._physicalStepSwingStart - stanceFoot, comp._physicalStepRight);
                if (gameplayCommand) {
                    constexpr float kMinimumRuntimeLane = 0.10f;
                    const float requestedLaneMagnitude = glm::max(
                        std::abs(lateralLane), kMinimumRuntimeLane);
                    float requestedLane = swingSide * requestedLaneMagnitude;
                    requestedLane += glm::clamp(
                        comp._gaitAdaptiveLateralOffset, -0.015f, 0.015f);
                    const float maximumRuntimeLane = glm::max(
                        comp.gaitMaxFootSeparation, kMinimumRuntimeLane);
                    turnPlan.rawFootSeparation = std::abs(requestedLane);
                    turnPlan.maximumFootSeparation = maximumRuntimeLane;
                    footSeparationLimited =
                        turnPlan.rawFootSeparation > maximumRuntimeLane + 1e-5f;
                    lateralLane = swingSide * glm::clamp(
                        std::abs(requestedLane),
                        kMinimumRuntimeLane, maximumRuntimeLane);
                    turnPlan.footSeparationLimited = footSeparationLimited;
                }
                requestedTarget = stanceFoot
                    + turnPlan.activeMidForward * placementDistance
                    + turnPlan.activeEndRight * lateralLane;
            } else {
                requestedTarget = comp._physicalStepSwingStart
                    + comp._physicalStepForward * placementDistance;
            }
            bool angularObjective =
                std::abs(admittedStepYaw) > 1e-6f;
            bool turnConditionedBudget = angularObjective;
            const float plannedCadenceDescentTime = turnConditionedBudget
                ? glm::max(cadenceDescentTime, kTurnCadenceDescentFloor)
                : cadenceDescentTime;
            // Turn admission must use the recovered whole-step smoothstep rather than
            // the front-loaded straight-walk curve. The old 1.05/0.63 coefficients are
            // the peak swing/descent derivatives for that trajectory partition.
            const float swingPathPeakCoefficient = turnConditionedBudget
                ? glm::max(
                    1.05f / glm::max(cadenceSwingTime, 0.01f),
                    0.63f / glm::max(plannedCadenceDescentTime, 0.01f))
                : glm::max(
                    (1.5f * kWalkingSwingHorizontalAtArrival)
                        / glm::max(cadenceSwingTime, 0.01f),
                    (1.5f * (1.0f - kWalkingSwingHorizontalAtArrival))
                        / glm::max(cadenceDescentTime
                            * ((kWalkingSwingLandingBrakeT - 0.70f)
                                / (1.0f - 0.70f)), 0.01f));
            auto requiredSwingTargetSpeed = [&](const glm::vec3& candidate) {
                return horizontalDistance(
                    candidate, comp._physicalStepSwingStart)
                    * swingPathPeakCoefficient;
            };
            // The outer sole travels the longer arc around the turn center and receives
            // the more conservative turn budget. Straight gait uses its own physical
            // swing limit so shortening the cadence cannot admit an unreachable foothold.
            const float insideTurnSwingSpeedLimit = glm::clamp(
                comp.gaitTurnInsideSwingSpeedLimit, 0.10f, 2.0f);
            const float outsideTurnSwingSpeedLimit = glm::clamp(
                comp.gaitTurnOutsideSwingSpeedLimit, 0.10f, 2.0f);
            const float straightSwingSpeedLimit = glm::clamp(
                comp.gaitSwingSpeedLimit, 0.25f, 4.0f);
            auto configuredSwingSpeedLimit = [&]() {
                return turnConditionedBudget
                    ? (turnPlan.outsideFoot
                        ? outsideTurnSwingSpeedLimit
                        : insideTurnSwingSpeedLimit)
                    : straightSwingSpeedLimit;
            };
            float swingSpeedLimit = configuredSwingSpeedLimit();
            constexpr float kSwingSpeedClosureTolerance = 0.005f;
            const float requestedSwingSpeed =
                requiredSwingTargetSpeed(requestedTarget);
            const float laneFloorYawBefore = admittedStepYaw;
            const float laneFloorSpeedBefore = continuousEnabled
                && turnConditionedBudget
                ? requiredSwingTargetSpeed(
                    stanceFoot + turnPlan.activeEndRight * lateralLane)
                : 0.0f;
            bool laneFloorYawReduced = false;

            // Advance scaling cannot shorten the anatomical lane. If an outside turn
            // reaches that floor, descend yaw and rebuild the heading-relative footprint
            // before grounding, reach, support prediction, or shadow feasibility run.
            // This realizes the previously diagnostic-only "await-descending-yaw" path
            // without widening the accepted target-speed closure.
            if (continuousEnabled && angularObjective && turnPlan.outsideFoot
                && laneFloorSpeedBefore
                    > swingSpeedLimit + kSwingSpeedClosureTolerance) {
                auto laneFloorFitsAtYaw = [&](float candidateYaw) {
                    const glm::quat candidateEndRotation = glm::normalize(
                        glm::angleAxis(candidateYaw,
                                       glm::vec3(0.0f, 1.0f, 0.0f))
                        * startRotation);
                    glm::vec3 candidateRight(0.0f), candidateForward(0.0f);
                    basisFromRotation(
                        candidateEndRotation,
                        candidateRight, candidateForward);
                    const glm::vec3 laneTarget =
                        stanceFoot + candidateRight * lateralLane;
                    return requiredSwingTargetSpeed(laneTarget)
                            <= swingSpeedLimit
                                + kSwingSpeedClosureTolerance
                        && requiredSoleAngularSpeed(candidateYaw)
                            <= swingAngularSpeedLimit + 0.001f;
                };

                float admittedYawScale = 0.0f;
                float rejectedYawScale = 1.0f;
                const bool zeroYawFits = laneFloorFitsAtYaw(0.0f);
                if (zeroYawFits) {
                    for (int iteration = 0; iteration < 12; ++iteration) {
                        const float candidateScale =
                            0.5f * (admittedYawScale + rejectedYawScale);
                        if (laneFloorFitsAtYaw(
                                laneFloorYawBefore * candidateScale)) {
                            admittedYawScale = candidateScale;
                        } else {
                            rejectedYawScale = candidateScale;
                        }
                    }
                }

                // Zero yaw is a valid result: it produces a stable translation recovery
                // step and leaves the heading request pending for the next foot. The old
                // strict-positive test kept the original infeasible yaw and aborted.
                if (zeroYawFits && admittedYawScale < 0.9999f) {
                    admittedStepYaw = laneFloorYawBefore * admittedYawScale;
                    midRotation = glm::normalize(
                        glm::angleAxis(0.5f * admittedStepYaw,
                                       glm::vec3(0.0f, 1.0f, 0.0f))
                        * startRotation);
                    endRotation = glm::normalize(
                        glm::angleAxis(admittedStepYaw,
                                       glm::vec3(0.0f, 1.0f, 0.0f))
                        * startRotation);
                    basisFromRotation(
                        midRotation, turnPlan.activeMidRight,
                        turnPlan.activeMidForward);
                    basisFromRotation(
                        endRotation, turnPlan.activeEndRight,
                        turnPlan.activeEndForward);
                    turnPlan.activeEndRotation = endRotation;
                    turnPlan.admittedYaw = admittedStepYaw;
                    turnPlan.admittedAngularSpeed =
                        requiredSoleAngularSpeed(admittedStepYaw);
                    turnPlan.outsideFoot =
                        swingSide * admittedStepYaw > 1e-6f;
                    angularObjective =
                        std::abs(admittedStepYaw) > 1e-6f;
                    turnConditionedBudget = angularObjective;
                    swingSpeedLimit = configuredSwingSpeedLimit();
                    requestedTarget = stanceFoot
                        + turnPlan.activeMidForward * placementDistance
                        + turnPlan.activeEndRight * lateralLane;
                    laneFloorYawReduced = true;
                }
            }
            const HitResult startGround = Physics::Raycast(
                comp._physicalStepSwingStart + glm::vec3(0, 0.25f, 0),
                glm::vec3(0, -1, 0), 0.75f, entity, false);
            const HitResult targetGround = Physics::Raycast(
                requestedTarget + glm::vec3(0, 0.35f, 0),
                glm::vec3(0, -1, 0), 1.0f, entity, false);
            const float soleCenterOffset = startGround.hit
                ? comp._physicalStepSwingStart.y - startGround.point.y : 0.0f;
            requestedTarget.y = targetGround.hit
                ? targetGround.point.y + soleCenterOffset : comp._physicalStepSwingStart.y;
            turnPlan.candidateGrounded = targetGround.hit;

            const glm::vec3 hip = physicalPosition(swing->hipIdx);
            // Simulate the same curve-following velocity and acceleration limiter used by
            // the runtime support controller.  This is intentionally candidate-dependent:
            // the late descent preload is built from the support position and incoming
            // velocity predicted at the instant that curve will actually be seeded.
            auto predictLandingHip = [&](const glm::vec3& candidate,
                                         glm::vec3* contactSupportVelocity) {
                if (!continuousEnabled) {
                    if (contactSupportVelocity)
                        *contactSupportVelocity = glm::vec3(0.0f);
                    return hip;
                }

                glm::vec3 simulatedPosition =
                    comp._physicalStepSupportTarget;
                glm::vec3 simulatedVelocity =
                    comp._gaitSupportCommandVelocity;
                simulatedVelocity.y = 0.0f;
                constexpr float kPredictionStep = 1.0f / 120.0f;
                constexpr float kSupportPositionGain = 4.0f;
                auto advanceSupportPrediction = [&](const glm::vec3& curvePosition,
                                                    const glm::vec3& curveVelocity,
                                                    float stepTime) {
                    glm::vec3 desiredVelocity = curveVelocity
                        + (curvePosition - simulatedPosition)
                            * kSupportPositionGain;
                    desiredVelocity.y = 0.0f;
                    const float desiredSpeed = glm::length(desiredVelocity);
                    if (desiredSpeed > configuredSupportMaxSpeed
                        && desiredSpeed > 1e-6f) {
                        desiredVelocity *=
                            configuredSupportMaxSpeed / desiredSpeed;
                    }
                    float maximumAcceleration = 1.75f;
                    if (glm::length(desiredVelocity)
                        < glm::length(simulatedVelocity)) {
                        maximumAcceleration = 2.50f;
                    }
                    glm::vec3 velocityDelta =
                        desiredVelocity - simulatedVelocity;
                    const float maximumDelta =
                        maximumAcceleration * stepTime;
                    const float deltaLength = glm::length(velocityDelta);
                    if (deltaLength > maximumDelta && deltaLength > 1e-6f)
                        velocityDelta *= maximumDelta / deltaLength;
                    simulatedVelocity += velocityDelta;
                    simulatedPosition += simulatedVelocity * stepTime;
                    simulatedPosition.y =
                        comp._physicalStepSupportTarget.y;
                };

                // TAKEOFF is contact-driven rather than duration-driven.  The accepted
                // gait normally opens the sole in about 100 ms; retaining that explicit
                // estimate is safer than pretending descent begins at plan admission.
                constexpr float kPredictedTakeoffTime = 0.10f;
                const float timeToDescent = kPredictedTakeoffTime
                    + cadenceSwingTime + cadenceArrivalSettleTime;
                for (float elapsed = 0.0f; elapsed < timeToDescent;) {
                    const float stepTime = glm::min(
                        kPredictionStep, timeToDescent - elapsed);
                    glm::vec3 curvePosition = simulatedPosition;
                    glm::vec3 curveVelocity = simulatedVelocity;
                    if (comp._gaitSupportCurveActive) {
                        evaluateActiveSupportCurveAt(
                            comp._gaitSupportCurveTime + elapsed + stepTime,
                            curvePosition, curveVelocity);
                    } else {
                        curvePosition += curveVelocity * stepTime;
                    }
                    advanceSupportPrediction(
                        curvePosition, curveVelocity, stepTime);
                    elapsed += stepTime;
                }

                const glm::vec3 preloadStart = simulatedPosition;
                const glm::vec3 preloadStartVelocity = simulatedVelocity;
                const float transferFraction = glm::clamp(
                    comp.transferSupportBias
                        + comp._gaitAdaptiveTransferBiasOffset,
                    kNewSupportLoadCommandTarget, 0.98f);
                const glm::vec3 fullTransferTarget = glm::mix(
                    stanceFoot, candidate, transferFraction);
                constexpr float kPreContactSupportFraction = 0.20f;
                glm::vec3 preloadEnd = glm::mix(
                    preloadStart, fullTransferTarget,
                    kPreContactSupportFraction);
                preloadEnd.y = preloadStart.y;
                const glm::vec3 preloadEndVelocity =
                    supportCurveEndVelocityForObjective(
                        preloadStart, preloadEnd, plannedCadenceDescentTime,
                        std::abs(admittedStepYaw) > 1e-6f);
                for (float elapsed = 0.0f;
                     elapsed < plannedCadenceDescentTime;) {
                    const float stepTime = glm::min(
                        kPredictionStep,
                        plannedCadenceDescentTime - elapsed);
                    glm::vec3 curvePosition(0.0f), curveVelocity(0.0f);
                    evaluateSupportCurveSegment(
                        preloadStart, preloadStartVelocity,
                        preloadEnd, preloadEndVelocity,
                        plannedCadenceDescentTime, elapsed + stepTime,
                        curvePosition, curveVelocity);
                    advanceSupportPrediction(
                        curvePosition, curveVelocity, stepTime);
                    elapsed += stepTime;
                }

                if (contactSupportVelocity)
                    *contactSupportVelocity = simulatedVelocity;
                glm::vec3 predictedSupportShift =
                    simulatedPosition - rag._locomotionCOM;
                predictedSupportShift.y = 0.0f;
                const glm::quat contactHeading = glm::normalize(glm::slerp(
                    startRotation, endRotation, 0.80f));
                const glm::quat headingDelta = glm::normalize(
                    contactHeading * glm::conjugate(startRotation));
                glm::vec3 hipFromCom = hip - rag._locomotionCOM;
                hipFromCom.y = 0.0f;
                glm::vec3 headingHipShift =
                    headingDelta * hipFromCom - hipFromCom;
                headingHipShift.y = 0.0f;
                return hip + predictedSupportShift + headingHipShift;
            };
            const float legLength = swing->segmentGeometryValid
                ? glm::length(swing->upperSegmentLocal)
                    + glm::length(swing->lowerSegmentLocal)
                : glm::length(skeleton.bones[swing->kneeIdx].localT)
                    + glm::length(skeleton.bones[swing->ankleIdx].localT);
            const float configuredReach = legLength
                * glm::clamp(comp.maxLegReachFraction, 0.70f, 0.99f);
            const float currentReach = glm::length(
                comp._physicalStepSwingStart
                    + ankleFromFootWorld(
                        *swing, swing->plantedFootWorldRotation) - hip);
            const float safeReachFraction = continuousEnabled
                ? glm::min(comp.safeReachFraction,
                           comp.gaitUsableReachFraction)
                : comp.safeReachFraction;
            const float safeAnatomicalReach = legLength * glm::clamp(
                safeReachFraction, 0.94f, 0.995f);
            const float antiSingularityCeiling = legLength * 0.995f;
            // The current physical stance is necessarily reachable. Tests 4-6 may begin
            // with an almost straight knee, so their captured reach can legitimately exceed
            // the generic anti-singularity fraction. Continuous gait instead couples its
            // small pelvis crouch with a conservative usable fraction; retaining knee bend
            // avoids magnifying millimetres of radial motor lag into centimetres of forward
            // landing error. Never shorten a captured physical pose just to satisfy the cap.
            comp._physicalStepReachLimit = glm::min(
                glm::max(configuredReach,
                         glm::max(currentReach, safeAnatomicalReach)),
                glm::max(currentReach, antiSingularityCeiling));

            auto groundCandidatePosition = [&](glm::vec3 candidate,
                                                float fallbackY) {
                const HitResult ground = Physics::Raycast(
                    candidate + glm::vec3(0, 0.35f, 0),
                    glm::vec3(0, -1, 0), 1.0f, entity, false);
                candidate.y = ground.hit
                    ? ground.point.y + soleCenterOffset : fallbackY;
                return candidate;
            };
            auto targetAtAdvanceScale = [&](float advanceScale) {
                if (!continuousEnabled)
                    return glm::mix(
                        comp._physicalStepSwingStart, requestedTarget,
                        glm::clamp(advanceScale, 0.0f, 1.0f));
                glm::vec3 candidate = stanceFoot
                    + turnPlan.activeMidForward
                        * placementDistance
                        * glm::clamp(advanceScale, 0.0f, 1.0f)
                    + turnPlan.activeEndRight * lateralLane;
                return groundCandidatePosition(candidate, requestedTarget.y);
            };
            // Scaling advance cannot remove the anatomical lane displacement. Measure
            // that immutable floor separately so a millimetre-scale speed overshoot is
            // not mistaken for an unbounded plan. Larger floors remain rejected only if
            // the descending-yaw pass above cannot rebuild an admissible footprint.
            const float minimumLaneSwingSpeed = turnConditionedBudget
                ? requiredSwingTargetSpeed(targetAtAdvanceScale(0.0f))
                : 0.0f;
            turnPlan.minimumLaneSwingSpeed = minimumLaneSwingSpeed;
            turnPlan.swingSpeedClosureTolerance = continuousEnabled
                ? kSwingSpeedClosureTolerance : 0.0f;
            const float maximumDynamicAdvanceScale =
                continuousEnabled && turnConditionedBudget
                    ? pairAdvanceScale : 1.0f;
            glm::vec3 dynamicallyAdmittedTarget =
                targetAtAdvanceScale(maximumDynamicAdvanceScale);
            const bool pairAdvanceLimited =
                maximumDynamicAdvanceScale < 0.999f;
            bool swingSpeedLimited = false;
            if (continuousEnabled
                && requiredSwingTargetSpeed(dynamicallyAdmittedTarget)
                    > swingSpeedLimit) {
                float admittedScale = 0.0f;
                float rejectedScale = maximumDynamicAdvanceScale;
                for (int iteration = 0; iteration < 12; ++iteration) {
                    const float candidateScale =
                        0.5f * (admittedScale + rejectedScale);
                    const glm::vec3 candidate =
                        targetAtAdvanceScale(candidateScale);
                    if (requiredSwingTargetSpeed(candidate)
                        <= swingSpeedLimit) {
                        admittedScale = candidateScale;
                    } else {
                        rejectedScale = candidateScale;
                    }
                }
                dynamicallyAdmittedTarget =
                    targetAtAdvanceScale(admittedScale);
                swingSpeedLimited = true;
            }
            auto groundedCandidate = [&](float horizontalScale) {
                glm::vec3 candidate = glm::mix(
                    comp._physicalStepSwingStart, dynamicallyAdmittedTarget,
                    glm::clamp(horizontalScale, 0.0f, 1.0f));
                return groundCandidatePosition(
                    candidate,
                    glm::mix(comp._physicalStepSwingStart.y,
                             dynamicallyAdmittedTarget.y,
                             glm::clamp(horizontalScale, 0.0f, 1.0f)));
            };
            auto targetReach = [&](const glm::vec3& candidate) {
                const glm::quat candidateRotation =
                    footWorldRotationForHeading(*swing, endRotation);
                const glm::vec3 candidateAnkle = candidate
                    + ankleFromFootWorld(*swing, candidateRotation);
                // Admission must be safe both at the current support pose and at the
                // predicted contact pose. If support motion happens to move the hip closer,
                // it must not be used to justify a foothold that is unreachable before that
                // motion completes.
                const glm::vec3 candidateLandingHip =
                    predictLandingHip(candidate, nullptr);
                return glm::max(
                    glm::length(candidateAnkle - hip),
                    glm::length(candidateAnkle - candidateLandingHip));
            };

            glm::vec3 target = dynamicallyAdmittedTarget;
            const float requestedReach = targetReach(requestedTarget);
            bool reachClamped =
                targetReach(target) > comp._physicalStepReachLimit;
            if (reachClamped) {
                float reachableScale = 0.0f;
                float unreachableScale = 1.0f;
                for (int iteration = 0; iteration < 12; ++iteration) {
                    const float candidateScale =
                        0.5f * (reachableScale + unreachableScale);
                    const glm::vec3 candidate = groundedCandidate(candidateScale);
                    if (targetReach(candidate) <= comp._physicalStepReachLimit)
                        reachableScale = candidateScale;
                    else
                        unreachableScale = candidateScale;
                }
                target = groundedCandidate(reachableScale);
            }

            // Counterfactual Slice 2j sample. Retain its prediction so the log can compare
            // the hypothetical settled solve with the live transient, but never use it to
            // project, reject, or otherwise route a foothold.
            Envelope admissionHipEnvelope;
            admissionHipEnvelope.twistAxis = swing->hipTwistAxis;
            admissionHipEnvelope.swingNormalDeg = swing->hipSwingNormalDeg;
            admissionHipEnvelope.swingPlaneDeg = swing->hipSwingPlaneDeg;
            admissionHipEnvelope.twistMinDeg = swing->hipTwistMinDeg;
            admissionHipEnvelope.twistMaxDeg = swing->hipTwistMaxDeg;
            Envelope admissionAnkleEnvelope;
            admissionAnkleEnvelope.twistAxis = swing->ankleAxis;
            admissionAnkleEnvelope.swingNormalDeg = swing->ankleSwingNormalDeg;
            admissionAnkleEnvelope.swingPlaneDeg = swing->ankleSwingPlaneDeg;
            admissionAnkleEnvelope.twistMinDeg = swing->ankleTwistMinDeg;
            admissionAnkleEnvelope.twistMaxDeg = swing->ankleTwistMaxDeg;
            const glm::quat admissionParentWorld = ParentWorldRot(
                rag, skeleton, animator, entityWorld, swing->hipIdx);
            const glm::vec3 admissionUpperLocal =
                swing->segmentGeometryValid
                    ? swing->upperSegmentLocal
                    : skeleton.bones[swing->kneeIdx].localT;
            const glm::vec3 admissionLowerLocal =
                swing->segmentGeometryValid
                    ? swing->lowerSegmentLocal
                    : skeleton.bones[swing->ankleIdx].localT;
            const float admissionUpperLength = glm::length(
                admissionUpperLocal);
            const float admissionLowerLength = glm::length(
                admissionLowerLocal);
            struct AdmissionOrientationResult {
                float maximumClampDeg = 0.0f;
                float contactSwivelDeg = 0.0f;
                float maximumClosure = 0.0f;
                bool allSamplesClosed = true;
            };
            auto sampleOrientationFeasibility = [&](const glm::vec3& foothold) {
                AdmissionOrientationResult result;
                const glm::quat footRotation =
                    footWorldRotationForHeading(*swing, endRotation);
                const glm::vec3 landingHip =
                    predictLandingHip(foothold, nullptr);
                constexpr float kLateSwingSamples[3] = {
                    0.60f, 0.80f, 1.00f
                };
                for (float trajectoryT : kLateSwingSamples) {
                    glm::vec3 sampledFoot = glm::mix(
                        comp._physicalStepSwingStart, foothold,
                        trajectoryT);
                    sampledFoot.y += std::sin(glm::pi<float>() * trajectoryT)
                        * comp.swingHeight;
                    const glm::vec3 sampledHip = glm::mix(
                        hip, landingHip, trajectoryT);
                    const glm::vec3 sampledAnkle = sampledFoot
                        + ankleFromFootWorld(*swing, footRotation);
                    const glm::vec3 toSample = sampledAnkle - sampledHip;
                    const float sampledDistance = glm::length(toSample);
                    if (sampledDistance <= 1e-4f
                        || sampledDistance
                            >= admissionUpperLength
                                + admissionLowerLength) {
                        result.maximumClampDeg = glm::max(
                            result.maximumClampDeg, 180.0f);
                        continue;
                    }
                    const float includedCos = glm::clamp(
                        (admissionUpperLength * admissionUpperLength
                         + admissionLowerLength * admissionLowerLength
                         - sampledDistance * sampledDistance)
                            / (2.0f * admissionUpperLength
                                * admissionLowerLength),
                        -1.0f, 1.0f);
                    const float kneeBend = glm::pi<float>()
                        - std::acos(includedCos);
                    const glm::quat kneeLocal = glm::normalize(
                        swing->referenceKneeLocal
                        * glm::angleAxis(
                            kneeBend - swing->referenceKneeBend,
                            glm::normalize(swing->kneeHingeAxis)));
                    const glm::vec3 sampleForward = glm::normalize(toSample);
                    const glm::vec3 kneePole = nominalKneePoleWorld(*swing);
                    glm::vec3 sampleBend = kneePole
                        - sampleForward
                            * glm::dot(kneePole, sampleForward);
                    if (glm::dot(sampleBend, sampleBend) < 1e-8f)
                        sampleBend = comp._physicalStepForward;
                    sampleBend -= sampleForward
                        * glm::dot(sampleBend, sampleForward);
                    if (glm::dot(sampleBend, sampleBend) < 1e-8f)
                        sampleBend = comp._physicalStepRight;
                    else
                        sampleBend = glm::normalize(sampleBend);
                    const float hipCosine = glm::clamp(
                        (admissionUpperLength * admissionUpperLength
                         + sampledDistance * sampledDistance
                         - admissionLowerLength * admissionLowerLength)
                            / (2.0f * admissionUpperLength
                                * sampledDistance),
                        -1.0f, 1.0f);
                    const float hipSine = std::sqrt(glm::max(
                        1.0f - hipCosine * hipCosine, 0.0f));
                    const glm::quat sampledParentWorld = glm::normalize(
                        glm::angleAxis(
                            admittedStepYaw * trajectoryT,
                            glm::vec3(0.0f, 1.0f, 0.0f))
                        * admissionParentWorld);
                    float bestCost = 1e9f;
                    float bestClamp = 180.0f;
                    float bestSwivel = 0.0f;
                    float bestClosure = 1.0f;
                    for (int swivelSample = 0;
                         swivelSample < 25; ++swivelSample) {
                        const float swivel = glm::mix(
                            glm::radians(-60.0f),
                            glm::radians(60.0f),
                            static_cast<float>(swivelSample) / 24.0f);
                        const glm::vec3 swivelBend = glm::normalize(
                            glm::angleAxis(swivel, sampleForward)
                                * sampleBend);
                        const glm::vec3 upperDirection = glm::normalize(
                            sampleForward * hipCosine
                            + swivelBend * hipSine);
                        const glm::vec3 sampledKnee = sampledHip
                            + upperDirection * admissionUpperLength;
                        const glm::vec3 lowerDirection =
                            sampledAnkle - sampledKnee;
                        glm::quat rawHipWorld(1, 0, 0, 0);
                        if (!RotationMatchingVectorPair(
                                admissionUpperLocal,
                                kneeLocal * admissionLowerLocal,
                                upperDirection, lowerDirection,
                                rawHipWorld)) {
                            continue;
                        }
                        const glm::quat rawHipLocal = glm::normalize(
                            glm::conjugate(sampledParentWorld)
                                * rawHipWorld);
                        const glm::quat boundedHipLocal = ClampToEnvelope(
                            admissionHipEnvelope,
                            skeleton.bones[swing->hipIdx].localR,
                            rawHipLocal, comp.hipLimitMarginDeg);
                        const glm::quat boundedHipWorld = glm::normalize(
                            sampledParentWorld * boundedHipLocal);
                        const glm::quat kneeWorld = glm::normalize(
                            boundedHipWorld * kneeLocal);
                        const glm::quat rawAnkleLocal = glm::normalize(
                            glm::conjugate(kneeWorld) * footRotation
                                * glm::conjugate(
                                    swing->referenceFootLocal));
                        const glm::quat boundedAnkleLocal =
                            ClampToEnvelope(
                                admissionAnkleEnvelope,
                                skeleton.bones[swing->ankleIdx].localR,
                                rawAnkleLocal,
                                comp.hipLimitMarginDeg);
                        const float ankleClamp = rotationDifferenceDeg(
                            rawAnkleLocal, boundedAnkleLocal);
                        const float hipClamp = rotationDifferenceDeg(
                            rawHipLocal, boundedHipLocal);
                        const glm::vec3 commandedKnee = sampledHip
                            + boundedHipWorld * admissionUpperLocal;
                        const glm::vec3 commandedAnkle = commandedKnee
                            + kneeWorld * admissionLowerLocal;
                        const glm::quat commandedFootWorld = glm::normalize(
                            kneeWorld * boundedAnkleLocal
                                * swing->referenceFootLocal);
                        const glm::vec3 commandedFoot = commandedAnkle
                            - commandedFootWorld
                                * swing->ankleFromFootLocal;
                        const float positionClosure = glm::length(
                            commandedFoot - sampledFoot);
                        constexpr float kPositionClosureTolerance = 0.003f;
                        if (positionClosure > kPositionClosureTolerance)
                            continue;
                        const float cost = ankleClamp * 20.0f
                            + hipClamp * 4.0f
                            + std::abs(glm::degrees(swivel)) * 0.002f;
                        if (cost < bestCost) {
                            bestCost = cost;
                            bestClamp = ankleClamp;
                            bestSwivel = swivel;
                            bestClosure = positionClosure;
                        }
                    }
                    if (bestCost >= 1e9f) {
                        result.maximumClampDeg = 180.0f;
                        result.maximumClosure = glm::max(
                            result.maximumClosure, bestClosure);
                        result.allSamplesClosed = false;
                        continue;
                    }
                    result.maximumClampDeg = glm::max(
                        result.maximumClampDeg, bestClamp);
                    result.maximumClosure = glm::max(
                        result.maximumClosure, bestClosure);
                    if (trajectoryT > 0.99f)
                        result.contactSwivelDeg =
                            glm::degrees(bestSwivel);
                }
                return result;
            };

            AdmissionOrientationResult orientationAdmission;
            if (continuousEnabled && angularObjective)
                orientationAdmission =
                    sampleOrientationFeasibility(target);
            comp._physicalStepFoothold = target;
            glm::vec3 predictedContactSupportVelocity(0.0f);
            const glm::vec3 predictedLandingHip = predictLandingHip(
                target, &predictedContactSupportVelocity);
            if (continuousEnabled) {
                comp._gaitIkPlanHip = predictedLandingHip;
                comp._gaitIkPlanHipValid = true;
            }

            // Slice 3 shadow admission. This deliberately does not feed footholdAccepted:
            // first prove that the predictor agrees with the already-validated 5-degree
            // executor. Unlike the retained Slice 2j counterfactual above, this evaluates
            // the exact physical segment geometry and zero-swivel, position-primary chain
            // that Slice 2n currently commands. Both legs are tested at the admitted end
            // heading while the stance sole remains fixed in world space.
            if (continuousEnabled && angularObjective) {
                using Constraint = Comp::TurnConstraintResult;
                struct ShadowLegResult {
                    Comp::TurnLegFeasibilityDiagnostics diagnostics;
                    Constraint constraint = Constraint::NotEvaluated;
                };

                auto envelopeForHip = [](const Leg& leg) {
                    Envelope envelope;
                    envelope.twistAxis = leg.hipTwistAxis;
                    envelope.swingNormalDeg = leg.hipSwingNormalDeg;
                    envelope.swingPlaneDeg = leg.hipSwingPlaneDeg;
                    envelope.twistMinDeg = leg.hipTwistMinDeg;
                    envelope.twistMaxDeg = leg.hipTwistMaxDeg;
                    return envelope;
                };
                auto envelopeForAnkle = [](const Leg& leg) {
                    Envelope envelope;
                    envelope.twistAxis = leg.ankleAxis;
                    envelope.swingNormalDeg = leg.ankleSwingNormalDeg;
                    envelope.swingPlaneDeg = leg.ankleSwingPlaneDeg;
                    envelope.twistMinDeg = leg.ankleTwistMinDeg;
                    envelope.twistMaxDeg = leg.ankleTwistMaxDeg;
                    return envelope;
                };
                auto physicalParentRotation = [&](const Leg& leg,
                                                  bool& valid) {
                    const int parent = skeleton.bones[leg.hipIdx].parent;
                    if (parent < 0) {
                        valid = false;
                        return glm::quat(1, 0, 0, 0);
                    }
                    return Physics::GetRagdollBoneRotation(
                        rag, parent, &valid);
                };
                auto kneePoleAtEnd = [&](const Leg& leg) {
                    glm::vec3 pole = leg.groundReferenceKneePoleValid
                        ? endRotation
                            * leg.groundReferenceKneePoleHeadingLocal
                        : glm::normalize(endRotation
                            * glm::conjugate(startRotation))
                            * leg.kneePoleWorld;
                    if (glm::dot(pole, pole) < 1e-8f)
                        pole = turnPlan.activeEndForward;
                    return glm::normalize(pole);
                };
                auto evaluateShadowLeg = [&](const Leg& leg,
                                              const glm::vec3& footPosition,
                                              const glm::quat& footRotation,
                                              const glm::vec3& predictedHip,
                                              const glm::quat& predictedParent,
                                              bool parentValid,
                                              bool swingLeg) {
                    ShadowLegResult result;
                    auto& d = result.diagnostics;
                    d.physicalGeometryValid = leg.segmentGeometryValid
                        && parentValid
                        && glm::dot(leg.upperSegmentLocal,
                                    leg.upperSegmentLocal) > 1e-8f
                        && glm::dot(leg.lowerSegmentLocal,
                                    leg.lowerSegmentLocal) > 1e-8f;
                    if (!d.physicalGeometryValid) {
                        result.constraint = Constraint::PhysicalGeometry;
                        return result;
                    }

                    const float upperLength = glm::length(
                        leg.upperSegmentLocal);
                    const float lowerLength = glm::length(
                        leg.lowerSegmentLocal);
                    const glm::vec3 desiredAnkle = footPosition
                        + ankleFromFootWorld(leg, footRotation);
                    const glm::vec3 toAnkle = desiredAnkle - predictedHip;
                    d.reach = glm::length(toAnkle);

                    bool currentHipValid = false;
                    bool currentAnkleValid = false;
                    const glm::vec3 currentHip =
                        Physics::GetRagdollBonePosition(
                            rag, leg.hipIdx, &currentHipValid);
                    const glm::vec3 currentAnkle =
                        Physics::GetRagdollBonePosition(
                            rag, leg.ankleIdx, &currentAnkleValid);
                    if (!currentHipValid || !currentAnkleValid) {
                        d.physicalGeometryValid = false;
                        result.constraint = Constraint::PhysicalGeometry;
                        return result;
                    }
                    const float legLength = upperLength + lowerLength;
                    const float currentReach = glm::length(
                        currentAnkle - currentHip);
                    const float configuredReach = legLength
                        * glm::clamp(comp.maxLegReachFraction,
                                     0.70f, 0.99f);
                    const float safeAnatomicalReach = legLength * glm::clamp(
                        safeReachFraction, 0.94f, 0.995f);
                    const float antiSingularityCeiling = legLength * 0.995f;
                    d.reachLimit = glm::min(
                        glm::max(configuredReach,
                                 glm::max(currentReach,
                                          safeAnatomicalReach)),
                        glm::max(currentReach,
                                 antiSingularityCeiling));
                    d.reachMargin = d.reachLimit - d.reach;
                    const float minimumReach = std::abs(
                        upperLength - lowerLength) + 1e-4f;
                    if (d.reach <= minimumReach
                        || d.reach > d.reachLimit + 0.0005f) {
                        result.constraint = swingLeg
                            ? Constraint::SwingReach
                            : Constraint::StanceReach;
                        return result;
                    }

                    const float includedCos = glm::clamp(
                        (upperLength * upperLength
                         + lowerLength * lowerLength
                         - d.reach * d.reach)
                            / (2.0f * upperLength * lowerLength),
                        -1.0f, 1.0f);
                    const float kneeBend = glm::pi<float>()
                        - std::acos(includedCos);
                    d.kneeBendDeg = glm::degrees(kneeBend);
                    const float kneeMinimum = glm::min(
                        leg.kneeMinDeg + comp.hipLimitMarginDeg,
                        0.5f * (leg.kneeMinDeg + leg.kneeMaxDeg));
                    const float kneeMaximum = glm::max(
                        leg.kneeMaxDeg - comp.hipLimitMarginDeg,
                        0.5f * (leg.kneeMinDeg + leg.kneeMaxDeg));
                    d.kneeMarginDeg = glm::min(
                        d.kneeBendDeg - kneeMinimum,
                        kneeMaximum - d.kneeBendDeg);
                    if (d.kneeMarginDeg < -0.05f) {
                        result.constraint = Constraint::KneeEnvelope;
                        return result;
                    }

                    const glm::quat kneeTarget = glm::normalize(
                        leg.referenceKneeLocal
                        * glm::angleAxis(
                            kneeBend - leg.referenceKneeBend,
                            glm::normalize(leg.kneeHingeAxis)));
                    const glm::vec3 worldForward = glm::normalize(toAnkle);
                    glm::vec3 worldBend = kneePoleAtEnd(leg)
                        - worldForward * glm::dot(
                            kneePoleAtEnd(leg), worldForward);
                    if (glm::dot(worldBend, worldBend) < 1e-8f)
                        worldBend = turnPlan.activeEndRight;
                    worldBend -= worldForward
                        * glm::dot(worldBend, worldForward);
                    if (glm::dot(worldBend, worldBend) < 1e-8f) {
                        result.constraint = Constraint::CommandClosure;
                        return result;
                    }
                    worldBend = glm::normalize(worldBend);
                    const float hipCos = glm::clamp(
                        (upperLength * upperLength
                         + d.reach * d.reach
                         - lowerLength * lowerLength)
                            / (2.0f * upperLength * d.reach),
                        -1.0f, 1.0f);
                    const float hipSin = std::sqrt(glm::max(
                        1.0f - hipCos * hipCos, 0.0f));
                    const glm::vec3 upperDirection = glm::normalize(
                        worldForward * hipCos + worldBend * hipSin);
                    const glm::vec3 predictedKnee = predictedHip
                        + upperDirection * upperLength;
                    const glm::vec3 lowerDirection =
                        desiredAnkle - predictedKnee;
                    glm::quat rawHipWorld(1, 0, 0, 0);
                    if (!RotationMatchingVectorPair(
                            leg.upperSegmentLocal,
                            kneeTarget * leg.lowerSegmentLocal,
                            upperDirection, lowerDirection,
                            rawHipWorld)) {
                        result.constraint = Constraint::CommandClosure;
                        return result;
                    }

                    const Envelope hipEnvelope = envelopeForHip(leg);
                    const glm::quat rawHipLocal = glm::normalize(
                        glm::conjugate(predictedParent) * rawHipWorld);
                    const glm::quat boundedHipLocal = ClampToEnvelope(
                        hipEnvelope,
                        skeleton.bones[leg.hipIdx].localR,
                        rawHipLocal, comp.hipLimitMarginDeg);
                    const EnvelopeMeasurement hipMeasurement = MeasureEnvelope(
                        hipEnvelope,
                        skeleton.bones[leg.hipIdx].localR,
                        rawHipLocal, comp.hipLimitMarginDeg);
                    d.hipClampDeg = rotationDifferenceDeg(
                        rawHipLocal, boundedHipLocal);
                    d.hipSwingReserve = hipMeasurement.swingReserve;
                    d.hipTwistMarginDeg = hipMeasurement.twistMarginDeg;
                    if (d.hipClampDeg > 0.10f) {
                        result.constraint = swingLeg
                            ? Constraint::SwingHipEnvelope
                            : Constraint::StanceHipEnvelope;
                        return result;
                    }

                    const glm::quat boundedHipWorld = glm::normalize(
                        predictedParent * boundedHipLocal);
                    const glm::quat kneeWorld = glm::normalize(
                        boundedHipWorld * kneeTarget);
                    const Envelope ankleEnvelope = envelopeForAnkle(leg);
                    const glm::quat rawAnkleLocal = glm::normalize(
                        glm::conjugate(kneeWorld) * footRotation
                        * glm::conjugate(leg.referenceFootLocal));
                    const glm::quat boundedAnkleLocal = ClampToEnvelope(
                        ankleEnvelope,
                        skeleton.bones[leg.ankleIdx].localR,
                        rawAnkleLocal, comp.hipLimitMarginDeg);
                    const glm::quat hardBoundedAnkleLocal = ClampToEnvelope(
                        ankleEnvelope,
                        skeleton.bones[leg.ankleIdx].localR,
                        rawAnkleLocal, 0.0f);
                    const EnvelopeMeasurement ankleMeasurement =
                        MeasureEnvelope(
                            ankleEnvelope,
                            skeleton.bones[leg.ankleIdx].localR,
                            rawAnkleLocal, comp.hipLimitMarginDeg);
                    d.ankleClampDeg = rotationDifferenceDeg(
                        rawAnkleLocal, boundedAnkleLocal);
                    d.ankleHardClampDeg = rotationDifferenceDeg(
                        rawAnkleLocal, hardBoundedAnkleLocal);
                    d.ankleSwingReserve = ankleMeasurement.swingReserve;
                    d.ankleTwistMarginDeg = ankleMeasurement.twistMarginDeg;
                    // The executor intentionally clamps against a safety envelope before
                    // the authored hard limit. The latest clean mirrored step consumed
                    // 1.66 degrees of the configured three-degree buffer with no hard
                    // clamp. Preserve at least one full degree of that buffer; consuming
                    // more, or touching the authored limit, remains unsafe.
                    constexpr float kMinimumRemainingAnkleSafetyReserveDeg =
                        1.00f;
                    const float maximumAnkleSafetyReserveConsumptionDeg =
                        glm::max(comp.hipLimitMarginDeg
                            - kMinimumRemainingAnkleSafetyReserveDeg,
                            0.0f);
                    if (d.ankleHardClampDeg > 0.10f
                        || d.ankleClampDeg
                            > maximumAnkleSafetyReserveConsumptionDeg) {
                        result.constraint = Constraint::AnkleEnvelope;
                        return result;
                    }

                    const glm::vec3 commandedKnee = predictedHip
                        + boundedHipWorld * leg.upperSegmentLocal;
                    const glm::vec3 commandedAnkle = commandedKnee
                        + kneeWorld * leg.lowerSegmentLocal;
                    const glm::quat commandedFootRotation = glm::normalize(
                        kneeWorld * boundedAnkleLocal
                        * leg.referenceFootLocal);
                    const glm::vec3 commandedFoot = commandedAnkle
                        - commandedFootRotation * leg.ankleFromFootLocal;
                    d.positionClosure = glm::length(
                        commandedFoot - footPosition);
                    if (d.positionClosure > 0.0035f) {
                        result.constraint = Constraint::CommandClosure;
                        return result;
                    }

                    d.safe = true;
                    result.constraint = Constraint::None;
                    return result;
                };

                const glm::quat headingDelta = glm::normalize(
                    endRotation * glm::conjugate(startRotation));
                const glm::quat contactHeading = glm::normalize(glm::slerp(
                    startRotation, endRotation, 0.80f));
                const glm::quat contactHeadingDelta = glm::normalize(
                    contactHeading * glm::conjugate(startRotation));
                glm::vec3 swingHipFromCom = hip - rag._locomotionCOM;
                swingHipFromCom.y = 0.0f;
                glm::vec3 contactHeadingHipShift =
                    contactHeadingDelta * swingHipFromCom - swingHipFromCom;
                contactHeadingHipShift.y = 0.0f;
                glm::vec3 predictedSupportShift = predictedLandingHip
                    - hip - contactHeadingHipShift;
                predictedSupportShift.y = 0.0f;
                auto predictHipAtEndHeading = [&](const glm::vec3& currentHip) {
                    glm::vec3 hipFromCom = currentHip - rag._locomotionCOM;
                    hipFromCom.y = 0.0f;
                    glm::vec3 headingShift =
                        headingDelta * hipFromCom - hipFromCom;
                    headingShift.y = 0.0f;
                    return currentHip + predictedSupportShift + headingShift;
                };

                bool swingParentValid = false;
                bool stanceParentValid = false;
                const glm::quat swingParentAtEnd = glm::normalize(
                    headingDelta
                    * physicalParentRotation(*swing, swingParentValid));
                const glm::quat stanceParentAtEnd = glm::normalize(
                    headingDelta
                    * physicalParentRotation(*stance, stanceParentValid));
                bool stanceHipValid = false;
                const glm::vec3 currentStanceHip =
                    Physics::GetRagdollBonePosition(
                        rag, stance->hipIdx, &stanceHipValid);
                if (!stanceHipValid)
                    stanceParentValid = false;

                const glm::quat swingFootAtEnd =
                    footWorldRotationForHeading(*swing, endRotation);
                const ShadowLegResult shadowSwing = evaluateShadowLeg(
                    *swing, target, swingFootAtEnd,
                    predictHipAtEndHeading(hip), swingParentAtEnd,
                    swingParentValid, true);
                const ShadowLegResult shadowStance = evaluateShadowLeg(
                    *stance, stanceFoot,
                    glm::normalize(stance->plantedFootWorldRotation),
                    predictHipAtEndHeading(currentStanceHip),
                    stanceParentAtEnd, stanceParentValid, false);
                turnPlan.shadowSwing = shadowSwing.diagnostics;
                turnPlan.shadowStance = shadowStance.diagnostics;
                turnPlan.shadowFeasibilityEvaluated = true;
                turnPlan.shadowRequestedSwingSpeed = requestedSwingSpeed;
                turnPlan.shadowAdmittedSwingSpeed =
                    requiredSwingTargetSpeed(target);
                turnPlan.shadowSwingSpeedLimit = swingSpeedLimit;
                turnPlan.shadowSwingSpeedClampLoss = glm::max(
                    turnPlan.shadowRequestedSwingSpeed
                        - turnPlan.shadowAdmittedSwingSpeed,
                    0.0f);
                // Keep the former eight-percent threshold as a comparison reference,
                // not a gate. The next mirrored run showed that intentional pair-budget
                // and descending-yaw target reshaping also creates large clamp loss:
                // twelve such predictions committed, while one zero-loss step aborted.
                constexpr float kInsideSwingSpeedClampReferenceFraction = 0.08f;
                turnPlan.shadowSwingSpeedClampReference =
                    turnPlan.outsideFoot
                        ? 0.0f
                        : swingSpeedLimit
                            * kInsideSwingSpeedClampReferenceFraction;

                const float shadowLane = plannedLaneCoefficient(
                    target - stanceFoot);
                constexpr float kShadowMinimumFootSeparation = 0.10f;
                if (!targetGround.hit) {
                    turnPlan.shadowLimitingConstraint = Constraint::Terrain;
                } else if (shadowLane * swingSide <= 0.0f) {
                    turnPlan.shadowLimitingConstraint =
                        Constraint::LaneCrossing;
                } else if (std::abs(shadowLane)
                           < kShadowMinimumFootSeparation - 0.0005f) {
                    turnPlan.shadowLimitingConstraint =
                        Constraint::FootSeparation;
                } else if (!shadowSwing.diagnostics.safe) {
                    turnPlan.shadowLimitingConstraint =
                        shadowSwing.constraint;
                } else if (!shadowStance.diagnostics.safe) {
                    turnPlan.shadowLimitingConstraint =
                        shadowStance.constraint;
                } else {
                    turnPlan.shadowLimitingConstraint = Constraint::None;
                    turnPlan.shadowFeasibilitySafe = true;
                }
            }
            const float requestedForward = glm::dot(
                requestedTarget - comp._physicalStepSwingStart, comp._physicalStepForward);
            const float plannedForward = glm::dot(
                target - comp._physicalStepSwingStart, comp._physicalStepForward);
            const float requestedSupportAdvance = plannedAdvanceCoefficient(
                requestedTarget - stanceFoot);
            const float plannedSupportAdvance = plannedAdvanceCoefficient(
                target - stanceFoot);
            const float legacyStartFrameAdvance = glm::dot(
                target - stanceFoot, comp._physicalStepForward);

            const glm::quat candidateRotation =
                footWorldRotationForHeading(*swing, endRotation);
            turnPlan.candidateFootPosition = requestedTarget;
            turnPlan.admittedFootPosition = target;
            turnPlan.candidateFootRotation = candidateRotation;
            turnPlan.admittedFootRotation = candidateRotation;
            turnPlan.requestedAdvance = requestedSupportAdvance;
            turnPlan.admittedAdvance = plannedSupportAdvance;
            // A translation smaller than the accepted landing tolerance cannot be
            // independently proved after ordinary physical tracking loss. Treat that
            // footprint as angular-only instead of arming a brittle combined gate.
            const float minimumUsefulTranslationObjective = glm::max(
                0.020f, comp.footTargetTolerance + 0.010f);
            const bool admittedTranslationObjective =
                plannedSupportAdvance >= minimumUsefulTranslationObjective;
            const bool requestedAngularObjective =
                std::abs(admittedStepYaw) > 1e-6f;
            turnPlan.objective = turnInitiationDeferred
                ? Comp::TurnStepObjective::None
                : (requestedAngularObjective
                    ? (admittedTranslationObjective
                        ? Comp::TurnStepObjective::Combined
                        : Comp::TurnStepObjective::Angular)
                    : Comp::TurnStepObjective::Translation);
            turnPlan.requestedSwingDistance = horizontalDistance(
                requestedTarget, comp._physicalStepSwingStart);
            turnPlan.admittedSwingDistance = horizontalDistance(
                target, comp._physicalStepSwingStart);
            turnPlan.requiredSwingSpeed = requestedSwingSpeed;
            turnPlan.admittedSwingSpeed =
                requiredSwingTargetSpeed(target);
            turnPlan.swingSpeedLimit = continuousEnabled
                ? swingSpeedLimit : 0.0f;
            const float admittedSpeedOvershoot = continuousEnabled
                ? glm::max(turnPlan.admittedSwingSpeed
                    - swingSpeedLimit, 0.0f)
                : 0.0f;
            turnPlan.swingSpeedLaneFloorApplied = swingSpeedLimited
                && admittedSpeedOvershoot > 0.0f
                && admittedSpeedOvershoot <= kSwingSpeedClosureTolerance;
            turnPlan.swingSpeedLaneFloorExceeded = swingSpeedLimited
                && admittedSpeedOvershoot > kSwingSpeedClosureTolerance;
            turnPlan.predictedContactHip = predictedLandingHip;
            turnPlan.predictedContactSupportVelocity =
                predictedContactSupportVelocity;
            turnPlan.requestedReach = requestedReach;
            turnPlan.admittedReach = targetReach(target);
            turnPlan.reachLimit = comp._physicalStepReachLimit;
            turnPlan.reachMargin =
                comp._physicalStepReachLimit - turnPlan.admittedReach;
            turnPlan.candidateFootSeparation = std::abs(
                plannedLaneCoefficient(requestedTarget - stanceFoot));
            turnPlan.admittedFootSeparation = std::abs(
                plannedLaneCoefficient(target - stanceFoot));
            turnPlan.sampledAnkleClampDeg =
                orientationAdmission.maximumClampDeg;
            turnPlan.admittedKneeSwivelDeg =
                orientationAdmission.contactSwivelDeg;
            turnPlan.sampledSwivelClosure =
                orientationAdmission.maximumClosure;
            turnPlan.sampledSwivelAccepted =
                orientationAdmission.allSamplesClosed;
            turnPlan.candidateEvaluated = true;
            turnPlan.limitingConstraint = turnInitiationDeferred
                ? Comp::TurnConstraintResult::TurnInitiationRole
                : (reachClamped
                ? Comp::TurnConstraintResult::SwingReach
                : (swingSpeedLimited
                    ? Comp::TurnConstraintResult::SwingLinearSpeed
                    : (pairAdvanceLimited || pairBudgetApplied
                        ? Comp::TurnConstraintResult::TurnPairBudget
                    : (angularSpeedLimited
                        ? Comp::TurnConstraintResult::SwingAngularSpeed
                        : (footSeparationLimited
                            ? Comp::TurnConstraintResult::FootSeparation
                        : (controllerYawLimited
                            ? Comp::TurnConstraintResult::ControllerYawLimit
                            : Comp::TurnConstraintResult::None))))));

            comp._gaitPlannedSupportAdvance = continuousEnabled
                ? plannedSupportAdvance : 0.0f;
            if (continuousEnabled)
                comp._gaitReachClampedStep = reachClamped;
            if (continuousEnabled) {
                // The settle gate validates achieved support-to-support advance, so admit
                // the foothold in that same space. The filtered loss is measured from prior
                // physical landings; ignoring it here admitted plans that were reachable
                // analytically but already predicted to finish below the 6 cm invariant.
                const float predictedAchievedAdvance =
                    plannedSupportAdvance - trackingReserve;
                const bool dynamicSwingFeasible =
                    turnPlan.admittedSwingSpeed
                        <= swingSpeedLimit
                            + turnPlan.swingSpeedClosureTolerance;
                const bool dynamicAngularFeasible =
                    turnPlan.admittedAngularSpeed
                        <= turnPlan.angularSpeedLimit + 0.001f;
                // A turn-dominant step may intentionally spend its entire budget on
                // angular progress and lane separation.  Its landing is re-planned from
                // the measured footprint, so the straight six-centimetre advance invariant
                // is not an admission gate for this objective.
                const bool conditionedPoseFeasible =
                    turnPlan.candidateGrounded
                        && dynamicSwingFeasible
                        && dynamicAngularFeasible
                        && turnPlan.admittedReach
                            <= turnPlan.reachLimit + 0.0005f;
                const bool minimumAdvanceReady =
                    predictedAchievedAdvance + 0.0005f >= minimumAdvance;
                const bool footholdAccepted = turnInitiationDeferred
                    ? conditionedPoseFeasible
                    : (requestedAngularObjective
                        ? conditionedPoseFeasible
                        : (turnExitBlendApplied
                        ? conditionedPoseFeasible
                            && minimumAdvanceReady
                        : minimumAdvanceReady));
                turnPlan.candidateAccepted = footholdAccepted;
                turnPlan.shadowRuntimeAccepted = footholdAccepted;
                turnPlan.activeHeadingPlan = footholdAccepted;
                if (!footholdAccepted)
                    turnPlan.limitingConstraint = requestedAngularObjective
                        ? (dynamicAngularFeasible
                            ? Comp::TurnConstraintResult::SwingLinearSpeed
                            : Comp::TurnConstraintResult::SwingAngularSpeed)
                        : Comp::TurnConstraintResult::MinimumAdvance;
                if (footholdAccepted && requestedAngularObjective) {
                    if (turnPlan.outsideFoot) {
                        const float admittedAdvanceRatio =
                            turnPlan.requestedAdvance > 1e-4f
                            ? turnPlan.admittedAdvance
                                / turnPlan.requestedAdvance
                            : 1.0f;
                        const float admittedLaneYawRatio =
                            laneFloorYawReduced
                                && std::abs(pairRequestedStepYaw) > 1e-6f
                            ? glm::clamp(std::abs(
                                admittedStepYaw / pairRequestedStepYaw),
                                0.0f, 1.0f)
                            : 1.0f;
                        comp._gaitTurnPairAdvanceScale = glm::clamp(
                            admittedAdvanceRatio, 0.25f, 1.0f);
                        comp._gaitTurnPairYawScale = glm::min(
                            admittedLaneYawRatio,
                            glm::clamp(
                                0.55f
                                    + 0.45f
                                        * comp._gaitTurnPairAdvanceScale,
                                0.55f, 1.0f));
                        comp._gaitTurnPairYawSign =
                            admittedStepYaw >= 0.0f ? 1.0f : -1.0f;
                        comp._gaitTurnPairPendingInside = true;
                        turnPlan.pairAdvanceScale =
                            comp._gaitTurnPairAdvanceScale;
                        turnPlan.pairYawScale =
                            comp._gaitTurnPairYawScale;
                        turnPlan.pairBudgetLatched = true;
                    } else if (pairBudgetApplied) {
                        // Consume exactly once.  An abort below clears the pair as well,
                        // while a successful inside plan must not constrain the next
                        // outside foot a second time.
                        comp._gaitTurnPairPendingInside = false;
                        comp._gaitTurnPairAdvanceScale = 1.0f;
                        comp._gaitTurnPairYawScale = 1.0f;
                        comp._gaitTurnPairYawSign = 0.0f;
                    }
                }
                if (comp.debug) {
                    if (!footholdAccepted && !minimumAdvanceReady
                        && !requestedAngularObjective
                        && !turnInitiationDeferred) {
                        spdlog::info(
                            "[LocomotionTurnAdmission] "
                            "result=MINIMUM_ADVANCE_REJECTED step={} "
                            "swing={} exitBlend={} objective={} "
                            "advance=(planned={:.3f},predicted={:.3f},"
                            "minimum={:.3f},reserve={:.3f})m "
                            "poseFeasible={} action=reject-before-takeoff",
                            comp._stepSequenceStepIndex,
                            turnPlan.swingFootLeft ? "LEFT" : "RIGHT",
                            turnPlan.turnExitBlendApplied ? "yes" : "no",
                            TurnObjectiveName(turnPlan.objective),
                            plannedSupportAdvance,
                            predictedAchievedAdvance,
                            minimumAdvance,
                            trackingReserve,
                            conditionedPoseFeasible ? "yes" : "NO");
                    }
                    if (turnPlan.turnInitiationDeferred) {
                        spdlog::info(
                            "[LocomotionTurnInitiation] step={} swing={} "
                            "action=ROLE_SWAP requested={:+.3f}deg admitted=+0.000deg "
                            "advance={:.3f}m reason=inside-foot-first next=outside-foot",
                            comp._stepSequenceStepIndex,
                            turnPlan.swingFootLeft ? "LEFT" : "RIGHT",
                            glm::degrees(turnPlan.requestedYaw),
                            turnPlan.admittedAdvance);
                    }
                    if (laneFloorYawReduced) {
                        spdlog::info(
                            "[LocomotionTurnAdmission] result=YAW_DESCENT step={} "
                            "swing={} outside={} yaw=({:+.3f}->{:+.3f})deg "
                            "laneFloor=({:.3f}->{:.3f})mps limit={:.3f}mps "
                            "tolerance={:.3f}mps action=rebuild-complete-footprint",
                            comp._stepSequenceStepIndex,
                            turnPlan.swingFootLeft ? "LEFT" : "RIGHT",
                            turnPlan.outsideFoot ? "yes" : "no",
                            glm::degrees(laneFloorYawBefore),
                            glm::degrees(turnPlan.admittedYaw),
                            laneFloorSpeedBefore,
                            turnPlan.minimumLaneSwingSpeed,
                            turnPlan.swingSpeedLimit,
                            turnPlan.swingSpeedClosureTolerance);
                    }
                    if (turnPlan.swingSpeedLaneFloorApplied
                        || turnPlan.swingSpeedLaneFloorExceeded) {
                        spdlog::info(
                            "[LocomotionTurnAdmission] result={} step={} "
                            "swing={} outside={} laneFloor={:.3f}mps "
                            "limit={:.3f}mps overshoot={:.3f}mps "
                            "tolerance={:.3f}mps accepted={} action={}",
                            turnPlan.swingSpeedLaneFloorApplied
                                ? "LANE_FLOOR_TOLERANCE"
                                : "LANE_FLOOR_EXCEEDED",
                            comp._stepSequenceStepIndex,
                            turnPlan.swingFootLeft ? "LEFT" : "RIGHT",
                            turnPlan.outsideFoot ? "yes" : "no",
                            turnPlan.minimumLaneSwingSpeed,
                            turnPlan.swingSpeedLimit,
                            glm::max(turnPlan.admittedSwingSpeed
                                - turnPlan.swingSpeedLimit, 0.0f),
                            turnPlan.swingSpeedClosureTolerance,
                            footholdAccepted ? "yes" : "NO",
                            turnPlan.swingSpeedLaneFloorApplied
                                ? "admit-bounded-floor"
                                : "await-descending-yaw");
                    }
                    if (turnPlan.shadowFeasibilityEvaluated) {
                        const auto& shadowSwing = turnPlan.shadowSwing;
                        const auto& shadowStance = turnPlan.shadowStance;
                        spdlog::info(
                            "[LocomotionTurnFeasibilityShadow] step={} swing={} outside={} "
                            "yaw=({:+.3f}->{:+.3f})deg runtimeAccepted={} shadowSafe={} "
                            "constraint={} tracking=(speed={:.3f}->{:.3f}/{:.3f}mps,"
                            "clampLoss={:.3f}mps,reference={:.3f}mps,gate=disabled) "
                            "swing=(geometry={},reach={:.3f}/{:.3f}m,margin={:+.3f}m,"
                            "knee={:.1f}deg/{:+.1f}deg,"
                            "hipClamp={:.2f}deg,hipReserve={:+.2f}/{:+.1f}deg,"
                            "ankleClamp={:.2f}/{:.2f}deg,ankleReserve={:+.2f}/{:+.1f}deg,"
                            "closure={:.4f}m) "
                            "stance=(geometry={},reach={:.3f}/{:.3f}m,margin={:+.3f}m,"
                            "knee={:.1f}deg/{:+.1f}deg,"
                            "hipClamp={:.2f}deg,hipReserve={:+.2f}/{:+.1f}deg,"
                            "ankleClamp={:.2f}/{:.2f}deg,ankleReserve={:+.2f}/{:+.1f}deg,"
                            "closure={:.4f}m)",
                            comp._stepSequenceStepIndex,
                            turnPlan.swingFootLeft ? "LEFT" : "RIGHT",
                            turnPlan.outsideFoot ? "yes" : "no",
                            glm::degrees(turnPlan.requestedYaw),
                            glm::degrees(turnPlan.admittedYaw),
                            footholdAccepted ? "yes" : "no",
                            turnPlan.shadowFeasibilitySafe ? "yes" : "NO",
                            TurnConstraintName(
                                turnPlan.shadowLimitingConstraint),
                            turnPlan.shadowRequestedSwingSpeed,
                            turnPlan.shadowAdmittedSwingSpeed,
                            turnPlan.shadowSwingSpeedLimit,
                            turnPlan.shadowSwingSpeedClampLoss,
                            turnPlan.shadowSwingSpeedClampReference,
                            shadowSwing.physicalGeometryValid ? "yes" : "NO",
                            shadowSwing.reach,
                            shadowSwing.reachLimit,
                            shadowSwing.reachMargin,
                            shadowSwing.kneeBendDeg,
                            shadowSwing.kneeMarginDeg,
                            shadowSwing.hipClampDeg,
                            shadowSwing.hipSwingReserve,
                            shadowSwing.hipTwistMarginDeg,
                            shadowSwing.ankleClampDeg,
                            shadowSwing.ankleHardClampDeg,
                            shadowSwing.ankleSwingReserve,
                            shadowSwing.ankleTwistMarginDeg,
                            shadowSwing.positionClosure,
                            shadowStance.physicalGeometryValid ? "yes" : "NO",
                            shadowStance.reach,
                            shadowStance.reachLimit,
                            shadowStance.reachMargin,
                            shadowStance.kneeBendDeg,
                            shadowStance.kneeMarginDeg,
                            shadowStance.hipClampDeg,
                            shadowStance.hipSwingReserve,
                            shadowStance.hipTwistMarginDeg,
                            shadowStance.ankleClampDeg,
                            shadowStance.ankleHardClampDeg,
                            shadowStance.ankleSwingReserve,
                            shadowStance.ankleTwistMarginDeg,
                            shadowStance.positionClosure);
                        if (footholdAccepted
                            && !turnPlan.shadowFeasibilitySafe) {
                            spdlog::warn(
                                "[LocomotionTurnFeasibilityMismatch] "
                                "result=SHADOW_WOULD_REJECT_RUNNING_STEP "
                                "step={} swing={} outside={} yaw={:+.3f}deg "
                                "constraint={} tracking=({:.3f}->{:.3f}/{:.3f}mps,"
                                "clampLoss={:.3f}mps,reference={:.3f}mps,gate=disabled) "
                                "ankleClamp=(swing={:.2f}/{:.2f},"
                                "stance={:.2f}/{:.2f})deg "
                                "action=keep-shadow-routing-unchanged",
                                comp._stepSequenceStepIndex,
                                turnPlan.swingFootLeft ? "LEFT" : "RIGHT",
                                turnPlan.outsideFoot ? "yes" : "no",
                                glm::degrees(turnPlan.admittedYaw),
                                TurnConstraintName(
                                    turnPlan.shadowLimitingConstraint),
                                turnPlan.shadowRequestedSwingSpeed,
                                turnPlan.shadowAdmittedSwingSpeed,
                                turnPlan.shadowSwingSpeedLimit,
                                turnPlan.shadowSwingSpeedClampLoss,
                                turnPlan.shadowSwingSpeedClampReference,
                                shadowSwing.ankleClampDeg,
                                shadowSwing.ankleHardClampDeg,
                                shadowStance.ankleClampDeg,
                                shadowStance.ankleHardClampDeg);
                        }
                    }
                    spdlog::info(
                        "[LocomotionTurnPlan] swing={} outside={} "
                        "startDeferred={} exitBlend={} "
                        "desiredError={:+.3f}deg "
                        "objective={} controller={} "
                        "headingAdvance=(nominal={:.3f}m,scale={:.3f}) "
                        "yaw=(requested={:+.3f},admitted={:+.3f},achieved={:+.3f})deg "
                        "advance=(requested={:.3f},admitted={:.3f},achieved={:.3f},"
                        "legacyStart={:.3f})m "
                        "pair=(applied={},latched={},advanceScale={:.3f},"
                        "yawScale={:.3f},support={:.3f}mps) "
                        "swingMotion=(distance={:.3f}->{:.3f}m,"
                        "speed={:.3f}->{:.3f}/{:.3f}mps,"
                        "laneFloor={:.3f}mps,tolerance={:.3f}mps,"
                        "floorApplied={},floorExceeded={},"
                        "angular={:.3f}->{:.3f}/{:.3f}radps) "
                        "reach=(requested={:.3f},admitted={:.3f},limit={:.3f},margin={:+.3f})m "
                        "contactHip=(pred=({:+.3f},{:+.3f},{:+.3f}),"
                        "supportVel=({:+.3f},{:+.3f})) "
                        "counterfactual=(ankleClamp={:.1f}deg,bestSwivel={:+.1f}deg,"
                        "closure={:.3f}m,accepted={}) "
                        "separation=(raw={:.3f},candidate={:.3f},admitted={:.3f},"
                        "max={:.3f},limited={})m "
                        "grounded={} accepted={} constraint={}",
                        comp._gaitTurnPlan.swingFootLeft
                            ? "LEFT" : "RIGHT",
                        turnPlan.outsideFoot ? "yes" : "no",
                        turnPlan.turnInitiationDeferred ? "yes" : "no",
                        turnPlan.turnExitBlendApplied ? "yes" : "no",
                        glm::degrees(turnPlan.desiredHeadingError),
                        TurnObjectiveName(turnPlan.objective),
                        turnPlan.turnInitiationDeferred
                            ? "measured-role-swap"
                            : (std::abs(turnPlan.admittedYaw) <= 1e-6f
                            ? "measured-walk"
                            : (turnPlan.outsideFoot
                                ? "measured-outside"
                                : "measured-inside")),
                        turnPlan.nominalAdvance,
                        turnPlan.headingAdvanceScale,
                        glm::degrees(turnPlan.requestedYaw),
                        glm::degrees(turnPlan.admittedYaw),
                        glm::degrees(turnPlan.achievedYaw),
                        turnPlan.requestedAdvance,
                        turnPlan.admittedAdvance,
                        turnPlan.achievedAdvance,
                        legacyStartFrameAdvance,
                        turnPlan.pairBudgetApplied ? "yes" : "no",
                        turnPlan.pairBudgetLatched ? "yes" : "no",
                        turnPlan.pairAdvanceScale,
                        turnPlan.pairYawScale,
                        turnPlan.supportTransportSpeed,
                        turnPlan.requestedSwingDistance,
                        turnPlan.admittedSwingDistance,
                        turnPlan.requiredSwingSpeed,
                        turnPlan.admittedSwingSpeed,
                        turnPlan.swingSpeedLimit,
                        turnPlan.minimumLaneSwingSpeed,
                        turnPlan.swingSpeedClosureTolerance,
                        turnPlan.swingSpeedLaneFloorApplied ? "yes" : "no",
                        turnPlan.swingSpeedLaneFloorExceeded ? "YES" : "no",
                        turnPlan.requiredAngularSpeed,
                        turnPlan.admittedAngularSpeed,
                        turnPlan.angularSpeedLimit,
                        turnPlan.requestedReach,
                        turnPlan.admittedReach,
                        turnPlan.reachLimit,
                        turnPlan.reachMargin,
                        turnPlan.predictedContactHip.x,
                        turnPlan.predictedContactHip.y,
                        turnPlan.predictedContactHip.z,
                        turnPlan.predictedContactSupportVelocity.x,
                        turnPlan.predictedContactSupportVelocity.z,
                        turnPlan.sampledAnkleClampDeg,
                        turnPlan.admittedKneeSwivelDeg,
                        turnPlan.sampledSwivelClosure,
                        turnPlan.sampledSwivelAccepted ? "yes" : "no",
                        turnPlan.rawFootSeparation,
                        turnPlan.candidateFootSeparation,
                        turnPlan.admittedFootSeparation,
                        turnPlan.maximumFootSeparation,
                        turnPlan.footSeparationLimited ? "yes" : "no",
                        turnPlan.candidateGrounded ? "yes" : "no",
                        turnPlan.candidateAccepted ? "yes" : "no",
                        TurnConstraintName(turnPlan.limitingConstraint));
                }
                return footholdAccepted;
            }
            const bool footholdAccepted = plannedForward >= 0.149f;
            turnPlan.candidateAccepted = footholdAccepted;
            if (!footholdAccepted)
                turnPlan.limitingConstraint =
                    Comp::TurnConstraintResult::MinimumAdvance;
            return footholdAccepted;
        };

        auto commitActiveTurnHeading = [&]() {
            auto& turnPlan = comp._gaitTurnPlan;
            if (!turnPlan.activeHeadingPlan) return;

            const glm::vec3 previousCommitted = turnPlan.committedForward;
            const float remainingBefore = resolvedHeadingDelta(
                previousCommitted, turnPlan.desiredForward);
            const bool yawBearingStep =
                std::abs(turnPlan.admittedYaw) > 1e-6f;
            if (std::abs(turnPlan.admittedYaw) > 1e-6f) {
                turnPlan.achievedYaw = turnPlan.admittedYaw
                    - glm::radians(comp._gaitHeadingErrorDeg);
                turnPlan.achievedTurnProgress =
                    turnPlan.achievedYaw / turnPlan.admittedYaw;
            } else {
                turnPlan.achievedYaw = 0.0f;
                turnPlan.achievedTurnProgress = 0.0f;
            }
            // Commit only rotation the physical controller actually achieved. Advancing
            // the logical basis to activeEndForward while the body was still 1-2 degrees
            // behind on every step accumulated several degrees of hidden turn debt, then
            // launched the exit stride while the pelvis was still catching up.
            const float committedTurnProgress = yawBearingStep
                ? glm::clamp(turnPlan.achievedTurnProgress, 0.0f, 1.0f)
                : 1.0f;
            const float physicallyCommittedYaw = turnPlan.admittedYaw
                * committedTurnProgress;
            glm::quat committedRotation = yawBearingStep
                ? glm::normalize(glm::angleAxis(
                    physicallyCommittedYaw, glm::vec3(0.0f, 1.0f, 0.0f))
                    * turnPlan.activeStartRotation)
                : glm::normalize(turnPlan.activeEndRotation);
            glm::vec3 committedForward = horizontalForward(committedRotation);
            float remainingAfter = resolvedHeadingDelta(
                committedForward, turnPlan.desiredForward);
            // Sub-degree residuals are below the useful resolution of a physical gait
            // step. Finish them at the measured handoff instead of scheduling another
            // turn-conditioned stride whose translation dominates its tiny yaw.
            constexpr float kMeasuredHeadingCommitToleranceDeg = 1.0f;
            if (yawBearingStep
                && std::abs(glm::degrees(remainingAfter))
                    <= kMeasuredHeadingCommitToleranceDeg) {
                committedForward = turnPlan.desiredForward;
                const float desiredYaw = std::atan2(
                    -committedForward.x, -committedForward.z);
                committedRotation = glm::angleAxis(
                    desiredYaw, glm::vec3(0.0f, 1.0f, 0.0f));
                remainingAfter = 0.0f;
            }
            glm::vec3 committedRight = glm::cross(
                committedForward, glm::vec3(0.0f, 1.0f, 0.0f));
            makeHorizontalBasis(committedRight, committedForward);
            turnPlan.stanceReferenceRotationDeltaDeg = rotationDifferenceDeg(
                turnPlan.stanceFootRotationAtAdmission,
                stance->plantedFootWorldRotation);
            turnPlan.plantReferenceRotationDeltaDeg = rotationDifferenceDeg(
                turnPlan.admittedFootRotation,
                swing->plantedFootWorldRotation);
            turnPlan.plannedTurnProgress = 1.0f;
            turnPlan.committedForward = committedForward;
            turnPlan.desiredHeadingError = remainingAfter;
            turnPlan.activeHeadingPlan = false;

            if (std::abs(turnPlan.admittedYaw) > 1e-6f) {
                // Arm exactly one conditioned zero-yaw step. The latch survives all
                // remaining turn steps and is consumed only by a committed exit blend.
                comp._gaitTurnExitBlendPending = true;
            } else if (turnPlan.turnExitBlendApplied) {
                comp._gaitTurnExitBlendPending = false;
            }

            comp._physicalStepForward = committedForward;
            comp._physicalStepRight = committedRight;
            comp._gaitHeadingTargetRot = glm::normalize(
                committedRotation);
            rag.locomotionTargetRot = comp._gaitHeadingTargetRot;
            const float committedYaw = std::atan2(
                -comp._physicalStepForward.x,
                -comp._physicalStepForward.z);
            comp._yaw = committedYaw - glm::radians(comp.facingOffsetDeg);
            if (comp._gaitPhysicalReversalActive
                && std::abs(turnPlan.admittedYaw) > 1e-6f) {
                ++comp._gaitReversalStepCount;
                if (comp.debug) {
                    spdlog::info(
                        "[LocomotionReversal] event=PROGRESS step={} count={} "
                        "swing={} yaw={:+.3f}deg remaining={:+.3f}deg",
                        comp._stepSequenceStepIndex,
                        comp._gaitReversalStepCount,
                        turnPlan.swingFootLeft ? "LEFT" : "RIGHT",
                        glm::degrees(turnPlan.admittedYaw),
                        glm::degrees(remainingAfter));
                }
                if (std::abs(glm::degrees(remainingAfter)) < 0.5f)
                    clearPhysicalReversal("heading-converged");
            }

            if (comp.debug) {
                const bool errorDecreased = comp._gaitStopRequested
                    || std::abs(remainingAfter)
                        <= std::abs(remainingBefore) + glm::radians(0.01f);
                spdlog::info(
                    "[LocomotionTurnCommit] step={} swing={} outside={} "
                    "startDeferred={} exitBlend={} "
                    "objective={} advance=({:.3f}->{:.3f}m,translation={}) "
                    "yaw=(admitted={:+.3f},achieved={:+.3f})deg "
                    "progress={:.3f} angular={} remaining=({:+.3f}->{:+.3f})deg "
                    "decreased={} drift=(stance={:.3f},plant={:.3f})m "
                    "referenceRotation=(stanceDelta={:.4f},plantDelta={:.4f})deg "
                    "touchdown=(sole={:.1f}deg,angular={:.3f}radps,"
                    "horizontal={:.3f}mps) "
                    "swingPeak=(linear={:.3f},angular={:.3f})",
                    comp._stepSequenceStepIndex,
                    turnPlan.swingFootLeft ? "LEFT" : "RIGHT",
                    turnPlan.outsideFoot ? "yes" : "no",
                    turnPlan.turnInitiationDeferred ? "yes" : "no",
                    turnPlan.turnExitBlendApplied ? "yes" : "no",
                    TurnObjectiveName(turnPlan.objective),
                    turnPlan.admittedAdvance,
                    turnPlan.achievedAdvance,
                    turnPlan.translationObjectiveSatisfied ? "yes" : "NO",
                    glm::degrees(turnPlan.admittedYaw),
                    glm::degrees(turnPlan.achievedYaw),
                    turnPlan.achievedTurnProgress,
                    turnPlan.angularObjectiveSatisfied ? "yes" : "NO",
                    glm::degrees(remainingBefore),
                    glm::degrees(remainingAfter),
                    errorDecreased ? "yes" : "NO",
                    comp._physicalStepMaxStanceDrift,
                    comp._physicalStepMaxPlantDrift,
                    turnPlan.stanceReferenceRotationDeltaDeg,
                    turnPlan.plantReferenceRotationDeltaDeg,
                    turnPlan.touchdownSoleErrorDeg,
                    turnPlan.touchdownAngularSpeed,
                    turnPlan.touchdownHorizontalSpeed,
                    turnPlan.achievedSwingSpeed,
                    turnPlan.achievedAngularSpeed);
                if (turnPlan.shadowFeasibilityEvaluated
                    && !turnPlan.shadowFeasibilitySafe) {
                    spdlog::warn(
                        "[LocomotionTurnFeasibilityMismatch] "
                        "result=SHADOW_FALSE_REJECTION actual=COMMITTED "
                        "step={} swing={} outside={} yaw={:+.3f}deg "
                        "constraint={} tracking=({:.3f}->{:.3f}/{:.3f}mps,"
                        "clampLoss={:.3f}mps,reference={:.3f}mps,gate=disabled) "
                        "ankleClamp=(swing={:.2f}/{:.2f},"
                        "stance={:.2f}/{:.2f})deg "
                        "action=fix-predictor-before-enabling-gate",
                        comp._stepSequenceStepIndex,
                        turnPlan.swingFootLeft ? "LEFT" : "RIGHT",
                        turnPlan.outsideFoot ? "yes" : "no",
                        glm::degrees(turnPlan.admittedYaw),
                        TurnConstraintName(
                            turnPlan.shadowLimitingConstraint),
                        turnPlan.shadowRequestedSwingSpeed,
                        turnPlan.shadowAdmittedSwingSpeed,
                        turnPlan.shadowSwingSpeedLimit,
                        turnPlan.shadowSwingSpeedClampLoss,
                        turnPlan.shadowSwingSpeedClampReference,
                        turnPlan.shadowSwing.ankleClampDeg,
                        turnPlan.shadowSwing.ankleHardClampDeg,
                        turnPlan.shadowStance.ankleClampDeg,
                        turnPlan.shadowStance.ankleHardClampDeg);
                }
            }
        };

        auto beginCommittedSupportTransport = [&]() {
            glm::vec3 committedForward =
                comp._gaitTurnPlan.committedForward;
            committedForward.y = 0.0f;
            if (glm::dot(committedForward, committedForward) < 1e-8f)
                committedForward = comp._physicalStepForward;
            committedForward = glm::normalize(committedForward);

            glm::vec3 incomingVelocity =
                comp._gaitSupportCommandVelocity;
            incomingVelocity.y = 0.0f;
            const float incomingSpeed = glm::length(incomingVelocity);
            if (incomingSpeed > configuredSupportMaxSpeed
                && incomingSpeed > 1e-6f) {
                incomingVelocity *=
                    configuredSupportMaxSpeed / incomingSpeed;
            }
            const float oldForwardComponent = glm::dot(
                incomingVelocity, committedForward);
            const float configuredTransportSpeed = glm::clamp(
                comp.gaitSupportTransportSpeed, 0.03f, 1.0f);
            const float maximumTransportSpeed = glm::max(
                glm::min(configuredSupportMaxSpeed,
                         configuredTransportSpeed), 0.03f);
            const float baseTransportSpeed = glm::clamp(
                glm::max(oldForwardComponent,
                         glm::min(continuousCommand.desiredSpeed,
                                  configuredTransportSpeed)),
                0.03f,
                maximumTransportSpeed);
            const float remainingHeadingError = resolvedHeadingDelta(
                committedForward, comp._gaitTurnPlan.desiredForward);
            const float headingAdvanceScale =
                turnAdvanceScaleForHeadingError(remainingHeadingError);
            const float transportSpeed =
                baseTransportSpeed * headingAdvanceScale;
            const glm::vec3 outgoingVelocity =
                committedForward * transportSpeed;

            // This segment begins at the exact committed command state. Its start tangent
            // is therefore C1 with transfer, while its endpoint and cruise tangent are in
            // the newly committed walking frame instead of the old foot-to-foot chord.
            comp._gaitSupportCurveActive = true;
            comp._gaitSupportCurveStep =
                comp._stepSequenceStepIndex;
            comp._gaitSupportCurveStart =
                comp._physicalStepSupportTarget;
            comp._gaitSupportCurveStartVelocity = incomingVelocity;
            comp._gaitSupportCurveTime = 0.0f;
            comp._gaitSupportCurveDuration = glm::max(
                cadenceSwingTime + cadenceArrivalSettleTime, 0.30f);
            comp._gaitSupportCurveEnd =
                comp._gaitSupportCurveStart
                + outgoingVelocity * comp._gaitSupportCurveDuration;
            comp._gaitSupportCurveEnd.y =
                comp._gaitSupportCurveStart.y;
            comp._gaitSupportCurveEndVelocity = outgoingVelocity;

            const glm::vec3 oldLateral = incomingVelocity
                - committedForward * oldForwardComponent;
            spdlog::info(
                "[LocomotionGait] SUPPORT_TRANSPORT_REBASE step={} "
                "heading=({:+.3f},{:+.3f}) remaining={:+.1f}deg "
                "advanceScale={:.3f} duration={:.3f}s "
                "velocity=({:+.3f},{:+.3f})->({:+.3f},{:+.3f})mps "
                "oldLateral={:.3f}mps",
                comp._stepSequenceStepIndex,
                committedForward.x,
                committedForward.z,
                glm::degrees(remainingHeadingError),
                headingAdvanceScale,
                comp._gaitSupportCurveDuration,
                incomingVelocity.x,
                incomingVelocity.z,
                outgoingVelocity.x,
                outgoingVelocity.z,
                glm::length(oldLateral));
        };

        auto abortSequence = [&](const char* reason) {
            if (comp._physicalStepPhase == kAbort || comp._physicalStepPhase == kIdle) return;
            const int abortedPhase = comp._physicalStepPhase;
            const bool abortedTurnExitBlend = continuousEnabled
                && comp._gaitTurnPlan.activeHeadingPlan
                && comp._gaitTurnPlan.turnExitBlendApplied
                && std::abs(comp._gaitTurnPlan.admittedYaw) <= 1e-6f;
            if (continuousEnabled
                && comp._gaitTurnPlan.candidateEvaluated) {
                comp._gaitTurnPlan.actualContactHip =
                    physicalPosition(swing->hipIdx);
                comp._gaitTurnPlan.contactHipError =
                    comp._gaitTurnPlan.actualContactHip
                    - comp._gaitTurnPlan.predictedContactHip;
            }
            // Once both feet are down, recover around the achieved world-space support
            // point. Returning to the step-entry COM made a successful-looking landing
            // slide backward before the abort became visible to the player.
            if (comp._physicalStepContactL && comp._physicalStepContactR) {
                comp._physicalStepFootBaselineL = leftFoot;
                comp._physicalStepFootBaselineR = rightFoot;
                comp._physicalStepComBaseline = comp._physicalStepSupportTarget;
                comp._physicalStepComCommand = 0.0f;
                comp._physicalStepComLateral = 0.0f;
                comp._physicalStepTargetLateral = 0.0f;
            }
            comp._physicalStepAborted = true;
            comp._physicalStepPhase = kAbort;
            comp._physicalStepPhaseTime = 0.0f;
            comp._physicalStepSettleTime = 0.0f;
            comp._physicalStepAirborneTime = 0.0f;
            if (continuousEnabled) {
                comp._gaitTurnPlan.activeHeadingPlan = false;
                // Exit conditioning is a one-shot re-spacing request. If its step fails,
                // recovery must replan from the measured stance as ordinary walking
                // instead of replaying the same failed transition until retry lockout.
                if (abortedTurnExitBlend) {
                    comp._gaitTurnExitBlendPending = false;
                    spdlog::info(
                        "[LocomotionTurnExit] action=CONSUME_ON_ABORT "
                        "phase={} reason={} next=measured-walk",
                        abortedPhase, reason);
                }
                comp._gaitTurnPairPendingInside = false;
                comp._gaitTurnPairAdvanceScale = 1.0f;
                comp._gaitTurnPairYawScale = 1.0f;
                comp._gaitTurnPairYawSign = 0.0f;
                comp._gaitTurnCancellationUnwindActive = false;
                comp._gaitTurnCancellationProgress = 0.0f;
                comp._gaitRetargetReferenceValid = false;
                comp._gaitRunning = false;
                comp._gaitContinuousCycle = false;
                comp._gaitBypassWeightShift = false;
                comp._gaitNewSupportLoad = 0.0f;
                comp._gaitSupportCurveActive = false;
                comp._gaitSupportCurveStep = -1;
                comp._gaitSupportCommandVelocity = glm::vec3(0.0f);
                clearPhysicalReversal("step-abort");
            }
            if (gameplayCommand) {
                constexpr int kMaximumAutomaticRetries = 2;
                constexpr float kAutomaticRetryCooldown = 0.35f;
                comp._runtimeRecoveryStableTime = 0.0f;
                if (comp._runtimeRestartBlockForwardValid) {
                    comp._runtimeRestartBlocked = true;
                    comp._runtimeRecoveryCooldown = 0.0f;
                    spdlog::warn(
                        "[LocoRuntime] RESTART_BLOCKED "
                        "reason=rejected-direction-stop-abort "
                        "action=release-or-change-direction-before-retry");
                } else {
                    ++comp._runtimeAutoRetryCount;
                    comp._runtimeRecoveryCooldown = kAutomaticRetryCooldown;
                    comp._runtimeRestartBlocked =
                        comp._runtimeAutoRetryCount > kMaximumAutomaticRetries;
                    if (comp._runtimeRestartBlocked) {
                        spdlog::warn(
                            "[LocoRuntime] RESTART_BLOCKED reason=retry-limit "
                            "attempts={} action=release-movement-before-retry",
                            comp._runtimeAutoRetryCount);
                    } else {
                        spdlog::warn(
                            "[LocoRuntime] AUTO_RETRY_QUEUED attempt={}/{} "
                            "cooldown={:.2f}s stableGate=0.25s "
                            "action=resume-held-intent-after-recovery",
                            comp._runtimeAutoRetryCount,
                            kMaximumAutomaticRetries,
                            kAutomaticRetryCooldown);
                    }
                }
            }
            spdlog::warn(
                "[LocomotionStep] ABORT {} centerRise={:.3f} "
                "soleClear={:.3f} penetration={:+.4f} forward={:.3f} "
                "contact=(phase={},raw={},touchdown={},support={},grounded={},near={}) "
                "target=(h={:.3f},fwd={:+.3f},lat={:+.3f},y={:.3f}) "
                "normalY={:.2f} vy=(api={:+.3f},fd={:+.3f}) upY={:+.2f} "
                "contactLocal=({:+.3f},{:+.3f},{:+.3f}) "
                "drift=({:.3f},{:.3f}) tilt={:.1f}",
                reason, comp._physicalStepClearance,
                comp._physicalStepSoleClearance,
                comp._physicalStepContactPenetration,
                comp._physicalStepForwardTravel,
                swingContactNow ? "yes" : "no",
                rawSwingContact ? "yes" : "no",
                swingTouchdownContactNow ? "yes" : "no",
                swingMaintainedSupportNow ? "yes" : "no",
                FootGrounded(rag, swing->footIdx) ? "yes" : "no",
                nearCapturedSupportSurface ? "yes" : "no",
                comp._physicalStepHorizontalTargetError,
                comp._physicalStepForwardTargetError,
                comp._physicalStepLateralTargetError,
                comp._physicalStepVerticalTargetError,
                contactNormal.y, swingVelocity.y,
                comp._physicalStepMeasuredVelocity.y, comp._physicalStepFootUpY,
                comp._physicalStepContactLocal.x, comp._physicalStepContactLocal.y,
                comp._physicalStepContactLocal.z, comp._physicalStepStanceDrift,
                comp._physicalStepPlantDrift, tiltDeg);
            if (continuousEnabled
                && comp._gaitTurnPlan.candidateEvaluated) {
                const auto& turnPlan = comp._gaitTurnPlan;
                if (turnPlan.shadowFeasibilityEvaluated
                    && turnPlan.shadowFeasibilitySafe
                    && std::abs(turnPlan.admittedYaw) > 1e-6f) {
                    spdlog::warn(
                        "[LocomotionTurnFeasibilityMismatch] "
                        "result=SHADOW_FALSE_ACCEPTANCE actual=ABORT "
                        "step={} phase={} swing={} outside={} yaw={:+.3f}deg "
                        "tracking=({:.3f}->{:.3f}/{:.3f}mps,"
                        "clampLoss={:.3f}mps,reference={:.3f}mps,gate=disabled) "
                        "abortReason={} "
                        "action=retain-shadow-and-expand-predictor",
                        comp._stepSequenceStepIndex,
                        abortedPhase,
                        turnPlan.swingFootLeft ? "LEFT" : "RIGHT",
                        turnPlan.outsideFoot ? "yes" : "no",
                        glm::degrees(turnPlan.admittedYaw),
                        turnPlan.shadowRequestedSwingSpeed,
                        turnPlan.shadowAdmittedSwingSpeed,
                        turnPlan.shadowSwingSpeedLimit,
                        turnPlan.shadowSwingSpeedClampLoss,
                        turnPlan.shadowSwingSpeedClampReference,
                        reason);
                } else if (turnPlan.shadowFeasibilityEvaluated
                           && !turnPlan.shadowFeasibilitySafe
                           && std::abs(turnPlan.admittedYaw) > 1e-6f) {
                    spdlog::info(
                        "[LocomotionTurnFeasibilityOutcome] "
                        "result=SHADOW_REJECTION_CONFIRMED actual=ABORT "
                        "step={} phase={} swing={} outside={} yaw={:+.3f}deg "
                        "constraint={} tracking=({:.3f}->{:.3f}/{:.3f}mps,"
                        "clampLoss={:.3f}mps,reference={:.3f}mps,gate=disabled) "
                        "abortReason={}",
                        comp._stepSequenceStepIndex,
                        abortedPhase,
                        turnPlan.swingFootLeft ? "LEFT" : "RIGHT",
                        turnPlan.outsideFoot ? "yes" : "no",
                        glm::degrees(turnPlan.admittedYaw),
                        TurnConstraintName(
                            turnPlan.shadowLimitingConstraint),
                        turnPlan.shadowRequestedSwingSpeed,
                        turnPlan.shadowAdmittedSwingSpeed,
                        turnPlan.shadowSwingSpeedLimit,
                        turnPlan.shadowSwingSpeedClampLoss,
                        turnPlan.shadowSwingSpeedClampReference,
                        reason);
                }
                spdlog::warn(
                    "[LocomotionTurnAbort] phase={} swing={} outside={} "
                    "yaw={:+.3f}deg constraint={} "
                    "pair=(applied={},advanceScale={:.3f},yawScale={:.3f},"
                    "support={:.3f}mps,arrivalT={:.3f}) "
                    "hip=(pred=({:+.3f},{:+.3f},{:+.3f}),"
                    "actual=({:+.3f},{:+.3f},{:+.3f}),"
                    "error=({:+.3f},{:+.3f},{:+.3f}))m "
                    "swingSpeed=(required={:.3f},admitted={:.3f},"
                    "limit={:.3f},achievedPeak={:.3f})mps "
                    "laneFloor=(minimum={:.3f},tolerance={:.3f},"
                    "applied={},exceeded={}) "
                    "angular=(required={:.3f},admitted={:.3f},limit={:.3f},"
                    "achievedPeak={:.3f})radps "
                    "governor=(trajectoryT={:.3f},linear={:.3f}mps,"
                    "tracking={:.3f}m,angular={:.3f}radps,"
                    "soleTracking={:.1f}deg,"
                    "ankleClamp={:.1f}deg) "
                    "ankle=(parentComp={:.1f}->{:.1f}deg,localStep={:.2f}deg,"
                    "localRate={:.3f}radps,residual={:.1f}deg) "
                    "jointAdmission=(scale={:.3f},requested=({:.1f},{:.1f},{:.1f})deg) "
                    "counterfactual=(planClamp={:.1f}deg,bestClamp={:.1f}deg,"
                    "bestSwivel={:+.1f}deg,closure={:.3f}m,accepted={},"
                    "applied={:+.1f}deg,priority={:.2f},relaxed={:.1f}deg,"
                    "swingReserve={:+.2f},twistMargin={:+.1f}deg) "
                    "fk=(valid={},ik={:.3f}m/{:.1f}deg,"
                    "motor={:.3f}m/{:.1f}deg,total={:.3f}m/{:.1f}deg) "
                    "path=(length={:.3f}m,minimum={:.3f}s,deadline={:.3f}s)",
                    abortedPhase,
                    turnPlan.swingFootLeft ? "LEFT" : "RIGHT",
                    turnPlan.outsideFoot ? "yes" : "no",
                    glm::degrees(turnPlan.admittedYaw),
                    TurnConstraintName(turnPlan.limitingConstraint),
                    turnPlan.pairBudgetApplied ? "yes" : "no",
                    turnPlan.pairAdvanceScale,
                    turnPlan.pairYawScale,
                    turnPlan.supportTransportSpeed,
                    turnPlan.arrivalTrajectoryT,
                    turnPlan.predictedContactHip.x,
                    turnPlan.predictedContactHip.y,
                    turnPlan.predictedContactHip.z,
                    turnPlan.actualContactHip.x,
                    turnPlan.actualContactHip.y,
                    turnPlan.actualContactHip.z,
                    turnPlan.contactHipError.x,
                    turnPlan.contactHipError.y,
                    turnPlan.contactHipError.z,
                    turnPlan.requiredSwingSpeed,
                    turnPlan.admittedSwingSpeed,
                    turnPlan.swingSpeedLimit,
                    turnPlan.achievedSwingSpeed,
                    turnPlan.minimumLaneSwingSpeed,
                    turnPlan.swingSpeedClosureTolerance,
                    turnPlan.swingSpeedLaneFloorApplied ? "yes" : "no",
                    turnPlan.swingSpeedLaneFloorExceeded ? "YES" : "no",
                    turnPlan.requiredAngularSpeed,
                    turnPlan.admittedAngularSpeed,
                    turnPlan.angularSpeedLimit,
                    turnPlan.achievedAngularSpeed,
                    comp._physicalStepTrajectoryT,
                    comp._gaitSwingCommandSpeed,
                    comp._gaitSwingCommandTrackingError,
                    comp._gaitSwingCommandAngularSpeed,
                    comp._gaitSwingSoleCommandErrorDeg,
                    comp._gaitIkAnkleEnvelopeClampDeg,
                    comp._gaitAnkleParentCompensationDeg,
                    comp._gaitAnkleParentCompensationAppliedDeg,
                    comp._gaitAnkleCommandDeltaDeg,
                    comp._gaitAnkleCommandAngularSpeed,
                    comp._gaitAnkleAchievableResidualDeg,
                    comp._gaitCoherentJointScale,
                    comp._gaitCoherentHipDeltaDeg,
                    comp._gaitCoherentKneeDeltaDeg,
                    comp._gaitCoherentAnkleDeltaDeg,
                    turnPlan.sampledAnkleClampDeg,
                    comp._gaitIkExactAnkleClampDeg,
                    comp._gaitIkKneeSwivelRequestedDeg,
                    comp._gaitIkKneeSwivelCandidateClosure,
                    comp._gaitIkKneeSwivelCandidateAccepted ? "yes" : "no",
                    comp._gaitIkKneeSwivelAppliedDeg,
                    comp._gaitIkOrientationPriority,
                    comp._gaitIkOrientationRelaxedDeg,
                    comp._gaitIkAnkleSwingReserve,
                    comp._gaitIkAnkleTwistMarginDeg,
                    comp._gaitFkValid ? "yes" : "no",
                    glm::length(comp._gaitFkDesiredToCommand),
                    comp._gaitFkDesiredToCommandOrientationDeg,
                    glm::length(comp._gaitFkCommandToPhysical),
                    comp._gaitFkCommandToPhysicalOrientationDeg,
                    glm::length(comp._gaitFkDesiredToPhysical),
                    comp._gaitFkDesiredToPhysicalOrientationDeg,
                    comp._gaitSwingPathLength,
                    comp._gaitSwingMinimumDuration,
                    comp._gaitSwingDeadline);
            }
            if (continuousEnabled && comp._gaitIkPlanHipValid
                && abortedPhase >= kTakeoff
                && abortedPhase <= kSupportReady) {
                spdlog::warn(
                    "[LocomotionGait] IK_ABORT phase={} "
                    "reach=(requested={:.3f},clamped={:.3f},max={:.3f},physical={:.3f}) "
                    "reachShortfall=({:.3f}m,fwd={:+.3f}m) "
                    "hipMove=(fwd={:+.3f},lat={:+.3f},y={:+.3f})m "
                    "envelopeClamp=(hip={:.1f},ankle={:.1f})deg "
                    "commandLag=(hip={:.1f},knee={:.1f})deg "
                    "kneeBend={:.1f}deg footError=(h={:.3f},fwd={:+.3f},lat={:+.3f})m "
                    "footControl=(corr={:.3f}m,fwd={:+.3f}m,"
                    "targetSpeed={:.3f}m/s,level={:.2f})",
                    abortedPhase,
                    comp._gaitIkRequestedReach,
                    comp._gaitIkClampedReach,
                    comp._gaitIkMaxReach,
                    comp._gaitIkPhysicalReach,
                    comp._gaitIkReachShortfall,
                    comp._gaitIkReachShortfallForward,
                    comp._gaitIkHipTravelForward,
                    comp._gaitIkHipTravelLateral,
                    comp._gaitIkHipTravelVertical,
                    comp._gaitIkHipEnvelopeClampDeg,
                    comp._gaitIkAnkleEnvelopeClampDeg,
                    comp._gaitIkHipCommandLagDeg,
                    comp._gaitIkKneeCommandLagDeg,
                    comp._gaitIkKneeBendDeg,
                    comp._physicalStepHorizontalTargetError,
                    comp._physicalStepForwardTargetError,
                    comp._physicalStepLateralTargetError,
                    comp._gaitFootCorrection,
                    comp._gaitFootCorrectionForward,
                    comp._gaitFootTargetSpeed,
                    comp._gaitSoleLevelBlend);
            }
        };

        auto requestRejectedReversalStop = [&](const char* admissionSite) {
            auto& turnPlan = comp._gaitTurnPlan;
            if (!continuousEnabled || !gameplayCommand
                || !comp._gaitPhysicalReversalActive
                || !turnPlan.candidateEvaluated
                || turnPlan.candidateAccepted)
                return false;

            comp._gaitStopRequested = true;
            comp._gaitLandingObjectiveStopRequested = false;
            comp._runtimeRestartBlocked = true;
            comp._runtimeRestartBlockForwardValid = true;
            comp._runtimeRestartBlockForward = turnPlan.desiredForward;

            // captureSwing only released logical ownership; takeoff has not happened.
            // Restore the measured sole to its existing plant so the double-support stop
            // begins from two owned contacts instead of an artificial airborne leg.
            swing->planted = true;
            comp._physicalStepDesiredFoot = swingFoot;
            comp._gaitSwingSoleCommandValid = false;
            turnPlan.activeHeadingPlan = false;
            comp._gaitTurnPairPendingInside = false;
            comp._gaitTurnPairAdvanceScale = 1.0f;
            comp._gaitTurnPairYawScale = 1.0f;
            comp._gaitTurnPairYawSign = 0.0f;

            spdlog::warn(
                "[LocomotionReversal] event=PLAN_REJECT site={} "
                "step={} swing={} requestedYaw={:+.3f}deg "
                "constraint={} swingSpeed={:.3f}/{:.3f}mps "
                "angularSpeed={:.3f}/{:.3f}radps "
                "action=controlled-stop-block-unchanged-input",
                admissionSite,
                comp._stepSequenceStepIndex,
                turnPlan.swingFootLeft ? "LEFT" : "RIGHT",
                glm::degrees(turnPlan.requestedYaw),
                TurnConstraintName(turnPlan.limitingConstraint),
                turnPlan.requiredSwingSpeed,
                turnPlan.swingSpeedLimit,
                turnPlan.requiredAngularSpeed,
                turnPlan.angularSpeedLimit);
            clearPhysicalReversal("planner-rejection");
            return true;
        };

        const float comError = comp._physicalStepTargetLateral - comp._physicalStepComLateral;
        const float forwardComError = glm::dot(
            comp._physicalStepSupportTarget - rag._locomotionCOM,
            comp._physicalStepForward);
        const float forwardComSpeed = glm::dot(
            rag._locomotionCOMVel, comp._physicalStepForward);
        constexpr float kWeightShiftForwardTolerance = 0.015f;
        constexpr float kWeightShiftForwardSpeedTolerance = 0.010f;
        const float weightShiftMinimumTime = cadenceWeightShiftTime;
        // Valid shifts in the current 1.5 Hz setup can need a little over three
        // seconds to settle. Leave margin above that observed range, but never wait
        // forever on an unreachable predicate again.
        const float gaitWeightShiftTimeout = glm::max(
            weightShiftMinimumTime * 3.0f, weightShiftMinimumTime + 3.0f);
        const bool weightShiftCommandReady =
            std::abs(comp._physicalStepComCommand - comp._physicalStepSupportSide) < 0.01f;
        const bool weightShiftLateralReady = std::abs(comError) < 0.01f;
        const bool weightShiftForwardReady = !continuousEnabled
            || std::abs(forwardComError) < kWeightShiftForwardTolerance;
        const bool weightShiftForwardSpeedReady = !continuousEnabled
            || std::abs(forwardComSpeed)
                < kWeightShiftForwardSpeedTolerance;
        const bool weightShiftContactsReady =
            comp._physicalStepContactL && comp._physicalStepContactR;
        const bool weightShiftTimeReady =
            comp._physicalStepPhaseTime >= weightShiftMinimumTime;
        if (continuousEnabled
            && comp._physicalStepPhase == kWeightShift
            && comp._gaitBypassWeightShift
            && !comp._gaitStopRequested
            && stanceContactNow && swingContactNow) {
            // TRANSFER already moved the COM over the new stance foot and verified load
            // ownership. Re-plan the released old foot immediately; do not settle and
            // perform the same weight shift a second time.
            comp._gaitBypassWeightShift = false;
            const float releasedAnchorError = glm::length(glm::vec2(
                swingFoot.x - swing->plantFoot.x,
                swingFoot.z - swing->plantFoot.z));
            captureSwing();
            if (planFoothold()) {
                comp._physicalStepPhase = kTakeoff;
                comp._physicalStepPhaseTime = 0.0f;
                comp._physicalStepPrevSwingContact = true;
                comp._gaitTakeoffContactRecoveryTime = 0.0f;
                comp._gaitTakeoffContactRecoveryActive = false;
                spdlog::info(
                    "[LocomotionGait] REAR_RELEASE step={} anchorError={:.3f}m "
                    "load={:.2f} action=takeoff",
                    comp._stepSequenceStepIndex,
                    releasedAnchorError,
                    comp._gaitNewSupportLoad);
            } else {
                if (!requestRejectedReversalStop("continuous-handoff")) {
                    abortSequence(
                        "continuous handoff foothold lacked tracking reserve");
                }
            }
        } else if (comp._physicalStepPhase == kWeightShift
            && (!continuousEnabled || !comp._gaitStopRequested)
            && weightShiftCommandReady
            && weightShiftLateralReady
            && weightShiftForwardReady
            && weightShiftForwardSpeedReady
            && weightShiftContactsReady
            && weightShiftTimeReady) {
            captureSwing();
            if (planFoothold()) {
                comp._physicalStepPhase = kTakeoff;
                comp._physicalStepPhaseTime = 0.0f;
                comp._physicalStepPrevSwingContact = true;
                comp._gaitTakeoffContactRecoveryTime = 0.0f;
                comp._gaitTakeoffContactRecoveryActive = false;
                if (continuousEnabled) {
                }
            } else {
                if (!requestRejectedReversalStop("weight-shift")) {
                    abortSequence(continuousEnabled
                        ? "latched foothold lacked support-advance tracking reserve"
                        : "latched foothold fell below 15 cm after reach clamp");
                }
            }
        } else if (continuousEnabled
                   && comp._physicalStepPhase == kWeightShift
                   && comp._physicalStepPhaseTime >= gaitWeightShiftTimeout) {
            spdlog::warn(
                "[LocomotionGait] WEIGHT_SHIFT_TIMEOUT step={} time={:.2f}/{:.2f}s "
                "predicates=(command={} error={:+.3f}/0.010,"
                "lateral={} error={:+.3f}/0.010m,"
                "forward={} error={:+.3f}/0.015m,"
                "forwardSpeed={} speed={:+.3f}/0.010m/s,"
                "contacts={} leftContact={} rightContact={},"
                "minimumTime={} elapsed={:.2f}/{:.2f}s)",
                comp._stepSequenceStepIndex,
                comp._physicalStepPhaseTime, gaitWeightShiftTimeout,
                weightShiftCommandReady ? "PASS" : "FAIL",
                comp._physicalStepComCommand - comp._physicalStepSupportSide,
                weightShiftLateralReady ? "PASS" : "FAIL", comError,
                weightShiftForwardReady ? "PASS" : "FAIL", forwardComError,
                weightShiftForwardSpeedReady ? "PASS" : "FAIL",
                forwardComSpeed,
                weightShiftContactsReady ? "PASS" : "FAIL",
                comp._physicalStepContactL ? "PASS" : "FAIL",
                comp._physicalStepContactR ? "PASS" : "FAIL",
                weightShiftTimeReady ? "PASS" : "FAIL",
                comp._physicalStepPhaseTime, weightShiftMinimumTime);
            abortSequence("weight-shift readiness timed out");
        }

        // A stop is committed immediately instead of waiting for the complete ordinary
        // step. Before mid-swing the foot returns to its previous plant. After mid-swing
        // it continues forward to a shorter, reachable foothold and then transfers support.
        if (continuousEnabled && comp._gaitStopRequested
            && comp._gaitCancelMode == 0
            && comp._physicalStepPhase >= kTakeoff
            && comp._physicalStepPhase <= kDescent) {
            const bool inSwing = comp._physicalStepPhase == kSwing;
            const float swingProgress = inSwing
                ? glm::clamp(comp._physicalStepPhaseTime / cadenceSwingTime, 0.0f, 1.0f)
                : (comp._physicalStepPhase == kTakeoff ? 0.0f : 1.0f);
            const bool returnToPlant = comp._physicalStepPhase == kTakeoff
                || (inSwing && swingProgress < 0.50f);
            if (returnToPlant) {
                const auto& turnPlan = comp._gaitTurnPlan;
                comp._gaitTurnCancellationUnwindActive =
                    turnPlan.activeHeadingPlan;
                comp._gaitTurnCancellationStartRotation = glm::normalize(
                    comp._gaitHeadingTargetRot);
                comp._gaitTurnCancellationEndRotation =
                    turnPlan.activeHeadingPlan
                    ? glm::normalize(turnPlan.activeStartRotation)
                    : comp._gaitTurnCancellationStartRotation;
                comp._gaitTurnCancellationStartForward = horizontalForward(
                    comp._gaitTurnCancellationStartRotation);
                comp._gaitTurnCancellationEndForward = horizontalForward(
                    comp._gaitTurnCancellationEndRotation);
                comp._gaitTurnCancellationProgress = 0.0f;
                comp._gaitCancelMode = 1;
                comp._physicalStepArcStart = swingFoot;
                comp._physicalStepFoothold = comp._physicalStepSwingStart;
                comp._gaitPlannedSupportAdvance = 0.0f;
                comp._physicalStepPhase = kSwing;
                comp._physicalStepPhaseTime = 0.0f;
                comp._physicalStepTrajectoryT = 0.0f;
                comp._physicalStepArrivalStableTime = 0.0f;
                comp._gaitSwingRecontactTime = 0.0f;
                if (comp.debug) {
                    spdlog::info(
                        "[LocomotionTurnHandoff] event=CANCEL_BEGIN mode=early-return "
                        "phase={} step={} activeCommitted={} progress={:.3f} "
                        "heading=(({:+.3f},{:+.3f})->({:+.3f},{:+.3f})) "
                        "unwind={:+.3f}deg action=return-old-plant",
                        gaitPhaseName(phaseAtFrameStart),
                        comp._stepSequenceStepIndex,
                        turnPlan.activeHeadingPlan ? "yes" : "no",
                        turnPlan.plannedTurnProgress,
                        comp._gaitTurnCancellationStartForward.x,
                        comp._gaitTurnCancellationStartForward.z,
                        comp._gaitTurnCancellationEndForward.x,
                        comp._gaitTurnCancellationEndForward.z,
                        glm::degrees(signedHeadingDelta(
                            comp._gaitTurnCancellationStartForward,
                            comp._gaitTurnCancellationEndForward)));
                }
            } else {
                comp._gaitCancelMode = 2;
                const float plannedAdvance = glm::dot(
                    comp._physicalStepFoothold - stanceFoot,
                    comp._physicalStepForward);
                const float currentAdvance = glm::dot(
                    swingFoot - stanceFoot, comp._physicalStepForward);
                const float shortenedAdvance = glm::clamp(
                    glm::max(0.03f, currentAdvance + 0.02f),
                    0.03f, glm::max(plannedAdvance, 0.03f));
                comp._physicalStepFoothold += comp._physicalStepForward
                    * (shortenedAdvance - plannedAdvance);
                comp._gaitPlannedSupportAdvance = shortenedAdvance;
                if (inSwing) {
                    comp._physicalStepArcStart = swingFoot;
                    comp._physicalStepPhaseTime = 0.0f;
                    comp._physicalStepTrajectoryT = 0.0f;
                    comp._physicalStepArrivalStableTime = 0.0f;
                } else if (comp._physicalStepPhase == kArrival) {
                    comp._physicalStepPhase = kDescent;
                    comp._physicalStepPhaseTime = 0.0f;
                }
                if (comp.debug) {
                    spdlog::info(
                        "[LocomotionTurnHandoff] event=CANCEL_BEGIN mode=late-finish "
                        "phase={} step={} activeCommitted={} progress={:.3f} "
                        "activeYaw={:+.3f}deg action=finish-admitted-heading",
                        gaitPhaseName(phaseAtFrameStart),
                        comp._stepSequenceStepIndex,
                        comp._gaitTurnPlan.activeHeadingPlan ? "yes" : "no",
                        comp._gaitTurnPlan.plannedTurnProgress,
                        glm::degrees(comp._gaitTurnPlan.admittedYaw));
                }
            }
        }

        const bool governedInsideTurnSwing = hasYawBearingTurnStep()
            && !comp._gaitTurnPlan.outsideFoot
            && comp._gaitCancelMode == 0;
        // Fast straight walking and turning share the measured-state command governor,
        // but their cadence budgets remain separate. This keeps outside turns bounded by
        // the same acceleration limits used at admission while leaving the straight-walk
        // overlap and 0.50 s cadence unchanged.
        const bool governedWalkingSwing = continuousEnabled
            && comp._gaitCancelMode == 0;
        const float takeoffHeight = glm::clamp(
            comp.takeoffHeight, 0.040f,
            glm::max(comp.swingHeight, 0.041f));
        glm::vec3 desiredFoot = comp._physicalStepSwingStart;
        if (comp._physicalStepPhase == kTakeoff) {
            // The governed walking path owns a continuous Cartesian command. Jumping the
            // sole target directly to takeoffHeight while independently filtering the
            // three joint targets created 5-12 cm of desired-to-commanded FK error on the
            // first TAKEOFF frame. Ramp the task-space lift instead; the coherent IK path
            // below can then solve every intermediate sole pose exactly.
            const float coherentTakeoffRiseTime = glm::max(
                cadenceTakeoffTarget, 0.06f);
            const float takeoffLift = governedWalkingSwing
                ? takeoffHeight * smoothstep(glm::clamp(
                    comp._physicalStepPhaseTime
                        / coherentTakeoffRiseTime,
                    0.0f, 1.0f))
                : takeoffHeight;
            desiredFoot.y += takeoffLift;
            const float releaseClearance = continuousEnabled
                ? 0.010f : glm::max(0.040f, takeoffHeight * 0.75f);
            const bool recoverableTakeoffContact = continuousEnabled
                && swingContactNow
                && comp._physicalStepClearance >= 0.025f;
            if (recoverableTakeoffContact) {
                if (!comp._gaitTakeoffContactRecoveryActive) {
                }
                comp._gaitTakeoffContactRecoveryActive = true;
            } else if (comp._gaitTakeoffContactRecoveryActive
                       && !swingContactNow) {
                comp._gaitTakeoffContactRecoveryActive = false;
            }
            const bool takeoffRecoveryStarted =
                comp._gaitTakeoffContactRecoveryTime > 0.0f
                || recoverableTakeoffContact;
            if (takeoffRecoveryStarted) {
                comp._gaitTakeoffContactRecoveryTime += dt;
                // A high foot center with a live ground manifold means a toe or heel edge
                // is still down. Keep the command vertical until that edge opens instead of
                // feeding the contact into the forward swing trajectory.
                const float recoveryLift = governedWalkingSwing
                    ? 0.025f * smoothstep(glm::clamp(
                        comp._gaitTakeoffContactRecoveryTime / 0.08f,
                        0.0f, 1.0f))
                    : 0.025f;
                desiredFoot.y += recoveryLift;
            }
            const bool airborneEvidence = !swingContactNow
                && comp._physicalStepSoleClearance >= releaseClearance;
            comp._physicalStepAirborneTime = airborneEvidence
                ? comp._physicalStepAirborneTime + dt : 0.0f;
            const float requiredAirborneTime = continuousEnabled
                ? 0.012f : 0.05f;
            // Cadence sets the earliest desired release, but a live toe/heel manifold
            // never authorizes horizontal swing. One to two contact-free frames at a
            // modest center clearance are enough to reject chatter without restoring the
            // old long serialized takeoff hold.
            const bool confirmedRelease =
                comp._physicalStepPhaseTime >= coherentTakeoffRiseTime
                && comp._physicalStepAirborneTime >= requiredAirborneTime;
            if (confirmedRelease) {
                spdlog::info(
                    "[LocomotionGait] TAKEOFF_RELEASE step={} "
                    "centerRise={:.3f}m soleClear={:.3f}m penetration={:+.4f}m "
                    "contactFree={:.3f}/{:.3f}s action=swing",
                    comp._stepSequenceStepIndex,
                    comp._physicalStepClearance,
                    comp._physicalStepSoleClearance,
                    comp._physicalStepContactPenetration,
                    comp._physicalStepAirborneTime,
                    requiredAirborneTime);
                if (continuousEnabled) {
                    comp._gaitLastLaunchPeriod = comp._gaitLastLaunchValid
                        ? glm::max(
                            comp._gaitRunTime - comp._gaitLastLaunchTime, dt)
                        : 0.0f;
                    spdlog::info(
                        "[LocomotionCadence] LAUNCH step={} foot={} "
                        "period={:.3f}s target={:.3f}s error={:+.3f}s "
                        "soleClear={:.3f}m",
                        comp._stepSequenceStepIndex,
                        comp._gaitTurnPlan.swingFootLeft ? "LEFT" : "RIGHT",
                        comp._gaitLastLaunchPeriod,
                        effectiveTargetStepPeriod,
                        comp._gaitLastLaunchValid
                            ? comp._gaitLastLaunchPeriod
                                - effectiveTargetStepPeriod
                            : 0.0f,
                        comp._physicalStepSoleClearance);
                    comp._gaitLastLaunchTime = comp._gaitRunTime;
                    comp._gaitLastLaunchValid = true;
                }
                comp._physicalStepArcStart = desiredFoot;
                comp._physicalStepPhase = kSwing;
                comp._physicalStepPhaseTime = 0.0f;
                comp._physicalStepTrajectoryT = 0.0f;
                comp._physicalStepAirborneTime = 0.0f;
                comp._gaitTakeoffContactRecoveryTime = 0.0f;
                comp._gaitTakeoffContactRecoveryActive = false;
                comp._gaitSwingRecontactTime = 0.0f;
            } else {
                const float takeoffDeadline =
                    glm::max(comp.takeoffTimeout, 0.01f)
                    + (takeoffRecoveryStarted ? 0.15f : 0.0f);
                if (comp._physicalStepPhaseTime >= takeoffDeadline) {
                    abortSequence(takeoffRecoveryStarted
                        ? "takeoff contact persisted through recovery window"
                        : "takeoff did not open swing contact");
                }
            }
        }

        constexpr float kSwingApexT = 0.35f;
        constexpr float kSwingArrivalT = 0.70f;
        const bool governedTurnStep = hasYawBearingTurnStep();
        const float activeSwingLandingBrakeT = governedTurnStep
            ? kTurnSwingLandingBrakeT
            : kWalkingSwingLandingBrakeT;
        const glm::vec3 hoverTarget = comp._physicalStepFoothold
            + glm::vec3(0.0f, glm::max(comp.arrivalHeight, 0.03f), 0.0f);
        auto trajectoryPoint = [&](float t) {
            t = glm::clamp(t, 0.0f, 1.0f);
            // Straight walking front-loads translation for its short overlapping descent.
            // Turn steps use the recovered whole-step smoothstep so the lateral arc does
            // not outrun the physical leg before the body has rotated underneath it.
            const float horizontalT = governedTurnStep
                ? smoothstep(t)
                : (t <= kSwingArrivalT
                    ? kWalkingSwingHorizontalAtArrival
                        * smoothstep(t / kSwingArrivalT)
                    : (t < activeSwingLandingBrakeT
                        ? glm::mix(
                            kWalkingSwingHorizontalAtArrival, 1.0f,
                            smoothstep((t - kSwingArrivalT)
                                / (activeSwingLandingBrakeT
                                    - kSwingArrivalT)))
                        : 1.0f));
            glm::vec3 point = glm::mix(
                comp._physicalStepArcStart, comp._physicalStepFoothold,
                horizontalT);
            const float apexY = glm::max(comp._physicalStepArcStart.y, hoverTarget.y)
                + glm::max(comp.swingHeight - comp.arrivalHeight, 0.02f);
            if (t < kSwingApexT) {
                point.y = glm::mix(
                    comp._physicalStepArcStart.y, apexY,
                    smoothstep(t / kSwingApexT));
            } else if (t < kSwingArrivalT) {
                point.y = glm::mix(
                    apexY, hoverTarget.y,
                    smoothstep((t - kSwingApexT)
                               / (kSwingArrivalT - kSwingApexT)));
            } else if (!governedTurnStep
                       && t < activeSwingLandingBrakeT) {
                point.y = hoverTarget.y;
            } else {
                const float descentStartT = governedTurnStep
                    ? kSwingArrivalT : activeSwingLandingBrakeT;
                point.y = glm::mix(
                    hoverTarget.y, comp._physicalStepFoothold.y,
                    smoothstep((t - descentStartT)
                               / (1.0f - descentStartT)));
            }
            return point;
        };

        const float governedCommandAcceleration = governedTurnStep
            ? glm::clamp(comp.gaitTurnLinearAcceleration, 0.25f, 10.0f)
            : glm::clamp(comp.gaitSwingAcceleration, 1.0f, 30.0f);
        const float governedCommandMaximumSpeed = governedTurnStep
            ? (comp._gaitTurnPlan.outsideFoot
                ? glm::clamp(comp.gaitTurnOutsideSwingSpeedLimit,
                             0.45f, 2.0f)
                : glm::clamp(comp.gaitTurnInsideSwingSpeedLimit,
                             0.45f, 2.0f))
            : glm::clamp(comp.gaitSwingSpeedLimit, 0.45f, 4.0f);
        const float governedCommandSpeedLimit = glm::min(glm::max(
            comp._gaitTurnPlan.admittedSwingSpeed, 0.45f),
            governedCommandMaximumSpeed);
        auto sampledTrajectoryLength = [&](float startT, float endT,
                                            const auto& commandPoint) {
            constexpr int kSamples = 24;
            float length = 0.0f;
            glm::vec3 previous = commandPoint(startT);
            for (int sample = 1; sample <= kSamples; ++sample) {
                const float t = glm::mix(
                    startT, endT,
                    static_cast<float>(sample)
                        / static_cast<float>(kSamples));
                const glm::vec3 point = commandPoint(t);
                length += glm::length(point - previous);
                previous = point;
            }
            return length;
        };
        if (governedWalkingSwing) {
            comp._gaitSwingPathLength = sampledTrajectoryLength(
                0.0f, kSwingArrivalT, trajectoryPoint);
            const float accelerationDistance =
                governedCommandSpeedLimit * governedCommandSpeedLimit
                / (2.0f * governedCommandAcceleration);
            comp._gaitSwingMinimumDuration =
                comp._gaitSwingPathLength <= accelerationDistance
                    ? std::sqrt(2.0f * comp._gaitSwingPathLength
                        / governedCommandAcceleration)
                    : governedCommandSpeedLimit
                            / governedCommandAcceleration
                        + (comp._gaitSwingPathLength
                            - accelerationDistance)
                            / governedCommandSpeedLimit;
            // Retain the old safety window, but never make it shorter than the
            // acceleration-limited time required by the actual 3D arc. The extra
            // reserve covers tracking-tube deceleration without hiding a stalled foot.
            const float existingDeadline = cadenceSwingTime
                + glm::max(comp.arrivalTimeout, 0.25f);
            const float trackingReserve = governedTurnStep
                ? 0.45f
                : glm::max(2.0f * cadenceArrivalSettleTime, 0.08f);
            const float derivedDeadline =
                comp._gaitSwingMinimumDuration + trackingReserve;
            comp._gaitSwingDeadline = glm::max(
                existingDeadline, derivedDeadline);
        }
        auto moveScalarToward = [](float current, float target,
                                   float maximumDelta) {
            if (current < target)
                return glm::min(current + maximumDelta, target);
            return glm::max(current - maximumDelta, target);
        };
        auto advanceGovernedSwingCommand = [&](float ceilingT, float brakeT,
                                                const auto& commandPoint) {
            const float currentT = glm::clamp(
                comp._physicalStepTrajectoryT, 0.0f, 1.0f);
            ceilingT = glm::clamp(ceilingT, currentT, 1.0f);
            brakeT = glm::clamp(brakeT, ceilingT, 1.0f);
            const glm::vec3 currentCommand = commandPoint(currentT);
            const glm::vec3 brakingEndpoint = commandPoint(brakeT);

            // The trajectory clock is subordinate to physical tracking for the moving
            // foot. Full authority is retained inside 2.5 cm; beyond that, command speed
            // falls smoothly to zero by 7 cm so the plant is never asked to chase an
            // ever-receding target.
            comp._gaitSwingCommandTrackingError = glm::length(
                swingFoot - currentCommand);
            const float trackingTubeInner = governedTurnStep
                ? 0.025f : 0.035f;
            const float trackingTubeOuter = governedTurnStep
                ? 0.070f : 0.085f;
            const float trackingScale = 1.0f - smoothstep(
                (comp._gaitSwingCommandTrackingError - trackingTubeInner)
                    / (trackingTubeOuter - trackingTubeInner));

            const float brakingDistance = glm::length(
                brakingEndpoint - currentCommand);
            const float brakingSpeed = std::sqrt(glm::max(
                2.0f * governedCommandAcceleration
                    * brakingDistance,
                0.0f));
            const float desiredSpeed = glm::min(
                governedCommandSpeedLimit, brakingSpeed)
                * trackingScale;
            comp._gaitSwingCommandSpeed = moveScalarToward(
                comp._gaitSwingCommandSpeed, desiredSpeed,
                governedCommandAcceleration * dt);

            const float distanceBudget = glm::max(
                comp._gaitSwingCommandSpeed * dt, 0.0f);
            float admittedT = currentT;
            if (ceilingT > currentT + 1e-6f && distanceBudget > 1e-7f) {
                if (glm::length(commandPoint(ceilingT) - currentCommand)
                        <= distanceBudget) {
                    admittedT = ceilingT;
                } else {
                    // Arc length has no closed form for this trajectory. A bounded search
                    // gives a deterministic parameter increment whose chord cannot exceed
                    // the acceleration-limited command distance for this frame.
                    float low = currentT;
                    float high = ceilingT;
                    for (int iteration = 0; iteration < 12; ++iteration) {
                        const float candidateT = 0.5f * (low + high);
                        const float candidateDistance = glm::length(
                            commandPoint(candidateT) - currentCommand);
                        if (candidateDistance <= distanceBudget)
                            low = candidateT;
                        else
                            high = candidateT;
                    }
                    admittedT = low;
                }
            }
            comp._physicalStepTrajectoryT = admittedT;
            return commandPoint(admittedT);
        };

        auto landingHorizontalSpeedLimit = [&]() {
            const float configured = glm::clamp(
                comp.touchdownMaxHorizontalSpeed, 0.05f, 2.0f);
            return hasYawBearingTurnStep()
                ? glm::min(configured, 0.12f) : configured;
        };
        auto landingAngularSpeedLimit = [&]() {
            const float configured = glm::clamp(
                comp.touchdownMaxAngularSpeed, 0.25f, 10.0f);
            return hasYawBearingTurnStep()
                ? glm::min(configured, 0.75f) : configured;
        };
        // The brake landmark is an approach gate, not touchdown acceptance. Straight
        // walking may begin its final vertical placement while residual horizontal and
        // angular motion are still damping; evaluateTouchdown retains the stricter
        // configured limits below. Turn-conditioned steps keep their previous envelope.
        auto landingBrakeHorizontalSpeedLimit = [&]() {
            return governedTurnStep
                ? landingHorizontalSpeedLimit() : 0.85f;
        };
        auto landingBrakeAngularSpeedLimit = [&]() {
            return governedTurnStep
                ? landingAngularSpeedLimit() : 4.25f;
        };
        auto landingBrakeKinematicsReady = [&]() {
            const float horizontalSpeed = glm::length(glm::vec2(
                swingVelocity.x, swingVelocity.z));
            const float commandSpeedLimit = glm::clamp(
                comp.touchdownMaxCommandSpeed, 0.05f, 1.0f);
            return (!continuousEnabled || (
                horizontalSpeed <= landingBrakeHorizontalSpeedLimit()
                && swingAngularVelocityOk
                && glm::length(swingAngularVelocity)
                    <= landingBrakeAngularSpeedLimit()
                && comp._gaitSwingCommandSpeed <= commandSpeedLimit));
        };

        const float arrivalTolerance = glm::max(
            comp.arrivalTolerance,
            continuousEnabled ? 0.025f : 0.01f);
        const float kSoleArrivalToleranceDeg = walkingOverlapHandoff
            ? 14.0f : 10.0f;
        auto updateArrivalStability = [&](bool allowAccumulation) {
            const float arrivalVerticalError = std::abs(
                swingFoot.y - hoverTarget.y);
            const bool soleAligned = !continuousEnabled
                || comp._gaitSoleAngularErrorDeg <= kSoleArrivalToleranceDeg;
            const bool arrivalWithinTolerance = allowAccumulation
                && !swingContactNow
                && comp._physicalStepHorizontalTargetError <= arrivalTolerance
                && arrivalVerticalError <= 0.025f
                && soleAligned;
            comp._physicalStepArrivalStableTime = arrivalWithinTolerance
                ? comp._physicalStepArrivalStableTime + dt : 0.0f;
            return arrivalWithinTolerance;
        };

        auto acceptTouchdown = [&]() {
            comp._physicalStepTouchdownAccepted = true;
            comp._physicalStepTouchdownPlant = swingFoot;
            auto& turnPlan = comp._gaitTurnPlan;
            turnPlan.touchdownSoleErrorDeg = comp._gaitSoleAngularErrorDeg;
            turnPlan.touchdownAngularSpeed = swingAngularVelocityOk
                ? glm::length(swingAngularVelocity) : 0.0f;
            turnPlan.touchdownHorizontalSpeed = glm::length(glm::vec2(
                swingVelocity.x, swingVelocity.z));
            turnPlan.actualContactHip = physicalPosition(swing->hipIdx);
            turnPlan.contactHipError =
                turnPlan.actualContactHip - turnPlan.predictedContactHip;
            const glm::quat admittedPlantRotation =
                !continuousEnabled || comp._gaitCancelMode == 1
                ? glm::normalize(swing->plantedFootWorldRotation)
                : glm::normalize(comp._gaitTurnPlan.admittedFootRotation);
            swing->plantedFootWorldRotation = admittedPlantRotation;
            comp._physicalStepTouchdownContactValid = swingContactNow
                && swingRotationOk
                && std::isfinite(contactPoint.x)
                && std::isfinite(contactPoint.y)
                && std::isfinite(contactPoint.z);
            if (comp._physicalStepTouchdownContactValid) {
                comp._physicalStepTouchdownContactWorld = contactPoint;
                comp._physicalStepTouchdownContactLocal =
                    comp._physicalStepContactLocal;
                // A sole that touches on an edge must translate its center while rotating
                // flat. Seed the persistent planted-center target from the immutable world
                // contact so leveling never asks that material point to slide through the
                // floor while also holding the body center fixed.
                swing->plantFoot =
                    comp._physicalStepTouchdownContactWorld
                    - admittedPlantRotation
                        * comp._physicalStepTouchdownContactLocal;
            } else {
                swing->plantFoot = swingFoot;
                comp._physicalStepTouchdownContactWorld = glm::vec3(0.0f);
                comp._physicalStepTouchdownContactLocal = glm::vec3(0.0f);
            }
            swing->planted = true;
            comp._physicalStepTouchdownVy = swingVelocity.y;
            comp._physicalStepTouchdownNormalY = contactNormal.y;
            comp._physicalStepPlantDrift = 0.0f;
            comp._physicalStepPlantCenterTravel = 0.0f;
            comp._physicalStepMaxPlantDrift = 0.0f;
            comp._gaitPlantPreviousDrift = 0.0f;
            comp._gaitPlantDriftRate = 0.0f;
            comp._gaitPlantRecoveryLogged = false;
            comp._physicalStepPlantSettledOffsetTime = 0.0f;
            comp._physicalStepPlantUnsafeTime = 0.0f;
            comp._physicalStepPlantAnchorRebased = false;
            comp._physicalStepPlantCenterAnchorActive = false;
            comp._physicalStepPlantContactMigrationLogged = false;
            comp._physicalStepPlantPivotReleaseLatched = false;
            comp._physicalStepPlantPivotStableTime = 0.0f;
            comp._physicalStepPlantPivotMaxStableTime = 0.0f;
            comp._physicalStepPlantPivotReleaseTriggerTime = 0.0f;
            comp._physicalStepPlantPivotReleaseTime = 0.0f;
            comp._physicalStepPlantPivotReleaseWeight = 0.0f;
            comp._physicalStepPlantCenterBlendTime = 0.0f;
            comp._physicalStepPlantAnchorTelemetryTime = 0.0f;
            comp._physicalStepPlantAnchorHandoffPhaseTime = -1.0f;
            comp._physicalStepPlantPivotContactBlockedTime = 0.0f;
            comp._physicalStepPlantPivotSoleBlockedTime = 0.0f;
            comp._physicalStepPlantPivotAngularBlockedTime = 0.0f;
            comp._physicalStepPlantPivotLinearBlockedTime = 0.0f;
            comp._physicalStepPlantContactMigration = 0.0f;
            comp._physicalStepPlantAngularSpeed = 0.0f;
            comp._physicalStepPlantCenterAnchorStart = swing->plantFoot;
            comp._physicalStepPlantCenterAnchorTarget = swingFoot;
            comp._gaitNewSupportLoadLatched = false;
            comp._gaitPlantCorrectionPeakRequested = 0.0f;
            comp._gaitPlantCorrectionPeakApplied = 0.0f;
            comp._gaitPlantCorrectionSaturated = false;
            comp._gaitPlantCorrectionRequested = 0.0f;
            comp._gaitPlantCorrectionApplied = 0.0f;
            comp._gaitPlantCorrectionAtLimit = false;
            // Retain the final landing IK command briefly while contact settles. Capturing
            // the first-impact joint pose immediately allowed the sole to rock backward.
            comp._physicalStepPlantPoseCaptured = false;
            comp._physicalStepPlantAcquireStableTime = 0.0f;
            const float pivotCenterShift = glm::length(glm::vec2(
                swing->plantFoot.x - swingFoot.x,
                swing->plantFoot.z - swingFoot.z));
            spdlog::info(
                "[LocomotionGait] TOUCHDOWN_ANCHOR step={} plant={} valid={} "
                "contactLocal=({:+.3f},{:+.3f},{:+.3f}) "
                "centerComp={:.3f}m sole={:.1f}deg "
                "motion=(horizontal={:.3f}mps,angular={:.3f}radps)",
                comp._stepSequenceStepIndex,
                comp._physicalStepSupportSide < 0 ? "RIGHT" : "LEFT",
                comp._physicalStepTouchdownContactValid ? "yes" : "NO",
                comp._physicalStepTouchdownContactLocal.x,
                comp._physicalStepTouchdownContactLocal.y,
                comp._physicalStepTouchdownContactLocal.z,
                pivotCenterShift,
                turnPlan.touchdownSoleErrorDeg,
                turnPlan.touchdownHorizontalSpeed,
                turnPlan.touchdownAngularSpeed);
            if (continuousEnabled && turnPlan.candidateEvaluated) {
                spdlog::info(
                    "[LocomotionTurnContact] step={} swing={} outside={} "
                    "hip=(pred=({:+.3f},{:+.3f},{:+.3f}),"
                    "actual=({:+.3f},{:+.3f},{:+.3f}),"
                    "error=({:+.3f},{:+.3f},{:+.3f}))m "
                    "swingSpeed=(required={:.3f},admitted={:.3f},"
                    "limit={:.3f},achievedPeak={:.3f},touchdown={:.3f})mps "
                    "angular=(required={:.3f},admitted={:.3f},limit={:.3f},"
                    "achievedPeak={:.3f},touchdown={:.3f})radps "
                    "governor=(linear={:.3f}mps,tracking={:.3f}m,"
                    "angular={:.3f}radps,"
                    "soleTracking={:.1f}deg,"
                    "ankleClamp={:.1f}deg) "
                    "ankle=(parentComp={:.1f}->{:.1f}deg,localStep={:.2f}deg,"
                    "localRate={:.3f}radps,residual={:.1f}deg) "
                    "counterfactual=(planClamp={:.1f}deg,bestClamp={:.1f}deg,"
                    "bestSwivel={:+.1f}deg,closure={:.3f}m,accepted={},"
                    "applied={:+.1f}deg,priority={:.2f},relaxed={:.1f}deg,"
                    "swingReserve={:+.2f},twistMargin={:+.1f}deg) "
                    "fk=(valid={},ik={:.3f}m/{:.1f}deg,"
                    "motor={:.3f}m/{:.1f}deg,total={:.3f}m/{:.1f}deg)",
                    comp._stepSequenceStepIndex,
                    turnPlan.swingFootLeft ? "LEFT" : "RIGHT",
                    turnPlan.outsideFoot ? "yes" : "no",
                    turnPlan.predictedContactHip.x,
                    turnPlan.predictedContactHip.y,
                    turnPlan.predictedContactHip.z,
                    turnPlan.actualContactHip.x,
                    turnPlan.actualContactHip.y,
                    turnPlan.actualContactHip.z,
                    turnPlan.contactHipError.x,
                    turnPlan.contactHipError.y,
                    turnPlan.contactHipError.z,
                    turnPlan.requiredSwingSpeed,
                    turnPlan.admittedSwingSpeed,
                    turnPlan.swingSpeedLimit,
                    turnPlan.achievedSwingSpeed,
                    turnPlan.touchdownHorizontalSpeed,
                    turnPlan.requiredAngularSpeed,
                    turnPlan.admittedAngularSpeed,
                    turnPlan.angularSpeedLimit,
                    turnPlan.achievedAngularSpeed,
                    turnPlan.touchdownAngularSpeed,
                    comp._gaitSwingCommandSpeed,
                    comp._gaitSwingCommandTrackingError,
                    comp._gaitSwingCommandAngularSpeed,
                    comp._gaitSwingSoleCommandErrorDeg,
                    comp._gaitIkAnkleEnvelopeClampDeg,
                    comp._gaitAnkleParentCompensationDeg,
                    comp._gaitAnkleParentCompensationAppliedDeg,
                    comp._gaitAnkleCommandDeltaDeg,
                    comp._gaitAnkleCommandAngularSpeed,
                    comp._gaitAnkleAchievableResidualDeg,
                    turnPlan.sampledAnkleClampDeg,
                    comp._gaitIkExactAnkleClampDeg,
                    comp._gaitIkKneeSwivelRequestedDeg,
                    comp._gaitIkKneeSwivelCandidateClosure,
                    comp._gaitIkKneeSwivelCandidateAccepted ? "yes" : "no",
                    comp._gaitIkKneeSwivelAppliedDeg,
                    comp._gaitIkOrientationPriority,
                    comp._gaitIkOrientationRelaxedDeg,
                    comp._gaitIkAnkleSwingReserve,
                    comp._gaitIkAnkleTwistMarginDeg,
                    comp._gaitFkValid ? "yes" : "no",
                    glm::length(comp._gaitFkDesiredToCommand),
                    comp._gaitFkDesiredToCommandOrientationDeg,
                    glm::length(comp._gaitFkCommandToPhysical),
                    comp._gaitFkCommandToPhysicalOrientationDeg,
                    glm::length(comp._gaitFkDesiredToPhysical),
                    comp._gaitFkDesiredToPhysicalOrientationDeg);
            }
            comp._supportTransferTransferStartTarget =
                comp._physicalStepSupportTarget;
            constexpr float kPlantAcquireSupportFraction = 0.20f;
            const float touchdownSupportFraction = continuousEnabled
                ? kPlantAcquireSupportFraction : 0.35f;
            if (walkingOverlapHandoff) {
                const float transferFraction = glm::clamp(
                    comp.transferSupportBias
                        + comp._gaitAdaptiveTransferBiasOffset,
                    kNewSupportLoadCommandTarget, 0.98f);
                comp._supportTransferTransferEndTarget = glm::mix(
                    stanceFoot, swingFoot, transferFraction);
            } else {
                comp._supportTransferTransferEndTarget = glm::mix(
                    comp._physicalStepSupportTarget, swingFoot,
                    touchdownSupportFraction);
            }
            comp._supportTransferTransferEndTarget.y =
                comp._physicalStepSupportTarget.y;
            if (continuousEnabled && comp._gaitSupportCurveActive) {
                if (walkingOverlapHandoff) {
                    // Contact is the synchronization point for the two parallel branches:
                    // the plant anchor proves itself while this curve advances load toward
                    // the 52% role-swap threshold. Drift/contact/tilt guards remain live and
                    // can still abort before the rear foot is released.
                    glm::vec3 incomingVelocity =
                        comp._gaitSupportCommandVelocity;
                    incomingVelocity.y = 0.0f;
                    const float incomingSpeed = glm::length(incomingVelocity);
                    if (incomingSpeed > configuredSupportMaxSpeed
                        && incomingSpeed > 1e-6f) {
                        incomingVelocity *=
                            configuredSupportMaxSpeed / incomingSpeed;
                    }
                    comp._gaitSupportCurveStart =
                        comp._physicalStepSupportTarget;
                    comp._gaitSupportCurveStartVelocity = incomingVelocity;
                    comp._gaitSupportCurveEnd =
                        comp._supportTransferTransferEndTarget;
                    comp._gaitSupportCurveTime = 0.0f;
                    comp._gaitSupportCurveDuration = cadenceLoadOverlapTime;
                    comp._gaitSupportCurveEndVelocity = supportCurveEndVelocity(
                        comp._gaitSupportCurveStart,
                        comp._gaitSupportCurveEnd,
                        comp._gaitSupportCurveDuration);
                    spdlog::info(
                        "[LocomotionGait] SUPPORT_CURVE_CONTACT step={} "
                        "mode=parallel-handoff duration={:.3f}s "
                        "targetLoad={:.2f} latch={:.2f} speedCap={:.2f}mps "
                        "accelCap=3.00mps2 load={:.2f}",
                        comp._stepSequenceStepIndex,
                        comp._gaitSupportCurveDuration,
                        kNewSupportLoadCommandTarget,
                        kNewSupportLoadAcquireThreshold,
                        configuredSupportMaxSpeed,
                        comp._gaitNewSupportLoad);
                } else {
                    // Turn-conditioned steps retain the conservative ownership-first path.
                    comp._gaitSupportCurveStart =
                        comp._physicalStepSupportTarget;
                    comp._gaitSupportCurveEnd = comp._gaitSupportCurveStart;
                    comp._gaitSupportCurveStartVelocity = glm::vec3(0.0f);
                    comp._gaitSupportCurveEndVelocity = glm::vec3(0.0f);
                    comp._gaitSupportCommandVelocity = glm::vec3(0.0f);
                    comp._gaitSupportCurveTime = 0.0f;
                    comp._gaitSupportCurveDuration = 0.0f;
                    spdlog::info(
                        "[LocomotionGait] SUPPORT_CURVE_CONTACT step={} "
                        "mode=hold-for-ownership fraction={:.2f} "
                        "supportSpeed=0.000 load={:.2f}",
                        comp._stepSequenceStepIndex,
                        kPlantAcquireSupportFraction,
                        comp._gaitNewSupportLoad);
                }
            }
            comp._physicalStepPhase = kSettle;
            comp._physicalStepPhaseTime = 0.0f;
            comp._physicalStepSettleTime = 0.0f;
        };

        auto evaluateTouchdown = [&](const char* stage, float descentProgress) {
            const float minNormalY = glm::clamp(
                comp.touchdownMinNormalY, 0.35f, 1.0f);
            const float maxVerticalSpeed = continuousEnabled
                ? glm::max(comp.touchdownMaxVerticalSpeed, 0.30f)
                : glm::max(comp.touchdownMaxVerticalSpeed, 0.05f);
            const float horizontalTolerance = continuousEnabled
                ? glm::max(comp.footTargetTolerance,
                           glm::max(comp.gaitMaxFootCorrection, 0.01f))
                : glm::max(comp.footTargetTolerance, 0.01f);
            // Straight steps previously accepted the first geometrically valid contact,
            // even when the sole was still translating near 1 m/s. Require the governed
            // command and measured foot to reach the landing segment before ownership.
            const float minimumProgress = continuousEnabled
                ? glm::clamp(comp.touchdownMinTrajectoryProgress,
                             0.45f, 1.0f)
                : 0.70f;
            const float verticalTolerance = continuousEnabled ? 0.035f : 0.030f;
            const bool progressOk = continuousEnabled
                ? comp._physicalStepTrajectoryT >= minimumProgress
                : descentProgress >= minimumProgress;
            const bool normalOk = contactNormal.y >= minNormalY;
            const bool velocityOk = std::abs(swingVelocity.y) <= maxVerticalSpeed;
            const bool horizontalOk =
                comp._physicalStepHorizontalTargetError <= horizontalTolerance;
            const bool verticalOk =
                comp._physicalStepVerticalTargetError <= verticalTolerance;
            const bool turnTouchdown = hasYawBearingTurnStep();
            const float soleToleranceDeg = continuousEnabled
                ? (turnTouchdown ? 10.0f : 25.0f) : 180.0f;
            const bool soleOk = !continuousEnabled
                || comp._gaitSoleAngularErrorDeg <= soleToleranceDeg;
            const float touchdownHorizontalSpeed = glm::length(glm::vec2(
                swingVelocity.x, swingVelocity.z));
            const float touchdownAngularSpeed = swingAngularVelocityOk
                ? glm::length(swingAngularVelocity) : 0.0f;
            const float horizontalSpeedLimit = landingHorizontalSpeedLimit();
            const float angularSpeedLimit = landingAngularSpeedLimit();
            const float commandSpeedLimit = glm::clamp(
                comp.touchdownMaxCommandSpeed, 0.05f, 1.0f);
            const bool landingAngularReady = !continuousEnabled
                || (swingAngularVelocityOk
                    && touchdownAngularSpeed <= angularSpeedLimit);
            const bool landingLinearReady = !continuousEnabled
                || touchdownHorizontalSpeed <= horizontalSpeedLimit;
            const bool landingCommandReady = !continuousEnabled
                || comp._gaitSwingCommandSpeed <= commandSpeedLimit;
            auto& turnPlan = comp._gaitTurnPlan;
            turnPlan.touchdownSoleErrorDeg = comp._gaitSoleAngularErrorDeg;
            turnPlan.touchdownAngularSpeed = touchdownAngularSpeed;
            turnPlan.touchdownHorizontalSpeed = touchdownHorizontalSpeed;
            const bool geometricTouchdownReady = progressOk && normalOk
                && velocityOk && horizontalOk && verticalOk;
            if (continuousEnabled && geometricTouchdownReady
                && (!soleOk || !landingAngularReady || !landingLinearReady
                    || !landingCommandReady)
                && !turnPlan.touchdownReadinessBlockedLogged) {
                turnPlan.touchdownReadinessBlockedLogged = true;
                spdlog::info(
                    "[LocomotionTouchdown] step={} stage={} action=WAIT "
                    "trajectory={:.3f}/{:.3f}[{}] "
                    "sole={:.1f}/{:.1f}deg[{}] "
                    "angular={:.3f}/{:.3f}radps[{}] "
                    "horizontal={:.3f}/{:.3f}mps[{}] "
                    "command={:.3f}/{:.3f}mps[{}]",
                    comp._stepSequenceStepIndex,
                    stage,
                    comp._physicalStepTrajectoryT,
                    minimumProgress,
                    progressOk ? "ok" : "BLOCK",
                    comp._gaitSoleAngularErrorDeg,
                    soleToleranceDeg,
                    soleOk ? "ok" : "BLOCK",
                    touchdownAngularSpeed,
                    angularSpeedLimit,
                    landingAngularReady ? "ok" : "BLOCK",
                    touchdownHorizontalSpeed,
                    horizontalSpeedLimit,
                    landingLinearReady ? "ok" : "BLOCK",
                    comp._gaitSwingCommandSpeed,
                    commandSpeedLimit,
                    landingCommandReady ? "ok" : "BLOCK");
            }
            if (progressOk && normalOk && velocityOk && horizontalOk
                && verticalOk && soleOk
                && landingAngularReady && landingLinearReady
                && landingCommandReady) {
                acceptTouchdown();
            }
        };

        if (comp._physicalStepPhase == kSwing) {
            const float trajectoryBeforeAdvance =
                comp._physicalStepTrajectoryT;
            const float activeSwingTime = comp._gaitCancelMode != 0
                ? glm::max(0.18f, cadenceSwingTime * 0.65f)
                : cadenceSwingTime;
            const float swingProgress = glm::clamp(
                comp._physicalStepPhaseTime / activeSwingTime,
                0.0f, 1.0f);
            const float swingCeilingT = kSwingArrivalT * swingProgress;
            if (governedWalkingSwing) {
                desiredFoot = advanceGovernedSwingCommand(
                    swingCeilingT, 0.82f, trajectoryPoint);
            } else {
                comp._physicalStepTrajectoryT = swingCeilingT;
                desiredFoot = trajectoryPoint(comp._physicalStepTrajectoryT);
            }
            // Accumulate the arrival confidence during the final approach. A well-tracked
            // step can therefore cross the ARRIVAL milestone without an artificial pose
            // hold; a lagging or misaligned sole still receives the existing bounded wait.
            updateArrivalStability(
                comp._physicalStepTrajectoryT >= kSwingArrivalT - 0.10f);
            constexpr float kRecontactReleaseConfirmTime = 0.012f;
            constexpr float kRecontactRecoveryTimeout = 0.20f;
            constexpr float kRecontactSafeSoleClearance = 0.010f;
            const bool earlySwingContact = swingContactNow;
            const bool recoveryWasActive = continuousEnabled
                && comp._gaitSwingRecontactTime > 0.0f;
            const bool recoverableRecontact = continuousEnabled
                && earlySwingContact
                && comp._physicalStepClearance >= 0.025f;
            if (recoverableRecontact || recoveryWasActive) {
                if (!recoveryWasActive) {
                    comp._physicalStepAirborneTime = 0.0f;
                    spdlog::info(
                        "[LocomotionGait] EARLY_RECONTACT_BEGIN step={} "
                        "trajectory={:.3f} centerRise={:.3f}m "
                        "soleClear={:.3f}m penetration={:+.4f}m "
                        "sole={:.1f}deg "
                        "action=pause-lift",
                        comp._stepSequenceStepIndex,
                        trajectoryBeforeAdvance,
                        comp._physicalStepClearance,
                        comp._physicalStepSoleClearance,
                        comp._physicalStepContactPenetration,
                        comp._gaitSoleAngularErrorDeg);
                }
                comp._gaitSwingRecontactTime += dt;
                // Recovery owns collision clearance only. Sole leveling remains a soft
                // swing objective: making its IK error a release condition can strand an
                // already-airborne foot at an unreachable frozen pose, then abort it back
                // to the old plant.
                const bool releaseEvidence = !swingContactNow;
                comp._physicalStepAirborneTime = releaseEvidence
                    ? comp._physicalStepAirborneTime + dt : 0.0f;
                const bool releaseConfirmed = releaseEvidence
                    && comp._physicalStepAirborneTime
                        >= kRecontactReleaseConfirmTime;
                if (earlySwingContact) {
                    comp._gaitStepRecontactPauseTime += dt;
                    // Hold the exact horizontal trajectory parameter that was active when
                    // the edge touched and continue lifting only while collision still owns
                    // that edge. Once contact clears, translation resumes immediately while
                    // a short contact-free confirmation runs in parallel.
                    comp._physicalStepTrajectoryT = trajectoryBeforeAdvance;
                    desiredFoot = trajectoryPoint(trajectoryBeforeAdvance);
                    desiredFoot.y = glm::max(
                        desiredFoot.y,
                        comp._physicalStepSwingStart.y
                            + glm::max(comp.swingHeight, takeoffHeight)
                            + glm::max(
                                kRecontactSafeSoleClearance
                                    - comp._physicalStepSoleClearance,
                                0.0f));
                    comp._physicalStepArrivalStableTime = 0.0f;
                    // Recovery owns the entire swing clock, not only trajectory T. Leaving
                    // phase time running accumulated a distant ceiling and consumed the
                    // safety deadline, so release produced a forward catch-up burst or an
                    // arrival timeout even though the foot had been deliberately paused.
                    comp._physicalStepPhaseTime = glm::max(
                        comp._physicalStepPhaseTime - dt, 0.0f);
                    if (governedWalkingSwing) {
                        comp._gaitSwingCommandSpeed = moveScalarToward(
                            comp._gaitSwingCommandSpeed, 0.0f,
                            governedCommandAcceleration * dt);
                    }
                    if (comp._gaitSwingRecontactTime
                        >= kRecontactRecoveryTimeout) {
                        abortSequence(
                            "early swing contact persisted through recovery window");
                    }
                } else if (!releaseConfirmed) {
                    // Contact has already lost authority. Keep the trajectory moving while
                    // the one-to-two-frame confirmation runs; low clearance is handled by
                    // the continuous vertical guard below and can never abort an airborne
                    // foot by itself.
                } else {
                    spdlog::info(
                        "[LocomotionGait] EARLY_RECONTACT_RELEASE step={} "
                        "recovery={:.3f}s contactFree={:.3f}s "
                        "centerRise={:.3f}m soleClear={:.3f}m "
                        "penetration={:+.4f}m "
                        "sole={:.1f}deg[soft] "
                        "action=resume-swing",
                        comp._stepSequenceStepIndex,
                        comp._gaitSwingRecontactTime,
                        comp._physicalStepAirborneTime,
                        comp._physicalStepClearance,
                        comp._physicalStepSoleClearance,
                        comp._physicalStepContactPenetration,
                        comp._gaitSoleAngularErrorDeg);
                    comp._gaitSwingRecontactTime = 0.0f;
                    comp._physicalStepAirborneTime = 0.0f;
                }
            } else if (earlySwingContact) {
                abortSequence("swing contacted before hover arrival");
            }
            if (comp._physicalStepPhase == kSwing
                && !earlySwingContact) {
                // Preserve forward cadence while protecting a tilted toe/heel. This is a
                // continuous vertical bias, not a phase gate: it raises the command only
                // when the measured lowest point approaches the old support surface and
                // fades out as either clearance or sole orientation recovers.
                constexpr float kSwingSoleClearanceGuard = 0.015f;
                const float clearanceDeficit = glm::max(
                    kSwingSoleClearanceGuard
                        - comp._physicalStepSoleClearance,
                    0.0f);
                const float tiltPressure = smoothstep(glm::clamp(
                    (comp._gaitSoleAngularErrorDeg - 25.0f) / 20.0f,
                    0.0f, 1.0f));
                desiredFoot.y += glm::min(
                    clearanceDeficit * tiltPressure, 0.015f);
            }
            if (comp._physicalStepPhase == kSwing && governedTurnStep
                && !earlySwingContact) {
                constexpr float kTurnWatchdogProgressEpsilon = 0.002f;
                if (comp._physicalStepTrajectoryT
                        >= comp._gaitSwingWatchdogProgress
                            + kTurnWatchdogProgressEpsilon) {
                    comp._gaitSwingWatchdogProgress =
                        comp._physicalStepTrajectoryT;
                    comp._gaitSwingNoProgressTime = 0.0f;
                } else {
                    comp._gaitSwingNoProgressTime += dt;
                }
            }
            const bool swingTrajectoryComplete = governedWalkingSwing
                ? comp._physicalStepTrajectoryT
                    >= kSwingArrivalT - 1e-4f
                : swingProgress >= 1.0f;
            if (comp._physicalStepPhase == kSwing
                && swingTrajectoryComplete) {
                comp._physicalStepTrajectoryT = kSwingArrivalT;
                comp._physicalStepPhase = kArrival;
                comp._physicalStepPhaseTime = 0.0f;
                comp._gaitTurnPlan.arrivalTrajectoryT = kSwingArrivalT;
                // The shared trajectory has only completed its approach landmark here;
                // horizontal motion intentionally continues through descent.  Snapping
                // to hoverTarget moved the foot all the way to the foothold for one frame,
                // then ARRIVAL moved it back to trajectoryPoint(kSwingArrivalT).  The
                // resulting forward/back velocity impulse grew with every planned step
                // and destabilized the following plant.
                desiredFoot = trajectoryPoint(kSwingArrivalT);
            } else if (comp._physicalStepPhase == kSwing
                       && governedWalkingSwing) {
                constexpr float kTurnSwingStallTimeout = 0.25f;
                constexpr float kTurnSwingHardExtension = 0.60f;
                const bool ordinaryDeadlineExpired = !governedTurnStep
                    && comp._physicalStepPhaseTime
                        >= comp._gaitSwingDeadline;
                const bool turnBaseDeadlineExpired = governedTurnStep
                    && comp._physicalStepPhaseTime
                        >= comp._gaitSwingDeadline;
                const bool turnProgressStalled = turnBaseDeadlineExpired
                    && comp._gaitSwingNoProgressTime
                        >= kTurnSwingStallTimeout;
                const bool turnHardDeadlineExpired = governedTurnStep
                    && comp._physicalStepPhaseTime
                        >= comp._gaitSwingDeadline
                            + kTurnSwingHardExtension;
                // A turn foot that is still making measurable progress may finish its
                // admitted arc. The hard extension remains an absolute safety bound.
                if (ordinaryDeadlineExpired || turnProgressStalled
                    || turnHardDeadlineExpired) {
                    spdlog::warn(
                        "[LocomotionSwingGovernor] step={} stage=SWING action=TIMEOUT "
                        "trajectoryT={:.3f}/{:.3f} commandSpeed={:.3f}mps "
                        "trackingError={:.3f}m path={:.3f}m "
                        "minimum={:.3f}s deadline={:.3f}s noProgress={:.3f}s "
                        "hardDeadline={:.3f}s",
                        comp._stepSequenceStepIndex,
                        comp._physicalStepTrajectoryT,
                        kSwingArrivalT,
                        comp._gaitSwingCommandSpeed,
                        comp._gaitSwingCommandTrackingError,
                        comp._gaitSwingPathLength,
                        comp._gaitSwingMinimumDuration,
                        comp._gaitSwingDeadline,
                        comp._gaitSwingNoProgressTime,
                        comp._gaitSwingDeadline + kTurnSwingHardExtension);
                    abortSequence(
                        "swing command could not reach arrival landmark");
                }
            }
        } else if (comp._physicalStepPhase == kArrival) {
            const bool insideTurnArrival =
                hasYawBearingTurnStep()
                && !comp._gaitTurnPlan.outsideFoot
                && comp._gaitCancelMode == 0;
            if (insideTurnArrival) {
                // ARRIVAL used to pin a lagging inside sole at t=0.70 while the
                // pelvis kept moving.  Crawl a bounded additional 12 percent of
                // the horizontal path at hover height so convergence keeps making
                // progress without turning the timeout into extra free time.
                const float arrivalWindow = glm::max(
                    comp.arrivalTimeout, 0.25f);
                // Reach the crawl endpoint before the unchanged timeout so the full-pose
                // gate has time to accumulate its required quiet interval. Previously the
                // ceiling first reached t=0.82 on the same frame that timeout fired.
                const float stabilityReserve = glm::max(
                    cadenceArrivalSettleTime + 2.0f * dt, 0.06f);
                const float crawlBudget = glm::max(
                    arrivalWindow - stabilityReserve, 0.05f);
                const float crawlProgress = smoothstep(glm::clamp(
                    comp._physicalStepPhaseTime
                        / glm::max(crawlBudget, 0.01f),
                    0.0f, 1.0f));
                const float crawlCeilingT = glm::mix(
                    kSwingArrivalT, 0.82f, crawlProgress);
                auto hoverTrajectoryPoint = [&](float t) {
                    glm::vec3 point = trajectoryPoint(t);
                    point.y = hoverTarget.y;
                    return point;
                };
                desiredFoot = advanceGovernedSwingCommand(
                    crawlCeilingT, 0.82f, hoverTrajectoryPoint);
                comp._gaitTurnPlan.arrivalTrajectoryT =
                    comp._physicalStepTrajectoryT;
            } else {
                comp._gaitTurnPlan.arrivalTrajectoryT = kSwingArrivalT;
                comp._physicalStepTrajectoryT =
                    comp._gaitTurnPlan.arrivalTrajectoryT;
                desiredFoot = trajectoryPoint(
                    comp._physicalStepTrajectoryT);
            }
            const float arrivalVerticalError = std::abs(
                swingFoot.y - hoverTarget.y);
            const bool soleAligned = !continuousEnabled
                || comp._gaitSoleAngularErrorDeg
                    <= kSoleArrivalToleranceDeg;
            updateArrivalStability(true);
            const bool recoverableArrivalContact = continuousEnabled
                && swingContactNow && comp._physicalStepClearance >= 0.040f;
            if (recoverableArrivalContact) {
                // The normal arrival target may already be slightly below the center of a
                // pitched sole whose long edge touched first. Re-open vertical clearance
                // while the horizontal/leveling feedback catches up instead of continuing
                // to pull that edge into the ground for the duration of the grace window.
                desiredFoot.y = glm::max(
                    hoverTarget.y, swingFoot.y + 0.030f);
                if (comp._gaitSwingRecontactTime <= 0.0f) {
                }
                comp._gaitSwingRecontactTime += dt;
                comp._physicalStepArrivalStableTime = 0.0f;
                // The measured ankle needs roughly a quarter second to unwind its final
                // 20-25 degrees. Keep this bounded, but do not declare failure before the
                // powered joint has had one physically plausible convergence interval.
                if (comp._gaitSwingRecontactTime >= 0.35f)
                    abortSequence("arrival contact persisted through recovery window");
            } else if (swingContactNow) {
                abortSequence("arrival hold contacted before descent");
            } else {
                if (continuousEnabled && comp._gaitSwingRecontactTime > 0.0f) {
                }
                comp._gaitSwingRecontactTime = 0.0f;
            }
            const bool arrivalMilestoneReady = insideTurnArrival
                ? comp._physicalStepArrivalStableTime
                    >= cadenceArrivalSettleTime
                : (continuousEnabled
                    ? soleAligned
                    : comp._physicalStepArrivalStableTime
                        >= cadenceArrivalSettleTime);
            if (comp._physicalStepPhase == kArrival && !swingContactNow
                && arrivalMilestoneReady) {
                // The ordinary gait keeps one immutable trajectory through descent, so
                // changing the FSM milestone does not rebuild or jump the desired target.
                // A cancellation is a recovery path and may still seed a bounded segment
                // once from the measured sole.
                if (comp._gaitCancelMode != 0)
                    comp._physicalStepArcStart = swingFoot;
                comp._physicalStepPhase = kDescent;
                comp._physicalStepPhaseTime = 0.0f;
            } else if (comp._physicalStepPhase == kArrival && !swingContactNow
                       && !hasYawBearingTurnStep()
                       && comp._physicalStepPhaseTime >= glm::max(
                           cadenceArrivalSettleTime, 0.03f)
                       && comp._physicalStepClearance >= 0.040f) {
                // Gameplay arrival is a transition, not a pose-verification stop. Continue
                // leveling and horizontal correction throughout descent and let touchdown
                // contact validate the actual landing.
                if (comp._gaitCancelMode != 0)
                    comp._physicalStepArcStart = swingFoot;
                comp._physicalStepPhase = kDescent;
                comp._physicalStepPhaseTime = 0.0f;
            } else if (comp._physicalStepPhase == kArrival && !swingContactNow
                       && comp._physicalStepPhaseTime >= glm::max(
                           comp.arrivalTimeout,
                           hasYawBearingTurnStep() ? 0.25f : 0.10f)) {
                // ARRIVAL is a hover milestone, not contact acceptance. A yaw-bearing
                // foot that has reached the foothold envelope may use descent to finish
                // its measured ankle convergence. Touchdown retains the strict 10-degree
                // sole and velocity gates, so this cannot accept a tilted plant.
                constexpr float kTurnArrivalRecoveryHorizontalReserve = 0.005f;
                constexpr float kTurnArrivalRecoverySoleToleranceDeg = 22.0f;
                const bool safeTurnArrivalRecovery = hasYawBearingTurnStep()
                    && comp._physicalStepHorizontalTargetError
                        <= arrivalTolerance
                            + kTurnArrivalRecoveryHorizontalReserve
                    && arrivalVerticalError <= 0.025f
                    && comp._physicalStepClearance >= 0.040f
                    && comp._gaitSoleAngularErrorDeg
                        <= kTurnArrivalRecoverySoleToleranceDeg
                    && swingAngularVelocityOk;
                if (safeTurnArrivalRecovery) {
                    spdlog::info(
                        "[LocomotionTurnArrival] step={} action=DESCENT_RECOVERY "
                        "horizontal={:.3f}/{:.3f}m hoverY={:.3f}/0.025m "
                        "sole={:.1f}/{:.1f}deg trajectoryT={:.3f} "
                        "touchdownSoleLimit=10.0deg",
                        comp._stepSequenceStepIndex,
                        comp._physicalStepHorizontalTargetError,
                        arrivalTolerance
                            + kTurnArrivalRecoveryHorizontalReserve,
                        arrivalVerticalError,
                        comp._gaitSoleAngularErrorDeg,
                        kTurnArrivalRecoverySoleToleranceDeg,
                        comp._gaitTurnPlan.arrivalTrajectoryT);
                    comp._physicalStepPhase = kDescent;
                    comp._physicalStepPhaseTime = 0.0f;
                } else {
                    spdlog::warn(
                        "[LocomotionStep] ARRIVAL_CHECK result=FAIL horizontal={:.3f}/{:.3f}[{}] "
                        "hoverY={:.3f}/0.025[{}] stable={:.3f}/{:.3f}s[{}] "
                        "soleError={:.1f}/{:.1f}[{}] fwd={:+.3f} lat={:+.3f} "
                        "trajectoryT={:.3f} pairApplied={}",
                        comp._physicalStepHorizontalTargetError, arrivalTolerance,
                        comp._physicalStepHorizontalTargetError <= arrivalTolerance ? "ok" : "FAIL",
                        arrivalVerticalError,
                        arrivalVerticalError <= 0.025f ? "ok" : "FAIL",
                        comp._physicalStepArrivalStableTime,
                        cadenceArrivalSettleTime,
                        comp._physicalStepArrivalStableTime >= cadenceArrivalSettleTime
                            ? "ok" : "FAIL",
                        comp._gaitSoleAngularErrorDeg,
                        kSoleArrivalToleranceDeg,
                        soleAligned ? "ok" : "FAIL",
                        comp._physicalStepForwardTargetError,
                        comp._physicalStepLateralTargetError,
                        comp._gaitTurnPlan.arrivalTrajectoryT,
                        comp._gaitTurnPlan.pairBudgetApplied ? "yes" : "no");
                    abortSequence("swing foot could not retain safe clearance before descent");
                }
            }
        } else if (comp._physicalStepPhase == kDescent) {
            const float descentClockProgress = glm::clamp(
                comp._physicalStepPhaseTime / activeCadenceDescentTime(),
                0.0f, 1.0f);
            const bool governedWalkingDescent = governedWalkingSwing;
            const float descentStartT = governedInsideTurnSwing
                ? glm::clamp(comp._gaitTurnPlan.arrivalTrajectoryT,
                             kSwingArrivalT, 0.82f)
                : kSwingArrivalT;
            if (governedWalkingDescent) {
                auto descentTrajectoryPoint = [&](float t) {
                    return trajectoryPoint(t);
                };
                const float clockCeilingT = glm::mix(
                    descentStartT, 1.0f, descentClockProgress);
                const bool atLandingBrake =
                    comp._physicalStepTrajectoryT
                        >= activeSwingLandingBrakeT - 1e-4f;
                if (!comp._gaitLandingBrakeReleased && atLandingBrake) {
                    comp._gaitLandingBrakeReadyTime = landingBrakeKinematicsReady()
                        ? comp._gaitLandingBrakeReadyTime + dt : 0.0f;
                    const float landingBrakeReleaseStableTime = governedTurnStep
                        ? 0.03f : 0.02f;
                    if (comp._gaitLandingBrakeReadyTime
                            >= landingBrakeReleaseStableTime) {
                        comp._gaitLandingBrakeReleased = true;
                        if (comp.debug) {
                            const float releaseHorizontalSpeed = glm::length(glm::vec2(
                                swingVelocity.x, swingVelocity.z));
                            const float releaseAngularSpeed = swingAngularVelocityOk
                                ? glm::length(swingAngularVelocity) : 0.0f;
                            spdlog::info(
                                "[LocomotionLandingBrake] step={} event=RELEASE "
                                "trajectoryT={:.3f} stable={:.3f}s "
                                "approach=(horizontal={:.3f}/{:.3f},"
                                "angular={:.3f}/{:.3f})",
                                comp._stepSequenceStepIndex,
                                comp._physicalStepTrajectoryT,
                                comp._gaitLandingBrakeReadyTime,
                                releaseHorizontalSpeed,
                                landingBrakeHorizontalSpeedLimit(),
                                releaseAngularSpeed,
                                landingBrakeAngularSpeedLimit());
                        }
                    }
                    const float fallbackHorizontalSpeed = glm::length(glm::vec2(
                        swingVelocity.x, swingVelocity.z));
                    const float fallbackAngularSpeed = swingAngularVelocityOk
                        ? glm::length(swingAngularVelocity) : 0.0f;
                    const float fallbackCommandSpeedLimit = glm::clamp(
                        comp.touchdownMaxCommandSpeed, 0.05f, 1.0f);
                    const float fallbackHorizontalTolerance = glm::max(
                        comp.footTargetTolerance, 0.030f);
                    constexpr float kFallbackVerticalTolerance = 0.060f;
                    constexpr float kFallbackSoleToleranceDeg = 25.0f;
                    const float fallbackMinimumDescentTime = glm::max(
                        cadenceDescentTime + 0.10f, 0.18f);
                    const bool safeStraightFallback =
                        !comp._gaitLandingBrakeReleased
                        && !hasYawBearingTurnStep()
                        && comp._physicalStepPhaseTime
                            >= fallbackMinimumDescentTime
                        && comp._physicalStepHorizontalTargetError
                            <= fallbackHorizontalTolerance
                        && comp._physicalStepVerticalTargetError
                            <= kFallbackVerticalTolerance
                        && comp._gaitSoleAngularErrorDeg
                            <= kFallbackSoleToleranceDeg
                        && comp._gaitSwingCommandSpeed
                            <= fallbackCommandSpeedLimit
                        && fallbackHorizontalSpeed
                            <= 2.0f * landingHorizontalSpeedLimit()
                        && swingAngularVelocityOk
                        && fallbackAngularSpeed
                            <= 2.0f * landingAngularSpeedLimit();
                    if (safeStraightFallback) {
                        comp._gaitLandingBrakeReleased = true;
                        spdlog::info(
                            "[LocomotionLandingBrake] step={} event=SAFE_FALLBACK "
                            "trajectoryT={:.3f} descent={:.3f}s "
                            "error=(horizontal={:.3f}/{:.3f},vertical={:.3f}/{:.3f})m "
                            "sole={:.1f}/{:.1f}deg motion=(horizontal={:.3f},angular={:.3f}) "
                            "action=continue-to-contact",
                            comp._stepSequenceStepIndex,
                            comp._physicalStepTrajectoryT,
                            comp._physicalStepPhaseTime,
                            comp._physicalStepHorizontalTargetError,
                            fallbackHorizontalTolerance,
                            comp._physicalStepVerticalTargetError,
                            kFallbackVerticalTolerance,
                            comp._gaitSoleAngularErrorDeg,
                            kFallbackSoleToleranceDeg,
                            fallbackHorizontalSpeed,
                            fallbackAngularSpeed);
                    }
                }
                if (!comp._gaitLandingBrakeReleased) {
                    // Approach and then hold the braking landmark. The governor owns
                    // the current admitted point, so a noisy readiness sample can no
                    // longer rewind t or pull the foot back toward an earlier pose.
                    const float previousTrajectoryT =
                        comp._physicalStepTrajectoryT;
                    const float descentCeilingT = glm::min(
                        clockCeilingT, activeSwingLandingBrakeT);
                    desiredFoot = advanceGovernedSwingCommand(
                        descentCeilingT, activeSwingLandingBrakeT,
                        descentTrajectoryPoint);
                    comp._physicalStepTrajectoryT = glm::max(
                        previousTrajectoryT,
                        comp._physicalStepTrajectoryT);
                    desiredFoot = descentTrajectoryPoint(
                        comp._physicalStepTrajectoryT);
                    if (atLandingBrake) {
                        comp._gaitSwingCommandSpeed = moveScalarToward(
                            comp._gaitSwingCommandSpeed, 0.0f,
                            governedCommandAcceleration * dt);
                    }
                } else {
                    desiredFoot = advanceGovernedSwingCommand(
                        clockCeilingT, 1.0f,
                        descentTrajectoryPoint);
                }
            } else {
                comp._physicalStepTrajectoryT = descentStartT
                    + (1.0f - descentStartT) * descentClockProgress;
                desiredFoot = comp._gaitCancelMode != 0
                    ? glm::mix(comp._physicalStepArcStart,
                               comp._physicalStepFoothold,
                               smoothstep(descentClockProgress))
                    : trajectoryPoint(comp._physicalStepTrajectoryT);
            }
            const float descentProgress = governedWalkingDescent
                ? glm::clamp(
                    (comp._physicalStepTrajectoryT - descentStartT)
                        / glm::max(1.0f - descentStartT, 1e-4f),
                    0.0f, 1.0f)
                : descentClockProgress;
            if (swingContactNow) {
                // Solver contact is reported before the newly landed sole has dissipated
                // its approach velocity. Keep validating on subsequent frames instead of
                // turning that first, expected impact sample into a gait abort.
                evaluateTouchdown("DESCENT", descentProgress);
            }
            if (comp._physicalStepPhase == kDescent
                && governedWalkingDescent && swingContactNow) {
                const float minimumContactProgress = glm::clamp(
                    comp.touchdownMinTrajectoryProgress, 0.45f, 1.0f);
                const float horizontalContactSpeed = glm::length(glm::vec2(
                    swingVelocity.x, swingVelocity.z));
                const float maximumVerticalContactSpeed = glm::max(
                    comp.touchdownMaxVerticalSpeed, 0.30f);
                constexpr float kLateContactRebaseRadius = 0.070f;
                const bool safeLateContact =
                    comp._physicalStepTrajectoryT >= minimumContactProgress
                    && contactNormal.y >= glm::clamp(
                        comp.touchdownMinNormalY, 0.35f, 1.0f)
                    && std::abs(swingVelocity.y)
                        <= maximumVerticalContactSpeed
                    && horizontalContactSpeed
                        <= landingHorizontalSpeedLimit()
                    && comp._physicalStepHorizontalTargetError
                        <= kLateContactRebaseRadius;
                if (safeLateContact) {
                    const glm::vec3 plannedFoothold =
                        comp._physicalStepFoothold;
                    const float contactTrajectoryT =
                        comp._physicalStepTrajectoryT;
                    comp._physicalStepFoothold = swingFoot;
                    comp._physicalStepTrajectoryT = 1.0f;
                    comp._gaitSwingCommandSpeed = 0.0f;
                    comp._physicalStepPhase = kTouchdownWait;
                    comp._physicalStepPhaseTime = 0.0f;
                    desiredFoot = swingFoot;
                    spdlog::info(
                        "[LocomotionContactRebase] step={} "
                        "trajectoryT={:.3f} offset=(fwd={:+.3f},lat={:+.3f},"
                        "horizontal={:.3f})m motion=(horizontal={:.3f},"
                        "vertical={:+.3f})mps action=freeze-achieved-contact",
                        comp._stepSequenceStepIndex,
                        contactTrajectoryT,
                        glm::dot(swingFoot - plannedFoothold,
                                 comp._physicalStepForward),
                        glm::dot(swingFoot - plannedFoothold,
                                 comp._physicalStepRight),
                        horizontalDistance(swingFoot, plannedFoothold),
                        horizontalContactSpeed,
                        swingVelocity.y);
                }
            }
            if (comp._physicalStepPhase == kDescent
                && descentProgress >= 1.0f) {
                comp._physicalStepPhase = kTouchdownWait;
                comp._physicalStepPhaseTime = 0.0f;
                desiredFoot = comp._physicalStepFoothold;
            } else if (comp._physicalStepPhase == kDescent
                       && governedWalkingDescent
                       && comp._physicalStepPhaseTime
                            >= activeCadenceDescentTime()
                                + glm::max(comp.plantTimeout, 0.25f)) {
                // A governed foot can arrive one integration step short of t=1 while it
                // is already over ground and safely converging on the foothold. Do not
                // turn that harmless governor remainder into a failed step. Hand command
                // ownership to the bounded touchdown wait, but leave strict physical
                // contact as the only path that can accept the landing.
                constexpr float kDescentHandoffMinimumT = 0.98f;
                constexpr float kDescentHandoffVerticalTolerance = 0.040f;
                constexpr float kDescentHandoffTrackingTolerance = 0.040f;
                constexpr float kDescentHandoffSoleToleranceDeg = 25.0f;
                const float handoffHorizontalTolerance = glm::max(
                    comp.footTargetTolerance, 0.040f);
                const float handoffHorizontalSpeed = glm::length(glm::vec2(
                    swingVelocity.x, swingVelocity.z));
                const float handoffAngularSpeed = swingAngularVelocityOk
                    ? glm::length(swingAngularVelocity) : 0.0f;
                const bool safeStraightDescentHandoff =
                    walkingOverlapHandoff
                    && comp._physicalStepTrajectoryT
                        >= kDescentHandoffMinimumT
                    && FootGrounded(rag, swing->footIdx)
                    && comp._physicalStepHorizontalTargetError
                        <= handoffHorizontalTolerance
                    && comp._physicalStepVerticalTargetError
                        <= kDescentHandoffVerticalTolerance
                    && comp._gaitSwingCommandTrackingError
                        <= kDescentHandoffTrackingTolerance
                    && swingVelocity.y <= 0.05f
                    && comp._gaitSoleAngularErrorDeg
                        <= kDescentHandoffSoleToleranceDeg
                    && handoffHorizontalSpeed
                        <= landingHorizontalSpeedLimit()
                    && swingAngularVelocityOk
                    && handoffAngularSpeed
                        <= 2.0f * landingAngularSpeedLimit();
                // A turn foot can be physically grounded and inside every touchdown
                // envelope while its command waits at the t=.90 landing brake. Hand that
                // bounded remainder to TOUCHDOWN_WAIT instead of aborting a valid plant;
                // strict contact acceptance still occurs there.
                const bool safeTurnDescentHandoff = governedTurnStep
                    && comp._physicalStepTrajectoryT
                        >= activeSwingLandingBrakeT - 1e-4f
                    && FootGrounded(rag, swing->footIdx)
                    && comp._physicalStepHorizontalTargetError
                        <= handoffHorizontalTolerance
                    && comp._physicalStepVerticalTargetError
                        <= kDescentHandoffVerticalTolerance
                    && comp._gaitSwingCommandTrackingError
                        <= kDescentHandoffTrackingTolerance
                    && swingVelocity.y <= 0.05f
                    && comp._gaitSoleAngularErrorDeg
                        <= kDescentHandoffSoleToleranceDeg
                    && handoffHorizontalSpeed
                        <= landingHorizontalSpeedLimit()
                    && swingAngularVelocityOk
                    && handoffAngularSpeed
                        <= 2.0f * landingAngularSpeedLimit();
                if (safeStraightDescentHandoff
                    || safeTurnDescentHandoff) {
                    const float governedTrajectoryT =
                        comp._physicalStepTrajectoryT;
                    comp._physicalStepTrajectoryT = 1.0f;
                    comp._physicalStepPhase = kTouchdownWait;
                    comp._physicalStepPhaseTime = 0.0f;
                    desiredFoot = comp._physicalStepFoothold;
                    spdlog::info(
                        "[LocomotionSwingGovernor] step={} stage=DESCENT "
                        "action=HANDOFF_TO_TOUCHDOWN_WAIT trajectoryT={:.3f} "
                        "error=(horizontal={:.3f}/{:.3f},vertical={:.3f}/{:.3f},"
                        "tracking={:.3f}/{:.3f})m motion=(horizontal={:.3f},"
                        "vertical={:+.3f},angular={:.3f}) sole={:.1f}/{:.1f}deg",
                        comp._stepSequenceStepIndex,
                        governedTrajectoryT,
                        comp._physicalStepHorizontalTargetError,
                        handoffHorizontalTolerance,
                        comp._physicalStepVerticalTargetError,
                        kDescentHandoffVerticalTolerance,
                        comp._gaitSwingCommandTrackingError,
                        kDescentHandoffTrackingTolerance,
                        handoffHorizontalSpeed,
                        swingVelocity.y,
                        handoffAngularSpeed,
                        comp._gaitSoleAngularErrorDeg,
                        kDescentHandoffSoleToleranceDeg);
                } else {
                    spdlog::warn(
                        "[LocomotionSwingGovernor] step={} stage=DESCENT action=TIMEOUT "
                        "trajectoryT={:.3f}/1.000 commandSpeed={:.3f}mps "
                        "trackingError={:.3f}m",
                        comp._stepSequenceStepIndex,
                        comp._physicalStepTrajectoryT,
                        comp._gaitSwingCommandSpeed,
                        comp._gaitSwingCommandTrackingError);
                    abortSequence(
                        "descent command could not reach foothold");
                }
            }
        } else if (comp._physicalStepPhase == kTouchdownWait) {
            comp._physicalStepTrajectoryT = 1.0f;
            // Cancellation disables governedWalkingSwing, but touchdown still validates
            // this retained command speed. Always brake it on the straight path so a
            // released input cannot freeze a stale 0.7 m/s command until timeout.
            if (walkingOverlapHandoff) {
                comp._gaitSwingCommandSpeed = moveScalarToward(
                    comp._gaitSwingCommandSpeed, 0.0f,
                    governedCommandAcceleration * dt);
            }
            desiredFoot = comp._physicalStepFoothold;
            if (swingContactNow) {
                evaluateTouchdown("TOUCHDOWN_WAIT", 1.0f);
            }
            if (comp._physicalStepPhase == kTouchdownWait
                && comp._physicalStepPhaseTime
                    >= glm::max(comp.plantTimeout,
                        hasYawBearingTurnStep() ? 0.25f : 0.10f)) {
                if (continuousEnabled) {
                    const float minNormalY = glm::clamp(
                        comp.touchdownMinNormalY, 0.35f, 1.0f);
                    const float maxVerticalSpeed = glm::max(
                        comp.touchdownMaxVerticalSpeed, 0.30f);
                    const float horizontalTolerance = glm::max(
                        comp.footTargetTolerance,
                        glm::max(comp.gaitMaxFootCorrection, 0.01f));
                    constexpr float kVerticalTolerance = 0.035f;
                    const float soleToleranceDeg = hasYawBearingTurnStep()
                        ? 10.0f : 25.0f;
                    const float angularSpeedLimit =
                        landingAngularSpeedLimit();
                    const float horizontalSpeedLimit =
                        landingHorizontalSpeedLimit();
                    const float commandSpeedLimit = glm::clamp(
                        comp.touchdownMaxCommandSpeed, 0.05f, 1.0f);
                    const float finalHorizontalSpeed = glm::length(glm::vec2(
                        swingVelocity.x, swingVelocity.z));
                    const float finalAngularSpeed = swingAngularVelocityOk
                        ? glm::length(swingAngularVelocity) : 0.0f;
                    spdlog::warn(
                        "[LocomotionTouchdown] step={} stage=TOUCHDOWN_WAIT "
                        "action=TIMEOUT role={} pairApplied={} "
                        "gates=(contact={};normal={:.3f}/{:.3f}[{}];"
                        "verticalSpeed={:+.3f}/{:.3f}[{}];"
                        "targetHorizontal={:.3f}/{:.3f}[{}];"
                        "targetVertical={:.3f}/{:.3f}[{}];"
                        "sole={:.1f}/{:.1f}[{}];"
                        "angular={:.3f}/{:.3f}[{}];"
                        "linear={:.3f}/{:.3f}[{}];"
                        "command={:.3f}/{:.3f}[{}])",
                        comp._stepSequenceStepIndex,
                        comp._gaitTurnPlan.outsideFoot
                            ? "outside" : "inside",
                        comp._gaitTurnPlan.pairBudgetApplied ? "yes" : "no",
                        swingContactNow ? "yes" : "NO",
                        contactNormal.y, minNormalY,
                        contactNormal.y >= minNormalY ? "ok" : "BLOCK",
                        swingVelocity.y, maxVerticalSpeed,
                        std::abs(swingVelocity.y) <= maxVerticalSpeed
                            ? "ok" : "BLOCK",
                        comp._physicalStepHorizontalTargetError,
                        horizontalTolerance,
                        comp._physicalStepHorizontalTargetError
                                <= horizontalTolerance
                            ? "ok" : "BLOCK",
                        comp._physicalStepVerticalTargetError,
                        kVerticalTolerance,
                        comp._physicalStepVerticalTargetError
                                <= kVerticalTolerance
                            ? "ok" : "BLOCK",
                        comp._gaitSoleAngularErrorDeg,
                        soleToleranceDeg,
                        comp._gaitSoleAngularErrorDeg
                                <= soleToleranceDeg
                            ? "ok" : "BLOCK",
                        finalAngularSpeed,
                        angularSpeedLimit,
                        swingAngularVelocityOk
                                && finalAngularSpeed <= angularSpeedLimit
                            ? "ok" : "BLOCK",
                        finalHorizontalSpeed,
                        horizontalSpeedLimit,
                        finalHorizontalSpeed <= horizontalSpeedLimit
                            ? "ok" : "BLOCK",
                        comp._gaitSwingCommandSpeed,
                        commandSpeedLimit,
                        comp._gaitSwingCommandSpeed <= commandSpeedLimit
                            ? "ok" : "BLOCK");
                }
                abortSequence("touchdown contact timed out");
            }
        } else if (comp._physicalStepPhase >= kSettle
                   && comp._physicalStepPhase <= kComplete) {
            comp._physicalStepTrajectoryT = 1.0f;
            desiredFoot = comp._physicalStepTouchdownAccepted
                ? swing->plantFoot : comp._physicalStepFoothold;
        }
        comp._physicalStepTargetError = glm::length(swingFoot - desiredFoot);

        if (transferEnabled && comp._physicalStepPhase >= kTransfer
            && comp._physicalStepPhase <= kHold) {
            comp._supportTransferContactLossTime = swingContactNow && stanceContactNow
                ? 0.0f : comp._supportTransferContactLossTime + dt;
        } else {
            comp._supportTransferContactLossTime = 0.0f;
        }

        const bool recoverableOldSupportDrift = continuousEnabled
            && transferEnabled && transferOrHold
            && !gaitOldSupportUnloaded
            && comp._physicalStepStanceDrift > 0.040f;
        if (recoverableOldSupportDrift
            && !comp._gaitOldSupportDriftAllowanceLogged) {
            comp._gaitOldSupportDriftAllowanceLogged = true;
            spdlog::warn(
                "[LocomotionGait] TRANSFER_DRIFT_RECOVER oldSupport={} "
                "oldDrift={:.3f}m newSupport={} newDrift={:.3f}m "
                "oldUnloaded={} action=adapt",
                comp._physicalStepSupportSide < 0 ? "LEFT" : "RIGHT",
                comp._physicalStepStanceDrift,
                comp._physicalStepSupportSide < 0 ? "RIGHT" : "LEFT",
                comp._physicalStepPlantDrift,
                gaitOldSupportUnloaded ? "yes" : "no");
        }

        // During continuous gait the old foot is in the process of becoming the next
        // swing foot. Its drift is a correction signal, not by itself proof that the
        // newly acquired support is unsafe. Preserve an emergency boundary only when
        // large old-foot motion is accompanied by another transfer instability.
        const bool severeOldSupportInstability = continuousEnabled
            && transferEnabled && transferOrHold
            && comp._physicalStepStanceDrift > 0.100f
            && (comp._physicalStepPlantDrift > 0.025f
                || tiltDeg >= 20.0f
                || comp._supportTransferComHorizontalSpeed > 0.35f);
        constexpr float kPlantSlipDriftLimit = 0.040f;
        constexpr float kPlantSlipGrowthLimit = 0.050f;
        constexpr float kPlantSlipPersistence = 0.18f;
        const bool growingUnacquiredPlantSlip = continuousEnabled
            && comp._physicalStepPhase == kSettle
            && !comp._physicalStepPlantPoseCaptured
            && comp._physicalStepPlantDrift > kPlantSlipDriftLimit
            && comp._gaitPlantDriftRate > kPlantSlipGrowthLimit;
        comp._physicalStepPlantUnsafeTime = growingUnacquiredPlantSlip
            ? comp._physicalStepPlantUnsafeTime + dt : 0.0f;

        if (comp._physicalStepPhase >= kTakeoff
            && comp._physicalStepPhase <= kSupportReady
            && !stanceContactNow) {
            abortSequence("stance contact was lost");
        } else if (comp._physicalStepPhase >= kTakeoff
                   && comp._physicalStepPhase <= kSupportReady
                   && comp._physicalStepStanceDrift > 0.040f) {
            abortSequence("stance foot exceeded 4 cm drift");
        } else if (comp._physicalStepPhase >= kTakeoff
                   && comp._physicalStepPhase <= kSupportReady
                   && tiltDeg >= 30.0f) {
            abortSequence("tilt reached 30 degrees");
        } else if (comp._physicalStepPhase == kSwing
                   && comp._physicalStepTrajectoryT < 0.65f
                   && comp._physicalStepPhaseTime >= 0.10f
                   && comp._physicalStepClearance < 0.030f) {
            abortSequence("airborne swing lost clearance");
        } else if (growingUnacquiredPlantSlip
                   && comp._physicalStepPlantUnsafeTime
                        >= kPlantSlipPersistence) {
            spdlog::warn(
                "[LocomotionGait] PLANT_SLIP_GUARD result=ABORT "
                "drift={:.3f}/{:.3f}m rate={:+.3f}/{:+.3f}mps "
                "persistent={:.3f}/{:.3f}s footSpeed={:.3f} "
                "supportSpeed={:.3f} correctionPeak={:.3f}/{:.3f}m "
                "saturated={}",
                comp._physicalStepPlantDrift,
                kPlantSlipDriftLimit,
                comp._gaitPlantDriftRate,
                kPlantSlipGrowthLimit,
                comp._physicalStepPlantUnsafeTime,
                kPlantSlipPersistence,
                glm::length(swingVelocity),
                glm::length(comp._gaitSupportCommandVelocity),
                comp._gaitPlantCorrectionPeakRequested,
                comp._gaitPlantCorrectionPeakApplied,
                comp._gaitPlantCorrectionSaturated ? "yes" : "no");
            abortSequence("new plant remained beyond 4 cm while still sliding");
        } else if ((comp._physicalStepPhase == kSettle
                    || comp._physicalStepPhase == kSupportReady)
                   && comp._physicalStepPlantPoseCaptured
                   && comp._physicalStepPlantDrift > 0.040f) {
            abortSequence("new plant exceeded 4 cm drift");
        } else if (transferEnabled && comp._physicalStepPhase >= kTransfer
                   && comp._physicalStepPhase <= kHold
                   && comp._supportTransferContactLossTime
                        > (walkingOverlapHandoff
                            ? (straightStartupSupport ? 0.18f : 0.10f)
                            : 0.05f)) {
            abortSequence("foot contact was lost during support transfer");
        } else if (transferEnabled && transferOrHold
                   && comp._physicalStepPlantDrift > 0.040f) {
            if (continuousEnabled) {
                spdlog::warn(
                    "[LocomotionGait] TRANSFER_DRIFT_ABORT reason=new_support "
                    "oldSupport={} oldDrift={:.3f}m newSupport={} newDrift={:.3f}m "
                    "anchorStage={} handoffAt={:.3f}s migration={:.3f}m "
                    "sole={:.1f}deg angular=(pitch={:+.3f},roll={:+.3f},"
                    "yaw={:+.3f},mag={:.3f})radps footVelocity="
                    "(fwd={:+.3f},lat={:+.3f},y={:+.3f})mps "
                    "contactLocal=({:+.3f},{:+.3f},{:+.3f}) "
                    "impactLocal=({:+.3f},{:+.3f},{:+.3f}) "
                    "correctionPeak={:.3f}/{:.3f}m saturated={} "
                    "supportSpeed={:.3f}mps load={:.3f} latched={}",
                    comp._physicalStepSupportSide < 0 ? "LEFT" : "RIGHT",
                    comp._physicalStepStanceDrift,
                    comp._physicalStepSupportSide < 0 ? "RIGHT" : "LEFT",
                    comp._physicalStepPlantDrift,
                    comp._physicalStepPlantCenterAnchorActive
                        ? "center" : "pivot",
                    comp._physicalStepPlantAnchorHandoffPhaseTime,
                    comp._physicalStepPlantContactMigration,
                    comp._gaitSoleAngularErrorDeg,
                    glm::dot(swingAngularVelocity, comp._physicalStepRight),
                    glm::dot(swingAngularVelocity, comp._physicalStepForward),
                    swingAngularVelocity.y,
                    comp._physicalStepPlantAngularSpeed,
                    glm::dot(swingVelocity, comp._physicalStepForward),
                    glm::dot(swingVelocity, comp._physicalStepRight),
                    swingVelocity.y,
                    comp._physicalStepContactLocal.x,
                    comp._physicalStepContactLocal.y,
                    comp._physicalStepContactLocal.z,
                    comp._physicalStepTouchdownContactLocal.x,
                    comp._physicalStepTouchdownContactLocal.y,
                    comp._physicalStepTouchdownContactLocal.z,
                    comp._gaitPlantCorrectionPeakRequested,
                    comp._gaitPlantCorrectionPeakApplied,
                    comp._gaitPlantCorrectionSaturated ? "yes" : "no",
                    glm::length(comp._gaitSupportCommandVelocity),
                    comp._gaitNewSupportLoad,
                    comp._gaitNewSupportLoadLatched ? "yes" : "no");
            }
            abortSequence("new support drift exceeded 4 cm during transfer");
        } else if (transferEnabled && transferOrHold
                   && !continuousEnabled
                   && comp._physicalStepStanceDrift > 0.040f) {
            abortSequence("old support drift exceeded 4 cm during transfer");
        } else if (severeOldSupportInstability) {
            spdlog::warn(
                "[LocomotionGait] TRANSFER_DRIFT_ABORT reason=compound_instability "
                "oldSupport={} oldDrift={:.3f}m newSupport={} newDrift={:.3f}m "
                "comSpeed={:.3f}mps tilt={:.1f}",
                comp._physicalStepSupportSide < 0 ? "LEFT" : "RIGHT",
                comp._physicalStepStanceDrift,
                comp._physicalStepSupportSide < 0 ? "RIGHT" : "LEFT",
                comp._physicalStepPlantDrift,
                comp._supportTransferComHorizontalSpeed,
                tiltDeg);
            abortSequence("severe old-support drift accompanied by unstable transfer");
        } else if (transferEnabled && comp._physicalStepPhase >= kTransfer
                   && comp._physicalStepPhase <= kHold && tiltDeg >= 30.0f) {
            abortSequence("tilt reached 30 degrees during support transfer");
        }

        // Stage one preserves the first credible impact point while the sole rolls flat.
        // Stage two freezes the measured sole center after a short level/quiet window.
        // That second ownership mode is important because the collision manifold's
        // deepest point can migrate from heel to toe; continuing to pull the original
        // material point back to the impact patch is positive feedback for rocking.
        const float kPlantPivotQuietTime = walkingOverlapHandoff
            ? cadenceWalkingPlantQuietTime : 0.08f;
        // A quiet 9-11 degree sole was already accepted as loaded support below, but the
        // impact-pivot stage required 8 degrees and therefore had no legal path to that
        // accepted state. Use one ownership angle throughout the handoff. Contact, angular
        // speed, linear speed, and the full quiet interval remain independent requirements.
        constexpr float kPlantOwnershipSoleToleranceDeg = 12.0f;
        // Straight landing has a wider acquisition envelope, then a small release band
        // once center ownership exists. This prevents a quiet 12.7 degree sole from
        // resetting the entire proof every frame while leaving turns at their original
        // conservative 12 degree limit.
        const float activePlantSoleToleranceDeg = walkingOverlapHandoff
            ? ((comp._physicalStepPlantCenterAnchorActive
                || comp._physicalStepPlantPoseCaptured) ? 16.0f : 14.0f)
            : kPlantOwnershipSoleToleranceDeg;
        constexpr float kPlantPivotAngularSpeedLimit = 0.75f;
        constexpr float kPlantPivotLinearSpeedLimit = 0.12f;
        constexpr float kPlantPivotReleaseTriggerTime = 0.032f;
        constexpr float kPlantPivotReleaseBlendDuration = 0.06f;
        constexpr float kPlantContactMigrationNotice = 0.060f;
        constexpr float kPlantCenterBlendDuration = 0.09f;
        constexpr float kPostHandoffAcquireMargin = 0.06f;
        const float swingHorizontalSpeed = glm::length(glm::vec2(
            swingVelocity.x, swingVelocity.z));
        const bool plantPivotStage = continuousEnabled
            && comp._physicalStepPhase == kSettle
            && comp._physicalStepTouchdownAccepted
            && comp._physicalStepTouchdownContactValid
            && !comp._physicalStepPlantCenterAnchorActive;
        const bool pivotContactReady = swingContactNow && stanceContactNow;
        const bool pivotSoleReady = comp._gaitSoleAngularErrorDeg
            <= activePlantSoleToleranceDeg;
        const bool pivotAngularReady = swingAngularVelocityOk
            && comp._physicalStepPlantAngularSpeed
                <= kPlantPivotAngularSpeedLimit;
        const bool pivotLinearReady = swingHorizontalSpeed
            <= kPlantPivotLinearSpeedLimit;
        // A capped position servo that coincides with continued foot motion is no
        // longer stabilizing the impact pivot; it is feeding energy into the planted
        // leg. This uses the preceding frame's correction measurement so the release
        // decision is based on the command that produced the motion seen here.
        const bool pivotReleaseOverloaded = plantPivotStage
            && comp._gaitPlantCorrectionAtLimit
            && (!pivotAngularReady || !pivotLinearReady);
        if (!comp._physicalStepPlantPivotReleaseLatched) {
            comp._physicalStepPlantPivotReleaseTriggerTime =
                pivotReleaseOverloaded
                ? comp._physicalStepPlantPivotReleaseTriggerTime + dt
                : 0.0f;
            if (comp._physicalStepPlantPivotReleaseTriggerTime
                    >= kPlantPivotReleaseTriggerTime) {
                comp._physicalStepPlantPivotReleaseLatched = true;
                comp._physicalStepPlantPivotReleaseTime = 0.0f;
                spdlog::info(
                    "[LocomotionGait] PLANT_PIVOT_RELEASE step={} plant={} "
                    "t={:.3f}s sustained={:.3f}/{:.3f}s "
                    "correction={:.3f}/{:.3f}m horizontalSpeed={:.3f}/{:.3f} "
                    "angularSpeed={:.3f}/{:.3f}radps "
                    "action=fade-pivot-and-require-center",
                    comp._stepSequenceStepIndex,
                    comp._physicalStepSupportSide < 0 ? "RIGHT" : "LEFT",
                    comp._physicalStepPhaseTime,
                    comp._physicalStepPlantPivotReleaseTriggerTime,
                    kPlantPivotReleaseTriggerTime,
                    comp._gaitPlantCorrectionRequested,
                    comp._gaitPlantCorrectionApplied,
                    swingHorizontalSpeed,
                    kPlantPivotLinearSpeedLimit,
                    comp._physicalStepPlantAngularSpeed,
                    kPlantPivotAngularSpeedLimit);
            }
        }
        if (comp._physicalStepPlantPivotReleaseLatched) {
            comp._physicalStepPlantPivotReleaseTime = glm::min(
                comp._physicalStepPlantPivotReleaseTime + dt,
                kPlantPivotReleaseBlendDuration);
            comp._physicalStepPlantPivotReleaseWeight = smoothstep(
                comp._physicalStepPlantPivotReleaseTime
                    / kPlantPivotReleaseBlendDuration);
        } else {
            comp._physicalStepPlantPivotReleaseWeight = 0.0f;
        }
        const float contactMigrationRelease = smoothstep(
            (comp._physicalStepPlantContactMigration - 0.030f) / 0.050f);
        const float pivotOwnershipRelease = glm::max(
            contactMigrationRelease,
            comp._physicalStepPlantPivotReleaseWeight);
        const bool pivotReleaseReady =
            !comp._physicalStepPlantPivotReleaseLatched
            || comp._physicalStepPlantPivotReleaseWeight >= 0.999f;
        const bool pivotQuiet = plantPivotStage
            && pivotContactReady && pivotSoleReady
            && pivotAngularReady && pivotLinearReady
            && pivotReleaseReady;
        comp._physicalStepPlantPivotStableTime = pivotQuiet
            ? comp._physicalStepPlantPivotStableTime + dt : 0.0f;
        comp._physicalStepPlantPivotMaxStableTime = glm::max(
            comp._physicalStepPlantPivotMaxStableTime,
            comp._physicalStepPlantPivotStableTime);

        if (plantPivotStage) {
            if (!pivotContactReady)
                comp._physicalStepPlantPivotContactBlockedTime += dt;
            if (!pivotSoleReady)
                comp._physicalStepPlantPivotSoleBlockedTime += dt;
            if (!pivotAngularReady)
                comp._physicalStepPlantPivotAngularBlockedTime += dt;
            if (!pivotLinearReady)
                comp._physicalStepPlantPivotLinearBlockedTime += dt;

            constexpr float kPlantAnchorTelemetryPeriod = 0.10f;
            comp._physicalStepPlantAnchorTelemetryTime += dt;
            if (comp._physicalStepPlantAnchorTelemetryTime
                    >= kPlantAnchorTelemetryPeriod) {
                comp._physicalStepPlantAnchorTelemetryTime -=
                    kPlantAnchorTelemetryPeriod;
                glm::vec3 pivotError(0.0f);
                if (swingRotationOk) {
                    const glm::vec3 measuredMaterialPoint = swingFoot
                        + swingRotation
                            * comp._physicalStepTouchdownContactLocal;
                    pivotError = comp._physicalStepTouchdownContactWorld
                        - measuredMaterialPoint;
                }
                const glm::vec3 centerTargetError =
                    swing->plantFoot - swingFoot;
                const glm::vec3 contactLocalDelta =
                    comp._physicalStepContactLocal
                    - comp._physicalStepTouchdownContactLocal;
                spdlog::info(
                    "[LocomotionGait] PLANT_ANCHOR_TRACE step={} plant={} "
                    "t={:.3f}s gates=(contact={},sole={},angular={},linear={}) "
                    "quiet={:.3f}/{:.3f}/{:.3f}s sole={:.1f}/{:.1f}deg "
                    "angular=(pitch={:+.3f},roll={:+.3f},yaw={:+.3f},mag={:.3f}/{:.3f}) "
                    "velocity=(fwd={:+.3f},lat={:+.3f},y={:+.3f},h={:.3f}/{:.3f}) "
                    "error=(pivotF={:+.3f},pivotLat={:+.3f},centerF={:+.3f},centerLat={:+.3f}) "
                    "contactDelta=({:+.3f},{:+.3f},{:+.3f}) migration={:.3f} "
                    "supportVelocity=(fwd={:+.3f},lat={:+.3f}) "
                    "correctionNow={:.3f}/{:.3f}m atLimit={} "
                    "correctionPeak={:.3f}/{:.3f}m saturated={} "
                    "pivotRelease=(latched={},trigger={:.3f}/{:.3f}s,weight={:.2f})",
                    comp._stepSequenceStepIndex,
                    comp._physicalStepSupportSide < 0 ? "RIGHT" : "LEFT",
                    comp._physicalStepPhaseTime,
                    pivotContactReady ? "ok" : "BLOCK",
                    pivotSoleReady ? "ok" : "BLOCK",
                    pivotAngularReady ? "ok" : "BLOCK",
                    pivotLinearReady ? "ok" : "BLOCK",
                    comp._physicalStepPlantPivotStableTime,
                    comp._physicalStepPlantPivotMaxStableTime,
                    kPlantPivotQuietTime,
                    comp._gaitSoleAngularErrorDeg,
                    activePlantSoleToleranceDeg,
                    glm::dot(swingAngularVelocity, comp._physicalStepRight),
                    glm::dot(swingAngularVelocity, comp._physicalStepForward),
                    swingAngularVelocity.y,
                    comp._physicalStepPlantAngularSpeed,
                    kPlantPivotAngularSpeedLimit,
                    glm::dot(swingVelocity, comp._physicalStepForward),
                    glm::dot(swingVelocity, comp._physicalStepRight),
                    swingVelocity.y,
                    swingHorizontalSpeed,
                    kPlantPivotLinearSpeedLimit,
                    glm::dot(pivotError, comp._physicalStepForward),
                    glm::dot(pivotError, comp._physicalStepRight),
                    glm::dot(centerTargetError, comp._physicalStepForward),
                    glm::dot(centerTargetError, comp._physicalStepRight),
                    contactLocalDelta.x,
                    contactLocalDelta.y,
                    contactLocalDelta.z,
                    comp._physicalStepPlantContactMigration,
                    glm::dot(comp._gaitSupportCommandVelocity,
                             comp._physicalStepForward),
                    glm::dot(comp._gaitSupportCommandVelocity,
                             comp._physicalStepRight),
                    comp._gaitPlantCorrectionRequested,
                    comp._gaitPlantCorrectionApplied,
                    comp._gaitPlantCorrectionAtLimit ? "yes" : "no",
                    comp._gaitPlantCorrectionPeakRequested,
                    comp._gaitPlantCorrectionPeakApplied,
                    comp._gaitPlantCorrectionSaturated ? "yes" : "no",
                    comp._physicalStepPlantPivotReleaseLatched
                        ? "yes" : "no",
                    comp._physicalStepPlantPivotReleaseTriggerTime,
                    kPlantPivotReleaseTriggerTime,
                    comp._physicalStepPlantPivotReleaseWeight);
            }
        }

        if (comp._physicalStepTouchdownContactValid
            && comp._physicalStepPlantContactMigration
                >= kPlantContactMigrationNotice
            && !comp._physicalStepPlantContactMigrationLogged) {
            comp._physicalStepPlantContactMigrationLogged = true;
            spdlog::info(
                "[LocomotionGait] PLANT_CONTACT_MIGRATION step={} plant={} "
                "distance={:.3f}m sole={:.1f}deg angularSpeed={:.3f}radps "
                "action=release-impact-pivot",
                comp._stepSequenceStepIndex,
                comp._physicalStepSupportSide < 0 ? "RIGHT" : "LEFT",
                comp._physicalStepPlantContactMigration,
                comp._gaitSoleAngularErrorDeg,
                comp._physicalStepPlantAngularSpeed);
        }

        const bool straightMigrationCenterHandoff = plantPivotStage
            && walkingOverlapHandoff
            && comp._physicalStepPlantContactMigration
                >= kPlantContactMigrationNotice
            && pivotContactReady && pivotSoleReady && pivotLinearReady
            // Migration can invalidate the impact point before the full 0.75 rad/s quiet
            // proof, but a rapidly whipping sole must not be promoted directly. The prior
            // run exposed a forced handoff at 3.15 rad/s; 1.5 retains the recoverable
            // 0.35-1.30 rad/s cases while leaving faster motion in the damping stage.
            && comp._physicalStepPlantAngularSpeed <= 1.50f;
        if (((comp._physicalStepPlantPivotStableTime
                    >= kPlantPivotQuietTime
                && pivotReleaseReady)
             || straightMigrationCenterHandoff)
            && !comp._physicalStepPlantCenterAnchorActive) {
            const glm::vec3 pivotCompatibleCenter =
                comp._physicalStepTouchdownContactWorld
                - nominalFootWorldRotation(*swing)
                    * comp._physicalStepTouchdownContactLocal;
            // Begin from the ownership mix actually used by IK on the preceding frame.
            // Starting at the raw pivot after migration had already released it would
            // reintroduce the obsolete-edge pull at the handoff boundary.
            const glm::vec3 currentOwnedCenter = glm::mix(
                pivotCompatibleCenter, swingFoot,
                pivotOwnershipRelease);
            comp._physicalStepPlantCenterAnchorActive = true;
            comp._physicalStepPlantCenterBlendTime = 0.0f;
            comp._physicalStepPlantAnchorHandoffPhaseTime =
                comp._physicalStepPhaseTime;
            comp._physicalStepPlantCenterAnchorStart = currentOwnedCenter;
            comp._physicalStepPlantCenterAnchorTarget = swingFoot;
            swing->plantFoot = currentOwnedCenter;

            // From this point drift means displacement of the captured sole center,
            // not displacement of a first-impact material point that no longer bears load.
            comp._physicalStepTouchdownContactValid = false;
            comp._physicalStepTouchdownPlant = swingFoot;
            comp._physicalStepPlantDrift = 0.0f;
            comp._physicalStepPlantCenterTravel = 0.0f;
            comp._gaitPlantPreviousDrift = 0.0f;
            comp._gaitPlantDriftRate = 0.0f;
            comp._physicalStepPlantAcquireStableTime = 0.0f;
            comp._physicalStepPlantSettledOffsetTime = 0.0f;
            comp._physicalStepPlantUnsafeTime = 0.0f;
            spdlog::info(
                "[LocomotionGait] PLANT_ANCHOR_HANDOFF step={} plant={} "
                "mode={} quiet={:.3f}s sole={:.1f}deg "
                "angularSpeed={:.3f}radps contactMigration={:.3f}m "
                "pivotRelease={:.2f} blend={:.3f}s centerShift={:.3f}m "
                "maxQuiet={:.3f}s blocked=(contact={:.3f},sole={:.3f},"
                "angular={:.3f},linear={:.3f})s "
                "supportReadyBudget={:.3f}s",
                comp._stepSequenceStepIndex,
                comp._physicalStepSupportSide < 0 ? "RIGHT" : "LEFT",
                straightMigrationCenterHandoff
                    ? "migration-reacquire-center"
                    : (comp._physicalStepPlantPivotReleaseLatched
                        ? "recovery-release-to-center" : "pivot-to-center"),
                comp._physicalStepPlantPivotStableTime,
                comp._gaitSoleAngularErrorDeg,
                comp._physicalStepPlantAngularSpeed,
                comp._physicalStepPlantContactMigration,
                comp._physicalStepPlantPivotReleaseWeight,
                kPlantCenterBlendDuration,
                horizontalDistance(
                    comp._physicalStepPlantCenterAnchorStart,
                    comp._physicalStepPlantCenterAnchorTarget),
                comp._physicalStepPlantPivotMaxStableTime,
                comp._physicalStepPlantPivotContactBlockedTime,
                comp._physicalStepPlantPivotSoleBlockedTime,
                comp._physicalStepPlantPivotAngularBlockedTime,
                comp._physicalStepPlantPivotLinearBlockedTime,
                kPlantCenterBlendDuration + cadencePlantAcquireTime
                    + kPostHandoffAcquireMargin);

            // The impact pivot has completed its job. Enter a separate state that owns
            // the center anchor and proves it under a stationary support command before
            // load transfer is allowed to begin.
            if (continuousEnabled && comp._physicalStepPhase == kSettle) {
                comp._physicalStepPhase = kSupportReady;
                comp._physicalStepPhaseTime = 0.0f;
                comp._physicalStepPlantAcquireStableTime = 0.0f;
                spdlog::info(
                    "[LocomotionGait] SUPPORT_OWNERSHIP_BEGIN step={} plant={} "
                    "anchor=center blend={:.3f}/{:.3f}s "
                    "requiredStable={:.3f}s supportAction=hold",
                    comp._stepSequenceStepIndex,
                    comp._physicalStepSupportSide < 0 ? "RIGHT" : "LEFT",
                    comp._physicalStepPlantCenterBlendTime,
                    kPlantCenterBlendDuration,
                    cadencePlantAcquireTime);
            }
        }
        if (comp._physicalStepPlantCenterAnchorActive) {
            comp._physicalStepPlantCenterBlendTime = glm::min(
                comp._physicalStepPlantCenterBlendTime + dt,
                kPlantCenterBlendDuration);
            const float centerBlend = smoothstep(
                comp._physicalStepPlantCenterBlendTime
                    / kPlantCenterBlendDuration);
            swing->plantFoot = glm::mix(
                comp._physicalStepPlantCenterAnchorStart,
                comp._physicalStepPlantCenterAnchorTarget,
                centerBlend);
        }

        // These values describe only the correction produced by this frame's pivot
        // solve. The release detector consumes them on the next frame.
        comp._gaitPlantCorrectionRequested = 0.0f;
        comp._gaitPlantCorrectionApplied = 0.0f;
        comp._gaitPlantCorrectionAtLimit = false;

        const bool liftAssistActive = comp._physicalStepPhase >= kTakeoff
                                   && comp._physicalStepPhase <= kDescent;
        if (liftAssistActive) {
            float liftFade = 1.0f;
            if (comp._physicalStepPhase >= kSwing
                && comp._physicalStepPhase <= kDescent) {
                const float descentProgress = glm::clamp(
                    (comp._physicalStepTrajectoryT - kSwingArrivalT)
                        / (1.0f - kSwingArrivalT),
                    0.0f, 1.0f);
                liftFade = comp._physicalStepTrajectoryT < kSwingArrivalT
                    ? 1.0f - 0.5f * smoothstep(
                        (comp._physicalStepTrajectoryT - 0.40f)
                            / (kSwingArrivalT - 0.40f))
                    : 0.5f * (1.0f
                        - smoothstep(descentProgress / 0.70f));
            }
            rag.locomotionLiftBone = swing->footIdx;
            rag.locomotionLiftTargetY = desiredFoot.y;
            rag.locomotionLiftFrequency = glm::max(comp.liftFrequency, 0.0f);
            rag.locomotionLiftMaxForce = glm::max(comp.liftMaxForce, 0.0f)
                                       * glm::clamp(liftFade, 0.0f, 1.0f);
        } else {
            rag.locomotionLiftBone = -1;
            rag.locomotionLiftMaxForce = 0.0f;
        }

        auto solveLegIK = [&](Leg& controlledLeg,
                              const glm::vec3& measuredFoot,
                              const glm::vec3& targetFoot,
                              bool movingFoot) {
            glm::vec3 controlledFoot = targetFoot;
            const bool governedWalkingSolve = governedWalkingSwing
                && movingFoot && &controlledLeg == swing
                && comp._physicalStepPhase >= kTakeoff
                && comp._physicalStepPhase <= kTouchdownWait;
            // Both turn roles need one coherent hip/knee/ankle rate scale so the physical
            // sole follows its oblique path without joint-rate fighting. Straight walking
            // keeps the established filtered hip/knee solve; sharing one rate scale across
            // all three joints made that path pulse between heavily throttled and full-rate
            // commands.
            const bool coherentWalkingSolve = governedWalkingSolve
                && hasYawBearingTurnStep();
            if (movingFoot) {
                comp._gaitFootCorrection = 0.0f;
                comp._gaitFootCorrectionForward = 0.0f;
                comp._gaitFootTargetSpeed = 0.0f;
            }
            if (continuousEnabled && movingFoot
                && comp._physicalStepPhase >= kSwing
                && comp._physicalStepPhase <= kTouchdownWait) {
                // Joint-space motors are closed-loop, but the sole previously had no
                // horizontal task-space feedback. _physicalStepDesiredFoot still contains the
                // previous frame here, so lead the moving target by its actual velocity
                // and add a bounded correction from the measured sole-center error. The
                // correction collapses to zero when the physical foot catches the nominal
                // path, so the admitted foothold remains the equilibrium rather than being
                // permanently displaced.
                glm::vec3 positionError = targetFoot - measuredFoot;
                positionError.y = 0.0f;
                glm::vec3 targetVelocity(0.0f);
                if (dt > 1e-6f) {
                    targetVelocity = (targetFoot - comp._physicalStepDesiredFoot) / dt;
                    targetVelocity.y = 0.0f;
                }
                const glm::vec3 positionCorrection = positionError
                    * glm::clamp(comp.gaitFootPositionGain, 0.0f, 1.0f);
                const glm::vec3 targetLeadCorrection = targetVelocity
                    * glm::max(comp.gaitFootVelocityLeadTime, 0.0f);
                glm::vec3 landingDampingCorrection(0.0f);
                if (governedWalkingSolve
                    && comp._physicalStepPhase >= kDescent) {
                    glm::vec3 measuredHorizontalVelocity = swingVelocity;
                    measuredHorizontalVelocity.y = 0.0f;
                    const float landingBlend = smoothstep(glm::clamp(
                        (comp._physicalStepTrajectoryT - kSwingArrivalT)
                            / glm::max(activeSwingLandingBrakeT
                                - kSwingArrivalT, 0.01f),
                        0.0f, 1.0f));
                    landingDampingCorrection = -measuredHorizontalVelocity
                        * glm::clamp(comp.gaitLandingLinearDampingTime,
                                     0.0f, 0.25f)
                        * landingBlend;
                }
                glm::vec3 footCorrection = positionCorrection
                    + targetLeadCorrection + landingDampingCorrection;
                const float maximumCorrection = glm::max(
                    comp.gaitMaxFootCorrection, 0.0f);
                const float correctionLength = glm::length(footCorrection);
                if (correctionLength > maximumCorrection
                    && correctionLength > 1e-6f) {
                    footCorrection *= maximumCorrection / correctionLength;
                }
                // Preserve task-space feedback across ARRIVAL -> DESCENT instead of
                // dropping it at the milestone. Fade it to zero before touchdown so the
                // admitted foothold remains the final equilibrium target.
                const float descentProgress = glm::clamp(
                    (comp._physicalStepTrajectoryT - kSwingArrivalT)
                        / (1.0f - kSwingArrivalT),
                    0.0f, 1.0f);
                if (governedWalkingSolve) {
                    // Proportional error has the foothold itself as its equilibrium and
                    // is still needed while the physical inside foot brakes. Fade only
                    // feed-forward; removing all feedback at t=1 caused the final 7-9 cm
                    // error to survive into touchdown wait.
                    footCorrection = positionCorrection
                        + targetLeadCorrection
                            * (1.0f - smoothstep(descentProgress))
                        + landingDampingCorrection;
                    const float governedCorrectionLength =
                        glm::length(footCorrection);
                    if (governedCorrectionLength > maximumCorrection
                        && governedCorrectionLength > 1e-6f) {
                        footCorrection *= maximumCorrection
                            / governedCorrectionLength;
                    }
                } else {
                    footCorrection *= 1.0f - smoothstep(descentProgress);
                }
                controlledFoot += footCorrection;
                comp._gaitFootCorrection = glm::length(footCorrection);
                comp._gaitFootCorrectionForward = glm::dot(
                    footCorrection, comp._physicalStepForward);
                comp._gaitFootTargetSpeed = glm::length(targetVelocity);
            }
            float soleLevelBlend = 0.0f;
            if (continuousEnabled
                && controlledLeg.groundReferenceFootRotationValid) {
                if (movingFoot && comp._physicalStepPhase >= kSwing
                    && comp._physicalStepPhase <= kDescent) {
                    const float trajectoryLevelTime =
                        comp._physicalStepTrajectoryT
                        / glm::max(kSwingArrivalT, 0.01f)
                        * cadenceSwingTime;
                    soleLevelBlend = smoothstep(glm::clamp(
                        trajectoryLevelTime
                            / glm::max(comp.gaitSoleLevelTime, 0.10f),
                        0.0f, 1.0f));
                } else if (!movingFoot || comp._physicalStepPhase >= kArrival) {
                    soleLevelBlend = 1.0f;
                }
            }
            if (movingFoot)
                comp._gaitSoleLevelBlend = soleLevelBlend;
            const glm::quat desiredFootWorldTarget = glm::normalize(glm::slerp(
                glm::normalize(controlledLeg.plantedFootWorldRotation),
                nominalFootWorldRotation(controlledLeg), soleLevelBlend));
            glm::quat desiredFootWorld = desiredFootWorldTarget;
            const bool governMovingSole = governedWalkingSolve
                && comp._physicalStepPhase >= kSwing
                && comp._physicalStepPhase <= kTouchdownWait;
            if (governMovingSole) {
                if (!comp._gaitSwingSoleCommandValid) {
                    comp._gaitSwingSoleCommandWorld =
                        physicalSwingFootRotationOk
                            ? glm::normalize(physicalSwingFootWorld)
                            : glm::normalize(
                                controlledLeg.plantedFootWorldRotation);
                    comp._gaitSwingSoleCommandValid = true;
                    comp._gaitSwingCommandAngularSpeed = 0.0f;
                }
                glm::quat command = glm::normalize(
                    comp._gaitSwingSoleCommandWorld);
                glm::quat target = desiredFootWorldTarget;
                if (glm::dot(command, target) < 0.0f)
                    target = -target;

                comp._gaitSwingSoleCommandErrorDeg =
                    physicalSwingFootRotationOk
                        ? rotationDifferenceDeg(
                            physicalSwingFootWorld, command)
                        : 0.0f;
                constexpr float kSoleTrackingInnerDeg = 8.0f;
                constexpr float kSoleTrackingOuterDeg = 20.0f;
                const float soleTrackingScale = 1.0f - smoothstep(
                    (comp._gaitSwingSoleCommandErrorDeg
                        - kSoleTrackingInnerDeg)
                        / (kSoleTrackingOuterDeg
                            - kSoleTrackingInnerDeg));
                const bool turnSoleGovernor = hasYawBearingTurnStep();
                const float angularCommandAcceleration = turnSoleGovernor
                    ? glm::clamp(comp.gaitTurnAngularAcceleration,
                                 1.0f, 40.0f)
                    : glm::clamp(comp.gaitSoleAngularAcceleration,
                                 1.0f, 40.0f);
                const float configuredAngularSpeedLimit = glm::max(
                    turnSoleGovernor
                        ? glm::clamp(comp.gaitTurnAngularSpeedLimit,
                                     0.25f, 8.0f)
                        : glm::clamp(comp.gaitSoleAngularSpeedLimit,
                                     0.25f, 8.0f),
                    0.75f);
                const float angularSpeedLimit = glm::clamp(
                    turnSoleGovernor
                        ? glm::max(
                            comp._gaitTurnPlan.admittedAngularSpeed, 0.75f)
                        : configuredAngularSpeedLimit,
                    0.75f, configuredAngularSpeedLimit);
                const float remainingAngle = glm::radians(
                    rotationDifferenceDeg(command, target));
                const float brakingSpeed = std::sqrt(glm::max(
                    2.0f * angularCommandAcceleration
                        * remainingAngle,
                    0.0f));
                const float desiredAngularSpeed = glm::min(
                    angularSpeedLimit, brakingSpeed) * soleTrackingScale;
                comp._gaitSwingCommandAngularSpeed = moveScalarToward(
                    comp._gaitSwingCommandAngularSpeed,
                    desiredAngularSpeed,
                    angularCommandAcceleration * dt);
                if (remainingAngle > 1e-6f) {
                    const float interpolation = glm::clamp(
                        comp._gaitSwingCommandAngularSpeed * dt
                            / remainingAngle,
                        0.0f, 1.0f);
                    command = glm::normalize(glm::slerp(
                        command, target, interpolation));
                } else {
                    command = target;
                }
                comp._gaitSwingSoleCommandWorld = command;
                desiredFootWorld = command;
            } else if (movingFoot && !governedWalkingSwing) {
                comp._gaitSwingCommandAngularSpeed = 0.0f;
                comp._gaitSwingSoleCommandErrorDeg = 0.0f;
                comp._gaitSwingSoleCommandValid = false;
            }
            if (governMovingSole && !hasYawBearingTurnStep()
                && swingAngularVelocityOk) {
                // The straight-walk chain filters hip and knee independently. Their
                // measured motion can therefore inject pitch/roll velocity into the sole
                // even while its world-space command is level. Add a small derivative
                // correction at the terminal task target; the ankle command below remains
                // envelope-, acceleration-, and rate-limited, so this damps the observed
                // 10-14 rad/s whip without creating a raw measured-pose chase.
                glm::vec3 levelingAngularVelocity = swingAngularVelocity;
                levelingAngularVelocity.y = 0.0f;
                const float levelingAngularSpeed =
                    glm::length(levelingAngularVelocity);
                if (levelingAngularSpeed > 1e-4f) {
                    constexpr float kSoleAngularDampingTime = 0.025f;
                    constexpr float kMaximumSoleDampingDeg = 6.0f;
                    const float dampingAngle = glm::min(
                        levelingAngularSpeed * kSoleAngularDampingTime,
                        glm::radians(kMaximumSoleDampingDeg));
                    desiredFootWorld = glm::normalize(
                        glm::angleAxis(
                            -dampingAngle,
                            levelingAngularVelocity / levelingAngularSpeed)
                        * desiredFootWorld);
                }
            }
            const bool allocateWalkingOrientation = coherentWalkingSolve;
            const glm::quat exactAllocatedFootWorld = desiredFootWorld;
            float orientationPriority = 1.0f;
            float relaxedOrientationDeg = 0.0f;
            const bool touchdownPivotOwnsPlant = continuousEnabled
                && !movingFoot && &controlledLeg == swing
                && comp._physicalStepTouchdownContactValid;
            if (touchdownPivotOwnsPlant) {
                // The captured contact is the translational invariant. As the ankle levels
                // the sole, derive the compatible body-center target around that world
                // pivot instead of commanding an impossible fixed center and fixed edge.
                const glm::vec3 pivotCompatibleCenter =
                    comp._physicalStepTouchdownContactWorld
                    - desiredFootWorld
                        * comp._physicalStepTouchdownContactLocal;
                // A large change in the collision-local contact means the sole has rolled
                // away from its impact edge. Fade both pivot position ownership and its
                // restoring correction, allowing the foot to quiet before the center
                // anchor captures it. Keeping full authority here caused the observed
                // heel/toe contact flips to grow on every subsequent step.
                controlledFoot = glm::mix(
                    pivotCompatibleCenter, measuredFoot,
                    pivotOwnershipRelease);
                if (swingRotationOk) {
                    const glm::vec3 measuredMaterialPoint = measuredFoot
                        + swingRotation
                            * comp._physicalStepTouchdownContactLocal;
                    glm::vec3 plantCorrection =
                        comp._physicalStepTouchdownContactWorld
                        - measuredMaterialPoint;
                    plantCorrection.y = 0.0f;
                    plantCorrection *= 0.65f
                        * (1.0f - pivotOwnershipRelease);
                    constexpr float kMaximumPlantCorrection = 0.020f;
                    const float correctionLength = glm::length(plantCorrection);
                    comp._gaitPlantCorrectionRequested = correctionLength;
                    comp._gaitPlantCorrectionAtLimit =
                        correctionLength >= kMaximumPlantCorrection;
                    comp._gaitPlantCorrectionPeakRequested = glm::max(
                        comp._gaitPlantCorrectionPeakRequested,
                        correctionLength);
                    if (correctionLength > kMaximumPlantCorrection
                        && correctionLength > 1e-6f) {
                        comp._gaitPlantCorrectionSaturated = true;
                        plantCorrection *=
                            kMaximumPlantCorrection / correctionLength;
                    }
                    comp._gaitPlantCorrectionPeakApplied = glm::max(
                        comp._gaitPlantCorrectionPeakApplied,
                        glm::length(plantCorrection));
                    comp._gaitPlantCorrectionApplied =
                        glm::length(plantCorrection);
                    controlledFoot += plantCorrection;
                }
            } else if (continuousEnabled && !movingFoot) {
                // Flat established stance feet retain the center-based restoring solve.
                glm::vec3 plantCorrection = targetFoot - measuredFoot;
                plantCorrection.y = 0.0f;
                plantCorrection *= 0.65f;
                constexpr float kMaximumPlantCorrection = 0.020f;
                const float correctionLength = glm::length(plantCorrection);
                if (correctionLength > kMaximumPlantCorrection
                    && correctionLength > 1e-6f) {
                    plantCorrection *=
                        kMaximumPlantCorrection / correctionLength;
                }
                controlledFoot += plantCorrection;
            }
            controlledLeg.desiredFoot = controlledFoot;
            const glm::vec3 hipPosition = physicalPosition(controlledLeg.hipIdx);
            glm::vec3 desiredAnkle = controlledFoot
                + ankleFromFootWorld(controlledLeg, desiredFootWorld);
            const glm::vec3 requestedAnkle = desiredAnkle;
            glm::vec3 toTarget = desiredAnkle - hipPosition;
            const float requestedReach = glm::length(toTarget);
            const glm::vec3 upperSegmentLocal =
                controlledLeg.segmentGeometryValid
                    ? controlledLeg.upperSegmentLocal
                    : skeleton.bones[controlledLeg.kneeIdx].localT;
            const glm::vec3 lowerSegmentLocal =
                controlledLeg.segmentGeometryValid
                    ? controlledLeg.lowerSegmentLocal
                    : skeleton.bones[controlledLeg.ankleIdx].localT;
            const float upperLength = glm::length(upperSegmentLocal);
            const float lowerLength = glm::length(lowerSegmentLocal);
            const float configuredReach = (upperLength + lowerLength)
                * glm::clamp(comp.maxLegReachFraction, 0.70f, 0.99f);
            const float maxReach = comp._physicalStepReachLimit > 1e-4f
                ? glm::max(configuredReach, comp._physicalStepReachLimit)
                : configuredReach;
            if (const float reach = glm::length(toTarget);
                reach > maxReach && reach > 1e-5f) {
                toTarget *= maxReach / reach;
                desiredAnkle = hipPosition + toTarget;
            }
            if (continuousEnabled && movingFoot) {
                const glm::vec3 physicalAnkle = physicalPosition(
                    controlledLeg.ankleIdx);
                const glm::vec3 reachCorrection = requestedAnkle - desiredAnkle;
                const glm::vec3 hipTravel = comp._gaitIkPlanHipValid
                    ? hipPosition - comp._gaitIkPlanHip : glm::vec3(0.0f);
                comp._gaitIkRequestedReach = requestedReach;
                comp._gaitIkClampedReach = glm::length(toTarget);
                comp._gaitIkMaxReach = maxReach;
                comp._gaitIkPhysicalReach = glm::length(
                    physicalAnkle - hipPosition);
                comp._gaitIkReachShortfall = glm::length(reachCorrection);
                comp._gaitIkReachShortfallForward = glm::dot(
                    reachCorrection, comp._physicalStepForward);
                comp._gaitIkHipTravelForward = glm::dot(
                    hipTravel, comp._physicalStepForward);
                comp._gaitIkHipTravelLateral = glm::dot(
                    hipTravel, comp._physicalStepRight);
                comp._gaitIkHipTravelVertical = hipTravel.y;
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
                comp._gaitIkKneeBendDeg = glm::degrees(kneeBend);
            const float kneeDelta = kneeBend - controlledLeg.referenceKneeBend;
            const glm::quat kneeTarget = glm::normalize(
                controlledLeg.referenceKneeLocal
                * glm::angleAxis(
                    kneeDelta, glm::normalize(controlledLeg.kneeHingeAxis)));

            const glm::vec3 worldForward = glm::normalize(toTarget);
            const glm::vec3 kneePole = nominalKneePoleWorld(controlledLeg);
            glm::vec3 worldBend = kneePole
                - worldForward * glm::dot(kneePole, worldForward);
            if (glm::dot(worldBend, worldBend) < 1e-8f)
                worldBend = comp._physicalStepForward;
            worldBend -= worldForward * glm::dot(worldBend, worldForward);
            if (glm::dot(worldBend, worldBend) < 1e-8f)
                worldBend = comp._physicalStepRight;
            else
                worldBend = glm::normalize(worldBend);
            const float hipCos = glm::clamp(
                (upperLength * upperLength + distance * distance
                 - lowerLength * lowerLength)
                / (2.0f * upperLength * distance), -1.0f, 1.0f);
            const float hipSin = std::sqrt(glm::max(1.0f - hipCos * hipCos, 0.0f));
            const glm::quat parentWorld = ParentWorldRot(
                rag, skeleton, animator, entityWorld, controlledLeg.hipIdx);
            Envelope hipEnvelope;
            hipEnvelope.twistAxis = controlledLeg.hipTwistAxis;
            hipEnvelope.swingNormalDeg = controlledLeg.hipSwingNormalDeg;
            hipEnvelope.swingPlaneDeg = controlledLeg.hipSwingPlaneDeg;
            hipEnvelope.twistMinDeg = controlledLeg.hipTwistMinDeg;
            hipEnvelope.twistMaxDeg = controlledLeg.hipTwistMaxDeg;
            Envelope ankleEnvelope;
            ankleEnvelope.twistAxis = controlledLeg.ankleAxis;
            ankleEnvelope.swingNormalDeg = controlledLeg.ankleSwingNormalDeg;
            ankleEnvelope.swingPlaneDeg = controlledLeg.ankleSwingPlaneDeg;
            ankleEnvelope.twistMinDeg = controlledLeg.ankleTwistMinDeg;
            ankleEnvelope.twistMaxDeg = controlledLeg.ankleTwistMaxDeg;
            const float configuredPoseResponse = glm::max(
                comp.standingPoseResponse, 0.01f);
            // A 100 ms standing filter consumes almost half of the requested 220 ms
            // swing and leaves the task-space governor waiting on normal servo lag. Use a
            // faster, still exponential response only for straight governed swing. The
            // tracking tube, joint envelopes, rate limits, and landing gates remain intact.
            const float poseResponse = governedWalkingSolve
                && !hasYawBearingTurnStep()
                ? glm::min(configuredPoseResponse, 0.05f)
                : movingFoot
                    ? configuredPoseResponse
                : glm::min(glm::max(
                    comp.standingPoseResponse, 0.01f), 0.05f);
            const float alpha = 1.0f - std::exp(-dt / poseResponse);

            // A two-bone position solve has one redundant knee-plane degree of freedom.
            // Slice 2m uses it continuously so turn orientation is shared by the whole
            // leg instead of being assigned entirely to the terminal ankle.
            struct WholeLegCandidate {
                float swivelRad = 0.0f;
                glm::quat unconstrainedHip { 1, 0, 0, 0 };
                glm::quat hip { 1, 0, 0, 0 };
                glm::quat kneeWorld { 1, 0, 0, 0 };
                glm::quat ankle { 1, 0, 0, 0 };
                float hipClampDeg = 0.0f;
                float ankleClampDeg = 0.0f;
                float positionClosure = 1.0f;
                float orientationClosureDeg = 180.0f;
                float cost = 0.0f;
                bool positionClosed = false;
            };
            auto evaluateWholeLeg = [&](float swivelRad,
                                        const glm::quat& footWorld) {
                WholeLegCandidate candidate;
                candidate.swivelRad = swivelRad;
                const glm::vec3 swivelBend = glm::normalize(
                    glm::angleAxis(swivelRad, worldForward) * worldBend);
                const glm::vec3 candidateUpper = glm::normalize(
                    worldForward * hipCos + swivelBend * hipSin);
                const glm::vec3 candidateAnkle = controlledFoot
                    + ankleFromFootWorld(controlledLeg, footWorld);
                const glm::vec3 candidateKnee = hipPosition
                    + candidateUpper * upperLength;
                const glm::vec3 candidateLower =
                    candidateAnkle - candidateKnee;
                glm::quat candidateHipWorld(1, 0, 0, 0);
                if (!RotationMatchingVectorPair(
                        upperSegmentLocal,
                        kneeTarget * lowerSegmentLocal,
                        candidateUpper, candidateLower,
                        candidateHipWorld)) {
                    candidate.cost = 1e9f;
                    return candidate;
                }
                candidate.unconstrainedHip = glm::normalize(
                    glm::conjugate(parentWorld) * candidateHipWorld);
                candidate.hip = ClampToEnvelope(
                    hipEnvelope,
                    skeleton.bones[controlledLeg.hipIdx].localR,
                    candidate.unconstrainedHip,
                    comp.hipLimitMarginDeg);
                const glm::quat candidateHipCommand =
                    coherentWalkingSolve
                        ? candidate.hip
                        : glm::normalize(glm::slerp(
                            controlledLeg.hipCommand,
                            candidate.hip, alpha));
                const glm::quat candidateKneeCommand =
                    coherentWalkingSolve
                        ? kneeTarget
                        : glm::normalize(glm::slerp(
                            controlledLeg.kneeCommand,
                            kneeTarget, alpha));
                const glm::quat resolvedHipWorld = glm::normalize(
                    parentWorld * candidateHipCommand);
                candidate.kneeWorld = glm::normalize(
                    resolvedHipWorld * candidateKneeCommand);
                const glm::quat requestedAnkleLocal = glm::normalize(
                    glm::conjugate(candidate.kneeWorld)
                    * footWorld
                    * glm::conjugate(controlledLeg.referenceFootLocal));
                const glm::quat boundedAnkleLocal = ClampToEnvelope(
                    ankleEnvelope,
                    skeleton.bones[controlledLeg.ankleIdx].localR,
                    requestedAnkleLocal,
                    comp.hipLimitMarginDeg);
                candidate.ankle = boundedAnkleLocal;
                candidate.hipClampDeg = rotationDifferenceDeg(
                    candidate.unconstrainedHip, candidate.hip);
                candidate.ankleClampDeg = rotationDifferenceDeg(
                    requestedAnkleLocal, boundedAnkleLocal);
                const glm::vec3 commandedKneePosition = hipPosition
                    + resolvedHipWorld * upperSegmentLocal;
                const glm::vec3 commandedAnklePosition =
                    commandedKneePosition
                    + candidate.kneeWorld * lowerSegmentLocal;
                const glm::quat commandedFootWorld = glm::normalize(
                    candidate.kneeWorld * boundedAnkleLocal
                    * controlledLeg.referenceFootLocal);
                const glm::vec3 commandedFootPosition =
                    commandedAnklePosition
                    - commandedFootWorld
                        * controlledLeg.ankleFromFootLocal;
                candidate.positionClosure = glm::length(
                    commandedFootPosition - controlledFoot);
                candidate.orientationClosureDeg = rotationDifferenceDeg(
                    commandedFootWorld, footWorld);
                constexpr float kPositionClosureTolerance = 0.003f;
                candidate.positionClosed =
                    candidate.positionClosure <= kPositionClosureTolerance;
                const float branchMotionDeg = std::abs(glm::degrees(
                    swivelRad - controlledLeg.kneePlaneSwivelRad));
                candidate.cost = candidate.positionClosed
                    ? candidate.ankleClampDeg * 20.0f
                    + candidate.hipClampDeg * 4.0f
                    + branchMotionDeg * 0.025f
                    + std::abs(glm::degrees(swivelRad)) * 0.002f
                    : 1e9f + candidate.positionClosure * 1000.0f;
                return candidate;
            };
            auto selectWholeLeg = [&](const glm::quat& footWorld) {
                WholeLegCandidate best = evaluateWholeLeg(
                    controlledLeg.kneePlaneSwivelRad, footWorld);
                constexpr int kSwivelSamples = 25;
                constexpr float kMaximumSwivelRad =
                    glm::radians(60.0f);
                for (int sample = 0; sample < kSwivelSamples; ++sample) {
                    const float u = static_cast<float>(sample)
                        / static_cast<float>(kSwivelSamples - 1);
                    const float swivel = glm::mix(
                        -kMaximumSwivelRad, kMaximumSwivelRad, u);
                    const WholeLegCandidate candidate =
                        evaluateWholeLeg(swivel, footWorld);
                    if (candidate.positionClosed
                        && candidate.cost < best.cost)
                        best = candidate;
                }
                float refinementRadius = glm::radians(2.5f);
                for (int refinement = 0; refinement < 2; ++refinement) {
                    for (int offset = -2; offset <= 2; ++offset) {
                        const float swivel = glm::clamp(
                            best.swivelRad
                                + static_cast<float>(offset)
                                    * refinementRadius,
                            -kMaximumSwivelRad, kMaximumSwivelRad);
                        const WholeLegCandidate candidate =
                            evaluateWholeLeg(swivel, footWorld);
                        if (candidate.positionClosed
                            && candidate.cost < best.cost)
                            best = candidate;
                    }
                    refinementRadius *= 0.5f;
                }
                return best;
            };

            WholeLegCandidate exactCandidate = evaluateWholeLeg(
                0.0f, exactAllocatedFootWorld);
            WholeLegCandidate selectedCandidate = exactCandidate;
            if (allocateWalkingOrientation) {
                const WholeLegCandidate counterfactualCandidate = selectWholeLeg(
                    exactAllocatedFootWorld);
                // Correct two-vector swivel candidates are retained counterfactually until
                // the zero-swivel runtime path passes mirrored physical validation. The
                // previous one-vector construction moved the sole laterally by 4-12 cm.
                // No sampled swivel may affect a joint target in this corrective slice.
                exactCandidate = counterfactualCandidate;
                controlledLeg.kneePlaneSwivelRad = 0.0f;
                selectedCandidate = evaluateWholeLeg(
                    0.0f, desiredFootWorld);
                comp._gaitIkKneeSwivelRequestedDeg = glm::degrees(
                    exactCandidate.swivelRad);
                comp._gaitIkKneeSwivelCandidateClosure =
                    exactCandidate.positionClosure;
                comp._gaitIkKneeSwivelCandidateAccepted =
                    exactCandidate.positionClosed;
            } else {
                controlledLeg.kneePlaneSwivelRad = 0.0f;
                selectedCandidate = evaluateWholeLeg(
                    0.0f, desiredFootWorld);
                comp._gaitIkKneeSwivelRequestedDeg = 0.0f;
                comp._gaitIkKneeSwivelCandidateClosure =
                    selectedCandidate.positionClosure;
                comp._gaitIkKneeSwivelCandidateAccepted =
                    selectedCandidate.positionClosed;
            }
            glm::quat hipTarget = selectedCandidate.hip;
            const glm::quat unconstrainedHipTarget =
                selectedCandidate.unconstrainedHip;
            if (continuousEnabled && movingFoot) {
                comp._gaitIkExactAnkleClampDeg =
                    exactCandidate.ankleClampDeg;
                comp._gaitIkKneeSwivelAppliedDeg = glm::degrees(
                    controlledLeg.kneePlaneSwivelRad);
                comp._gaitIkOrientationPriority = orientationPriority;
                comp._gaitIkOrientationRelaxedDeg =
                    relaxedOrientationDeg;
            }

            // A governed Cartesian sole command is already acceleration/rate limited and
            // advances only while the physical foot remains inside its tracking tube.
            // Filtering hip and knee again in joint space destroys the two-bone closure:
            // the three commands no longer describe the sole pose the governor admitted.
            // On this route, write the coherent analytical pair. Other locomotion paths
            // retain their established response filter.
            const glm::quat previousHipCommand = glm::normalize(
                controlledLeg.hipCommand);
            const glm::quat previousKneeCommand = glm::normalize(
                controlledLeg.kneeCommand);
            const glm::quat previousAnkleCommand = glm::normalize(
                controlledLeg.ankleCommand);
            const glm::quat nextHipCommand = coherentWalkingSolve
                ? glm::normalize(hipTarget)
                : glm::normalize(glm::slerp(
                    controlledLeg.hipCommand, hipTarget, alpha));
            const glm::quat nextKneeCommand = coherentWalkingSolve
                ? glm::normalize(kneeTarget)
                : glm::normalize(glm::slerp(
                    controlledLeg.kneeCommand, kneeTarget, alpha));

            // The sole is terminal; preserving its old local pose lets pelvis/knee motion
            // rotate the entire foot away from the world-space foothold. Counter-rotate at
            // the powered ankle instead. The reference terminal relation remains fixed,
            // while the ankle target makes that relation produce the captured sole world
            // orientation after the commanded hip/knee solve.
            glm::quat commandedHipWorld = glm::normalize(
                parentWorld * nextHipCommand);
            glm::quat commandedKneeWorld = glm::normalize(
                commandedHipWorld * nextKneeCommand);
            bool physicalKneeWorldOk = false;
            const glm::quat measuredKneeWorld =
                Physics::GetRagdollBoneRotation(
                    rag, controlledLeg.kneeIdx, &physicalKneeWorldOk);
            // A commanded-parent target is smooth but inherits upstream tracking error;
            // an exact measured-parent target cancels that error but can move abruptly as
            // the powered knee oscillates. For the governed inside sole, retain only a
            // bounded amount of that measured compensation and rate-limit the final local
            // motor command below. Other gait paths keep their previous exact target.
            const glm::quat measuredAnkleParentWorld = continuousEnabled
                && physicalKneeWorldOk
                ? glm::normalize(measuredKneeWorld)
                : commandedKneeWorld;
            const glm::quat nominalAnkleTarget = glm::normalize(
                glm::conjugate(commandedKneeWorld)
                * desiredFootWorld
                * glm::conjugate(controlledLeg.referenceFootLocal));
            glm::quat measuredCompensatedAnkleTarget = glm::normalize(
                glm::conjugate(measuredAnkleParentWorld)
                * desiredFootWorld
                * glm::conjugate(controlledLeg.referenceFootLocal));
            if (glm::dot(nominalAnkleTarget,
                         measuredCompensatedAnkleTarget) < 0.0f) {
                measuredCompensatedAnkleTarget =
                    -measuredCompensatedAnkleTarget;
            }
            // The coherent route derives the ankle from the commanded parent so all three
            // joint commands encode one chain pose. Measured-parent cancellation broke
            // that closure: its large ankle delta throttled the shared hip/knee rate and
            // fed physical oscillation back into the next command.
            glm::quat ankleTarget = coherentWalkingSolve
                ? nominalAnkleTarget
                : measuredCompensatedAnkleTarget;
            if (coherentWalkingSolve) {
                comp._gaitAnkleParentCompensationDeg =
                    rotationDifferenceDeg(
                        nominalAnkleTarget,
                        measuredCompensatedAnkleTarget);
                comp._gaitAnkleParentCompensationAppliedDeg = 0.0f;
            } else if (governMovingSole) {
                // Six degrees is enough during quiet tracking, but the logs show 12-15
                // degrees of measured parent disagreement during the unstable lift. Admit
                // that extra cancellation only under real sole tilt; the final ankle
                // command remains rate limited below, avoiding the old unfiltered chase.
                const float parentCompensationPressure = smoothstep(glm::clamp(
                    (comp._gaitSoleAngularErrorDeg - 15.0f) / 25.0f,
                    0.0f, 1.0f));
                const float maximumParentCompensationDeg = glm::mix(
                    6.0f, 15.0f, parentCompensationPressure);
                comp._gaitAnkleParentCompensationDeg =
                    rotationDifferenceDeg(
                        nominalAnkleTarget,
                        measuredCompensatedAnkleTarget);
                comp._gaitAnkleParentCompensationAppliedDeg = glm::min(
                    comp._gaitAnkleParentCompensationDeg,
                    maximumParentCompensationDeg);
                const float compensationBlend =
                    comp._gaitAnkleParentCompensationDeg > 1e-4f
                        ? comp._gaitAnkleParentCompensationAppliedDeg
                            / comp._gaitAnkleParentCompensationDeg
                        : 0.0f;
                ankleTarget = glm::normalize(glm::slerp(
                    nominalAnkleTarget,
                    measuredCompensatedAnkleTarget,
                    compensationBlend));
            } else if (movingFoot) {
                comp._gaitAnkleParentCompensationDeg = 0.0f;
                comp._gaitAnkleParentCompensationAppliedDeg = 0.0f;
                comp._gaitAnkleCommandDeltaDeg = 0.0f;
                comp._gaitAnkleAchievableResidualDeg = 0.0f;
                comp._gaitAnkleCommandAngularSpeed = 0.0f;
                comp._gaitAnkleDiagnosticCooldown = 0.0f;
            }
            const glm::quat unconstrainedAnkleTarget = ankleTarget;
            ankleTarget = ClampToEnvelope(ankleEnvelope,
                skeleton.bones[controlledLeg.ankleIdx].localR, ankleTarget,
                comp.hipLimitMarginDeg);

            if (continuousEnabled && movingFoot) {
                comp._gaitIkHipEnvelopeClampDeg = rotationDifferenceDeg(
                    unconstrainedHipTarget, hipTarget);
                comp._gaitIkAnkleEnvelopeClampDeg = rotationDifferenceDeg(
                    unconstrainedAnkleTarget, ankleTarget);
                const EnvelopeMeasurement ankleMeasurement = MeasureEnvelope(
                    ankleEnvelope,
                    skeleton.bones[controlledLeg.ankleIdx].localR,
                    unconstrainedAnkleTarget,
                    comp.hipLimitMarginDeg);
                comp._gaitIkAnkleSwingUsage =
                    ankleMeasurement.swingUsage;
                comp._gaitIkAnkleSwingReserve =
                    ankleMeasurement.swingReserve;
                comp._gaitIkAnkleTwistDeg =
                    ankleMeasurement.twistDeg;
                comp._gaitIkAnkleTwistMarginDeg =
                    ankleMeasurement.twistMarginDeg;
            }

            if (coherentWalkingSolve) {
                // Admit one fraction of the complete joint target. Independent joint
                // filters let the ankle race ahead to cancel a parent that had not moved
                // yet; one shared fraction keeps the whole chain on a single temporal
                // manifold. The residual is intentional, reported by FK, and causes the
                // outer measured-state governor to wait rather than commanding a snap.
                comp._gaitCoherentHipDeltaDeg = rotationDifferenceDeg(
                    previousHipCommand, nextHipCommand);
                comp._gaitCoherentKneeDeltaDeg = rotationDifferenceDeg(
                    previousKneeCommand, nextKneeCommand);
                comp._gaitCoherentAnkleDeltaDeg = rotationDifferenceDeg(
                    previousAnkleCommand, ankleTarget);
                constexpr float kHipCommandRateDeg = 240.0f;
                constexpr float kKneeCommandRateDeg = 300.0f;
                constexpr float kAnkleCommandRateDeg = 240.0f;
                auto jointAdmission = [&](float requestedDeg,
                                          float rateDeg) {
                    return requestedDeg > 1e-5f
                        ? glm::clamp(rateDeg * dt / requestedDeg,
                            0.0f, 1.0f)
                        : 1.0f;
                };
                comp._gaitCoherentJointScale = glm::min(
                    jointAdmission(comp._gaitCoherentHipDeltaDeg,
                                   kHipCommandRateDeg),
                    glm::min(
                        jointAdmission(comp._gaitCoherentKneeDeltaDeg,
                                       kKneeCommandRateDeg),
                        jointAdmission(comp._gaitCoherentAnkleDeltaDeg,
                                       kAnkleCommandRateDeg)));
                controlledLeg.hipCommand = glm::normalize(glm::slerp(
                    previousHipCommand, nextHipCommand,
                    comp._gaitCoherentJointScale));
                controlledLeg.kneeCommand = glm::normalize(glm::slerp(
                    previousKneeCommand, nextKneeCommand,
                    comp._gaitCoherentJointScale));
                controlledLeg.ankleCommand = glm::normalize(glm::slerp(
                    previousAnkleCommand, ankleTarget,
                    comp._gaitCoherentJointScale));
                comp._gaitAnkleCommandDeltaDeg = rotationDifferenceDeg(
                    previousAnkleCommand, controlledLeg.ankleCommand);
                comp._gaitAnkleCommandAngularSpeed = dt > 1e-6f
                    ? glm::radians(comp._gaitAnkleCommandDeltaDeg) / dt
                    : 0.0f;

                commandedHipWorld = glm::normalize(
                    parentWorld * controlledLeg.hipCommand);
                commandedKneeWorld = glm::normalize(
                    commandedHipWorld * controlledLeg.kneeCommand);
                const glm::quat achievableFootWorld = glm::normalize(
                    commandedKneeWorld
                    * controlledLeg.ankleCommand
                    * controlledLeg.referenceFootLocal);
                comp._gaitAnkleAchievableResidualDeg =
                    rotationDifferenceDeg(
                        achievableFootWorld, desiredFootWorld);
                comp._gaitSwingSoleCommandWorld = achievableFootWorld;
                comp._gaitSwingSoleCommandValid = true;
                comp._gaitSwingSoleCommandErrorDeg =
                    physicalSwingFootRotationOk
                        ? rotationDifferenceDeg(
                            physicalSwingFootWorld,
                            achievableFootWorld)
                        : 0.0f;

                comp._gaitAnkleDiagnosticCooldown = glm::max(
                    comp._gaitAnkleDiagnosticCooldown - dt, 0.0f);
                const bool commandClosureLarge =
                    comp._gaitAnkleAchievableResidualDeg > 1.0f;
                const bool physicalParentDisagreement =
                    comp._gaitAnkleParentCompensationDeg > 6.0f;
                if ((commandClosureLarge || physicalParentDisagreement)
                    && comp._gaitAnkleDiagnosticCooldown <= 0.0f) {
                    comp._gaitAnkleDiagnosticCooldown = 0.10f;
                    spdlog::info(
                        "[LocomotionTurnAnkle] step={} phase={} swing={} "
                        "controller=coherent parentFeedback={:.1f}deg "
                        "appliedParentFeedback={:.1f}deg jointScale={:.3f} "
                        "jointDelta=({:.1f},{:.1f},{:.1f})deg localStep={:.2f}deg "
                        "localRate={:.3f}radps worldResidual={:.1f}deg "
                        "soleTracking={:.1f}deg envelopeClamp={:.1f}deg",
                        comp._stepSequenceStepIndex,
                        comp._physicalStepPhase,
                        comp._gaitTurnPlan.swingFootLeft
                            ? "LEFT" : "RIGHT",
                        comp._gaitAnkleParentCompensationDeg,
                        comp._gaitAnkleParentCompensationAppliedDeg,
                        comp._gaitCoherentJointScale,
                        comp._gaitCoherentHipDeltaDeg,
                        comp._gaitCoherentKneeDeltaDeg,
                        comp._gaitCoherentAnkleDeltaDeg,
                        comp._gaitAnkleCommandDeltaDeg,
                        comp._gaitAnkleCommandAngularSpeed,
                        comp._gaitAnkleAchievableResidualDeg,
                        comp._gaitSwingSoleCommandErrorDeg,
                        comp._gaitIkAnkleEnvelopeClampDeg);
                }
            } else if (governMovingSole) {
                controlledLeg.hipCommand = nextHipCommand;
                controlledLeg.kneeCommand = nextKneeCommand;
                glm::quat currentAnkleCommand = glm::normalize(
                    controlledLeg.ankleCommand);
                glm::quat boundedAnkleTarget = ankleTarget;
                if (glm::dot(currentAnkleCommand, boundedAnkleTarget) < 0.0f)
                    boundedAnkleTarget = -boundedAnkleTarget;

                constexpr float kLocalAnkleAcceleration = 20.0f;
                const float governedSoleSpeed = hasYawBearingTurnStep()
                    ? glm::max(comp._gaitTurnPlan.admittedAngularSpeed,
                               0.75f)
                    : glm::clamp(comp.gaitSoleAngularSpeedLimit,
                                 0.75f, 8.0f);
                const float localAnkleSpeedLimit = glm::clamp(
                    governedSoleSpeed + 1.0f,
                    1.75f, 4.25f);
                const float remainingLocalAngle = glm::radians(
                    rotationDifferenceDeg(
                        currentAnkleCommand, boundedAnkleTarget));
                const float brakingLocalSpeed = std::sqrt(glm::max(
                    2.0f * kLocalAnkleAcceleration
                        * remainingLocalAngle,
                    0.0f));
                const float desiredLocalSpeed = glm::min(
                    localAnkleSpeedLimit, brakingLocalSpeed);
                comp._gaitAnkleCommandAngularSpeed = moveScalarToward(
                    comp._gaitAnkleCommandAngularSpeed,
                    desiredLocalSpeed,
                    kLocalAnkleAcceleration * dt);
                const float localStepAngle = glm::min(
                    remainingLocalAngle,
                    comp._gaitAnkleCommandAngularSpeed * dt);
                const float localInterpolation = remainingLocalAngle > 1e-6f
                    ? localStepAngle / remainingLocalAngle : 1.0f;
                glm::quat nextAnkleCommand = glm::normalize(glm::slerp(
                    currentAnkleCommand,
                    boundedAnkleTarget,
                    localInterpolation));
                nextAnkleCommand = ClampToEnvelope(
                    ankleEnvelope,
                    skeleton.bones[controlledLeg.ankleIdx].localR,
                    nextAnkleCommand,
                    comp.hipLimitMarginDeg);
                comp._gaitAnkleCommandDeltaDeg = rotationDifferenceDeg(
                    currentAnkleCommand, nextAnkleCommand);
                controlledLeg.ankleCommand = nextAnkleCommand;

                // Publish what the bounded local motor can actually ask the physical
                // chain to produce. The outer world-space governor continues from this
                // reachable orientation next frame instead of integrating through an
                // ankle-envelope boundary that the motor can never cross.
                const glm::quat achievableFootWorld = glm::normalize(
                    measuredAnkleParentWorld
                    * controlledLeg.ankleCommand
                    * controlledLeg.referenceFootLocal);
                comp._gaitAnkleAchievableResidualDeg =
                    rotationDifferenceDeg(
                        achievableFootWorld, desiredFootWorld);
                comp._gaitSwingSoleCommandWorld = achievableFootWorld;
                comp._gaitSwingSoleCommandValid = true;
                comp._gaitSwingSoleCommandErrorDeg =
                    physicalSwingFootRotationOk
                        ? rotationDifferenceDeg(
                            physicalSwingFootWorld,
                            achievableFootWorld)
                        : 0.0f;

                comp._gaitAnkleDiagnosticCooldown = glm::max(
                    comp._gaitAnkleDiagnosticCooldown - dt, 0.0f);
                const bool compensationLimited =
                    comp._gaitAnkleParentCompensationDeg
                        > comp._gaitAnkleParentCompensationAppliedDeg + 0.1f;
                const bool envelopeLimited =
                    comp._gaitIkAnkleEnvelopeClampDeg > 1.0f;
                const bool reachableResidualLarge =
                    comp._gaitAnkleAchievableResidualDeg > 8.0f;
                if ((compensationLimited || envelopeLimited
                     || reachableResidualLarge)
                    && comp._gaitAnkleDiagnosticCooldown <= 0.0f) {
                    comp._gaitAnkleDiagnosticCooldown = 0.10f;
                    spdlog::info(
                        "[LocomotionTurnAnkle] step={} phase={} swing={} "
                        "parentComp={:.1f}->{:.1f}deg envelopeClamp={:.1f}deg "
                        "localStep={:.2f}deg localRate={:.3f}radps "
                        "worldResidual={:.1f}deg soleTracking={:.1f}deg "
                        "counterfactual=(bestClamp={:.1f}deg,bestSwivel={:+.1f}deg,"
                        "closure={:.3f}m,accepted={},applied={:+.1f}deg,"
                        "priority={:.2f},relaxed={:.1f}deg) "
                        "envelope=(swingUsage={:.2f},reserve={:+.2f},"
                        "twist={:+.1f}deg,margin={:+.1f}deg)",
                        comp._stepSequenceStepIndex,
                        comp._physicalStepPhase,
                        comp._gaitTurnPlan.swingFootLeft
                            ? "LEFT" : "RIGHT",
                        comp._gaitAnkleParentCompensationDeg,
                        comp._gaitAnkleParentCompensationAppliedDeg,
                        comp._gaitIkAnkleEnvelopeClampDeg,
                        comp._gaitAnkleCommandDeltaDeg,
                        comp._gaitAnkleCommandAngularSpeed,
                        comp._gaitAnkleAchievableResidualDeg,
                        comp._gaitSwingSoleCommandErrorDeg,
                        comp._gaitIkExactAnkleClampDeg,
                        comp._gaitIkKneeSwivelRequestedDeg,
                        comp._gaitIkKneeSwivelCandidateClosure,
                        comp._gaitIkKneeSwivelCandidateAccepted ? "yes" : "no",
                        comp._gaitIkKneeSwivelAppliedDeg,
                        comp._gaitIkOrientationPriority,
                        comp._gaitIkOrientationRelaxedDeg,
                        comp._gaitIkAnkleSwingUsage,
                        comp._gaitIkAnkleSwingReserve,
                        comp._gaitIkAnkleTwistDeg,
                        comp._gaitIkAnkleTwistMarginDeg);
                }
            } else {
                controlledLeg.hipCommand = nextHipCommand;
                controlledLeg.kneeCommand = nextKneeCommand;
                controlledLeg.ankleCommand = glm::normalize(glm::slerp(
                    controlledLeg.ankleCommand, ankleTarget, alpha));
            }
            if (continuousEnabled && movingFoot) {
                comp._gaitIkHipCommandLagDeg = rotationDifferenceDeg(
                    controlledLeg.hipCommand, hipTarget);
                comp._gaitIkKneeCommandLagDeg = rotationDifferenceDeg(
                    controlledLeg.kneeCommand, kneeTarget);
            }
            if (continuousEnabled && movingFoot
                && &controlledLeg == swing) {
                // Forward-kinematics closure diagnostic. Reconstruct the sole pose from
                // the exact joint commands written below, then split total physical error
                // into desired->commanded (IK/composition) and commanded->physical
                // (powered-ragdoll tracking). This is deliberately read-only.
                const glm::vec3 commandedKneePosition = hipPosition
                    + commandedHipWorld
                        * upperSegmentLocal;
                const glm::vec3 commandedAnklePosition =
                    commandedKneePosition
                    + commandedKneeWorld
                        * lowerSegmentLocal;
                const glm::quat commandedFootWorld = glm::normalize(
                    commandedKneeWorld
                    * controlledLeg.ankleCommand
                    * controlledLeg.referenceFootLocal);
                const glm::vec3 commandedFootPosition =
                    commandedAnklePosition
                    - commandedFootWorld
                        * controlledLeg.ankleFromFootLocal;
                bool physicalFootWorldOk = false;
                const glm::quat measuredFootWorld =
                    Physics::GetRagdollBoneRotation(
                        rag, controlledLeg.footIdx,
                        &physicalFootWorldOk);

                comp._gaitFkDesiredPosition = controlledFoot;
                comp._gaitFkCommandPosition = commandedFootPosition;
                comp._gaitFkPhysicalPosition = measuredFoot;
                comp._gaitFkDesiredToCommand =
                    commandedFootPosition - controlledFoot;
                comp._gaitFkCommandToPhysical =
                    measuredFoot - commandedFootPosition;
                comp._gaitFkDesiredToPhysical =
                    measuredFoot - controlledFoot;
                comp._gaitFkDesiredToCommandOrientationDeg =
                    rotationDifferenceDeg(
                        desiredFootWorld, commandedFootWorld);
                comp._gaitFkCommandToPhysicalOrientationDeg =
                    physicalFootWorldOk
                        ? rotationDifferenceDeg(
                            commandedFootWorld, measuredFootWorld)
                        : 0.0f;
                comp._gaitFkDesiredToPhysicalOrientationDeg =
                    physicalFootWorldOk
                        ? rotationDifferenceDeg(
                            desiredFootWorld, measuredFootWorld)
                        : 0.0f;
                comp._gaitFkValid = physicalFootWorldOk;
                comp._gaitFkDiagnosticCooldown = glm::max(
                    comp._gaitFkDiagnosticCooldown - dt, 0.0f);
                if (governedInsideTurnSwing
                    && comp._gaitFkDiagnosticCooldown <= 0.0f) {
                    comp._gaitFkDiagnosticCooldown = 0.10f;
                    spdlog::info(
                        "[LocomotionTurnFK] step={} phase={} swing={} controller={} "
                        "position=(desired=({:+.3f},{:+.3f},{:+.3f}),"
                        "command=({:+.3f},{:+.3f},{:+.3f}),"
                        "physical=({:+.3f},{:+.3f},{:+.3f}))m "
                        "closure=(ik=({:+.3f},{:+.3f},{:+.3f})|{:.3f}m,"
                        "motor=({:+.3f},{:+.3f},{:+.3f})|{:.3f}m,"
                        "total=({:+.3f},{:+.3f},{:+.3f})|{:.3f}m) "
                        "orientation=(ik={:.1f}deg,motor={:.1f}deg,total={:.1f}deg) "
                        "jointAdmission=(scale={:.3f},requested=({:.1f},{:.1f},{:.1f})deg) "
                        "geometry=({},{:.3f},{:.3f})m "
                        "counterfactual=(planClamp={:.1f}deg,"
                        "bestSwivel={:+.1f}deg,bestClamp={:.1f}deg,"
                        "closure={:.3f}m,accepted={}) "
                        "appliedSwivel={:+.1f}deg runtimeSwivel=disabled",
                        comp._stepSequenceStepIndex,
                        comp._physicalStepPhase,
                        comp._gaitTurnPlan.swingFootLeft
                            ? "LEFT" : "RIGHT",
                        coherentWalkingSolve ? "coherent" : "legacy",
                        comp._gaitFkDesiredPosition.x,
                        comp._gaitFkDesiredPosition.y,
                        comp._gaitFkDesiredPosition.z,
                        comp._gaitFkCommandPosition.x,
                        comp._gaitFkCommandPosition.y,
                        comp._gaitFkCommandPosition.z,
                        comp._gaitFkPhysicalPosition.x,
                        comp._gaitFkPhysicalPosition.y,
                        comp._gaitFkPhysicalPosition.z,
                        comp._gaitFkDesiredToCommand.x,
                        comp._gaitFkDesiredToCommand.y,
                        comp._gaitFkDesiredToCommand.z,
                        glm::length(comp._gaitFkDesiredToCommand),
                        comp._gaitFkCommandToPhysical.x,
                        comp._gaitFkCommandToPhysical.y,
                        comp._gaitFkCommandToPhysical.z,
                        glm::length(comp._gaitFkCommandToPhysical),
                        comp._gaitFkDesiredToPhysical.x,
                        comp._gaitFkDesiredToPhysical.y,
                        comp._gaitFkDesiredToPhysical.z,
                        glm::length(comp._gaitFkDesiredToPhysical),
                        comp._gaitFkDesiredToCommandOrientationDeg,
                        comp._gaitFkCommandToPhysicalOrientationDeg,
                        comp._gaitFkDesiredToPhysicalOrientationDeg,
                        comp._gaitCoherentJointScale,
                        comp._gaitCoherentHipDeltaDeg,
                        comp._gaitCoherentKneeDeltaDeg,
                        comp._gaitCoherentAnkleDeltaDeg,
                        controlledLeg.segmentGeometryValid
                            ? "ragdoll" : "skeleton-fallback",
                        upperLength,
                        lowerLength,
                        comp._gaitTurnPlan.sampledAnkleClampDeg,
                        comp._gaitIkKneeSwivelRequestedDeg,
                        comp._gaitIkExactAnkleClampDeg,
                        comp._gaitIkKneeSwivelCandidateClosure,
                        comp._gaitIkKneeSwivelCandidateAccepted ? "yes" : "no",
                        comp._gaitIkKneeSwivelAppliedDeg);
                }
            }
            controlledLeg.footCommand = controlledLeg.referenceFootLocal;
        };

        const bool applyMovingSwingIK = comp._physicalStepPhase >= kTakeoff
            && comp._physicalStepPhase <= kTouchdownWait;
        const bool applyPlantedSwingIK = continuousEnabled
            && comp._physicalStepTouchdownAccepted && swing->planted
            && comp._physicalStepPhase >= kSettle
            && comp._physicalStepPhase <= kInterStep;
        if (applyMovingSwingIK) {
            solveLegIK(*swing, swingFoot, desiredFoot, true);
        } else if (applyPlantedSwingIK) {
            // Contact ends the sole trajectory, not the body's motion. Re-solve the leg
            // every frame against the captured world anchor while the pelvis passes over
            // it; retaining touchdown joint angles here was dragging the new support foot.
            solveLegIK(*swing, swingFoot, swing->plantFoot, false);
        }
        const bool applyPlantedStanceIK = continuousEnabled
            && stance->planted && stance->plantSolveValid
            && comp._physicalStepPhase >= kWeightShift
            && comp._physicalStepPhase <= kInterStep;
        if (applyPlantedStanceIK)
            solveLegIK(*stance, stanceFoot, stance->plantFoot, false);
        // Preserve the nominal target as history only after IK has consumed the previous
        // frame. Writing it before solve made targetVelocity identically zero and silently
        // disabled the configured 80 ms feed-forward term.
        comp._physicalStepDesiredFoot = desiredFoot;
        const bool holdTouchdownPose = comp._physicalStepPhase >= kSettle
            && (comp._physicalStepPhase <= kComplete
                || (continuousEnabled && comp._physicalStepPhase == kStopping));
        const bool holdMultiStepBaseline = multiStepEnabled
            && comp._stepSequenceStepIndex >= 2 && comp._physicalStepPhase == kWeightShift;
        const float poseWeight = glm::clamp(
            comp._poseBlend * comp.poseWeight, 0.0f, 1.0f);
        const float shutdownPoseBlend = continuousEnabled
            && comp._physicalStepPhase == kStopping
            ? 1.0f - smoothstep(comp._physicalStepPhaseTime
                / glm::max(comp.gaitStopTime, 0.25f))
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
        if (applyMovingSwingIK || applyPlantedSwingIK
            || holdTouchdownPose || holdMultiStepBaseline)
            writeLegPose(*swing);
        const bool writeStancePose = applyPlantedStanceIK || (multiStepEnabled
            && comp._stepSequenceStepIndex >= 2
            && comp._physicalStepPhase >= kWeightShift
            && (comp._physicalStepPhase <= kComplete
                || (continuousEnabled && comp._physicalStepPhase == kStopping)));
        if (writeStancePose)
            writeLegPose(*stance);

        auto beginStopping = [&]() {
            auto& turnPlan = comp._gaitTurnPlan;
            const int handoffPhase = comp._physicalStepPhase;
            const bool hadActiveHeading = turnPlan.activeHeadingPlan;
            const float admittedYaw = turnPlan.admittedYaw;
            const float admittedProgress = turnPlan.plannedTurnProgress;
            const float cancellationProgress =
                comp._gaitTurnCancellationProgress;
            const glm::vec3 desiredForward = turnPlan.desiredForward;
            const glm::vec3 handoffForward = horizontalForward(
                comp._gaitHeadingTargetRot);
            const float issuedYaw = signedHeadingDelta(
                turnPlan.activeStartForward, handoffForward);
            const float remainingToDesired = resolvedHeadingDelta(
                handoffForward, desiredForward);

            // A stop establishes a new standing heading without moving any body directly.
            // Early return has already unwound toward the step-entry target; a late finish
            // has completed the immutable admitted target through transfer. Publish the
            // target actually owned by the heading motor so the next run cannot restart
            // from a stale pre-cancellation frame.
            commitRuntimeStandingHeading(handoffForward);
            comp._gaitTurnCancellationUnwindActive = false;
            comp._gaitTurnCancellationProgress = 0.0f;
            clearPhysicalReversal(
                comp._runtimeRestartBlockForwardValid
                    ? "planner-rejection-stop" : "controlled-stop");

            if (comp.debug) {
                const char* mode = comp._gaitCancelMode == 1
                    ? "early-return"
                    : (comp._gaitCancelMode == 2
                        ? "late-finish"
                        : (handoffPhase == kWeightShift
                            || handoffPhase == kInterStep
                            ? "double-support" : "post-transfer"));
                spdlog::info(
                    "[LocomotionTurnHandoff] event=STOP_REBASE mode={} "
                    "phase={} step={} activeCommitted={} "
                    "admittedYaw={:+.3f}deg issuedYaw={:+.3f}deg "
                    "progress=(active={:.3f},cancel={:.3f}) "
                    "heading=({:+.3f},{:+.3f}) remainingDesired={:+.3f}deg "
                    "action=publish-standing-heading",
                    mode, gaitPhaseName(handoffPhase),
                    comp._stepSequenceStepIndex,
                    hadActiveHeading ? "yes" : "no",
                    glm::degrees(admittedYaw),
                    glm::degrees(issuedYaw),
                    admittedProgress, cancellationProgress,
                    handoffForward.x, handoffForward.z,
                    glm::degrees(remainingToDesired));
            }
            capturePhysicalLocalPose(*swing);
            capturePhysicalLocalPose(*stance);
            comp._gaitSwingCommandSpeed = 0.0f;
            comp._gaitSwingCommandAngularSpeed = 0.0f;
            comp._gaitSwingCommandTrackingError = 0.0f;
            comp._gaitSwingSoleCommandErrorDeg = 0.0f;
            comp._gaitAnkleCommandAngularSpeed = 0.0f;
            comp._gaitAnkleCommandDeltaDeg = 0.0f;
            comp._gaitAnkleParentCompensationDeg = 0.0f;
            comp._gaitAnkleParentCompensationAppliedDeg = 0.0f;
            comp._gaitAnkleAchievableResidualDeg = 0.0f;
            comp._gaitAnkleDiagnosticCooldown = 0.0f;
            comp._gaitIkAnkleEnvelopeClampDeg = 0.0f;
            comp._gaitIkExactAnkleClampDeg = 0.0f;
            comp._gaitIkKneeSwivelRequestedDeg = 0.0f;
            comp._gaitIkKneeSwivelAppliedDeg = 0.0f;
            comp._gaitIkOrientationPriority = 1.0f;
            comp._gaitIkOrientationRelaxedDeg = 0.0f;
            comp._gaitIkAnkleSwingUsage = 0.0f;
            comp._gaitIkAnkleSwingReserve = 0.0f;
            comp._gaitIkAnkleTwistDeg = 0.0f;
            comp._gaitIkAnkleTwistMarginDeg = 0.0f;
            comp._gaitFkDesiredPosition = glm::vec3(0.0f);
            comp._gaitFkCommandPosition = glm::vec3(0.0f);
            comp._gaitFkPhysicalPosition = glm::vec3(0.0f);
            comp._gaitFkDesiredToCommand = glm::vec3(0.0f);
            comp._gaitFkCommandToPhysical = glm::vec3(0.0f);
            comp._gaitFkDesiredToPhysical = glm::vec3(0.0f);
            comp._gaitFkDesiredToCommandOrientationDeg = 0.0f;
            comp._gaitFkCommandToPhysicalOrientationDeg = 0.0f;
            comp._gaitFkDesiredToPhysicalOrientationDeg = 0.0f;
            comp._gaitFkDiagnosticCooldown = 0.0f;
            comp._gaitFkValid = false;
            comp._gaitSwingPathLength = 0.0f;
            comp._gaitSwingMinimumDuration = 0.0f;
            comp._gaitSwingDeadline = 0.0f;
            comp._gaitSwingWatchdogProgress = 0.0f;
            comp._gaitSwingNoProgressTime = 0.0f;
            comp._gaitSwingSoleCommandValid = false;
            comp._gaitTurnPairPendingInside = false;
            comp._gaitTurnPairAdvanceScale = 1.0f;
            comp._gaitTurnPairYawScale = 1.0f;
            comp._gaitTurnPairYawSign = 0.0f;
            comp._gaitTurnExitBlendPending = false;
            comp._gaitStopStartTarget = comp._physicalStepSupportTarget;
            comp._gaitStopEndTarget = comp._gaitStopStartTarget;
            comp._gaitStopFootTargetL = leftFoot;
            comp._gaitStopFootTargetR = rightFoot;
            comp._gaitStopFootDriftL = 0.0f;
            comp._gaitStopFootDriftR = 0.0f;
            comp._gaitStopMaxFootDrift = 0.0f;
            comp._gaitStopSettleFootDriftL = 0.0f;
            comp._gaitStopSettleFootDriftR = 0.0f;
            comp._gaitStopMaxSettleFootDrift = 0.0f;
            comp._gaitStopSettleReferenceValid = false;
            comp._gaitStopStableTime = 0.0f;
            comp._gaitContinuousCycle = false;
            comp._gaitBypassWeightShift = false;
            comp._gaitSupportCurveActive = false;
            comp._gaitSupportCurveStep = -1;
            comp._gaitSupportCommandVelocity = glm::vec3(0.0f);
            comp._physicalStepPhase = kStopping;
            comp._physicalStepPhaseTime = 0.0f;
        };

        if (continuousEnabled && comp._gaitStopRequested
            && comp._gaitCancelMode == 0
            && (comp._physicalStepPhase == kWeightShift
                || comp._physicalStepPhase == kInterStep)) {
            // Both soles are still owned in these phases, so no swing cancellation is
            // necessary. Enter the validated stop directly instead of admitting a throwaway
            // foothold or starting another role swap.
            beginStopping();
        }

        auto beginTransfer = [&]() {
            const float transferFraction = glm::clamp(
                comp.transferSupportBias + comp._gaitAdaptiveTransferBiasOffset,
                kNewSupportLoadCommandTarget, 0.98f);
            glm::vec3 comStart = rag._locomotionCOM;
            glm::vec3 newPlant = swingFoot;
            comStart.y = comp._physicalStepSupportTarget.y;
            newPlant.y = comp._physicalStepSupportTarget.y;
            comp._supportTransferTransferStartTarget =
                comp._physicalStepSupportTarget;
            if (continuousEnabled) {
                // Transfer along the complete old-to-new support span. The former path
                // moved laterally toward the new foot but stopped at the forward midpoint,
                // so the old leg often remained loaded even after TRANSFER completed.
                comp._supportTransferTransferEndTarget = glm::mix(
                    stanceFoot, newPlant, transferFraction);
                comp._supportTransferTransferEndTarget.y =
                    comp._physicalStepSupportTarget.y;
            } else {
                comp._supportTransferTransferEndTarget = glm::mix(
                    comStart, newPlant, transferFraction);
            }
            if (continuousEnabled && comp._gaitSupportCurveActive) {
                const bool continuingParallelHandoff =
                    walkingOverlapHandoff
                    && comp._physicalStepTouchdownAccepted
                    && comp._gaitSupportCurveDuration > 0.0f;
                if (!continuingParallelHandoff) {
                // Acquisition deliberately used only a partial-load target. Continue from
                // the exact command position and velocity toward the full support handoff;
                // this restores load progression without a settle-to-transfer stop.
                glm::vec3 incomingVelocity =
                    comp._gaitSupportCommandVelocity;
                incomingVelocity.y = 0.0f;
                const float incomingSpeed = glm::length(incomingVelocity);
                if (incomingSpeed > configuredSupportMaxSpeed
                    && incomingSpeed > 1e-6f) {
                    incomingVelocity *= configuredSupportMaxSpeed / incomingSpeed;
                }
                comp._gaitSupportCurveStart =
                    comp._physicalStepSupportTarget;
                comp._gaitSupportCurveStartVelocity = incomingVelocity;
                comp._gaitSupportCurveEnd =
                    comp._supportTransferTransferEndTarget;
                comp._gaitSupportCurveEnd.y =
                    comp._gaitSupportCurveStart.y;
                comp._gaitSupportCurveTime = 0.0f;
                comp._gaitSupportCurveDuration = glm::max(
                    cadenceTransferTime * 1.50f, 0.30f);
                comp._gaitSupportCurveEndVelocity = supportCurveEndVelocity(
                    comp._gaitSupportCurveStart,
                    comp._gaitSupportCurveEnd,
                    comp._gaitSupportCurveDuration);
                const float outgoingSpeed = glm::length(
                    comp._gaitSupportCurveEndVelocity);
                spdlog::info(
                    "[LocomotionGait] SUPPORT_CURVE_ACQUIRED step={} "
                    "plant={} duration={:.3f}s speed={:.3f}->{:.3f} "
                    "load={:.2f} anchorDrift={:.3f} centerTravel={:.3f} "
                    "correctionPeak={:.3f}/{:.3f}m saturated={} rebased={} "
                    "centerAnchor={} pivotRelease={}/{:.2f} "
                    "maxQuiet={:.3f}/{:.3f}s "
                    "blocked=(contact={:.3f},sole={:.3f},angular={:.3f},"
                    "linear={:.3f})s migration={:.3f}m angularNow={:.3f}radps",
                    comp._stepSequenceStepIndex,
                    comp._physicalStepSupportSide < 0 ? "RIGHT" : "LEFT",
                    comp._gaitSupportCurveDuration,
                    glm::length(incomingVelocity), outgoingSpeed,
                    comp._gaitNewSupportLoad,
                    comp._physicalStepPlantDrift,
                    comp._physicalStepPlantCenterTravel,
                    comp._gaitPlantCorrectionPeakRequested,
                    comp._gaitPlantCorrectionPeakApplied,
                    comp._gaitPlantCorrectionSaturated ? "yes" : "no",
                    comp._physicalStepPlantAnchorRebased ? "yes" : "no",
                    comp._physicalStepPlantCenterAnchorActive ? "yes" : "no",
                    comp._physicalStepPlantPivotReleaseLatched
                        ? "yes" : "no",
                    comp._physicalStepPlantPivotReleaseWeight,
                    comp._physicalStepPlantPivotMaxStableTime,
                    kPlantPivotQuietTime,
                    comp._physicalStepPlantPivotContactBlockedTime,
                    comp._physicalStepPlantPivotSoleBlockedTime,
                    comp._physicalStepPlantPivotAngularBlockedTime,
                    comp._physicalStepPlantPivotLinearBlockedTime,
                    comp._physicalStepPlantContactMigration,
                    comp._physicalStepPlantAngularSpeed);
                } else {
                    spdlog::info(
                        "[LocomotionGait] SUPPORT_CURVE_ACQUIRED step={} "
                        "plant={} mode=parallel-continue time={:.3f}/{:.3f}s "
                        "load={:.2f} speed={:.3f}mps",
                        comp._stepSequenceStepIndex,
                        comp._physicalStepSupportSide < 0 ? "RIGHT" : "LEFT",
                        comp._gaitSupportCurveTime,
                        comp._gaitSupportCurveDuration,
                        comp._gaitNewSupportLoad,
                        glm::length(comp._gaitSupportCommandVelocity));
                }
            }
            comp._supportTransferTransferT = 0.0f;
            comp._supportTransferHoldStableTime = 0.0f;
            comp._supportTransferContactLossTime = 0.0f;
            const bool parallelHandoffReady = walkingOverlapHandoff
                && comp._gaitNewSupportLoadLatched;
            comp._physicalStepPhase = parallelHandoffReady
                ? kHold : kTransfer;
            comp._physicalStepPhaseTime = 0.0f;
            if (parallelHandoffReady) {
                spdlog::info(
                    "[LocomotionGait] PARALLEL_HANDOFF_READY step={} "
                    "load={:.3f} action=skip-serialized-transfer",
                    comp._stepSequenceStepIndex,
                    comp._gaitNewSupportLoad);
            }
        };

        auto updateGaitAdaptation = [&](bool updateTranslationObjective) {
            const float response = glm::clamp(
                comp.gaitAdaptationResponse, 0.01f, 1.0f);
            if (!comp.gaitAdaptationEnabled) {
                comp._gaitAdaptiveStrideOffset = glm::mix(
                    comp._gaitAdaptiveStrideOffset, 0.0f, response);
                comp._gaitAdaptiveLateralOffset = glm::mix(
                    comp._gaitAdaptiveLateralOffset, 0.0f, response);
                comp._gaitAdaptivePeriodOffset = glm::mix(
                    comp._gaitAdaptivePeriodOffset, 0.0f, response);
                comp._gaitAdaptiveTransferBiasOffset = glm::mix(
                    comp._gaitAdaptiveTransferBiasOffset, 0.0f, response);
                comp._gaitStress *= 0.60f;
                comp._gaitRecoveryFailureSteps = 0;
                return;
            }

            auto filter = [&](float& state, float sample) {
                state = glm::mix(state, sample, response);
            };
            const float previousFilteredDrift = comp._gaitFilteredDrift;
            const float previousFilteredUnload =
                comp._gaitFilteredUnloadDeficit;
            const float stepDrift = comp._gaitStepMaxRelevantDrift;
            const float geometricUnloadDeficit = glm::max(
                comp._supportTransferComToNewSupport + 0.020f
                    - comp._supportTransferComToOldSupport,
                0.0f);
            const float releaseQualityDeficit = gaitOldSupportUnloaded
                ? 0.0f
                : glm::max(comp._physicalStepStanceDrift - 0.040f, 0.0f);
            const float unloadDeficit = glm::max(
                geometricUnloadDeficit, releaseQualityDeficit);
            if (updateTranslationObjective) {
                filter(comp._gaitFilteredForwardError,
                       comp._physicalStepForwardTargetError);
            }
            filter(comp._gaitFilteredLateralError,
                   comp._physicalStepLateralTargetError);
            filter(comp._gaitFilteredTouchdownSpeed,
                   std::abs(comp._physicalStepTouchdownVy));
            filter(comp._gaitFilteredDrift, stepDrift);
            filter(comp._gaitFilteredMotorRatio,
                   comp._physicalStepMaxMotorRatio);
            filter(comp._gaitFilteredUnloadDeficit, unloadDeficit);

            const float maxStrideCorrection = glm::max(
                comp.gaitMaxStrideCorrection, 0.0f);
            const bool authorityLimited = comp._gaitReachClampedStep
                || comp._gaitFilteredMotorRatio > 0.80f;
            const float landingFeedForward = authorityLimited
                ? 0.0f
                : glm::clamp(comp._gaitFilteredForwardError * 0.50f,
                             -maxStrideCorrection, maxStrideCorrection);
            const float driftPenalty = glm::clamp(
                glm::max(comp._gaitFilteredDrift - 0.015f, 0.0f) * 1.5f,
                0.0f, maxStrideCorrection);
            const float authorityPenalty = authorityLimited
                ? glm::min(maxStrideCorrection, 0.015f) : 0.0f;
            if (updateTranslationObjective) {
                const float targetStrideOffset = glm::clamp(
                    landingFeedForward - driftPenalty - authorityPenalty,
                    -maxStrideCorrection, maxStrideCorrection);
                comp._gaitAdaptiveStrideOffset = glm::mix(
                    comp._gaitAdaptiveStrideOffset,
                    targetStrideOffset, response);
            }

            const float targetLateralOffset = glm::clamp(
                comp._gaitFilteredLateralError * 0.35f,
                -0.015f, 0.015f);
            comp._gaitAdaptiveLateralOffset = glm::mix(
                comp._gaitAdaptiveLateralOffset,
                targetLateralOffset, response);

            const float touchdownLimit = glm::max(
                comp.touchdownMaxVerticalSpeed, 0.30f);
            const float impactSlowdown = glm::max(
                comp._gaitFilteredTouchdownSpeed - touchdownLimit, 0.0f) * 0.80f;
            const float driftSlowdown = glm::max(
                comp._gaitFilteredDrift - 0.015f, 0.0f) * 5.0f;
            const float motorSlowdown = glm::max(
                comp._gaitFilteredMotorRatio - 0.75f, 0.0f) * 0.50f;
            const float reachSlowdown = comp._gaitReachClampedStep ? 0.10f : 0.0f;
            const float targetPeriodOffset = glm::clamp(
                impactSlowdown + driftSlowdown + motorSlowdown + reachSlowdown,
                0.0f, glm::max(comp.gaitMaxPeriodSlowdown, 0.0f));
            comp._gaitAdaptivePeriodOffset = glm::mix(
                comp._gaitAdaptivePeriodOffset,
                targetPeriodOffset, response);

            const float targetTransferBiasOffset = glm::clamp(
                comp._gaitFilteredUnloadDeficit * 1.5f, 0.0f, 0.06f);
            comp._gaitAdaptiveTransferBiasOffset = glm::mix(
                comp._gaitAdaptiveTransferBiasOffset,
                targetTransferBiasOffset, response);

            const float impactStress = glm::max(
                comp._gaitFilteredTouchdownSpeed - touchdownLimit, 0.0f)
                / glm::max(touchdownLimit, 0.10f) * 0.50f;
            const float driftStress = glm::max(
                comp._gaitFilteredDrift - 0.020f, 0.0f) / 0.020f * 0.75f;
            const float motorStress = glm::max(
                comp._gaitFilteredMotorRatio - 0.80f, 0.0f) / 0.20f * 0.75f;
            const float reachStress = comp._gaitReachClampedStep ? 0.35f : 0.0f;
            const float unloadStress = glm::clamp(
                comp._gaitFilteredUnloadDeficit / 0.020f, 0.0f, 1.0f) * 0.25f;
            const float stepStress = impactStress + driftStress + motorStress
                + reachStress + unloadStress;
            comp._gaitStress = comp._gaitStress * 0.60f + stepStress;
            const float stressThreshold = glm::max(
                comp.gaitStressStopThreshold, 0.5f);
            const bool driftWorsening = comp._gaitFilteredDrift
                > previousFilteredDrift + 0.001f;
            const bool unloadWorsening = comp._gaitFilteredUnloadDeficit
                > previousFilteredUnload + 0.001f;
            auto correctionNearLimit = [](float value, float limit) {
                return limit <= 1e-4f || std::abs(value) >= limit * 0.80f;
            };
            const float periodCorrectionLimit = glm::max(
                comp.gaitMaxPeriodSlowdown, 0.0f);
            const float transferCorrectionLimit = glm::min(
                0.060f,
                glm::max(0.98f - glm::clamp(
                    comp.transferSupportBias,
                    kNewSupportLoadCommandTarget, 0.98f), 0.0f));
            const bool strideDriftCorrectionSaturated =
                maxStrideCorrection <= 1e-4f
                || comp._gaitAdaptiveStrideOffset
                    <= -maxStrideCorrection * 0.80f;
            const bool driftCorrectionSaturated =
                strideDriftCorrectionSaturated
                && correctionNearLimit(
                    comp._gaitAdaptivePeriodOffset, periodCorrectionLimit);
            const bool unloadCorrectionSaturated = correctionNearLimit(
                comp._gaitAdaptiveTransferBiasOffset,
                transferCorrectionLimit);
            const bool unresolvedDrift = driftStress > 0.0f
                && driftWorsening && driftCorrectionSaturated;
            const bool unresolvedUnload = unloadStress > 0.0f
                && unloadWorsening && unloadCorrectionSaturated;
            const bool unresolvedRecoverableStress =
                comp._gaitStress >= stressThreshold
                && (unresolvedDrift || unresolvedUnload);
            comp._gaitRecoveryFailureSteps = unresolvedRecoverableStress
                ? comp._gaitRecoveryFailureSteps + 1
                : glm::max(comp._gaitRecoveryFailureSteps - 1, 0);

            // Impact and authority failures may request recovery directly. Drift and
            // unload errors must first keep worsening for several completed steps after
            // their bounded corrections are substantially exhausted.
            const float urgentStress = impactStress + motorStress + reachStress;
            const bool urgentRecovery = urgentStress >= stressThreshold;
            const bool exhaustedRecovery =
                comp._gaitRecoveryFailureSteps >= 3;
            if (!comp._gaitStopRequested
                && (urgentRecovery || exhaustedRecovery)) {
                spdlog::warn(
                    "[LocomotionGait] ADAPTIVE_STOP_REQUEST step={} "
                    "reason={} failSteps={} "
                    "stress={:.2f}/{:.2f} components=(impact={:.2f},drift={:.2f},"
                    "motor={:.2f},reach={:.2f},unload={:.2f}) "
                    "filtered=(impact={:.3f}mps,drift={:.3f}m,motor={:.2f},"
                    "unload={:.3f}m) correction=(stride={:+.3f}m,"
                    "period=+{:.2f}s,transfer={:+.3f}) saturation=(drift={},unload={})",
                    comp._stepSequenceStepIndex,
                    urgentRecovery ? "urgent" : "recovery_exhausted",
                    comp._gaitRecoveryFailureSteps,
                    comp._gaitStress, stressThreshold,
                    impactStress, driftStress, motorStress,
                    reachStress, unloadStress,
                    comp._gaitFilteredTouchdownSpeed,
                    comp._gaitFilteredDrift,
                    comp._gaitFilteredMotorRatio,
                    comp._gaitFilteredUnloadDeficit,
                    comp._gaitAdaptiveStrideOffset,
                    comp._gaitAdaptivePeriodOffset,
                    comp._gaitAdaptiveTransferBiasOffset,
                    driftCorrectionSaturated ? "yes" : "no",
                    unloadCorrectionSaturated ? "yes" : "no");
                comp._gaitAdaptiveStopRequested = true;
                comp._gaitStopRequested = true;
            }
        };

        const bool locksOff = rag.locomotionFootLockWeights[0] <= 0.001f
                           && rag.locomotionFootLockWeights[1] <= 0.001f
                           && rag._locomotionFootLockForce[0] <= 0.5f
                           && rag._locomotionFootLockForce[1] <= 0.5f;
        const bool loadedSoleReady = !continuousEnabled
            || comp._gaitSoleAngularErrorDeg
                <= activePlantSoleToleranceDeg;
        const bool landingMotionReady = continuousEnabled
            ? glm::length(swingVelocity) < 0.18f
                && horizontalSpeed < 0.35f
            : glm::length(leftVelocity) < 0.15f
                && glm::length(rightVelocity) < 0.15f
                && horizontalSpeed < 0.15f;
        const bool landingStable = comp._physicalStepPlantPoseCaptured
            && comp._physicalStepContactL && comp._physicalStepContactR
            && landingMotionReady
            && loadedSoleReady
            && comp._physicalStepPlantDrift <= 0.040f
            && tiltDeg < 30.0f
            && !rag._locomotionSupportSaturated
            && rag.locomotionLiftBone < 0
            && rag._locomotionLiftForce <= 0.5f
            && locksOff
            && !comp._physicalStepMotorSaturated;
        if (comp._gaitLandingVerificationPending
            && comp._physicalStepPhase >= kSettle
            && comp._physicalStepPhase <= kHold) {
            comp._gaitLandingStableTime = landingStable
                ? comp._gaitLandingStableTime + dt : 0.0f;
        }

        if (comp._physicalStepPhase == kSettle
            || comp._physicalStepPhase == kSupportReady) {
            const float minimumSupportAdvance = glm::min(
                comp.gaitMinStepLength, comp.gaitMaxStepLength);
            if (!comp._physicalStepPlantPoseCaptured) {
                const bool provingSupportOwnership =
                    comp._physicalStepPhase == kSupportReady;
                const char* incomingPlantSide =
                    comp._physicalStepSupportSide < 0 ? "RIGHT" : "LEFT";
                const float plantSpeed = glm::length(swingVelocity);
                const float maxAcquireSpeed = continuousEnabled
                    ? glm::max(comp.plantAcquireMaxSpeed, 0.18f)
                    : glm::max(comp.plantAcquireMaxSpeed, 0.01f);
                constexpr float kPlantAcquireDriftLimit = 0.030f;
                constexpr float kPlantAcquireDriftReleaseLimit = 0.038f;
                constexpr float kPlantAcquireGrowthLimit = 0.020f;
                constexpr float kSettledOffsetDriftLimit = 0.040f;
                constexpr float kSettledOffsetGrowthLimit = 0.010f;
                constexpr float kSettledOffsetSpeedLimit = 0.030f;
                constexpr float kSettledOffsetTime = 0.10f;
                const bool settledOffsetCandidate = continuousEnabled
                    && !provingSupportOwnership
                    && !comp._physicalStepPlantAnchorRebased
                    && !comp._physicalStepPlantCenterAnchorActive
                    && !comp._physicalStepPlantPivotReleaseLatched
                    && swingContactNow && rawSwingContact && stanceContactNow
                    && plantSpeed <= kSettledOffsetSpeedLimit
                    && loadedSoleReady
                    && comp._physicalStepPlantDrift > kPlantAcquireDriftLimit
                    && comp._physicalStepPlantDrift <= kSettledOffsetDriftLimit
                    && std::abs(comp._gaitPlantDriftRate)
                        <= kSettledOffsetGrowthLimit;
                comp._physicalStepPlantSettledOffsetTime = settledOffsetCandidate
                    ? comp._physicalStepPlantSettledOffsetTime + dt : 0.0f;
                if (comp._physicalStepPlantSettledOffsetTime
                        >= kSettledOffsetTime
                    && swingRotationOk
                    && std::isfinite(contactPoint.x)
                    && std::isfinite(contactPoint.y)
                    && std::isfinite(contactPoint.z)) {
                    const float previousAnchorDrift =
                        comp._physicalStepPlantDrift;
                    const float previousCenterTravel =
                        comp._physicalStepPlantCenterTravel;
                    const float previousDriftRate = comp._gaitPlantDriftRate;
                    const glm::vec3 previousContactLocal =
                        comp._physicalStepTouchdownContactLocal;

                    // A sole that has stopped on a nearby patch is no longer sliding.
                    // Capture that physical contact as the new invariant instead of
                    // pulling the loaded foot back toward its first-impact patch.
                    comp._physicalStepTouchdownContactWorld = contactPoint;
                    comp._physicalStepTouchdownContactLocal =
                        comp._physicalStepContactLocal;
                    comp._physicalStepTouchdownContactValid = true;
                    comp._physicalStepTouchdownPlant = swingFoot;
                    swing->plantFoot =
                        comp._physicalStepTouchdownContactWorld
                        - nominalFootWorldRotation(*swing)
                            * comp._physicalStepTouchdownContactLocal;
                    comp._physicalStepPlantDrift = 0.0f;
                    comp._physicalStepPlantCenterTravel = 0.0f;
                    comp._gaitPlantPreviousDrift = 0.0f;
                    comp._gaitPlantDriftRate = 0.0f;
                    comp._physicalStepPlantAcquireStableTime = 0.0f;
                    comp._physicalStepPlantSettledOffsetTime = 0.0f;
                    comp._physicalStepPlantAnchorRebased = true;
                    comp._physicalStepPlantContactMigration = 0.0f;
                    comp._physicalStepPlantContactMigrationLogged = false;
                    comp._physicalStepPlantPivotStableTime = 0.0f;
                    spdlog::info(
                        "[LocomotionGait] PLANT_ANCHOR_REBASE step={} plant={} "
                        "oldDrift={:.3f}m centerTravel={:.3f}m rate={:+.3f}mps "
                        "contactLocal=({:+.3f},{:+.3f},{:+.3f})->"
                        "({:+.3f},{:+.3f},{:+.3f}) "
                        "correctionPeak={:.3f}/{:.3f}m saturated={} "
                        "action=acquire-settled-contact",
                        comp._stepSequenceStepIndex,
                        incomingPlantSide,
                        previousAnchorDrift,
                        previousCenterTravel,
                        previousDriftRate,
                        previousContactLocal.x,
                        previousContactLocal.y,
                        previousContactLocal.z,
                        comp._physicalStepTouchdownContactLocal.x,
                        comp._physicalStepTouchdownContactLocal.y,
                        comp._physicalStepTouchdownContactLocal.z,
                        comp._gaitPlantCorrectionPeakRequested,
                        comp._gaitPlantCorrectionPeakApplied,
                        comp._gaitPlantCorrectionSaturated ? "yes" : "no");
                }
                // Enter center ownership inside 30 mm, but do not discard an established
                // straight-walk center anchor for harmless 30-38 mm solver motion. The
                // existing 40 mm slip/transfer guards remain the hard safety boundary.
                const float activePlantDriftLimit = walkingOverlapHandoff
                        && provingSupportOwnership
                        && comp._physicalStepPlantCenterAnchorActive
                    ? kPlantAcquireDriftReleaseLimit
                    : kPlantAcquireDriftLimit;
                const bool plantDriftReady = !continuousEnabled
                    || comp._physicalStepPlantDrift
                        <= activePlantDriftLimit;
                const bool plantDriftNoLongerGrowing = !continuousEnabled
                    || comp._gaitPlantDriftRate
                        <= kPlantAcquireGrowthLimit;
                // Impact stability and support ownership are deliberately different
                // invariants. SETTLE must still promote the material-point pivot to a
                // center anchor. For straight walking, however, the center correction can
                // finish while transfer begins: contact, sole, speed, and drift are the
                // physical proof, and the anchor blend continues to run in the background.
                // Turn-conditioned steps retain the completed-blend requirement.
                const bool plantAnchorOwnershipReady = !continuousEnabled
                    || (provingSupportOwnership
                        && phaseAtFrameStart == kSupportReady
                        && comp._physicalStepPlantCenterAnchorActive
                        && (walkingOverlapHandoff
                            || comp._physicalStepPlantCenterBlendTime
                                >= kPlantCenterBlendDuration));
                const bool acquisitionKinematicallyStable =
                    swingContactNow && stanceContactNow
                    && plantSpeed <= maxAcquireSpeed
                    && loadedSoleReady
                    && plantDriftReady
                    && plantDriftNoLongerGrowing
                    && plantAnchorOwnershipReady;
                constexpr float kRecoverableAcquireDriftLimit = 0.030f;
                constexpr float kRecoverableAcquireGrowthLimit = 0.080f;
                constexpr float kRecoverableAcquireGraceTime = 0.20f;
                const float baseAcquireTimeout = glm::max(
                    comp.plantAcquireTimeout, 0.20f);
                const bool recoverableDriftTrend =
                    comp._physicalStepPlantDrift
                        <= kRecoverableAcquireDriftLimit
                    && comp._gaitPlantDriftRate
                        <= kRecoverableAcquireGrowthLimit;
                const bool recoverableSettledOffset =
                    comp._physicalStepPlantDrift > kPlantAcquireDriftLimit
                    && comp._physicalStepPlantDrift
                        <= kSettledOffsetDriftLimit
                    && plantSpeed <= kSettledOffsetSpeedLimit
                    && std::abs(comp._gaitPlantDriftRate)
                        <= kSettledOffsetGrowthLimit;
                const bool recoverableAcquireGrace = continuousEnabled
                    && swingContactNow && stanceContactNow
                    && plantSpeed <= maxAcquireSpeed
                    && loadedSoleReady
                    && (recoverableDriftTrend
                        || recoverableSettledOffset);
                const float ordinaryAcquireTimeout = baseAcquireTimeout
                    + (recoverableAcquireGrace
                        ? kRecoverableAcquireGraceTime : 0.0f);
                // SUPPORT_READY owns a fresh clock. Its bounded deadline includes the
                // immutable center blend plus an independent stability proof.
                const float postHandoffAcquireDeadline = provingSupportOwnership
                    ? kPlantCenterBlendDuration + cadencePlantAcquireTime
                        + kPostHandoffAcquireMargin
                    : 0.0f;
                const float activeAcquireTimeout = glm::max(
                    ordinaryAcquireTimeout, postHandoffAcquireDeadline);
                // The very first straight support handoff starts from a standing contact
                // distribution rather than the repeatable walking limit cycle. Give that
                // center anchor 50 ms of continuous proof before moving load; subsequent
                // steps retain the cadence-sized overlap proof.
                const float requiredPlantAcquireTime =
                    straightStartupSupport && provingSupportOwnership
                        ? glm::max(cadencePlantAcquireTime, 0.05f)
                        : cadencePlantAcquireTime;
                if (continuousEnabled
                    && comp._physicalStepPlantDrift > 0.020f
                    && !comp._gaitPlantRecoveryLogged) {
                    comp._gaitPlantRecoveryLogged = true;
                    spdlog::info(
                        "[LocomotionGait] PLANT_RECOVERY step={} "
                        "anchorDrift={:.3f}/{:.3f}m centerTravel={:.3f}m "
                        "rate={:+.3f}/{:+.3f}mps "
                        "supportSpeed={:.3f} correctionPeak={:.3f}/{:.3f}m "
                        "saturated={} action=decelerate-lateral-and-correct",
                        comp._stepSequenceStepIndex,
                        comp._physicalStepPlantDrift,
                        kPlantAcquireDriftLimit,
                        comp._physicalStepPlantCenterTravel,
                        comp._gaitPlantDriftRate,
                        kPlantAcquireGrowthLimit,
                        glm::length(comp._gaitSupportCommandVelocity),
                        comp._gaitPlantCorrectionPeakRequested,
                        comp._gaitPlantCorrectionPeakApplied,
                        comp._gaitPlantCorrectionSaturated ? "yes" : "no");
                }
                comp._physicalStepPlantAcquireStableTime = acquisitionKinematicallyStable
                    ? comp._physicalStepPlantAcquireStableTime + dt : 0.0f;
                if (recoverableAcquireGrace
                    && comp._physicalStepPhaseTime >= baseAcquireTimeout
                    && comp._physicalStepPhaseTime - dt < baseAcquireTimeout) {
                    spdlog::info(
                        "[LocomotionGait] PLANT_ACQUIRE_GRACE step={} plant={} "
                        "mode={} timeout={:.3f}->{:.3f}s anchorDrift={:.3f} "
                        "centerTravel={:.3f} rate={:+.3f} speed={:.3f} sole={:.1f}",
                        comp._stepSequenceStepIndex,
                        incomingPlantSide,
                        recoverableSettledOffset ? "settled-offset" : "trend",
                        baseAcquireTimeout, activeAcquireTimeout,
                        comp._physicalStepPlantDrift,
                        comp._physicalStepPlantCenterTravel,
                        comp._gaitPlantDriftRate,
                        plantSpeed,
                        comp._gaitSoleAngularErrorDeg);
                }
                if (comp._physicalStepPlantAcquireStableTime
                    >= requiredPlantAcquireTime) {
                    const float retainedFromTarget =
                        comp._gaitPlannedSupportAdvance
                        - glm::max(comp._physicalStepForwardTargetError, 0.0f);
                    // Runtime landing accepts a bounded tracking miss and feeds it into the
                    // next foothold. A stable physical contact is recoverable; treating a
                    // few centimetres of servo lag as ABORT caused the release-to-restart
                    // behavior and pulled the COM back to the old baseline.
                    using TurnObjective = Comp::TurnStepObjective;
                    const bool translationLandingObjective =
                        comp._gaitTurnPlan.objective
                            == TurnObjective::Translation
                        || comp._gaitTurnPlan.objective
                            == TurnObjective::Combined;
                    const float objectiveAdvanceTolerance = glm::max(
                        comp.footTargetTolerance, 0.01f);
                    const float requiredAdvance = comp._gaitCancelMode == 1
                        || !translationLandingObjective
                        ? 0.0f
                        : (comp._gaitTurnPlan.objective
                            == TurnObjective::Combined
                            ? glm::max(comp._gaitPlannedSupportAdvance
                                - objectiveAdvanceTolerance, 0.0f)
                            : glm::min(minimumSupportAdvance, glm::max(
                                0.03f, comp._gaitPlannedSupportAdvance
                                    - objectiveAdvanceTolerance)));
                    constexpr float kLandingAdvanceHysteresis = 0.010f;
                    // The validation run's eight recovery-triggering landings missed the
                    // existing hysteresis by only 2-7 mm after contact, ownership, sole,
                    // and drift had already passed. Keep that measured numerical/servo
                    // closure separate from the ordinary hysteresis and from real recovery.
                    constexpr float kLandingObjectiveClosureTolerance = 0.008f;
                    const bool retainsThroughHysteresis = !continuousEnabled
                        || comp._gaitAchievedSupportAdvance
                            + kLandingAdvanceHysteresis >= requiredAdvance;
                    const bool retainsMinimumAdvance = !continuousEnabled
                        || comp._gaitAchievedSupportAdvance
                            + kLandingAdvanceHysteresis
                            + (translationLandingObjective
                                ? kLandingObjectiveClosureTolerance : 0.0f)
                            >= requiredAdvance;
                    comp._gaitTurnPlan.translationObjectiveSatisfied =
                        !translationLandingObjective || retainsMinimumAdvance;
                    if (continuousEnabled
                        && comp._gaitAchievedSupportAdvance < requiredAdvance
                        && retainsThroughHysteresis) {
                        spdlog::info(
                            "[LocomotionGait] LANDING_ADVANCE_CHECK "
                            "result=HYSTERESIS achieved={:.3f}/{:.3f} margin={:.3f}m",
                            comp._gaitAchievedSupportAdvance,
                            requiredAdvance,
                            kLandingAdvanceHysteresis);
                    }
                    if (continuousEnabled
                        && translationLandingObjective
                        && comp._gaitAchievedSupportAdvance < requiredAdvance
                        && !retainsThroughHysteresis
                        && retainsMinimumAdvance) {
                        spdlog::info(
                            "[LocomotionGait] LANDING_ADVANCE_CHECK "
                            "result=OBJECTIVE_CLOSURE objective={} "
                            "achieved={:.3f}/{:.3f} hysteresis={:.3f}m "
                            "closure={:.3f}m action=accept-stable-contact",
                            TurnObjectiveName(comp._gaitTurnPlan.objective),
                            comp._gaitAchievedSupportAdvance,
                            requiredAdvance,
                            kLandingAdvanceHysteresis,
                            kLandingObjectiveClosureTolerance);
                    }
                    if (!retainsMinimumAdvance) {
                        if (comp._gaitTurnPlan.objective
                            == TurnObjective::Angular) {
                            // A center-owned, quiet turning footprint remains useful even
                            // when impact tracking loses more forward advance than the
                            // straight gait permits. Preserve the physical plant and let the
                            // next plan originate from the measured feet. The completed
                            // transfer still has to satisfy load, contact, sole, drift, tilt,
                            // heading, and saturation before yaw can commit.
                            spdlog::warn(
                                "[LocomotionGait] LANDING_ADVANCE_CHECK "
                                "result=TURN_REPLAN planned={:.3f} "
                                "retainedFromTarget={:.3f} achieved={:.3f}/{:.3f} "
                                "forwardError={:+.3f} yaw={:+.3f}deg "
                                "stanceDrift={:.3f} action=accept-actual-footprint",
                                comp._gaitPlannedSupportAdvance,
                                retainedFromTarget,
                                comp._gaitAchievedSupportAdvance,
                                requiredAdvance,
                                comp._physicalStepForwardTargetError,
                                glm::degrees(
                                    comp._gaitTurnPlan.admittedYaw),
                                comp._physicalStepStanceDrift);
                        } else if (comp._gaitTurnPlan.objective
                            == TurnObjective::Combined) {
                            // The angular half of a quiet combined contact remains valid
                            // even when the requested forward component lands short. Keep
                            // the measured footprint, commit only after the ordinary angular
                            // transfer proof, and let the next plan retry translation from
                            // the physical feet. Translation-only steps remain strict below.
                            spdlog::warn(
                                "[LocomotionGait] LANDING_ADVANCE_CHECK "
                                "result=COMBINED_REPLAN objective=combined "
                                "planned={:.3f} retainedFromTarget={:.3f} "
                                "achieved={:.3f}/{:.3f} forwardError={:+.3f} "
                                "yaw={:+.3f}deg stanceDrift={:.3f} "
                                "action=accept-actual-footprint-retry-translation",
                                comp._gaitPlannedSupportAdvance,
                                retainedFromTarget,
                                comp._gaitAchievedSupportAdvance,
                                requiredAdvance,
                                comp._physicalStepForwardTargetError,
                                glm::degrees(
                                    comp._gaitTurnPlan.admittedYaw),
                                comp._physicalStepStanceDrift);
                        } else {
                            spdlog::warn(
                                "[LocomotionGait] LANDING_ADVANCE_CHECK result=RECOVER "
                                "cause=landing-objective-shortfall objective={} "
                                "planned={:.3f} retainedFromTarget={:.3f} "
                                "achieved={:.3f}/{:.3f} forwardError={:+.3f} "
                                "stanceDrift={:.3f}",
                                TurnObjectiveName(comp._gaitTurnPlan.objective),
                                comp._gaitPlannedSupportAdvance,
                                retainedFromTarget,
                                comp._gaitAchievedSupportAdvance,
                                requiredAdvance,
                                comp._physicalStepForwardTargetError,
                                comp._physicalStepStanceDrift);
                            // Straight translation still requires a meaningful forward
                            // support change. Finish this contact safely and stop so a held
                            // command can restart from a valid baseline.
                            comp._gaitStopRequested = true;
                            comp._gaitLandingObjectiveStopRequested = true;
                            comp._gaitCancelMode = 2;
                        }
                    }
                    if (!continuousEnabled)
                        capturePhysicalLocalPose(*swing);
                    // TRANSFER consumes only the center-owned anchor proved in the explicit
                    // SUPPORT_READY state. A quiet impact pivot is never promoted directly.
                    comp._physicalStepPlantPoseCaptured = true;
                    comp._physicalStepSettleTime = 0.0f;
                    comp._gaitLandingVerificationPending = true;
                    comp._gaitLandingStableTime = 0.0f;
                    spdlog::info(
                        "[LocomotionGait] SUPPORT_OWNERSHIP_READY step={} plant={} "
                        "stable={:.3f}/{:.3f}s centerBlend={:.3f}/{:.3f}s "
                        "overlap={} "
                        "drift={:.3f}m speed={:.3f}mps sole={:.1f}deg "
                        "supportAction=release-transfer",
                        comp._stepSequenceStepIndex,
                        incomingPlantSide,
                        comp._physicalStepPlantAcquireStableTime,
                        requiredPlantAcquireTime,
                        comp._physicalStepPlantCenterBlendTime,
                        kPlantCenterBlendDuration,
                        walkingOverlapHandoff ? "yes" : "no",
                        comp._physicalStepPlantDrift,
                        plantSpeed,
                        comp._gaitSoleAngularErrorDeg);

                    // Landing verification can continue alongside transfer now that
                    // support ownership itself is established. An early-return
                    // cancellation verifies in place and enters standing directly.
                    if (comp._gaitCancelMode != 1 && transferEnabled)
                        beginTransfer();
                } else if (comp._physicalStepPhaseTime
                               >= activeAcquireTimeout) {
                    spdlog::warn(
                        "[LocomotionStep] PLANT_ACQUIRE result=FAIL state={} plant={} contact={} "
                        "stance={} speed={:.3f}/{:.3f} stable={:.3f}/{:.3f}s "
                        "timeout={:.3f}/{:.3f}s forward={:.3f} "
                        "anchorDrift={:.3f}/{:.3f} centerTravel={:.3f} "
                        "driftRate={:+.3f}/{:+.3f} sole={:.1f}/{:.1f}[{}] "
                        "correctionPeak={:.3f}/{:.3f} saturated={} "
                        "rebased={} centerAnchor={} ownershipReady={} "
                        "pivotRelease={}/{:.2f} handoffAt={:.3f}s "
                        "migration={:.3f}m maxQuiet={:.3f}/{:.3f}s "
                        "blocked=(contact={:.3f},sole={:.3f},angular={:.3f},"
                        "linear={:.3f})s "
                        "angularSpeed={:.3f}radps pivotQuiet={:.3f}s "
                        "settled={:.3f}s",
                        provingSupportOwnership ? "SUPPORT_READY" : "SETTLE",
                        incomingPlantSide,
                        swingContactNow ? "yes" : "no",
                        stanceContactNow ? "yes" : "no",
                        plantSpeed, maxAcquireSpeed,
                        comp._physicalStepPlantAcquireStableTime,
                        requiredPlantAcquireTime,
                        comp._physicalStepPhaseTime,
                        activeAcquireTimeout,
                        comp._physicalStepForwardTravel,
                        comp._physicalStepPlantDrift,
                        kPlantAcquireDriftLimit,
                        comp._physicalStepPlantCenterTravel,
                        comp._gaitPlantDriftRate,
                        kPlantAcquireGrowthLimit,
                        comp._gaitSoleAngularErrorDeg,
                        kPlantOwnershipSoleToleranceDeg,
                        loadedSoleReady ? "ok" : "FAIL",
                        comp._gaitPlantCorrectionPeakRequested,
                        comp._gaitPlantCorrectionPeakApplied,
                        comp._gaitPlantCorrectionSaturated ? "yes" : "no",
                        comp._physicalStepPlantAnchorRebased ? "yes" : "no",
                        comp._physicalStepPlantCenterAnchorActive ? "yes" : "no",
                        plantAnchorOwnershipReady ? "yes" : "no",
                        comp._physicalStepPlantPivotReleaseLatched
                            ? "yes" : "no",
                        comp._physicalStepPlantPivotReleaseWeight,
                        comp._physicalStepPlantAnchorHandoffPhaseTime,
                        comp._physicalStepPlantContactMigration,
                        comp._physicalStepPlantPivotMaxStableTime,
                        kPlantPivotQuietTime,
                        comp._physicalStepPlantPivotContactBlockedTime,
                        comp._physicalStepPlantPivotSoleBlockedTime,
                        comp._physicalStepPlantPivotAngularBlockedTime,
                        comp._physicalStepPlantPivotLinearBlockedTime,
                        comp._physicalStepPlantAngularSpeed,
                        comp._physicalStepPlantPivotStableTime,
                        comp._physicalStepPlantSettledOffsetTime);
                    abortSequence(provingSupportOwnership
                        ? "center-owned support did not stabilize before readiness timeout"
                        : "impact pivot did not reach center ownership before settle timeout");
                }
            }
            if (comp._gaitCancelMode == 1
                && comp._physicalStepPlantPoseCaptured
                && comp._gaitLandingStableTime >= cadenceLandingVerifyTime) {
                comp._gaitLandingVerificationPending = false;
                beginStopping();
            }
        } else if (comp._physicalStepPhase == kTransfer) {
            const float minimumDynamicTransferTime = glm::min(
                cadenceTransferTime, 0.08f);
            // Validate the next release while the support curve is still transferring.
            // HOLD remains the authoritative full gate below, but its quiet-time clock no
            // longer starts from zero after TRANSFER has already demonstrated the same
            // contact/load/sole conditions for several frames.
            const bool transferPreloadStable = continuousEnabled
                && comp._gaitNewSupportLoadLatched
                && loadedSoleReady
                && swingContactNow && stanceContactNow
                && glm::length(swingVelocity) < 0.18f
                && comp._physicalStepStanceDrift <= 0.040f
                && comp._physicalStepPlantDrift <= 0.040f
                && tiltDeg < 30.0f
                && !rag._locomotionSupportSaturated
                && rag.locomotionLiftBone < 0
                && rag._locomotionLiftForce <= 0.5f
                && !comp._physicalStepMotorSaturated;
            comp._supportTransferHoldStableTime = transferPreloadStable
                ? comp._supportTransferHoldStableTime + dt : 0.0f;
            const bool loadHandoffReady = continuousEnabled
                && comp._physicalStepPhaseTime >= minimumDynamicTransferTime
                && comp._gaitNewSupportLoadLatched
                && loadedSoleReady;
            if (loadHandoffReady
                || comp._physicalStepPhaseTime >= cadenceTransferTime) {
                // HOLD validates the physical handoff but never freezes the continuous
                // support curve. Position and feed-forward velocity keep advancing.
                comp._physicalStepPhase = kHold;
                comp._physicalStepPhaseTime = 0.0f;
            }
        } else if (comp._physicalStepPhase == kHold) {
            const float comTolerance = glm::max(comp.transferComTolerance, 0.01f);
            const float newSupportRadius = glm::max(comTolerance, 0.065f);
            const bool comAtTarget = comp._supportTransferComError <= comTolerance;
            const float liveSupportError = horizontalDistance(
                rag._locomotionCOM, comp._physicalStepSupportTarget);
            const float liveSupportTolerance = glm::max(comTolerance, 0.065f);
            // The continuous curve deliberately cruises beyond its Hermite endpoint.
            // Validating against that stale endpoint made HOLD completion depend on the
            // landing-verification window coinciding with the instant the COM passed it.
            // Track the command that actually owns support instead; load, contact, sole,
            // drift, motion, tilt, and saturation remain independent safety gates below.
            const bool transferPositionReady = continuousEnabled
                ? liveSupportError <= liveSupportTolerance
                : comAtTarget;
            const bool insideNewSupport =
                comp._supportTransferComToNewSupport <= newSupportRadius;
            // We do not yet expose a per-foot normal impulse, so project the COM along the
            // complete old-to-new support span. Straight walking acquires at 52% and keeps
            // moving the same curve after the role swap; 48% hysteresis prevents sub-frame
            // COM noise from repeatedly resetting the stable window. Turn-conditioned
            // steps retain their conservative 68%/64% handoff.
            const bool oldLegUnloaded = comp._gaitNewSupportLoadLatched;
            const bool locksOff = rag.locomotionFootLockWeights[0] <= 0.001f
                               && rag.locomotionFootLockWeights[1] <= 0.001f
                               && rag._locomotionFootLockForce[0] <= 0.5f
                               && rag._locomotionFootLockForce[1] <= 0.5f;
            const bool transferMotionReady = continuousEnabled
                ? glm::length(swingVelocity) < 0.18f
                    && comp._supportTransferComHorizontalSpeed < 0.50f
                : glm::length(leftVelocity) < 0.15f
                    && glm::length(rightVelocity) < 0.15f
                    && comp._supportTransferComHorizontalSpeed < 0.15f;
            const bool transferContactReady =
                swingContactNow && stanceContactNow;
            const bool stanceDriftReady = continuousEnabled
                || comp._physicalStepStanceDrift <= 0.040f;
            const bool plantDriftReady =
                comp._physicalStepPlantDrift <= 0.040f;
            const bool transferTiltReady = tiltDeg < 30.0f;
            const bool supportAuthorityReady =
                !rag._locomotionSupportSaturated;
            const bool liftReleased = rag.locomotionLiftBone < 0
                && rag._locomotionLiftForce <= 0.5f;
            const bool transferMotorReady =
                !comp._physicalStepMotorSaturated;
            using TurnObjective = Comp::TurnStepObjective;
            const bool requiresAngularObjective =
                comp._gaitTurnPlan.objective == TurnObjective::Angular
                || comp._gaitTurnPlan.objective == TurnObjective::Combined;
            const float admittedYawDeg = std::abs(glm::degrees(
                comp._gaitTurnPlan.admittedYaw));
            const float angularObjectiveToleranceDeg = glm::max(
                2.0f, admittedYawDeg * 0.40f);
            const bool angularDirectionReady =
                comp._gaitTurnPlan.achievedYaw
                    * comp._gaitTurnPlan.admittedYaw >= 0.0f;
            const bool angularMagnitudeReady =
                std::abs(glm::degrees(comp._gaitTurnPlan.achievedYaw))
                    + angularObjectiveToleranceDeg >= admittedYawDeg;
            const bool angularObjectiveReady = !requiresAngularObjective
                || (comp._gaitTurnPlan.plannedTurnProgress >= 0.999f
                    && std::abs(comp._gaitHeadingErrorDeg)
                        <= angularObjectiveToleranceDeg
                    && angularDirectionReady
                    && angularMagnitudeReady);
            comp._gaitTurnPlan.angularObjectiveSatisfied =
                angularObjectiveReady;
            const bool stableTransfer = transferPositionReady
                && transferContactReady
                && oldLegUnloaded
                && loadedSoleReady
                && transferMotionReady
                && stanceDriftReady
                && plantDriftReady
                && transferTiltReady
                && supportAuthorityReady
                && liftReleased
                && locksOff
                && transferMotorReady
                && angularObjectiveReady;
            comp._supportTransferHoldStableTime = stableTransfer
                ? comp._supportTransferHoldStableTime + dt : 0.0f;

            const bool landingVerified = !comp._gaitLandingVerificationPending
                || comp._gaitLandingStableTime >= cadenceLandingVerifyTime;
            const float requiredTransferHoldTime = continuousEnabled
                ? glm::min(cadenceTransferHoldTime, 0.03f)
                : cadenceTransferHoldTime;
            if (landingVerified
                && comp._supportTransferHoldStableTime
                    >= requiredTransferHoldTime) {
                comp._gaitLandingVerificationPending = false;
                using TurnObjective = Comp::TurnStepObjective;
                const bool completedTranslationObjective =
                    comp._gaitTurnPlan.objective
                        == TurnObjective::Translation
                    || comp._gaitTurnPlan.objective
                        == TurnObjective::Combined;
                const bool completedStraightTranslationObjective =
                    comp._gaitTurnPlan.objective
                        == TurnObjective::Translation;
                if (comp._gaitCancelMode == 0) {
                    // Pure turns freeze only forward stride learning. Contact, lateral,
                    // drift, motor, impact, and load-transfer feedback remain live.
                    updateGaitAdaptation(completedTranslationObjective);
                }
                if (continuousEnabled
                    && completedStraightTranslationObjective) {
                    // Combined turn footprints retain their ordinary forward-error and
                    // stride adaptation above, but their turn-dominant loss must not
                    // inflate the reserve used to admit a later straight/exit step.
                    const float settledLoss = glm::max(
                        comp._gaitPlannedSupportAdvance
                            - comp._gaitAchievedSupportAdvance,
                        0.0f);
                    comp._gaitSettledTrackingLoss = settledLoss
                        > comp._gaitSettledTrackingLoss
                        ? settledLoss
                        : glm::mix(comp._gaitSettledTrackingLoss,
                                   settledLoss, 0.25f);
                }
                if (multiStepEnabled) {
                    if (continuousEnabled) {
                        comp._stepSequenceStepsCompleted = comp._stepSequenceStepIndex;
                        comp._gaitPreviousStepPeriod = comp._gaitLastStepPeriod;
                        comp._gaitLastStepPeriod = glm::max(
                            comp._gaitRunTime - comp._gaitStepStartTime, dt);
                        comp._gaitPreviousStepLength = comp._gaitLastStepLength;
                        comp._gaitLastStepLength = comp._physicalStepForwardTravel;
                        comp._gaitPreviousSupportAdvance =
                            comp._gaitLastSupportAdvance;
                        comp._gaitLastSupportAdvance =
                            comp._gaitAchievedSupportAdvance;
                        const float comAdvance = glm::dot(
                            rag._locomotionCOM - comp._gaitStepStartCom,
                            stepObjectiveForward());
                        comp._gaitMeasuredSpeed = comAdvance
                            / comp._gaitLastStepPeriod;
                        spdlog::info(
                            "[LocomotionCadence] ACTUAL step={} period={:.3f}s "
                            "target={:.3f}s equation={:.3f}s error={:+.3f}s "
                            "rate={:.2f}steps/s recontactPause={:.3f}s "
                            "speculativeRejected={} stability=(solePeak={:.1f}deg,"
                            "angularPeak={:.2f}radps)",
                            comp._stepSequenceStepIndex,
                            comp._gaitLastStepPeriod,
                            effectiveTargetStepPeriod,
                            cadenceEquationPeriod,
                            comp._gaitLastStepPeriod
                                - effectiveTargetStepPeriod,
                            1.0f / glm::max(
                                comp._gaitLastStepPeriod, 0.01f),
                            comp._gaitStepRecontactPauseTime,
                            comp._gaitStepSpeculativeContacts,
                            comp._gaitStepMaxSoleErrorDeg,
                            comp._gaitStepMaxSwingAngularSpeed);
                        const float stepDrift =
                            comp._gaitStepMaxRelevantDrift;
                        comp._gaitMaxDrift = glm::max(
                            comp._gaitMaxDrift, stepDrift);
                        comp._gaitPeakTilt = glm::max(
                            comp._gaitPeakTilt, comp._physicalStepPeakTilt);
                        comp._gaitMaxMotorRatio = glm::max(
                            comp._gaitMaxMotorRatio, comp._physicalStepMaxMotorRatio);

                        if (comp._gaitCancelMode == 0)
                            commitActiveTurnHeading();

                        const bool shouldStop = comp._gaitStopRequested;

                        if (shouldStop) {
                            beginStopping();
                        } else {
                            beginCommittedSupportTransport();
                            comp._physicalStepPhase = kInterStep;
                            comp._physicalStepPhaseTime = 0.0f;
                            comp._stepSequenceInterStepStableTime = 0.0f;
                            comp._gaitInterStepRecenterStart =
                                comp._physicalStepSupportTarget;
                            // TRANSFER already established the next stable support point.
                            // Re-centering to the sole midpoint here pulled the character
                            // backward after every successful step. Hold that achieved
                            // world-space target while the next role swap is admitted.
                            comp._gaitInterStepRecenterTarget =
                                comp._physicalStepSupportTarget;
                            comp._gaitInterStepRecenterT = 0.0f;
                            comp._gaitInterStepCenterError =
                                horizontalDistance(
                                    rag._locomotionCOM,
                                    comp._gaitInterStepRecenterTarget);
                        }
                    } else {
                    const int stepSlot = glm::clamp(comp._stepSequenceStepIndex - 1, 0, 1);
                    comp._stepSequenceStepForward[stepSlot] = comp._physicalStepForwardTravel;
                    comp._stepSequenceStepMaxDrift[stepSlot] = glm::max(
                        comp._physicalStepMaxStanceDrift, comp._physicalStepMaxPlantDrift);
                    comp._stepSequenceStepPeakTilt[stepSlot] = comp._physicalStepPeakTilt;
                    comp._stepSequenceStepMotorRatio[stepSlot] = comp._physicalStepMaxMotorRatio;
                    comp._stepSequenceStepsCompleted = comp._stepSequenceStepIndex;

                    if (comp._stepSequenceStepIndex == 1) {
                        comp._physicalStepPhase = kInterStep;
                        comp._physicalStepPhaseTime = 0.0f;
                        comp._stepSequenceInterStepStableTime = 0.0f;
                    } else {
                        const float driftTolerance = glm::max(
                            comp.driftGrowthTolerance, 0.0f);
                        const bool driftGrowthOk = comp._stepSequenceStepMaxDrift[1]
                            <= comp._stepSequenceStepMaxDrift[0] + driftTolerance;
                        const bool motorGrowthOk = comp._stepSequenceStepMotorRatio[1]
                            <= comp._stepSequenceStepMotorRatio[0] + 0.05f;
                        const bool finalTiltOk = std::abs(
                            tiltDeg - comp._stepSequenceInitialTilt) <= 10.0f;
                        const bool contactEdgesOk = comp._stepSequenceContactTransitionsL == 2
                                                 && comp._stepSequenceContactTransitionsR == 2;
                        const bool sequencePass = driftGrowthOk && motorGrowthOk
                            && finalTiltOk && contactEdgesOk
                            && comp._stepSequenceStepsCompleted == 2;
                        if (sequencePass) {
                            comp._physicalStepPhase = kComplete;
                            comp._physicalStepPhaseTime = 0.0f;
                        } else {
                            abortSequence("two-step accumulation check failed");
                        }
                    }
                    }
                } else {
                    comp._physicalStepPhase = kComplete;
                    comp._physicalStepPhaseTime = 0.0f;
                }
            } else if (comp._physicalStepPhaseTime >= glm::max(
                           comp.transferTimeout,
                           cadenceTransferHoldTime)) {
                spdlog::warn(
                    "[LocomotionGait] TRANSFER_CHECK result=FAIL "
                    "endpointErr={:.3f}/{:.3f} commandErr={:.3f}/{:.3f}[{}] "
                    "newDistance={:.3f}/{:.3f}[{};telemetry] "
                    "load=(actual={:.3f},command={:.3f},target={:.2f},latch={:.2f})[{}] "
                    "contact=({},{}) speed={:.3f} drift=({:.3f},{:.3f}) "
                    "tilt={:.1f}/30 stable={:.3f}/{:.3f}s "
                    "gates=(position={},contact={},load={},sole={},motion={},"
                    "stanceDrift={},plantDrift={},tilt={},support={},lift={},"
                    "locks={},motor={},landing={},objective={}({}))",
                    comp._supportTransferComError, comTolerance,
                    liveSupportError, liveSupportTolerance,
                    transferPositionReady ? "ok" : "FAIL",
                    comp._supportTransferComToNewSupport, newSupportRadius,
                    insideNewSupport ? "near" : "far",
                    comp._gaitNewSupportLoad,
                    gaitSupportCommandLoad,
                    kNewSupportLoadCommandTarget,
                    kNewSupportLoadAcquireThreshold,
                    oldLegUnloaded ? "ok" : "FAIL",
                    comp._physicalStepContactL ? "L" : "-",
                    comp._physicalStepContactR ? "R" : "-",
                    comp._supportTransferComHorizontalSpeed,
                    comp._physicalStepStanceDrift, comp._physicalStepPlantDrift,
                    tiltDeg,
                    comp._supportTransferHoldStableTime,
                    requiredTransferHoldTime,
                    transferPositionReady ? "ok" : "FAIL",
                    transferContactReady ? "ok" : "FAIL",
                    oldLegUnloaded ? "ok" : "FAIL",
                    loadedSoleReady ? "ok" : "FAIL",
                    transferMotionReady ? "ok" : "FAIL",
                    stanceDriftReady ? "ok" : "FAIL",
                    plantDriftReady ? "ok" : "FAIL",
                    transferTiltReady ? "ok" : "FAIL",
                    supportAuthorityReady ? "ok" : "FAIL",
                    liftReleased ? "ok" : "FAIL",
                    locksOff ? "ok" : "FAIL",
                    transferMotorReady ? "ok" : "FAIL",
                    landingVerified ? "ok" : "FAIL",
                    TurnObjectiveName(comp._gaitTurnPlan.objective),
                    angularObjectiveReady ? "ok" : "FAIL");
                abortSequence("support transfer did not settle before hold timeout");
            }
        } else if (comp._physicalStepPhase == kInterStep) {
            const float interStepTiltLimit = continuousEnabled
                ? glm::clamp(comp.gaitInterStepTiltLimit, 12.0f, 25.0f)
                : 30.0f;
            const float interStepHeadingLimit = continuousEnabled
                ? glm::clamp(comp.gaitInterStepHeadingLimit, 2.0f, 20.0f)
                : 180.0f;
            const bool recenterReady = !continuousEnabled
                || (comp._gaitInterStepRecenterT >= 0.999f
                    && comp._gaitInterStepCenterError
                        <= glm::max(comp.transferComTolerance, 0.04f));
            const bool uprightReady = !continuousEnabled
                || !rag._locomotionUprightSaturated;
            const bool headingReady = !continuousEnabled
                || std::abs(comp._gaitHeadingErrorDeg) <= interStepHeadingLimit;
            const bool interStepReady = continuousEnabled
                ? swingContactNow && stanceContactNow
                    && comp._gaitNewSupportLoadLatched
                    && loadedSoleReady
                    && tiltDeg <= interStepTiltLimit
                    && headingReady
                    && uprightReady
                    && !rag._locomotionSupportSaturated
                    && !comp._physicalStepMotorSaturated
                : swingContactNow && stanceContactNow
                    && glm::length(leftVelocity) < 0.15f
                    && glm::length(rightVelocity) < 0.15f
                    && comp._supportTransferComHorizontalSpeed < 0.15f
                    && tiltDeg <= interStepTiltLimit
                    && headingReady
                    && recenterReady
                    && uprightReady
                    && !rag._locomotionSupportSaturated
                    && !comp._physicalStepMotorSaturated;
            comp._stepSequenceInterStepStableTime = interStepReady
                ? comp._stepSequenceInterStepStableTime + dt : 0.0f;
            const float requiredInterStepTime = continuousEnabled
                ? 0.0f : cadenceInterStepTime;
            if (comp._stepSequenceInterStepStableTime >= requiredInterStepTime) {
                const float completedPlannedAdvance =
                    comp._gaitPlannedSupportAdvance;
                const float completedAchievedAdvance =
                    comp._gaitAchievedSupportAdvance;
                const bool completedReachClamped =
                    comp._gaitReachClampedStep;
                // The legs already hold their last analytic support commands.
                // Recapturing their measured rotations here made whatever tracking error
                // remained after this step the commanded starting pose of the next one.
                // Tests 4-6 retain their already-validated physical handoff behavior.
                if (!continuousEnabled) {
                    capturePhysicalLocalPose(*swing);
                    capturePhysicalLocalPose(*stance);
                }
                const int oldSupportSide = comp._physicalStepSupportSide;
                const glm::vec3 completedSupportTarget =
                    comp._physicalStepSupportTarget;
                comp._physicalStepFootBaselineL = leftFoot;
                comp._physicalStepFootBaselineR = rightFoot;
                comp._physicalStepComBaseline = rag._locomotionCOM;
                comp._physicalStepSupportSide = -oldSupportSide;
                if (continuousEnabled) {
                    comp._physicalStepSupportTarget = completedSupportTarget;
                    comp._physicalStepComCommand =
                        static_cast<float>(comp._physicalStepSupportSide);
                    comp._gaitContinuousCycle = true;
                    comp._gaitBypassWeightShift = true;
                    comp._gaitCycleSupportTarget = completedSupportTarget;
                } else {
                    comp._physicalStepSupportTarget =
                        comp._physicalStepComBaseline;
                    comp._physicalStepComCommand = 0.0f;
                }
                comp._physicalStepComLateral = 0.0f;
                comp._physicalStepTargetLateral = 0.0f;
                comp._physicalStepPhase = kWeightShift;
                comp._physicalStepPhaseTime = 0.0f;
                comp._physicalStepSettleTime = 0.0f;
                comp._physicalStepAirborneTime = 0.0f;
                comp._physicalStepArrivalStableTime = 0.0f;
                comp._physicalStepReachLimit = 0.0f;
                comp._gaitPlannedSupportAdvance = 0.0f;
                comp._gaitAchievedSupportAdvance = 0.0f;
                comp._physicalStepPlantAcquireStableTime = 0.0f;
                comp._physicalStepPlantSettledOffsetTime = 0.0f;
                comp._physicalStepPlantUnsafeTime = 0.0f;
                comp._physicalStepPlantAnchorRebased = false;
                comp._physicalStepPlantCenterAnchorActive = false;
                comp._physicalStepPlantContactMigrationLogged = false;
                comp._physicalStepPlantPivotReleaseLatched = false;
                comp._physicalStepPlantPivotStableTime = 0.0f;
                comp._physicalStepPlantPivotMaxStableTime = 0.0f;
                comp._physicalStepPlantPivotReleaseTriggerTime = 0.0f;
                comp._physicalStepPlantPivotReleaseTime = 0.0f;
                comp._physicalStepPlantPivotReleaseWeight = 0.0f;
                comp._physicalStepPlantCenterBlendTime = 0.0f;
                comp._physicalStepPlantAnchorTelemetryTime = 0.0f;
                comp._physicalStepPlantAnchorHandoffPhaseTime = -1.0f;
                comp._physicalStepPlantPivotContactBlockedTime = 0.0f;
                comp._physicalStepPlantPivotSoleBlockedTime = 0.0f;
                comp._physicalStepPlantPivotAngularBlockedTime = 0.0f;
                comp._physicalStepPlantPivotLinearBlockedTime = 0.0f;
                comp._physicalStepPlantContactMigration = 0.0f;
                comp._physicalStepPlantAngularSpeed = 0.0f;
                comp._physicalStepPlantCenterAnchorStart = glm::vec3(0.0f);
                comp._physicalStepPlantCenterAnchorTarget = glm::vec3(0.0f);
                comp._gaitPlantPreviousDrift = 0.0f;
                comp._gaitPlantDriftRate = 0.0f;
                comp._gaitPlantRecoveryLogged = false;
                comp._gaitPlantCorrectionPeakRequested = 0.0f;
                comp._gaitPlantCorrectionPeakApplied = 0.0f;
                comp._gaitPlantCorrectionSaturated = false;
                comp._gaitPlantCorrectionRequested = 0.0f;
                comp._gaitPlantCorrectionApplied = 0.0f;
                comp._gaitPlantCorrectionAtLimit = false;
                comp._physicalStepTrajectoryT = 0.0f;
                comp._physicalStepTouchdownAccepted = false;
                comp._physicalStepTouchdownContactValid = false;
                comp._physicalStepPlantCenterTravel = 0.0f;
                comp._gaitNewSupportLoadLatched = false;
                comp._physicalStepMaxStanceDrift = 0.0f;
                comp._physicalStepMaxPlantDrift = 0.0f;
                comp._gaitStepMaxRelevantDrift = 0.0f;
                comp._gaitTakeoffContactRecoveryTime = 0.0f;
                comp._gaitTakeoffContactRecoveryActive = false;
                comp._gaitSwingRecontactTime = 0.0f;
                comp._gaitIkPlanHipValid = false;
                comp._gaitReachClampedStep = false;
                comp._gaitOldSupportDriftAllowanceLogged = false;
                comp._gaitCancelMode = 0;
                comp._gaitLandingVerificationPending = false;
                comp._gaitLandingStableTime = 0.0f;
                comp._physicalStepInitialTilt = tiltDeg;
                comp._physicalStepPeakTilt = tiltDeg;
                comp._physicalStepFinalTilt = tiltDeg;
                comp._physicalStepMaxMotorRatio = 0.0f;
                comp._physicalStepMotorSaturated = false;
                comp._physicalStepPlantPoseCaptured = false;
                comp._physicalStepPreviousSwingFootValid = false;
                comp._physicalStepPrevSwingContact = true;
                comp._supportTransferTransferT = 0.0f;
                comp._supportTransferHoldStableTime = 0.0f;
                comp._supportTransferContactLossTime = 0.0f;
                comp._supportTransferComError = 0.0f;
                comp._supportTransferComToOldSupport = 0.0f;
                comp._supportTransferComToNewSupport = 0.0f;
                comp._supportTransferTransferStartTarget =
                    comp._physicalStepSupportTarget;
                comp._supportTransferTransferEndTarget =
                    comp._physicalStepSupportTarget;
                if (continuousEnabled) {
                    const float minimumStep = glm::min(
                        comp.gaitMinStepLength, comp.gaitMaxStepLength);
                    const float maximumStep = glm::max(
                        comp.gaitMinStepLength, comp.gaitMaxStepLength);
                    const float baseTrackingReserve = 0.010f;
                    const float trackingReserve = glm::min(glm::max(
                        baseTrackingReserve,
                        comp._gaitSettledTrackingLoss + 0.003f), 0.040f);
                    const float minimumCommand = glm::min(
                        maximumStep, minimumStep + trackingReserve);
                    comp._gaitReachCommandCeiling = glm::clamp(
                        comp._gaitReachCommandCeiling,
                        minimumCommand, maximumStep);
                    const float speedFeedbackTarget = glm::clamp(
                        comp.gaitNominalAdvance + comp._gaitAdaptiveStrideOffset
                            + comp.gaitPlacementGain
                            * (continuousCommand.desiredSpeed
                               - comp._gaitMeasuredSpeed),
                        minimumCommand, maximumStep);
                    const float previousCommand =
                        comp._gaitCommandedStepLength;
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
                        comp._gaitReachCommandCeiling = glm::min(
                            comp._gaitReachCommandCeiling,
                            reachLimitedTarget);
                        comp._gaitReachClearSteps = 0;
                        nextCommand = glm::min(
                            nextCommand, comp._gaitReachCommandCeiling);
                    } else if (comp._gaitReachCommandCeiling
                               < maximumStep - 1e-4f) {
                        ++comp._gaitReachClearSteps;
                        const bool releaseCeiling =
                            comp._gaitReachClearSteps >= 3;
                        if (releaseCeiling) {
                            comp._gaitReachCommandCeiling = glm::min(
                                maximumStep,
                                comp._gaitReachCommandCeiling + 0.005f);
                            comp._gaitReachClearSteps = 0;
                        }
                        nextCommand = glm::min(
                            nextCommand, comp._gaitReachCommandCeiling);
                    } else {
                        comp._gaitReachCommandCeiling = maximumStep;
                        comp._gaitReachClearSteps = 0;
                    }
                    comp._gaitCommandedStepLength = glm::clamp(
                        nextCommand, minimumCommand, maximumStep);
                    ++comp._stepSequenceStepIndex;
                    comp._runtimeAutoRetryCount = 0;
                    comp._runtimeRecoveryStableTime = 0.0f;
                    comp._gaitStepStartTime = comp._gaitRunTime;
                    comp._gaitStepStartCom = rag._locomotionCOM;
                } else {
                    comp._stepSequenceStepIndex = 2;
                }
            } else if (comp._physicalStepPhaseTime >= glm::max(
                           comp.transferTimeout, 0.50f)) {
                if (continuousEnabled) {
                    spdlog::warn(
                        "[LocomotionGait] INTERSTEP_CHECK result=FAIL "
                        "contact=({},{}) speed=(L={:.3f},R={:.3f},COM={:.3f}) "
                        "tilt={:.1f}/{:.1f} tiltRate={:.3f}rad/s "
                        "heading={:+.1f}/{:.1f}deg ready={} "
                        "saturation=(support={},upright={},heading={},motor={}) "
                        "stable={:.3f}/{:.3f}s recenter=(t={:.2f},err={:.3f}m,ready={})",
                        swingContactNow ? "S" : "-",
                        stanceContactNow ? "T" : "-",
                        glm::length(leftVelocity),
                        glm::length(rightVelocity),
                        comp._supportTransferComHorizontalSpeed,
                        tiltDeg, interStepTiltLimit,
                        comp._gaitRootTiltRate,
                        comp._gaitHeadingErrorDeg, interStepHeadingLimit,
                        headingReady ? "yes" : "NO",
                        rag._locomotionSupportSaturated ? "YES" : "no",
                        rag._locomotionUprightSaturated ? "YES" : "no",
                        rag._locomotionHeadingSaturated ? "YES" : "no",
                        comp._physicalStepMotorSaturated ? "YES" : "no",
                        comp._stepSequenceInterStepStableTime,
                        cadenceInterStepTime,
                        comp._gaitInterStepRecenterT,
                        comp._gaitInterStepCenterError,
                        recenterReady ? "yes" : "NO");
                }
                abortSequence("inter-step handoff did not remain ready");
            }
        } else if (continuousEnabled && comp._physicalStepPhase == kStopping) {
            const float supportError = horizontalDistance(
                rag._locomotionCOM, comp._gaitStopEndTarget);
            const bool poseReleased = comp._physicalStepPhaseTime >= glm::max(
                    comp.gaitStopTime, 0.25f)
                && comp._gaitCrouchBlend <= 0.001f;
            if (poseReleased && !comp._gaitStopSettleReferenceValid) {
                // The walking-to-standing pose blend deliberately changes the staggered
                // stance geometry. Capture the physical soles once that command is complete,
                // then measure only continued motion from this settled-pose reference.
                comp._gaitStopSettleFootTargetL = leftFoot;
                comp._gaitStopSettleFootTargetR = rightFoot;
                comp._gaitStopSettleFootDriftL = 0.0f;
                comp._gaitStopSettleFootDriftR = 0.0f;
                comp._gaitStopMaxSettleFootDrift = 0.0f;
                comp._gaitStopSettleReferenceValid = true;
                comp._gaitStopStableTime = 0.0f;
            }
            const bool stopFeetSettled = comp._gaitStopSettleReferenceValid
                && comp._gaitStopSettleFootDriftL <= 0.040f
                && comp._gaitStopSettleFootDriftR <= 0.040f;
            const bool settled = poseReleased
                && supportError <= glm::max(comp.transferComTolerance, 0.04f)
                && comp._physicalStepContactL && comp._physicalStepContactR
                && glm::length(leftVelocity) < 0.15f
                && glm::length(rightVelocity) < 0.15f
                && comp._supportTransferComHorizontalSpeed < 0.15f
                && stopFeetSettled
                && tiltDeg < 30.0f
                && !rag._locomotionSupportSaturated
                && !comp._physicalStepMotorSaturated;
            comp._gaitStopStableTime = settled
                ? comp._gaitStopStableTime + dt : 0.0f;
            if (comp._gaitStopStableTime >= glm::max(
                    comp.gaitStopHoldTime, 0.25f)) {
                comp._physicalStepPhase = kReturnStand;
                comp._physicalStepPhaseTime = 0.0f;
                comp._gaitStopStableTime = 0.0f;
                rag.locomotionSupportTargetWeight = 0.0f;
            } else if (comp._physicalStepPhaseTime >= glm::max(
                           comp.gaitStopTime + comp.transferTimeout, 1.0f)) {
                spdlog::warn(
                    "[LocomotionGait] STOP_CHECK result=FAIL supportError={:.3f}/{:.3f}m "
                    "contact=({},{}) speed=(L={:.3f},R={:.3f},COM={:.3f}) "
                    "shutdownFeet=(transition=({:.3f},{:.3f},max={:.3f})m,"
                    "settleDrift=({:.3f},{:.3f},max={:.3f})/0.040m,"
                    "locks=({:.2f},{:.2f}),force=({:.0f},{:.0f})N) "
                    "poseReleased={} settleReference={} crouch={:.3f} tilt={:.1f} "
                    "saturation=(support={},motor={})",
                    supportError, glm::max(comp.transferComTolerance, 0.04f),
                    comp._physicalStepContactL ? "L" : "-",
                    comp._physicalStepContactR ? "R" : "-",
                    glm::length(leftVelocity),
                    glm::length(rightVelocity),
                    comp._supportTransferComHorizontalSpeed,
                    comp._gaitStopFootDriftL,
                    comp._gaitStopFootDriftR,
                    comp._gaitStopMaxFootDrift,
                    comp._gaitStopSettleFootDriftL,
                    comp._gaitStopSettleFootDriftR,
                    comp._gaitStopMaxSettleFootDrift,
                    rag.locomotionFootLockWeights[0],
                    rag.locomotionFootLockWeights[1],
                    rag._locomotionFootLockForce[0],
                    rag._locomotionFootLockForce[1],
                    poseReleased ? "yes" : "NO",
                    comp._gaitStopSettleReferenceValid ? "yes" : "NO",
                    comp._gaitCrouchBlend,
                    tiltDeg,
                    rag._locomotionSupportSaturated ? "YES" : "no",
                    comp._physicalStepMotorSaturated ? "YES" : "no");
                abortSequence("continuous-gait stop did not settle");
            }
        } else if (continuousEnabled && comp._physicalStepPhase == kReturnStand) {
            const bool locksOff = rag.locomotionFootLockWeights[0] <= 0.001f
                               && rag.locomotionFootLockWeights[1] <= 0.001f
                               && rag._locomotionFootLockForce[0] <= 0.5f
                               && rag._locomotionFootLockForce[1] <= 0.5f;
            const bool standingSettled = comp._physicalStepContactL && comp._physicalStepContactR
                && glm::length(leftVelocity) < 0.15f
                && glm::length(rightVelocity) < 0.15f
                && comp._supportTransferComHorizontalSpeed < 0.15f
                && tiltDeg < 15.0f
                && std::abs(comp._gaitHeadingErrorDeg)
                    <= glm::clamp(comp.gaitInterStepHeadingLimit, 2.0f, 20.0f)
                && !rag._locomotionSupportSaturated
                && !rag._locomotionUprightSaturated
                && rag.locomotionLiftBone < 0
                && rag._locomotionLiftForce <= 0.5f
                && locksOff
                && !comp._physicalStepMotorSaturated;
            comp._gaitStopStableTime = standingSettled
                ? comp._gaitStopStableTime + dt : 0.0f;
            if (comp._gaitStopStableTime >= 0.50f) {
                const int completed = comp._stepSequenceStepsCompleted;
                const int edgeTotal = comp._stepSequenceContactTransitionsL
                                    + comp._stepSequenceContactTransitionsR;
                const bool completedEdgeCountOk = edgeTotal == 2 * completed
                    && std::abs(comp._stepSequenceContactTransitionsL
                               - comp._stepSequenceContactTransitionsR) <= 2;
                const bool cancelledReturnEdgeCountOk =
                    comp._gaitCancelMode == 1
                    && edgeTotal == 2 * completed + 2
                    && std::abs(comp._stepSequenceContactTransitionsL
                              - comp._stepSequenceContactTransitionsR) <= 2;
                const bool edgeCountOk = completedEdgeCountOk
                    || cancelledReturnEdgeCountOk;
                const bool lengthConverged = completed < 3
                    || std::abs(comp._gaitLastSupportAdvance
                              - comp._gaitPreviousSupportAdvance) <= 0.030f;
                const bool periodConverged = completed < 3
                    || std::abs(comp._gaitLastStepPeriod
                              - comp._gaitPreviousStepPeriod) <= 0.75f;
                const float totalForward = glm::dot(
                    rag._locomotionCOM - comp._gaitStartCom,
                    comp._physicalStepForward);
                const bool currentStopSafe =
                    comp._gaitStopSettleReferenceValid
                    && comp._gaitStopMaxSettleFootDrift <= 0.040f;
                // Historical peaks, edge counts, and convergence remain regression
                // diagnostics. Active-phase guards already handle real loss of contact,
                // support, or upright stability; a safely settled recovery must not be
                // converted back into an abort by the error that requested recovery.
                const bool pass = currentStopSafe && standingSettled;
                const bool adaptiveRecovery =
                    comp._gaitAdaptiveStopRequested;
                const bool landingObjectiveRecovery =
                    comp._gaitLandingObjectiveStopRequested;
                comp._gaitRunning = false;
                if (pass) {
                    comp._physicalStepPhase = kIdle;
                    comp._physicalStepPhaseTime = 0.0f;
                    comp._physicalStepSupportSide = 0;
                    comp._gaitStopRequested = false;
                    comp._gaitAdaptiveStopRequested = false;
                    comp._gaitLandingObjectiveStopRequested = false;
                    comp._gaitRecoveryFailureSteps = 0;
                    comp._gaitStress *= 0.50f;
                    spdlog::info(
                        "[LocoRuntime] STOP_COMPLETE steps={} time={:.2f}s "
                        "forward={:.3f} finalTilt={:.1f} recovery={} "
                        "stopSettleDrift={:.3f}m diagnostics=(edges={},length={},"
                        "period={},historyDrift={:.3f}m,historyTilt={:.1f},"
                        "historyMotor={:.2f}) ready=IDLE",
                        completed, comp._gaitRunTime, totalForward, tiltDeg,
                        adaptiveRecovery ? "adaptive"
                            : (landingObjectiveRecovery
                                ? "landing-objective" : "commanded"),
                        comp._gaitStopMaxSettleFootDrift,
                        edgeCountOk ? "ok" : "mismatch",
                        lengthConverged ? "ok" : "variable",
                        periodConverged ? "ok" : "variable",
                        comp._gaitMaxDrift,
                        comp._gaitPeakTilt,
                        comp._gaitMaxMotorRatio);
                } else {
                    spdlog::warn(
                        "[LocomotionGait] STOP_COMPLETION_CHECK result=FAIL "
                        "standing={} settleReference={} stopSettleDrift={:.3f}/0.040m "
                        "history=(drift={:.3f}m,tilt={:.1f},motor={:.2f})",
                        standingSettled ? "yes" : "NO",
                        comp._gaitStopSettleReferenceValid ? "yes" : "NO",
                        comp._gaitStopMaxSettleFootDrift,
                        comp._gaitMaxDrift,
                        comp._gaitPeakTilt,
                        comp._gaitMaxMotorRatio);
                    abortSequence("continuous-gait completion check failed");
                }
            } else if (comp._physicalStepPhaseTime >= 3.0f) {
                abortSequence("return to standing did not settle");
            }
        } else if (comp._physicalStepPhase == kAbort
                   && std::abs(comp._physicalStepComCommand) < 0.01f
                   && std::abs(comp._physicalStepComLateral) < 0.01f
                   && (!transferEnabled || horizontalDistance(
                           rag._locomotionCOM, comp._physicalStepComBaseline) < 0.05f)
                   && comp._physicalStepPhaseTime >= glm::max(comp.weightShiftDuration, 0.01f)) {
            spdlog::warn(
                "[LocomotionStep] reset after aborted sequence support={} "
                "maxDrift=({:.3f},{:.3f}) peakTilt={:.1f}",
                comp._physicalStepSupportSide < 0 ? "LEFT" : "RIGHT",
                comp._physicalStepMaxStanceDrift, comp._physicalStepMaxPlantDrift,
                comp._physicalStepPeakTilt);
            comp._physicalStepPhase = kIdle;
            comp._physicalStepPhaseTime = 0.0f;
            comp._physicalStepSupportSide = 0;
            comp._physicalStepTouchdownAccepted = false;
            comp._physicalStepTouchdownContactValid = false;
        }

        comp._physicalStepPrevSwingContact = swingContactNow;

        if (comp.debug && phaseAtFrameStart != comp._physicalStepPhase) {
            auto phaseName = [&](int phase) {
                switch (phase) {
                    case kIdle:          return "IDLE";
                    case kWeightShift:   return "WEIGHT_SHIFT";
                    case kTakeoff:       return "TAKEOFF";
                    case kSwing:         return "SWING";
                    case kArrival:       return "ARRIVAL";
                    case kDescent:       return "DESCENT";
                    case kTouchdownWait: return "TOUCHDOWN_WAIT";
                    case kSettle:        return "SETTLE";
                    case kSupportReady:  return "SUPPORT_READY";
                    case kTransfer:      return "TRANSFER";
                    case kHold:          return "HOLD";
                    case kInterStep:     return "INTER_STEP";
                    case kComplete:      return "COMPLETE";
                    case kAbort:         return "ABORT";
                    case kStopping:      return "STOPPING";
                    case kReturnStand:   return "RETURN_STAND";
                    default:             return "UNKNOWN";
                }
            };
            auto cadenceBudget = [&](int phase) {
                switch (phase) {
                    case kWeightShift: return cadenceWeightShiftTime;
                    case kTakeoff:    return cadenceTakeoffTarget;
                    case kSwing:       return cadenceSwingTime;
                    case kArrival:     return cadenceArrivalSettleTime;
                    case kDescent:     return activeCadenceDescentTime();
                    case kTouchdownWait: return cadenceTouchdownTarget;
                    case kSettle:       return cadenceLandingVerifyTime;
                    case kSupportReady: return cadencePlantAcquireTime;
                    case kTransfer:     return cadenceTransferTime;
                    case kHold:         return cadenceTransferHoldTime;
                    case kInterStep:    return cadenceInterStepTime;
                    default:            return 0.0f;
                }
            };
            const float dwell = phaseTimeAtFrameStart + dt;
            const float budget = cadenceBudget(phaseAtFrameStart);
            const bool previousFootOwned = phaseAtFrameStart >= kTakeoff
                                        && phaseAtFrameStart <= kTouchdownWait;
            const bool currentFootOwned = comp._physicalStepPhase >= kTakeoff
                                       && comp._physicalStepPhase <= kTouchdownWait;
            const float footTargetStep = previousFootOwned && currentFootOwned
                ? glm::length(desiredFoot - previousDesiredFoot) : 0.0f;
            const float supportTargetStep = glm::length(
                supportTarget - previousSupportTarget);
            const float supportCurveT = comp._gaitSupportCurveActive
                ? comp._gaitSupportCurveTime
                    / glm::max(comp._gaitSupportCurveDuration, 0.01f)
                : -1.0f;
            const float rearAnchorError = stance->planted
                ? glm::length(glm::vec2(
                    stanceFoot.x - stance->plantFoot.x,
                    stanceFoot.z - stance->plantFoot.z))
                : 0.0f;
            spdlog::info(
                "[LocomotionPhase] {} -> {} dwell={:.3f}s budget={:.3f}s "
                "excess={:.3f}s targetStep=(foot={:.4f}m,support={:.4f}m) "
                "trajectory={:.2f} supportCurve={:.2f} supportSpeed={:.3f} "
                "newLoad={:.2f} drift=(rear={:.3f},plant={:.3f}) "
                "plantRate={:+.3f} rearAnchor={:.3f} soleError={:.1f}deg",
                phaseName(phaseAtFrameStart),
                phaseName(comp._physicalStepPhase),
                dwell, budget, glm::max(dwell - budget, 0.0f),
                footTargetStep, supportTargetStep,
                comp._physicalStepTrajectoryT,
                supportCurveT,
                glm::length(glm::vec2(supportVelocity.x, supportVelocity.z)),
                comp._gaitNewSupportLoad,
                comp._physicalStepStanceDrift,
                comp._physicalStepPlantDrift,
                comp._gaitPlantDriftRate,
                rearAnchorError,
                comp._gaitSoleAngularErrorDeg);
        }

        if (comp.debug) {
            DebugDraw::Sphere(comp._physicalStepComBaseline, 0.025f, {0.2f, 0.7f, 1.0f});
            DebugDraw::Sphere(supportTarget, 0.035f, {1.0f, 0.7f, 0.1f});
            DebugDraw::Line(comp._physicalStepComBaseline, supportTarget, {1.0f, 0.7f, 0.1f});
            DebugDraw::Sphere(comp._physicalStepFoothold, 0.045f, {0.2f, 1.0f, 0.2f});
            DebugDraw::Sphere(desiredFoot, 0.035f, {1.0f, 0.4f, 0.1f});
            DebugDraw::Line(swingFoot, desiredFoot, {1.0f, 0.4f, 0.1f});
            if (comp._gaitFkValid
                && comp._physicalStepPhase >= kTakeoff
                && comp._physicalStepPhase <= kTouchdownWait) {
                // Magenta is the sole position encoded by the admitted joint command.
                // The two error segments make command-space closure (orange->magenta)
                // distinct from powered-ragdoll tracking (magenta->physical cyan).
                DebugDraw::Sphere(
                    comp._gaitFkCommandPosition, 0.022f,
                    {1.0f, 0.15f, 0.95f});
                DebugDraw::Line(
                    comp._gaitFkDesiredPosition,
                    comp._gaitFkCommandPosition,
                    {1.0f, 0.15f, 0.95f});
                DebugDraw::Sphere(
                    comp._gaitFkPhysicalPosition, 0.018f,
                    {0.15f, 0.95f, 1.0f});
                DebugDraw::Line(
                    comp._gaitFkCommandPosition,
                    comp._gaitFkPhysicalPosition,
                    {0.15f, 0.95f, 1.0f});
            }
            if (comp._gaitTurnPlan.candidateEvaluated) {
                const auto& turnPlan = comp._gaitTurnPlan;
                const glm::vec3 startOrigin =
                    turnPlan.planOrigin + glm::vec3(0.0f, 0.025f, 0.0f);
                const glm::vec3 endOrigin =
                    turnPlan.planOrigin + glm::vec3(0.0f, 0.050f, 0.0f);
                DebugDraw::Line(
                    startOrigin,
                    startOrigin + turnPlan.activeStartForward * 0.18f,
                    {0.1f, 0.8f, 1.0f});
                DebugDraw::Line(
                    startOrigin,
                    startOrigin + turnPlan.activeStartRight * 0.12f,
                    {0.1f, 0.45f, 1.0f});
                DebugDraw::Line(
                    endOrigin,
                    endOrigin + turnPlan.activeEndForward * 0.18f,
                    {1.0f, 0.2f, 0.8f});
                DebugDraw::Line(
                    endOrigin,
                    endOrigin + turnPlan.activeEndRight * 0.12f,
                    {1.0f, 0.55f, 0.8f});

                const glm::vec3 candidateOrigin =
                    turnPlan.candidateFootPosition
                    + glm::vec3(0.0f, 0.015f, 0.0f);
                const glm::vec3 candidateRight =
                    turnPlan.candidateFootRotation * glm::vec3(1.0f, 0.0f, 0.0f);
                const glm::vec3 candidateUp =
                    turnPlan.candidateFootRotation * glm::vec3(0.0f, 1.0f, 0.0f);
                const glm::vec3 candidateForward =
                    turnPlan.candidateFootRotation * glm::vec3(0.0f, 0.0f, -1.0f);
                DebugDraw::Sphere(
                    candidateOrigin, 0.025f, {1.0f, 0.75f, 0.1f});
                DebugDraw::Line(
                    candidateOrigin, candidateOrigin + candidateRight * 0.08f,
                    {1.0f, 0.25f, 0.2f});
                DebugDraw::Line(
                    candidateOrigin, candidateOrigin + candidateUp * 0.06f,
                    {0.2f, 1.0f, 0.3f});
                DebugDraw::Line(
                    candidateOrigin, candidateOrigin + candidateForward * 0.10f,
                    {0.2f, 0.55f, 1.0f});
            }
            if (comp._physicalStepPhase >= kSwing && comp._physicalStepPhase <= kComplete) {
                glm::vec3 previous = trajectoryPoint(0.0f);
                for (int i = 1; i <= 12; ++i) {
                    const glm::vec3 next = trajectoryPoint(static_cast<float>(i) / 12.0f);
                    DebugDraw::Line(previous, next, {0.8f, 0.3f, 1.0f});
                    previous = next;
                }
                DebugDraw::Line(hoverTarget, comp._physicalStepFoothold,
                                {0.2f, 0.9f, 1.0f});
            }
        }
    }
