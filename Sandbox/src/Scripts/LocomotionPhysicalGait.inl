// Included in LocamotionControllerSystem's private section. This file owns the
// physical step state machine, foot planning, and constrained ragdoll leg solve.
    static void ResetPhysicalGait(Comp& c)
    {
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
        c._gaitRunTime = 0.0f;
        c._gaitStepStartTime = 0.0f;
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
        c._runtimeTurnActive = false;
        c._runtimeTurnElapsed = 0.0f;
        c._runtimeTurnDuration = 0.0f;
        c._runtimeTurnTotalYaw = 0.0f;
        c._runtimeTurnAppliedYaw = 0.0f;
        c._runtimeDesiredForward = glm::vec3(0.0f, 0.0f, -1.0f);
        c._runtimeTurnTargetForward = glm::vec3(0.0f, 0.0f, -1.0f);
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
        constexpr int kTransfer = 8;
        constexpr int kHold = 9;
        constexpr int kInterStep = 10;
        constexpr int kComplete = 11;
        constexpr int kAbort = 12;
        constexpr int kStopping = 13;
        constexpr int kReturnStand = 14;
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

        // Cadence is a whole-cycle property. Compressing just SWING made the foot fast
        // while leaving several seconds of serialized validation around it, so derive one
        // scale for every cadence-bearing phase. Safety timeouts remain unscaled.
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
                         / glm::max(configuredStepBudget, 0.01f), 0.20f, 1.0f)
            : 1.0f;
        comp._gaitPhaseTimeScale = cadenceScale;
        auto cadenceTime = [&](float configured, float floor) {
            return glm::max(configured * cadenceScale, floor);
        };
        const float cadenceWeightShiftTime = cadenceTime(comp.weightShiftDuration, 0.20f);
        const float cadenceSwingTime = cadenceTime(comp.swingDuration, 0.28f);
        const float cadenceArrivalSettleTime = cadenceTime(comp.arrivalSettleDuration, 0.03f);
        // Descent is also the final task-space convergence window. Keep enough physical
        // time for the measured sole to catch the target after a fast swing.
        const float cadenceDescentTime = cadenceTime(comp.descentDuration, 0.18f);
        const float cadencePlantAcquireTime = cadenceTime(comp.plantAcquireDuration, 0.04f);
        const float cadenceLandingVerifyTime = cadenceTime(comp.contactSettleDuration, 0.08f);
        const float cadenceTransferTime = cadenceTime(comp.transferDuration, 0.14f);
        const float cadenceTransferHoldTime = cadenceTime(comp.transferHoldDuration, 0.06f);
        const float cadenceInterStepTime = cadenceTime(comp.interStepDuration, 0.04f);

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
            }
        }
        if (!comp._physicalStepBaselineValid) return;

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
        }

        auto commitRuntimeStandingHeading = [&](glm::vec3 forward) {
            setGaitHeading(forward);
            const float targetYaw = std::atan2(
                -comp._physicalStepForward.x, -comp._physicalStepForward.z);
            // This is the source used by the general standing controller at the start
            // of the next update, so the eased physical turn and its target cannot fight.
            comp._yaw = targetYaw - glm::radians(comp.facingOffsetDeg);
        };
        auto rotateRuntimeReferences = [&](const glm::vec3& pivot,
                                           const glm::quat& turn) {
            auto rotatePoint = [&](glm::vec3 point) {
                return pivot + turn * (point - pivot);
            };
            comp._physicalStepFootBaselineL = rotatePoint(comp._physicalStepFootBaselineL);
            comp._physicalStepFootBaselineR = rotatePoint(comp._physicalStepFootBaselineR);
            comp._physicalStepComBaseline = rotatePoint(comp._physicalStepComBaseline);
            comp._physicalStepSupportTarget = rotatePoint(comp._physicalStepSupportTarget);
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
                beginRuntimeTurn(comp._physicalStepForward, desiredForward, "RETARGET");
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
                    turn * comp._physicalStepForward);
                comp._runtimeTurnAppliedYaw = desiredAppliedYaw;
            }

            comp._physicalStepPreviousSwingFootValid = false;
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
                    comp._physicalStepForward.x, comp._physicalStepForward.z);
            }
        };

        if (comp._physicalStepPhase == kIdle) {
            if (gameplayCommand && comp._runtimeTurnActive) {
                if (!continuousCommand.startRequested) {
                    comp._runtimeTurnActive = false;
                    comp._runtimeDesiredForward = comp._physicalStepForward;
                    spdlog::info(
                        "[LocoRuntime] TURN_BLEND CANCELED applied={:+.1f}deg "
                        "heading=({:+.2f},{:+.2f}) reason=input-release",
                        glm::degrees(comp._runtimeTurnAppliedYaw),
                        comp._physicalStepForward.x, comp._physicalStepForward.z);
                } else {
                    advanceRuntimeTurn(continuousCommand.desiredForward);
                }
                return;
            }
            const bool startLeftSupport = continuousCommand.startRequested
                && continuousCommand.initialSupportSide < 0;
            const bool startRightSupport = continuousCommand.startRequested
                && continuousCommand.initialSupportSide > 0;
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

                comp._physicalStepFootBaselineL = leftFoot;
                comp._physicalStepFootBaselineR = rightFoot;
                comp._physicalStepComBaseline = rag._locomotionCOM;
                setGaitHeading(desiredForward);
                comp._physicalStepSupportTarget = comp._physicalStepComBaseline;
                comp._physicalStepSupportSide = startLeftSupport ? -1 : 1;
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
                    comp._gaitRecoveryFailureSteps = 0;
                    comp._gaitRunTime = 0.0f;
                    comp._gaitStepStartTime = 0.0f;
                    comp._gaitLastStepPeriod = 0.0f;
                    comp._gaitPreviousStepPeriod = 0.0f;
                    comp._gaitMeasuredSpeed = 0.0f;
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
        if (continuousEnabled && comp._physicalStepPhase == kSettle
            && comp._physicalStepTouchdownAccepted) {
            // A credible touchdown begins accepting weight immediately while plant
            // acquisition verifies the contact. This short pre-transfer uses immutable
            // endpoints and hands off continuously to the full support transfer.
            const float touchdownTransferT = glm::clamp(
                comp._physicalStepPhaseTime
                    / glm::max(cadencePlantAcquireTime, 0.04f),
                0.0f, 1.0f);
            supportTarget = glm::mix(
                comp._supportTransferTransferStartTarget,
                comp._supportTransferTransferEndTarget,
                smoothstep(touchdownTransferT));
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
        // stop-then-burst. Pre-contact motion is deliberately limited to 20% of the
        // eventual span. Credible contact advances only to a partial-load target while
        // the sole settles; plant acquisition C1-reseeds the curve to the full handoff.
        const float configuredSupportMaxSpeed = glm::clamp(
            comp.gaitSupportMaxSpeed, 0.01f, 1.0f);
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
                0.70f, 0.98f);
            comp._gaitSupportCurveActive = true;
            comp._gaitSupportCurveStep = comp._stepSequenceStepIndex;
            comp._gaitSupportCurveTime = 0.0f;
            comp._gaitSupportCurveDuration = glm::max(
                cadenceDescentTime, 0.18f);
            comp._gaitSupportCurveStart = comp._physicalStepSupportTarget;
            const glm::vec3 fullTransferTarget = glm::mix(
                oldSupport, comp._physicalStepFoothold, transferFraction);
            constexpr float kPreContactSupportFraction = 0.20f;
            comp._gaitSupportCurveEnd = glm::mix(
                comp._gaitSupportCurveStart, fullTransferTarget,
                kPreContactSupportFraction);
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

            const float forwardDistance = glm::max(glm::dot(
                comp._gaitSupportCurveEnd - comp._gaitSupportCurveStart,
                comp._physicalStepForward), 0.0f);
            const float outgoingSpeed = glm::clamp(
                0.25f * forwardDistance / comp._gaitSupportCurveDuration,
                0.03f, 0.10f);
            comp._gaitSupportCurveEndVelocity =
                comp._physicalStepForward * outgoingSpeed;
            startedSupportCurveThisFrame = true;
            spdlog::info(
                "[LocomotionGait] SUPPORT_CURVE_BEGIN step={} duration={:.3f}s "
                "mode=preload fraction=0.20 load={:.2f} "
                "limits=(speed={:.2f}mps,accel=1.75mps2) "
                "velocity=({:+.3f},{:+.3f})->({:+.3f},{:+.3f})",
                comp._stepSequenceStepIndex,
                comp._gaitSupportCurveDuration,
                comp._gaitNewSupportLoad,
                configuredSupportMaxSpeed,
                comp._gaitSupportCurveStartVelocity.x,
                comp._gaitSupportCurveStartVelocity.z,
                comp._gaitSupportCurveEndVelocity.x,
                comp._gaitSupportCurveEndVelocity.z);
        }
        const bool supportCurveOwnsWalkingTarget = continuousEnabled
            && comp._gaitRunning && comp._gaitSupportCurveActive
            && comp._physicalStepPhase >= kWeightShift
            && comp._physicalStepPhase <= kInterStep;
        if (supportCurveOwnsWalkingTarget) {
            if (!startedSupportCurveThisFrame)
                comp._gaitSupportCurveTime += dt;
            const float duration = glm::max(
                comp._gaitSupportCurveDuration, 0.01f);
            const float curveT = glm::clamp(
                comp._gaitSupportCurveTime / duration, 0.0f, 1.0f);
            const float t2 = curveT * curveT;
            const float t3 = t2 * curveT;
            const float h00 = 2.0f * t3 - 3.0f * t2 + 1.0f;
            const float h10 = t3 - 2.0f * t2 + curveT;
            const float h01 = -2.0f * t3 + 3.0f * t2;
            const float h11 = t3 - t2;
            glm::vec3 curveTarget = h00 * comp._gaitSupportCurveStart
                + h10 * duration * comp._gaitSupportCurveStartVelocity
                + h01 * comp._gaitSupportCurveEnd
                + h11 * duration * comp._gaitSupportCurveEndVelocity;

            const float dh00 = 6.0f * t2 - 6.0f * curveT;
            const float dh10 = 3.0f * t2 - 4.0f * curveT + 1.0f;
            const float dh01 = -6.0f * t2 + 6.0f * curveT;
            const float dh11 = 3.0f * t2 - 2.0f * curveT;
            glm::vec3 curveVelocity =
                (dh00 * comp._gaitSupportCurveStart
                 + dh01 * comp._gaitSupportCurveEnd) / duration
                + dh10 * comp._gaitSupportCurveStartVelocity
                + dh11 * comp._gaitSupportCurveEndVelocity;

            if (comp._gaitSupportCurveTime > duration) {
                // Preserve forward momentum after the lateral transfer finishes. The next
                // late-swing curve captures this exact position and velocity, so neither
                // the pelvis nor its feed-forward command stops between steps.
                const float cruiseTime = comp._gaitSupportCurveTime - duration;
                curveTarget = comp._gaitSupportCurveEnd
                    + comp._gaitSupportCurveEndVelocity * cruiseTime;
                curveVelocity = comp._gaitSupportCurveEndVelocity;
            }
            curveTarget.y = comp._gaitSupportCurveStart.y;
            curveVelocity.y = 0.0f;

            // Hermite endpoint tangents do not bound the polynomial's middle velocity.
            // Track the curve through an explicit command-space speed and acceleration
            // limiter so a large support span cannot yank a planted rear foot.
            constexpr float kSupportPositionGain = 4.0f;
            glm::vec3 desiredVelocity = curveVelocity
                + (curveTarget - comp._physicalStepSupportTarget)
                    * kSupportPositionGain;
            desiredVelocity.y = 0.0f;
            const bool acquiringPlant = comp._physicalStepPhase == kSettle
                && comp._physicalStepTouchdownAccepted
                && !comp._physicalStepPlantPoseCaptured;
            float maximumSupportSpeed = configuredSupportMaxSpeed;
            float maximumSupportAcceleration = 1.75f;
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
                const float forwardSpeed = glm::clamp(glm::dot(
                    desiredVelocity, comp._physicalStepForward),
                    -0.08f, 0.10f);
                const float lateralLimit = glm::mix(
                    0.10f, 0.03f, driftPressure);
                const float lateralSpeed = glm::clamp(glm::dot(
                    desiredVelocity, comp._physicalStepRight),
                    -lateralLimit, lateralLimit);
                desiredVelocity = comp._physicalStepForward * forwardSpeed
                    + comp._physicalStepRight * lateralSpeed;
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
                    0.85f, 1.75f, transferRamp);
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
        const bool swingContactNow = FootContact(
            rag, swing->footIdx, &contactNormal, &contactPoint);
        const bool stanceContactNow = FootContact(rag, stance->footIdx);
        glm::vec3 supportSpan = swingFoot - stanceFoot;
        supportSpan.y = 0.0f;
        const float supportSpanLengthSq = glm::dot(supportSpan, supportSpan);
        comp._gaitNewSupportLoad = supportSpanLengthSq > 1e-6f
            ? glm::clamp(glm::dot(
                glm::vec3(rag._locomotionCOM.x - stanceFoot.x, 0.0f,
                          rag._locomotionCOM.z - stanceFoot.z),
                supportSpan) / supportSpanLengthSq, 0.0f, 1.0f)
            : 0.5f;
        constexpr float kNewSupportLoadAcquireThreshold = 0.68f;
        constexpr float kNewSupportLoadReleaseThreshold = 0.64f;
        const bool loadHandoffPhase = comp._physicalStepPhase >= kTransfer
                                   && comp._physicalStepPhase <= kInterStep;
        if (!continuousEnabled || !loadHandoffPhase) {
            comp._gaitNewSupportLoadLatched = false;
        } else if (!comp._gaitNewSupportLoadLatched
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
                   && comp._gaitNewSupportLoad
                        < kNewSupportLoadReleaseThreshold) {
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
        bool swingAngularVelocityOk = false;
        const glm::vec3 swingAngularVelocity =
            Physics::GetRagdollBoneAngularVelocity(
                rag, swing->footIdx, &swingAngularVelocityOk);
        comp._physicalStepPlantAngularSpeed = swingAngularVelocityOk
            ? glm::length(swingAngularVelocity) : 0.0f;
        comp._physicalStepFootUpY = swingRotationOk
            ? (swingRotation * glm::vec3(0.0f, 1.0f, 0.0f)).y : 0.0f;
        comp._physicalStepContactPoint = contactPoint;
        comp._physicalStepContactLocal = swingContactNow && swingRotationOk
            ? glm::conjugate(swingRotation) * (contactPoint - swingFoot)
            : glm::vec3(0.0f);
        if (comp._physicalStepTouchdownAccepted
            && comp._physicalStepTouchdownContactValid
            && swingContactNow && swingRotationOk) {
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
        comp._physicalStepForwardTravel = glm::dot(
            swingFoot - comp._physicalStepSwingStart, comp._physicalStepForward);
        comp._gaitAchievedSupportAdvance = continuousEnabled
            ? glm::dot(swingFoot - stanceFoot, comp._physicalStepForward) : 0.0f;
        if (comp._physicalStepPhase >= kTakeoff) {
            const glm::vec3 targetDelta = swingFoot - comp._physicalStepFoothold;
            comp._physicalStepForwardTargetError = glm::dot(
                comp._physicalStepFoothold - swingFoot, comp._physicalStepForward);
            comp._physicalStepLateralTargetError = glm::dot(
                targetDelta, comp._physicalStepRight);
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
            leg.commandValid = true;
            leg.plantSolveValid = true;
        };

        auto captureSwing = [&]() {
            comp._physicalStepSwingStart = swingFoot;
            comp._physicalStepArcStart = swingFoot;
            comp._physicalStepDesiredFoot = swingFoot;
            captureLegSolveReference(*swing, swingFoot);
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
            leg.commandValid = true;
        };

        // The foothold is a target for the sole body, while the two-bone solve ends at
        // the ankle. Keep their separation in sole-local space. The settled sole reference
        // is heading-local: pitch/roll remain ground aligned while yaw follows the latched
        // gait frame. The per-step planted rotation is only the continuity endpoint.
        auto nominalFootWorldRotation = [&](const Leg& leg) {
            return continuousEnabled && leg.groundReferenceFootRotationValid
                ? glm::normalize(comp._gaitHeadingTargetRot
                    * leg.groundReferenceFootHeadingLocalRotation)
                : glm::normalize(leg.plantedFootWorldRotation);
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
        auto nominalAnkleFromFootWorld = [&]() {
            return ankleFromFootWorld(*swing, nominalFootWorldRotation(*swing));
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

        auto planFoothold = [&]() {
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
            const float placementDistance = continuousEnabled
                ? glm::clamp(comp._gaitCommandedStepLength,
                    minimumCommand, maximumAdvance)
                : glm::clamp(comp.stepLength, 0.15f, 0.25f);
            glm::vec3 requestedTarget;
            if (continuousEnabled) {
                // A limit cycle places the next support relative to the current support,
                // not relative to where this swing foot happened to land one cycle ago.
                // Gameplay turns give each anatomical foot a signed lane in the new frame;
                // a raw projection can change sign during a 90/180-degree reframe and ask
                // the swing leg to cross through the planted leg.
                float lateralLane = glm::dot(
                    comp._physicalStepSwingStart - stanceFoot, comp._physicalStepRight);
                if (gameplayCommand) {
                    constexpr float kMinimumRuntimeLane = 0.10f;
                    const float laneMagnitude = glm::clamp(
                        std::abs(lateralLane), kMinimumRuntimeLane, 0.24f);
                    const float swingSide = swing == &comp._legL ? -1.0f : 1.0f;
                    lateralLane = swingSide * laneMagnitude;
                    lateralLane += glm::clamp(
                        comp._gaitAdaptiveLateralOffset, -0.015f, 0.015f);
                }
                requestedTarget = stanceFoot
                    + comp._physicalStepForward * placementDistance
                    + comp._physicalStepRight * lateralLane;
            } else {
                requestedTarget = comp._physicalStepSwingStart
                    + comp._physicalStepForward * placementDistance;
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

            const glm::vec3 hip = physicalPosition(swing->hipIdx);
            glm::vec3 predictedLandingHip = hip;
            if (continuousEnabled) {
                // Weight shift waits until the COM is within 1 cm of its stance target.
                // Include that small remaining horizontal correction when testing the
                // grounded foothold, so planning and landing use the same expected pelvis
                // position instead of the pre-takeoff snapshot.
                glm::vec3 remainingComShift =
                    comp._physicalStepSupportTarget - rag._locomotionCOM;
                remainingComShift.y = 0.0f;
                if (const float remaining = glm::length(remainingComShift);
                    remaining > 0.015f && remaining > 1e-5f)
                    remainingComShift *= 0.015f / remaining;
                predictedLandingHip += remainingComShift;
            }
            if (continuousEnabled) {
                comp._gaitIkPlanHip = predictedLandingHip;
                comp._gaitIkPlanHipValid = true;
            }
            const float legLength = glm::length(skeleton.bones[swing->kneeIdx].localT)
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

            auto groundedCandidate = [&](float horizontalScale) {
                glm::vec3 candidate = glm::mix(
                    comp._physicalStepSwingStart, requestedTarget,
                    glm::clamp(horizontalScale, 0.0f, 1.0f));
                const HitResult ground = Physics::Raycast(
                    candidate + glm::vec3(0, 0.35f, 0),
                    glm::vec3(0, -1, 0), 1.0f, entity, false);
                candidate.y = ground.hit
                    ? ground.point.y + soleCenterOffset
                    : glm::mix(comp._physicalStepSwingStart.y, requestedTarget.y,
                               glm::clamp(horizontalScale, 0.0f, 1.0f));
                return candidate;
            };
            auto targetReach = [&](const glm::vec3& candidate) {
                return glm::length(candidate + nominalAnkleFromFootWorld()
                    - predictedLandingHip);
            };

            glm::vec3 target = requestedTarget;
            const float requestedReach = targetReach(requestedTarget);
            bool reachClamped = requestedReach > comp._physicalStepReachLimit;
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
            comp._physicalStepFoothold = target;
            const float requestedForward = glm::dot(
                requestedTarget - comp._physicalStepSwingStart, comp._physicalStepForward);
            const float plannedForward = glm::dot(
                target - comp._physicalStepSwingStart, comp._physicalStepForward);
            const float requestedSupportAdvance = glm::dot(
                requestedTarget - stanceFoot, comp._physicalStepForward);
            const float plannedSupportAdvance = glm::dot(
                target - stanceFoot, comp._physicalStepForward);
            comp._gaitPlannedSupportAdvance = continuousEnabled
                ? plannedSupportAdvance : 0.0f;
            if (continuousEnabled)
                comp._gaitReachClampedStep = reachClamped;
            if (continuousEnabled) {
                const glm::vec3 footholdDelta = target - comp._physicalStepSwingStart;
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
                return footholdAccepted;
            }
            return plannedForward >= 0.149f;
        };

        auto abortSequence = [&](const char* reason) {
            if (comp._physicalStepPhase == kAbort || comp._physicalStepPhase == kIdle) return;
            const int abortedPhase = comp._physicalStepPhase;
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
                comp._gaitRunning = false;
                comp._gaitContinuousCycle = false;
                comp._gaitBypassWeightShift = false;
                comp._gaitNewSupportLoad = 0.0f;
                comp._gaitSupportCurveActive = false;
                comp._gaitSupportCurveStep = -1;
                comp._gaitSupportCommandVelocity = glm::vec3(0.0f);
            }
            if (gameplayCommand) {
                constexpr int kMaximumAutomaticRetries = 2;
                constexpr float kAutomaticRetryCooldown = 0.35f;
                ++comp._runtimeAutoRetryCount;
                comp._runtimeRecoveryCooldown = kAutomaticRetryCooldown;
                comp._runtimeRecoveryStableTime = 0.0f;
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
            spdlog::warn(
                "[LocomotionStep] ABORT {} clear={:.3f} forward={:.3f} contact={} "
                "target=(h={:.3f},fwd={:+.3f},lat={:+.3f},y={:.3f}) "
                "normalY={:.2f} vy=(api={:+.3f},fd={:+.3f}) upY={:+.2f} "
                "contactLocal=({:+.3f},{:+.3f},{:+.3f}) "
                "drift=({:.3f},{:.3f}) tilt={:.1f}",
                reason, comp._physicalStepClearance, comp._physicalStepForwardTravel,
                swingContactNow ? "yes" : "no",
                comp._physicalStepHorizontalTargetError,
                comp._physicalStepForwardTargetError,
                comp._physicalStepLateralTargetError,
                comp._physicalStepVerticalTargetError,
                contactNormal.y, swingVelocity.y,
                comp._physicalStepMeasuredVelocity.y, comp._physicalStepFootUpY,
                comp._physicalStepContactLocal.x, comp._physicalStepContactLocal.y,
                comp._physicalStepContactLocal.z, comp._physicalStepStanceDrift,
                comp._physicalStepPlantDrift, tiltDeg);
            if (continuousEnabled && comp._gaitIkPlanHipValid
                && abortedPhase >= kTakeoff && abortedPhase <= kSettle) {
                spdlog::warn(
                    "[LocomotionGait] IK_ABORT phase={} "
                    "reach=(requested={:.3f},clamped={:.3f},max={:.3f},physical={:.3f}) "
                    "reachShortfall=({:.3f}m,fwd={:+.3f}m) "
                    "hipMove=(fwd={:+.3f},lat={:+.3f},y={:+.3f})m "
                    "hipEnvelopeClamp={:.1f}deg commandLag=(hip={:.1f},knee={:.1f})deg "
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
                abortSequence(
                    "continuous handoff foothold lacked tracking reserve");
            }
        } else if (comp._physicalStepPhase == kWeightShift
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
                abortSequence(continuousEnabled
                    ? "latched foothold lacked support-advance tracking reserve"
                    : "latched foothold fell below 15 cm after reach clamp");
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
                comp._gaitCancelMode = 1;
                comp._physicalStepArcStart = swingFoot;
                comp._physicalStepFoothold = comp._physicalStepSwingStart;
                comp._gaitPlannedSupportAdvance = 0.0f;
                comp._physicalStepPhase = kSwing;
                comp._physicalStepPhaseTime = 0.0f;
                comp._physicalStepTrajectoryT = 0.0f;
                comp._physicalStepArrivalStableTime = 0.0f;
                comp._gaitSwingRecontactTime = 0.0f;
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
            }
        }

        const float takeoffHeight = glm::clamp(
            comp.takeoffHeight, 0.040f,
            glm::max(comp.swingHeight, 0.041f));
        glm::vec3 desiredFoot = comp._physicalStepSwingStart;
        if (comp._physicalStepPhase == kTakeoff) {
            desiredFoot.y += takeoffHeight;
            const float releaseClearance = glm::max(0.040f, takeoffHeight * 0.75f);
            const bool recoverableTakeoffContact = continuousEnabled
                && swingContactNow && comp._physicalStepClearance >= 0.040f;
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
                desiredFoot.y += 0.025f;
            }
            const bool airborneEvidence = !swingContactNow
                && comp._physicalStepClearance >= releaseClearance;
            comp._physicalStepAirborneTime = airborneEvidence
                ? comp._physicalStepAirborneTime + dt : 0.0f;
            const float requiredAirborneTime = continuousEnabled
                ? 0.025f : 0.05f;
            if (comp._physicalStepAirborneTime >= requiredAirborneTime) {
                comp._physicalStepArcStart = desiredFoot;
                comp._physicalStepPhase = kSwing;
                comp._physicalStepPhaseTime = 0.0f;
                comp._physicalStepTrajectoryT = 0.0f;
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
        const glm::vec3 hoverTarget = comp._physicalStepFoothold
            + glm::vec3(0.0f, glm::max(comp.arrivalHeight, 0.03f), 0.0f);
        auto trajectoryPoint = [&](float t) {
            t = glm::clamp(t, 0.0f, 1.0f);
            // Continue advancing horizontally during descent. Completing the horizontal
            // path at ARRIVAL made the foot stop in the air and then drop vertically.
            const float horizontalT = smoothstep(t);
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
            } else {
                point.y = glm::mix(
                    hoverTarget.y, comp._physicalStepFoothold.y,
                    smoothstep((t - kSwingArrivalT)
                               / (1.0f - kSwingArrivalT)));
            }
            return point;
        };

        const float arrivalTolerance = glm::max(
            comp.arrivalTolerance,
            continuousEnabled ? 0.025f : 0.01f);
        constexpr float kSoleArrivalToleranceDeg = 10.0f;
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
                    - nominalFootWorldRotation(*swing)
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
                "centerComp={:.3f}m sole={:.1f}deg",
                comp._stepSequenceStepIndex,
                comp._physicalStepSupportSide < 0 ? "RIGHT" : "LEFT",
                comp._physicalStepTouchdownContactValid ? "yes" : "NO",
                comp._physicalStepTouchdownContactLocal.x,
                comp._physicalStepTouchdownContactLocal.y,
                comp._physicalStepTouchdownContactLocal.z,
                pivotCenterShift,
                comp._gaitSoleAngularErrorDeg);
            comp._supportTransferTransferStartTarget =
                comp._physicalStepSupportTarget;
            constexpr float kPlantAcquireSupportFraction = 0.20f;
            const float touchdownSupportFraction = continuousEnabled
                ? kPlantAcquireSupportFraction : 0.35f;
            comp._supportTransferTransferEndTarget = glm::mix(
                comp._physicalStepSupportTarget, swingFoot,
                touchdownSupportFraction);
            comp._supportTransferTransferEndTarget.y =
                comp._physicalStepSupportTarget.y;
            if (continuousEnabled && comp._gaitSupportCurveActive) {
                // Replace the predicted foothold with a partial-load target around the
                // actual collision-supported plant. Preserve current position and velocity,
                // but do not pull the COM through the full handoff until the sole has proved
                // stable. Plant acquisition performs the second C1 re-seed.
                glm::vec3 incomingVelocity =
                    comp._gaitSupportCommandVelocity;
                incomingVelocity.y = 0.0f;
                comp._gaitSupportCurveStart =
                    comp._physicalStepSupportTarget;
                comp._gaitSupportCurveStartVelocity = incomingVelocity;
                comp._gaitSupportCurveEnd =
                    comp._supportTransferTransferEndTarget;
                comp._gaitSupportCurveEnd.y =
                    comp._gaitSupportCurveStart.y;
                comp._gaitSupportCurveTime = 0.0f;
                comp._gaitSupportCurveDuration = glm::max(
                    cadencePlantAcquireTime + cadenceTransferTime * 1.50f,
                    0.45f);
                const float remainingForward = glm::max(glm::dot(
                    comp._gaitSupportCurveEnd - comp._gaitSupportCurveStart,
                    comp._physicalStepForward), 0.0f);
                const float outgoingSpeed = glm::clamp(
                    0.25f * remainingForward
                        / comp._gaitSupportCurveDuration,
                    0.03f, 0.10f);
                comp._gaitSupportCurveEndVelocity =
                    comp._physicalStepForward * outgoingSpeed;
                spdlog::info(
                    "[LocomotionGait] SUPPORT_CURVE_CONTACT step={} "
                    "mode=plant-acquire fraction={:.2f} duration={:.3f}s "
                    "incomingSpeed={:.3f} load={:.2f}",
                    comp._stepSequenceStepIndex,
                    kPlantAcquireSupportFraction,
                    comp._gaitSupportCurveDuration,
                    glm::length(incomingVelocity),
                    comp._gaitNewSupportLoad);
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
            // Contact can arrive a few frames early as the support-relative target settles.
            // Once descent has genuinely begun, a near-ground, accurately tracked, slow,
            // upward-facing contact is stronger evidence than a brittle time boundary.
            const float minimumProgress = continuousEnabled ? 0.45f : 0.70f;
            const float verticalTolerance = continuousEnabled ? 0.035f : 0.030f;
            const bool progressOk = descentProgress >= minimumProgress;
            const bool normalOk = contactNormal.y >= minNormalY;
            const bool velocityOk = std::abs(swingVelocity.y) <= maxVerticalSpeed;
            const bool horizontalOk =
                comp._physicalStepHorizontalTargetError <= horizontalTolerance;
            const bool verticalOk =
                comp._physicalStepVerticalTargetError <= verticalTolerance;
            const float soleToleranceDeg = continuousEnabled ? 15.0f : 180.0f;
            const bool soleOk = !continuousEnabled
                || comp._gaitSoleAngularErrorDeg <= soleToleranceDeg;
            if (progressOk && normalOk && velocityOk && horizontalOk
                && verticalOk && soleOk) {
                acceptTouchdown();
            }
        };

        if (comp._physicalStepPhase == kSwing) {
            const float activeSwingTime = comp._gaitCancelMode != 0
                ? glm::max(0.18f, cadenceSwingTime * 0.65f)
                : cadenceSwingTime;
            const float swingProgress = glm::clamp(
                comp._physicalStepPhaseTime / activeSwingTime,
                0.0f, 1.0f);
            comp._physicalStepTrajectoryT = kSwingArrivalT * swingProgress;
            desiredFoot = trajectoryPoint(comp._physicalStepTrajectoryT);
            // Accumulate the arrival confidence during the final approach. A well-tracked
            // step can therefore cross the ARRIVAL milestone without an artificial pose
            // hold; a lagging or misaligned sole still receives the existing bounded wait.
            updateArrivalStability(
                comp._physicalStepTrajectoryT >= kSwingArrivalT - 0.10f);
            const bool earlySwingContact = swingContactNow
                && comp._physicalStepPhaseTime >= 0.10f;
            const bool recoverableRecontact = continuousEnabled
                && earlySwingContact && comp._physicalStepClearance >= 0.040f;
            if (recoverableRecontact) {
                if (comp._gaitSwingRecontactTime <= 0.0f) {
                }
                comp._gaitSwingRecontactTime += dt;
                // Keep opening vertical clearance while a toe/heel edge releases.
                desiredFoot.y = glm::max(
                    desiredFoot.y,
                    comp._physicalStepSwingStart.y
                        + glm::max(comp.swingHeight, takeoffHeight));
                if (comp._gaitSwingRecontactTime >= 0.10f)
                    abortSequence("early swing contact persisted through recovery window");
            } else {
                if (continuousEnabled && comp._gaitSwingRecontactTime > 0.0f
                    && !swingContactNow) {
                }
                comp._gaitSwingRecontactTime = 0.0f;
                if (earlySwingContact)
                    abortSequence("swing contacted before hover arrival");
            }
            if (comp._physicalStepPhase == kSwing && swingProgress >= 1.0f) {
                comp._physicalStepPhase = kArrival;
                comp._physicalStepPhaseTime = 0.0f;
                // The shared trajectory has only completed its approach landmark here;
                // horizontal motion intentionally continues through descent.  Snapping
                // to hoverTarget moved the foot all the way to the foothold for one frame,
                // then ARRIVAL moved it back to trajectoryPoint(kSwingArrivalT).  The
                // resulting forward/back velocity impulse grew with every planned step
                // and destabilized the following plant.
                desiredFoot = trajectoryPoint(kSwingArrivalT);
            }
        } else if (comp._physicalStepPhase == kArrival) {
            comp._physicalStepTrajectoryT = kSwingArrivalT;
            desiredFoot = trajectoryPoint(comp._physicalStepTrajectoryT);
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
            const bool arrivalMilestoneReady = continuousEnabled
                ? soleAligned
                : comp._physicalStepArrivalStableTime
                    >= cadenceArrivalSettleTime;
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
                           comp.arrivalTimeout, 0.10f)) {
                spdlog::warn(
                    "[LocomotionStep] ARRIVAL_CHECK result=FAIL horizontal={:.3f}/{:.3f}[{}] "
                    "hoverY={:.3f}/0.025[{}] stable={:.3f}/{:.3f}s[{}] "
                    "soleError={:.1f}/{:.1f}[{}] fwd={:+.3f} lat={:+.3f}",
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
                    comp._physicalStepLateralTargetError);
                abortSequence("swing foot could not retain safe clearance before descent");
            }
        } else if (comp._physicalStepPhase == kDescent) {
            const float descentProgress = glm::clamp(
                comp._physicalStepPhaseTime / cadenceDescentTime,
                0.0f, 1.0f);
            comp._physicalStepTrajectoryT = kSwingArrivalT
                + (1.0f - kSwingArrivalT) * descentProgress;
            desiredFoot = comp._gaitCancelMode != 0
                ? glm::mix(comp._physicalStepArcStart,
                           comp._physicalStepFoothold,
                           smoothstep(descentProgress))
                : trajectoryPoint(comp._physicalStepTrajectoryT);
            if (swingContactNow) {
                // Solver contact is reported before the newly landed sole has dissipated
                // its approach velocity. Keep validating on subsequent frames instead of
                // turning that first, expected impact sample into a gait abort.
                evaluateTouchdown("DESCENT", descentProgress);
            }
            if (comp._physicalStepPhase == kDescent
                && descentProgress >= 1.0f) {
                comp._physicalStepPhase = kTouchdownWait;
                comp._physicalStepPhaseTime = 0.0f;
                desiredFoot = comp._physicalStepFoothold;
            }
        } else if (comp._physicalStepPhase == kTouchdownWait) {
            comp._physicalStepTrajectoryT = 1.0f;
            desiredFoot = comp._physicalStepFoothold;
            if (swingContactNow) {
                evaluateTouchdown("TOUCHDOWN_WAIT", 1.0f);
            }
            if (comp._physicalStepPhase == kTouchdownWait
                && comp._physicalStepPhaseTime
                    >= glm::max(comp.plantTimeout, 0.10f)) {
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

        if (comp._physicalStepPhase >= kTakeoff && comp._physicalStepPhase <= kSettle
            && !stanceContactNow) {
            abortSequence("stance contact was lost");
        } else if (comp._physicalStepPhase >= kTakeoff && comp._physicalStepPhase <= kSettle
                   && comp._physicalStepStanceDrift > 0.040f) {
            abortSequence("stance foot exceeded 4 cm drift");
        } else if (comp._physicalStepPhase >= kTakeoff && comp._physicalStepPhase <= kSettle
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
        } else if (comp._physicalStepPhase == kSettle
                   && comp._physicalStepPlantPoseCaptured
                   && comp._physicalStepPlantDrift > 0.040f) {
            abortSequence("new plant exceeded 4 cm drift");
        } else if (transferEnabled && comp._physicalStepPhase >= kTransfer
                   && comp._physicalStepPhase <= kHold
                   && comp._supportTransferContactLossTime > 0.05f) {
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
        constexpr float kPlantPivotQuietTime = 0.08f;
        constexpr float kPlantPivotSoleToleranceDeg = 8.0f;
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
            <= kPlantPivotSoleToleranceDeg;
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
                    kPlantPivotSoleToleranceDeg,
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

        if (comp._physicalStepPlantPivotStableTime
                >= kPlantPivotQuietTime
            && pivotReleaseReady
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
                "deadlineProtectedUntil={:.3f}s",
                comp._stepSequenceStepIndex,
                comp._physicalStepSupportSide < 0 ? "RIGHT" : "LEFT",
                comp._physicalStepPlantPivotReleaseLatched
                    ? "recovery-release-to-center" : "pivot-to-center",
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
                comp._physicalStepPlantAnchorHandoffPhaseTime
                    + cadencePlantAcquireTime
                    + kPostHandoffAcquireMargin);
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
            if (movingFoot) {
                comp._gaitFootCorrection = 0.0f;
                comp._gaitFootCorrectionForward = 0.0f;
                comp._gaitFootTargetSpeed = 0.0f;
            }
            if (continuousEnabled && movingFoot
                && comp._physicalStepPhase >= kSwing
                && comp._physicalStepPhase <= kDescent) {
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
                glm::vec3 footCorrection = positionError
                    * glm::clamp(comp.gaitFootPositionGain, 0.0f, 1.0f)
                    + targetVelocity
                    * glm::max(comp.gaitFootVelocityLeadTime, 0.0f);
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
                footCorrection *= 1.0f - smoothstep(descentProgress);
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
            const glm::quat desiredFootWorld = glm::normalize(glm::slerp(
                glm::normalize(controlledLeg.plantedFootWorldRotation),
                nominalFootWorldRotation(controlledLeg), soleLevelBlend));
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
            const float upperLength = glm::length(
                skeleton.bones[controlledLeg.kneeIdx].localT);
            const float lowerLength = glm::length(
                skeleton.bones[controlledLeg.ankleIdx].localT);
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
            const glm::vec3 desiredUpper = glm::normalize(
                worldForward * hipCos + worldBend * hipSin);
            const glm::quat hipWorld = glm::normalize(
                RotationBetween(controlledLeg.referenceUpperWorld, desiredUpper)
                * controlledLeg.referenceHipWorld);
            const glm::quat parentWorld = ParentWorldRot(
                rag, skeleton, animator, entityWorld, controlledLeg.hipIdx);
            glm::quat hipTarget = glm::normalize(glm::conjugate(parentWorld) * hipWorld);
            const glm::quat unconstrainedHipTarget = hipTarget;
            Envelope hipEnvelope;
            hipEnvelope.twistAxis = controlledLeg.hipTwistAxis;
            hipEnvelope.swingNormalDeg = controlledLeg.hipSwingNormalDeg;
            hipEnvelope.swingPlaneDeg = controlledLeg.hipSwingPlaneDeg;
            hipEnvelope.twistMinDeg = controlledLeg.hipTwistMinDeg;
            hipEnvelope.twistMaxDeg = controlledLeg.hipTwistMaxDeg;
            hipTarget = ClampToEnvelope(hipEnvelope,
                skeleton.bones[controlledLeg.hipIdx].localR, hipTarget,
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
                    rag, controlledLeg.kneeIdx, &physicalKneeWorldOk);
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
                * glm::conjugate(controlledLeg.referenceFootLocal));
            Envelope ankleEnvelope;
            ankleEnvelope.twistAxis = controlledLeg.ankleAxis;
            ankleEnvelope.swingNormalDeg = controlledLeg.ankleSwingNormalDeg;
            ankleEnvelope.swingPlaneDeg = controlledLeg.ankleSwingPlaneDeg;
            ankleEnvelope.twistMinDeg = controlledLeg.ankleTwistMinDeg;
            ankleEnvelope.twistMaxDeg = controlledLeg.ankleTwistMaxDeg;
            ankleTarget = ClampToEnvelope(ankleEnvelope,
                skeleton.bones[controlledLeg.ankleIdx].localR, ankleTarget,
                comp.hipLimitMarginDeg);

            if (continuousEnabled && movingFoot)
                comp._gaitIkHipEnvelopeClampDeg = rotationDifferenceDeg(
                    unconstrainedHipTarget, hipTarget);

            const float poseResponse = movingFoot
                ? glm::max(comp.standingPoseResponse, 0.01f)
                : glm::min(glm::max(comp.standingPoseResponse, 0.01f), 0.05f);
            const float alpha = 1.0f - std::exp(-dt / poseResponse);
            controlledLeg.hipCommand = glm::normalize(
                glm::slerp(controlledLeg.hipCommand, hipTarget, alpha));
            controlledLeg.kneeCommand = glm::normalize(
                glm::slerp(controlledLeg.kneeCommand, kneeTarget, alpha));
            controlledLeg.ankleCommand = glm::normalize(
                glm::slerp(controlledLeg.ankleCommand, ankleTarget, alpha));
            if (continuousEnabled && movingFoot) {
                comp._gaitIkHipCommandLagDeg = rotationDifferenceDeg(
                    controlledLeg.hipCommand, hipTarget);
                comp._gaitIkKneeCommandLagDeg = rotationDifferenceDeg(
                    controlledLeg.kneeCommand, kneeTarget);
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
            capturePhysicalLocalPose(*swing);
            capturePhysicalLocalPose(*stance);
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

        auto beginTransfer = [&]() {
            const float transferFraction = glm::clamp(
                comp.transferSupportBias + comp._gaitAdaptiveTransferBiasOffset,
                0.70f, 0.98f);
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
                const float remainingForward = glm::max(glm::dot(
                    comp._gaitSupportCurveEnd - comp._gaitSupportCurveStart,
                    comp._physicalStepForward), 0.0f);
                const float outgoingSpeed = glm::clamp(
                    0.25f * remainingForward
                        / comp._gaitSupportCurveDuration,
                    0.03f, 0.10f);
                comp._gaitSupportCurveEndVelocity =
                    comp._physicalStepForward * outgoingSpeed;
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
            }
            comp._supportTransferTransferT = 0.0f;
            comp._supportTransferHoldStableTime = 0.0f;
            comp._supportTransferContactLossTime = 0.0f;
            comp._physicalStepPhase = kTransfer;
            comp._physicalStepPhaseTime = 0.0f;
        };

        auto updateGaitAdaptation = [&]() {
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
            filter(comp._gaitFilteredForwardError,
                   comp._physicalStepForwardTargetError);
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
            const float targetStrideOffset = glm::clamp(
                landingFeedForward - driftPenalty - authorityPenalty,
                -maxStrideCorrection, maxStrideCorrection);
            comp._gaitAdaptiveStrideOffset = glm::mix(
                comp._gaitAdaptiveStrideOffset,
                targetStrideOffset, response);

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
                    comp.transferSupportBias, 0.70f, 0.98f), 0.0f));
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
        constexpr float kLoadedSoleToleranceDeg = 12.0f;
        const bool loadedSoleReady = !continuousEnabled
            || comp._gaitSoleAngularErrorDeg <= kLoadedSoleToleranceDeg;
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

        if (comp._physicalStepPhase == kSettle) {
            const float minimumSupportAdvance = glm::min(
                comp.gaitMinStepLength, comp.gaitMaxStepLength);
            if (!comp._physicalStepPlantPoseCaptured) {
                const char* incomingPlantSide =
                    comp._physicalStepSupportSide < 0 ? "RIGHT" : "LEFT";
                const float plantSpeed = glm::length(swingVelocity);
                const float maxAcquireSpeed = continuousEnabled
                    ? glm::max(comp.plantAcquireMaxSpeed, 0.18f)
                    : glm::max(comp.plantAcquireMaxSpeed, 0.01f);
                constexpr float kPlantAcquireDriftLimit = 0.030f;
                constexpr float kPlantAcquireGrowthLimit = 0.020f;
                constexpr float kSettledOffsetDriftLimit = 0.040f;
                constexpr float kSettledOffsetGrowthLimit = 0.010f;
                constexpr float kSettledOffsetSpeedLimit = 0.030f;
                constexpr float kSettledOffsetTime = 0.10f;
                const bool settledOffsetCandidate = continuousEnabled
                    && !comp._physicalStepPlantAnchorRebased
                    && !comp._physicalStepPlantCenterAnchorActive
                    && !comp._physicalStepPlantPivotReleaseLatched
                    && swingContactNow && stanceContactNow
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
                const bool plantDriftReady = !continuousEnabled
                    || comp._physicalStepPlantDrift
                        <= kPlantAcquireDriftLimit;
                const bool plantDriftNoLongerGrowing = !continuousEnabled
                    || comp._gaitPlantDriftRate
                        <= kPlantAcquireGrowthLimit;
                // Once the impact pivot has been released as an active recovery, the
                // quiet center capture is mandatory. A merely slow foot must not start
                // support transfer while ownership is still fading away from the edge.
                const bool plantAnchorOwnershipReady = !continuousEnabled
                    || !comp._physicalStepPlantPivotReleaseLatched
                    || comp._physicalStepPlantCenterAnchorActive;
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
                // A center handoff changes the drift invariant and intentionally resets
                // the stable proof. A handoff near the old absolute timeout must receive
                // enough bounded time to prove that new invariant; otherwise the state
                // transition itself deterministically causes an abort a few frames later.
                const float postHandoffAcquireDeadline =
                    comp._physicalStepPlantAnchorHandoffPhaseTime >= 0.0f
                    ? comp._physicalStepPlantAnchorHandoffPhaseTime
                        + cadencePlantAcquireTime
                        + kPostHandoffAcquireMargin
                    : 0.0f;
                const float activeAcquireTimeout = glm::max(
                    ordinaryAcquireTimeout, postHandoffAcquireDeadline);
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
                    >= cadencePlantAcquireTime) {
                    const float retainedFromTarget =
                        comp._gaitPlannedSupportAdvance
                        - glm::max(comp._physicalStepForwardTargetError, 0.0f);
                    // Runtime landing accepts a bounded tracking miss and feeds it into the
                    // next foothold. A stable physical contact is recoverable; treating a
                    // few centimetres of servo lag as ABORT caused the release-to-restart
                    // behavior and pulled the COM back to the old baseline.
                    const float requiredAdvance = comp._gaitCancelMode == 1
                        ? 0.0f
                        : glm::min(minimumSupportAdvance, glm::max(
                            0.03f, comp._gaitPlannedSupportAdvance
                                - glm::max(comp.footTargetTolerance, 0.01f)));
                    constexpr float kLandingAdvanceHysteresis = 0.010f;
                    const bool retainsMinimumAdvance = !continuousEnabled
                        || comp._gaitAchievedSupportAdvance
                            + kLandingAdvanceHysteresis >= requiredAdvance;
                    if (continuousEnabled
                        && comp._gaitAchievedSupportAdvance < requiredAdvance
                        && retainsMinimumAdvance) {
                        spdlog::info(
                            "[LocomotionGait] LANDING_ADVANCE_CHECK "
                            "result=HYSTERESIS achieved={:.3f}/{:.3f} margin={:.3f}m",
                            comp._gaitAchievedSupportAdvance,
                            requiredAdvance,
                            kLandingAdvanceHysteresis);
                    }
                    if (!retainsMinimumAdvance) {
                        spdlog::warn(
                            "[LocomotionGait] LANDING_ADVANCE_CHECK result=RECOVER "
                            "planned={:.3f} retainedFromTarget={:.3f} "
                            "achieved={:.3f}/{:.3f} forwardError={:+.3f} "
                            "stanceDrift={:.3f}",
                            comp._gaitPlannedSupportAdvance,
                            retainedFromTarget,
                            comp._gaitAchievedSupportAdvance,
                            requiredAdvance,
                            comp._physicalStepForwardTargetError,
                            comp._physicalStepStanceDrift);
                        // Finish this contact safely and stop; held input can start again
                        // automatically once standing, without a mandatory key release.
                        comp._gaitStopRequested = true;
                        comp._gaitCancelMode = 2;
                    }
                    if (!continuousEnabled)
                        capturePhysicalLocalPose(*swing);
                    // The two-stage plant has now captured either the quiet sole center or,
                    // as a fallback, a credible settled contact. Keep solving the planted
                    // leg while the pelvis passes over that world-space support.
                    comp._physicalStepPlantPoseCaptured = true;
                    comp._physicalStepSettleTime = 0.0f;
                    comp._gaitLandingVerificationPending = true;
                    comp._gaitLandingStableTime = 0.0f;

                    // Transfer and landing verification now run together. An early-return
                    // cancellation has no new support foot, so it verifies in place and
                    // enters the standing transition directly.
                    if (comp._gaitCancelMode != 1 && transferEnabled)
                        beginTransfer();
                } else if (comp._physicalStepPhaseTime
                               >= activeAcquireTimeout) {
                    spdlog::warn(
                        "[LocomotionStep] PLANT_ACQUIRE result=FAIL plant={} contact={} "
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
                        incomingPlantSide,
                        swingContactNow ? "yes" : "no",
                        stanceContactNow ? "yes" : "no",
                        plantSpeed, maxAcquireSpeed,
                        comp._physicalStepPlantAcquireStableTime,
                        cadencePlantAcquireTime,
                        comp._physicalStepPhaseTime,
                        activeAcquireTimeout,
                        comp._physicalStepForwardTravel,
                        comp._physicalStepPlantDrift,
                        kPlantAcquireDriftLimit,
                        comp._physicalStepPlantCenterTravel,
                        comp._gaitPlantDriftRate,
                        kPlantAcquireGrowthLimit,
                        comp._gaitSoleAngularErrorDeg,
                        kLoadedSoleToleranceDeg,
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
                    abortSequence("new plant did not settle before acquisition timeout");
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
                comp._supportTransferHoldStableTime = 0.0f;
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
            // complete old-to-new support span. Acquire at 68%, but retain ownership down
            // to 64% so sub-frame COM noise cannot repeatedly reset the stable window.
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
            const bool stableTransfer = transferPositionReady
                && swingContactNow && stanceContactNow
                && oldLegUnloaded
                && loadedSoleReady
                && transferMotionReady
                && (continuousEnabled || comp._physicalStepStanceDrift <= 0.040f)
                && comp._physicalStepPlantDrift <= 0.040f
                && tiltDeg < 30.0f
                && !rag._locomotionSupportSaturated
                && rag.locomotionLiftBone < 0
                && rag._locomotionLiftForce <= 0.5f
                && locksOff
                && !comp._physicalStepMotorSaturated;
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
                if (comp._gaitCancelMode == 0)
                    updateGaitAdaptation();
                if (continuousEnabled) {
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
                            comp._physicalStepForward);
                        comp._gaitMeasuredSpeed = comAdvance
                            / comp._gaitLastStepPeriod;
                        const float stepDrift =
                            comp._gaitStepMaxRelevantDrift;
                        comp._gaitMaxDrift = glm::max(
                            comp._gaitMaxDrift, stepDrift);
                        comp._gaitPeakTilt = glm::max(
                            comp._gaitPeakTilt, comp._physicalStepPeakTilt);
                        comp._gaitMaxMotorRatio = glm::max(
                            comp._gaitMaxMotorRatio, comp._physicalStepMaxMotorRatio);

                        const bool shouldStop = comp._gaitStopRequested;

                        if (shouldStop) {
                            beginStopping();
                        } else {
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
                    "newRegion={:.3f}/{:.3f}[{}] "
                    "newLoad={:.2f}/{:.2f}[{}] "
                    "contact=({},{}) speed={:.3f} drift=({:.3f},{:.3f}) "
                    "tilt={:.1f}/30 supportSat={} motorSat={} stable={:.3f}/{:.3f}s",
                    comp._supportTransferComError, comTolerance,
                    liveSupportError, liveSupportTolerance,
                    transferPositionReady ? "ok" : "FAIL",
                    comp._supportTransferComToNewSupport, newSupportRadius,
                    insideNewSupport ? "ok" : "FAIL",
                    comp._gaitNewSupportLoad,
                    kNewSupportLoadAcquireThreshold,
                    oldLegUnloaded ? "ok" : "FAIL",
                    comp._physicalStepContactL ? "L" : "-",
                    comp._physicalStepContactR ? "R" : "-",
                    comp._supportTransferComHorizontalSpeed,
                    comp._physicalStepStanceDrift, comp._physicalStepPlantDrift,
                    tiltDeg,
                    rag._locomotionSupportSaturated ? "YES" : "no",
                    comp._physicalStepMotorSaturated ? "YES" : "no",
                    comp._supportTransferHoldStableTime,
                    cadenceTransferHoldTime);
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
                comp._gaitRunning = false;
                if (pass) {
                    comp._physicalStepPhase = kIdle;
                    comp._physicalStepPhaseTime = 0.0f;
                    comp._physicalStepSupportSide = 0;
                    comp._gaitStopRequested = false;
                    comp._gaitAdaptiveStopRequested = false;
                    comp._gaitRecoveryFailureSteps = 0;
                    comp._gaitStress *= 0.50f;
                    spdlog::info(
                        "[LocoRuntime] STOP_COMPLETE steps={} time={:.2f}s "
                        "forward={:.3f} finalTilt={:.1f} recovery={} "
                        "stopSettleDrift={:.3f}m diagnostics=(edges={},length={},"
                        "period={},historyDrift={:.3f}m,historyTilt={:.1f},"
                        "historyMotor={:.2f}) ready=IDLE",
                        completed, comp._gaitRunTime, totalForward, tiltDeg,
                        adaptiveRecovery ? "adaptive" : "commanded",
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
                    case kSwing:       return cadenceSwingTime;
                    case kArrival:     return cadenceArrivalSettleTime;
                    case kDescent:     return cadenceDescentTime;
                    case kSettle:      return cadencePlantAcquireTime;
                    case kTransfer:    return cadenceTransferTime;
                    case kHold:        return cadenceTransferHoldTime;
                    case kInterStep:   return cadenceInterStepTime;
                    default:           return 0.0f;
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
