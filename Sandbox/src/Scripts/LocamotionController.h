#pragma once
#include "Scene/Scripting.h"
#include "Scene/Scene.h"
#include "Scene/ComponentRegistry.h"
#include <imgui.h>
#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>
#include "Core/Input.h"
#include "Scene/Physics/PhysicsAPI.h"
#include "Scene/Physics/Rigidbody.h"
#include "Scene/Physics/RagdollComponent.h"
#include "Animation/AnimationComponents.h"
#include "Animation/AnimationSampler.h"
#include "Animation/IKSolver.h"
#include "DebugDraw.h"
#include "CameraDirector.h"
#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>
#include <glm/gtc/constants.hpp>
#include <cmath>

// ---- Data -------------------------------------------------------------------
// PR1 of Docs/character-locomotion-design.md: root drive + input. Put this on a
// skinned character that also has a RagdollComponent (a built .ragdoll). The left
// stick steers the character camera-relative; the system drags the ragdoll's hips
// along kinematically (RagdollComponent::locomotionTargetPos/Rot -- joint motors
// alone can't move/support the rootless pelvis) while every other body stays
// Dynamic and Powered, dangling/reacting off the moving hips.
//
// strideLength/stepHeight/dutyFactor/bobAltitude/leanGain are PR2/PR3 fields (gait
// generator + sloppiness tuning) -- declared here already per the design doc's data
// table, but not read yet: today the limbs just hang off whatever pose the
// AnimatorComponent already holds.

struct LocamotionControllerComponent
{
    // UNITS: Meters
    float maxSpeed          = 2.5f;
    float acceleration      = 10.0f;
    float deceleration      = 15.0f;
    float turnRate          = 8.0f;
    float deadzone          = 0.2f;    // stick magnitude below which input is ignored
    float strideLength      = 0.55f;
    float stepHeight        = 0.12f;
    float dutyFactor        = 0.6f;
    float hipHeight         = 0.8f;    // fallback; CAPTURED from the .ragdoll bind pose once built
    float bobAltitude       = 0.03f;
    float leanGain          = 0.25f;
    float groundRayLength   = 1.0f;    // must exceed hipHeight or the probe undershoots the ground
    float facingOffsetDeg   = 0.0f;    // yaw correction if the model walks sideways/backward
                                       // (rig bind facing vs the controller's -Z convention)
    float ankleHeight       = 0.08f;   // ankle-bone center above the sole (IKChain convention)

    // ---- Gait leg chains (bone names; CesiumMan defaults) ----
    // tip = ankle; knee/hip resolved as its parent/grandparent, same convention as
    // IKChain. pelvisBone should be the .ragdoll ROOT bone (the body the root drive
    // moves) -- it anchors the world->model mapping for foot targets.
    std::string pelvisBone    = "torso_joint_3";
    std::string leftFootBone  = "leg_joint_L_3";
    std::string rightFootBone = "leg_joint_R_3";

    // Gait diagnostics: ~4 Hz spdlog state dump + viewport markers (drawn into the
    // same DebugDraw batch as colliders/ragdolls, so the viewport "Debug Draw"
    // checkbox must also be on). Markers: GREEN sphere = planted foot target,
    // ORANGE = swing foot target, RED sphere + line = actual simulated foot and its
    // tracking error to the target, YELLOW = hips drive target.
    bool gaitDebug = false;

    // ---- Runtime (not serialized) ----
    struct LegState {
        int       tip = -1, knee = -1, hip = -1;      // resolved bone indices
        glm::vec3 planted      { 0.0f };  // world point the foot is pinned to
        glm::vec3 swingStart   { 0.0f };  // world point the foot lifted off from
        glm::vec3 lastSwingEnd { 0.0f };  // where the current swing will plant
        glm::vec3 stanceModel  { 0.0f };  // bind foot offset from pelvis (model space)
        bool wasPlanted = true, init = false, bindResolved = false;
    };
    glm::vec3 _velocity     { 0.0f };
    glm::vec3 _rootPos      { 0.0f };  // integrated root path (see the re-anchoring note in OnUpdate)
    bool      _rootInit     = false;
    float     _yaw          = 0.0f;    // radians; world convention: yaw 0 = facing -Z
    bool      _hipHeightSet = false;
    float     _gaitPhase    = 0.0f;    // cycles; L leg at phase, R leg at phase+0.5
    float     _dbgTimer     = 0.0f;    // gaitDebug log throttle
    int       _pelvisIdx    = -1;
    glm::vec3 _poleWorld    { 0, 0, -1 }; // last movement dir (world); knee-bend hint
    LegState  _legL, _legR;
};

// ---- Inspector UI -----------------------------------------------------------

template<>
inline void DrawComponentInspector<LocamotionControllerComponent>(LocamotionControllerComponent& c)
{
    ImGui::DragFloat("Max Speed",         &c.maxSpeed,        0.1f,  0.0f, 20.0f);
    ImGui::DragFloat("Acceleration",      &c.acceleration,    0.1f,  0.0f, 50.0f);
    ImGui::DragFloat("Deceleration",      &c.deceleration,    0.1f,  0.0f, 50.0f);
    ImGui::DragFloat("Turn Rate",         &c.turnRate,        0.1f,  0.0f, 30.0f);
    ImGui::DragFloat("Deadzone",          &c.deadzone,        0.01f, 0.0f, 0.9f);
    ImGui::DragFloat("Hip Height",        &c.hipHeight,       0.01f, 0.05f, 3.0f);
    ImGui::DragFloat("Ground Ray Length", &c.groundRayLength, 0.05f, 0.1f, 10.0f);
    ImGui::DragFloat("Facing Offset",     &c.facingOffsetDeg, 1.0f, -180.0f, 180.0f, "%.0f deg");
    ImGui::DragFloat("Ankle Height",      &c.ankleHeight,     0.005f, 0.0f, 0.3f);
    ImGui::Separator();
    ImGui::TextDisabled("Gait:");
    ImGui::DragFloat("Stride Length",     &c.strideLength,    0.01f, 0.05f, 3.0f);
    ImGui::DragFloat("Step Height",       &c.stepHeight,      0.01f, 0.0f,  1.0f);
    ImGui::DragFloat("Duty Factor",       &c.dutyFactor,      0.01f, 0.1f,  0.9f);
    ImGui::DragFloat("Bob Altitude",      &c.bobAltitude,     0.005f,0.0f,  0.5f);   // PR3
    ImGui::DragFloat("Lean Gain",         &c.leanGain,        0.01f, 0.0f,  2.0f);   // PR3
    auto boneField = [](const char* label, std::string& name) {
        char buf[128];
        std::snprintf(buf, sizeof(buf), "%s", name.c_str());
        if (ImGui::InputText(label, buf, sizeof(buf))) name = buf;
    };
    boneField("Pelvis Bone",     c.pelvisBone);
    boneField("Left Foot Bone",  c.leftFootBone);
    boneField("Right Foot Bone", c.rightFootBone);
    ImGui::Checkbox("Gait Debug", &c.gaitDebug);
    if (c.gaitDebug)
        ImGui::TextDisabled("needs viewport Debug Draw ON; green/orange=targets,\nred=actual foot, yellow=hips");
    ImGui::Separator();
    ImGui::TextDisabled("Left stick = move (camera-relative)");
    ImGui::TextDisabled(c._hipHeightSet ? "Hip height: captured from ragdoll bind pose"
                                        : "Hip height: fallback (ragdoll not built yet)");
}

// ---- Serialization ----------------------------------------------------------

template<>
inline std::string SerializeComponent<LocamotionControllerComponent>(const LocamotionControllerComponent& c)
{
    nlohmann::json j;
    j["maxSpeed"]        = c.maxSpeed;
    j["acceleration"]    = c.acceleration;
    j["deceleration"]    = c.deceleration;
    j["turnRate"]        = c.turnRate;
    j["deadzone"]        = c.deadzone;
    j["strideLength"]    = c.strideLength;
    j["stepHeight"]      = c.stepHeight;
    j["dutyFactor"]      = c.dutyFactor;
    j["hipHeight"]       = c.hipHeight;
    j["bobAltitude"]     = c.bobAltitude;
    j["leanGain"]        = c.leanGain;
    j["groundRayLength"] = c.groundRayLength;
    j["facingOffsetDeg"] = c.facingOffsetDeg;
    j["pelvisBone"]      = c.pelvisBone;
    j["leftFootBone"]    = c.leftFootBone;
    j["rightFootBone"]   = c.rightFootBone;
    j["gaitDebug"]       = c.gaitDebug;
    j["ankleHeight"]     = c.ankleHeight;
    return j.dump();
}

template<>
inline void DeserializeComponent<LocamotionControllerComponent>(LocamotionControllerComponent& c, const std::string& data)
{
    auto j              = nlohmann::json::parse(data);
    c.maxSpeed          = j.value("maxSpeed",        2.5f);
    c.acceleration      = j.value("acceleration",    10.0f);
    c.deceleration      = j.value("deceleration",    15.0f);
    c.turnRate          = j.value("turnRate",        8.0f);
    c.deadzone          = j.value("deadzone",        0.2f);
    c.strideLength      = j.value("strideLength",    0.55f);
    c.stepHeight        = j.value("stepHeight",      0.12f);
    c.dutyFactor        = j.value("dutyFactor",      0.6f);
    c.hipHeight         = j.value("hipHeight",       0.8f);
    c.bobAltitude       = j.value("bobAltitude",     0.03f);
    c.leanGain          = j.value("leanGain",        0.25f);
    c.groundRayLength   = j.value("groundRayLength", 1.0f);
    c.facingOffsetDeg   = j.value("facingOffsetDeg", 0.0f);
    c.pelvisBone        = j.value("pelvisBone",      std::string("torso_joint_3"));
    c.leftFootBone      = j.value("leftFootBone",    std::string("leg_joint_L_3"));
    c.rightFootBone     = j.value("rightFootBone",   std::string("leg_joint_R_3"));
    c.gaitDebug         = j.value("gaitDebug",       false);
    c.ankleHeight       = j.value("ankleHeight",     0.08f);
}

// ---- Registration -----------------------------------------------------------

DECLARE_COMPONENT(LocamotionControllerComponent, "LocamotionController")

// ---- Behavior ---------------------------------------------------------------

class LocamotionControllerSystem : public GameSystem
{
    DECLARE_SYSTEM(LocamotionControllerSystem, 100)
public:
    void OnStart(Scene& scene) override
    {
        Input::BindAxis("MoveX", GamepadAxis::LeftX);
        Input::BindAxis("MoveY", GamepadAxis::LeftY);
    }

    void OnUpdate(Scene& scene, float dt) override
    {
        const float h   = Input::GetAxis("MoveX");
        const float f   = -Input::GetAxis("MoveY");
        const float mag = glm::min(glm::length(glm::vec2(h, f)), 1.0f);

        glm::vec3 camRight, camForward;
        CameraRelativeBasis(scene, camRight, camForward);

        for (auto [entity, comp] : scene.View<LocamotionControllerComponent>().each())
        {
            if (!scene.Has<RagdollComponent>(entity) || !scene.Has<TransformComponent>(entity)) continue;
            auto& rag = scene.Get<RagdollComponent>(entity);
            auto& xf  = scene.Get<TransformComponent>(entity);

            // --- Step 3: Powering Ragdoll (gate) ---
            // Scenes author the ragdoll as Animated; a player-controlled rig needs to
            // be an active ragdoll to be driven at all. Flipped HERE, not OnStart:
            // this system (prio 100) starts before PhysicsSystem (prio 200), so at
            // OnStart the ragdoll isn't built yet and SetRagdollMode would no-op.
            // Also how control resumes after a get-up hands the rig back to Animated.
            if (rag.mode == RagdollMode::Animated)
                Physics::SetRagdollMode(rag, RagdollMode::Powered);

            // Still not Powered = the ragdoll isn't built (no .ragdoll asset?) or a
            // knockdown owns the rig (Limp; see the design doc's reaction hand-off)
            // -- stand down cleanly so the engine keeps/releases the hips as needed.
            if (rag.mode != RagdollMode::Powered) {
                rag.locomotionActive = false;
                comp._velocity = glm::vec3(0.0f);
                // Feet re-pin and the root path re-seeds from the body when control
                // resumes (the rig may have been knocked/carried anywhere meanwhile).
                comp._legL.init = comp._legR.init = false;
                comp._rootInit  = false;
                continue;
            }

            // --- Step 1: Root Drive ---

            // Deadzone remap, then camera-relative desired velocity.
            const float speed = (mag > comp.deadzone)
                ? (mag - comp.deadzone) / (1.0f - comp.deadzone) * comp.maxSpeed : 0.0f;
            glm::vec3 desiredDir = camRight * h + camForward * f;
            glm::vec3 desiredVel = glm::length(desiredDir) > 1e-4f
                ? glm::normalize(desiredDir) * speed : glm::vec3(0.0f);

            // Ease current velocity toward desired, separate accel/decel rates.
            glm::vec3 delta = desiredVel - comp._velocity;
            const float distToTarget = glm::length(delta);
            const float rate = glm::length(desiredVel) > glm::length(comp._velocity)
                ? comp.acceleration : comp.deceleration;
            const float step = rate * dt;
            comp._velocity += (distToTarget > step && distToTarget > 1e-5f)
                ? (delta / distToTarget) * step : delta;

            // Face the movement heading (shortest-arc yaw ease), only while moving.
            // R_y(theta)*(0,0,-1) = (-sin theta, 0, -cos theta), so facing velocity v
            // needs theta = atan2(-v.x, -v.z). atan2(+v.x, ...) negates theta for
            // every heading = a MIRRORED facing (the input-vs-facing table reflected
            // about 45 deg, with 45 as the mirror's fixed point).
            if (glm::length(comp._velocity) > 0.05f) {
                const float targetYaw = std::atan2(-comp._velocity.x, -comp._velocity.z);
                float diff = std::fmod(targetYaw - comp._yaw + glm::pi<float>(), glm::two_pi<float>());
                if (diff < 0.0f) diff += glm::two_pi<float>();
                diff -= glm::pi<float>();
                comp._yaw += diff * glm::min(comp.turnRate * dt, 1.0f);
            }

            // Hip height comes from the rig itself: the .ragdoll bind pose's standing
            // clearance (hips above the lowest body -- what the get-up stands up to).
            // Captured lazily, NOT in OnStart: the ragdoll is built by PhysicsSystem
            // (prio 200) AFTER this system starts, so the value reads 0 until the
            // first live frame. Never derived from the entity origin -- its height is
            // model-specific (often the feet), and on the first frame it hasn't been
            // re-rooted to the hips yet, which bakes in a crumple-to-the-floor value.
            if (!comp._hipHeightSet) {
                const float clearance = Physics::GetRagdollStandHipClearance(rag);
                if (clearance > 0.0f) {
                    // Slightly below full bind extension: keeps planted-foot targets
                    // inside the leg chain's reach (clearance is root-to-lowest-body-
                    // CENTER, which is ramrod-straight territory for the IK) and the
                    // permanent knee bend reads nicely drunk anyway. The Hip Height
                    // slider can be dragged live in play mode to taste.
                    comp.hipHeight     = clearance * 0.92f;
                    comp._hipHeightSet = true;
                }
            }

            // STATEFUL root path: integrate our own target position, never re-anchor
            // to the simulated body each frame. Re-anchoring (`xf.position + v*dt`)
            // made the drive frame-rate dependent: targets are recomputed per RENDER
            // frame but consumed per 60 Hz physics tick, so at high fps each tick
            // advanced the hips by only ONE render-frame's delta (~25% speed at
            // 240 fps) -- the gait phase then outran the body and both feet rode
            // perpetually ahead of the pelvis.
            if (!comp._rootInit) {
                comp._rootPos  = xf.position;
                comp._rootInit = true;
            }
            comp._rootPos.x += comp._velocity.x * dt;
            comp._rootPos.z += comp._velocity.z * dt;
            // Divergence guard: if the body is far off the path (teleported, wedged),
            // re-seed rather than dragging it across the map.
            const glm::vec2 pathGap { comp._rootPos.x - xf.position.x,
                                      comp._rootPos.z - xf.position.z };
            if (glm::length(pathGap) > 1.0f) {
                comp._rootPos.x = xf.position.x;
                comp._rootPos.z = xf.position.z;
            }

            // Ground probe straight down at the path position -- ignoring the whole
            // ragdoll (EntityIgnoreFilter covers every body owned by this entity),
            // so the drive follows gentle slopes/steps.
            HitResult hit = Physics::Raycast(glm::vec3(comp._rootPos.x, xf.position.y, comp._rootPos.z),
                                             glm::vec3(0, -1, 0), comp.groundRayLength, entity);
            const float groundY = hit.hit ? hit.point.y : (xf.position.y - comp.hipHeight);

            // locomotionTargetRot is a WORLD HEADING: the engine composes it on top
            // of the hips bone's bind orientation (rootBindRot), so identity = upright
            // -- never build the bone's raw orientation here (model bone frames vary;
            // driving CesiumMan's Z-up hips to a raw yaw laid the rig down sideways).
            rag.locomotionActive    = true;
            rag.locomotionTargetPos = glm::vec3(comp._rootPos.x, groundY + comp.hipHeight, comp._rootPos.z);
            rag.locomotionTargetRot =
                glm::angleAxis(comp._yaw + glm::radians(comp.facingOffsetDeg), glm::vec3(0, 1, 0));

            // --- Step 2: Gait Generator ---
            // Phase-driven plant/swing foot targets -> two-bone IK -> leg rotations
            // written into anim.pose, which SyncRagdollPowered's motors chase this
            // frame (we run at prio 100, physics at 200) and UpdateAnimators resets
            // to bind afterwards (clip -1) -- so the write must happen EVERY frame.
            if (comp._hipHeightSet &&
                scene.Has<SkinnedMeshComponent>(entity) && scene.Has<AnimatorComponent>(entity))
                UpdateGait(scene, entity, comp, rag, xf, dt);
        }
    }

    void OnDestroy(Scene& scene) override {}

private:
    // Normalized rotation of a (possibly scaled) model-space bone matrix.
    static glm::quat OrientationOf(const glm::mat4& m)
    {
        glm::mat3 r(m);
        r[0] = glm::normalize(r[0]);
        r[1] = glm::normalize(r[1]);
        r[2] = glm::normalize(r[2]);
        return glm::normalize(glm::quat_cast(r));
    }

    // Lazily resolve tip/knee/hip indices from the ankle bone name (tip's parent
    // chain), re-validating against the name like IKSystem does.
    static bool ResolveLeg(const Diamond::Skeleton& skel, const std::string& footBone,
                           LocamotionControllerComponent::LegState& leg)
    {
        const int n = (int)skel.bones.size();
        if (leg.tip < 0 || leg.tip >= n || skel.bones[leg.tip].name != footBone) {
            leg.tip = skel.Find(footBone);
            if (leg.tip < 0) return false;
            leg.knee = skel.bones[leg.tip].parent;
            leg.hip  = leg.knee >= 0 ? skel.bones[leg.knee].parent : -1;
            leg.bindResolved = false;
        }
        return leg.hip >= 0;
    }

    // Ground height under a world point: ray from hip height straight down, ignoring
    // every body of the character. Falls back to "hips minus clearance" on a miss.
    static glm::vec3 GroundAt(Scene& scene, entt::entity self, const glm::vec3& probe,
                              const LocamotionControllerComponent& comp, const glm::vec3& hipsW)
    {
        // Targets are for the ANKLE bone, whose center sits above the sole -- lift
        // the ground hit by ankleHeight (same correction as IKChain::ankleHeight)
        // or the leg has to overreach by a foot's thickness on every plant.
        const glm::vec3 lift { 0.0f, comp.ankleHeight, 0.0f };
        const glm::vec3 origin { probe.x, hipsW.y, probe.z };
        HitResult hit = Physics::Raycast(origin, glm::vec3(0, -1, 0), comp.groundRayLength, self);
        return (hit.hit ? hit.point
                        : glm::vec3(probe.x, hipsW.y - comp.hipHeight, probe.z)) + lift;
    }

    // The gait generator (design doc step 4/5). Foot targets live in WORLD space so a
    // planted foot is truly pinned (zero slide) while the hips travel; the solve runs
    // in MODEL space. The two are related through the HEADING, not the entity
    // transform alone: the root drive yaws the hips BODY while the entity/model frame
    // never rotates (the readback re-root only moves position), so the physics chain
    // realizes world = heading * entityRot * model (anchored at the hips). Foot
    // targets are therefore un-yawed by conj(heading) on the way into model space --
    // without this the legs would step in the spawn-facing direction forever.
    void UpdateGait(Scene& scene, entt::entity entity, LocamotionControllerComponent& comp,
                    RagdollComponent& rag, TransformComponent& xf, float dt)
    {
        auto& smc  = scene.Get<SkinnedMeshComponent>(entity);
        auto& anim = scene.Get<AnimatorComponent>(entity);
        const Diamond::Skeleton& skel = smc.skeleton;
        const int n = (int)skel.bones.size();
        if (n == 0 || (int)anim.pose.size() != n) return;   // pose not built yet (first frame)

        // Anchor on the bone the ragdoll ROOT BODY is actually bound to -- the
        // world<->model identity is anchored at the body the root drive moves.
        // Guessing by name put the anchor mid-torso on CesiumMan (torso_joint_3 is
        // NOT the root bone): every foot target rode ~0.35 too high and the legs
        // paddled at shin height. The authored pelvisBone is only the fallback for
        // the frames before the ragdoll reports its root.
        const int rootBone = Physics::GetRagdollRootBone(rag);
        if (rootBone >= 0 && rootBone < n) {
            comp._pelvisIdx = rootBone;
        } else if (comp._pelvisIdx < 0 || comp._pelvisIdx >= n ||
                   skel.bones[comp._pelvisIdx].name != comp.pelvisBone) {
            comp._pelvisIdx = skel.Find(comp.pelvisBone);
        }
        if (comp._pelvisIdx < 0) return;
        if (!ResolveLeg(skel, comp.leftFootBone,  comp._legL)) return;
        if (!ResolveLeg(skel, comp.rightFootBone, comp._legR)) return;

        // World <-> gait-model mapping (see comment above): world = heading*entityRot*model*s.
        const glm::quat heading  = glm::angleAxis(comp._yaw + glm::radians(comp.facingOffsetDeg),
                                                  glm::vec3(0, 1, 0));
        const float     s        = glm::max((xf.scale.x + xf.scale.y + xf.scale.z) / 3.0f, 1e-4f);
        const glm::quat entConj  = glm::conjugate(xf.rotation);
        const glm::quat headConj = glm::conjugate(heading);
        auto dirToModel = [&](const glm::vec3& v) { return (entConj * (headConj * v)) / s; };
        auto dirToWorld = [&](const glm::vec3& v) { return heading * (xf.rotation * v) * s; };

        const glm::vec3 hipsW = rag.locomotionTargetPos;   // where the root drive sends the hips
        const glm::vec3 velH  { comp._velocity.x, 0.0f, comp._velocity.z };
        const float     speedH = glm::length(velH);
        const bool      moving = speedH > 0.15f;
        if (moving) comp._poleWorld = velH / speedH;       // knees bend along motion

        // Distance-driven phase (one cycle = two steps): stride rate tracks speed, so
        // planted feet genuinely don't slide. Frozen when idle.
        comp._gaitPhase += (speedH * dt) / glm::max(2.0f * comp.strideLength, 1e-3f);
        comp._gaitPhase -= std::floor(comp._gaitPhase);

        // Model-space snapshot of the current (bind) pose.
        std::vector<glm::mat4> world;
        Diamond::ComputeWorldTransforms(skel, anim.pose, world);
        const glm::vec3 pelvisModel = glm::vec3(world[comp._pelvisIdx][3]);

        // ---- Diagnostics (gaitDebug) ----
        // shouldLog throttles the spdlog dump to ~4 Hz; draws submit every frame.
        // Actual (simulated) bone positions come from anim.palette -- in Powered mode
        // SyncRagdollPoses overwrote it from the physics bodies last frame, so
        // worldMat * palette[i] * inverse(inverseBind) is where the flesh REALLY is,
        // vs. world[] which is where the gait pose ASKS it to be.
        comp._dbgTimer += dt;
        const bool shouldLog = comp.gaitDebug && comp._dbgTimer >= 0.25f;
        if (shouldLog) comp._dbgTimer = 0.0f;
        glm::mat4 entityWorldMat { 1.0f };
        bool      haveActual = false;
        if (comp.gaitDebug) {
            entityWorldMat = scene.GetTransformSystem().GetWorldMatrix(entity);
            haveActual     = (int)anim.palette.size() == n;
            DebugDraw::Sphere(hipsW, 0.06f, { 1.0f, 1.0f, 0.2f });          // hips drive target
            DebugDraw::Line(hipsW, hipsW - glm::vec3(0, comp.hipHeight, 0),
                            { 1.0f, 1.0f, 0.2f });
        }
        if (shouldLog)
            spdlog::info("[Gait] anchor='{}' hipsTgt=({:.2f},{:.2f},{:.2f}) entityY={:.2f} hipH={:.2f} speed={:.2f} phase={:.2f} moving={}",
                         skel.bones[comp._pelvisIdx].name,
                         hipsW.x, hipsW.y, hipsW.z, xf.position.y, comp.hipHeight,
                         speedH, comp._gaitPhase, moving);

        struct LegRun { LocamotionControllerComponent::LegState& leg; float phaseOffset; const char* name; };
        LegRun legs[2] = { { comp._legL, 0.0f, "L" }, { comp._legR, 0.5f, "R" } };

        for (LegRun& lr : legs) {
            auto& leg = lr.leg;

            // Bind-pose stance offset from the pelvis (model space), resolved once.
            if (!leg.bindResolved) {
                const glm::vec3 bindFoot   = glm::vec3(glm::inverse(skel.bones[leg.tip].inverseBind)[3]);
                const glm::vec3 bindPelvis = glm::vec3(glm::inverse(skel.bones[comp._pelvisIdx].inverseBind)[3]);
                leg.stanceModel  = bindFoot - bindPelvis;
                leg.bindResolved = true;
            }

            // Stance point: the foot's bind offset swung to the current heading,
            // horizontally, grounded under the hips.
            glm::vec3 stanceOff = dirToWorld(leg.stanceModel);
            stanceOff.y = 0.0f;
            const glm::vec3 stanceProbe = hipsW + stanceOff;

            float legPhase = comp._gaitPhase + lr.phaseOffset;
            legPhase -= std::floor(legPhase);
            const bool planted = !moving || legPhase < comp.dutyFactor;

            float swingT = -1.0f;   // >=0 only while swinging (diagnostics)
            glm::vec3 targetW;
            if (!leg.init) {
                leg.planted      = GroundAt(scene, entity, stanceProbe, comp, hipsW);
                leg.lastSwingEnd = leg.planted;
                leg.init         = true;
                leg.wasPlanted   = true;
                targetW          = leg.planted;
            } else if (planted) {
                if (!leg.wasPlanted)
                    leg.planted = leg.lastSwingEnd;   // plant: pin where the swing ended
                if (!moving) {
                    // Idle: re-home a foot left far from its stance (e.g. after a stop
                    // mid-stride) instead of standing twisted forever.
                    const glm::vec3 home = GroundAt(scene, entity, stanceProbe, comp, hipsW);
                    if (glm::length(home - leg.planted) > comp.strideLength * 0.75f)
                        leg.planted = home;
                }
                targetW = leg.planted;
            } else {
                if (leg.wasPlanted)
                    leg.swingStart = leg.planted;     // liftoff
                const float t = glm::clamp((legPhase - comp.dutyFactor) /
                                           glm::max(1.0f - comp.dutyFactor, 1e-3f), 0.0f, 1.0f);
                // Predicted plant point, grounded there (follows slopes/steps). Ahead
                // by dutyFactor*strideLength so the plant is SYMMETRIC: the hips
                // travel duty*2*stride while a foot is down, so starting that far
                // ahead ends equally far behind (0.5*stride ended 40% overextended).
                const glm::vec3 dir = velH / speedH;
                const glm::vec3 plantPoint =
                    GroundAt(scene, entity, stanceProbe + dir * (comp.dutyFactor * comp.strideLength), comp, hipsW);
                const float ts = t * t * (3.0f - 2.0f * t);   // smoothstep travel
                targetW = glm::mix(leg.swingStart, plantPoint, ts)
                        + glm::vec3(0, 1, 0) * (comp.stepHeight * std::sin(glm::pi<float>() * t));
                leg.lastSwingEnd = plantPoint;
                swingT = t;
            }
            leg.wasPlanted = planted;

            // World -> model (un-yaw through the heading, anchored at the hips).
            const glm::vec3 targetM = pelvisModel + dirToModel(targetW - hipsW);

            const glm::vec3 a     = glm::vec3(world[leg.hip][3]);
            const glm::vec3 b     = glm::vec3(world[leg.knee][3]);
            const glm::vec3 c     = glm::vec3(world[leg.tip][3]);
            const glm::quat rootR = OrientationOf(world[leg.hip]);
            const glm::quat midR  = OrientationOf(world[leg.knee]);
            const glm::vec3 pole  = b + dirToModel(comp._poleWorld) * 0.5f;

            glm::quat newRootR, newMidR;
            Diamond::SolveTwoBone(a, b, c, rootR, midR, targetM, pole, newRootR, newMidR);

            // Solved MODEL-space rotations -> LOCAL, full weight (IKSystem's pattern).
            const int       hipParent  = skel.bones[leg.hip].parent;
            const glm::quat hipParentR = hipParent >= 0 ? OrientationOf(world[hipParent])
                                                        : glm::quat(1, 0, 0, 0);
            anim.pose[leg.hip].rotation  = glm::normalize(glm::conjugate(hipParentR) * newRootR);
            anim.pose[leg.knee].rotation = glm::normalize(glm::conjugate(newRootR)   * newMidR);

            // ---- Per-leg diagnostics ----
            if (comp.gaitDebug) {
                // Reachability of the commanded target: the solver clamps an
                // out-of-range target, so need > reach = leg fully extended,
                // pointing at a target it can never touch.
                const float reach = glm::length(b - a) + glm::length(c - b);
                const float need  = glm::length(targetM - a);

                // Where the simulated foot ACTUALLY is (physics readback palette).
                glm::vec3 actualFootW { 0.0f };
                if (haveActual) {
                    const glm::mat4 actualModel =
                        anim.palette[leg.tip] * glm::inverse(skel.bones[leg.tip].inverseBind);
                    actualFootW = glm::vec3(entityWorldMat * glm::vec4(glm::vec3(actualModel[3]), 1.0f));
                }

                const glm::vec3 tgtColor = swingT >= 0.0f ? glm::vec3(1.0f, 0.6f, 0.1f)   // swing
                                                          : glm::vec3(0.2f, 1.0f, 0.2f);  // planted
                DebugDraw::Sphere(targetW, 0.05f, tgtColor);
                if (haveActual) {
                    DebugDraw::Sphere(actualFootW, 0.04f, { 1.0f, 0.2f, 0.2f });
                    DebugDraw::Line(actualFootW, targetW, { 1.0f, 0.2f, 0.2f });
                }

                if (shouldLog) {
                    const float err = haveActual ? glm::length(actualFootW - targetW) : -1.0f;
                    spdlog::info("[Gait {}] {} ph={:.2f} tgtW=({:.2f},{:.2f},{:.2f}) need={:.2f} reach={:.2f}{} err={:.2f}",
                                 lr.name,
                                 swingT >= 0.0f ? "SWING" : "PLANT",
                                 legPhase, targetW.x, targetW.y, targetW.z,
                                 need, reach, need > reach ? " UNREACHABLE" : "", err);
                }
            }
        }
    }

    // World-space right/forward for the input basis. Prefers the primary camera's
    // CameraDirectorComponent::yawDeg (a static authored angle) over its live
    // TransformComponent -- CameraDirectorSystem runs AFTER this system (priority
    // 450 vs 100), so the live transform would be one frame stale, but the authored
    // yaw never is. Falls back to the camera's transform (a plain, non-director
    // camera), then to world axes if there's no camera at all.
    static void CameraRelativeBasis(Scene& scene, glm::vec3& outRight, glm::vec3& outForward)
    {
        outRight   = glm::vec3(1, 0, 0);
        outForward = glm::vec3(0, 0, -1);

        entt::entity cam = scene.GetPrimaryCamera();
        if (cam == entt::null) return;

        if (scene.Has<CameraDirectorComponent>(cam)) {
            const float yr = glm::radians(scene.Get<CameraDirectorComponent>(cam).yawDeg);
            outForward = glm::vec3(std::sin(yr), 0.0f, -std::cos(yr));
            outRight   = glm::vec3(std::cos(yr), 0.0f,  std::sin(yr));
        } else if (scene.Has<TransformComponent>(cam)) {
            auto& cxf = scene.Get<TransformComponent>(cam);
            outForward = cxf.rotation * glm::vec3(0, 0, -1);
            outRight   = cxf.rotation * glm::vec3(1, 0, 0);
            outForward.y = 0.0f;
            outForward = glm::length(outForward) > 1e-4f ? glm::normalize(outForward) : glm::vec3(0, 0, -1);
            outRight.y   = 0.0f;
            outRight   = glm::length(outRight)   > 1e-4f ? glm::normalize(outRight)   : glm::vec3(1, 0, 0);
        }
    }
};
