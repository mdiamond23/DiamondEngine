#pragma once
#include "Scene/Scripting.h"
#include "Scene/Scene.h"
#include "Scene/ComponentRegistry.h"
#include "Scene/Physics/PhysicsAPI.h"
#include "Scene/Physics/Collision.h"
#include "Core/Input.h"
#include "Animation/IKComponent.h"
#include "Animation/AnimationComponents.h"
#include "Animation/AnimationSampler.h"
#include "PlayerInput.h"
#include <imgui.h>
#include <nlohmann/json.hpp>
#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>
#include <string>
#include "Scene/SceneSystem.h"

// ---- Data -------------------------------------------------------------------
// Hold-to-punch, Gang Beasts style. Put this on a skinned character that runs the
// powered-ragdoll locomotion rig (SkinnedMesh + Animator (+ IK provisioned here)).
//
//   • LEFT BUMPER  — throw the LEFT arm straight forward. Hold to keep it out;
//                    release to pull it back to the animated pose.
//   • RIGHT BUMPER — same for the RIGHT arm.
//
// Each punch drives that arm's IK chain toward a point `reach` ahead of the
// shoulder bone along the rig's `modelForward` axis (MODEL space — CesiumMan faces
// +X). The target is built in the model frame on purpose: on this rig the entity
// transform never yaws (locomotion turns the physics hips directly), so a
// world-space "forward" would be stuck at the spawn orientation. A body-relative
// target follows the physical facing for free, because the pose the joint motors
// chase is itself body-relative. On a Powered ragdoll the IK only shapes that
// TARGET pose — the motors swing the physical arm after it, so the punch lands
// with real momentum and can be blocked/deflected by the world.
//
// Optional grab (disabled by default): while a bumper is held, a sphere cast leaves that hand along the punch
// direction. The first dynamic body it finds is pinned to a kinematic "hand proxy"
// (which tracks the IK hand) by a force-limited motorized joint (ConstraintType::Grab)
// — so every held punch doubles as a grab. The motor's force cap (`grabStrength`)
// makes WEIGHT MATTER: a body needs ~mass*g just to hover, so heavy things sag while
// light ones snap up. Rotation is free so the object dangles. Releasing the bumper
// destroys the joint (dropping the object) and retracts the arm. Both hands can grab
// — including the same object for a two-hand hold.
//
// `grabIgnoreHolder` (default on) drops held objects into the holder's collision group
// so they don't shove the character around. Turn it OFF to let held objects bash into
// the holder — a fun shove mechanic.

struct PunchComponent
{
    // Hand (end-effector) bones. NOTE CesiumMan's arm naming is asymmetric:
    // left arm is torso -> L__4_ -> L__3_ -> L__2_(hand), right is torso -> R -> R__2_ -> R__3_(hand).
    std::string leftHandBone  = "Skeleton_arm_joint_L__2_";
    std::string rightHandBone = "Skeleton_arm_joint_R__3_";

    // Model-space forward axis of the rig — the punch is thrown along this axis of
    // the character's BODY, so it tracks the physical facing automatically.
    // CesiumMan (Z-up, gait facingOffset 90°) faces +X; flip the sign if a rig
    // punches backward, or re-aim for rigs with a different convention.
    glm::vec3 modelForward { 1.0f, 0.0f, 0.0f };

    float reach          = 0.8f;   // how far ahead of the shoulder to plant the target (solver clamps to full extension)
    float punchEase      = 0.08f;  // IK weight/target smoothing time (s): lower = snappier jab
    // Instant kick (N*s) given to the fist body on the press edge, along the CURRENT
    // physical facing. This is what makes it a jab: the joint motors can follow and
    // hold the punch pose but can't launch the arm's inertia fast enough to read as
    // a punch — the impulse supplies the momentum, the motors ride it out and hold.
    // Sizing: fist+forearm ~2 kg effective, so J ~= 2*desired fist m/s. 12 ~= 6 m/s.
    float punchImpulse   = 12.0f;
    float shoulderHeight = 1.3f;   // FALLBACK anchor height above the entity origin (used only if the shoulder bone can't be resolved)

    // --- combat hit ---
    // A swept sphere follows the physical fist from its previous position to a short
    // distance ahead. Only an actively thrown, forward-moving fist can register, and
    // each press can damage at most one victim. Speed selects flinch vs knockdown;
    // impulse scales with that measured physical speed and is capped for stability.
    float hitRadius          = 0.13f;
    float hitForwardReach    = 0.10f;
    float flinchSpeed        = 1.25f;
    float knockdownSpeed     = 4.0f;
    float hitImpulsePerSpeed = 2.0f;
    float maxHitImpulse      = 20.0f;

    // --- grab ---
    bool  grabbingEnabled = false; // opt-in; default gameplay is pure punching
    float grabRadius   = 0.15f;    // sphere-cast radius for the grab probe
    float grabRange    = 0.35f;    // how far past the hand the grab probe sweeps
    float grabStrength = 800.0f;   // motor force budget (N). Lifts up to ~grabStrength/9.81 kg; heavier sags
    float grabResponse = 4.0f;     // motor spring frequency (Hz). Lower = looser/draggier (drunk), higher = snappier
    bool  grabIgnoreHolder = true; // held object ignores the holder's body (off = it can shove you — fun mechanic)

    // --- per-arm runtime (not serialized) ---
    struct Arm
    {
        entt::entity _handProxy = entt::null;                          // kinematic body tracking the IK hand (grab anchor)
        Physics::ConstraintHandle _grab = Physics::kInvalidConstraint; // live grab joint, or invalid when not holding
        entt::entity _grabbed   = entt::null;                          // the entity currently grabbed, or null
        uint32_t _objOrigGroup  = Physics::kUngroupedCollision;        // grabbed object's collision group before grab
        bool     _ownsObjGroup  = false;                               // this arm restores the object's group on release
        bool     _holderRef     = false;                               // this grab holds a ref on the holder's exclusion group
        glm::vec3 _previousHandPos { 0.0f };                           // previous physical fist sample for swept hits
        bool     _havePreviousHand = false;
        bool     _hitConsumed = true;                                  // one victim maximum per button press
        bool     _requiresRelease = false;                              // recovery cannot turn a held bumper into a new punch
    };
    Arm _arms[2];   // 0 = left, 1 = right

    // --- holder-level runtime (not serialized) ---
    // The holder's collision group is shared by both arms' grabs, so it's swapped in
    // when the FIRST exclusion is applied and restored when the LAST grab releases.
    uint32_t _holderOrigGroup = Physics::kUngroupedCollision;
    int      _holderRefs      = 0;
};

// ---- Inspector UI -----------------------------------------------------------

template<>
inline void DrawComponentInspector<PunchComponent>(PunchComponent& c)
{
    char bufL[128], bufR[128];
    std::snprintf(bufL, sizeof(bufL), "%s", c.leftHandBone.c_str());
    if (ImGui::InputText("Left Hand Bone", bufL, sizeof(bufL)))
        c.leftHandBone = bufL;
    std::snprintf(bufR, sizeof(bufR), "%s", c.rightHandBone.c_str());
    if (ImGui::InputText("Right Hand Bone", bufR, sizeof(bufR)))
        c.rightHandBone = bufR;
    ImGui::DragFloat3("Model Forward",  &c.modelForward.x, 0.01f, -1.0f, 1.0f);
    ImGui::DragFloat("Reach",           &c.reach,          0.01f, 0.0f,  5.0f);
    ImGui::DragFloat("Punch Ease",      &c.punchEase,      0.01f, 0.0f,  1.0f, "%.2f s");
    ImGui::DragFloat("Punch Impulse",   &c.punchImpulse,   1.0f,  0.0f,  500.0f, "%.0f N*s");
    ImGui::DragFloat("Shoulder Height", &c.shoulderHeight, 0.01f, 0.0f,  3.0f);
    ImGui::SeparatorText("Combat Hit");
    ImGui::DragFloat("Hit Radius",       &c.hitRadius,          0.01f, 0.01f, 1.0f, "%.2f m");
    ImGui::DragFloat("Hit Forward Reach",&c.hitForwardReach,    0.01f, 0.0f,  1.0f, "%.2f m");
    ImGui::DragFloat("Flinch Speed",     &c.flinchSpeed,        0.1f,  0.0f, 30.0f, "%.1f m/s");
    ImGui::DragFloat("Knockdown Speed",  &c.knockdownSpeed,     0.1f,  0.0f, 50.0f, "%.1f m/s");
    ImGui::DragFloat("Hit Impulse / Speed", &c.hitImpulsePerSpeed, 0.1f, 0.0f, 20.0f, "%.1f kg");
    ImGui::DragFloat("Max Hit Impulse",  &c.maxHitImpulse,      0.5f,  0.0f, 100.0f, "%.1f N*s");
    ImGui::SeparatorText("Grab");
    ImGui::Checkbox("Enable Grabbing", &c.grabbingEnabled);
    if (c.grabbingEnabled) {
        ImGui::DragFloat("Grab Radius",     &c.grabRadius,     0.01f, 0.0f,  1.0f);
        ImGui::DragFloat("Grab Range",      &c.grabRange,      0.01f, 0.0f,  2.0f);
        ImGui::DragFloat("Grab Strength",   &c.grabStrength,   5.0f,  0.0f, 10000.0f, "%.0f N");
        ImGui::DragFloat("Grab Response",   &c.grabResponse,   0.1f,  0.5f, 15.0f,    "%.1f Hz");
        ImGui::Checkbox("Ignore Holder Collision", &c.grabIgnoreHolder);
        ImGui::TextDisabled("Lifts up to ~%.0f kg; heavier sags", c.grabStrength / 9.81f);
    } else {
        ImGui::TextDisabled("Disabled: bumpers only punch.");
    }
    ImGui::TextDisabled("LB/RB = punch that arm (play mode); hold to keep it out");
    if (c.grabbingEnabled)
        ImGui::TextDisabled("A held punch grabs what it touches; release to drop");
}

// ---- Serialization ----------------------------------------------------------

template<>
inline std::string SerializeComponent<PunchComponent>(const PunchComponent& c)
{
    nlohmann::json j;
    j["leftHandBone"]     = c.leftHandBone;
    j["rightHandBone"]    = c.rightHandBone;
    j["modelForward"]     = { c.modelForward.x, c.modelForward.y, c.modelForward.z };
    j["reach"]            = c.reach;
    j["punchEase"]        = c.punchEase;
    j["punchImpulse"]     = c.punchImpulse;
    j["shoulderHeight"]   = c.shoulderHeight;
    j["hitRadius"]          = c.hitRadius;
    j["hitForwardReach"]    = c.hitForwardReach;
    j["flinchSpeed"]        = c.flinchSpeed;
    j["knockdownSpeed"]     = c.knockdownSpeed;
    j["hitImpulsePerSpeed"] = c.hitImpulsePerSpeed;
    j["maxHitImpulse"]      = c.maxHitImpulse;
    j["grabbingEnabled"]  = c.grabbingEnabled;
    j["grabRadius"]       = c.grabRadius;
    j["grabRange"]        = c.grabRange;
    j["grabStrength"]     = c.grabStrength;
    j["grabResponse"]     = c.grabResponse;
    j["grabIgnoreHolder"] = c.grabIgnoreHolder;
    return j.dump();
}

template<>
inline void DeserializeComponent<PunchComponent>(PunchComponent& c, const std::string& data)
{
    auto j             = nlohmann::json::parse(data);
    c.leftHandBone     = j.value("leftHandBone",  std::string("Skeleton_arm_joint_L__2_"));
    c.rightHandBone    = j.value("rightHandBone", std::string("Skeleton_arm_joint_R__3_"));
    if (auto it = j.find("modelForward"); it != j.end() && it->is_array() && it->size() == 3)
        c.modelForward = { (*it)[0].get<float>(), (*it)[1].get<float>(), (*it)[2].get<float>() };
    c.reach            = j.value("reach",          0.8f);
    c.punchEase        = j.value("punchEase",      0.08f);
    c.punchImpulse     = j.value("punchImpulse",   12.0f);
    c.shoulderHeight   = j.value("shoulderHeight", 1.3f);
    c.hitRadius          = j.value("hitRadius",          0.13f);
    c.hitForwardReach    = j.value("hitForwardReach",    0.10f);
    c.flinchSpeed        = j.value("flinchSpeed",        1.25f);
    c.knockdownSpeed     = j.value("knockdownSpeed",     4.0f);
    c.hitImpulsePerSpeed = j.value("hitImpulsePerSpeed", 2.0f);
    c.maxHitImpulse      = j.value("maxHitImpulse",      20.0f);
    c.grabbingEnabled  = j.value("grabbingEnabled", false);
    c.grabRadius       = j.value("grabRadius",     0.15f);
    c.grabRange        = j.value("grabRange",      0.35f);
    c.grabStrength     = j.value("grabStrength",   800.0f);
    c.grabResponse     = j.value("grabResponse",   4.0f);
    c.grabIgnoreHolder = j.value("grabIgnoreHolder", true);
}

// ---- Registration -----------------------------------------------------------

DECLARE_COMPONENT(PunchComponent, "Punch")

// ---- Behavior ---------------------------------------------------------------

class PunchSystem : public GameSystem
{
    DECLARE_SYSTEM(PunchSystem, 100)
public:
    void OnStart(Scene&) override
    {
        // Scene cycling remains an editor/demo shortcut owned by controller one.
        // Character actions are sampled per entity through PlayerInputComponent.
        Input::BindAction("ChangeScene", GamepadButton::South);
    }

    void OnUpdate(Scene& scene, float dt) override
    {
        // Cycle to the next listed scene (kNoScene wraps to 0). Empty list =
        // no-op: possible when playing an unsaved scene with no packager list.
        if (Input::IsPressed("ChangeScene") && !SceneSystem::SceneList().empty())
        {
            SceneSystem::LoadSceneByIndex(
                (SceneSystem::CurrentIndex() + 1) % SceneSystem::SceneList().size());
        }

        for (auto [entity, punch] : scene.View<PunchComponent>().each())
        {
            // An incapacitated character cannot keep an old punch pose alive or
            // launch attacks while physics owns its body. Clearing the previous-hand
            // sample also prevents the get-up/re-root displacement becoming a hit sweep.
            if (scene.Has<RagdollComponent>(entity)) {
                const auto& rag = scene.Get<RagdollComponent>(entity);
                const bool canAttack = rag.mode == RagdollMode::Powered
                    && Physics::GetRagdollActivity(rag)
                        == Physics::RagdollActivity::None;
                if (!canAttack) {
                    SuspendPunching(scene, entity, punch);
                    continue;
                }
            }

            const PlayerCommandState command = PlayerInput::ReadCommandOrDefault(scene, entity);
            const bool held[2] = {
                command.punchLeftHeld,
                command.punchRightHeld
            };
            const bool pressed[2] = {
                command.punchLeftPressed,
                command.punchRightPressed
            };

            // UpdateIK only runs on entities with a skinned mesh + animator; without
            // them there is no arm to drive.
            if (!scene.Has<SkinnedMeshComponent>(entity) || !scene.Has<AnimatorComponent>(entity))
                continue;

            // Model->world bone matrices from the last-solved TARGET pose, computed
            // ONCE per entity and shared by both arms (the IK anchor comes from it).
            // NOTE this frame ignores the physical heading (the entity transform
            // never yaws on a locomotion rig) — that's fine for the IK target, which
            // round-trips world->model through the same matrix inside UpdateIK.
            const glm::mat4 worldMat = scene.GetTransformSystem().GetWorldMatrix(entity);
            std::vector<glm::mat4> boneWorld;
            const bool havePose = ComputeBoneWorld(scene, entity, worldMat, boneWorld);

            // Punch direction in the (un-yawed) pose frame: the rig's model forward.
            // Body-relative, so the physical punch follows wherever the hips face.
            const glm::vec3 fwdLen  = glm::mat3(worldMat) * punch.modelForward;
            const glm::vec3 poseDir = glm::length(fwdLen) > 1e-4f
                ? glm::normalize(fwdLen) : glm::vec3(0, 0, -1);

            for (int side = 0; side < 2; ++side)
            {
                PunchComponent::Arm& arm = punch._arms[side];
                const std::string& handBone = side == 0 ? punch.leftHandBone : punch.rightHandBone;
                if (!held[side]) arm._requiresRelease = false;
                const bool punchHeld = held[side] && !arm._requiresRelease;
                const bool punchPressed = pressed[side] && !arm._requiresRelease;

                // Lazily provision an IK chain per hand bone the first time we see the
                // entity in play mode (an inspector-authored chain with a matching end
                // effector is adopted instead — we overwrite target/weight/ease each
                // frame either way). NOTE: use the returned pointer within this
                // iteration only — the next EnsureChain may grow the vector.
                IKChain* chain = EnsureChain(scene, entity, handBone, punch.punchEase);
                if (!chain) continue;
                chain->easeTime = punch.punchEase;

                // IK target: anchored at the arm chain's root (the shoulder bone, two
                // parents above the hand) so the punch rides the hip bob and spine
                // lean, thrown along modelForward. Falls back to a fixed height over
                // the entity origin if the pose isn't available.
                glm::vec3 anchor(0.0f);
                if (!havePose || !PoseShoulder(scene, entity, boneWorld, handBone, anchor)) {
                    if (scene.Has<TransformComponent>(entity))
                        anchor = scene.Get<TransformComponent>(entity).position;
                    anchor.y += punch.shoulderHeight;
                }

                chain->targetEntity   = entt::null;
                chain->targetWorldPos = anchor + poseDir * punch.reach;
                chain->weight         = punchHeld ? 1.0f : 0.0f;   // eased by punchEase

                // --- Grab (same bumper: a held punch grabs what it touches) ----
                // Grabbing happens in PHYSICAL space: the proxy and the probe use the
                // simulated hand read from the skinning palette (which carries the
                // ragdoll readback, heading included), NOT the target pose — the pose
                // hand ignores the physical yaw and motor lag, so anchoring there
                // would drift off the visible fist as the character turns.
                glm::vec3 handPos(0.0f), castDir = poseDir;
                const bool haveHand = PhysicalHand(scene, entity, worldMat, handBone, handPos, castDir);

                // Launch: kick the fist's momentum on the press edge (the motors then
                // ride it out to the IK pose and hold). Without this the punch reads
                // as a slow reach — see punchImpulse.
                if (punchPressed) {
                    arm._hitConsumed = false;
                    LaunchPunch(scene, entity, worldMat, handBone, punch.modelForward, punch.punchImpulse);
                }

                if (punchHeld && !arm._hitConsumed && haveHand
                    && chain->_curWeight > 0.10f)
                    TryHit(scene, entity, punch, arm, handBone, handPos, castDir);
                if (!punchHeld) arm._hitConsumed = true;

                if (punch.grabbingEnabled) {
                    entt::entity proxy = EnsureHandProxy(
                        scene, arm, side == 0 ? "HandProxyL" : "HandProxyR");
                    if (haveHand && proxy != entt::null && scene.Has<TransformComponent>(proxy))
                        scene.Get<TransformComponent>(proxy).position = handPos;

                    const bool proxyLive = proxy != entt::null
                        && scene.Has<RigidBodyComponent>(proxy)
                        && scene.Get<RigidBodyComponent>(proxy)._bodyId != 0xFFFFFFFFu;

                    if (!punchHeld) {
                        ReleaseGrab(scene, entity, punch, side);
                    } else if (arm._grab == Physics::kInvalidConstraint && haveHand
                               && proxyLive && chain->_curWeight > 0.25f) {
                        TryGrab(scene, entity, punch, side, castDir);
                    } else if (arm._grab != Physics::kInvalidConstraint
                               && (!scene.GetRegistry().valid(arm._grabbed)
                                   || !scene.Has<RigidBodyComponent>(arm._grabbed))) {
                        ReleaseGrab(scene, entity, punch, side);
                    }
                } else {
                    // Also handles disabling the option during play while holding an object.
                    ReleaseGrab(scene, entity, punch, side);
                }

                if (haveHand) {
                    arm._previousHandPos = handPos;
                    arm._havePreviousHand = true;
                } else {
                    arm._havePreviousHand = false;
                }
            }
        }
    }

    void OnDestroy(Scene& scene) override
    {
        // Drop anything still held (release joints and restore collision groups) so
        // nothing outlives the session.
        for (auto [entity, punch] : scene.View<PunchComponent>().each())
            for (int side = 0; side < 2; ++side)
                ReleaseGrab(scene, entity, punch, side);
    }

private:
    void SuspendPunching(Scene& scene, entt::entity entity,
                         PunchComponent& punch)
    {
        IKComponent* ik = scene.GetRegistry().try_get<IKComponent>(entity);
        for (int side = 0; side < 2; ++side) {
            PunchComponent::Arm& arm = punch._arms[side];
            arm._hitConsumed = true;
            arm._havePreviousHand = false;
            arm._requiresRelease = true;
            ReleaseGrab(scene, entity, punch, side);

            if (ik) {
                const std::string& handBone = side == 0
                    ? punch.leftHandBone : punch.rightHandBone;
                for (IKChain& chain : ik->chains)
                    if (chain.endEffectorBone == handBone)
                        chain.weight = 0.0f;
            }
        }
    }

    // Make sure the entity has an IKComponent with a chain for the given hand bone,
    // creating them if needed. Returns the chain to drive (valid until the next
    // EnsureChain call — the vector may reallocate).
    IKChain* EnsureChain(Scene& scene, entt::entity entity, const std::string& handBone, float ease)
    {
        if (!scene.Has<IKComponent>(entity))
            scene.Add<IKComponent>(entity);
        auto& ik = scene.Get<IKComponent>(entity);

        for (auto& c : ik.chains)
            if (c.endEffectorBone == handBone) return &c;

        IKChain c;
        c.endEffectorBone = handBone;
        c.weight          = 0.0f;
        c.easeTime        = ease;
        ik.chains.push_back(c);
        return &ik.chains.back();
    }

    // Lazily provision one arm's kinematic "hand proxy": a small sensor sphere that
    // follows the IK hand and anchors the grab joint. Created in play (so it isn't
    // serialized) and wiped when the scene restores on stop. Casts skip sensors, so
    // the proxies never block each other's (or their own) grab probes.
    entt::entity EnsureHandProxy(Scene& scene, PunchComponent::Arm& arm, const char* name)
    {
        if (arm._handProxy != entt::null && scene.GetRegistry().valid(arm._handProxy))
            return arm._handProxy;

        entt::entity proxy = scene.CreateEntity(name);
        auto& rb        = scene.Add<RigidBodyComponent>(proxy);
        rb.bodyType     = BodyType::Kinematic;   // driven by the transform we set each frame
        rb.gravityScale = 0.0f;

        auto& col     = scene.Add<ColliderComponent>(proxy);
        col.shapeType = CollisionShape::Sphere;
        col.radius    = 0.05f;
        col.isTrigger = true;   // sensor: anchors the joint without shoving the world

        arm._handProxy = proxy;
        return proxy;
    }

    // Model->world bone matrices from the (last-solved) TARGET pose. Mirrors the
    // model->world read in UpdateIK — deliberately excludes the physical heading,
    // matching the frame UpdateIK maps IK targets through. False if the entity
    // can't be posed this frame.
    static bool ComputeBoneWorld(Scene& scene, entt::entity entity, const glm::mat4& worldMat,
                                 std::vector<glm::mat4>& out)
    {
        auto& smc  = scene.Get<SkinnedMeshComponent>(entity);
        auto& anim = scene.Get<AnimatorComponent>(entity);

        const Diamond::Skeleton& skel = smc.skeleton;
        const int n = (int)skel.bones.size();
        if (n == 0 || (int)anim.pose.size() != n) return false;

        std::vector<glm::mat4> model;
        Diamond::ComputeWorldTransforms(skel, anim.pose, model);
        out.resize(model.size());
        for (size_t i = 0; i < model.size(); ++i) out[i] = worldMat * model[i];
        return true;
    }

    // Pose-frame position of one arm's shoulder (chain root: two parents above the
    // hand, matching the 2-bone IK chain). False if the hand bone doesn't resolve;
    // the shoulder falls back up the chain as far as the hierarchy allows.
    static bool PoseShoulder(Scene& scene, entt::entity entity, const std::vector<glm::mat4>& boneWorld,
                             const std::string& handBone, glm::vec3& shoulderOut)
    {
        const Diamond::Skeleton& skel = scene.Get<SkinnedMeshComponent>(entity).skeleton;
        const int tip = skel.Find(handBone);
        if (tip < 0 || tip >= (int)boneWorld.size()) return false;

        int shoulder = tip;
        for (int up = 0; up < 2; ++up) {
            const int p = skel.bones[shoulder].parent;
            if (p < 0 || p >= (int)boneWorld.size()) break;
            shoulder = p;
        }
        shoulderOut = glm::vec3(boneWorld[shoulder][3]);
        return true;
    }

    // Kick the fist body with an instant impulse along the CURRENT physical punch
    // direction: modelForward re-expressed through the chest bone's live (readback)
    // orientation, so a turned or leaning character launches the right way — the
    // un-yawed pose frame used for the IK target would fire in the spawn direction.
    static void LaunchPunch(Scene& scene, entt::entity entity, const glm::mat4& worldMat,
                            const std::string& handBone, glm::vec3 modelForward, float impulse)
    {
        if (impulse <= 0.0f || !scene.Has<RagdollComponent>(entity)) return;
        auto& smc  = scene.Get<SkinnedMeshComponent>(entity);
        auto& anim = scene.Get<AnimatorComponent>(entity);
        const Diamond::Skeleton& skel = smc.skeleton;
        const int n = (int)skel.bones.size();
        if (n == 0 || (int)anim.palette.size() != n) return;
        if (glm::length(modelForward) < 1e-4f) return;

        const int tip = skel.Find(handBone);
        if (tip < 0) return;
        // Chest = 3 parents above the hand (hand <- forearm <- upper arm <- chest).
        int chest = tip;
        for (int up = 0; up < 3 && skel.bones[chest].parent >= 0; ++up)
            chest = skel.bones[chest].parent;

        auto orient = [](const glm::mat4& m) {
            return glm::quat_cast(glm::mat3(glm::normalize(glm::vec3(m[0])),
                                            glm::normalize(glm::vec3(m[1])),
                                            glm::normalize(glm::vec3(m[2]))));
        };
        const glm::mat4 bindModel = glm::inverse(skel.bones[chest].inverseBind);
        const glm::quat rBind     = orient(bindModel);
        const glm::quat rNow      = orient(anim.palette[chest] * bindModel);

        // Bind-model forward -> chest-local -> current (physics-read) model -> world.
        const glm::vec3 dirModel = rNow * (glm::conjugate(rBind) * glm::normalize(modelForward));
        const glm::vec3 dirWorld = glm::mat3(worldMat) * dirModel;
        if (glm::length(dirWorld) < 1e-4f) return;

        Physics::AddRagdollBoneImpulse(scene.Get<RagdollComponent>(entity), tip,
                                       glm::normalize(dirWorld) * impulse);
    }

    // PHYSICAL hand: world position of the simulated hand plus the forearm's pointing
    // direction, read from the skinning palette — which SyncRagdollPoses rebuilds from
    // the ragdoll bodies every frame, so this is the visible fist (heading and motor
    // lag included), unlike the target pose. dirOut is left untouched (caller's
    // fallback) when the forearm direction degenerates.
    static bool PhysicalHand(Scene& scene, entt::entity entity, const glm::mat4& worldMat,
                             const std::string& handBone, glm::vec3& handOut, glm::vec3& dirOut)
    {
        auto& smc  = scene.Get<SkinnedMeshComponent>(entity);
        auto& anim = scene.Get<AnimatorComponent>(entity);
        const Diamond::Skeleton& skel = smc.skeleton;
        const int n = (int)skel.bones.size();
        if (n == 0 || (int)anim.palette.size() != n) return false;
        const int tip = skel.Find(handBone);
        if (tip < 0) return false;

        auto boneWorldPos = [&](int i) {
            const glm::mat4 model = anim.palette[i] * glm::inverse(skel.bones[i].inverseBind);
            return glm::vec3(worldMat * glm::vec4(glm::vec3(model[3]), 1.0f));
        };
        handOut = boneWorldPos(tip);
        if (const int mid = skel.bones[tip].parent; mid >= 0) {
            const glm::vec3 d = handOut - boneWorldPos(mid);
            if (glm::length(d) > 1e-4f) dirOut = glm::normalize(d);
        }
        return true;
    }

    // Sweep the physical fist between render-frame samples. This catches fast jabs
    // without turning the IK reach volume into a damage volume. The hand body's live
    // forward speed selects a stagger or knockdown, while _hitConsumed guarantees a
    // held bumper cannot repeatedly damage the same (or several) characters.
    static void TryHit(Scene& scene, entt::entity self, PunchComponent& punch,
                       PunchComponent::Arm& arm, const std::string& handBone,
                       glm::vec3 handPos, glm::vec3 forward)
    {
        if (!arm._havePreviousHand || !scene.Has<RagdollComponent>(self)) return;
        if (glm::length(forward) < 1e-4f) return;
        forward = glm::normalize(forward);

        const Diamond::Skeleton& skeleton = scene.Get<SkinnedMeshComponent>(self).skeleton;
        const int handBoneIndex = skeleton.Find(handBone);
        if (handBoneIndex < 0) return;

        bool velocityValid = false;
        const glm::vec3 handVelocity = Physics::GetRagdollBoneLinearVelocity(
            scene.Get<RagdollComponent>(self), handBoneIndex, &velocityValid);
        if (!velocityValid) return;

        const float forwardSpeed = glm::dot(handVelocity, forward);
        const float flinchSpeed = glm::max(punch.flinchSpeed, 0.0f);
        if (forwardSpeed < flinchSpeed) return;

        glm::vec3 start = arm._previousHandPos;
        // Ignore discontinuities from teleports, respawns, or a get-up re-root.
        if (glm::length(handPos - start) > 0.75f) start = handPos;
        const glm::vec3 trace = handPos
            + forward * glm::max(punch.hitForwardReach, 0.0f) - start;
        const float distance = glm::length(trace);
        if (distance < 1e-4f || punch.hitRadius <= 0.0f) return;

        const auto hits = Physics::SphereCastMulti(
            start, punch.hitRadius, trace / distance, distance, self);
        for (const HitResult& hit : hits) {
            if (hit.entity == entt::null || hit.entity == self
                || !scene.Has<RagdollComponent>(hit.entity))
                continue;

            const float knockdownSpeed = glm::max(punch.knockdownSpeed, flinchSpeed);
            const Physics::RagdollHitReaction reaction =
                forwardSpeed >= knockdownSpeed
                    ? Physics::RagdollHitReaction::Knockdown
                    : Physics::RagdollHitReaction::Flinch;
            const float impulseMagnitude = glm::clamp(
                forwardSpeed * glm::max(punch.hitImpulsePerSpeed, 0.0f),
                0.0f, glm::max(punch.maxHitImpulse, 0.0f));
            const Physics::RagdollImpactResult result = Physics::ApplyRagdollImpact(
                scene.Get<RagdollComponent>(hit.entity), hit.point,
                forward * impulseMagnitude, reaction);
            if (!result.applied) continue;

            arm._hitConsumed = true;
            spdlog::info(
                "Punch hit: attacker={} victim={} speed={:.2f}m/s impulse={:.2f}N*s "
                "reaction={} bone={}",
                entt::to_integral(self), entt::to_integral(hit.entity),
                forwardSpeed, impulseMagnitude,
                reaction == Physics::RagdollHitReaction::Knockdown
                    ? "knockdown" : "flinch",
                result.boneIndex);
            return;
        }
    }

    // Sphere-cast past the fist; pin the first dynamic body found to this arm's proxy
    // with a force-limited Grab joint, and (if enabled) exclude it from colliding with
    // the holder. The cast ignores only one entity, so we filter the rest here.
    void TryGrab(Scene& scene, entt::entity self, PunchComponent& punch, int side, glm::vec3 dir)
    {
        PunchComponent::Arm& arm = punch._arms[side];
        auto& proxyRb = scene.Get<RigidBodyComponent>(arm._handProxy);
        const glm::vec3 handPos = scene.Get<TransformComponent>(arm._handProxy).position;

        std::vector<HitResult> hits =
            Physics::SphereCastMulti(handPos, punch.grabRadius, dir, punch.grabRange, self);

        for (const HitResult& h : hits) {
            if (h.entity == self || h.entity == entt::null) continue;
            if (h.entity == punch._arms[0]._handProxy || h.entity == punch._arms[1]._handProxy) continue;
            if (!scene.Has<RigidBodyComponent>(h.entity)) continue;          // need a body to pin
            auto& targetRb = scene.Get<RigidBodyComponent>(h.entity);
            if (targetRb.bodyType != BodyType::Dynamic) continue;            // only grab free objects
            if (targetRb._bodyId == 0xFFFFFFFFu) continue;

            ConstraintComponent desc;
            desc.type           = ConstraintType::Grab;   // force-limited motorized pull, rotation free
            desc.anchor         = handPos;                 // body1/body2 frames coincide at the hand
            desc.motorMaxForce  = punch.grabStrength;
            desc.motorFrequency = punch.grabResponse;
            desc.motorDamping   = 1.0f;                    // critically damped

            Physics::ConstraintHandle handle = Physics::CreateConstraint(desc, proxyRb, targetRb);
            if (handle == Physics::kInvalidConstraint) return;
            arm._grab    = handle;
            arm._grabbed = h.entity;

            ApplyHolderExclusion(scene, self, punch, side);
            return;   // stop at the first viable candidate
        }
    }

    // Destroy this arm's grab joint and undo its share of the collision exclusion.
    // Idempotent. The holder's group is refcounted across both arms; the grabbed
    // object's group restore is owned by exactly one arm and handed off if the other
    // hand still holds the same object.
    void ReleaseGrab(Scene& scene, entt::entity self, PunchComponent& punch, int side)
    {
        PunchComponent::Arm& arm   = punch._arms[side];
        PunchComponent::Arm& other = punch._arms[side ^ 1];

        if (arm._ownsObjGroup) {
            arm._ownsObjGroup = false;
            if (other._grab != Physics::kInvalidConstraint && other._grabbed == arm._grabbed) {
                // Two-hand hold on the same object: the surviving hand takes over the restore.
                other._ownsObjGroup = true;
                other._objOrigGroup = arm._objOrigGroup;
            } else {
                RestoreGroup(scene, arm._grabbed, arm._objOrigGroup);
            }
        }
        if (arm._holderRef) {
            arm._holderRef = false;
            if (--punch._holderRefs <= 0) {
                punch._holderRefs = 0;
                RestoreGroup(scene, self, punch._holderOrigGroup);
            }
        }
        if (arm._grab != Physics::kInvalidConstraint) {
            Physics::ReleaseConstraint(arm._grab);
            arm._grab = Physics::kInvalidConstraint;
        }
        arm._grabbed = entt::null;
    }

    // Drop the grabbed object into the holder's collision group so the two stop
    // colliding (GroupExcludeFilter: same non-zero group = no collision). The holder's
    // original group is snapshotted on the FIRST exclusion (refcounted across arms);
    // the object's group is snapshotted unless the other hand already moved it.
    void ApplyHolderExclusion(Scene& scene, entt::entity self, PunchComponent& punch, int side)
    {
        PunchComponent::Arm& arm   = punch._arms[side];
        PunchComponent::Arm& other = punch._arms[side ^ 1];
        if (!punch.grabIgnoreHolder) return;
        if (!scene.Has<RigidBodyComponent>(self) || !scene.Has<RigidBodyComponent>(arm._grabbed)) return;

        auto& holderRb = scene.Get<RigidBodyComponent>(self);
        auto& objRb    = scene.Get<RigidBodyComponent>(arm._grabbed);
        if (holderRb._bodyId == 0xFFFFFFFFu || objRb._bodyId == 0xFFFFFFFFu) return;

        // Unique non-zero group for this holder, well clear of user groups and ragdolls (1000+).
        const uint32_t group = 0x40000000u + (uint32_t)entt::to_integral(self);

        if (punch._holderRefs == 0) {
            punch._holderOrigGroup = Physics::GetCollisionGroup(holderRb);
            Physics::SetCollisionGroup(holderRb, group);
        }
        ++punch._holderRefs;
        arm._holderRef = true;

        // Second hand on the same object: it's already in the holder group; the first
        // hand keeps ownership of the restore.
        const bool otherOwnsSameObj = other._ownsObjGroup && other._grabbed == arm._grabbed;
        if (!otherOwnsSameObj) {
            arm._objOrigGroup = Physics::GetCollisionGroup(objRb);
            arm._ownsObjGroup = true;
            Physics::SetCollisionGroup(objRb, group);
        }
    }

    // Put an entity's collision group back to its snapshot (ungrouped clears it).
    static void RestoreGroup(Scene& scene, entt::entity e, uint32_t group)
    {
        if (e == entt::null || !scene.GetRegistry().valid(e) || !scene.Has<RigidBodyComponent>(e)) return;
        auto& rb = scene.Get<RigidBodyComponent>(e);
        if (group == Physics::kUngroupedCollision) Physics::ClearCollisionGroup(rb);
        else                                       Physics::SetCollisionGroup(rb, group);
    }

};
