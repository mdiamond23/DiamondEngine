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
#include <imgui.h>
#include <nlohmann/json.hpp>
#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>
#include <string>
#include "Scene/SceneSystem.h"

// ---- Data -------------------------------------------------------------------
// A move-and-grab test, Gang Beasts style. Put this on a skinned character that
// also has an IKComponent with a left-arm chain (end effector = the hand bone).
// Arm movement and grabbing are on SEPARATE inputs so you can wave the hand around
// without it grabbing:
//   • LEFT STICK  — steers and extends the arm: push in a direction to reach that
//                   way (magnitude = how far it extends), center it to retract.
//   • LEFT TRIGGER — grab/release only. Hold while the arm is extended near an
//                    object to grab it; release to drop.
//
// Grab: a sphere cast leaves the hand along the reach direction. The first dynamic body
// it finds is pinned to a kinematic "hand proxy" (which tracks the IK hand) by a SOLVER-
// STABLE force-limited joint (a motorized SixDOF, ConstraintType::Grab). The motor's
// force cap (`grabStrength`) is what makes WEIGHT MATTER: a body needs ~mass*g just to
// hover, so heavy things sag and won't lift while light ones snap up. Rotation is free so
// the object dangles. Releasing the trigger destroys the joint and the object drops.
//
// `grabIgnoreHolder` (default on) drops the held object into the holder's collision group
// so it doesn't shove the character around. Turn it OFF to let held objects bash into the
// holder — a fun shove mechanic.

struct PunchComponent
{
    std::string handBone = "Skeleton_arm_joint_L__2_"; // IK chain to drive (matched by endEffectorBone)
    float reach          = 0.8f;   // how far in front to plant the target (solver clamps to full extension)
    float shoulderHeight = 1.3f;   // target anchor height above the entity origin (model units)

    // --- grab ---
    float grabRadius   = 0.15f;    // sphere-cast radius for the grab probe
    float grabRange    = 0.35f;    // how far past the hand the grab probe sweeps
    float grabStrength = 800.0f;   // motor force budget (N). Lifts up to ~grabStrength/9.81 kg; heavier sags
    float grabResponse = 4.0f;     // motor spring frequency (Hz). Lower = looser/draggier (drunk), higher = snappier
    bool  grabIgnoreHolder = true; // held object ignores the holder's body (off = it can shove you — fun mechanic)

    // --- runtime (not serialized) ---
    glm::vec3    _dir        { 0, 0, -1 };                          // current arm aim (world); follows the stick
    entt::entity _handProxy  = entt::null;                          // kinematic body tracking the IK hand (grab anchor)
    Physics::ConstraintHandle _grab = Physics::kInvalidConstraint;  // live grab joint, or invalid when not holding
    entt::entity _grabbed    = entt::null;                          // the entity currently grabbed, or null
    uint32_t     _objOrigGroup    = Physics::kUngroupedCollision;   // grabbed object's collision group before grab
    uint32_t     _holderOrigGroup = Physics::kUngroupedCollision;   // holder's collision group before grab
    bool         _excluded   = false;                              // whether collision exclusion is currently applied
};

// ---- Inspector UI -----------------------------------------------------------

template<>
inline void DrawComponentInspector<PunchComponent>(PunchComponent& c)
{
    char buf[128];
    std::snprintf(buf, sizeof(buf), "%s", c.handBone.c_str());
    if (ImGui::InputText("Hand Bone", buf, sizeof(buf)))
        c.handBone = buf;
    ImGui::DragFloat("Reach",           &c.reach,          0.01f, 0.0f, 5.0f);
    ImGui::DragFloat("Shoulder Height", &c.shoulderHeight, 0.01f, 0.0f, 3.0f);
    ImGui::DragFloat("Grab Radius",     &c.grabRadius,     0.01f, 0.0f, 1.0f);
    ImGui::DragFloat("Grab Range",      &c.grabRange,      0.01f, 0.0f, 2.0f);
    ImGui::DragFloat("Grab Strength",   &c.grabStrength,   5.0f,  0.0f, 10000.0f, "%.0f N");
    ImGui::DragFloat("Grab Response",   &c.grabResponse,   0.1f,  0.5f, 15.0f,    "%.1f Hz");
    ImGui::Checkbox("Ignore Holder Collision", &c.grabIgnoreHolder);
    ImGui::TextDisabled("Lifts up to ~%.0f kg; heavier sags", c.grabStrength / 9.81f);
    ImGui::TextDisabled("Left stick = move/extend arm (play mode)");
    ImGui::TextDisabled("Left trigger = grab while extended; release to drop");
}

// ---- Serialization ----------------------------------------------------------

template<>
inline std::string SerializeComponent<PunchComponent>(const PunchComponent& c)
{
    nlohmann::json j;
    j["handBone"]       = c.handBone;
    j["reach"]          = c.reach;
    j["shoulderHeight"] = c.shoulderHeight;
    j["grabRadius"]     = c.grabRadius;
    j["grabRange"]      = c.grabRange;
    j["grabStrength"]     = c.grabStrength;
    j["grabResponse"]     = c.grabResponse;
    j["grabIgnoreHolder"] = c.grabIgnoreHolder;
    return j.dump();
}

template<>
inline void DeserializeComponent<PunchComponent>(PunchComponent& c, const std::string& data)
{
    auto j           = nlohmann::json::parse(data);
    c.handBone       = j.value("handBone",       std::string("Skeleton_arm_joint_L__2_"));
    c.reach          = j.value("reach",          0.8f);
    c.shoulderHeight = j.value("shoulderHeight", 1.3f);
    c.grabRadius     = j.value("grabRadius",     0.15f);
    c.grabRange      = j.value("grabRange",      0.35f);
    c.grabStrength   = j.value("grabStrength",   800.0f);
    c.grabResponse   = j.value("grabResponse",   4.0f);
    c.grabIgnoreHolder = j.value("grabIgnoreHolder", true);
}

// ---- Registration -----------------------------------------------------------

DECLARE_COMPONENT(PunchComponent, "Punch")

// ---- Behavior ---------------------------------------------------------------

class PunchSystem : public GameSystem
{
    DECLARE_SYSTEM(PunchSystem, 100)
public:
    void OnStart(Scene& scene) override
    {
        Input::BindAxis("ArmX",        GamepadAxis::LeftX);
        Input::BindAxis("ArmY",        GamepadAxis::LeftY);
        Input::BindAxis("GrabTrigger", GamepadAxis::LeftTrigger);
        Input::BindAction("ChangeScene", GamepadButton::South);
    }

    void OnUpdate(Scene& scene, float dt) override
    {
        constexpr float kDeadzone = 0.2f;   // ignore stick noise / rest drift

        // Cycle to the next listed scene (kNoScene wraps to 0). Empty list =
        // no-op: possible when playing an unsaved scene with no packager list.
        if (Input::IsPressed("ChangeScene") && !SceneSystem::SceneList().empty())
        {
            SceneSystem::LoadSceneByIndex(
                (SceneSystem::CurrentIndex() + 1) % SceneSystem::SceneList().size());
        }

        const float h        = Input::GetAxis("ArmX");
        const float f        = Input::GetAxis("ArmY");
        const float mag      = glm::min(glm::length(glm::vec2(h, f)), 1.0f);
        const bool  grabHeld = Input::GetAxis("GrabTrigger") > 0.5f;

        // Stick push past the deadzone, remapped to 0..1 — drives how far the arm
        // extends. Centered stick = 0 = arm eases back to the animated pose.
        const float extend = mag > kDeadzone ? (mag - kDeadzone) / (1.0f - kDeadzone) : 0.0f;

        for (auto [entity, punch] : scene.View<PunchComponent>().each())
        {
            // The IK chain is normally authored in the inspector, but IKComponent is
            // not serialized — so for this test we lazily provision it (and a chain for
            // the configured hand bone) the first time we see the entity in play mode.
            IKChain* chain = EnsureChain(scene, entity, punch);
            if (!chain) continue;

            // The LEFT STICK steers and extends the arm every frame, independent of the
            // grab trigger: re-aim only while pushing (so a released stick retracts along
            // the last direction rather than snapping forward).
            if (extend > 0.0f)
                punch._dir = PunchDir(scene, entity, h, f);

            glm::vec3 anchor = glm::vec3(0.0f);
            if (scene.Has<TransformComponent>(entity))
                anchor = scene.Get<TransformComponent>(entity).position;
            anchor.y += punch.shoulderHeight;

            chain->targetEntity   = entt::null;
            chain->targetWorldPos = anchor + punch._dir * punch.reach;
            chain->weight         = extend;   // eased toward by the chain's easeTime

            // --- Grab (LEFT TRIGGER, independent of arm movement) --------------
            // The hand proxy is a kinematic body glued to the (last-solved) IK hand each
            // frame; the grab joint's force-limited motor drags the object toward it.
            entt::entity proxy = EnsureHandProxy(scene, punch);

            glm::vec3 handPos;
            const bool haveHand = HandWorldPos(scene, entity, punch.handBone, handPos);
            if (haveHand && proxy != entt::null && scene.Has<TransformComponent>(proxy))
                scene.Get<TransformComponent>(proxy).position = handPos;

            const bool proxyLive = proxy != entt::null && scene.Has<RigidBodyComponent>(proxy) &&
                                   scene.Get<RigidBodyComponent>(proxy)._bodyId != 0xFFFFFFFFu;

            if (!grabHeld) {
                ReleaseGrab(scene, entity, punch);   // releasing the trigger drops the object
            } else if (punch._grab == Physics::kInvalidConstraint && haveHand && proxyLive &&
                       chain->_curWeight > 0.8f) {
                // Trigger held and the arm is extended near a target — probe for a grab.
                TryGrab(scene, entity, proxy, punch);
            } else if (punch._grab != Physics::kInvalidConstraint &&
                       (!scene.GetRegistry().valid(punch._grabbed) ||
                        !scene.Has<RigidBodyComponent>(punch._grabbed))) {
                ReleaseGrab(scene, entity, punch);   // grabbed entity went away
            }
        }
    }

    void OnDestroy(Scene& scene) override
    {
        // Drop anything still held (release the joint and restore collision groups) so it
        // doesn't outlive the session.
        for (auto [entity, punch] : scene.View<PunchComponent>().each())
            ReleaseGrab(scene, entity, punch);
    }

private:
    // Make sure the entity has an IKComponent with a chain for the punch hand bone,
    // creating them if needed. Returns the chain to drive, or nullptr if the entity
    // can't run IK at all (no skinned mesh / animator). IKComponent isn't serialized,
    // so the chain is provisioned here rather than authored in the scene.
    IKChain* EnsureChain(Scene& scene, entt::entity entity, PunchComponent& punch)
    {
        // UpdateIK only runs on entities with a skinned mesh + animator; without them
        // a chain would never be solved.
        if (!scene.Has<SkinnedMeshComponent>(entity) || !scene.Has<AnimatorComponent>(entity))
            return nullptr;

        if (!scene.Has<IKComponent>(entity))
            scene.Add<IKComponent>(entity);
        auto& ik = scene.Get<IKComponent>(entity);

        for (auto& c : ik.chains)
            if (c.endEffectorBone == punch.handBone) return &c;

        // No chain for this bone yet — make one. easeTime gives the extend/retract feel.
        IKChain c;
        c.endEffectorBone = punch.handBone;
        c.weight          = 0.0f;
        c.easeTime        = 0.10f;
        ik.chains.push_back(c);
        return &ik.chains.back();
    }

    // Lazily provision the kinematic "hand proxy": a small sensor sphere that follows the
    // IK hand and serves as the grab joint's anchor body. Created in play (so it isn't
    // serialized) and wiped when the scene restores on stop.
    entt::entity EnsureHandProxy(Scene& scene, PunchComponent& punch)
    {
        if (punch._handProxy != entt::null && scene.GetRegistry().valid(punch._handProxy))
            return punch._handProxy;

        entt::entity proxy = scene.CreateEntity("HandProxy");
        auto& rb        = scene.Add<RigidBodyComponent>(proxy);
        rb.bodyType     = BodyType::Kinematic;   // driven by the transform we set each frame
        rb.gravityScale = 0.0f;

        auto& col     = scene.Add<ColliderComponent>(proxy);
        col.shapeType = CollisionShape::Sphere;
        col.radius    = 0.05f;
        col.isTrigger = true;   // sensor: anchors the joint without shoving the world

        punch._handProxy = proxy;
        return proxy;
    }

    // World-space position of the hand bone's origin, read from the (last-solved)
    // animated pose. Mirrors the model->world read in UpdateIK. Returns false if the
    // entity can't be skinned/posed or the bone name doesn't resolve.
    static bool HandWorldPos(Scene& scene, entt::entity entity, const std::string& handBone,
                             glm::vec3& out)
    {
        if (!scene.Has<SkinnedMeshComponent>(entity) || !scene.Has<AnimatorComponent>(entity))
            return false;
        auto& smc  = scene.Get<SkinnedMeshComponent>(entity);
        auto& anim = scene.Get<AnimatorComponent>(entity);

        const Diamond::Skeleton& skel = smc.skeleton;
        const int n = (int)skel.bones.size();
        if (n == 0 || (int)anim.pose.size() != n) return false;

        const int tip = skel.Find(handBone);
        if (tip < 0) return false;

        std::vector<glm::mat4> world;
        Diamond::ComputeWorldTransforms(skel, anim.pose, world);
        const glm::vec3 handModel = glm::vec3(world[tip][3]);
        const glm::mat4 worldMat  = scene.GetTransformSystem().GetWorldMatrix(entity);
        out = glm::vec3(worldMat * glm::vec4(handModel, 1.0f));
        return true;
    }

    // Sphere-cast from the hand; pin the first dynamic body found to the proxy with a
    // force-limited Grab joint, and (if enabled) exclude it from colliding with the holder.
    // Skips ourselves and the proxy (the cast ignores only one entity, so we filter here).
    void TryGrab(Scene& scene, entt::entity self, entt::entity proxy, PunchComponent& punch)
    {
        auto& proxyRb = scene.Get<RigidBodyComponent>(proxy);
        const glm::vec3 handPos = scene.Get<TransformComponent>(proxy).position;

        std::vector<HitResult> hits =
            Physics::SphereCastMulti(handPos, punch.grabRadius, punch._dir, punch.grabRange, self);

        for (const HitResult& h : hits) {
            if (h.entity == self || h.entity == proxy || h.entity == entt::null) continue;
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
            punch._grab    = handle;
            punch._grabbed = h.entity;

            ApplyHolderExclusion(scene, self, punch);
            return;   // stop at the first viable candidate
        }
    }

    // Destroy the live grab joint and undo any collision exclusion. Idempotent.
    void ReleaseGrab(Scene& scene, entt::entity self, PunchComponent& punch)
    {
        RemoveHolderExclusion(scene, self, punch);
        if (punch._grab != Physics::kInvalidConstraint) {
            Physics::ReleaseConstraint(punch._grab);
            punch._grab = Physics::kInvalidConstraint;
        }
        punch._grabbed = entt::null;
    }

    // Drop the grabbed object into the holder's collision group so the two stop colliding
    // (GroupExcludeFilter: same non-zero group = no collision). Snapshots both groups so
    // RemoveHolderExclusion can put them back. No-op if disabled or the holder has no body.
    void ApplyHolderExclusion(Scene& scene, entt::entity self, PunchComponent& punch)
    {
        punch._excluded = false;
        if (!punch.grabIgnoreHolder) return;
        if (!scene.Has<RigidBodyComponent>(self) || !scene.Has<RigidBodyComponent>(punch._grabbed)) return;

        auto& holderRb = scene.Get<RigidBodyComponent>(self);
        auto& objRb    = scene.Get<RigidBodyComponent>(punch._grabbed);
        if (holderRb._bodyId == 0xFFFFFFFFu || objRb._bodyId == 0xFFFFFFFFu) return;

        punch._objOrigGroup    = Physics::GetCollisionGroup(objRb);
        punch._holderOrigGroup = Physics::GetCollisionGroup(holderRb);

        // Unique non-zero group for this holder, well clear of user groups and ragdolls (1000+).
        const uint32_t group = 0x40000000u + (uint32_t)entt::to_integral(self);
        Physics::SetCollisionGroup(holderRb, group);
        Physics::SetCollisionGroup(objRb,    group);
        punch._excluded = true;
    }

    // Restore the collision groups snapshotted at grab time (back to ungrouped if they
    // were ungrouped, so the object collides with the world normally again).
    void RemoveHolderExclusion(Scene& scene, entt::entity self, PunchComponent& punch)
    {
        if (!punch._excluded) return;
        punch._excluded = false;

        auto restore = [](Scene& s, entt::entity e, uint32_t group) {
            if (e == entt::null || !s.GetRegistry().valid(e) || !s.Has<RigidBodyComponent>(e)) return;
            auto& rb = s.Get<RigidBodyComponent>(e);
            if (group == Physics::kUngroupedCollision) Physics::ClearCollisionGroup(rb);
            else                                       Physics::SetCollisionGroup(rb, group);
        };
        restore(scene, self,          punch._holderOrigGroup);
        restore(scene, punch._grabbed, punch._objOrigGroup);
    }

    // World-space punch direction from the left-stick tilt, oriented by the entity's
    // yaw (same flatten convention as MoveSystem). Defaults to forward when centered.
    static glm::vec3 PunchDir(Scene& scene, entt::entity entity, float h, float f)
    {
        glm::vec3 right(1, 0, 0), forward(0, 0, -1);
        if (scene.Has<TransformComponent>(entity)) {
            auto& xf = scene.Get<TransformComponent>(entity);
            right   = xf.rotation * glm::vec3(1, 0,  0);
            forward = xf.rotation * glm::vec3(0, 0, -1);
            right.y   = 0.0f; right   = glm::length(right)   > 1e-4f ? glm::normalize(right)   : glm::vec3(1, 0, 0);
            forward.y = 0.0f; forward = glm::length(forward) > 1e-4f ? glm::normalize(forward) : glm::vec3(0, 0, -1);
        }
        glm::vec3 dir = right * h + forward * f;
        return glm::length(dir) > 1e-3f ? glm::normalize(dir) : forward;
    }
};
