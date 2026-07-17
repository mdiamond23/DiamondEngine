#pragma once
#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>
#include <entt/entt.hpp>
#include <vector>
#include "Rigidbody.h"
#include "Constraint.h"
#include "RagdollComponent.h"

class Scene;

struct HitResult {
    bool         hit      = false;
    float        distance = 0.0f;
    entt::entity entity   = entt::null;
    glm::vec3    point    { 0.0f };
    glm::vec3    normal   { 0.0f };
    glm::vec3    traceStart { 0.0f };
    glm::vec3    traceEnd   { 0.0f };
};

// Physics manipulation — valid only during play mode (between OnStart and OnDestroy).
// All functions are silent no-ops if the physics session is not active or the body
// handle is invalid.  Include this header in game scripts that need to push bodies around.
namespace Physics {

    // ---- Forces (accumulated each step, zeroed after application) ----
    void AddForce(const RigidBodyComponent& rb, glm::vec3 force);
    void AddForceAtPoint(const RigidBodyComponent& rb, glm::vec3 force, glm::vec3 worldPoint);
    void AddTorque(const RigidBodyComponent& rb, glm::vec3 torque);

    // ---- Impulses (instant velocity change) ----
    void AddImpulse(const RigidBodyComponent& rb, glm::vec3 impulse);
    void AddImpulseAtPoint(const RigidBodyComponent& rb, glm::vec3 impulse, glm::vec3 worldPoint);
    void AddAngularImpulse(const RigidBodyComponent& rb, glm::vec3 impulse);

    // ---- Velocity ----
    void      SetLinearVelocity(const RigidBodyComponent& rb, glm::vec3 velocity);
    glm::vec3 GetLinearVelocity(const RigidBodyComponent& rb);
    void      SetAngularVelocity(const RigidBodyComponent& rb, glm::vec3 angularVelocity);
    glm::vec3 GetAngularVelocity(const RigidBodyComponent& rb);

    // ---- Sleep state ----
    void Activate(const RigidBodyComponent& rb);    // wake a sleeping body
    void Deactivate(const RigidBodyComponent& rb);  // allow body to sleep

    // ---- World settings ----
    // Default: (0, -9.81, 0). No-op if physics session is not active.
    void SetGravity(glm::vec3 gravity);

    // ---- Runtime constraints ----
    // Opaque handle to a joint built at runtime via CreateConstraint. Compare
    // against kInvalidConstraint to detect a failed build.
    using ConstraintHandle = uint32_t;
    inline constexpr ConstraintHandle kInvalidConstraint = 0xFFFFFFFFu;

    // Build a physics joint at runtime between two live bodies, described by `desc`
    // — the same fields the editor authors on a ConstraintComponent (type, world-space
    // anchor/axis, limits, motor). `self` is body 2, `target` is body 1, matching the
    // component path's convention. Returns a handle to release later, or
    // kInvalidConstraint if the session is inactive or either body isn't live.
    //
    // Unlike a ConstraintComponent, this joint is NOT a component and is NOT serialized —
    // it lives only until ReleaseConstraint, or until either body is destroyed (which
    // tears it down automatically). Built for transient runtime joints like grabs: e.g.
    // a Point constraint from a grabbed body to a hand-proxy body.
    ConstraintHandle CreateConstraint(const ConstraintComponent& desc,
                                      const RigidBodyComponent& self,
                                      const RigidBodyComponent& target);

    // As above, but anchors `self` to the immovable static world (Body::sFixedToWorld).
    ConstraintHandle CreateConstraintToWorld(const ConstraintComponent& desc,
                                             const RigidBodyComponent& self);

    // Destroy a runtime constraint created by CreateConstraint. Silent no-op if the
    // handle is invalid or already gone (e.g. a referenced body was destroyed). Safe to
    // call once per handle; handles are never reused, so a stale handle never aliases.
    void ReleaseConstraint(ConstraintHandle handle);

    // ---- Collision groups (runtime) ----
    // Bodies sharing the same NON-ZERO group never collide (engine GroupExcludeFilter).
    // Use to make a held object ignore its holder: snapshot both groups with
    // GetCollisionGroup, SetCollisionGroup both to one shared non-zero id while held, then
    // restore on release — SetCollisionGroup if the snapshot was a real group, else
    // ClearCollisionGroup to go back to ungrouped. kUngroupedCollision (0xFFFFFFFF) is the
    // value GetCollisionGroup returns for an ungrouped body.
    inline constexpr uint32_t kUngroupedCollision = 0xFFFFFFFFu;
    uint32_t GetCollisionGroup(const RigidBodyComponent& rb);
    void     SetCollisionGroup(const RigidBodyComponent& rb, uint32_t group);
    void     ClearCollisionGroup(const RigidBodyComponent& rb);   // back to ungrouped

    // ---- Constraint motors ----
    // Drives a hinge motor's live target each frame. The motor's mode (Velocity
    // or Position) is configured on the ConstraintComponent; this sets the target
    // in that mode's units — degrees (Position) or deg/s (Velocity). Silent no-op
    // if the constraint isn't built yet, isn't a hinge, or has no motor enabled.
    void SetMotorTarget(const ConstraintComponent& cc, float target);

    // Drives a swing-twist motor's live target orientation each frame. targetBS is
    // the desired orientation of body 2 relative to body 1 (the bone's local
    // rotation) — the form an animation pose provides; identity = aligned to the
    // parent's frame. Silent no-op if the constraint isn't built, isn't a
    // swing-twist, or has no motor enabled.
    void SetMotorTargetOrientation(const ConstraintComponent& cc, glm::quat targetBS);

    // ---- Raycasting ----
    // Casts a ray from origin along direction for up to distance units.
    // Only hits entities with a ColliderComponent registered in the physics world.
    // Returns a HitResult with hit=false if nothing is hit or the session is not active.
    HitResult Raycast(glm::vec3 origin, glm::vec3 direction, float distance,
                      entt::entity ignore = entt::null, bool drawDebug = false);

    // Returns all hits along the ray sorted nearest-first. Empty if nothing hit.
    std::vector<HitResult> RaycastMulti(glm::vec3 origin, glm::vec3 direction, float distance,
                                        entt::entity ignore = entt::null, bool drawDebug = false);

    HitResult SphereCast(glm::vec3 origin, float radius, glm::vec3 direction, float distance,
                         entt::entity ignore = entt::null, bool drawDebug = false);

    // Returns all hits along the sweep sorted nearest-first. Empty if nothing hit.
    std::vector<HitResult> SphereCastMulti(glm::vec3 origin, float radius, glm::vec3 direction, float distance,
                                           entt::entity ignore = entt::null, bool drawDebug = false);

    // ---- Ragdoll ----
    // Flip a built ragdoll between Animated (kinematic, shadows the animation), Limp
    // (dynamic, flops passively) and Powered (dynamic, joint motors fight toward the
    // animation pose — active ragdoll). See Docs/ragdoll-design.md. No-op if the
    // ragdoll hasn't been built (play start) or the session isn't active.
    void SetRagdollMode(RagdollComponent& rag, RagdollMode mode);

    // Set a Powered ragdoll's muscle strength in [0,1] — scales every joint motor's
    // torque budget. 1 = full authored strength, 0 = effectively limp. The knob for
    // hit-reactions and get-up blends. Only affects Powered mode.
    void SetRagdollStrength(RagdollComponent& rag, float strength);

    // The rig's standing hip clearance, measured from the .ragdoll's bind pose at
    // build time (hips body height above the lowest body). The natural hip-height
    // target for a locomotion controller -- the same reference the procedural get-up
    // stands back up to. Returns 0 until the ragdoll is built (play start), so poll
    // it from OnUpdate and treat 0 as "not ready yet".
    float GetRagdollStandHipClearance(const RagdollComponent& rag);

    // Skeleton bone index of the ragdoll's ROOT body (the hips body the locomotion/
    // get-up root drives move). -1 until the ragdoll is built. A locomotion
    // controller must anchor its world<->model foot-target math on exactly this
    // bone -- guessing a "pelvis" by name anchors the mapping on the wrong joint
    // and displaces every target by the bone offset.
    int GetRagdollRootBone(const RagdollComponent& rag);

    // Start a procedural get-up: the rig heaves itself upright by driving its motors
    // toward the bind/stand pose (cones opened, strength ramping up with wobble), then
    // hands back to Animated. Works from Limp (the usual trigger) or any built ragdoll.
    // Also fires automatically once a limp rig settles (RagdollConfig::getupDelay). No-op
    // if the ragdoll isn't built. See Docs/ragdoll-design.md.
    void RagdollGetUp(RagdollComponent& rag);

    // Overwrites the bone palette of every Limp ragdoll from its simulated bodies
    // (the animation->physics readback) and re-roots the entity to the hips. MUST be
    // called once per frame AFTER UpdateAnimators (so it isn't clobbered) and after
    // the physics step. No-op when no ragdoll is limp.
    void SyncRagdollPoses(Scene& scene);

    // ---- Debug visualization ----
    // Submits wireframe collider shapes for all entities in the scene to DebugDraw.
    // Call Physics::DrawColliders(scene) then DebugDraw::Flush(vp) each frame.
    // Works in both edit and play mode — does not require an active physics session.
    void DrawColliders(Scene& scene, glm::vec3 color = {0.0f, 1.0f, 0.0f});

    // Wireframes for ragdoll bodies (which aren't ColliderComponents). Play mode only.
    void DrawRagdolls(Scene& scene, glm::vec3 color = {1.0f, 0.4f, 0.0f});
}
