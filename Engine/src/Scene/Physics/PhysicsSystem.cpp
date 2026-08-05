// Jolt must be included before anything else that might define conflicting macros
#include <Jolt/Jolt.h>
#include <Jolt/RegisterTypes.h>
#include <Jolt/Core/Factory.h>
#include <Jolt/Core/TempAllocator.h>
#include <Jolt/Core/JobSystemThreadPool.h>
#include <Jolt/Physics/PhysicsSystem.h>
#include <Jolt/Physics/Body/BodyCreationSettings.h>
#include <Jolt/Physics/Body/BodyActivationListener.h>
#include <Jolt/Physics/Collision/ContactListener.h>
#include <Jolt/Physics/Collision/Shape/BoxShape.h>
#include <Jolt/Physics/Collision/Shape/SphereShape.h>
#include <Jolt/Physics/Collision/Shape/CapsuleShape.h>
#include <Jolt/Physics/Collision/Shape/ConvexHullShape.h>
#include <Jolt/Physics/Collision/Shape/MeshShape.h>
#include <Jolt/Physics/Collision/Shape/RotatedTranslatedShape.h>
#include <Jolt/Physics/Collision/RayCast.h>
#include <Jolt/Physics/Collision/CastResult.h>
#include <Jolt/Physics/Collision/ShapeCast.h>
#include <Jolt/Physics/Collision/CollisionCollectorImpl.h>
#include <Jolt/Physics/Collision/CollisionGroup.h>
#include <Jolt/Physics/Collision/GroupFilter.h>
#include <Jolt/Physics/Body/BodyLockInterface.h>
#include <Jolt/Physics/Body/BodyLock.h>
#include <Jolt/Physics/Body/BodyLockMulti.h>
#include <Jolt/Physics/Body/Body.h>
#include <Jolt/Physics/Body/MotionProperties.h>
#include <Jolt/Physics/Constraints/Constraint.h>
#include <Jolt/Physics/Constraints/TwoBodyConstraint.h>
#include <Jolt/Physics/Constraints/HingeConstraint.h>
#include <Jolt/Physics/Constraints/SwingTwistConstraint.h>
#include <Jolt/Physics/Constraints/FixedConstraint.h>
#include <Jolt/Physics/Constraints/PointConstraint.h>
#include <Jolt/Physics/Constraints/SixDOFConstraint.h>

#include "Scene/Physics/PhysicsSystem.h"
#include "Scene/Physics/PhysicsAPI.h"
#include "Scene/Physics/Rigidbody.h"
#include "Scene/Physics/Collision.h"
#include "Scene/Physics/Constraint.h"
#include "Scene/Physics/RagdollComponent.h"
#include "Scene/Components.h"
#include "Scene/Scene.h"
#include "Animation/AnimationComponents.h"   // SkinnedMeshComponent + AnimatorComponent (ragdoll readback)
#include "DebugDraw.h"
#include "Assets/ModelImporter.h"
#include "Profiling/CPUProfiler.h"

#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>
#include <glm/gtc/matrix_transform.hpp>

#include <spdlog/spdlog.h>

#include <unordered_map>
#include <unordered_set>
#include <vector>
#include <mutex>
#include <utility>
#include <cstdint>
#include <algorithm>
#include <cmath>
#include <thread>

// ---------------------------------------------------------------------------
// Constants
// ---------------------------------------------------------------------------
static constexpr float     FIXED_DT               = 1.0f / 60.0f;
static constexpr JPH::uint cMaxBodies             = 10240;
static constexpr JPH::uint cNumBodyMutexes        = 0;      // 0 = Jolt picks a sensible default
static constexpr JPH::uint cMaxBodyPairs          = 65536;
static constexpr JPH::uint cMaxContactConstraints = 10240;

// ---------------------------------------------------------------------------
// Physics layers
//
// Object layers  : STATIC (immovable), DYNAMIC (simulated + kinematic)
// Broad-phase layers : NON_MOVING, MOVING  (Jolt optimisation buckets)
// ---------------------------------------------------------------------------
namespace PhysicsLayers {
    static constexpr JPH::ObjectLayer STATIC    = 0;
    static constexpr JPH::ObjectLayer DYNAMIC   = 1;
    static constexpr JPH::uint        NUM_LAYERS = 2;
}

namespace BPLayers {
    static constexpr JPH::BroadPhaseLayer NON_MOVING { 0 };
    static constexpr JPH::BroadPhaseLayer MOVING     { 1 };
    static constexpr JPH::uint            NUM_LAYERS  = 2;
}

class BPLayerInterfaceImpl final : public JPH::BroadPhaseLayerInterface {
public:
    BPLayerInterfaceImpl() {
        m_objectToBP[PhysicsLayers::STATIC]  = BPLayers::NON_MOVING;
        m_objectToBP[PhysicsLayers::DYNAMIC] = BPLayers::MOVING;
    }
    JPH::uint            GetNumBroadPhaseLayers()                        const override { return BPLayers::NUM_LAYERS; }
    JPH::BroadPhaseLayer GetBroadPhaseLayer(JPH::ObjectLayer layer)      const override { return m_objectToBP[layer]; }
#if defined(JPH_EXTERNAL_PROFILE) || defined(JPH_PROFILE_ENABLED)
    const char* GetBroadPhaseLayerName(JPH::BroadPhaseLayer layer) const override {
        switch ((JPH::BroadPhaseLayer::Type)layer) {
            case (JPH::BroadPhaseLayer::Type)BPLayers::NON_MOVING: return "NON_MOVING";
            case (JPH::BroadPhaseLayer::Type)BPLayers::MOVING:     return "MOVING";
            default:                                                return "INVALID";
        }
    }
#endif
private:
    JPH::BroadPhaseLayer m_objectToBP[PhysicsLayers::NUM_LAYERS] {};
};

class ObjectVsBPLayerFilterImpl final : public JPH::ObjectVsBroadPhaseLayerFilter {
public:
    bool ShouldCollide(JPH::ObjectLayer layer, JPH::BroadPhaseLayer bpLayer) const override {
        switch (layer) {
            case PhysicsLayers::STATIC:  return bpLayer == BPLayers::MOVING;
            case PhysicsLayers::DYNAMIC: return true;
            default:                     return false;
        }
    }
};

class ObjectLayerPairFilterImpl final : public JPH::ObjectLayerPairFilter {
public:
    bool ShouldCollide(JPH::ObjectLayer a, JPH::ObjectLayer b) const override {
        switch (a) {
            case PhysicsLayers::STATIC:  return b == PhysicsLayers::DYNAMIC;
            case PhysicsLayers::DYNAMIC: return true;
            default:                     return false;
        }
    }
};

// Group filter: two bodies in the same non-zero collision group never collide.
// One shared instance is attached to every grouped body's CollisionGroup, so the
// overlapping bones of a ragdoll (same group) ignore each other while different
// ragdolls (different group numbers) and ungrouped geometry still collide.
class GroupExcludeFilter final : public JPH::GroupFilter {
public:
    bool CanCollide(const JPH::CollisionGroup& a, const JPH::CollisionGroup& b) const override {
        JPH::CollisionGroup::GroupID ga = a.GetGroupID();
        return !(ga != 0 && ga == b.GetGroupID());
    }
};

// ---------------------------------------------------------------------------
// GLM <-> Jolt conversion helpers
// ---------------------------------------------------------------------------
static inline JPH::Vec3 ToJolt(const glm::vec3& v) { return { v.x, v.y, v.z }; }
static inline JPH::Quat ToJolt(const glm::quat& q) { return JPH::Quat(q.x, q.y, q.z, q.w); }
static inline glm::vec3 FromJolt(JPH::Vec3Arg v)   { return { v.GetX(), v.GetY(), v.GetZ() }; }
static inline glm::quat FromJolt(JPH::QuatArg q)   { return { q.GetW(), q.GetX(), q.GetY(), q.GetZ() }; }

static float QuatAngleDeg(glm::quat q) {
    q = glm::normalize(q);
    return glm::degrees(2.0f * std::acos(glm::clamp(std::abs(q.w), 0.0f, 1.0f)));
}

static float SignedTwistDeg(glm::quat q, glm::vec3 axis) {
    q = glm::normalize(q);
    if (q.w < 0.0f) q = -q;
    axis = glm::normalize(axis);
    return glm::degrees(2.0f * std::atan2(
        glm::dot(glm::vec3(q.x, q.y, q.z), axis), q.w));
}

// ---------------------------------------------------------------------------
// Contact events + listener
//
// Jolt fires callbacks during Update() (on the physics thread in MT mode).
// We queue events and drain them after the step to safely call std::function.
// ---------------------------------------------------------------------------
struct ContactEvent {
    enum class Type { Enter, Stay, Exit, TriggerEnter, TriggerStay, TriggerExit };
    Type         type;
    entt::entity entityA     = entt::null;
    entt::entity entityB     = entt::null;
    glm::vec3    contactPoint  { 0.0f };
    glm::vec3    contactNormal { 0.0f };       // points from body A (1) toward body B (2)
    float        penetrationDepth = 0.0f;      // positive overlap; negative speculative contact
    // Jolt BodyIDs (index+sequence) of the two bodies — needed to map a contact onto a
    // specific ragdoll bone (a ragdoll is one entity with many bodies). Set on Enter only.
    uint32_t     bodyA = 0xFFFFFFFFu;
    uint32_t     bodyB = 0xFFFFFFFFu;
    // Momentum-based impact estimate (kg·m/s); 0 unless a real moving hit. Drives ragdoll auto-limp.
    float        impactMagnitude = 0.0f;
};

// Momentum-based impact estimate for a contact: reduced mass × closing speed along the
// contact normal. Only a dynamic body contributes mass — a kinematic ragdoll body has
// infinite mass, so the reduced mass collapses to the hitter's mass, and two non-dynamic
// bodies (e.g. an animated ragdoll resting on the static floor) read ~0. This is what the
// ragdoll auto-limp threshold compares against (see AutoTriggerRagdollImpacts).
static float ComputeImpactMagnitude(const JPH::Body& a, const JPH::Body& b,
                                    const JPH::ContactManifold& manifold) {
    auto invMassOf = [](const JPH::Body& body) -> float {
        if (body.GetMotionType() != JPH::EMotionType::Dynamic) return 0.0f;
        const JPH::MotionProperties* mp = body.GetMotionPropertiesUnchecked();
        return mp ? mp->GetInverseMass() : 0.0f;
    };
    float invSum = invMassOf(a) + invMassOf(b);
    if (invSum <= 0.0f) return 0.0f;                       // neither side dynamic → no momentum
    float reducedMass = 1.0f / invSum;
    // Velocity AT the contact point (linear + angular), not the COM: a swinging limb
    // barely translates its COM while its tip moves fast, so COM velocity would miss
    // rotational whacks. Position read access is valid in this callback (Jolt samples
    // read body bounds here).
    JPH::RVec3 cp = manifold.GetWorldSpaceContactPointOn1(0);
    JPH::Vec3 vRel = b.GetPointVelocity(cp) - a.GetPointVelocity(cp);
    float closing = vRel.Dot(manifold.mWorldSpaceNormal); // normal points A→B
    return reducedMass * std::abs(closing);
}

// Lookup table populated by PhysicsSystem when creating/destroying bodies.
// ContactListener holds a const-ref so it can resolve BodyID → entity.
struct BodyRecord {
    entt::entity entity   = entt::null;
    bool         isSensor = false; // or trigger
};

class ContactListener final : public JPH::ContactListener {
public:
    explicit ContactListener(const std::unordered_map<uint32_t, BodyRecord>& bodyMap)
        : m_bodyMap(bodyMap) {}

    JPH::ValidateResult OnContactValidate(const JPH::Body&, const JPH::Body&,
        JPH::RVec3Arg, const JPH::CollideShapeResult&) override {
        return JPH::ValidateResult::AcceptAllContactsForThisBodyPair;
    }

    void OnContactAdded(const JPH::Body& a, const JPH::Body& b,
        const JPH::ContactManifold& manifold, JPH::ContactSettings&) override {
        bool isTrigger = a.IsSensor() || b.IsSensor();
        ContactEvent ev;
        ev.type    = isTrigger ? ContactEvent::Type::TriggerEnter : ContactEvent::Type::Enter;
        ev.entityA = static_cast<entt::entity>(static_cast<uint32_t>(a.GetUserData()));
        ev.entityB = static_cast<entt::entity>(static_cast<uint32_t>(b.GetUserData()));
        ev.bodyA   = a.GetID().GetIndexAndSequenceNumber();
        ev.bodyB   = b.GetID().GetIndexAndSequenceNumber();
        if (!isTrigger) {
            JPH::RVec3 wp = manifold.GetWorldSpaceContactPointOn1(0);
            JPH::Vec3  wn = manifold.mWorldSpaceNormal;
            ev.contactPoint    = { (float)wp.GetX(), (float)wp.GetY(), (float)wp.GetZ() };
            ev.contactNormal   = { wn.GetX(), wn.GetY(), wn.GetZ() };
            ev.penetrationDepth = manifold.mPenetrationDepth;
            ev.impactMagnitude = ComputeImpactMagnitude(a, b, manifold);
        }
        std::lock_guard lock(m_mutex);
        m_events.push_back(ev);
    }

    void OnContactPersisted(const JPH::Body& a, const JPH::Body& b,
        const JPH::ContactManifold& manifold, JPH::ContactSettings&) override {
        bool isTrigger = a.IsSensor() || b.IsSensor();
        ContactEvent ev;
        ev.type = isTrigger ? ContactEvent::Type::TriggerStay : ContactEvent::Type::Stay;
        ev.entityA = static_cast<entt::entity>(static_cast<uint32_t>(a.GetUserData()));
        ev.entityB = static_cast<entt::entity>(static_cast<uint32_t>(b.GetUserData()));
        ev.bodyA   = a.GetID().GetIndexAndSequenceNumber();
        ev.bodyB   = b.GetID().GetIndexAndSequenceNumber();
        JPH::RVec3 wp = manifold.GetWorldSpaceContactPointOn1(0);
        JPH::Vec3  wn = manifold.mWorldSpaceNormal;
        ev.contactPoint  = { (float)wp.GetX(), (float)wp.GetY(), (float)wp.GetZ() };
        ev.contactNormal = { wn.GetX(), wn.GetY(), wn.GetZ() };
        ev.penetrationDepth = manifold.mPenetrationDepth;
        std::lock_guard lock(m_mutex);
        m_events.push_back(ev);
    }

    void OnContactRemoved(const JPH::SubShapeIDPair& pair) override {
        // OnContactRemoved only gives us BodyIDs, not full Body objects,
        // so we resolve entity and sensor flag from the lookup table.
        auto itA = m_bodyMap.find(pair.GetBody1ID().GetIndexAndSequenceNumber());
        auto itB = m_bodyMap.find(pair.GetBody2ID().GetIndexAndSequenceNumber());
        if (itA == m_bodyMap.end() || itB == m_bodyMap.end()) return;

        bool isTrigger = itA->second.isSensor || itB->second.isSensor;
        ContactEvent ev;
        ev.type    = isTrigger ? ContactEvent::Type::TriggerExit : ContactEvent::Type::Exit;
        ev.entityA = itA->second.entity;
        ev.entityB = itB->second.entity;
        ev.bodyA   = pair.GetBody1ID().GetIndexAndSequenceNumber();
        ev.bodyB   = pair.GetBody2ID().GetIndexAndSequenceNumber();
        std::lock_guard lock(m_mutex);
        m_events.push_back(ev);
    }

    std::vector<ContactEvent> DrainEvents() {
        std::lock_guard lock(m_mutex);
        return std::exchange(m_events, {});
    }

private:
    const std::unordered_map<uint32_t, BodyRecord>& m_bodyMap;
    std::mutex                                       m_mutex;
    std::vector<ContactEvent>                        m_events;
};

// ---------------------------------------------------------------------------
// Ragdoll runtime instance — the internal counterpart of RagdollComponent.
// Bodies + joints are plain physics primitives (kinematic capsules that follow
// the animation, joints that hold them together), so they live in the same
// bodyMap / constraintMap as everything else and are torn down by the normal
// OnDestroy sweep. This struct only adds the per-body data the ragdoll passes
// need: which bone each body drives, and its shape (for debug draw). See
// Docs/ragdoll-design.md ("Internal instance").
// ---------------------------------------------------------------------------
struct RagdollBody {
    JPH::BodyID           id;
    int                   boneIndex = -1;   // skeleton bone this body drives
    // Shape in WORLD units (model dims * entity scale), for DrawRagdolls.
    RagdollBodyDef::Shape shape       = RagdollBodyDef::Shape::Capsule;
    float                 radius      = 0.0f;
    float                 halfHeight  = 0.0f;
    glm::vec3             halfExtents { 0.0f };
    glm::vec3             localOffset { 0.0f };           // shape offset from the bone frame
    glm::quat             localRotation { 1, 0, 0, 0 };
    bool                  isFoot = false;                  // planted sole; balance torque acts here
    float                 mass = 1.0f;                     // kg (for the get-up root lift force)
    // World transform captured when the get-up->animation cross-blend begins; the body is
    // driven kinematically from here toward its live animation transform over the blend.
    glm::vec3             blendStartPos { 0.0f };
    glm::quat             blendStartRot { 1, 0, 0, 0 };
};

// One ragdoll joint, with enough data to re-bake it at the live pose on the flip
// to Limp (see RebuildRagdollJointsAtPose). The world anchor/axis aren't stored —
// they're recomputed from the child body's current transform at rebuild time.
struct RagdollJoint {
    uint32_t            constraintId = 0xFFFFFFFFu; // current id in constraintMap
    int                 parentSlot   = -1;          // index into bodies (parent)
    int                 childSlot    = -1;          // index into bodies (self)
    ConstraintComponent cc;                          // type + limits template
    glm::vec3           twistAxisLocal { 0, 1, 0 };  // bone-local twist/hinge axis
    // Child-relative-to-parent body orientation captured the last time this joint was
    // (re)built — i.e. the constraint's actual zero. Equals the bind-relative pose for
    // the initial build, but the sprawled pose after a Limp rebuild. The get-up motor
    // path references hinge targets to this so they drive the right way regardless of
    // where the joint was last anchored. See SyncRagdollPowered / RebuildRagdollJointsAtPose.
    glm::quat           buildRel { 1, 0, 0, 0 };
    // Child-relative-to-parent orientation captured at get-up START (the sprawled pose).
    // The get-up motor target sweeps startRel -> bind over the heave so the limbs travel
    // smoothly instead of the stiff motors springing straight to the final pose.
    glm::quat           startRel { 1, 0, 0, 0 };
};

// A timed procedural "move" layered on the Powered ragdoll: it drives the joint motors
// toward a target pose while ramping muscle strength over a window, then ends. Both the
// hit-react flinch and the get-up are instances of this — the seed of a fuller
// procedural-move system (punch/kick/grab) later. Strength is read each substep by
// SyncRagdollPowered; the target pose + wobble shape what the motors chase.
struct RagReaction {
    // GetUpBlend is the tail of a get-up: after the stand is held, the bodies cross-blend
    // (kinematically) from the held stand pose into the live animation pose, then hand off
    // to Animated — so resuming the clip eases in instead of popping.
    enum class Kind { None, Flinch, GetUp, GetUpBlend };
    Kind  kind          = Kind::None;
    float timer         = 0.0f;    // seconds elapsed
    float duration      = 0.4f;    // total ramp window
    float startStrength = 0.3f;    // muscle strength at t=0
    float endStrength   = 1.0f;    // muscle strength at t=duration
    bool  targetStand   = false;   // true = drive motors to the bind/stand pose; false = the animation
    float wobble        = 0.0f;    // per-joint torque noise amplitude (drunk flail); 0 = clean
};

struct RagdollInstance {
    entt::entity             entity = entt::null;
    std::vector<RagdollBody> bodies;
    std::vector<RagdollJoint> joints;         // joints into constraintMap (+ rebuild data)
    int                      rootBodySlot = -1; // index into bodies of the hips (re-root target)
    uint32_t                 group = 0;        // per-ragdoll self-collision group
    RagdollMode              mode  = RagdollMode::Animated;

    // Bind-pose reference for the get-up root assist (DriveRagdollGetUpRoot): the hips
    // body's world orientation at build (= "upright"), and the hips' height above the
    // lowest body at bind (≈ standing hip clearance over the feet — the lift target).
    glm::quat                rootBindRot { 1, 0, 0, 0 };
    float                    standHipClearance = 0.0f;
    // Total mass of every body in the rig. The get-up lift acts on the pelvis alone,
    // but the pelvis is the suspension point for the whole chain hanging off it via
    // joints — holding it static needs ≈ totalMass·g, not just the pelvis's own weight.
    float                    totalMass = 1.0f;
    // Absolute world height the hips servo toward during a get-up, captured ONCE at
    // get-up start (ground-under-the-feet + standHipClearance). Fixed, not chased off the
    // rig's own lowest body — otherwise dangling feet rise with the hips and it levitates.
    float                    getupTargetY = 0.0f;
    // Root transform at get-up start. During the heave the hips are driven KINEMATICALLY
    // (MoveKinematic) from here to the standing transform — a velocity/force on one dynamic
    // body can't drag the 18-body chain up through the joints; a kinematic root always wins.
    glm::vec3                getupStartPos { 0, 0, 0 };
    glm::quat                getupStartRot { 1, 0, 0, 0 };

    // Active procedural move (flinch / get-up); Kind::None when idle. Advanced by
    // TickRagdollReactions, started by StartRagdollFlinch / StartRagdollGetUp.
    RagReaction reaction;
    // Auto get-up: how long the limp rig has been ~at rest, vs cfg->getupDelay.
    float       restTimer = 0.0f;
    // How long the hips have continuously exceeded locomotionTipLimpDeg (tip-limp grace).
    float       tipTimer  = 0.0f;

    // True while the locomotion root drive is engaged (see DriveRagdollLocomotionRoot;
    // the root is held Kinematic in legacy mode, stays Dynamic in the physical mode).
    // Tracked here so the drive only flips the root's motion type on an actual on/off
    // transition, and so a reaction taking over the root (ReleaseLocomotionRoot)
    // leaves consistent bookkeeping.
    bool        _locomotionDriving = false;
    // Always-present, normally disabled world-to-root SixDOF spring. Translation stays
    // free; Test 7 enables pitch/roll upright motors plus a gentler heading motor.
    uint32_t    locomotionUprightConstraintId = 0xFFFFFFFFu;
};

// ---------------------------------------------------------------------------
// Impl — owns all Jolt objects (kept out of the public header via PIMPL)
// ---------------------------------------------------------------------------
struct PhysicsSystem::Impl {
    BPLayerInterfaceImpl      bpLayerInterface;
    ObjectVsBPLayerFilterImpl objVsBPFilter;
    ObjectLayerPairFilterImpl objLayerFilter;

    std::unordered_map<uint32_t, BodyRecord> bodyMap; // BodyID → entity info
    std::vector<entt::entity>                pendingCreate; // deferred from signal callbacks

    // Constraint tables — the joint analog of bodyMap. constraintMap owns the
    // live Jolt joints; bodyToConstraints is the reverse index that lets body
    // teardown find and remove the joints touching it (a constraint may live on
    // a different entity than the body being destroyed). pendingConstraints is
    // the deferred-creation queue, mirroring pendingCreate.
    std::unordered_map<uint32_t, JPH::Ref<JPH::Constraint>> constraintMap;
    std::unordered_map<uint32_t, std::vector<uint32_t>>     bodyToConstraints;
    std::vector<entt::entity>                               pendingConstraints;
    uint32_t                                               nextConstraintId = 0;

    // Ragdoll instances, keyed by RagdollComponent::_ragdollId. Their bodies live
    // in bodyMap and their joints in constraintMap, so OnDestroy tears them down
    // with everything else; this map just drives the per-frame follow + readback.
    std::unordered_map<uint32_t, RagdollInstance> ragdolls;
    uint32_t                                      nextRagdollId = 0;
    double                                        simTime = 0.0;   // running physics clock (for get-up wobble noise)
    double                                        locoMotorDebugNext = 0.0;

    std::unique_ptr<JPH::TempAllocatorImpl>       tempAllocator;
    std::unique_ptr<JPH::JobSystemThreadPool>      jobSystem;
    std::unique_ptr<JPH::PhysicsSystem>            joltSystem;
    std::unique_ptr<ContactListener>               contactListener;
    JPH::Ref<JPH::GroupFilter>                     groupFilter;   // shared by all grouped bodies
};

// ---------------------------------------------------------------------------
// Active session pointer — set in OnStart, cleared in OnDestroy.
// Signal callbacks and Physics:: API functions use this to reach Jolt.
// ---------------------------------------------------------------------------
static PhysicsSystem::Impl* s_Impl = nullptr;

static JPH::SixDOFConstraint* GetLocomotionUprightConstraint(
    PhysicsSystem::Impl& impl, const RagdollInstance& inst)
{
    if (inst.locomotionUprightConstraintId == 0xFFFFFFFFu) return nullptr;
    auto it = impl.constraintMap.find(inst.locomotionUprightConstraintId);
    if (it == impl.constraintMap.end()
        || it->second->GetSubType() != JPH::EConstraintSubType::SixDOF)
        return nullptr;
    return static_cast<JPH::SixDOFConstraint*>(it->second.GetPtr());
}

static void DisableLocomotionUprightConstraint(
    PhysicsSystem::Impl& impl, const RagdollInstance& inst)
{
    JPH::SixDOFConstraint* motor = GetLocomotionUprightConstraint(impl, inst);
    if (!motor) return;
    using Axis = JPH::SixDOFConstraint::EAxis;
    motor->SetMotorState(Axis::RotationX, JPH::EMotorState::Off);
    motor->SetMotorState(Axis::RotationY, JPH::EMotorState::Off);
    motor->SetMotorState(Axis::RotationZ, JPH::EMotorState::Off);
}

static std::unordered_map<std::string, JPH::ShapeRefC> s_convexHullCache;
static std::unordered_map<std::string, JPH::ShapeRefC> s_triangleMeshCache;

// Removes every constraint referencing the given body from the Jolt system.
// Called before a body is destroyed — Jolt requires joints be removed before
// the bodies they reference. Because a constraint is indexed under BOTH of its
// bodies, destroying either endpoint (even one that doesn't own the
// ConstraintComponent) finds and tears down the joint, preventing a dangling
// reference crash on the next solve.
static void RemoveConstraintsTouching(uint32_t bodyId) {
    if (!s_Impl) return;
    auto it = s_Impl->bodyToConstraints.find(bodyId);
    if (it == s_Impl->bodyToConstraints.end()) return;
    for (uint32_t cid : it->second) {
        auto cit = s_Impl->constraintMap.find(cid);
        if (cit == s_Impl->constraintMap.end()) continue; // already removed via other endpoint
        s_Impl->joltSystem->RemoveConstraint(cit->second);
        s_Impl->constraintMap.erase(cit);
    }
    s_Impl->bodyToConstraints.erase(it);
}

// ---------------------------------------------------------------------------
// Runtime lifecycle signal callbacks
// ---------------------------------------------------------------------------

// Called when RigidBodyComponent OR ColliderComponent is added during play.
// We defer body creation to the next OnUpdate so both components are present.
static void OnBodyComponentAdded(entt::registry&, entt::entity entity) {
    if (!s_Impl) return;
    auto& pending = s_Impl->pendingCreate;
    if (std::find(pending.begin(), pending.end(), entity) == pending.end())
        pending.push_back(entity);
}

// Called just before RigidBodyComponent is removed — body ID lives in rb._bodyId.
static void OnRbDestroyed(entt::registry& reg, entt::entity entity) {
    if (!s_Impl) return;
    auto& rb = reg.get<RigidBodyComponent>(entity);
    if (rb._bodyId == 0xFFFFFFFFu) return;
    RemoveConstraintsTouching(rb._bodyId); // joints must go before the body they reference
    JPH::BodyInterface& bi = s_Impl->joltSystem->GetBodyInterface();
    JPH::BodyID bodyID(rb._bodyId);
    bi.RemoveBody(bodyID);
    bi.DestroyBody(bodyID);
    s_Impl->bodyMap.erase(rb._bodyId);
}

// Called just before ColliderComponent is removed — handles collider-only entities
// whose body ID is stored in col._bodyId (full-physics entities use rb._bodyId).
static void OnColDestroyed(entt::registry& reg, entt::entity entity) {
    if (!s_Impl) return;
    auto& col = reg.get<ColliderComponent>(entity);
    if (col._bodyId == 0xFFFFFFFFu) return; // not a collider-only body, already handled by OnRbDestroyed
    RemoveConstraintsTouching(col._bodyId); // joints must go before the body they reference
    JPH::BodyInterface& bi = s_Impl->joltSystem->GetBodyInterface();
    JPH::BodyID bodyID(col._bodyId);
    bi.RemoveBody(bodyID);
    bi.DestroyBody(bodyID);
    s_Impl->bodyMap.erase(col._bodyId);
}

// ---------------------------------------------------------------------------
// Constraint lifecycle
// ---------------------------------------------------------------------------

// Resolve an entity to the Jolt body ID it owns (full-physics or collider-only),
// or the invalid sentinel if it has no body yet.
static uint32_t BodyIdOf(entt::registry& reg, entt::entity e) {
    if (!reg.valid(e)) return 0xFFFFFFFFu;
    if (reg.all_of<RigidBodyComponent>(e)) return reg.get<RigidBodyComponent>(e)._bodyId;
    if (reg.all_of<ColliderComponent>(e))  return reg.get<ColliderComponent>(e)._bodyId;
    return 0xFFFFFFFFu;
}

// Any unit vector perpendicular to a — used as the hinge's reference (normal)
// axis, which Jolt requires to be perpendicular to the rotation axis.
static glm::vec3 AnyPerpendicular(const glm::vec3& a) {
    glm::vec3 n = glm::normalize(a);
    glm::vec3 ref = (std::abs(n.y) < 0.99f) ? glm::vec3(0, 1, 0) : glm::vec3(1, 0, 0);
    return glm::normalize(glm::cross(n, ref));
}

// Build a normalized rotation quaternion from a (possibly scaled / sheared /
// degenerate) world matrix's basis. Columns are normalized individually; a
// zero-length or non-finite column — e.g. an uninitialized animation-palette
// entry for a bone the animator never wrote — would otherwise poison the
// quaternion with NaN and trip Jolt's IsNormalized() assert deep in the
// constraint solver (Quat::RotateAxisX). Falls back to identity so one bad bone
// can't crash the whole step.
static glm::quat SafeOrientation(const glm::mat4& m) {
    auto col = [](const glm::vec3& c, const glm::vec3& fallback) {
        float len = glm::length(c);
        return (std::isfinite(len) && len > 1e-6f) ? c / len : fallback;
    };
    glm::vec3 x = col(glm::vec3(m[0]), glm::vec3(1, 0, 0));
    glm::vec3 y = col(glm::vec3(m[1]), glm::vec3(0, 1, 0));
    glm::vec3 z = col(glm::vec3(m[2]), glm::vec3(0, 0, 1));
    glm::quat q  = glm::quat_cast(glm::mat3(x, y, z));
    float     ql = glm::length(q);
    if (!std::isfinite(ql) || ql < 1e-6f) return glm::quat(1, 0, 0, 0);
    return q / ql;
}

// Normalize a world-space axis, falling back to the (normalized) bone-local axis
// and finally +Y if the world axis is degenerate. Never returns a NaN/zero axis.
static glm::vec3 SafeAxis(const glm::vec3& world, const glm::vec3& localFallback) {
    float len = glm::length(world);
    if (std::isfinite(len) && len > 1e-6f) return world / len;
    float fl = glm::length(localFallback);
    if (std::isfinite(fl) && fl > 1e-6f) return localFallback / fl;
    return glm::vec3(0, 1, 0);
}

// Called when a ConstraintComponent is added during play — defer creation to the
// next OnUpdate, by which point the referenced bodies are guaranteed to exist.
static void OnConstraintAdded(entt::registry&, entt::entity entity) {
    if (!s_Impl) return;
    auto& pending = s_Impl->pendingConstraints;
    if (std::find(pending.begin(), pending.end(), entity) == pending.end())
        pending.push_back(entity);
}

// Called just before a ConstraintComponent is removed (without its entity being
// destroyed). Tears the single joint out of Jolt. Body-driven teardown goes
// through RemoveConstraintsTouching instead.
static void OnConstraintDestroyed(entt::registry& reg, entt::entity entity) {
    if (!s_Impl) return;
    auto& cc = reg.get<ConstraintComponent>(entity);
    if (cc._constraintId == 0xFFFFFFFFu) return;
    auto it = s_Impl->constraintMap.find(cc._constraintId);
    if (it != s_Impl->constraintMap.end()) {
        s_Impl->joltSystem->RemoveConstraint(it->second);
        s_Impl->constraintMap.erase(it);
    }
}

// Builds a Jolt joint of the component's type from its world-space anchor/axis.
// body1 is the anchor/parent body, body2 is the entity's own body.
static JPH::Ref<JPH::TwoBodyConstraint> BuildConstraint(const ConstraintComponent& cc,
                                                        JPH::Body& body1, JPH::Body& body2)
{
    switch (cc.type) {
        case ConstraintType::Hinge:
        default: {
            glm::vec3 hingeAxis  = glm::normalize(cc.axis);
            glm::vec3 normalAxis = AnyPerpendicular(hingeAxis);

            JPH::HingeConstraintSettings s;
            s.mSpace       = JPH::EConstraintSpace::WorldSpace;
            s.mPoint1      = s.mPoint2      = JPH::RVec3(cc.anchor.x, cc.anchor.y, cc.anchor.z);
            s.mHingeAxis1  = s.mHingeAxis2  = ToJolt(hingeAxis);
            s.mNormalAxis1 = s.mNormalAxis2 = ToJolt(normalAxis);
            if (cc.hasLimits) {
                // Jolt wants radians, with min in [-pi, 0] and max in [0, pi].
                s.mLimitsMin = glm::radians(glm::clamp(cc.limitMin, -180.0f, 0.0f));
                s.mLimitsMax = glm::radians(glm::clamp(cc.limitMax,    0.0f, 180.0f));
            }
            // Motor capability goes in the settings; mode + target are applied to
            // the live constraint below (they aren't part of the settings).
            if (cc.motorMode != MotorMode::Off) {
                s.mMotorSettings = JPH::MotorSettings(cc.motorFrequency, cc.motorDamping);
                s.mMotorSettings.SetTorqueLimit(glm::max(cc.motorMaxTorque, 0.0f));
            }
            JPH::Ref<JPH::TwoBodyConstraint> c = s.Create(body1, body2);
            if (cc.motorMode != MotorMode::Off) {
                auto* hinge = static_cast<JPH::HingeConstraint*>(c.GetPtr());
                if (cc.motorMode == MotorMode::Velocity) {
                    hinge->SetMotorState(JPH::EMotorState::Velocity);
                    hinge->SetTargetAngularVelocity(glm::radians(cc.motorTarget)); // deg/s -> rad/s
                } else {
                    hinge->SetMotorState(JPH::EMotorState::Position);
                    hinge->SetTargetAngle(glm::radians(cc.motorTarget));           // deg -> rad
                }
            }
            return c;
        }
        case ConstraintType::SwingTwist: {
            // `axis` is the twist axis (along the bone); plane axis is any
            // perpendicular. Swing is an elliptical cone (two half-angles);
            // twist is rotation about the twist axis, limited to [min, max].
            glm::vec3 twistAxis = glm::normalize(cc.axis);
            glm::vec3 planeAxis = AnyPerpendicular(twistAxis);

            JPH::SwingTwistConstraintSettings s;
            s.mSpace       = JPH::EConstraintSpace::WorldSpace;
            s.mPosition1   = s.mPosition2   = JPH::RVec3(cc.anchor.x, cc.anchor.y, cc.anchor.z);
            s.mTwistAxis1  = s.mTwistAxis2  = ToJolt(twistAxis);
            s.mPlaneAxis1  = s.mPlaneAxis2  = ToJolt(planeAxis);
            s.mNormalHalfConeAngle = glm::radians(glm::clamp(cc.swingNormalDeg, 0.0f, 180.0f));
            s.mPlaneHalfConeAngle  = glm::radians(glm::clamp(cc.swingPlaneDeg,  0.0f, 180.0f));
            s.mTwistMinAngle       = glm::radians(glm::clamp(cc.twistMinDeg, -180.0f, 0.0f));
            s.mTwistMaxAngle       = glm::radians(glm::clamp(cc.twistMaxDeg,    0.0f, 180.0f));

            if (cc.motorMode != MotorMode::Off) {
                JPH::MotorSettings m(cc.motorFrequency, cc.motorDamping);
                m.SetTorqueLimit(glm::max(cc.motorMaxTorque, 0.0f));
                s.mSwingMotorSettings = m;
                s.mTwistMotorSettings = m;
            }
            // Motor capability goes in the settings; mode + target are applied to
            // the live constraint below (they aren't part of the settings).
            JPH::Ref<JPH::TwoBodyConstraint> c = s.Create(body1, body2);
            if (cc.motorMode != MotorMode::Off) {
                auto* st = static_cast<JPH::SwingTwistConstraint*>(c.GetPtr());
                JPH::EMotorState state = (cc.motorMode == MotorMode::Velocity)
                    ? JPH::EMotorState::Velocity : JPH::EMotorState::Position;
                st->SetSwingMotorState(state);
                st->SetTwistMotorState(state);
                if (cc.motorMode == MotorMode::Velocity) {
                    st->SetTargetAngularVelocityCS(ToJolt(glm::radians(cc.motorTargetEuler))); // deg/s -> rad/s
                } else {
                    // Position target relative to the rest pose. SetTargetOrientationBS
                    // drives R2 = R1 * q, so identity would mean "align body2 to body1",
                    // not rest. Capture the rest relative rotation and apply the euler
                    // target on top (in the bone's local frame), so Target (0,0,0) holds
                    // the authored pose and the euler nudges it from there. For a world
                    // anchor, body1 is sFixedToWorld (identity), so restBS = body2's pose.
                    JPH::Quat restBS = body1.GetRotation().Conjugated() * body2.GetRotation();
                    JPH::Quat offset = ToJolt(glm::quat(glm::radians(cc.motorTargetEuler)));
                    st->SetTargetOrientationBS(restBS * offset);
                }
            }
            return c;
        }
        case ConstraintType::Fixed: {
            JPH::FixedConstraintSettings s;
            s.mSpace       = JPH::EConstraintSpace::WorldSpace;
            s.mPoint1      = s.mPoint2 =    JPH::RVec3(cc.anchor.x, cc.anchor.y, cc.anchor.z);
            return s.Create(body1, body2);
        }
        case ConstraintType::Point: {
            // Removes the 3 translational DOF at the anchor; rotation stays free.
            // Only the anchor is used — no axis, limits, or motor.
            JPH::PointConstraintSettings s;
            s.mSpace  = JPH::EConstraintSpace::WorldSpace;
            s.mPoint1 = s.mPoint2 = JPH::RVec3(cc.anchor.x, cc.anchor.y, cc.anchor.z);
            return s.Create(body1, body2);
        }
        case ConstraintType::Grab: {
            // A motorized SixDOF used as a soft, force-limited grab. Translation axes
            // are left free but driven by position motors toward the anchor (target 0 in
            // body1's frame); rotation is free so the held body dangles. The force limit
            // (motorMaxForce) caps the pull, which is what makes weight matter — the body
            // sags if its weight exceeds the budget. body1 is the (kinematic) hand anchor;
            // moving body1 drags body2 along up to the force cap. Solver-driven = stable.
            using EAxis = JPH::SixDOFConstraintSettings::EAxis;
            JPH::SixDOFConstraintSettings s;
            s.mSpace     = JPH::EConstraintSpace::WorldSpace;
            s.mPosition1 = s.mPosition2 = JPH::RVec3(cc.anchor.x, cc.anchor.y, cc.anchor.z);

            JPH::MotorSettings m(cc.motorFrequency, cc.motorDamping);
            m.SetForceLimit(glm::max(cc.motorMaxForce, 0.0f));   // ±budget on the linear motor
            s.mMotorSettings[EAxis::TranslationX] = m;
            s.mMotorSettings[EAxis::TranslationY] = m;
            s.mMotorSettings[EAxis::TranslationZ] = m;

            JPH::Ref<JPH::TwoBodyConstraint> c = s.Create(body1, body2);
            auto* six = static_cast<JPH::SixDOFConstraint*>(c.GetPtr());
            six->SetMotorState(EAxis::TranslationX, JPH::EMotorState::Position);
            six->SetMotorState(EAxis::TranslationY, JPH::EMotorState::Position);
            six->SetMotorState(EAxis::TranslationZ, JPH::EMotorState::Position);
            six->SetTargetPositionCS(JPH::Vec3::sZero());        // hold body2 at body1's anchor
            return c;
        }
    }
}

// Adds the joint to Jolt and registers it under both bodies so destroying either
// endpoint tears it down. bodyB == sentinel means the world (no reverse index).
static void RegisterConstraint(PhysicsSystem::Impl& impl, ConstraintComponent& cc,
                               JPH::Ref<JPH::TwoBodyConstraint> constraint,
                               uint32_t bodyA, uint32_t bodyB)
{
    if (!constraint) return;
    impl.joltSystem->AddConstraint(constraint);
    uint32_t cid = impl.nextConstraintId++;
    cc._constraintId        = cid;
    impl.constraintMap[cid] = constraint;
    impl.bodyToConstraints[bodyA].push_back(cid);
    if (bodyB != 0xFFFFFFFFu)
        impl.bodyToConstraints[bodyB].push_back(cid);
}

// Builds the live joint. bodyA is the entity's own body; bodyB is the target
// body, or the invalid sentinel to attach to the immovable world. The Jolt
// joint must be created while the bodies are locked, so all work happens inside
// the lock scope. body1 = target/world (parent), body2 = self.
static void CreateConstraint(PhysicsSystem::Impl& impl, entt::entity /*entity*/,
                             ConstraintComponent& cc, uint32_t bodyA, uint32_t bodyB)
{
    if (bodyB == 0xFFFFFFFFu) {
        JPH::BodyLockWrite lock(impl.joltSystem->GetBodyLockInterface(), JPH::BodyID(bodyA));
        if (!lock.Succeeded()) return;
        auto c = BuildConstraint(cc, JPH::Body::sFixedToWorld, lock.GetBody());
        RegisterConstraint(impl, cc, c, bodyA, bodyB);
    } else {
        // Lock both bodies together (sorted internally) to avoid lock-order issues.
        JPH::BodyID ids[2] = { JPH::BodyID(bodyB), JPH::BodyID(bodyA) };
        JPH::BodyLockMultiWrite lock(impl.joltSystem->GetBodyLockInterface(), ids, 2);
        JPH::Body* target = lock.GetBody(0); // bodyB
        JPH::Body* self   = lock.GetBody(1); // bodyA
        if (!target || !self) return;
        auto c = BuildConstraint(cc, *target, *self);
        RegisterConstraint(impl, cc, c, bodyA, bodyB);
    }
}

// ---------------------------------------------------------------------------
// CreateShape
// Returns a JPH::ShapeRefC (Jolt's ref-counted shape handle), or nullptr on failure.
// ---------------------------------------------------------------------------
static JPH::ShapeRefC CreateShape(const ColliderComponent& col, const glm::vec3& meshScale = glm::vec3(1.0f))
{
    JPH::ShapeRefC shape;

    switch (col.shapeType)
    {
    case CollisionShape::Box:
    {
        JPH::BoxShapeSettings settings(ToJolt(col.halfExtents));
        auto result = settings.Create();
        if (!result.IsValid()) return nullptr;
        shape = result.Get();
        break;
    }
    case CollisionShape::Sphere:
    {
        JPH::SphereShapeSettings settings(col.radius);
        auto result = settings.Create();
        if (!result.IsValid()) return nullptr;
        shape = result.Get();
        break;
    }
    case CollisionShape::Capsule:
    {
        JPH::CapsuleShapeSettings settings(col.halfHeight, col.radius);
        auto result = settings.Create();
        if (!result.IsValid()) return nullptr;
        shape = result.Get();
        break;
    }
    case CollisionShape::ConvexHull:
    {
        if (col.meshPath.empty()) {
            spdlog::warn("CreateShape: ConvexHull collider has no meshPath");
            return nullptr;
        }
        char scaleSuffix[64];
        snprintf(scaleSuffix, sizeof(scaleSuffix), "|%.6f,%.6f,%.6f", meshScale.x, meshScale.y, meshScale.z);
        std::string cacheKey = col.meshPath + scaleSuffix;

        auto cacheIt = s_convexHullCache.find(cacheKey);
        if (cacheIt != s_convexHullCache.end()) { shape = cacheIt->second; break; }

        auto meshes = Diamond::ModelImporter::Load(col.meshPath);
        if (meshes.empty()) return nullptr;

        JPH::Array<JPH::Vec3> points;
        for (auto& mesh : meshes)
            for (auto& v : mesh.Vertices)
                points.push_back({ v.Position.x * meshScale.x, v.Position.y * meshScale.y, v.Position.z * meshScale.z });

        if (points.empty()) return nullptr;

        JPH::ConvexHullShapeSettings settings(points);
        auto result = settings.Create();
        if (!result.IsValid()) {
            spdlog::warn("CreateShape: ConvexHull creation failed for '{}'", col.meshPath);
            return nullptr;
        }
        shape = result.Get();
        s_convexHullCache[cacheKey] = shape;
        break;
    }
    case CollisionShape::TriangleMesh:
    {
        if (col.meshPath.empty()) {
            spdlog::warn("CreateShape: TriangleMesh collider has no meshPath");
            return nullptr;
        }
        char scaleSuffix[64];
        snprintf(scaleSuffix, sizeof(scaleSuffix), "|%.6f,%.6f,%.6f", meshScale.x, meshScale.y, meshScale.z);
        std::string cacheKey = col.meshPath + scaleSuffix;

        auto cacheIt = s_triangleMeshCache.find(cacheKey);
        if (cacheIt != s_triangleMeshCache.end()) { shape = cacheIt->second; break; }

        auto meshes = Diamond::ModelImporter::Load(col.meshPath);
        if (meshes.empty()) return nullptr;

        JPH::TriangleList triangles;
        for (auto& mesh : meshes) {
            for (size_t i = 0; i + 2 < mesh.Indices.size(); i += 3) {
                const auto& v0 = mesh.Vertices[mesh.Indices[i    ]].Position;
                const auto& v1 = mesh.Vertices[mesh.Indices[i + 1]].Position;
                const auto& v2 = mesh.Vertices[mesh.Indices[i + 2]].Position;
                triangles.push_back(JPH::Triangle(
                    JPH::Float3(v0.x * meshScale.x, v0.y * meshScale.y, v0.z * meshScale.z),
                    JPH::Float3(v1.x * meshScale.x, v1.y * meshScale.y, v1.z * meshScale.z),
                    JPH::Float3(v2.x * meshScale.x, v2.y * meshScale.y, v2.z * meshScale.z)
                ));
            }
        }

        if (triangles.empty()) return nullptr;

        JPH::MeshShapeSettings settings(std::move(triangles));
        auto result = settings.Create();
        if (!result.IsValid()) {
            spdlog::warn("CreateShape: TriangleMesh creation failed for '{}'", col.meshPath);
            return nullptr;
        }
        shape = result.Get();
        s_triangleMeshCache[cacheKey] = shape;
        break;
    }
    default:
        return nullptr;
    }

    bool hasOffset   = col.localOffset   != glm::vec3(0.0f);
    bool hasRotation = col.localRotation != glm::quat(1, 0, 0, 0);
    if (hasOffset || hasRotation)
    {
        JPH::RotatedTranslatedShapeSettings rtSettings(
            ToJolt(col.localOffset), ToJolt(col.localRotation), shape);
        auto result = rtSettings.Create();
        if (!result.IsValid()) return nullptr;
        return result.Get();
    }

    return shape;
}

// ---------------------------------------------------------------------------
// CreateBody
// Builds a Jolt body from the entity's components and registers it in bodyMap.
// bi        = joltSystem->GetBodyInterface()
// bodyMap   = impl->bodyMap (BodyID -> entity lookup for the contact listener)
// ---------------------------------------------------------------------------
// If the collider has no meshPath but the entity has a MeshComponent, inherit its path.
static void InheritMeshPath(ColliderComponent& col, entt::entity entity, entt::registry& reg)
{
    if (!col.meshPath.empty()) return;
    if (col.shapeType != CollisionShape::ConvexHull && col.shapeType != CollisionShape::TriangleMesh) return;
    if (reg.all_of<MeshComponent>(entity))
        col.meshPath = reg.get<MeshComponent>(entity).meshPath;
}

static void CreateBody(JPH::BodyInterface& bi,
    std::unordered_map<uint32_t, BodyRecord>& bodyMap,
    entt::entity entity, RigidBodyComponent& rb,
    const ColliderComponent& col, const TransformComponent& xform,
    const JPH::GroupFilter* groupFilter)
{
    if (col.shapeType == CollisionShape::TriangleMesh && rb.bodyType != BodyType::Static) {
        spdlog::warn("CreateBody: TriangleMesh collider requires a Static body; skipping.");
        return;
    }

    JPH::ShapeRefC shapeRef = CreateShape(col, xform.scale);
    if (!shapeRef) return;

    JPH::EMotionType motionType;
    JPH::ObjectLayer layer;

    switch (rb.bodyType)
    {
    case BodyType::Static:
        motionType = JPH::EMotionType::Static;
        layer = PhysicsLayers::STATIC;
        break;
    case BodyType::Kinematic:
        motionType = JPH::EMotionType::Kinematic;
        layer = PhysicsLayers::DYNAMIC;
        break;
    case BodyType::Dynamic:
        motionType = JPH::EMotionType::Dynamic;
        layer = PhysicsLayers::DYNAMIC;
        break;
    default:
        return;
    }

    JPH::BodyCreationSettings settings(
        shapeRef,
        ToJolt(xform.position),
        ToJolt(xform.rotation),
        motionType,
        layer
    );

    settings.mIsSensor       = col.isTrigger;
    settings.mLinearDamping  = rb.linearDamping;
    settings.mAngularDamping = rb.angularDamping;
    settings.mGravityFactor  = rb.gravityScale;
    settings.mUserData       = (JPH::uint64)entt::to_integral(entity);
    if (rb.continuousCollision)  // CCD: sweep the shape so a fast body doesn't tunnel
        settings.mMotionQuality = JPH::EMotionQuality::LinearCast;

    if (rb.bodyType == BodyType::Dynamic)
    {
        settings.mOverrideMassProperties       = JPH::EOverrideMassProperties::CalculateInertia;
        settings.mMassPropertiesOverride.mMass = rb.mass;
    }

    if (rb.lockRotX || rb.lockRotY || rb.lockRotZ)
    {
        JPH::EAllowedDOFs dof =
            JPH::EAllowedDOFs::TranslationX |
            JPH::EAllowedDOFs::TranslationY |
            JPH::EAllowedDOFs::TranslationZ;
        if (!rb.lockRotX) dof |= JPH::EAllowedDOFs::RotationX;
        if (!rb.lockRotY) dof |= JPH::EAllowedDOFs::RotationY;
        if (!rb.lockRotZ) dof |= JPH::EAllowedDOFs::RotationZ;
        settings.mAllowedDOFs = dof;
    }

    if (col.material)
    {
        settings.mFriction    = col.material->dynamicFriction;
        settings.mRestitution = col.material->restitution;
    }

    if (col.collisionGroup != 0 && groupFilter) {
        settings.mCollisionGroup = JPH::CollisionGroup(
            groupFilter,
            (JPH::CollisionGroup::GroupID)col.collisionGroup,
            (JPH::CollisionGroup::SubGroupID)entt::to_integral(entity));
    }

    // create body id with final body settings
    JPH::BodyID bodyID = bi.CreateAndAddBody(settings, JPH::EActivation::Activate);
    if (bodyID.IsInvalid()) return; // hit cMaxBodies limit

    // write ID back to component and register in lookup table
    rb._bodyId          = bodyID.GetIndexAndSequenceNumber();
    bodyMap[rb._bodyId] = { entity, col.isTrigger };
}

// Forward declaration — definition follows OnStart below.
static void CreateStaticCollider(JPH::BodyInterface&,
    std::unordered_map<uint32_t, BodyRecord>&,
    entt::entity, ColliderComponent&, const TransformComponent&,
    const JPH::GroupFilter*);

// ---------------------------------------------------------------------------
// Ragdoll build (Route B — reuse existing primitives). See Docs/ragdoll-design.md.
// ---------------------------------------------------------------------------

// Shortest-arc rotation taking unit vector `from` onto unit vector `to`.
static glm::quat RotationFromTo(const glm::vec3& from, const glm::vec3& to) {
    glm::vec3 f = glm::normalize(from);
    glm::vec3 t = glm::normalize(to);
    float d = glm::dot(f, t);
    if (d >  0.99999f) return glm::quat(1, 0, 0, 0);
    if (d < -0.99999f) {                           // opposite: rotate 180° about any perpendicular
        glm::vec3 axis = glm::cross(glm::vec3(1, 0, 0), f);
        if (glm::dot(axis, axis) < 1e-6f) axis = glm::cross(glm::vec3(0, 0, 1), f);
        return glm::angleAxis(3.14159265358979f, glm::normalize(axis));
    }
    glm::vec3 axis = glm::normalize(glm::cross(f, t));
    return glm::angleAxis(std::acos(glm::clamp(d, -1.0f, 1.0f)), axis);
}

// Builds a ragdoll body's shape in WORLD units, offset off the bone frame so the
// body's reference frame is the BONE origin (concern #3) and the capsule spans
// along the bone toward its child. `scale` bakes in the entity/import scale
// (concern #4). Mirrors CreateShape's RotatedTranslatedShape wrapping.
static JPH::ShapeRefC CreateRagdollShape(const RagdollBodyDef& def, float scale,
                                         glm::vec3& outOffset, glm::quat& outRot)
{
    outOffset = glm::vec3(0.0f);
    outRot    = glm::quat(1, 0, 0, 0);

    JPH::ShapeRefC shape;
    switch (def.shape) {
        case RagdollBodyDef::Shape::Box: {
            // Jolt's box convex radius (default 0.05m) must not exceed the smallest
            // half-extent, or Create() fails — and ragdoll bones are often thinner
            // than that. Shrink the radius to fit.
            glm::vec3 he = def.halfExtents * scale;
            float minHE   = glm::min(he.x, glm::min(he.y, he.z));
            float convexR = glm::clamp(minHE * 0.5f, 0.001f, 0.05f);
            JPH::BoxShapeSettings s(ToJolt(he), convexR);
            auto r = s.Create(); if (!r.IsValid()) return nullptr; shape = r.Get();
            break;
        }
        case RagdollBodyDef::Shape::Sphere: {
            JPH::SphereShapeSettings s(def.radius * scale);
            auto r = s.Create(); if (!r.IsValid()) return nullptr; shape = r.Get();
            break;
        }
        default: { // Capsule — aligned along the bone, pushed half its length toward the child.
            float radius     = def.radius     * scale;
            float halfHeight = def.halfHeight * scale;
            JPH::CapsuleShapeSettings s(halfHeight, radius);
            auto r = s.Create(); if (!r.IsValid()) return nullptr; shape = r.Get();
            outRot    = RotationFromTo(glm::vec3(0, 1, 0), def.twistAxisLocal);
            outOffset = glm::normalize(def.twistAxisLocal) * (halfHeight + radius);
            break;
        }
    }

    if (def.shape == RagdollBodyDef::Shape::Capsule) {
        JPH::RotatedTranslatedShapeSettings rt(ToJolt(outOffset), ToJolt(outRot), shape);
        auto r = rt.Create(); if (!r.IsValid()) return nullptr;
        return r.Get();
    }
    return shape;
}

// A flat box sole whose complete transform is shared by Jolt and debug drawing. The
// body stays anchored at the skeleton's foot-bone pivot. At bind, outRot cancels the
// bone orientation and outOffset converts an authored bind-world upward displacement
// into the bone frame. A pivot on the ground plane therefore needs approximately the
// box half-thickness plus a small contact margin, not the old zero offset.
static JPH::ShapeRefC CreateFootSole(const glm::vec3& he, const glm::quat& boneRot,
                                     float centerHeight,
                                     glm::vec3& outOffset, glm::quat& outRot)
{
    JPH::BoxShapeSettings bs(ToJolt(he), glm::min(glm::min(he.x, he.y), he.z) * 0.5f);
    auto br = bs.Create(); if (!br.IsValid()) return nullptr;
    outRot = glm::normalize(glm::conjugate(boneRot));
    outOffset = outRot * glm::vec3(0.0f, centerHeight, 0.0f);
    JPH::RotatedTranslatedShapeSettings rt(ToJolt(outOffset), ToJolt(outRot), br.Get());
    auto rr = rt.Create(); if (!rr.IsValid()) return nullptr;
    return rr.Get();
}

// Builds the kinematic body chain + joints for one RagdollComponent and records a
// RagdollInstance. Bodies start at the bind pose; the per-frame follow then drives
// them to the live animation. Called at play start (and never re-entered for an
// already-built ragdoll).
static void BuildRagdoll(PhysicsSystem::Impl& impl, Scene& scene,
                         entt::entity entity, RagdollComponent& rag)
{
    if (rag._ragdollId != 0xFFFFFFFFu) return;            // already built
    if (!rag.config || rag.config->bodies.empty()) return;

    auto& reg = scene.GetRegistry();
    if (!reg.all_of<SkinnedMeshComponent>(entity)) {
        spdlog::warn("BuildRagdoll: entity has no SkinnedMeshComponent; skipping.");
        return;
    }
    const Diamond::Skeleton& skel = reg.get<SkinnedMeshComponent>(entity).skeleton;

    glm::mat4 entityWorld = scene.GetTransformSystem().GetWorldMatrix(entity);
    glm::vec3 c0(entityWorld[0]), c1(entityWorld[1]), c2(entityWorld[2]);
    float scale = (glm::length(c0) + glm::length(c1) + glm::length(c2)) / 3.0f; // assume ~uniform
    if (scale < 1e-6f) scale = 1.0f;

    JPH::BodyInterface& bi = impl.joltSystem->GetBodyInterface();
    const JPH::GroupFilter* gf = impl.groupFilter;

    RagdollInstance inst;
    inst.entity = entity;
    inst.group  = 1000u + impl.nextRagdollId;             // avoid clashing with user collisionGroups

    const auto& defs = rag.config->bodies;
    std::unordered_map<std::string, int> nameToSlot;      // boneName -> slot in inst.bodies
    std::vector<glm::vec3> anchor(defs.size());           // each body's world bone origin
    std::vector<glm::vec3> axisWorld(defs.size());        // each body's world twist/hinge axis

    // Bind-pose root reference for the get-up assist, gathered during pass 1.
    float     minBindY    = 1e30f;
    float     rootBindY   = 0.0f;
    glm::quat rootBindRot { 1, 0, 0, 0 };

    // --- pass 1: bodies -----------------------------------------------------
    for (size_t k = 0; k < defs.size(); ++k) {
        const RagdollBodyDef& def = defs[k];
        int bone = skel.Find(def.boneName);
        if (bone < 0) { spdlog::warn("BuildRagdoll: bone '{}' not in skeleton; skipping body.", def.boneName); continue; }

        glm::mat4 boneModel = glm::inverse(skel.bones[bone].inverseBind); // bind model-space frame
        glm::mat4 bw        = entityWorld * boneModel;
        glm::vec3 pos       = glm::vec3(bw[3]);
        glm::quat rot       = SafeOrientation(bw);
        anchor[k]    = pos;
        const glm::vec3 jointAxisLocal = def.jointType == ConstraintType::Hinge
            ? def.hingeAxisLocal : def.twistAxisLocal;
        axisWorld[k] = SafeAxis(glm::mat3(bw) * jointAxisLocal, jointAxisLocal);

        // Both builders return the exact local wrapper around the primitive.
        glm::vec3 shapeOffset(0.0f); glm::quat shapeRot(1.0f, 0.0f, 0.0f, 0.0f);
        const glm::vec3 footHE = glm::max(def.halfExtents * scale, glm::vec3(0.001f));
        JPH::ShapeRefC shapeRef = def.isFoot
            ? CreateFootSole(footHE, rot,
                             glm::max(def.soleCenterHeight, 0.0f) * scale,
                             shapeOffset, shapeRot)
            : CreateRagdollShape(def, scale, shapeOffset, shapeRot);
        if (!shapeRef) continue;

        JPH::BodyCreationSettings settings(shapeRef, ToJolt(pos), ToJolt(rot),
                                           JPH::EMotionType::Kinematic, PhysicsLayers::DYNAMIC);
        settings.mUserData = (JPH::uint64)entt::to_integral(entity);
        // CCD: ragdoll bones are thin, fast-moving capsules once limp — sweep them so a
        // flung limb doesn't tunnel through the floor/stairs (or get tunnelled by a hit).
        settings.mMotionQuality = JPH::EMotionQuality::LinearCast;
        // Grip: ragdoll bodies default to Jolt's 0.2 friction, so planted feet skate and
        // limbs slide off everything. High friction makes the rig bite the ground — feet
        // hold, a shoved character scuffs to a stop instead of gliding. (TODO Stage B:
        // per-bone friction so only the soles are sticky.)
        settings.mFriction = 0.8f;
        // Mass set now (ignored while kinematic) so it's correct the instant we flip to Dynamic.
        settings.mOverrideMassProperties       = JPH::EOverrideMassProperties::CalculateInertia;
        settings.mMassPropertiesOverride.mMass = glm::max(def.mass, 0.001f);
        if (gf) settings.mCollisionGroup = JPH::CollisionGroup(
            gf, (JPH::CollisionGroup::GroupID)inst.group,
            (JPH::CollisionGroup::SubGroupID)inst.bodies.size());

        JPH::BodyID id = bi.CreateAndAddBody(settings, JPH::EActivation::Activate);
        if (id.IsInvalid()) continue;
        impl.bodyMap[id.GetIndexAndSequenceNumber()] = { entity, false };

        RagdollBody rb;
        rb.id            = id;
        rb.boneIndex     = bone;
        // Debug drawing uses the exact translated/rotated wrapper used by Jolt.
        rb.shape         = def.isFoot ? RagdollBodyDef::Shape::Box : def.shape;
        rb.radius        = def.radius     * scale;
        rb.halfHeight    = def.halfHeight * scale;
        rb.halfExtents   = def.isFoot ? footHE : def.halfExtents * scale;
        rb.localOffset   = shapeOffset;
        rb.localRotation = shapeRot;
        rb.isFoot        = def.isFoot;
        rb.mass          = glm::max(def.mass, 0.001f);
        minBindY = glm::min(minBindY, pos.y);
        nameToSlot[def.boneName] = (int)inst.bodies.size();
        if (def.parentBoneName.empty() && inst.rootBodySlot < 0) {
            inst.rootBodySlot = (int)inst.bodies.size();
            rootBindY   = pos.y;     // hips height at bind
            rootBindRot = rot;       // hips "upright" orientation at bind
        }
        inst.bodies.push_back(rb);
    }

    inst.rootBindRot       = rootBindRot;
    inst.standHipClearance = (minBindY < 1e29f) ? glm::max(rootBindY - minBindY, 0.0f) : 0.0f;
    inst.totalMass = 0.0f;
    for (const RagdollBody& b : inst.bodies) inst.totalMass += b.mass;
    inst.totalMass = glm::max(inst.totalMass, 0.001f);

    // --- pass 2: joints (skip the root) -------------------------------------
    for (size_t k = 0; k < defs.size(); ++k) {
        const RagdollBodyDef& def = defs[k];
        if (def.parentBoneName.empty()) continue;
        auto itSelf = nameToSlot.find(def.boneName);
        auto itPar  = nameToSlot.find(def.parentBoneName);
        if (itSelf == nameToSlot.end() || itPar == nameToSlot.end()) continue;

        JPH::BodyID selfId = inst.bodies[itSelf->second].id;
        JPH::BodyID parId  = inst.bodies[itPar->second].id;

        // Feed an in-memory ConstraintComponent through the shared BuildConstraint.
        ConstraintComponent cc;
        cc.type   = def.jointType;
        cc.anchor = anchor[k];          // joint sits at the child bone's origin
        cc.axis   = axisWorld[k];       // twist (SwingTwist) or hinge axis, world space
        cc.swingNormalDeg = def.swingNormalDeg;
        cc.swingPlaneDeg  = def.swingPlaneDeg;
        cc.twistMinDeg    = def.twistMinDeg;
        cc.twistMaxDeg    = def.twistMaxDeg;
        cc.hasLimits      = true;       // hinge: clamp; ignored by the other types
        cc.limitMin       = def.hingeMinDeg;
        cc.limitMax       = def.hingeMaxDeg;
        // Bake a Position-mode motor only on articulated joints. Fixed/point joints
        // have no rotational target and must never be treated as muscles; in particular,
        // a fixed terminal foot is a rigid extension of its ankle body.
        const bool articulated = def.jointType == ConstraintType::Hinge
                               || def.jointType == ConstraintType::SwingTwist;
        cc.motorMode      = articulated ? MotorMode::Position : MotorMode::Off;
        cc.motorMaxTorque = def.motorMaxTorque;
        cc.motorFrequency = def.motorFrequency;
        cc.motorDamping   = def.motorDamping;

        // Lock both bodies together (sorted internally), build, register. body1 =
        // parent, body2 = self — matching CreateConstraint's convention.
        JPH::BodyID ids[2] = { parId, selfId };
        JPH::BodyLockMultiWrite lock(impl.joltSystem->GetBodyLockInterface(), ids, 2);
        JPH::Body* parent = lock.GetBody(0);
        JPH::Body* self   = lock.GetBody(1);
        if (!parent || !self) continue;
        JPH::Ref<JPH::TwoBodyConstraint> c = BuildConstraint(cc, *parent, *self);
        if (!c) continue;

        impl.joltSystem->AddConstraint(c);
        uint32_t cid = impl.nextConstraintId++;
        impl.constraintMap[cid] = c;
        impl.bodyToConstraints[selfId.GetIndexAndSequenceNumber()].push_back(cid);
        impl.bodyToConstraints[parId.GetIndexAndSequenceNumber()].push_back(cid);

        // Keep the data needed to re-bake this joint at the live pose on the flip
        // to Limp (RebuildRagdollJointsAtPose). cc here carries type + limits; the
        // world anchor/axis are recomputed from the child body at that time.
        RagdollJoint joint;
        joint.constraintId   = cid;
        joint.parentSlot     = itPar->second;
        joint.childSlot      = itSelf->second;
        joint.cc             = cc;
        joint.twistAxisLocal = def.jointType == ConstraintType::Hinge
            ? def.hingeAxisLocal : def.twistAxisLocal;
        // The constraint's zero = the child-relative-to-parent body orientation now (the
        // bind pose). Get-up hinge targets reference this. (See RagdollJoint::buildRel.)
        joint.buildRel       = glm::normalize(glm::conjugate(FromJolt(parent->GetRotation())) *
                                              FromJolt(self->GetRotation()));
        inst.joints.push_back(joint);
    }

    if (inst.bodies.empty()) return;

    // Solver-backed orientation spring, disabled until Test 7 requests it. Constraint X
    // is world up, so RotationX is heading/yaw and RotationY/Z are the two tilt axes. All
    // translations and rotations are free at the limit level: only force-limited position
    // motors can act, and the solver sees the planted limb chain.
    if (inst.rootBodySlot >= 0) {
        const JPH::BodyID rootId = inst.bodies[inst.rootBodySlot].id;
        JPH::BodyLockWrite lock(impl.joltSystem->GetBodyLockInterface(), rootId);
        if (lock.Succeeded()) {
            JPH::SixDOFConstraintSettings settings;
            settings.mSpace = JPH::EConstraintSpace::WorldSpace;
            settings.mPosition1 = settings.mPosition2 =
                lock.GetBody().GetCenterOfMassPosition();
            settings.mAxisX1 = settings.mAxisX2 = JPH::Vec3::sAxisY();
            settings.mAxisY1 = settings.mAxisY2 = JPH::Vec3::sAxisZ();
            settings.mMotorSettings[JPH::SixDOFConstraintSettings::RotationX]
                .mSpringSettings = JPH::SpringSettings(
                    JPH::ESpringMode::StiffnessAndDamping, 180.0f, 50.0f);
            settings.mMotorSettings[JPH::SixDOFConstraintSettings::RotationX]
                .SetTorqueLimit(80.0f);
            for (int axis = JPH::SixDOFConstraintSettings::RotationY;
                 axis <= JPH::SixDOFConstraintSettings::RotationZ; ++axis) {
                settings.mMotorSettings[axis].mSpringSettings = JPH::SpringSettings(
                    JPH::ESpringMode::StiffnessAndDamping, 700.0f, 100.0f);
                settings.mMotorSettings[axis].SetTorqueLimit(250.0f);
            }
            JPH::Ref<JPH::TwoBodyConstraint> upright = settings.Create(
                JPH::Body::sFixedToWorld, lock.GetBody());
            if (upright) {
                auto* motor = static_cast<JPH::SixDOFConstraint*>(upright.GetPtr());
                motor->SetTargetOrientationCS(JPH::Quat::sIdentity());
                impl.joltSystem->AddConstraint(upright);
                const uint32_t cid = impl.nextConstraintId++;
                impl.constraintMap[cid] = upright;
                impl.bodyToConstraints[rootId.GetIndexAndSequenceNumber()].push_back(cid);
                inst.locomotionUprightConstraintId = cid;
            }
        }
    }
    rag._ragdollId = impl.nextRagdollId++;
    rag.mode       = RagdollMode::Animated;
    inst.mode      = RagdollMode::Animated;
    spdlog::info("BuildRagdoll: {} bodies, {} joints (group {})",
                 inst.bodies.size(), inst.joints.size(), inst.group);
    impl.ragdolls[rag._ragdollId] = std::move(inst);
}

// Re-bake every joint of a ragdoll at the bodies' CURRENT (live, animation-driven)
// transforms. The kinematic follow drives the bodies to the animation pose, but the
// joints were created at the bind pose, so each swing/twist cone (and hinge range)
// is centered on the bind-relative orientation. Flipping to Dynamic from a pose far
// from bind would start a limb well outside its limit, and the solver would yank it
// back violently — the classic ragdoll "explosion." Recreating the joints here
// re-centers every limit on the pose the ragdoll actually starts flopping from, so
// no limb is born out of range. Call while the bodies are still kinematic (i.e. at
// their live targets), just before flipping them Dynamic. See Docs/ragdoll-design.md.
static void RebuildRagdollJointsAtPose(PhysicsSystem::Impl& impl, RagdollInstance& inst)
{
    for (RagdollJoint& j : inst.joints) {
        if (j.parentSlot < 0 || j.childSlot < 0) continue;
        JPH::BodyID parId  = inst.bodies[j.parentSlot].id;
        JPH::BodyID selfId = inst.bodies[j.childSlot].id;

        // Drop the stale (bind-pose) joint.
        auto cit = impl.constraintMap.find(j.constraintId);
        if (cit != impl.constraintMap.end()) {
            impl.joltSystem->RemoveConstraint(cit->second);
            impl.constraintMap.erase(cit);
        }

        JPH::BodyID ids[2] = { parId, selfId };
        JPH::BodyLockMultiWrite lock(impl.joltSystem->GetBodyLockInterface(), ids, 2);
        JPH::Body* parent = lock.GetBody(0);
        JPH::Body* self   = lock.GetBody(1);
        if (!parent || !self) continue;

        // Rebuild from the live child-body frame: anchor at the bone origin (the
        // body's reference point), twist/hinge axis = the bone-local axis rotated
        // into the body's current world orientation.
        ConstraintComponent cc = j.cc;
        JPH::RVec3 p = self->GetPosition();
        cc.anchor = glm::vec3((float)p.GetX(), (float)p.GetY(), (float)p.GetZ());
        cc.axis   = FromJolt(self->GetRotation() * ToJolt(SafeAxis(j.twistAxisLocal, j.twistAxisLocal)));

        JPH::Ref<JPH::TwoBodyConstraint> c = BuildConstraint(cc, *parent, *self);
        if (!c) continue;
        impl.joltSystem->AddConstraint(c);
        uint32_t cid = impl.nextConstraintId++;
        impl.constraintMap[cid] = c;
        impl.bodyToConstraints[selfId.GetIndexAndSequenceNumber()].push_back(cid);
        impl.bodyToConstraints[parId.GetIndexAndSequenceNumber()].push_back(cid);
        j.constraintId = cid;
        // New zero: the joint is now anchored at the live (sprawled) pose, so update
        // buildRel for the get-up hinge math that references it.
        j.buildRel = glm::normalize(glm::conjugate(FromJolt(parent->GetRotation())) *
                                    FromJolt(self->GetRotation()));
    }
}

// Per sub-step (Animated mode): drive every ragdoll body to its bone's current
// model->world transform via MoveKinematic, so a body that later goes Dynamic
// already carries the right velocity (concern #1). The animation palette is the
// source — reconstructing world[i] = palette[i] * inverse(inverseBind) avoids a
// second pose evaluation.
static void SyncRagdollKinematic(Scene& scene, PhysicsSystem::Impl& impl, JPH::BodyInterface& bi)
{
    if (impl.ragdolls.empty()) return;
    auto& reg = scene.GetRegistry();
    for (auto& [id, inst] : impl.ragdolls) {
        if (inst.mode != RagdollMode::Animated || !reg.valid(inst.entity)) continue;
        auto* smc = reg.try_get<SkinnedMeshComponent>(inst.entity);
        if (!smc) continue;
        auto* anim = reg.try_get<AnimatorComponent>(inst.entity);
        const Diamond::Skeleton& skel = smc->skeleton;
        const int n = (int)skel.bones.size();
        const bool hasPalette = anim && (int)anim->palette.size() == n;

        glm::mat4 entityWorld = scene.GetTransformSystem().GetWorldMatrix(inst.entity);
        for (const RagdollBody& b : inst.bodies) {
            if (b.boneIndex < 0 || b.boneIndex >= n) continue;
            glm::mat4 boneModel = glm::inverse(skel.bones[b.boneIndex].inverseBind);
            if (hasPalette) boneModel = anim->palette[b.boneIndex] * boneModel;
            glm::mat4 bw  = entityWorld * boneModel;
            glm::vec3 pos = glm::vec3(bw[3]);
            glm::quat rot = SafeOrientation(bw);
            // A non-finite pose (degenerate palette/bind matrix) would inject NaN
            // into the kinematic body and later crash the constraint solver — skip
            // this body for the frame rather than poison the simulation.
            if (!std::isfinite(pos.x) || !std::isfinite(pos.y) || !std::isfinite(pos.z)) continue;
            bi.MoveKinematic(b.id, ToJolt(pos), ToJolt(rot), FIXED_DT);
        }
    }
}

// The live animation world transform for one ragdoll body (same reconstruction as
// SyncRagdollKinematic). Returns false for a degenerate/non-finite pose.
static bool RagdollBodyAnimTransform(Scene& scene, RagdollInstance& inst,
                                     const SkinnedMeshComponent& smc, const AnimatorComponent* anim,
                                     const RagdollBody& b, glm::vec3& outPos, glm::quat& outRot)
{
    const Diamond::Skeleton& skel = smc.skeleton;
    const int n = (int)skel.bones.size();
    if (b.boneIndex < 0 || b.boneIndex >= n) return false;
    const bool hasPalette = anim && (int)anim->palette.size() == n;
    glm::mat4 entityWorld = scene.GetTransformSystem().GetWorldMatrix(inst.entity);
    glm::mat4 boneModel = glm::inverse(skel.bones[b.boneIndex].inverseBind);
    if (hasPalette) boneModel = anim->palette[b.boneIndex] * boneModel;
    glm::mat4 bw  = entityWorld * boneModel;
    glm::vec3 pos = glm::vec3(bw[3]);
    glm::quat rot = SafeOrientation(bw);
    if (!std::isfinite(pos.x) || !std::isfinite(pos.y) || !std::isfinite(pos.z)) return false;
    outPos = pos; outRot = rot;
    return true;
}

// Per sub-step (get-up cross-blend): ease every body kinematically from its captured held-
// stand transform toward its live animation transform, so handing back to Animated doesn't
// pop. When the window elapses TickRagdollReactions finalizes the switch to Animated.
//
// The blend is done HIERARCHICALLY in each body's parent-relative (local) frame, not on its
// world transform: we decompose both endpoints — A = the held-stand pose, B = the live
// animation pose — into a rotation/offset relative to the parent body, slerp/lerp there, then
// forward-compose down the joint tree so every child rides its parent's already-blended frame.
// Blending world transforms per body independently (the old path) let each limb take its own
// straight-line path through space, so mid-blend they drifted off their joints and the bones
// stretched/detached — that disjointed flicker was the get-up pop. Composing in local space
// keeps the rig rigid: limbs rotate about their joints and the endpoints reproduce exactly.
static void SyncRagdollGetUpBlend(Scene& scene, PhysicsSystem::Impl& impl, JPH::BodyInterface& bi)
{
    if (impl.ragdolls.empty()) return;
    auto& reg = scene.GetRegistry();
    for (auto& [id, inst] : impl.ragdolls) {
        if (inst.reaction.kind != RagReaction::Kind::GetUpBlend || !reg.valid(inst.entity)) continue;
        auto* smc = reg.try_get<SkinnedMeshComponent>(inst.entity);
        if (!smc) continue;
        auto* anim = reg.try_get<AnimatorComponent>(inst.entity);

        const int n = (int)inst.bodies.size();
        if (n == 0) continue;

        const float t = glm::clamp(inst.reaction.timer /
                                   glm::max(inst.reaction.duration, 1e-4f), 0.0f, 1.0f);
        const float e = t * t * (3.0f - 2.0f * t);   // smoothstep

        // Endpoint world transforms per body: A = captured held-stand pose, B = live anim pose.
        std::vector<glm::vec3> aPos(n), bPos(n);
        std::vector<glm::quat> aRot(n), bRot(n);
        std::vector<int>       parent(n, -1);
        for (int i = 0; i < n; ++i) {
            const RagdollBody& body = inst.bodies[i];
            aPos[i] = body.blendStartPos;
            aRot[i] = glm::normalize(body.blendStartRot);
            glm::vec3 p; glm::quat r;
            if (RagdollBodyAnimTransform(scene, inst, *smc, anim, body, p, r)) {
                bPos[i] = p; bRot[i] = glm::normalize(r);
            } else {                       // degenerate anim pose: hold this body at its start
                bPos[i] = aPos[i]; bRot[i] = aRot[i];
            }
        }
        // Body tree from the joints (child -> parent); the root (hips) stays parentless.
        for (const RagdollJoint& j : inst.joints)
            if (j.childSlot >= 0 && j.childSlot < n && j.parentSlot >= 0 && j.parentSlot < n)
                parent[j.childSlot] = j.parentSlot;

        // Visit parents before children so each recompose can read its parent's blended frame.
        std::vector<int>  order; order.reserve(n);
        std::vector<char> placed(n, 0);
        for (int i = 0; i < n; ++i) if (parent[i] < 0) { order.push_back(i); placed[i] = 1; }
        for (size_t h = 0; h < order.size(); ++h)
            for (int i = 0; i < n; ++i)
                if (!placed[i] && parent[i] == order[h]) { order.push_back(i); placed[i] = 1; }
        for (int i = 0; i < n; ++i) if (!placed[i]) { parent[i] = -1; order.push_back(i); }  // stragglers: world-relative

        std::vector<glm::vec3> outPos(n); std::vector<glm::quat> outRot(n);
        for (int slot : order) {
            const int par = parent[slot];
            // Decompose each endpoint into its parent's frame (root: relative to world).
            glm::quat aLoc, bLoc; glm::vec3 aOff, bOff;
            if (par < 0) {
                aLoc = aRot[slot]; aOff = aPos[slot];
                bLoc = bRot[slot]; bOff = bPos[slot];
            } else {
                aLoc = glm::conjugate(aRot[par]) * aRot[slot];
                aOff = glm::conjugate(aRot[par]) * (aPos[slot] - aPos[par]);
                bLoc = glm::conjugate(bRot[par]) * bRot[slot];
                bOff = glm::conjugate(bRot[par]) * (bPos[slot] - bPos[par]);
            }
            aLoc = glm::normalize(aLoc); bLoc = glm::normalize(bLoc);
            if (glm::dot(aLoc, bLoc) < 0.0f) bLoc = -bLoc;   // shortest arc
            const glm::quat locR = glm::slerp(aLoc, bLoc, e);
            const glm::vec3 locP = glm::mix(aOff, bOff, e);
            // Re-compose onto the parent's already-blended world frame.
            if (par < 0) {
                outRot[slot] = glm::normalize(locR);
                outPos[slot] = locP;
            } else {
                outRot[slot] = glm::normalize(outRot[par] * locR);
                outPos[slot] = outPos[par] + outRot[par] * locP;
            }
            bi.MoveKinematic(inst.bodies[slot].id, ToJolt(outPos[slot]),
                             ToJolt(outRot[slot]), FIXED_DT);
        }
    }
}

// Engage or disengage the Position motors baked on every ragdoll joint. Powered
// mode turns them on (driven toward targets by SyncRagdollPowered); Animated/Limp
// turn them off. Kinematic bodies ignore motors, so this mainly matters on the
// Powered→on and Limp→off transitions (and after a joint rebuild, which recreates
// the constraints with their baked Position motor re-enabled).
static void SetRagdollMotorsEnabled(PhysicsSystem::Impl& impl, RagdollInstance& inst, bool on)
{
    const JPH::EMotorState state = on ? JPH::EMotorState::Position : JPH::EMotorState::Off;
    for (const RagdollJoint& j : inst.joints) {
        auto it = impl.constraintMap.find(j.constraintId);
        if (it == impl.constraintMap.end()) continue;
        JPH::Constraint* c = it->second;
        switch (c->GetSubType()) {
            case JPH::EConstraintSubType::Hinge:
                static_cast<JPH::HingeConstraint*>(c)->SetMotorState(state);
                break;
            case JPH::EConstraintSubType::SwingTwist: {
                auto* st = static_cast<JPH::SwingTwistConstraint*>(c);
                st->SetSwingMotorState(state);
                st->SetTwistMotorState(state);
                break;
            }
            default: break;
        }
    }
}

// The get-up root drive (per sub-step, get-up only). Joint motors can't lift the body — the
// root (hips) has no parent joint, so no motor acts on it; and a velocity/force on the single
// root body just gets absorbed by the 18-body chain hanging off its joints (verified: the
// hips stall ~0.27 above the feet and never reach the ~0.66 standing clearance). So during the
// heave the hips are KINEMATIC and we MoveKinematic them along an eased arc from the sprawled
// start transform to the standing transform (fixed absolute height + upright orientation). A
// kinematic body reaches its target regardless of the chain, dragging the body up; the limb
// motors + wobble then pose it underneath for the sloppy stand. getupBalance scales how far
// toward fully-upright/standing-height we drive (0 = no kinematic drive, the legacy behavior).
static void DriveRagdollGetUpRoot(PhysicsSystem::Impl& impl, RagdollInstance& inst,
                                  float /*strength*/, float balance)
{
    if (inst.rootBodySlot < 0 || balance <= 0.0f) return;
    JPH::BodyInterface& bi = impl.joltSystem->GetBodyInterface();
    const RagdollBody& root = inst.bodies[inst.rootBodySlot];

    // Eased progress along the heave (smoothstep), clamped — holds at 1 once at the top.
    const float gt = glm::clamp(inst.reaction.timer /
                                glm::max(inst.reaction.duration, 1e-4f), 0.0f, 1.0f);
    const float ge = gt * gt * (3.0f - 2.0f * gt);
    const float a  = ge * glm::clamp(balance, 0.0f, 1.0f);   // balance scales the destination

    // Target: keep the start XZ, lerp height up to the fixed standing target, and slerp the
    // orientation from the sprawled start toward upright (bind). MoveKinematic sets the body's
    // velocity to arrive in one step, so the constrained limbs get dragged along smoothly.
    glm::vec3 targetPos = inst.getupStartPos;
    targetPos.y         = glm::mix(inst.getupStartPos.y, inst.getupTargetY, a);
    glm::quat targetRot = glm::slerp(inst.getupStartRot, inst.rootBindRot, a);

    bi.MoveKinematic(root.id, ToJolt(targetPos), ToJolt(glm::normalize(targetRot)), FIXED_DT);
}

// Disengage the locomotion root drive so a reaction (flinch impulse, get-up's own
// kinematic heave) can take over the root cleanly. Ensures the root is Dynamic (a
// no-op under the physical drive, which never kinematizes it) and clears the flag;
// the controller reclaims the root next frame (DriveRagdollLocomotionRoot) once the
// reaction ends via ClearRagdollReaction / SetRagdollMode, if still commanding movement.
static void ReleaseLocomotionRoot(PhysicsSystem::Impl& impl, RagdollInstance& inst)
{
    if (!inst._locomotionDriving || inst.rootBodySlot < 0) return;
    JPH::BodyInterface& bi = impl.joltSystem->GetBodyInterface();
    bi.SetMotionType(inst.bodies[inst.rootBodySlot].id, JPH::EMotionType::Dynamic, JPH::EActivation::Activate);
    inst._locomotionDriving = false;
}

// Drives the dynamic pelvis toward the locomotion velocity and heading.
static void DriveRagdollLocomotionRoot(PhysicsSystem::Impl& impl, RagdollInstance& inst,
                                       RagdollComponent& rag)
{
    if (inst.rootBodySlot < 0) return;
    rag._locomotionUprightDeltaAngularVelocity = glm::vec3(0.0f);
    rag._locomotionUprightTorque = glm::vec3(0.0f);
    rag._locomotionUprightTorqueActive = false;
    rag._locomotionUprightSaturated = false;
    rag._locomotionHeadingErrorDeg = 0.0f;
    rag._locomotionHeadingSaturated = false;
    if (inst.reaction.kind != RagReaction::Kind::None) {
        DisableLocomotionUprightConstraint(impl, inst);
        return;   // a reaction owns the root
    }

    JPH::BodyInterface& bi = impl.joltSystem->GetBodyInterface();
    const RagdollBody& root = inst.bodies[inst.rootBodySlot];
    const bool want = rag.locomotionActive &&
                      std::isfinite(rag.locomotionTargetVel.x) &&
                      std::isfinite(rag.locomotionTargetVel.y) &&
                      std::isfinite(rag.locomotionTargetVel.z);

    if (want != inst._locomotionDriving) {
        bi.SetMotionType(root.id, JPH::EMotionType::Dynamic, JPH::EActivation::Activate);  // forces/contacts act on it
        inst._locomotionDriving = want;
    }
    if (!want) {
        DisableLocomotionUprightConstraint(impl, inst);
        return;
    }

    const glm::quat curRot = FromJolt(bi.GetRotation(root.id));
    const glm::vec3 upNow  = curRot * (glm::conjugate(inst.rootBindRot) * glm::vec3(0, 1, 0));
    const glm::vec3 leanH(upNow.x, 0.0f, upNow.z);

    const glm::quat targetHeading = glm::normalize(rag.locomotionTargetRot);
    const glm::quat targetRootRot = glm::normalize(targetHeading * inst.rootBindRot);
    glm::vec3 actualForward = curRot
        * (glm::conjugate(inst.rootBindRot) * glm::vec3(0.0f, 0.0f, -1.0f));
    glm::vec3 desiredForward = targetHeading * glm::vec3(0.0f, 0.0f, -1.0f);
    actualForward.y = desiredForward.y = 0.0f;
    if (glm::dot(actualForward, actualForward) > 1e-8f
        && glm::dot(desiredForward, desiredForward) > 1e-8f) {
        actualForward = glm::normalize(actualForward);
        desiredForward = glm::normalize(desiredForward);
        rag._locomotionHeadingErrorDeg = glm::degrees(std::atan2(
            glm::dot(glm::cross(actualForward, desiredForward), glm::vec3(0, 1, 0)),
            glm::clamp(glm::dot(actualForward, desiredForward), -1.0f, 1.0f)));
    }

    const float tiltDeg = glm::degrees(std::acos(glm::clamp(upNow.y, -1.0f, 1.0f)));
    const glm::vec3 vCur = FromJolt(bi.GetLinearVelocity(root.id));
    const glm::vec3 wCur = FromJolt(bi.GetAngularVelocity(root.id));
    rag._locomotionRootVel = vCur;
    rag._locomotionRootAngularVelocity = wCur;
    glm::vec3 moveF(0.0f), balF(0.0f);
    rag._locomotionSupportForce = glm::vec3(0.0f);
    rag._locomotionSupportPositionForce = glm::vec3(0.0f);
    rag._locomotionSupportDampingForce = glm::vec3(0.0f);
    rag._locomotionSupportSaturated = false;
    rag._locomotionLiftForce = 0.0f;

    // Mass-weighted COM and the support segment formed by grounded feet.
    glm::vec3 com(0.0f), comVel(0.0f); float totalMass = 0.0f;
    for (const RagdollBody& b : inst.bodies) {
        com += FromJolt(bi.GetPosition(b.id)) * b.mass;
        comVel += FromJolt(bi.GetLinearVelocity(b.id)) * b.mass;
        totalMass += b.mass;
    }
    rag._locomotionCOMValid = totalMass > 1e-6f;
    if (rag._locomotionCOMValid) {
        com /= totalMass;
        comVel /= totalMass;
    }
    rag._locomotionCOM = com;
    rag._locomotionCOMVel = comVel;

    glm::vec3 footXZ[2]; int footCount = 0;
    int footSlot = 0;
    rag._locomotionFootBones[0] = rag._locomotionFootBones[1] = -1;
    rag._locomotionFootGrounded[0] = rag._locomotionFootGrounded[1] = false;
    for (const RagdollBody& b : inst.bodies) {
        if (!b.isFoot) continue;
        const glm::vec3 fp = FromJolt(bi.GetPosition(b.id));
        HitResult fg = Physics::Raycast(fp, glm::vec3(0, -1, 0), b.halfExtents.y + 0.05f, inst.entity, false);
        if (footSlot < 2) {
            rag._locomotionFootBones[footSlot] = b.boneIndex;
            rag._locomotionFootGrounded[footSlot] = fg.hit;
            ++footSlot;
        }
        if (fg.hit && footCount < 2)
            footXZ[footCount++] = glm::vec3(fp.x, 0.0f, fp.z);
    }

    // A planted foot must be a physical fact, not merely an IK target. Apply a capped
    // tangential spring at each commanded plant point. There is intentionally no equal and
    // opposite body force: this is the static-friction reaction supplied by the world. Y is
    // untouched so gravity/contact still decide whether the sole actually bears weight.
    bool footLockActive = false;
    for (int lockSlot = 0; lockSlot < 2; ++lockSlot) {
        rag._locomotionFootLockError[lockSlot] = 0.0f;
        rag._locomotionFootLockForce[lockSlot] = 0.0f;
        const int lockBone = rag.locomotionFootLockBones[lockSlot];
        const glm::vec3 target = rag.locomotionFootLockTargets[lockSlot];
        const float lockWeight = glm::clamp(rag.locomotionFootLockWeights[lockSlot],
                                            0.0f, 1.0f);
        if (lockBone < 0 || lockWeight <= 0.001f
            || !std::isfinite(target.x) || !std::isfinite(target.z)) continue;

        for (const RagdollBody& body : inst.bodies) {
            if (!body.isFoot || body.boneIndex != lockBone) continue;
            bool grounded = false;
            for (int foot = 0; foot < 2; ++foot) {
                if (rag._locomotionFootBones[foot] == lockBone) {
                    grounded = rag._locomotionFootGrounded[foot];
                    break;
                }
            }

            const glm::vec3 position = FromJolt(bi.GetPosition(body.id));
            glm::vec3 error = target - position;
            error.y = 0.0f;
            rag._locomotionFootLockError[lockSlot] = glm::length(error);
            footLockActive = footLockActive || grounded;
            if (!grounded) break;

            glm::vec3 velocity = FromJolt(bi.GetLinearVelocity(body.id));
            velocity.y = 0.0f;
            const float omega = glm::two_pi<float>()
                              * glm::max(rag.locomotionFootLockFrequency, 0.0f);
            glm::vec3 force = (omega * omega * error
                             - 2.0f * glm::max(rag.locomotionFootLockDamping, 0.0f)
                               * omega * velocity)
                            * glm::max(rag.locomotionFootLockEffectiveMass, 0.0f)
                            * lockWeight;
            const float maxForce = glm::max(rag.locomotionFootLockMaxForce, 0.0f)
                                 * lockWeight;
            if (const float magnitude = glm::length(force);
                magnitude > maxForce && magnitude > 1e-6f)
                force *= maxForce / magnitude;
            bi.AddForce(body.id, ToJolt(force));
            rag._locomotionFootLockForce[lockSlot] = glm::length(force);
            break;
        }
    }

    const glm::vec3 comXZ(com.x, 0.0f, com.z);
    auto baseSeparation = [&](const glm::vec3& queryXZ) -> glm::vec3 {
        if (footCount == 0) return glm::vec3(0.0f);
        glm::vec3 closest = footXZ[0];
        if (footCount == 2) {
            const glm::vec3 ab     = footXZ[1] - footXZ[0];
            const float     abLen2 = glm::dot(ab, ab);
            const float     t      = abLen2 > 1e-8f
                ? glm::clamp(glm::dot(queryXZ - footXZ[0], ab) / abLen2, 0.0f, 1.0f) : 0.0f;
            closest = footXZ[0] + ab * t;
        }
        const glm::vec3 toQuery = queryXZ - closest;
        const float     dist    = glm::length(toQuery);
        const float     outside = glm::max(0.0f, dist - glm::max(rag.locomotionSupportRadius, 0.0f));
        return outside > 1e-5f ? (toQuery / dist) * outside : glm::vec3(0.0f);
    };

    const glm::vec3 baseErr = baseSeparation(comXZ);
    const float comOutside = glm::length(baseErr);

    const glm::vec3 hipPosNow = FromJolt(bi.GetPosition(root.id));
    const glm::vec3 hipErr     = baseSeparation(glm::vec3(hipPosNow.x, 0.0f, hipPosNow.z));
    const float hipOutside = glm::length(hipErr);

    // SIMBICON virtual hip torques and their reaction on the root. Applied here (per substep)
    // rather than script-side so the torque lands on every step regardless of frame rate.
    if (rag.locomotionSimbiconBlend > 0.001f) {
        glm::vec3 reaction(0.0f);
        for (int i = 0; i < 2; ++i) {
            const int bone = rag.locomotionHipBones[i];
            const glm::vec3& torque = rag.locomotionHipTorque[i];
            if (bone < 0 || !std::isfinite(torque.x) || !std::isfinite(torque.y) ||
                !std::isfinite(torque.z)) continue;
            for (const RagdollBody& b : inst.bodies) {
                if (b.boneIndex != bone) continue;
                bi.AddTorque(b.id, ToJolt(torque));
                reaction -= torque;
                break;
            }
        }
        bi.AddTorque(root.id, ToJolt(reaction));   // net on the pelvis = tau_torso
    }

    // Assisted swing-foot unload. Joint motors can fold the knee correctly while a sticky
    // sole remains planted; in that case they drag the body backward instead of initiating
    // swing. This is deliberately a ONE-SIDED spring: once the sole reaches or passes the
    // target it stops lifting rather than reversing and hammering the foot back into the
    // floor. The equal-and-opposite pelvis reaction keeps it an internal force.
    if (rag.locomotionLiftBone >= 0 && std::isfinite(rag.locomotionLiftTargetY)) {
        for (const RagdollBody& body : inst.bodies) {
            if (body.boneIndex != rag.locomotionLiftBone) continue;
            const float y = (float)bi.GetPosition(body.id).GetY();
            const float vy = bi.GetLinearVelocity(body.id).GetY();
            const float omega = glm::two_pi<float>() * glm::max(rag.locomotionLiftFrequency, 0.0f);
            const float accel = omega * omega * (rag.locomotionLiftTargetY - y)
                              - 2.0f * glm::max(rag.locomotionLiftDamping, 0.0f) * omega * vy;
            const float force = glm::clamp(
                accel * glm::max(rag.locomotionLiftEffectiveMass, 0.0f),
                0.0f, glm::max(rag.locomotionLiftMaxForce, 0.0f));
            const glm::vec3 lift(0.0f, force, 0.0f);
            bi.AddForce(body.id, ToJolt(lift));
            bi.AddForce(root.id, ToJolt(-lift));
            rag._locomotionLiftForce = force;
            break;
        }
    }

    // One-sided vertical support. A velocity write here destroys the vertical momentum a
    // gait needs, so under SIMBICON it becomes a force and can be scaled to zero once the
    // stance leg carries the body.
    if (rag.locomotionDynamicRoot && tiltDeg < rag.locomotionFallenTilt) {
        const glm::vec3& hipPos = hipPosNow;
        HitResult ground = Physics::Raycast(hipPos, glm::vec3(0, -1, 0),
                                            inst.standHipClearance * 1.5f + 0.3f,
                                            inst.entity, false);
        if (ground.hit) {
            const float supportFactor = 1.0f - glm::clamp(
                comOutside / glm::max(rag.locomotionSupportFalloff, 1e-4f), 0.0f, 1.0f);
            const float ws      = glm::two_pi<float>() * glm::max(rag.locomotionSupportFreq, 0.0f);
            const float targetY = ground.point.y + inst.standHipClearance
                                + rag.locomotionHeightOffset;
            const float err     = targetY - hipPos.y;
            float accel = ws * ws * err - 2.0f * ws * vCur.y;
            accel = glm::clamp(accel, 0.0f, glm::max(rag.locomotionUpAccel, 0.0f) * supportFactor);
            const float blend = glm::clamp(rag.locomotionSimbiconBlend, 0.0f, 1.0f);
            const float supportScale = glm::max(rag.locomotionSupportScale, 0.0f);
            if (footLockActive) {
                // Once a stance foot is physically anchored, keep the pelvis support a
                // force-limited safety net. A vertical velocity overwrite would erase load
                // transfer through the leg and produce the sideways "floating" failure.
                bi.AddForce(root.id, ToJolt(glm::vec3(
                    0.0f, accel * inst.totalMass * supportScale, 0.0f)));
            } else if (blend > 0.001f) {
                bi.AddForce(root.id, ToJolt(glm::vec3(
                    0.0f, accel * inst.totalMass * blend * supportScale, 0.0f)));
            }
            if (!footLockActive && blend < 0.999f)
                bi.SetLinearVelocity(root.id, ToJolt(glm::vec3(
                    vCur.x, vCur.y + accel * FIXED_DT * (1.0f - blend) * supportScale,
                    vCur.z)));
        }
    }

    // Do not drag or right a prone ragdoll.
    if (tiltDeg < rag.locomotionFallenTilt) {
        const float sb = glm::clamp(rag.locomotionSimbiconBlend, 0.0f, 1.0f);
        const float legacy = 1.0f - sb;
        const bool simbicon = sb > 0.5f;
        glm::vec3 velErr = rag.locomotionTargetVel - vCur; velErr.y = 0.0f;
        glm::vec3 desiredLean = glm::vec3(rag.locomotionTargetVel.x, 0.0f, rag.locomotionTargetVel.z)
                              * rag.locomotionMoveLean;
        if (const float l = glm::length(desiredLean); l > 0.45f) desiredLean *= 0.45f / l;
        const glm::vec3 balanceErr = footCount > 0 ? baseErr : leanH;
        const glm::vec3 normalBalanceForce =
            -(balanceErr - desiredLean) * glm::max(rag.locomotionBalanceForce, 0.0f);
        const float supportTargetWeight = glm::clamp(rag.locomotionSupportTargetWeight, 0.0f, 1.0f);
        glm::vec3 supportTargetForce = normalBalanceForce;
        if (supportTargetWeight > 0.0f &&
            std::isfinite(rag.locomotionSupportTarget.x) &&
            std::isfinite(rag.locomotionSupportTarget.z) &&
            std::isfinite(rag.locomotionSupportTargetVel.x) &&
            std::isfinite(rag.locomotionSupportTargetVel.z)) {
            const glm::vec3 targetPos(rag.locomotionSupportTarget.x, 0.0f,
                                      rag.locomotionSupportTarget.z);
            const glm::vec3 targetVel(rag.locomotionSupportTargetVel.x, 0.0f,
                                      rag.locomotionSupportTargetVel.z);
            const glm::vec3 horizontalCOMVel(comVel.x, 0.0f, comVel.z);
            const float frequency = glm::max(rag.locomotionCOMSupportFreq, 0.0f);
            const float damping = glm::max(rag.locomotionCOMSupportDamping, 0.0f);
            const float omega = glm::two_pi<float>() * frequency;
            glm::vec3 positionAccel = omega * omega * (targetPos - comXZ);
            glm::vec3 dampingAccel =
                2.0f * damping * omega * (targetVel - horizontalCOMVel);
            glm::vec3 accel = positionAccel + dampingAccel;
            const float maxAccel = glm::max(rag.locomotionCOMSupportMaxAccel, 0.0f);
            if (const float length = glm::length(accel);
                length > maxAccel && length > 1e-6f) {
                const float scale = maxAccel / length;
                positionAccel *= scale;
                dampingAccel *= scale;
                accel *= scale;
                rag._locomotionSupportSaturated = true;
            }
            supportTargetForce = accel * inst.totalMass;
            rag._locomotionSupportForce = supportTargetForce;
            rag._locomotionSupportPositionForce = positionAccel * inst.totalMass;
            rag._locomotionSupportDampingForce = dampingAccel * inst.totalMass;
        }
        moveF = velErr * glm::max(rag.locomotionMoveForce, 0.0f) * legacy;
        balF = glm::mix(normalBalanceForce, supportTargetForce, supportTargetWeight) * legacy;

        // Prevent the pelvis from outrunning the planted support base.
        if (!simbicon && hipOutside > glm::max(rag.locomotionLeashDistance, 0.0f)) {
            const glm::vec3 leashDir = hipErr / hipOutside;   // outward, away from the base
            const float     outwardF = glm::dot(moveF, leashDir);
            if (outwardF > 0.0f) moveF -= leashDir * outwardF;
            const float     outwardV = glm::dot(vCur, leashDir);
            if (outwardV > 0.0f)
                bi.SetLinearVelocity(root.id, ToJolt(vCur - leashDir * outwardV));
        }

        bi.AddForce(root.id, ToJolt(moveF + balF));

        const float uprightWeight = glm::clamp(
            rag.locomotionUprightScale, 0.0f, 1.0f) * legacy;
        if (uprightWeight > 0.001f && rag.locomotionTorqueUpright) {
            // One solver-owned spring: translation is free, pitch/roll drive the bind-
            // upright frame, and a gentler yaw axis preserves the requested heading. Jolt
            // applies the bounded impulses in the articulated joint/contact solve; there
            // is no external PD torque and no angular-velocity overwrite here.
            JPH::SixDOFConstraint* motor =
                GetLocomotionUprightConstraint(impl, inst);
            if (motor) {
                using Axis = JPH::SixDOFConstraint::EAxis;
                const float stiffness = glm::max(
                    rag.locomotionTorqueUprightStiffness, 0.0f);
                const float damping = glm::max(
                    rag.locomotionTorqueUprightDamping, 0.0f);
                const float maxTorque = glm::max(
                    rag.locomotionTorqueUprightMaxTorque, 0.0f) * uprightWeight;
                for (const Axis axis : { Axis::RotationY, Axis::RotationZ }) {
                    JPH::MotorSettings& settings = motor->GetMotorSettings(axis);
                    settings.mSpringSettings = JPH::SpringSettings(
                        JPH::ESpringMode::StiffnessAndDamping,
                        stiffness, damping);
                    settings.SetTorqueLimit(maxTorque);
                    motor->SetMotorState(axis, JPH::EMotorState::Position);
                }
                const float headingStiffness = glm::max(
                    rag.locomotionTorqueHeadingStiffness, 0.0f);
                const float headingDamping = glm::max(
                    rag.locomotionTorqueHeadingDamping, 0.0f);
                const float headingMaxTorque = glm::max(
                    rag.locomotionTorqueHeadingMaxTorque, 0.0f) * uprightWeight;
                JPH::MotorSettings& headingSettings =
                    motor->GetMotorSettings(Axis::RotationX);
                headingSettings.mSpringSettings = JPH::SpringSettings(
                    JPH::ESpringMode::StiffnessAndDamping,
                    headingStiffness, headingDamping);
                headingSettings.SetTorqueLimit(headingMaxTorque);
                motor->SetMotorState(Axis::RotationX, JPH::EMotorState::Position);
                // Body 1 is Jolt's fixed world body, so body-space orientation is the
                // absolute desired root rotation. This preserves the F6/F7 heading instead
                // of recapturing the yaw error produced by each landing.
                motor->SetTargetOrientationBS(ToJolt(targetRootRot));

                // The lambda is the previous solver step's constraint-space angular
                // impulse. Rotate it into world space for causal diagnostics.
                const JPH::Vec3 torqueCS =
                    motor->GetTotalLambdaMotorRotation() / FIXED_DT;
                const JPH::Quat constraintFrameWorld =
                    bi.GetRotation(root.id)
                    * motor->GetConstraintToBody2Matrix().GetQuaternion();
                rag._locomotionUprightTorque = FromJolt(
                    constraintFrameWorld * torqueCS);
                rag._locomotionUprightTorqueActive = true;
                const float appliedAxisTorque = glm::max(
                    std::abs(torqueCS.GetY()), std::abs(torqueCS.GetZ()));
                const bool tiltSaturated = maxTorque > 1e-4f
                    && appliedAxisTorque >= maxTorque * 0.98f;
                rag._locomotionHeadingSaturated = headingMaxTorque > 1e-4f
                    && std::abs(torqueCS.GetX()) >= headingMaxTorque * 0.98f;
                rag._locomotionUprightSaturated = tiltSaturated
                    || rag._locomotionHeadingSaturated;
            }
        } else if (uprightWeight > 0.001f) {
            DisableLocomotionUprightConstraint(impl, inst);
            // Legacy implicitly damped upright spring. Torque mode bypasses this branch
            // completely; running both controllers was the source of the recovery shake.
            const glm::quat targetRot =
                glm::normalize(glm::normalize(rag.locomotionTargetRot) * inst.rootBindRot);
            glm::quat delta = glm::normalize(targetRot * glm::conjugate(curRot));
            if (delta.w < 0.0f) delta = -delta;            // short way
            const float     angle   = 2.0f * std::acos(glm::clamp(delta.w, -1.0f, 1.0f));
            const glm::vec3 axisRaw = glm::vec3(delta.x, delta.y, delta.z);
            const glm::vec3 axis    = glm::length(axisRaw) > 1e-6f ? glm::normalize(axisRaw) : glm::vec3(0.0f);
            const float ws = glm::two_pi<float>() * glm::max(rag.locomotionBalanceFreq, 0.0f);
            glm::vec3 dw = (wCur + axis * (ws * ws * angle * FIXED_DT)) / (1.0f + 2.0f * ws * FIXED_DT) - wCur;
            const float maxDw = glm::max(rag.locomotionBalanceAccel, 0.0f) * FIXED_DT;
            if (const float l = glm::length(dw); l > maxDw && l > 1e-6f) {
                dw *= maxDw / l;
                rag._locomotionUprightSaturated = true;
            }
            dw *= uprightWeight;
            rag._locomotionUprightDeltaAngularVelocity = dw;
            bi.SetAngularVelocity(root.id, ToJolt(wCur + dw));
        } else {
            DisableLocomotionUprightConstraint(impl, inst);
        }
    }

    // Debug: throttled (~10 Hz) tilt / spin / feet dump to diagnose tips + jitter.
    if (rag.locomotionDebug) {
        inst.tipTimer += FIXED_DT;
        if (inst.tipTimer >= 0.1f) {
            inst.tipTimer = 0.0f;
            const float spd    = glm::length(glm::vec2(vCur.x, vCur.z));
            const float tgtSpd = glm::length(glm::vec2(rag.locomotionTargetVel.x, rag.locomotionTargetVel.z));
            float fyL = 0, fwL = 0, fyR = 0, fwR = 0; int fi = 0;
            for (const RagdollBody& b : inst.bodies) {
                if (!b.isFoot) continue;
                const glm::vec3 fp = FromJolt(bi.GetPosition(b.id));
                const float fw = glm::length(FromJolt(bi.GetAngularVelocity(b.id)));
                if (fi == 0) { fyL = fp.y; fwL = fw; } else { fyR = fp.y; fwR = fw; }
                ++fi;
            }
            const glm::vec3 tipR = glm::cross(wCur, upNow);
            spdlog::info("[Loco] spd={:.2f}/{:.2f} vel=({:+.2f},{:+.2f}) lean=({:+.2f},{:+.2f}) tipRate={:.2f} moveF={:.0f} balF={:.0f} rootW={:.1f} rootY={:.2f} feetY=({:.2f},{:.2f}) feetW=({:.0f},{:.0f}) feetDown={} comOut={:.3f} hipOut={:.3f} lockE=({:.3f},{:.3f}) lockF=({:.0f},{:.0f})",
                         spd, tgtSpd, vCur.x, vCur.z, upNow.x, upNow.z,
                         glm::length(glm::vec2(tipR.x, tipR.z)),
                         glm::length(glm::vec2(moveF.x, moveF.z)), glm::length(glm::vec2(balF.x, balF.z)),
                         glm::length(wCur), FromJolt(bi.GetPosition(root.id)).y, fyL, fyR, fwL, fwR,
                         footCount, comOutside, hipOutside,
                         rag._locomotionFootLockError[0], rag._locomotionFootLockError[1],
                         rag._locomotionFootLockForce[0], rag._locomotionFootLockForce[1]);
        }
    }
}

// Per sub-step (Powered mode): drive every joint motor toward the live animation
// pose so the ragdoll actively holds / recovers an animated stance (active ragdoll).
// Unlike SyncRagdollKinematic the bodies are Dynamic — the motors, not MoveKinematic,
// move them, so the rig reacts to and pushes against the world while still chasing
// the animation. The swing-twist target is the desired orientation of the child body
// relative to the parent body: SetTargetOrientationBS wants exactly R1^-1 * R2 (the
// same convention as the restBS line in BuildConstraint), which we read straight from
// the retained local Pose by forward-composing model-space bone rotations. Hinges get
// the scalar angle about their axis, referenced to the bind pose (the hinge's zero).
// Motor torque is scaled by the rig's strength each frame, so a hit-reaction or
// get-up blend can soften the muscles by lowering RagdollComponent::strength.
//
// During a get-up reaction (reaction.targetStand) the target flips from the animation
// to the bind/stand pose, the hinge zero is taken from the joint's live buildRel (the
// cones were opened so a sprawled limb can straighten), and per-joint torque is jittered
// by reaction.wobble for the sloppy drunken flail.
static void SyncRagdollPowered(Scene& scene, PhysicsSystem::Impl& impl)
{
    if (impl.ragdolls.empty()) return;
    auto& reg = scene.GetRegistry();
    JPH::BodyInterface& bi = impl.joltSystem->GetBodyInterface();
    for (auto& [id, inst] : impl.ragdolls) {
        if (inst.mode != RagdollMode::Powered || !reg.valid(inst.entity)) continue;
        // The get-up tail cross-blend drives the bodies kinematically (SyncRagdollGetUpBlend);
        // the motors are off, so skip the Powered motor/root drive for it this whole phase.
        if (inst.reaction.kind == RagReaction::Kind::GetUpBlend) continue;
        auto* smc  = reg.try_get<SkinnedMeshComponent>(inst.entity);
        auto* anim = reg.try_get<AnimatorComponent>(inst.entity);
        auto* rag  = reg.try_get<RagdollComponent>(inst.entity);
        if (!smc || !anim) continue;
        if (rag) {
            for (int i = 0; i < 6; ++i) {
                rag->_locomotionMotorBones[i] = -1;
                rag->_locomotionMotorAppliedTorque[i] = 0.0f;
                rag->_locomotionMotorTorqueLimit[i] = 0.0f;
                rag->_locomotionMotorSaturationRatio[i] = 0.0f;
                rag->_locomotionMotorSaturated[i] = false;
            }
        }
        const Diamond::Skeleton& skel = smc->skeleton;
        const int n = (int)skel.bones.size();
        if ((int)anim->pose.size() != n) continue;   // pose not built yet this frame

        const float strength = rag ? glm::clamp(rag->strength, 0.0f, 4.0f) : 1.0f;   // >1 = over-drive muscles
        const bool  toStand  = inst.reaction.targetStand;   // get-up: chase the stand pose
        const float wobble   = inst.reaction.wobble;        // get-up: per-joint torque jitter
        const bool logLegMotors = rag && rag->locomotionActive &&
                                  impl.simTime >= impl.locoMotorDebugNext;
        bool loggedLegMotor = false;

        // Model-space bone rotations for the live (animated) pose and for the bind
        // pose, forward-composed down the skeleton. Rotations only — translation and
        // scale don't change a joint's relative orientation.
        std::vector<glm::quat> animRot(n), bindRot(n);
        for (int i = 0; i < n; ++i) {
            int p = skel.bones[i].parent;
            glm::quat la = glm::normalize(anim->pose[i].rotation);
            glm::quat lb = glm::normalize(skel.bones[i].localR);
            animRot[i] = (p < 0) ? la : glm::normalize(animRot[p] * la);
            bindRot[i] = (p < 0) ? lb : glm::normalize(bindRot[p] * lb);
        }

        for (const RagdollJoint& j : inst.joints) {
            if (j.parentSlot < 0 || j.childSlot < 0) continue;
            int pb = inst.bodies[j.parentSlot].boneIndex;
            int cb = inst.bodies[j.childSlot].boneIndex;
            if (pb < 0 || pb >= n || cb < 0 || cb >= n) continue;

            auto cit = impl.constraintMap.find(j.constraintId);
            if (cit == impl.constraintMap.end()) continue;
            JPH::Constraint* c = cit->second;

            // Target child-relative-to-parent orientation: the animation pose normally;
            // during a get-up it SWEEPS from the sprawled start pose toward the stand pose
            // over the heave, so the limbs travel smoothly instead of the stiff motors
            // springing straight to the final pose.
            glm::quat desiredRel = glm::normalize(glm::conjugate(animRot[pb]) * animRot[cb]);
            glm::quat restRel    = glm::normalize(glm::conjugate(bindRot[pb]) * bindRot[cb]);
            glm::quat target     = desiredRel;
            if (toStand) {
                float gt = glm::clamp((float)(inst.reaction.timer /
                               glm::max(inst.reaction.duration, 1e-4f)), 0.0f, 1.0f);
                float ge = gt * gt * (3.0f - 2.0f * gt);                  // smoothstep
                target = glm::slerp(glm::normalize(j.startRel), restRel, ge);
            }

            // Per-joint torque, jittered by the get-up wobble for the drunken flail.
            float jt = j.cc.motorMaxTorque * strength;
            // SIMBICON drives the hips with explicit virtual torques; a live position motor
            // on the same joint fights them, so fade it out as the blend comes in.
            if (rag && (cb == rag->locomotionHipBones[0] || cb == rag->locomotionHipBones[1]))
                jt *= 1.0f - glm::clamp(rag->locomotionSimbiconBlend, 0.0f, 1.0f);
            if (wobble > 0.0f) {
                float ph = (float)(j.constraintId % 29) * 0.7f;
                float nz = std::sin((float)impl.simTime * 9.0f + ph) *
                           std::sin((float)impl.simTime * 3.7f + ph * 1.7f);
                jt *= glm::max(0.0f, 1.0f + wobble * nz);
            }

            const std::string& boneName = skel.bones[cb].name;
            bool motorDisabled = false;
            if (rag) {
                for (int disabledBone : rag->locomotionDisabledMotorBones) {
                    if (disabledBone == cb) { motorDisabled = true; break; }
                }
            }
            const bool hasMotor = c->GetSubType() == JPH::EConstraintSubType::Hinge
                               || c->GetSubType() == JPH::EConstraintSubType::SwingTwist;
            const bool isLegMotor = hasMotor && (
                boneName == "leg_joint_L_1" || boneName == "leg_joint_R_1" ||
                boneName == "leg_joint_L_2" || boneName == "leg_joint_R_2" ||
                boneName == "leg_joint_L_3" || boneName == "leg_joint_R_3");
            float appliedMotorTorque = 0.0f;
            float limitTwistTorque = 0.0f;
            float limitSwingYTorque = 0.0f;
            float limitSwingZTorque = 0.0f;
            if (c->GetSubType() == JPH::EConstraintSubType::Hinge) {
                auto* hinge = static_cast<JPH::HingeConstraint*>(c);
                const float impulse = hinge->GetTotalLambdaMotor();
                appliedMotorTorque = std::abs(impulse) / FIXED_DT;
                limitTwistTorque =
                    hinge->GetTotalLambdaRotationLimits() / FIXED_DT;
            } else if (c->GetSubType() == JPH::EConstraintSubType::SwingTwist) {
                auto* swingTwist =
                    static_cast<JPH::SwingTwistConstraint*>(c);
                const JPH::Vec3 impulse = swingTwist->GetTotalLambdaMotor();
                const float maxImpulse = glm::max(std::abs(impulse.GetX()),
                    glm::max(std::abs(impulse.GetY()), std::abs(impulse.GetZ())));
                appliedMotorTorque = maxImpulse / FIXED_DT;
                limitTwistTorque =
                    swingTwist->GetTotalLambdaTwist() / FIXED_DT;
                limitSwingYTorque =
                    swingTwist->GetTotalLambdaSwingY() / FIXED_DT;
                limitSwingZTorque =
                    swingTwist->GetTotalLambdaSwingZ() / FIXED_DT;
            }
            const float motorSaturationRatio = jt > 1e-4f
                ? appliedMotorTorque / jt : 0.0f;
            const bool motorSaturated = !motorDisabled && jt > 1e-4f
                && motorSaturationRatio >= 0.98f;
            if (rag && isLegMotor) {
                int motorSlot = -1;
                for (int i = 0; i < 6; ++i) {
                    if (rag->_locomotionMotorBones[i] == cb) {
                        motorSlot = i;
                        break;
                    }
                    if (motorSlot < 0 && rag->_locomotionMotorBones[i] < 0)
                        motorSlot = i;
                }
                if (motorSlot >= 0) {
                    rag->_locomotionMotorBones[motorSlot] = cb;
                    rag->_locomotionMotorAppliedTorque[motorSlot] = appliedMotorTorque;
                    rag->_locomotionMotorTorqueLimit[motorSlot] = jt;
                    rag->_locomotionMotorSaturationRatio[motorSlot] = motorSaturationRatio;
                    rag->_locomotionMotorSaturated[motorSlot] = motorSaturated;
                }
            }
            if (logLegMotors && isLegMotor) {
                const glm::quat parentRot = FromJolt(bi.GetRotation(inst.bodies[j.parentSlot].id));
                const glm::quat childRot = FromJolt(bi.GetRotation(inst.bodies[j.childSlot].id));
                const glm::vec3 parentAngularVelocity = FromJolt(
                    bi.GetAngularVelocity(inst.bodies[j.parentSlot].id));
                const glm::vec3 childAngularVelocity = FromJolt(
                    bi.GetAngularVelocity(inst.bodies[j.childSlot].id));
                const float relativeAngularSpeed = glm::length(
                    childAngularVelocity - parentAngularVelocity);
                const glm::quat actualRel = glm::normalize(glm::conjugate(parentRot) * childRot);
                const glm::quat commandDelta = glm::normalize(glm::conjugate(restRel) * target);
                const glm::quat actualDelta = glm::normalize(glm::conjugate(restRel) * actualRel);
                const glm::quat errorDelta = glm::normalize(glm::conjugate(actualRel) * target);
                spdlog::info(
                    "[LocoMotor] {} type={} muted={} command={:.1f} actual={:.1f} error={:.1f} commandTwist={:+.1f} actualTwist={:+.1f} relativeRate={:.3f}radps applied={:.1f}/{:.0f} ratio={:.2f} sat={} limitTorque=(twist={:+.1f},swingY={:+.1f},swingZ={:+.1f})Nm limits=({:.0f},{:.0f},{:.0f},{:.0f})",
                    boneName,
                    c->GetSubType() == JPH::EConstraintSubType::Hinge ? "hinge" : "swing",
                    motorDisabled ? "YES" : "no",
                    QuatAngleDeg(commandDelta), QuatAngleDeg(actualDelta), QuatAngleDeg(errorDelta),
                    SignedTwistDeg(commandDelta, j.twistAxisLocal),
                    SignedTwistDeg(actualDelta, j.twistAxisLocal),
                    relativeAngularSpeed,
                    appliedMotorTorque, jt, motorSaturationRatio,
                    motorSaturated ? "YES" : "no",
                    limitTwistTorque,
                    limitSwingYTorque,
                    limitSwingZTorque,
                    j.cc.swingNormalDeg, j.cc.swingPlaneDeg,
                    j.cc.twistMinDeg, j.cc.twistMaxDeg);
                loggedLegMotor = true;
            }

            switch (c->GetSubType()) {
                case JPH::EConstraintSubType::SwingTwist: {
                    auto* st = static_cast<JPH::SwingTwistConstraint*>(c);
                    const JPH::EMotorState state = motorDisabled
                        ? JPH::EMotorState::Off : JPH::EMotorState::Position;
                    st->SetSwingMotorState(state);
                    st->SetTwistMotorState(state);
                    st->GetSwingMotorSettings().SetTorqueLimit(jt);
                    st->GetTwistMotorSettings().SetTorqueLimit(jt);
                    st->SetTargetOrientationBS(ToJolt(target));
                    break;
                }
                case JPH::EConstraintSubType::Hinge: {
                    auto* hinge = static_cast<JPH::HingeConstraint*>(c);
                    hinge->SetMotorState(motorDisabled
                        ? JPH::EMotorState::Off : JPH::EMotorState::Position);
                    // Drive the rotation FROM the hinge's zero TO the target, about the
                    // hinge axis. The zero is the relative orientation the joint was built
                    // at: the bind pose normally, but the sprawled pose after a Limp
                    // rebuild — so a get-up reads it from buildRel rather than assuming bind.
                    glm::quat zeroRel = toStand ? glm::normalize(j.buildRel) : restRel;
                    glm::quat delta   = glm::normalize(glm::conjugate(zeroRel) * target);
                    // delta lives in the CHILD body's local frame (target = zeroRel * delta),
                    // so extract the twist about the child-local hinge axis. (Projecting onto
                    // zeroRel*axis — the parent-frame axis — only coincides when zeroRel ≈
                    // identity, true for knees but not elbows: arm hinges read ~0° commanded
                    // while the pose asked 30-117°.) The twist angle about the shared hinge
                    // axis is the same scalar in either frame, matching Jolt's convention.
                    if (delta.w < 0.0f) delta = -delta;   // shortest arc: keep angle in (-180°, 180°]
                    glm::vec3 axisC = glm::normalize(SafeAxis(j.twistAxisLocal, j.twistAxisLocal));
                    float angle = 2.0f * std::atan2(glm::dot(glm::vec3(delta.x, delta.y, delta.z), axisC), delta.w);
                    hinge->GetMotorSettings().SetTorqueLimit(jt);
                    hinge->SetTargetAngle(angle);
                    break;
                }
                default: break;
            }
        }

        if (loggedLegMotor) impl.locoMotorDebugNext = impl.simTime + 0.25;

        // Get-up: the joint motors alone can't lift the root, so directly assist the hips
        // (upright + lift). Runs while heaving and while holding the stand.
        if (inst.reaction.kind == RagReaction::Kind::GetUp)
            DriveRagdollGetUpRoot(impl, inst, strength,
                                  (rag && rag->config) ? rag->config->getupBalance : 1.0f);

        // Locomotion controller root drive (no-ops while a reaction owns the root).
        if (rag) DriveRagdollLocomotionRoot(impl, inst, *rag);

        // Keep the dynamic bodies awake so the motors keep correcting toward the pose.
        for (const RagdollBody& b : inst.bodies)
            bi.ActivateBody(b.id);
    }
}

// ---------------------------------------------------------------------------
// PhysicsSystem — OnStart / OnUpdate / OnDestroy
// ---------------------------------------------------------------------------
void PhysicsSystem::OnStart(Scene& scene) {
    JPH::RegisterDefaultAllocator();

    JPH::Factory::sInstance = new JPH::Factory();
    JPH::RegisterTypes();

    m_impl = std::make_unique<Impl>();
    m_impl->tempAllocator = std::make_unique<JPH::TempAllocatorImpl>(32 * 1024 * 1024);
    int numThreads = std::max(1, static_cast<int>(std::thread::hardware_concurrency()) - 1);
    m_impl->jobSystem = std::make_unique<JPH::JobSystemThreadPool>(
        JPH::cMaxPhysicsJobs, JPH::cMaxPhysicsBarriers, numThreads);

    m_impl->joltSystem = std::make_unique<JPH::PhysicsSystem>();
    m_impl->joltSystem->Init(
        cMaxBodies, cNumBodyMutexes, cMaxBodyPairs, cMaxContactConstraints,
        m_impl->bpLayerInterface, m_impl->objVsBPFilter, m_impl->objLayerFilter
    );

    m_impl->contactListener = std::make_unique<ContactListener>(m_impl->bodyMap);
    m_impl->joltSystem->SetContactListener(m_impl->contactListener.get());

    m_impl->groupFilter = new GroupExcludeFilter();

    JPH::BodyInterface& bi = m_impl->joltSystem->GetBodyInterface();
    const JPH::GroupFilter* gf = m_impl->groupFilter;

    // Entities with RigidBodyComponent + ColliderComponent → full physics body
    for (auto [entity, rb, col, xform] : scene.View<RigidBodyComponent, ColliderComponent, TransformComponent>().each()) {
        InheritMeshPath(col, entity, scene.GetRegistry());
        CreateBody(bi, m_impl->bodyMap, entity, rb, col, xform, gf);
    }

    // Entities with only ColliderComponent → static collision geometry
    for (auto [entity, col, xform] : scene.View<ColliderComponent, TransformComponent>().each()) {
        if (scene.Has<RigidBodyComponent>(entity)) continue;
        InheritMeshPath(col, entity, scene.GetRegistry());
        CreateStaticCollider(bi, m_impl->bodyMap, entity, col, xform, gf);
    }

    // Queue pre-existing constraints (from the loaded scene). Their bodies were
    // just created above, so the first OnUpdate drain will build them. on_construct
    // doesn't fire for components that existed before we connect the signal below.
    for (auto [entity, cc] : scene.View<ConstraintComponent>().each())
        m_impl->pendingConstraints.push_back(entity);

    // Build ragdolls — kinematic body chains that follow the animation until flipped
    // to Dynamic. Their bodies/joints register in bodyMap/constraintMap, so the
    // OnDestroy sweep tears them down with everything else.
    for (auto [entity, rag] : scene.View<RagdollComponent>().each())
        BuildRagdoll(*m_impl, scene, entity, rag);

    // Expose the active impl and connect EnTT signals for runtime body lifecycle.
    // OnBodyComponentAdded defers creation; OnRb/ColDestroyed tear down immediately.
    s_Impl = m_impl.get();
    auto& reg = scene.GetRegistry();
    reg.on_construct<RigidBodyComponent>().connect<&OnBodyComponentAdded>();
    reg.on_construct<ColliderComponent>().connect<&OnBodyComponentAdded>();
    reg.on_destroy<RigidBodyComponent>().connect<&OnRbDestroyed>();
    reg.on_destroy<ColliderComponent>().connect<&OnColDestroyed>();
    reg.on_construct<ConstraintComponent>().connect<&OnConstraintAdded>();
    reg.on_destroy<ConstraintComponent>().connect<&OnConstraintDestroyed>();
}

// ---------------------------------------------------------------------------
// CreateStaticCollider
// Creates a static Jolt body for entities that have a ColliderComponent but
// no RigidBodyComponent (pure collision geometry, static world meshes, etc.).
// The body ID is written into col._bodyId so OnDestroy can remove it.
// ---------------------------------------------------------------------------
static void CreateStaticCollider(JPH::BodyInterface& bi,
    std::unordered_map<uint32_t, BodyRecord>& bodyMap,
    entt::entity entity, ColliderComponent& col, const TransformComponent& xform,
    const JPH::GroupFilter* groupFilter)
{
    JPH::ShapeRefC shapeRef = CreateShape(col, xform.scale);
    if (!shapeRef) return;

    JPH::BodyCreationSettings settings(
        shapeRef,
        ToJolt(xform.position),
        ToJolt(xform.rotation),
        JPH::EMotionType::Static,
        PhysicsLayers::STATIC
    );
    settings.mIsSensor = col.isTrigger;
    settings.mUserData = (JPH::uint64)entt::to_integral(entity);

    if (col.material) {
        settings.mFriction    = col.material->dynamicFriction;
        settings.mRestitution = col.material->restitution;
    }

    if (col.collisionGroup != 0 && groupFilter) {
        settings.mCollisionGroup = JPH::CollisionGroup(
            groupFilter,
            (JPH::CollisionGroup::GroupID)col.collisionGroup,
            (JPH::CollisionGroup::SubGroupID)entt::to_integral(entity));
    }

    JPH::BodyID bodyID = bi.CreateAndAddBody(settings, JPH::EActivation::DontActivate);
    if (bodyID.IsInvalid()) return;

    col._bodyId          = bodyID.GetIndexAndSequenceNumber();
    bodyMap[col._bodyId] = { entity, col.isTrigger };
}

// ---------------------------------------------------------------------------
// Per-step helpers
// ---------------------------------------------------------------------------

// Before each step: push TransformComponent positions into kinematic bodies so
// Jolt knows where to move them. Dynamic and Static bodies ignore this.
static void SyncKinematicBodies(Scene& scene, JPH::BodyInterface& bi)
{
    for (auto [entity, rb, xform] : scene.View<RigidBodyComponent, TransformComponent>().each()) {
        if (rb.bodyType != BodyType::Kinematic) continue;
        if (rb._bodyId == 0xFFFFFFFFu) continue;
        bi.MoveKinematic(JPH::BodyID(rb._bodyId),
            ToJolt(xform.position), ToJolt(xform.rotation), FIXED_DT);
    }
}

// After each step: write Jolt's simulated positions back into TransformComponent.
// Only Dynamic bodies — kinematic bodies are driven by the transform, not physics.
static void SyncTransforms(Scene& scene, JPH::BodyInterface& bi)
{
    for (auto [entity, rb, xform] : scene.View<RigidBodyComponent, TransformComponent>().each()) {
        if (rb.bodyType != BodyType::Dynamic) continue;
        if (rb._bodyId == 0xFFFFFFFFu) continue;
        JPH::BodyID bodyID(rb._bodyId);
        xform.position = FromJolt(bi.GetPosition(bodyID));
        xform.rotation = FromJolt(bi.GetRotation(bodyID));
    }
}

// After each step: drain queued contact events and fire the matching std::function
// callbacks on both participants. Collision normal is stored pointing from B → A;
// for entity B's callback we negate so the normal always points away from the other body.
static void DispatchCallbacks(Scene& scene, std::vector<ContactEvent> events)
{
    for (const auto& ev : events) {
        auto fire = [&](entt::entity self, entt::entity other, bool flipNormal) {
            if (!scene.Has<ColliderComponent>(self)) return;
            auto& col = scene.Get<ColliderComponent>(self);
            glm::vec3 normal = flipNormal ? -ev.contactNormal : ev.contactNormal;
            CollisionContact contact { other, ev.contactPoint, normal };
            switch (ev.type) {
                case ContactEvent::Type::Enter:
                    if (col.onCollisionEnter) col.onCollisionEnter(contact); break;
                case ContactEvent::Type::Stay:
                    if (col.onCollisionStay)  col.onCollisionStay(contact);  break;
                case ContactEvent::Type::Exit:
                    if (col.onCollisionExit)  col.onCollisionExit(contact);  break;
                case ContactEvent::Type::TriggerEnter:
                    if (col.onTriggerEnter) col.onTriggerEnter(other); break;
                case ContactEvent::Type::TriggerStay:
                    if (col.onTriggerStay) col.onTriggerStay(other); break;
                case ContactEvent::Type::TriggerExit:
                    if (col.onTriggerExit)  col.onTriggerExit(other);  break;
            }
        };
        fire(ev.entityA, ev.entityB, false);
        fire(ev.entityB, ev.entityA, true);
    }
}

// ---- Procedural reactions (flinch + get-up) ---------------------------------
// Both are RagReactions: a timed motor-driven move layered on Powered mode. A FLINCH
// (sub-knockdown hit) keeps chasing the animation while strength recovers; a GET-UP
// (post-knockdown recovery) chases the bind/stand pose with opened cones so sprawled
// limbs can straighten. TickRagdollReactions advances both; SyncRagdollPowered reads
// the strength + target each substep.

// Open every joint's limits wide (open=true) so the get-up motors can haul sprawled
// limbs back toward the stand pose — the Limp rebuild centered each cone on the sprawl,
// which would otherwise pin the limbs. open=false restores the authored limits from the
// joint's cc. Runtime setters only; no constraint rebuild.
static void WidenRagdollJoints(PhysicsSystem::Impl& impl, RagdollInstance& inst, bool open)
{
    for (RagdollJoint& j : inst.joints) {
        auto it = impl.constraintMap.find(j.constraintId);
        if (it == impl.constraintMap.end()) continue;
        JPH::Constraint* c = it->second;
        switch (c->GetSubType()) {
            case JPH::EConstraintSubType::SwingTwist: {
                auto* st = static_cast<JPH::SwingTwistConstraint*>(c);
                if (open) {
                    st->SetNormalHalfConeAngle(glm::radians(175.0f));
                    st->SetPlaneHalfConeAngle (glm::radians(175.0f));
                    st->SetTwistMinAngle(glm::radians(-175.0f));
                    st->SetTwistMaxAngle(glm::radians( 175.0f));
                } else {
                    st->SetNormalHalfConeAngle(glm::radians(glm::clamp(j.cc.swingNormalDeg, 0.0f, 180.0f)));
                    st->SetPlaneHalfConeAngle (glm::radians(glm::clamp(j.cc.swingPlaneDeg,  0.0f, 180.0f)));
                    st->SetTwistMinAngle(glm::radians(glm::clamp(j.cc.twistMinDeg, -180.0f, 0.0f)));
                    st->SetTwistMaxAngle(glm::radians(glm::clamp(j.cc.twistMaxDeg,    0.0f, 180.0f)));
                }
                break;
            }
            case JPH::EConstraintSubType::Hinge: {
                auto* h = static_cast<JPH::HingeConstraint*>(c);
                if (open) h->SetLimits(glm::radians(-179.0f), glm::radians(179.0f));
                else      h->SetLimits(glm::radians(glm::clamp(j.cc.limitMin, -180.0f, 0.0f)),
                                       glm::radians(glm::clamp(j.cc.limitMax,    0.0f, 180.0f)));
                break;
            }
            default: break;
        }
    }
}

// Cancel any active reaction and put the joints back to their authored limits (a get-up
// or standing hold leaves the cones open). Called on an explicit mode change or when a
// reaction is interrupted, so a later plain Powered/active rig isn't left floppy.
static void ClearRagdollReaction(PhysicsSystem::Impl& impl, RagdollInstance& inst)
{
    if (inst.reaction.kind == RagReaction::Kind::GetUp ||
        inst.reaction.kind == RagReaction::Kind::GetUpBlend) {
        WidenRagdollJoints(impl, inst, false);   // idempotent (blend already restored them)
        // The get-up drove the hips kinematically; hand the root back to the motion type the
        // rest of the rig is in (match a sibling body) so it rejoins the dynamic simulation.
        if (inst.rootBodySlot >= 0 && inst.bodies.size() > 1) {
            JPH::BodyInterface& bi = impl.joltSystem->GetBodyInterface();
            int sib = (inst.rootBodySlot == 0) ? 1 : 0;
            bi.SetMotionType(inst.bodies[inst.rootBodySlot].id,
                             bi.GetMotionType(inst.bodies[sib].id),
                             JPH::EActivation::Activate);
        }
    }
    // The root is Dynamic again either way (get-up/blend restored it above; a flinch's
    // ReleaseLocomotionRoot never re-kinematized it) -- keep the flag in sync so a
    // locomotion controller still commanding movement cleanly reclaims the root next frame.
    inst._locomotionDriving = false;
    inst.reaction  = RagReaction{};
    inst.restTimer = 0.0f;
}

// Begin (or refresh) a flinch: go Powered, snapshot the timing from the config, apply
// the initial strength dip. The caller adds the directional impulse to the struck bone.
static void StartRagdollFlinch(PhysicsSystem::Impl& impl, RagdollInstance& inst, RagdollComponent& rag)
{
    ReleaseLocomotionRoot(impl, inst);   // let the impulse move the whole body, hips included
    Physics::SetRagdollMode(rag, RagdollMode::Powered);   // kinematic -> dynamic, motors on
    RagReaction& r  = inst.reaction;
    r.kind          = RagReaction::Kind::Flinch;
    r.timer         = 0.0f;
    r.duration      = glm::max(rag.config ? rag.config->flinchRecovery : 0.4f, 0.01f);
    r.startStrength = glm::clamp(rag.config ? rag.config->flinchStrength : 0.3f, 0.0f, 1.0f);
    r.endStrength   = 1.0f;
    r.targetStand   = false;   // flinch keeps tracking the animation
    r.wobble        = 0.0f;
    rag.strength    = r.startStrength;
}

// Begin a procedural get-up: open the joint cones, go Powered, and ramp muscle strength
// 0 -> getupStrength while the motors chase the bind/stand pose (targetStand). Works from
// Limp (the usual case) or any mode. Deliberately under-powered + wobbly = sloppy.
static void StartRagdollGetUp(PhysicsSystem::Impl& impl, RagdollInstance& inst, RagdollComponent& rag)
{
    ReleaseLocomotionRoot(impl, inst);   // the heave takes the root over next, kinematically
    Physics::SetRagdollMode(rag, RagdollMode::Powered);   // clears any prior reaction first
    WidenRagdollJoints(impl, inst, true);                 // ...then open the cones for the heave

    // Snapshot the sprawled relative pose per joint so the motor target can sweep from
    // here toward the stand pose (smooth travel instead of a stiff spring to the target).
    JPH::BodyInterface& bi = impl.joltSystem->GetBodyInterface();
    for (RagdollJoint& j : inst.joints) {
        if (j.parentSlot < 0 || j.childSlot < 0) continue;
        glm::quat pr = FromJolt(bi.GetRotation(inst.bodies[j.parentSlot].id));
        glm::quat cr = FromJolt(bi.GetRotation(inst.bodies[j.childSlot].id));
        j.startRel = glm::normalize(glm::conjugate(pr) * cr);
    }

    // Fixed standing-hip height: ground under the sprawled rig (its lowest body right now,
    // resting on the floor) plus the bind-pose hip clearance. Captured once so the hips
    // servo to an ABSOLUTE goal and stop there, instead of chasing their own rising feet.
    float groundY = 1e30f;
    for (const RagdollBody& b : inst.bodies)
        groundY = glm::min(groundY, (float)bi.GetPosition(b.id).GetY());
    inst.getupTargetY = (groundY < 1e29f ? groundY : 0.0f) + inst.standHipClearance;

    // Snapshot the hips' transform and drive them KINEMATICALLY for the heave: a kinematic
    // root reaches the standing pose regardless of the dynamic chain hanging off it, then the
    // limb motors pose the body underneath. (Dynamic velocity/force on one body just gets
    // absorbed by the joints.) Restored to Dynamic in ClearRagdollReaction.
    if (inst.rootBodySlot >= 0) {
        const RagdollBody& root = inst.bodies[inst.rootBodySlot];
        inst.getupStartPos = FromJolt(bi.GetPosition(root.id));
        inst.getupStartRot = FromJolt(bi.GetRotation(root.id));
        bi.SetMotionType(root.id, JPH::EMotionType::Kinematic, JPH::EActivation::Activate);
    }

    RagReaction& r  = inst.reaction;
    r.kind          = RagReaction::Kind::GetUp;
    r.timer         = 0.0f;
    r.duration      = glm::max(rag.config ? rag.config->getupDuration : 1.2f, 0.05f);
    r.startStrength = 0.0f;
    r.endStrength   = glm::clamp(rag.config ? rag.config->getupStrength : 0.7f, 0.0f, 1.0f);
    r.targetStand   = true;
    r.wobble        = glm::max(rag.config ? rag.config->getupWobble : 0.0f, 0.0f);
    rag.strength    = 0.0f;
    inst.restTimer  = 0.0f;
    spdlog::info("Ragdoll {}: get-up start (heave {:.2f}s, peak strength {:.2f})",
                 rag._ragdollId, r.duration, r.endStrength);
}

// Begin the get-up -> animation cross-blend: the stand has been held, now ease every body
// from its current (held-stand) transform into the live animation pose, kinematically, then
// hand to Animated. All bodies go Kinematic and motors off (the blend fully drives them);
// the widened cones + kinematic root are restored on the way (ClearRagdollReaction handles
// the same cleanup when this finalizes). With getupBlend == 0 there is nothing to ease, so
// the caller switches straight to Animated instead of calling this.
static void StartRagdollGetUpBlend(PhysicsSystem::Impl& impl, RagdollInstance& inst, RagdollComponent& rag)
{
    JPH::BodyInterface& bi = impl.joltSystem->GetBodyInterface();
    WidenRagdollJoints(impl, inst, false);              // heave's over — restore authored cones
    SetRagdollMotorsEnabled(impl, inst, false);         // the kinematic blend drives the bodies
    for (RagdollBody& b : inst.bodies) {
        b.blendStartPos = FromJolt(bi.GetPosition(b.id));
        b.blendStartRot = FromJolt(bi.GetRotation(b.id));
        bi.SetMotionType(b.id, JPH::EMotionType::Kinematic, JPH::EActivation::Activate);
    }
    RagReaction& r = inst.reaction;
    r.kind     = RagReaction::Kind::GetUpBlend;
    r.timer    = 0.0f;
    r.duration = glm::max(rag.config ? rag.config->getupBlend : 0.25f, 1e-3f);
    inst.restTimer = 0.0f;
    spdlog::info("Ragdoll {}: get-up blending to animation ({:.2f}s)", rag._ragdollId, r.duration);
}

// True if every body of the rig has nearly stopped moving (settled on the floor).
static bool RagdollAtRest(JPH::BodyInterface& bi, const RagdollInstance& inst)
{
    for (const RagdollBody& b : inst.bodies)
        if (bi.GetLinearVelocity(b.id).Length()  > 0.25f ||
            bi.GetAngularVelocity(b.id).Length() > 1.5f) return false;
    return true;
}

// Per frame: (1) advance an active flinch/get-up reaction, ramping strength with a smooth
// ease and ending it when the window elapses; (2) for a settled Limp rig, auto-start a
// get-up once it has been at rest for cfg->getupDelay seconds. A reaction is abandoned if
// something else moved the rig out of Powered (a follow-up knockdown, a manual switch).
static void TickRagdollReactions(Scene& scene, PhysicsSystem::Impl& impl, float dt)
{
    if (impl.ragdolls.empty()) return;
    auto& reg = scene.GetRegistry();
    JPH::BodyInterface& bi = impl.joltSystem->GetBodyInterface();
    for (auto& [id, inst] : impl.ragdolls) {
        auto* rag = reg.valid(inst.entity) ? reg.try_get<RagdollComponent>(inst.entity) : nullptr;
        if (!rag) { inst.reaction.kind = RagReaction::Kind::None; continue; }

        // (1) advance an active reaction.
        if (inst.reaction.kind != RagReaction::Kind::None) {
            if (inst.mode != RagdollMode::Powered) {     // knocked down / mode changed under us
                ClearRagdollReaction(impl, inst);
                continue;
            }
            RagReaction& r = inst.reaction;

            // Get-up -> animation cross-blend (the tail of a get-up): the kinematic blend in
            // SyncRagdollGetUpBlend eases the bodies into the live pose; when it elapses we
            // finalize to Animated (already at the anim pose, so no pop).
            if (r.kind == RagReaction::Kind::GetUpBlend) {
                r.timer += dt;
                if (r.timer >= r.duration)
                    Physics::SetRagdollMode(*rag, RagdollMode::Animated);   // clears reaction
                continue;
            }

            // Get-up that has reached the top: hold the stand (kinematic root) for
            // getupHold seconds, then hand back to the animation clip. The hand-off goes to
            // Animated, NOT Powered: once the kinematic root is released the motors can't
            // balance, so a Powered handoff would just topple. A getupBlend window first
            // cross-blends the held pose into the live animation so it eases in instead of
            // popping. getupHold == 0 holds indefinitely (manual hand-off), the old behavior.
            if (r.kind == RagReaction::Kind::GetUp && r.timer >= r.duration) {
                rag->strength = r.endStrength;
                const float hold = rag->config ? rag->config->getupHold : 0.0f;
                if (hold > 0.0f) {
                    inst.restTimer += dt;   // idle during the hold — reused as the hold clock
                    if (inst.restTimer >= hold) {
                        const float blend = rag->config ? rag->config->getupBlend : 0.0f;
                        if (blend > 0.0f) {
                            StartRagdollGetUpBlend(impl, inst, *rag);
                        } else {
                            spdlog::info("Ragdoll {}: get-up hold elapsed — resuming animation", id);
                            Physics::SetRagdollMode(*rag, RagdollMode::Animated);  // clears reaction
                        }
                    }
                }
                continue;
            }

            r.timer += dt;
            const float t    = glm::clamp(r.timer / r.duration, 0.0f, 1.0f);
            const float ease = t * t * (3.0f - 2.0f * t);   // smoothstep
            rag->strength    = glm::mix(r.startStrength, r.endStrength, ease);

            // TEMP DIAGNOSTIC: sample the get-up heave ~10x/s. clearance = hips height
            // above the lowest body (≈ feet): a gradual rise climbs smoothly; a snap stays
            // flat then jumps in one step. Remove once get-up motion is dialed in.
            if (r.kind == RagReaction::Kind::GetUp && inst.rootBodySlot >= 0 &&
                (int)((r.timer - dt) * 10.0f) != (int)(r.timer * 10.0f)) {
                float hipsY = (float)bi.GetPosition(inst.bodies[inst.rootBodySlot].id).GetY();
                float minY  = hipsY;
                for (const RagdollBody& b : inst.bodies)
                    minY = glm::min(minY, (float)bi.GetPosition(b.id).GetY());
                spdlog::info("  get-up t={:.2f} str={:.2f} hipsY={:.3f} clearance={:.3f} target={:.3f}",
                             t, rag->strength, hipsY, hipsY - minY, inst.standHipClearance);
            }

            if (t >= 1.0f) {
                if (r.kind == RagReaction::Kind::Flinch) {
                    // Recovered footing: full strength, but STAY an active ragdoll holding
                    // the animation pose. No kinematic snap back to the walk frame — that
                    // teleport was the "snaps up" artifact. Gameplay/inspector hands off.
                    rag->strength = 1.0f;
                    ClearRagdollReaction(impl, inst);
                } else {
                    // Get-up just reached the top — hold from here (handled above next tick).
                    spdlog::info("Ragdoll {}: get-up reached the top — holding the stand", id);
                }
            }
            continue;
        }

        // (2) auto get-up: a settled limp rig heaves itself up after a dwell.
        if (inst.mode == RagdollMode::Limp && rag->config && rag->config->getupDelay > 0.0f) {
            if (RagdollAtRest(bi, inst)) {
                inst.restTimer += dt;
                if (inst.restTimer >= rag->config->getupDelay)
                    StartRagdollGetUp(impl, inst, *rag);
            } else {
                inst.restTimer = 0.0f;
            }
        } else {
            inst.restTimer = 0.0f;
        }
    }
}

// Convert Jolt's body-pair contacts into per-foot support contacts. This deliberately
// differs from the short foot ray used by the legacy balance base: a step transition needs
// proof that the sole actually left and then re-contacted a supporting surface, not merely
// that it remained close to one.
static void UpdateRagdollFootContacts(PhysicsSystem::Impl& impl, Scene& scene,
                                      const std::vector<ContactEvent>& events)
{
    auto& reg = scene.GetRegistry();
    for (auto& [rid, inst] : impl.ragdolls) {
        (void)rid;
        if (!reg.valid(inst.entity)) continue;
        auto* rag = reg.try_get<RagdollComponent>(inst.entity);
        if (!rag) continue;
        for (int i = 0; i < 2; ++i) {
            rag->_locomotionFootContact[i] = false;
            rag->_locomotionFootContactNormal[i] = glm::vec3(0.0f);
            rag->_locomotionFootContactPoint[i] = glm::vec3(0.0f);
            rag->_locomotionFootPenetration[i] = 0.0f;
            rag->_locomotionFootSoleMinY[i] = 0.0f;
        }

        // Measure the real live wrapper shape, not the foot-bone pivot. For an oriented
        // box, projecting its three half-extent axes onto world Y gives the exact AABB
        // lower face used to compare against the reported contact point.
        JPH::BodyInterface& bi = impl.joltSystem->GetBodyInterface();
        for (const RagdollBody& body : inst.bodies) {
            if (!body.isFoot) continue;
            const glm::vec3 bodyPos = FromJolt(bi.GetPosition(body.id));
            const glm::quat bodyRot = FromJolt(bi.GetRotation(body.id));
            const glm::vec3 center = bodyPos + bodyRot * body.localOffset;
            const glm::quat rotation = glm::normalize(bodyRot * body.localRotation);
            const float extentY =
                std::abs((rotation * glm::vec3(body.halfExtents.x, 0.0f, 0.0f)).y) +
                std::abs((rotation * glm::vec3(0.0f, body.halfExtents.y, 0.0f)).y) +
                std::abs((rotation * glm::vec3(0.0f, 0.0f, body.halfExtents.z)).y);
            for (int i = 0; i < 2; ++i) {
                if (rag->_locomotionFootBones[i] == body.boneIndex)
                    rag->_locomotionFootSoleMinY[i] = center.y - extentY;
            }
        }

        for (const ContactEvent& ev : events) {
            if (ev.type != ContactEvent::Type::Enter && ev.type != ContactEvent::Type::Stay)
                continue;
            if (ev.entityA == inst.entity && ev.entityB == inst.entity) continue;

            for (const RagdollBody& body : inst.bodies) {
                if (!body.isFoot) continue;
                const uint32_t id = body.id.GetIndexAndSequenceNumber();
                const bool isA = ev.bodyA == id;
                const bool isB = ev.bodyB == id;
                if (!isA && !isB) continue;

                // manifold normal points A -> B. The support normal acting on the foot is
                // opposite it when the foot is A, and along it when the foot is B.
                const glm::vec3 supportNormal = isA ? -ev.contactNormal : ev.contactNormal;
                if (supportNormal.y < 0.35f) continue;
                for (int i = 0; i < 2; ++i) {
                    if (rag->_locomotionFootBones[i] != body.boneIndex) continue;
                    if (!rag->_locomotionFootContact[i] ||
                        supportNormal.y > rag->_locomotionFootContactNormal[i].y) {
                        rag->_locomotionFootContact[i] = true;
                        rag->_locomotionFootContactNormal[i] = glm::normalize(supportNormal);
                        rag->_locomotionFootContactPoint[i] = ev.contactPoint;
                        rag->_locomotionFootPenetration[i] = ev.penetrationDepth;
                    }
                }
            }
        }
    }
}

// After each step: scan contacts for hits hard enough to auto-react a ragdoll. A
// ragdoll is one entity owning many bodies, so we match each contact endpoint's BodyID
// against the ragdoll's body list. Two tiers, both gated on the rig's config (0 = that
// tier off): a hit at/above impactThreshold KNOCKS DOWN (-> Limp, full flop); a lighter
// hit at/above flinchThreshold STAGGERS (-> Powered flinch, strength dip + recover). In
// both cases the struck bone gets an impulse along the contact normal so the reaction is
// directional — it spins from where it was hit. Manual Physics::SetRagdollMode still
// works regardless of thresholds.
static void AutoTriggerRagdollImpacts(PhysicsSystem::Impl& impl, Scene& scene,
                                      const std::vector<ContactEvent>& events)
{
    if (impl.ragdolls.empty()) return;
    auto& reg = scene.GetRegistry();
    JPH::BodyInterface& bi = impl.joltSystem->GetBodyInterface();

    for (const auto& ev : events) {
        if (ev.type != ContactEvent::Type::Enter) continue; // only the instant of impact
        if (ev.impactMagnitude <= 0.0f) continue;

        // The struck body may be either endpoint. The normal points A→B, so to push the
        // ragdoll along the hit it's +normal when the ragdoll is B, -normal when it's A.
        struct Side { uint32_t bodyId; float dirSign; };
        const Side sides[2] = { { ev.bodyA, -1.0f }, { ev.bodyB, +1.0f } };

        for (const Side& side : sides) {
            if (side.bodyId == 0xFFFFFFFFu) continue;
            for (auto& [rid, inst] : impl.ragdolls) {
                // Skip rigs that are already down; a flinch (Powered) can still be
                // refreshed or escalated to a knockdown by a follow-up hit.
                if (inst.mode == RagdollMode::Limp || !reg.valid(inst.entity)) continue;
                auto* rag = reg.try_get<RagdollComponent>(inst.entity);
                if (!rag || !rag->config) continue;

                int slot = -1;
                for (int s = 0; s < (int)inst.bodies.size(); ++s)
                    if (inst.bodies[s].id.GetIndexAndSequenceNumber() == side.bodyId) { slot = s; break; }
                if (slot < 0) continue;

                // Two tiers: knockdown (>= impactThreshold) wins over flinch
                // (>= flinchThreshold). 0 disables a tier.
                const float knock  = rag->config->impactThreshold;
                const float flinch = rag->config->flinchThreshold;
                const bool  doKnock  = knock  > 0.0f && ev.impactMagnitude >= knock;
                // A get-up still HEAVING must not be flinch-interrupted — its own limbs
                // slapping the floor read as impacts and would abandon the recovery. Only
                // a full knockdown cuts a get-up short. (Once it's standing/holding, hits
                // flinch normally again.)
                const bool  getupHeaving = inst.reaction.kind == RagReaction::Kind::GetUp &&
                                           inst.reaction.timer < inst.reaction.duration;
                const bool  doFlinch = !doKnock && !getupHeaving &&
                                       flinch > 0.0f && ev.impactMagnitude >= flinch;
                if (!doKnock && !doFlinch) continue;

                // Knockdown re-bakes joints at the live pose and flips Kinematic→Dynamic;
                // flinch goes Powered with a strength dip that recovers. Then kick the
                // struck bone directionally so the reaction spins from the hit.
                if (doKnock) Physics::SetRagdollMode(*rag, RagdollMode::Limp);
                else         StartRagdollFlinch(impl, inst, *rag);
                glm::vec3 impulse = ev.contactNormal * (side.dirSign * ev.impactMagnitude);
                bi.AddImpulse(inst.bodies[slot].id, ToJolt(impulse), ToJolt(ev.contactPoint));
                spdlog::info("Ragdoll {} auto-{}: impact {:.1f} (knock>={:.1f}, flinch>={:.1f}) bone slot {}",
                             rid, doKnock ? "limp" : "flinch", ev.impactMagnitude, knock, flinch, slot);
                break; // this endpoint handled
            }
        }
    }
}

// ---------------------------------------------------------------------------
// PhysicsSystem methods
// ---------------------------------------------------------------------------
void PhysicsSystem::OnUpdate(Scene& scene, float dt) {
    if (!m_impl) return;

    // Drain deferred creates — components added during the previous frame are now
    // fully initialised, so we can safely call CreateBody / CreateStaticCollider.
    if (!m_impl->pendingCreate.empty()) {
        auto& reg = scene.GetRegistry();
        JPH::BodyInterface& pbi = m_impl->joltSystem->GetBodyInterface();
        for (entt::entity entity : m_impl->pendingCreate) {
            if (!reg.valid(entity)) continue;
            bool hasRb  = reg.all_of<RigidBodyComponent>(entity);
            bool hasCol = reg.all_of<ColliderComponent>(entity);
            bool hasXf  = reg.all_of<TransformComponent>(entity);
            if (!hasCol || !hasXf) continue; // incomplete setup — will retry next add
            if (hasRb) {
                auto& rb    = reg.get<RigidBodyComponent>(entity);
                auto& col   = reg.get<ColliderComponent>(entity);
                auto& xform = reg.get<TransformComponent>(entity);
                if (rb._bodyId != 0xFFFFFFFFu) continue; // already created
                InheritMeshPath(col, entity, reg);
                CreateBody(pbi, m_impl->bodyMap, entity, rb, col, xform, m_impl->groupFilter);
            } else {
                auto& col   = reg.get<ColliderComponent>(entity);
                auto& xform = reg.get<TransformComponent>(entity);
                if (col._bodyId != 0xFFFFFFFFu) continue; // already created
                InheritMeshPath(col, entity, reg);
                CreateStaticCollider(pbi, m_impl->bodyMap, entity, col, xform, m_impl->groupFilter);
            }
        }
        m_impl->pendingCreate.clear();
    }

    // Drain deferred constraints. An entry waits (stays queued) until BOTH bodies
    // it needs are live: this entity's own body, and — for entity-to-entity
    // joints — the target entity's body (which may be created later in the frame
    // or a later frame). targetUuid 0 attaches to the static world (no wait).
    if (!m_impl->pendingConstraints.empty()) {
        auto& reg = scene.GetRegistry();
        std::vector<entt::entity> stillPending;
        for (entt::entity e : m_impl->pendingConstraints) {
            if (!reg.valid(e) || !reg.all_of<ConstraintComponent>(e)) continue;
            auto& cc = reg.get<ConstraintComponent>(e);
            if (cc._constraintId != 0xFFFFFFFFu) continue; // already built
            uint32_t bodyA = BodyIdOf(reg, e);
            if (bodyA == 0xFFFFFFFFu) { stillPending.push_back(e); continue; } // own body not ready

            uint32_t bodyB = 0xFFFFFFFFu; // world
            if (cc.targetUuid != 0) {
                entt::entity target = scene.FindByUuid(cc.targetUuid);
                if (!reg.valid(target)) { stillPending.push_back(e); continue; } // target entity not created yet
                bodyB = BodyIdOf(reg, target);
                if (bodyB == 0xFFFFFFFFu) { stillPending.push_back(e); continue; } // target body not ready
            }
            CreateConstraint(*m_impl, e, cc, bodyA, bodyB);
        }
        m_impl->pendingConstraints.swap(stillPending);
    }

    // Advance hit-react flinches once per frame (wall-clock dt) before the substeps,
    // so the ramped muscle strength is what SyncRagdollPowered reads this frame.
    TickRagdollReactions(scene, *m_impl, dt);

    // Clamp dt contribution — prevents death spiral when fps drops below 60.
    m_accumulator += std::min(dt, FIXED_DT);

    JPH::BodyInterface& bi = m_impl->joltSystem->GetBodyInterface();
    while (m_accumulator >= FIXED_DT) {
        m_impl->simTime += FIXED_DT;                 // physics clock for get-up wobble noise
        SyncKinematicBodies(scene, bi);
        SyncRagdollKinematic(scene, *m_impl, bi);   // Animated ragdolls follow the pose
        SyncRagdollGetUpBlend(scene, *m_impl, bi);  // get-up tail: ease bodies into the anim pose
        SyncRagdollPowered(scene, *m_impl);         // Powered ragdolls drive motors toward the pose
        m_impl->joltSystem->Update(FIXED_DT, 2, m_impl->tempAllocator.get(), m_impl->jobSystem.get());
        SyncTransforms(scene, bi);
        std::vector<ContactEvent> events = m_impl->contactListener->DrainEvents();
        UpdateRagdollFootContacts(*m_impl, scene, events);
        AutoTriggerRagdollImpacts(*m_impl, scene, events); // hard hits flip an Animated ragdoll to Limp
        DispatchCallbacks(scene, std::move(events));
        m_accumulator -= FIXED_DT;
    }
}

void PhysicsSystem::OnDestroy(Scene& scene) {
    if (!m_impl) return;

    // Nullify session pointer first so signal callbacks become no-ops immediately.
    s_Impl = nullptr;

    // Disconnect signals — prevents stale callbacks if registry outlives us.
    auto& reg = scene.GetRegistry();
    reg.on_construct<RigidBodyComponent>().disconnect<&OnBodyComponentAdded>();
    reg.on_construct<ColliderComponent>().disconnect<&OnBodyComponentAdded>();
    reg.on_destroy<RigidBodyComponent>().disconnect<&OnRbDestroyed>();
    reg.on_destroy<ColliderComponent>().disconnect<&OnColDestroyed>();
    reg.on_construct<ConstraintComponent>().disconnect<&OnConstraintAdded>();
    reg.on_destroy<ConstraintComponent>().disconnect<&OnConstraintDestroyed>();

    // Ragdoll bodies/joints live in bodyMap/constraintMap, so the sweeps below
    // destroy them; just drop the instance bookkeeping.
    m_impl->ragdolls.clear();

    // Constraints must be removed before the bodies they reference.
    for (auto& [id, constraint] : m_impl->constraintMap)
        m_impl->joltSystem->RemoveConstraint(constraint);
    m_impl->constraintMap.clear();
    m_impl->bodyToConstraints.clear();
    m_impl->pendingConstraints.clear();

    JPH::BodyInterface& bi = m_impl->joltSystem->GetBodyInterface();
    for (auto& [id, record] : m_impl->bodyMap) {
        JPH::BodyID bodyID(id);
        bi.RemoveBody(bodyID);
        bi.DestroyBody(bodyID);
    }
    m_impl->bodyMap.clear();
    m_impl->pendingCreate.clear();

    s_convexHullCache.clear();
    s_triangleMeshCache.clear();

    m_impl.reset();

    JPH::UnregisterTypes();
    delete JPH::Factory::sInstance;
    JPH::Factory::sInstance = nullptr;
}

// Constructor and destructor defined here so std::unique_ptr<Impl> only needs a complete
// type in this translation unit, not in any header consumer.
PhysicsSystem::PhysicsSystem() = default;
PhysicsSystem::~PhysicsSystem() = default;

// ---------------------------------------------------------------------------
// Physics:: — runtime manipulation API (implementations)
// ---------------------------------------------------------------------------
namespace Physics {

// Helpers local to this translation unit
static inline JPH::BodyInterface* BI() {
    return s_Impl ? &s_Impl->joltSystem->GetBodyInterface() : nullptr;
}
static inline JPH::BodyID BID(const RigidBodyComponent& rb) {
    return JPH::BodyID(rb._bodyId);
}
static inline bool Valid(const RigidBodyComponent& rb) {
    return rb._bodyId != 0xFFFFFFFFu;
}

void AddForce(const RigidBodyComponent& rb, glm::vec3 f) {
    if (auto* bi = BI(); bi && Valid(rb)) bi->AddForce(BID(rb), ToJolt(f));
}
void AddForceAtPoint(const RigidBodyComponent& rb, glm::vec3 f, glm::vec3 p) {
    if (auto* bi = BI(); bi && Valid(rb)) bi->AddForce(BID(rb), ToJolt(f), ToJolt(p));
}
void AddTorque(const RigidBodyComponent& rb, glm::vec3 t) {
    if (auto* bi = BI(); bi && Valid(rb)) bi->AddTorque(BID(rb), ToJolt(t));
}
void AddImpulse(const RigidBodyComponent& rb, glm::vec3 imp) {
    if (auto* bi = BI(); bi && Valid(rb)) bi->AddImpulse(BID(rb), ToJolt(imp));
}
void AddImpulseAtPoint(const RigidBodyComponent& rb, glm::vec3 imp, glm::vec3 p) {
    if (auto* bi = BI(); bi && Valid(rb)) bi->AddImpulse(BID(rb), ToJolt(imp), ToJolt(p));
}
void AddAngularImpulse(const RigidBodyComponent& rb, glm::vec3 imp) {
    if (auto* bi = BI(); bi && Valid(rb)) bi->AddAngularImpulse(BID(rb), ToJolt(imp));
}
void SetLinearVelocity(const RigidBodyComponent& rb, glm::vec3 v) {
    if (auto* bi = BI(); bi && Valid(rb)) bi->SetLinearVelocity(BID(rb), ToJolt(v));
}
glm::vec3 GetLinearVelocity(const RigidBodyComponent& rb) {
    auto* bi = BI();
    return (bi && Valid(rb)) ? FromJolt(bi->GetLinearVelocity(BID(rb))) : glm::vec3{};
}
void SetAngularVelocity(const RigidBodyComponent& rb, glm::vec3 v) {
    if (auto* bi = BI(); bi && Valid(rb)) bi->SetAngularVelocity(BID(rb), ToJolt(v));
}
glm::vec3 GetAngularVelocity(const RigidBodyComponent& rb) {
    auto* bi = BI();
    return (bi && Valid(rb)) ? FromJolt(bi->GetAngularVelocity(BID(rb))) : glm::vec3{};
}

void Activate(const RigidBodyComponent& rb) {
    if (auto* bi = BI(); bi && Valid(rb)) bi->ActivateBody(BID(rb));
}
void Deactivate(const RigidBodyComponent& rb) {
    if (auto* bi = BI(); bi && Valid(rb)) bi->DeactivateBody(BID(rb));
}

void SetGravity(glm::vec3 gravity) {
    if (s_Impl) s_Impl->joltSystem->SetGravity(ToJolt(gravity));
}

void SetMotorTarget(const ConstraintComponent& cc, float target) {
    if (!s_Impl || cc._constraintId == 0xFFFFFFFFu) return;
    auto it = s_Impl->constraintMap.find(cc._constraintId);
    if (it == s_Impl->constraintMap.end()) return;

    JPH::Constraint* c = it->second;
    if (c->GetSubType() != JPH::EConstraintSubType::Hinge) return; // hinge motors only for now
    auto* hinge = static_cast<JPH::HingeConstraint*>(c);

    switch (hinge->GetMotorState()) {
        case JPH::EMotorState::Position: hinge->SetTargetAngle(glm::radians(target)); break;            // deg -> rad
        case JPH::EMotorState::Velocity: hinge->SetTargetAngularVelocity(glm::radians(target)); break;  // deg/s -> rad/s
        default: return; // motor off — nothing to drive
    }

    // Wake the driven bodies so a settled joint responds to the new target.
    if (auto* bi = BI()) {
        if (JPH::Body* b1 = hinge->GetBody1(); b1 && !b1->IsStatic()) bi->ActivateBody(b1->GetID());
        if (JPH::Body* b2 = hinge->GetBody2(); b2 && !b2->IsStatic()) bi->ActivateBody(b2->GetID());
    }
}

void SetMotorTargetOrientation(const ConstraintComponent& cc, glm::quat targetBS) {
    if (!s_Impl || cc._constraintId == 0xFFFFFFFFu) return;
    auto it = s_Impl->constraintMap.find(cc._constraintId);
    if (it == s_Impl->constraintMap.end()) return;

    JPH::Constraint* c = it->second;
    if (c->GetSubType() != JPH::EConstraintSubType::SwingTwist) return; // swing-twist only
    auto* st = static_cast<JPH::SwingTwistConstraint*>(c);
    if (st->GetSwingMotorState() == JPH::EMotorState::Off &&
        st->GetTwistMotorState() == JPH::EMotorState::Off) return; // no motor to drive

    st->SetTargetOrientationBS(ToJolt(targetBS));

    // Wake the driven bodies so a settled joint responds to the new target.
    if (auto* bi = BI()) {
        if (JPH::Body* b1 = st->GetBody1(); b1 && !b1->IsStatic()) bi->ActivateBody(b1->GetID());
        if (JPH::Body* b2 = st->GetBody2(); b2 && !b2->IsStatic()) bi->ActivateBody(b2->GetID());
    }
}

// ---- Runtime constraints ----------------------------------------------------

// Imperative create/destroy that bypasses the ConstraintComponent path: it builds
// straight into constraintMap via the same BuildConstraint/RegisterConstraint the
// component drain uses, and hands back the table id as an opaque handle. For
// transient joints (grabs) that shouldn't be authored components.
ConstraintHandle CreateConstraint(const ConstraintComponent& desc,
                                  const RigidBodyComponent& self,
                                  const RigidBodyComponent& target) {
    if (!s_Impl || !Valid(self) || !Valid(target)) return kInvalidConstraint;
    ConstraintComponent cc = desc;          // local copy receives the assigned _constraintId
    ::CreateConstraint(*s_Impl, entt::null, cc, self._bodyId, target._bodyId);
    return cc._constraintId;                // stays sentinel if the Jolt build failed
}

ConstraintHandle CreateConstraintToWorld(const ConstraintComponent& desc,
                                         const RigidBodyComponent& self) {
    if (!s_Impl || !Valid(self)) return kInvalidConstraint;
    ConstraintComponent cc = desc;
    ::CreateConstraint(*s_Impl, entt::null, cc, self._bodyId, 0xFFFFFFFFu);
    return cc._constraintId;
}

void ReleaseConstraint(ConstraintHandle handle) {
    if (!s_Impl || handle == kInvalidConstraint) return;
    auto it = s_Impl->constraintMap.find(handle);
    if (it == s_Impl->constraintMap.end()) return; // already gone (e.g. a body was destroyed)
    s_Impl->joltSystem->RemoveConstraint(it->second);
    s_Impl->constraintMap.erase(it);
    // The handle may linger in bodyToConstraints, but that's harmless:
    // RemoveConstraintsTouching skips ids no longer in constraintMap, and ids never repeat.
}

// ---- Collision groups (runtime) ---------------------------------------------
// Two bodies sharing the same NON-ZERO group id (and the engine's GroupExcludeFilter)
// never collide. The grab system uses this to stop a held object from shoving its
// holder: it reads both bodies' groups, drops them into a shared group while held, then
// restores. cInvalidGroup (0xFFFFFFFF) means "ungrouped" (collides with everything).

uint32_t GetCollisionGroup(const RigidBodyComponent& rb) {
    if (!s_Impl || !Valid(rb)) return (uint32_t)JPH::CollisionGroup::cInvalidGroup;
    JPH::BodyLockRead lock(s_Impl->joltSystem->GetBodyLockInterface(), BID(rb));
    if (!lock.Succeeded()) return (uint32_t)JPH::CollisionGroup::cInvalidGroup;
    return (uint32_t)lock.GetBody().GetCollisionGroup().GetGroupID();
}

void SetCollisionGroup(const RigidBodyComponent& rb, uint32_t group) {
    if (!s_Impl || !Valid(rb)) return;
    JPH::BodyLockWrite lock(s_Impl->joltSystem->GetBodyLockInterface(), BID(rb));
    if (!lock.Succeeded()) return;
    // Attach the exclude filter; subgroup is ignored by GroupExcludeFilter so 0 is fine.
    lock.GetBody().SetCollisionGroup(JPH::CollisionGroup(
        s_Impl->groupFilter, (JPH::CollisionGroup::GroupID)group, (JPH::CollisionGroup::SubGroupID)0));
}

void ClearCollisionGroup(const RigidBodyComponent& rb) {
    if (!s_Impl || !Valid(rb)) return;
    JPH::BodyLockWrite lock(s_Impl->joltSystem->GetBodyLockInterface(), BID(rb));
    if (!lock.Succeeded()) return;
    // Default group = no filter, ungrouped: collides with everything again.
    lock.GetBody().SetCollisionGroup(JPH::CollisionGroup());
}

// ---- Ragdoll ----------------------------------------------------------------

// Flip a ragdoll between Animated (kinematic, follows the pose) and Limp (dynamic,
// flops). Velocity is inherited from the kinematic follow on the way to Limp
// (concern #1); on the way back the next follow snaps it to the animation.
void SetRagdollMode(RagdollComponent& rag, RagdollMode mode) {
    if (!s_Impl || rag._ragdollId == 0xFFFFFFFFu) return;
    auto it = s_Impl->ragdolls.find(rag._ragdollId);
    if (it == s_Impl->ragdolls.end()) return;
    RagdollInstance& inst = it->second;

    if (mode != RagdollMode::Powered)
        DisableLocomotionUprightConstraint(*s_Impl, inst);

    // An explicit mode set cancels any in-flight reaction (flinch / get-up / stand hold)
    // and restores the authored joint limits a get-up had opened. Reaction starters call
    // this first, then set their own reaction, so this never clobbers the new one.
    ClearRagdollReaction(*s_Impl, inst);

    if (inst.mode != mode) {
        JPH::BodyInterface& bi = s_Impl->joltSystem->GetBodyInterface();
        // Re-center the joint limits on the live pose before the bodies go Dynamic,
        // so no limb is born outside its cone and yanked back (the explosion). Only
        // Limp does this: a Powered rig must KEEP its bind-relative joint frames so
        // its motors can drive back toward the animated pose.
        if (mode == RagdollMode::Limp)
            RebuildRagdollJointsAtPose(*s_Impl, inst);
        JPH::EMotionType mt = (mode == RagdollMode::Animated)
            ? JPH::EMotionType::Kinematic : JPH::EMotionType::Dynamic;
        for (const RagdollBody& b : inst.bodies)
            bi.SetMotionType(b.id, mt, JPH::EActivation::Activate);
        // Motors drive only in Powered mode; Animated (kinematic) and Limp run passive.
        SetRagdollMotorsEnabled(*s_Impl, inst, mode == RagdollMode::Powered);
        inst.mode = mode;
        // This block just force-set every body's motion type (including root), so any
        // locomotion kinematic hold is moot -- keep the flag in sync so a later resume
        // of locomotion re-establishes it cleanly instead of thinking it's still held.
        inst._locomotionDriving = false;
    }
    rag.mode = mode;
}

// Muscle strength for an active (Powered) ragdoll. Stored on the component and read
// by SyncRagdollPowered each substep to scale motor torque; takes effect next step.
void SetRagdollStrength(RagdollComponent& rag, float strength) {
    rag.strength = glm::clamp(strength, 0.0f, 4.0f);
}

// Bind-pose standing hip clearance -- 0 until the ragdoll is built.
float GetRagdollStandHipClearance(const RagdollComponent& rag) {
    if (!s_Impl || rag._ragdollId == 0xFFFFFFFFu) return 0.0f;
    auto it = s_Impl->ragdolls.find(rag._ragdollId);
    return it == s_Impl->ragdolls.end() ? 0.0f : it->second.standHipClearance;
}

// Bone index of the root (hips) body -- -1 until the ragdoll is built.
int GetRagdollRootBone(const RagdollComponent& rag) {
    if (!s_Impl || rag._ragdollId == 0xFFFFFFFFu) return -1;
    auto it = s_Impl->ragdolls.find(rag._ragdollId);
    if (it == s_Impl->ragdolls.end() || it->second.rootBodySlot < 0) return -1;
    return it->second.bodies[it->second.rootBodySlot].boneIndex;
}

// Current tilt of the root/hips, degrees, 0 = upright -- same convention
// DriveRagdollLocomotionRoot uses internally (composed against rootBindRot, which is
// NOT world-Y for every rig; e.g. CesiumMan's hips are Z-up in bind). Exposed so a
// script-side fallen-tilt gate reads the SAME value the engine's own horizontal/
// upright systems gate on, rather than re-deriving tilt with a raw world-Y comparison
// that reads ~90 deg constantly on a Z-up-bind rig even while genuinely upright (the
// 2026-07-21 "fights walking" regression -- a spurious fallen-tilt gate blocking
// stepping/IK almost all the time). Returns 0 until the ragdoll is built.
float GetRagdollTiltDeg(const RagdollComponent& rag) {
    if (!s_Impl || rag._ragdollId == 0xFFFFFFFFu) return 0.0f;
    auto it = s_Impl->ragdolls.find(rag._ragdollId);
    if (it == s_Impl->ragdolls.end() || it->second.rootBodySlot < 0) return 0.0f;
    const RagdollInstance& inst = it->second;
    JPH::BodyInterface& bi = s_Impl->joltSystem->GetBodyInterface();
    const glm::quat curRot = FromJolt(bi.GetRotation(inst.bodies[inst.rootBodySlot].id));
    const glm::vec3 upNow  = curRot * (glm::conjugate(inst.rootBindRot) * glm::vec3(0, 1, 0));
    return glm::degrees(std::acos(glm::clamp(upNow.y, -1.0f, 1.0f)));
}

// Instant impulse on the ragdoll body mapped to a skeleton bone. See PhysicsAPI.h —
// the launch primitive for gameplay moves (punches) the position motors are too
// slow to generate momentum for.
bool AddRagdollBoneImpulse(const RagdollComponent& rag, int boneIndex, glm::vec3 impulse) {
    if (!s_Impl || rag._ragdollId == 0xFFFFFFFFu || boneIndex < 0) return false;
    auto it = s_Impl->ragdolls.find(rag._ragdollId);
    if (it == s_Impl->ragdolls.end()) return false;
    for (const RagdollBody& b : it->second.bodies) {
        if (b.boneIndex != boneIndex) continue;
        s_Impl->joltSystem->GetBodyInterface().AddImpulse(b.id, ToJolt(impulse));
        return true;
    }
    return false;
}

// Per-step world torque on a bone's body -- the actuator for SIMBICON's virtual PD
// controllers, which command torque rather than a joint-motor target.
bool AddRagdollBoneTorque(const RagdollComponent& rag, int boneIndex, glm::vec3 torque) {
    if (!s_Impl || rag._ragdollId == 0xFFFFFFFFu || boneIndex < 0) return false;
    auto it = s_Impl->ragdolls.find(rag._ragdollId);
    if (it == s_Impl->ragdolls.end()) return false;
    for (const RagdollBody& b : it->second.bodies) {
        if (b.boneIndex != boneIndex) continue;
        s_Impl->joltSystem->GetBodyInterface().AddTorque(b.id, ToJolt(torque));
        return true;
    }
    return false;
}

static const RagdollBody* FindRagdollBone(const RagdollComponent& rag, int boneIndex) {
    if (!s_Impl || rag._ragdollId == 0xFFFFFFFFu || boneIndex < 0) return nullptr;
    auto it = s_Impl->ragdolls.find(rag._ragdollId);
    if (it == s_Impl->ragdolls.end()) return nullptr;
    for (const RagdollBody& b : it->second.bodies)
        if (b.boneIndex == boneIndex) return &b;
    return nullptr;
}

glm::quat GetRagdollBoneRotation(const RagdollComponent& rag, int boneIndex, bool* ok) {
    const RagdollBody* b = FindRagdollBone(rag, boneIndex);
    if (ok) *ok = b != nullptr;
    if (!b) return glm::quat(1, 0, 0, 0);
    return FromJolt(s_Impl->joltSystem->GetBodyInterface().GetRotation(b->id));
}

glm::vec3 GetRagdollBoneAngularVelocity(const RagdollComponent& rag, int boneIndex, bool* ok) {
    const RagdollBody* b = FindRagdollBone(rag, boneIndex);
    if (ok) *ok = b != nullptr;
    if (!b) return glm::vec3(0.0f);
    return FromJolt(s_Impl->joltSystem->GetBodyInterface().GetAngularVelocity(b->id));
}

glm::vec3 GetRagdollBonePosition(const RagdollComponent& rag, int boneIndex, bool* ok) {
    const RagdollBody* b = FindRagdollBone(rag, boneIndex);
    if (ok) *ok = b != nullptr;
    if (!b) return glm::vec3(0.0f);
    return FromJolt(s_Impl->joltSystem->GetBodyInterface().GetPosition(b->id));
}

glm::vec3 GetRagdollBoneLinearVelocity(const RagdollComponent& rag, int boneIndex, bool* ok) {
    const RagdollBody* b = FindRagdollBone(rag, boneIndex);
    if (ok) *ok = b != nullptr;
    if (!b) return glm::vec3(0.0f);
    return FromJolt(s_Impl->joltSystem->GetBodyInterface().GetLinearVelocity(b->id));
}

bool RotateRagdollYaw(RagdollComponent& rag, glm::vec3 worldPivot, float yawRadians) {
    if (!s_Impl || rag._ragdollId == 0xFFFFFFFFu
        || !std::isfinite(yawRadians)
        || !std::isfinite(worldPivot.x)
        || !std::isfinite(worldPivot.y)
        || !std::isfinite(worldPivot.z)) return false;
    auto it = s_Impl->ragdolls.find(rag._ragdollId);
    if (it == s_Impl->ragdolls.end() || it->second.bodies.empty()) return false;

    RagdollInstance& inst = it->second;
    JPH::BodyInterface& bi = s_Impl->joltSystem->GetBodyInterface();
    const glm::quat turn = glm::angleAxis(
        yawRadians, glm::vec3(0.0f, 1.0f, 0.0f));
    struct BodySnapshot {
        JPH::BodyID id;
        glm::vec3 position;
        glm::quat rotation;
        glm::vec3 linearVelocity;
        glm::vec3 angularVelocity;
    };
    std::vector<BodySnapshot> snapshots;
    snapshots.reserve(inst.bodies.size());
    for (const RagdollBody& body : inst.bodies) {
        snapshots.push_back({
            body.id,
            FromJolt(bi.GetPosition(body.id)),
            FromJolt(bi.GetRotation(body.id)),
            FromJolt(bi.GetLinearVelocity(body.id)),
            FromJolt(bi.GetAngularVelocity(body.id))
        });
    }

    // The world-backed upright constraint owns a world frame. Recreate it after the
    // teleport so it cannot pull the root back toward its pre-snap frame.
    DisableLocomotionUprightConstraint(*s_Impl, inst);
    for (const BodySnapshot& body : snapshots) {
        const glm::vec3 position = worldPivot
            + turn * (body.position - worldPivot);
        const glm::quat rotation = glm::normalize(turn * body.rotation);
        bi.SetPositionAndRotation(
            body.id,
            JPH::RVec3(position.x, position.y, position.z),
            ToJolt(rotation), JPH::EActivation::Activate);
        bi.SetLinearVelocity(body.id, ToJolt(turn * body.linearVelocity));
        bi.SetAngularVelocity(body.id, ToJolt(turn * body.angularVelocity));
        bi.InvalidateContactCache(body.id);
    }

    auto rotatePoint = [&](glm::vec3 point) {
        return worldPivot + turn * (point - worldPivot);
    };
    rag.locomotionTargetPos = rotatePoint(rag.locomotionTargetPos);
    rag.locomotionSupportTarget = rotatePoint(rag.locomotionSupportTarget);
    rag.locomotionSupportTargetVel = turn * rag.locomotionSupportTargetVel;
    rag._locomotionRootVel = turn * rag._locomotionRootVel;
    rag._locomotionRootAngularVelocity = turn
        * rag._locomotionRootAngularVelocity;
    if (rag._locomotionCOMValid)
        rag._locomotionCOM = rotatePoint(rag._locomotionCOM);
    rag._locomotionCOMVel = turn * rag._locomotionCOMVel;
    for (int i = 0; i < 2; ++i) {
        rag._locomotionFootContactPoint[i] = rotatePoint(
            rag._locomotionFootContactPoint[i]);
        rag._locomotionFootContactNormal[i] = turn
            * rag._locomotionFootContactNormal[i];
    }
    return true;
}

// Kick off a procedural get-up (the auto-on-settle path uses the same StartRagdollGetUp).
void RagdollGetUp(RagdollComponent& rag) {
    if (!s_Impl || rag._ragdollId == 0xFFFFFFFFu) return;
    auto it = s_Impl->ragdolls.find(rag._ragdollId);
    if (it == s_Impl->ragdolls.end()) return;
    StartRagdollGetUp(*s_Impl, it->second, rag);
}

// After the physics step AND after the animation palette is built: for each Limp
// ragdoll, overwrite the skinning palette from the simulated bodies and re-root
// the entity transform to the hips (concern #2). Mapped bones read their body's
// world transform; unmapped bones ride their nearest physics-driven ancestor by
// keeping their freshly-animated local offset. Call once per frame from the main
// loop, after UpdateAnimators.
void SyncRagdollPoses(Scene& scene) {
    if (!s_Impl || s_Impl->ragdolls.empty()) return;
    DIAMOND_PROFILE_SCOPE("Ragdoll Sync");
    auto& reg = scene.GetRegistry();
    JPH::BodyInterface& bi = s_Impl->joltSystem->GetBodyInterface();

    for (auto& [id, inst] : s_Impl->ragdolls) {
        // Both Limp and Powered are dynamic — skin them from the simulated bodies.
        // Only Animated keeps the animation palette as-is.
        if (inst.mode == RagdollMode::Animated || !reg.valid(inst.entity)) continue;
        auto* smc  = reg.try_get<SkinnedMeshComponent>(inst.entity);
        auto* anim = reg.try_get<AnimatorComponent>(inst.entity);
        if (!smc || !anim) continue;
        const Diamond::Skeleton& skel = smc->skeleton;
        const int n = (int)skel.bones.size();
        if ((int)anim->palette.size() != n) continue;

        // Decompose the entity world matrix into translation, rotation and uniform
        // scale so model space can be reconstructed without the spurious 1/scale a
        // naive inverse() would bake into the (scale-free) rigid body transforms.
        glm::mat4 ew = scene.GetTransformSystem().GetWorldMatrix(inst.entity);
        glm::vec3 te(ew[3]);
        glm::vec3 c0(ew[0]), c1(ew[1]), c2(ew[2]);
        float lx = glm::length(c0), ly = glm::length(c1), lz = glm::length(c2);
        float s  = (lx + ly + lz) / 3.0f; if (s < 1e-6f) s = 1.0f;
        glm::quat ReConj = glm::conjugate(glm::quat_cast(glm::mat3(c0 / lx, c1 / ly, c2 / lz)));

        std::vector<int> boneToBody(n, -1);
        for (int k = 0; k < (int)inst.bodies.size(); ++k) {
            int bone = inst.bodies[k].boneIndex;
            if (bone >= 0 && bone < n) boneToBody[bone] = k;
        }

        // Animated model-space world of every bone, from the palette just built.
        std::vector<glm::mat4> worldAnim(n), world(n);
        for (int i = 0; i < n; ++i)
            worldAnim[i] = anim->palette[i] * glm::inverse(skel.bones[i].inverseBind);

        for (int i = 0; i < n; ++i) {
            int parent = skel.bones[i].parent;
            if (boneToBody[i] >= 0) {
                const RagdollBody& b = inst.bodies[boneToBody[i]];
                glm::vec3 pm = (ReConj * (FromJolt(bi.GetPosition(b.id)) - te)) / s;
                glm::quat rm =  ReConj *  FromJolt(bi.GetRotation(b.id));
                world[i] = glm::translate(glm::mat4(1.0f), pm) * glm::mat4_cast(rm);
            } else if (parent < 0) {
                world[i] = worldAnim[i];
            } else {
                world[i] = world[parent] * (glm::inverse(worldAnim[parent]) * worldAnim[i]);
            }
            anim->palette[i] = world[i] * skel.bones[i].inverseBind;
        }

        // Re-root the entity origin to the hips for bounds/gameplay (skip if parented,
        // where TransformComponent is parent-relative). The skin is unaffected — the
        // entity matrix cancels in the readback above.
        if (inst.rootBodySlot >= 0 && reg.all_of<TransformComponent>(inst.entity)) {
            bool parented = reg.all_of<HierarchyComponent>(inst.entity) &&
                            reg.get<HierarchyComponent>(inst.entity).parent != entt::null;
            if (!parented)
                reg.get<TransformComponent>(inst.entity).position =
                    FromJolt(bi.GetPosition(inst.bodies[inst.rootBodySlot].id));
        }
    }
}

// Debug wireframes for every ragdoll body at its live simulated transform. Unlike
// DrawColliders these bodies aren't ColliderComponents, so they need their own pass.
void DrawRagdolls(Scene& scene, glm::vec3 color) {
    if (!s_Impl) return;
    JPH::BodyInterface& bi = s_Impl->joltSystem->GetBodyInterface();
    for (auto& [id, inst] : s_Impl->ragdolls) {
        for (const RagdollBody& b : inst.bodies) {
            glm::vec3 wp = FromJolt(bi.GetPosition(b.id));
            glm::quat wr = FromJolt(bi.GetRotation(b.id));
            glm::vec3 cpos = wp + wr * b.localOffset;
            glm::quat crot = wr * b.localRotation;
            switch (b.shape) {
                case RagdollBodyDef::Shape::Box:    DebugDraw::Box(cpos, crot, b.halfExtents, color); break;
                case RagdollBodyDef::Shape::Sphere: DebugDraw::Sphere(cpos, b.radius, color);         break;
                default:                            DebugDraw::Capsule(cpos, crot, b.halfHeight, b.radius, color); break;
            }
        }
    }
}

class SingleBodyIgnoreFilter final : public JPH::BodyFilter {
public:
    explicit SingleBodyIgnoreFilter(JPH::BodyID id) : m_id(id) {}
    bool ShouldCollide(const JPH::BodyID& id)  const override { return id != m_id; }
    bool ShouldCollideLocked(const JPH::Body&) const override { return true; }
private:
    JPH::BodyID m_id;
};

// Ignores EVERY body owned by one entity (matched via Body::GetUserData, which every
// body sets to its entity). A ragdoll is one entity with many bodies, so ignoring a
// single BodyID isn't enough — a downward foot raycast would still hit the character's
// other limb capsules. The UserData lives on the Body, so the filtering happens in the
// locked callback; the broadphase pass passes everything through.
//
// Also rejects SENSOR bodies (isTrigger colliders): casts probe for solid geometry
// (camera collision, ground rays, grab/punch sweeps), and non-solid trigger volumes —
// e.g. the punch system's hand proxies riding a character's hands — must not read as
// walls. Detect trigger volumes with the onTrigger* callbacks, not casts.
class EntityIgnoreFilter final : public JPH::BodyFilter {
public:
    explicit EntityIgnoreFilter(entt::entity e)
        : m_ignore(e == entt::null ? ~0ull : (JPH::uint64)entt::to_integral(e)) {}
    bool ShouldCollide(const JPH::BodyID&)         const override { return true; }
    bool ShouldCollideLocked(const JPH::Body& b)   const override {
        return !b.IsSensor() && b.GetUserData() != m_ignore;
    }
private:
    JPH::uint64 m_ignore;
};

HitResult Raycast(glm::vec3 origin, glm::vec3 direction, float distance,
                  entt::entity ignore, bool drawDebug) {
    HitResult result;
    float len = glm::length(direction);
    if (!s_Impl || len < 1e-6f || distance <= 0.0f) return result;

    glm::vec3 normDir = direction / len;
    result.traceStart = origin;
    result.traceEnd   = origin + normDir * distance;

    // Jolt ray: origin + direction * [0..1], so direction encodes the max distance
    JPH::RRayCast ray {
        JPH::RVec3(origin.x, origin.y, origin.z),
        JPH::Vec3(normDir.x * distance, normDir.y * distance, normDir.z * distance)
    };

    // Ignore every body owned by the entity (a ragdoll has many).
    EntityIgnoreFilter bodyFilter(ignore);

    JPH::BroadPhaseLayerFilter bpFilter;
    JPH::ObjectLayerFilter     objFilter;
    JPH::RayCastResult hit;
    if (!s_Impl->joltSystem->GetNarrowPhaseQuery().CastRay(ray, hit, bpFilter, objFilter, bodyFilter)) {
        if (drawDebug)
            DebugDraw::Line(result.traceStart, result.traceEnd, {1.0f, 0.0f, 0.0f});
        return result;
    }

    result.hit      = true;
    result.distance = hit.mFraction * distance;
    result.point    = origin + normDir * result.distance;

    auto it = s_Impl->bodyMap.find(hit.mBodyID.GetIndexAndSequenceNumber());
    if (it != s_Impl->bodyMap.end())
        result.entity = it->second.entity;

    JPH::BodyLockRead lock(s_Impl->joltSystem->GetBodyLockInterface(), hit.mBodyID);
    if (lock.Succeeded()) {
        JPH::Vec3 n = lock.GetBody().GetWorldSpaceSurfaceNormal(
            hit.mSubShapeID2,
            JPH::RVec3(result.point.x, result.point.y, result.point.z));
        result.normal = FromJolt(n);
    }

    if (drawDebug) {
        DebugDraw::Line(result.traceStart, result.point, {0.0f, 1.0f, 0.0f});
        DebugDraw::Sphere(result.point, 0.05f, {1.0f, 1.0f, 0.0f});
    }

    return result;
}

HitResult SphereCast(glm::vec3 origin, float radius, glm::vec3 direction, float distance,
                     entt::entity ignore, bool drawDebug) {
    HitResult result;
    float len = glm::length(direction);
    if (!s_Impl || len < 1e-6f || distance <= 0.0f || radius <= 0.0f) return result;

    glm::vec3 normDir = direction / len;
    result.traceStart = origin;
    result.traceEnd   = origin + normDir * distance;

    JPH::SphereShapeSettings sphereSettings(radius);
    auto shapeResult = sphereSettings.Create();
    if (!shapeResult.IsValid()) return result;

    JPH::RShapeCast shapeCast = JPH::RShapeCast::sFromWorldTransform(
        shapeResult.Get(),
        JPH::Vec3::sReplicate(1.0f),
        JPH::RMat44::sTranslation(JPH::RVec3(origin.x, origin.y, origin.z)),
        JPH::Vec3(normDir.x * distance, normDir.y * distance, normDir.z * distance)
    );

    EntityIgnoreFilter bodyFilter(ignore);

    JPH::ShapeCastSettings settings;
    JPH::ClosestHitCollisionCollector<JPH::CastShapeCollector> collector;
    JPH::BroadPhaseLayerFilter bpFilter;
    JPH::ObjectLayerFilter     objFilter;

    s_Impl->joltSystem->GetNarrowPhaseQuery().CastShape(
        shapeCast, settings, JPH::RVec3::sZero(), collector,
        bpFilter, objFilter, bodyFilter
    );

    if (!collector.HadHit()) {
        if (drawDebug) {
            DebugDraw::Line(result.traceStart, result.traceEnd, {1.0f, 0.0f, 0.0f});
            DebugDraw::Sphere(result.traceStart, radius, {1.0f, 0.0f, 0.0f});
        }
        return result;
    }

    const auto& hit = collector.mHit;
    result.hit      = true;
    result.distance = hit.mFraction * distance;
    result.point    = FromJolt((JPH::Vec3)hit.mContactPointOn2);

    auto it = s_Impl->bodyMap.find(hit.mBodyID2.GetIndexAndSequenceNumber());
    if (it != s_Impl->bodyMap.end())
        result.entity = it->second.entity;

    JPH::BodyLockRead lock(s_Impl->joltSystem->GetBodyLockInterface(), hit.mBodyID2);
    if (lock.Succeeded()) {
        JPH::Vec3 n = lock.GetBody().GetWorldSpaceSurfaceNormal(
            hit.mSubShapeID2,
            hit.mContactPointOn2);
        result.normal = FromJolt(n);
    }

    if (drawDebug) {
        glm::vec3 impactCenter = origin + normDir * result.distance;
        DebugDraw::Line(result.traceStart, impactCenter, {0.0f, 1.0f, 0.0f});
        DebugDraw::Sphere(result.traceStart, radius, {0.0f, 1.0f, 0.0f});
        DebugDraw::Sphere(impactCenter,      radius, {1.0f, 1.0f, 0.0f});
    }

    return result;
}

std::vector<HitResult> RaycastMulti(glm::vec3 origin, glm::vec3 direction, float distance,
                                    entt::entity ignore, bool drawDebug) {
    std::vector<HitResult> results;
    float len = glm::length(direction);
    if (!s_Impl || len < 1e-6f || distance <= 0.0f) return results;

    glm::vec3 normDir = direction / len;
    glm::vec3 traceEnd = origin + normDir * distance;

    JPH::RRayCast ray {
        JPH::RVec3(origin.x, origin.y, origin.z),
        JPH::Vec3(normDir.x * distance, normDir.y * distance, normDir.z * distance)
    };

    EntityIgnoreFilter bodyFilter(ignore);

    JPH::RayCastSettings              rayCastSettings;
    JPH::AllHitCollisionCollector<JPH::CastRayCollector> collector;
    JPH::BroadPhaseLayerFilter bpFilter;
    JPH::ObjectLayerFilter     objFilter;
    s_Impl->joltSystem->GetNarrowPhaseQuery().CastRay(
        ray, rayCastSettings, collector, bpFilter, objFilter, bodyFilter);

    if (drawDebug)
        DebugDraw::Line(origin, traceEnd, collector.HadHit() ? glm::vec3{0,1,0} : glm::vec3{1,0,0});

    if (!collector.HadHit()) return results;

    // Sort nearest-first
    auto& hits = collector.mHits;
    std::sort(hits.begin(), hits.end(),
        [](const JPH::RayCastResult& a, const JPH::RayCastResult& b) {
            return a.mFraction < b.mFraction;
        });

    results.reserve(hits.size());
    for (const auto& hit : hits) {
        HitResult r;
        r.hit        = true;
        r.distance   = hit.mFraction * distance;
        r.traceStart = origin;
        r.traceEnd   = traceEnd;
        r.point      = origin + normDir * r.distance;

        auto it = s_Impl->bodyMap.find(hit.mBodyID.GetIndexAndSequenceNumber());
        if (it != s_Impl->bodyMap.end())
            r.entity = it->second.entity;

        JPH::BodyLockRead lock(s_Impl->joltSystem->GetBodyLockInterface(), hit.mBodyID);
        if (lock.Succeeded()) {
            JPH::Vec3 n = lock.GetBody().GetWorldSpaceSurfaceNormal(
                hit.mSubShapeID2,
                JPH::RVec3(r.point.x, r.point.y, r.point.z));
            r.normal = FromJolt(n);
        }

        if (drawDebug)
            DebugDraw::Sphere(r.point, 0.05f, {1.0f, 1.0f, 0.0f});

        results.push_back(r);
    }

    return results;
}

std::vector<HitResult> SphereCastMulti(glm::vec3 origin, float radius, glm::vec3 direction, float distance,
                                       entt::entity ignore, bool drawDebug) {
    std::vector<HitResult> results;
    float len = glm::length(direction);
    if (!s_Impl || len < 1e-6f || distance <= 0.0f || radius <= 0.0f) return results;

    glm::vec3 normDir  = direction / len;
    glm::vec3 traceEnd = origin + normDir * distance;

    JPH::SphereShapeSettings sphereSettings(radius);
    auto shapeResult = sphereSettings.Create();
    if (!shapeResult.IsValid()) return results;

    JPH::RShapeCast shapeCast = JPH::RShapeCast::sFromWorldTransform(
        shapeResult.Get(),
        JPH::Vec3::sReplicate(1.0f),
        JPH::RMat44::sTranslation(JPH::RVec3(origin.x, origin.y, origin.z)),
        JPH::Vec3(normDir.x * distance, normDir.y * distance, normDir.z * distance)
    );

    EntityIgnoreFilter bodyFilter(ignore);

    JPH::ShapeCastSettings settings;
    JPH::AllHitCollisionCollector<JPH::CastShapeCollector> collector;
    JPH::BroadPhaseLayerFilter bpFilter;
    JPH::ObjectLayerFilter     objFilter;
    s_Impl->joltSystem->GetNarrowPhaseQuery().CastShape(
        shapeCast, settings, JPH::RVec3::sZero(), collector,
        bpFilter, objFilter, bodyFilter);

    if (drawDebug) {
        glm::vec3 lineColor = collector.HadHit() ? glm::vec3{0,1,0} : glm::vec3{1,0,0};
        DebugDraw::Line(origin, traceEnd, lineColor);
        DebugDraw::Sphere(origin, radius, lineColor);
    }

    if (!collector.HadHit()) return results;

    // Sort nearest-first
    auto& hits = collector.mHits;
    std::sort(hits.begin(), hits.end(),
        [](const JPH::ShapeCastResult& a, const JPH::ShapeCastResult& b) {
            return a.mFraction < b.mFraction;
        });

    results.reserve(hits.size());
    for (const auto& hit : hits) {
        HitResult r;
        r.hit        = true;
        r.distance   = hit.mFraction * distance;
        r.traceStart = origin;
        r.traceEnd   = traceEnd;
        r.point      = FromJolt((JPH::Vec3)hit.mContactPointOn2);

        auto it = s_Impl->bodyMap.find(hit.mBodyID2.GetIndexAndSequenceNumber());
        if (it != s_Impl->bodyMap.end())
            r.entity = it->second.entity;

        JPH::BodyLockRead lock(s_Impl->joltSystem->GetBodyLockInterface(), hit.mBodyID2);
        if (lock.Succeeded()) {
            JPH::Vec3 n = lock.GetBody().GetWorldSpaceSurfaceNormal(
                hit.mSubShapeID2, hit.mContactPointOn2);
            r.normal = FromJolt(n);
        }

        if (drawDebug) {
            glm::vec3 impactCenter = origin + normDir * r.distance;
            DebugDraw::Sphere(impactCenter, radius, {1.0f, 1.0f, 0.0f});
        }

        results.push_back(r);
    }

    return results;
}

void DrawColliders(Scene& scene, glm::vec3 color) {
    static std::unordered_map<std::string, std::vector<Diamond::MeshData>> s_visCache;

    for (auto [entity, col, xform] : scene.View<ColliderComponent, TransformComponent>().each()) {
        glm::quat worldRot = xform.rotation * col.localRotation;
        glm::vec3 worldPos = xform.position + xform.rotation * col.localOffset;

        switch (col.shapeType) {
            case CollisionShape::Box:
                DebugDraw::Box(worldPos, worldRot, col.halfExtents, color);
                break;
            case CollisionShape::Sphere:
                DebugDraw::Sphere(worldPos, col.radius, color);
                break;
            case CollisionShape::Capsule:
                DebugDraw::Capsule(worldPos, worldRot, col.halfHeight, col.radius, color);
                break;
            case CollisionShape::ConvexHull:
            case CollisionShape::TriangleMesh:
            {
                // Resolve path — fall back to MeshComponent if collider path is empty
                std::string path = col.meshPath;
                if (path.empty() && scene.Has<MeshComponent>(entity))
                    path = scene.Get<MeshComponent>(entity).meshPath;
                if (path.empty()) break;

                auto [it, inserted] = s_visCache.emplace(path, std::vector<Diamond::MeshData>{});
                if (inserted) it->second = Diamond::ModelImporter::Load(path);

                glm::mat4 transform =
                    glm::translate(glm::mat4(1.0f), worldPos) *
                    glm::mat4_cast(worldRot) *
                    glm::scale(glm::mat4(1.0f), xform.scale);

                for (auto& mesh : it->second) {
                    for (size_t i = 0; i + 2 < mesh.Indices.size(); i += 3) {
                        glm::vec3 a = transform * glm::vec4(mesh.Vertices[mesh.Indices[i    ]].Position, 1.0f);
                        glm::vec3 b = transform * glm::vec4(mesh.Vertices[mesh.Indices[i + 1]].Position, 1.0f);
                        glm::vec3 c = transform * glm::vec4(mesh.Vertices[mesh.Indices[i + 2]].Position, 1.0f);
                        DebugDraw::Line(a, b, color);
                        DebugDraw::Line(b, c, color);
                        DebugDraw::Line(c, a, color);
                    }
                }
                break;
            }
            default: break;
        }
    }
}

} // namespace Physics
