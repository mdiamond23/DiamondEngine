// Jolt must be included before anything else that might define conflicting macros
#include <Jolt/Jolt.h>
#include <Jolt/RegisterTypes.h>
#include <Jolt/Core/Factory.h>
#include <Jolt/Core/TempAllocator.h>
#include <Jolt/Core/JobSystemSingleThreaded.h>
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
#include <Jolt/Physics/Constraints/Constraint.h>
#include <Jolt/Physics/Constraints/TwoBodyConstraint.h>
#include <Jolt/Physics/Constraints/HingeConstraint.h>
#include <Jolt/Physics/Constraints/SwingTwistConstraint.h>
#include <Jolt/Physics/Constraints/FixedConstraint.h>
#include <Jolt/Physics/Constraints/PointConstraint.h>

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
    glm::vec3    contactNormal { 0.0f };
};

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
        if (!isTrigger) {
            JPH::RVec3 wp = manifold.GetWorldSpaceContactPointOn1(0);
            JPH::Vec3  wn = manifold.mWorldSpaceNormal;
            ev.contactPoint  = { (float)wp.GetX(), (float)wp.GetY(), (float)wp.GetZ() };
            ev.contactNormal = { wn.GetX(), wn.GetY(), wn.GetZ() };
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
        JPH::RVec3 wp = manifold.GetWorldSpaceContactPointOn1(0);
        JPH::Vec3  wn = manifold.mWorldSpaceNormal;
        ev.contactPoint  = { (float)wp.GetX(), (float)wp.GetY(), (float)wp.GetZ() };
        ev.contactNormal = { wn.GetX(), wn.GetY(), wn.GetZ() };
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
};

struct RagdollInstance {
    entt::entity             entity = entt::null;
    std::vector<RagdollBody> bodies;
    std::vector<RagdollJoint> joints;         // joints into constraintMap (+ rebuild data)
    int                      rootBodySlot = -1; // index into bodies of the hips (re-root target)
    uint32_t                 group = 0;        // per-ragdoll self-collision group
    RagdollMode              mode  = RagdollMode::Animated;
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

    std::unique_ptr<JPH::TempAllocatorImpl>       tempAllocator;
    std::unique_ptr<JPH::JobSystemSingleThreaded>  jobSystem;     // swap for JobSystemThreadPool to enable MT
    std::unique_ptr<JPH::PhysicsSystem>            joltSystem;
    std::unique_ptr<ContactListener>               contactListener;
    JPH::Ref<JPH::GroupFilter>                     groupFilter;   // shared by all grouped bodies
};

// ---------------------------------------------------------------------------
// Active session pointer — set in OnStart, cleared in OnDestroy.
// Signal callbacks and Physics:: API functions use this to reach Jolt.
// ---------------------------------------------------------------------------
static PhysicsSystem::Impl* s_Impl = nullptr;

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
        axisWorld[k] = SafeAxis(glm::mat3(bw) * def.twistAxisLocal, def.twistAxisLocal);

        glm::vec3 shapeOffset; glm::quat shapeRot;
        JPH::ShapeRefC shapeRef = CreateRagdollShape(def, scale, shapeOffset, shapeRot);
        if (!shapeRef) continue;

        JPH::BodyCreationSettings settings(shapeRef, ToJolt(pos), ToJolt(rot),
                                           JPH::EMotionType::Kinematic, PhysicsLayers::DYNAMIC);
        settings.mUserData = (JPH::uint64)entt::to_integral(entity);
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
        rb.shape         = def.shape;
        rb.radius        = def.radius     * scale;
        rb.halfHeight    = def.halfHeight * scale;
        rb.halfExtents   = def.halfExtents * scale;
        rb.localOffset   = shapeOffset;
        rb.localRotation = shapeRot;
        nameToSlot[def.boneName] = (int)inst.bodies.size();
        if (def.parentBoneName.empty() && inst.rootBodySlot < 0)
            inst.rootBodySlot = (int)inst.bodies.size();
        inst.bodies.push_back(rb);
    }

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
        cc.motorMode      = MotorMode::Off;  // v1 passive ragdoll

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
        joint.twistAxisLocal = def.twistAxisLocal;
        inst.joints.push_back(joint);
    }

    if (inst.bodies.empty()) return;
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

// ---------------------------------------------------------------------------
// PhysicsSystem — OnStart / OnUpdate / OnDestroy
// ---------------------------------------------------------------------------
void PhysicsSystem::OnStart(Scene& scene) {
    JPH::RegisterDefaultAllocator();

    JPH::Factory::sInstance = new JPH::Factory();
    JPH::RegisterTypes();

    m_impl = std::make_unique<Impl>();
    m_impl->tempAllocator = std::make_unique<JPH::TempAllocatorImpl>(32 * 1024 * 1024);
    m_impl->jobSystem     = std::make_unique<JPH::JobSystemSingleThreaded>(2048);

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

    // Clamp dt contribution — prevents death spiral when fps drops below 60.
    m_accumulator += std::min(dt, FIXED_DT);

    JPH::BodyInterface& bi = m_impl->joltSystem->GetBodyInterface();
    while (m_accumulator >= FIXED_DT) {
        SyncKinematicBodies(scene, bi);
        SyncRagdollKinematic(scene, *m_impl, bi);   // Animated ragdolls follow the pose
        m_impl->joltSystem->Update(FIXED_DT, 2, m_impl->tempAllocator.get(), m_impl->jobSystem.get());
        SyncTransforms(scene, bi);
        DispatchCallbacks(scene, m_impl->contactListener->DrainEvents());
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

// ---- Ragdoll ----------------------------------------------------------------

// Flip a ragdoll between Animated (kinematic, follows the pose) and Limp (dynamic,
// flops). Velocity is inherited from the kinematic follow on the way to Limp
// (concern #1); on the way back the next follow snaps it to the animation.
void SetRagdollMode(RagdollComponent& rag, RagdollMode mode) {
    if (!s_Impl || rag._ragdollId == 0xFFFFFFFFu) return;
    auto it = s_Impl->ragdolls.find(rag._ragdollId);
    if (it == s_Impl->ragdolls.end()) return;
    RagdollInstance& inst = it->second;

    if (inst.mode != mode) {
        JPH::BodyInterface& bi = s_Impl->joltSystem->GetBodyInterface();
        // Re-center the joint limits on the live pose before the bodies go Dynamic,
        // so no limb is born outside its cone and yanked back (the explosion).
        if (mode == RagdollMode::Limp)
            RebuildRagdollJointsAtPose(*s_Impl, inst);
        JPH::EMotionType mt = (mode == RagdollMode::Limp)
            ? JPH::EMotionType::Dynamic : JPH::EMotionType::Kinematic;
        for (const RagdollBody& b : inst.bodies)
            bi.SetMotionType(b.id, mt, JPH::EActivation::Activate);
        inst.mode = mode;
    }
    rag.mode = mode;
}

// After the physics step AND after the animation palette is built: for each Limp
// ragdoll, overwrite the skinning palette from the simulated bodies and re-root
// the entity transform to the hips (concern #2). Mapped bones read their body's
// world transform; unmapped bones ride their nearest physics-driven ancestor by
// keeping their freshly-animated local offset. Call once per frame from the main
// loop, after UpdateAnimators.
void SyncRagdollPoses(Scene& scene) {
    if (!s_Impl || s_Impl->ragdolls.empty()) return;
    auto& reg = scene.GetRegistry();
    JPH::BodyInterface& bi = s_Impl->joltSystem->GetBodyInterface();

    for (auto& [id, inst] : s_Impl->ragdolls) {
        if (inst.mode != RagdollMode::Limp || !reg.valid(inst.entity)) continue;
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

    // Find the Jolt BodyID for the entity to ignore (reverse lookup in bodyMap)
    JPH::BodyID ignoreID = JPH::BodyID();
    if (ignore != entt::null) {
        for (auto& [id, record] : s_Impl->bodyMap) {
            if (record.entity == ignore) { ignoreID = JPH::BodyID(id); break; }
        }
    }
    SingleBodyIgnoreFilter bodyFilter(ignoreID);

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

    JPH::BodyID ignoreID = JPH::BodyID();
    if (ignore != entt::null) {
        for (auto& [id, record] : s_Impl->bodyMap) {
            if (record.entity == ignore) { ignoreID = JPH::BodyID(id); break; }
        }
    }
    SingleBodyIgnoreFilter bodyFilter(ignoreID);

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

    JPH::BodyID ignoreID = JPH::BodyID();
    if (ignore != entt::null) {
        for (auto& [id, record] : s_Impl->bodyMap) {
            if (record.entity == ignore) { ignoreID = JPH::BodyID(id); break; }
        }
    }
    SingleBodyIgnoreFilter bodyFilter(ignoreID);

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

    JPH::BodyID ignoreID = JPH::BodyID();
    if (ignore != entt::null) {
        for (auto& [id, record] : s_Impl->bodyMap) {
            if (record.entity == ignore) { ignoreID = JPH::BodyID(id); break; }
        }
    }
    SingleBodyIgnoreFilter bodyFilter(ignoreID);

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
