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
#include <Jolt/Physics/Body/BodyLockInterface.h>

#include "Scene/Physics/PhysicsSystem.h"
#include "Scene/Physics/PhysicsAPI.h"
#include "Scene/Physics/Rigidbody.h"
#include "Scene/Physics/Collision.h"
#include "Scene/Components.h"
#include "Scene/Scene.h"
#include "DebugDraw.h"
#include "Assets/ModelImporter.h"

#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>

#include <spdlog/spdlog.h>

#include <unordered_map>
#include <unordered_set>
#include <vector>
#include <mutex>
#include <utility>
#include <cstdint>
#include <algorithm>

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
// Impl — owns all Jolt objects (kept out of the public header via PIMPL)
// ---------------------------------------------------------------------------
struct PhysicsSystem::Impl {
    BPLayerInterfaceImpl      bpLayerInterface;
    ObjectVsBPLayerFilterImpl objVsBPFilter;
    ObjectLayerPairFilterImpl objLayerFilter;

    std::unordered_map<uint32_t, BodyRecord> bodyMap; // BodyID → entity info
    std::vector<entt::entity>                pendingCreate; // deferred from signal callbacks

    std::unique_ptr<JPH::TempAllocatorImpl>       tempAllocator;
    std::unique_ptr<JPH::JobSystemSingleThreaded>  jobSystem;     // swap for JobSystemThreadPool to enable MT
    std::unique_ptr<JPH::PhysicsSystem>            joltSystem;
    std::unique_ptr<ContactListener>               contactListener;
};

// ---------------------------------------------------------------------------
// Active session pointer — set in OnStart, cleared in OnDestroy.
// Signal callbacks and Physics:: API functions use this to reach Jolt.
// ---------------------------------------------------------------------------
static PhysicsSystem::Impl* s_Impl = nullptr;

static std::unordered_map<std::string, JPH::ShapeRefC> s_convexHullCache;
static std::unordered_map<std::string, JPH::ShapeRefC> s_triangleMeshCache;

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
    JPH::BodyInterface& bi = s_Impl->joltSystem->GetBodyInterface();
    JPH::BodyID bodyID(col._bodyId);
    bi.RemoveBody(bodyID);
    bi.DestroyBody(bodyID);
    s_Impl->bodyMap.erase(col._bodyId);
}

// ---------------------------------------------------------------------------
// CreateShape
// Returns a JPH::ShapeRefC (Jolt's ref-counted shape handle), or nullptr on failure.
// ---------------------------------------------------------------------------
static JPH::ShapeRefC CreateShape(const ColliderComponent& col)
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
        auto cacheIt = s_convexHullCache.find(col.meshPath);
        if (cacheIt != s_convexHullCache.end()) { shape = cacheIt->second; break; }

        auto meshes = Diamond::ModelImporter::Load(col.meshPath);
        if (meshes.empty()) return nullptr;

        JPH::Array<JPH::Vec3> points;
        for (auto& mesh : meshes)
            for (auto& v : mesh.Vertices)
                points.push_back({ v.Position.x, v.Position.y, v.Position.z });

        if (points.empty()) return nullptr;

        JPH::ConvexHullShapeSettings settings(points);
        auto result = settings.Create();
        if (!result.IsValid()) {
            spdlog::warn("CreateShape: ConvexHull creation failed for '{}'", col.meshPath);
            return nullptr;
        }
        shape = result.Get();
        s_convexHullCache[col.meshPath] = shape;
        break;
    }
    case CollisionShape::TriangleMesh:
    {
        if (col.meshPath.empty()) {
            spdlog::warn("CreateShape: TriangleMesh collider has no meshPath");
            return nullptr;
        }
        auto cacheIt = s_triangleMeshCache.find(col.meshPath);
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
                    JPH::Float3(v0.x, v0.y, v0.z),
                    JPH::Float3(v1.x, v1.y, v1.z),
                    JPH::Float3(v2.x, v2.y, v2.z)
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
        s_triangleMeshCache[col.meshPath] = shape;
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
    const ColliderComponent& col, const TransformComponent& xform)
{
    if (col.shapeType == CollisionShape::TriangleMesh && rb.bodyType != BodyType::Static) {
        spdlog::warn("CreateBody: TriangleMesh collider requires a Static body; skipping.");
        return;
    }

    JPH::ShapeRefC shapeRef = CreateShape(col);
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
    entt::entity, ColliderComponent&, const TransformComponent&);

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

    JPH::BodyInterface& bi = m_impl->joltSystem->GetBodyInterface();

    // Entities with RigidBodyComponent + ColliderComponent → full physics body
    for (auto [entity, rb, col, xform] : scene.View<RigidBodyComponent, ColliderComponent, TransformComponent>().each()) {
        InheritMeshPath(col, entity, scene.GetRegistry());
        CreateBody(bi, m_impl->bodyMap, entity, rb, col, xform);
    }

    // Entities with only ColliderComponent → static collision geometry
    for (auto [entity, col, xform] : scene.View<ColliderComponent, TransformComponent>().each()) {
        if (scene.Has<RigidBodyComponent>(entity)) continue;
        InheritMeshPath(col, entity, scene.GetRegistry());
        CreateStaticCollider(bi, m_impl->bodyMap, entity, col, xform);
    }

    // Expose the active impl and connect EnTT signals for runtime body lifecycle.
    // OnBodyComponentAdded defers creation; OnRb/ColDestroyed tear down immediately.
    s_Impl = m_impl.get();
    auto& reg = scene.GetRegistry();
    reg.on_construct<RigidBodyComponent>().connect<&OnBodyComponentAdded>();
    reg.on_construct<ColliderComponent>().connect<&OnBodyComponentAdded>();
    reg.on_destroy<RigidBodyComponent>().connect<&OnRbDestroyed>();
    reg.on_destroy<ColliderComponent>().connect<&OnColDestroyed>();

}

// ---------------------------------------------------------------------------
// CreateStaticCollider
// Creates a static Jolt body for entities that have a ColliderComponent but
// no RigidBodyComponent (pure collision geometry, static world meshes, etc.).
// The body ID is written into col._bodyId so OnDestroy can remove it.
// ---------------------------------------------------------------------------
static void CreateStaticCollider(JPH::BodyInterface& bi,
    std::unordered_map<uint32_t, BodyRecord>& bodyMap,
    entt::entity entity, ColliderComponent& col, const TransformComponent& xform)
{
    JPH::ShapeRefC shapeRef = CreateShape(col);
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
                CreateBody(pbi, m_impl->bodyMap, entity, rb, col, xform);
            } else {
                auto& col   = reg.get<ColliderComponent>(entity);
                auto& xform = reg.get<TransformComponent>(entity);
                if (col._bodyId != 0xFFFFFFFFu) continue; // already created
                InheritMeshPath(col, entity, reg);
                CreateStaticCollider(pbi, m_impl->bodyMap, entity, col, xform);
            }
        }
        m_impl->pendingCreate.clear();
    }

    // Clamp dt contribution — prevents death spiral when fps drops below 60.
    m_accumulator += std::min(dt, FIXED_DT);

    JPH::BodyInterface& bi = m_impl->joltSystem->GetBodyInterface();
    while (m_accumulator >= FIXED_DT) {
        SyncKinematicBodies(scene, bi);
        m_impl->joltSystem->Update(FIXED_DT, 1, m_impl->tempAllocator.get(), m_impl->jobSystem.get());
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
