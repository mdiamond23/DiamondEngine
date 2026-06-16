# Physics System Design (Jolt)

This document supersedes the "Collision + Physics" section of `gameplay-foundation-design.md`. We are implementing full rigid body simulation (previously Phase 2) directly, skipping Phase 1's collision-only scope.

---

## Goals

- Rigid body simulation with Static, Dynamic, and Kinematic body types
- All primitive collision shapes plus convex hull and triangle mesh
- Trigger volumes with entry/exit callbacks
- Collision callbacks via `std::function`
- Fixed timestep at 60 Hz, capped to one step per frame to prevent death spirals
- `PhysicsMaterial` asset for reusable friction and restitution values
- Single-threaded first; designed so the job system can be swapped to multi-threaded with one line

---

## Dependency — Jolt Physics

Added via FetchContent in `Engine/CMakeLists.txt`:

```cmake
FetchContent_Declare(
    JoltPhysics
    GIT_REPOSITORY https://github.com/jrouwe/JoltPhysics.git
    GIT_TAG        v5.2.0
    SOURCE_SUBDIR  Build
)
set(PHYSICS_DOUBLE_PRECISION OFF CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(JoltPhysics)

# In Engine target link:
target_link_libraries(MyEngine PRIVATE Jolt)
target_include_directories(MyEngine PRIVATE ${joltphysics_SOURCE_DIR})
```

Jolt requires C++17 or later (C++20 is fine). No Python or code generation step.

---

## Components

### RigidBodyComponent

Controls how Jolt simulates the body. Stores the internal Jolt `BodyID` and collision callbacks.

```cpp
enum class BodyType { Static, Dynamic, Kinematic };

struct CollisionContact {
    entt::entity other;
    glm::vec3    contactPoint;
    glm::vec3    contactNormal;
};

struct RigidBodyComponent {
    BodyType bodyType       = BodyType::Static;
    float    mass           = 1.0f;           // Dynamic only
    float    linearDamping  = 0.05f;
    float    angularDamping = 0.05f;
    float    gravityScale   = 1.0f;
    bool     lockRotX       = false;
    bool     lockRotY       = false;
    bool     lockRotZ       = false;

    // Internal — managed by PhysicsSystem, do not set manually
    uint32_t _bodyId = JPH::BodyID::cInvalidBodyID;
};
```

**Kinematic usage:** Scripts move the entity's `TransformComponent` (position/rotation). `PhysicsSystem` reads those values and calls `MoveKinematic()` on the Jolt body each step. This lets Jolt compute the body's velocity so it correctly pushes dynamic bodies.

### ColliderComponent

Defines the collision shape and whether the body is a trigger (sensor).

```cpp
enum class CollisionShape { Box, Sphere, Capsule, ConvexHull, TriangleMesh };

struct ColliderComponent {
    CollisionShape shapeType = CollisionShape::Box;

    // Box
    glm::vec3 halfExtents { 0.5f, 0.5f, 0.5f };
    // Sphere + Capsule
    float radius     = 0.5f;
    // Capsule (cylinder halfHeight, capped by radius on each end)
    float halfHeight = 0.5f;
    // ConvexHull / TriangleMesh — path to mesh asset
    std::string meshPath;

    glm::vec3 localOffset   { 0.0f };
    glm::quat localRotation { 1, 0, 0, 0 };  // identity

    bool isTrigger = false;

    // Collision callbacks — set by scripts, fired by PhysicsSystem
    std::function<void(CollisionContact)> onCollisionEnter;
    std::function<void(CollisionContact)> onCollisionStay;
    std::function<void(CollisionContact)> onCollisionExit;
    std::function<void(entt::entity)>     onTriggerEnter;
    std::function<void(entt::entity)>     onTriggerExit;

    std::shared_ptr<PhysicsMaterial> material;  // nullptr = engine default
};
```

`isTrigger = true` maps to a Jolt sensor body — overlaps are detected via `ContactListener` but no collision response is generated.

A Jolt body is created when an entity has **both** a `ColliderComponent` and a `RigidBodyComponent`. Entities with only `ColliderComponent` are not currently supported (add `RigidBodyComponent` with `BodyType::Static` for static world geometry).

### PhysicsMaterial

A reusable asset that controls surface behaviour. Saved as a small JSON file (`.physmat`).

```cpp
struct PhysicsMaterial {
    float staticFriction  = 0.5f;
    float dynamicFriction = 0.5f;
    float restitution     = 0.3f;  // bounciness [0, 1]
};
```

Multiple `ColliderComponent`s point to the same `shared_ptr<PhysicsMaterial>` — editing the asset updates all colliders that reference it. `nullptr` uses Jolt's built-in defaults.

---

## PhysicsSystem

Implemented as a `GameSystem` so it participates in the play/stop lifecycle automatically.

```cpp
class PhysicsSystem : public GameSystem {
    DECLARE_SYSTEM(PhysicsSystem, 200)
public:
    void OnStart(Scene& scene)            override;
    void OnUpdate(Scene& scene, float dt) override;
    void OnDestroy(Scene& scene)          override;
private:
    std::unique_ptr<JPH::TempAllocatorImpl> m_tempAllocator;
    std::unique_ptr<JPH::JobSystem>         m_jobSystem;   // ← swap this for MT
    std::unique_ptr<JPH::PhysicsSystem>     m_joltSystem;
    std::unique_ptr<ContactListener>        m_contactListener;
    float                                   m_accumulator = 0.0f;
};
```

**Why priority 200?** Pre-physics scripts (player movement, AI) run at priorities 100–199 and set kinematic body positions. PhysicsSystem reads those positions at 200, steps, and writes results back to `TransformComponent`. Post-physics scripts (collision reactions) run at 300–399.

| Priority | Purpose |
|---|---|
| 0–99   | Core engine |
| 100–199 | Pre-physics scripts (movement, AI) |
| **200** | **PhysicsSystem** |
| 300–399 | Post-physics scripts (gameplay reactions) |
| 400–499 | Camera |
| 500+   | Late update |

### Initialization (`OnStart`)

```cpp
void PhysicsSystem::OnStart(Scene& scene) {
    JPH::RegisterDefaultAllocator();
    JPH::Factory::sInstance = new JPH::Factory();
    JPH::RegisterTypes();

    m_tempAllocator = std::make_unique<JPH::TempAllocatorImpl>(10 * 1024 * 1024);
    m_jobSystem     = std::make_unique<JPH::JobSystemSingleThreaded>(JPH::cMaxPhysicsJobs);

    // See PhysicsLayers.h for layer/filter setup
    m_joltSystem = std::make_unique<JPH::PhysicsSystem>();
    m_joltSystem->Init(
        cMaxBodies, cNumBodyMutexes, cMaxBodyPairs, cMaxContactConstraints,
        broadPhaseLayerInterface, objectVsBroadPhaseFilter, objectLayerPairFilter
    );

    m_contactListener = std::make_unique<ContactListener>(scene);
    m_joltSystem->SetContactListener(m_contactListener.get());

    // Create bodies for all entities that have both components
    auto view = scene.View<RigidBodyComponent, ColliderComponent, TransformComponent>();
    for (auto [entity, rb, col, xform] : view.each())
        CreateBody(scene, entity, rb, col, xform);

    // Listen for runtime component additions/removals
    scene.Registry().on_construct<ColliderComponent>().connect<&PhysicsSystem::OnColliderAdded>(this);
    scene.Registry().on_destroy<ColliderComponent>().connect<&PhysicsSystem::OnColliderRemoved>(this);
}
```

### Fixed Timestep (`OnUpdate`)

```cpp
static constexpr float FIXED_DT = 1.0f / 60.0f;

void PhysicsSystem::OnUpdate(Scene& scene, float dt) {
    // Clamp dt contribution — prevents death spiral when fps < 60.
    // At 30 fps: accumulator gets 1/60 added (not 1/30), so exactly one step fires.
    // At 120 fps: accumulator gets 1/120 added, step fires every other frame.
    m_accumulator += std::min(dt, FIXED_DT);

    while (m_accumulator >= FIXED_DT) {
        SyncKinematicBodies(scene);   // Transform → Jolt (kinematic only)
        m_joltSystem->Update(FIXED_DT, 1, m_tempAllocator.get(), m_jobSystem.get());
        SyncTransforms(scene);        // Jolt → Transform (dynamic + kinematic)
        DispatchCallbacks(scene);     // Fire OnCollision*/OnTrigger* callbacks
        m_accumulator -= FIXED_DT;
    }
}
```

### Transform Sync

```cpp
void PhysicsSystem::SyncKinematicBodies(Scene& scene) {
    auto& bi = m_joltSystem->GetBodyInterface();
    for (auto [entity, rb, xform] : scene.View<RigidBodyComponent, TransformComponent>().each()) {
        if (rb.bodyType != BodyType::Kinematic) continue;
        bi.MoveKinematic(
            JPH::BodyID(rb._bodyId),
            ToJolt(xform.position),
            ToJolt(xform.rotation),
            FIXED_DT
        );
    }
}

void PhysicsSystem::SyncTransforms(Scene& scene) {
    auto& bi = m_joltSystem->GetBodyInterfaceNoLock();
    for (auto [entity, rb, xform] : scene.View<RigidBodyComponent, TransformComponent>().each()) {
        if (rb.bodyType == BodyType::Static) continue;
        xform.position = FromJolt(bi.GetPosition(JPH::BodyID(rb._bodyId)));
        xform.rotation = FromJolt(bi.GetRotation(JPH::BodyID(rb._bodyId)));
    }
}
```

---

## Collision and Trigger Callbacks

### ContactListener

`ContactListener` (private, in `Engine/src/Physics/`) implements `JPH::ContactListener`. It accumulates events into queues and `PhysicsSystem::DispatchCallbacks()` drains them after each step — never firing `std::function` from inside Jolt's solver thread.

```cpp
// Events queued inside Jolt callbacks (called from solver context):
struct ContactEvent {
    enum class Type { Enter, Stay, Exit, TriggerEnter, TriggerExit };
    Type         type;
    entt::entity a, b;
    glm::vec3    contactPoint;
    glm::vec3    contactNormal;
};
```

`OnContactAdded` → `ContactEvent::Enter` or `TriggerEnter`
`OnContactPersisted` → `ContactEvent::Stay`
`OnContactRemoved` → `ContactEvent::Exit` or `TriggerExit`

`Stay` tracking requires a `std::unordered_set<BodyPair>` of currently active pairs; `OnContactRemoved` fires when a pair leaves the set.

### Dispatching

```cpp
void PhysicsSystem::DispatchCallbacks(Scene& scene) {
    for (auto& event : m_contactListener->DrainEvents()) {
        auto fire = [&](entt::entity e, entt::entity other) {
            if (!scene.Has<RigidBodyComponent>(e)) return;
            auto& rb = scene.Get<RigidBodyComponent>(e);
            CollisionContact contact { other, event.contactPoint, event.contactNormal };
            switch (event.type) {
                case ContactEvent::Type::Enter:       if (rb.onCollisionEnter) rb.onCollisionEnter(contact); break;
                case ContactEvent::Type::Stay:        if (rb.onCollisionStay)  rb.onCollisionStay(contact);  break;
                case ContactEvent::Type::Exit:        if (rb.onCollisionExit)  rb.onCollisionExit(contact);  break;
                case ContactEvent::Type::TriggerEnter: if (rb.onTriggerEnter) rb.onTriggerEnter(other);     break;
                case ContactEvent::Type::TriggerExit:  if (rb.onTriggerExit)  rb.onTriggerExit(other);      break;
            }
        };
        fire(event.a, event.b);
        fire(event.b, event.a);  // symmetric — both entities get the callback
    }
}
```

### Script Usage

```cpp
class DoorSystem : public GameSystem {
    DECLARE_SYSTEM(DoorSystem, 300)
    void OnStart(Scene& scene) override {
        for (auto [entity, door] : scene.View<DoorComponent>().each()) {
            auto& rb = scene.Get<RigidBodyComponent>(entity);
            rb.onTriggerEnter = [&scene, entity](entt::entity other) {
                scene.Get<DoorComponent>(entity).open = true;
            };
        }
    }
};
```

---

## Constraints (Joints)

Constraints connect two bodies (or one body and the immovable world) and remove
some of the 6 relative degrees of freedom between them. Each joint type fixes a
different subset: a hinge leaves one rotational DOF, a slider one translational,
a point joint three rotational, a fixed joint none.

**Status:** the hinge is implemented end to end (component, deferred creation,
destruction safety, entity-to-entity targeting, angular limits, serialization,
duplication). The remaining joint types and behaviours below are designed-for
but not yet built — see *Roadmap*.

### ConstraintComponent

Plain data only — no Jolt types — so `Engine/include/` consumers never need the
Jolt include path, exactly like `RigidBodyComponent`. The live `JPH::Constraint`
is owned by `PhysicsSystem` and referenced here by an opaque id.

```cpp
enum class ConstraintType { Hinge /*, Point, Fixed, Slider, Distance (future) */ };

struct ConstraintComponent {
    ConstraintType type = ConstraintType::Hinge;

    // The other body. 0 = attach to the immovable world (Body::sFixedToWorld).
    // Non-zero = another entity, referenced by its persistent IDComponent UUID
    // so the link survives save/load (a raw entt::entity handle would not).
    uint64_t targetUuid = 0;

    // Anchor point and rotation axis in WORLD space, as authored in the editor.
    // PhysicsSystem converts these to per-body local frames at creation time via
    // Jolt's EConstraintSpace::WorldSpace mode.
    glm::vec3 anchor { 0.0f, 0.0f, 0.0f };
    glm::vec3 axis   { 0.0f, 0.0f, 1.0f };

    // Optional angular limits, in DEGREES (converted to radians at creation).
    bool  hasLimits = false;
    float limitMin  = -90.0f;
    float limitMax  =  90.0f;

    // Internal — managed by PhysicsSystem, do not set manually. Invalid sentinel
    // mirrors RigidBodyComponent::_bodyId. Never serialized.
    uint32_t _constraintId = 0xFFFFFFFF;
};
```

**Anchor frames.** A joint must stay valid as both bodies move, so the anchor is
not stored as a frozen world point — Jolt converts the single authored world
anchor into a local frame on *each* body. Every step it transforms both back to
world space and applies impulses to keep them coincident. The drift between the
two frames is exactly what the solver corrects.

### Why constraints are different from every other component

A `MeshComponent` or `RigidBodyComponent` describes one entity and dies with it.
A constraint is the first thing in the engine that **spans two entities** and must
stay valid as both independently spawn, move, and die. Three of the lifecycle
mechanisms below exist solely because of this.

### Creation — deferred, like bodies

A body needs only its own entity to be set up. A constraint needs **two live Jolt
bodies**, and at scene-load time the target entity (or its body) may not exist yet.
So constraint creation reuses the body system's deferred queue pattern:

- `on_construct<ConstraintComponent>` pushes the entity to `pendingConstraints`.
- `OnStart` scans pre-existing constraints into the same queue (signals don't fire
  for components that existed before the connection).
- `OnUpdate` drains the queue *after* the body-creation drain. An entry stays
  queued until its readiness condition holds: **this entity's body is live, and —
  for entity-to-entity joints — the target entity exists (`Scene::FindByUuid`) and
  its body is live too.** Missing prerequisites just mean "wait one more frame,"
  never a crash.

```cpp
// readiness check, per queued entity
uint32_t bodyA = BodyIdOf(reg, e);                 // own body
if (bodyA == invalid) { stillPending.push_back(e); continue; }
uint32_t bodyB = invalid;                          // world
if (cc.targetUuid != 0) {
    entt::entity target = scene.FindByUuid(cc.targetUuid);
    if (!reg.valid(target)) { stillPending.push_back(e); continue; }
    bodyB = BodyIdOf(reg, target);
    if (bodyB == invalid) { stillPending.push_back(e); continue; }
}
CreateConstraint(impl, e, cc, bodyA, bodyB);
```

Building the joint requires `JPH::Body&` references, obtained by locking. World
joints lock one body (`BodyLockWrite`) and use `Body::sFixedToWorld` as body1.
Entity-to-entity joints lock both with `BodyLockMultiWrite` (which sorts the IDs
internally, so it stays deadlock-safe if the job system is later made
multi-threaded). Convention: **body1 = target/world (parent), body2 = self.**

### Destruction — the cross-entity cleanup

Jolt requires a joint be removed before either body it references is destroyed.
The trap: a constraint stored on entity A may reference entity B, and deleting B
never fires A's `on_destroy` hook. So `PhysicsSystem` keeps a reverse index,
`bodyToConstraints` (bodyID → constraint ids), populated under **both** bodies at
creation. Body teardown (`OnRbDestroyed` / `OnColDestroyed`) calls
`RemoveConstraintsTouching(bodyId)` *before* removing the body, so destroying
either endpoint tears the joint out first.

```cpp
struct Impl {
    std::unordered_map<uint32_t, JPH::Ref<JPH::Constraint>> constraintMap;     // id → live joint
    std::unordered_map<uint32_t, std::vector<uint32_t>>     bodyToConstraints; // bodyID → ids
    std::vector<entt::entity>                               pendingConstraints;
    uint32_t                                               nextConstraintId = 0;
};
```

This `bodyToConstraints` index is the one piece of machinery with no body-era
equivalent — bodies never needed to know "who refers to me."

### Limits

When `hasLimits` is set, the hinge swing is clamped to `[limitMin, limitMax]`.
Authored in degrees; converted to radians and clamped to Jolt's required ranges
(`mLimitsMin ∈ [-π, 0]`, `mLimitsMax ∈ [0, π]`) at creation. The zero angle is the
bodies' relative orientation at creation time, *not* world vertical. Limits are
currently **hard** stops (a sharp halt); soft spring limits are on the roadmap.

### Authoring notes

- The body must start **offset from the anchor** for a hinge to do anything — an
  anchor on the centre of mass gives zero lever arm. For a *gravity-driven* swing
  the offset must also be **perpendicular to gravity** (the axis horizontal);
  an anchor directly above or below the COM is the stable equilibrium and will not
  move. A vertical-axis "door" needs a **motor** (roadmap), not gravity.
- The offset must be perpendicular to the hinge axis; an offset *along* the axis
  has no lever arm.

### Editor + persistence

The inspector exposes type, target (a dropdown of "World" + scene entities),
anchor, axis, and limits. The serializer round-trips every field except the
runtime `_constraintId`, resolving `targetUuid` lazily during the deferred
creation pass. Entity duplication copies the component (resetting `_constraintId`
so the copy builds its own joint); a duplicated entity-to-entity joint keeps the
original `targetUuid`, so the copy connects to the same target.

### Roadmap

| Feature | Notes |
|---|---|
| **Motors** | Drive a joint toward a target velocity or angle with a capped torque/force (`JPH::MotorSettings`). Unlocks powered wheels, servos, and the gravity-independent swinging door. The cap is *why* a motor can stall under load. |
| **Soft limits** | `mLimitsSpringSettings` — the joint overshoots and springs back instead of a hard stop. Smoother and gentler on the solver. |
| **More types** | Point (ball-socket), Fixed (weld), Slider (piston), Distance (rope/rod). Each is a new branch in `BuildConstraint` plus the matching authoring fields; Point/Fixed need no axis. |
| **Joint visualization** | Draw the anchor, axis, and limit wedge with `DebugDraw`. Authoring is currently "blind"; this is the highest-value workflow add. |
| **Duplicate target-remapping** | When a *connected pair* is duplicated together, remap the copy's `targetUuid` to the copied target (build an old→new UUID map across the selection) so the copies wire to each other, not the originals. |
| **Breakable joints** | Remove the constraint when its applied impulse exceeds a threshold this step. |

Each new behaviour slots into the same groove the hinge established: an optional
data field on `ConstraintComponent`, applied in `BuildConstraint`, surfaced in the
inspector, round-tripped by the serializer. The cross-entity lifecycle (deferred
creation, `bodyToConstraints` cleanup, UUID targeting) is type-agnostic and is
reused as-is.

---

## Collision Layers

Defined in the private header `PhysicsLayers.h`:

| Object Layer | Body types | Collides with |
|---|---|---|
| `STATIC`  | Static bodies | DYNAMIC |
| `DYNAMIC` | Dynamic + Kinematic | STATIC, DYNAMIC |
| `TRIGGER` | Trigger sensors | DYNAMIC |

Broad phase layers (`BP_NON_MOVING`, `BP_MOVING`) map to non-moving and moving object layers respectively. This is the standard Jolt two-layer setup from the hello world sample.

Custom user layers are deferred. The layer table is defined in one place — extending it later is straightforward.

---

## Query API

Physics queries live on `PhysicsSystem` (accessible via a scene-level free function wrapper):

```cpp
struct RaycastHit {
    entt::entity entity;
    glm::vec3    point;
    glm::vec3    normal;
    float        distance;
};

std::optional<RaycastHit> Raycast(glm::vec3 origin, glm::vec3 direction, float maxDist);
std::vector<entt::entity> OverlapSphere(glm::vec3 center, float radius);
std::vector<entt::entity> OverlapBox(glm::vec3 center, glm::vec3 halfExtents, glm::quat rotation);
```

The `Scene` class gains thin wrappers that forward to `PhysicsSystem` (which Scene can optionally hold a pointer to when play mode is active).

---

## Multithreading Upgrade Path

The only Jolt-side change needed:

```cpp
// Single-threaded (now):
m_jobSystem = std::make_unique<JPH::JobSystemSingleThreaded>(JPH::cMaxPhysicsJobs);

// Multi-threaded (later):
m_jobSystem = std::make_unique<JPH::JobSystemThreadPool>(
    JPH::cMaxPhysicsJobs, JPH::cMaxPhysicsBarriers, threadCount
);
```

`m_jobSystem` is typed as `std::unique_ptr<JPH::JobSystem>`, so nothing else changes. Both implementations satisfy the same interface.

`TempAllocatorImpl` is already thread-safe. `ContactListener` queues will need a `std::mutex` guard around the event queue — add `std::lock_guard` on enqueue and drain.

---

## File Layout

```
Engine/
  include/
    Physics/
      PhysicsSystem.h       -- PhysicsSystem : GameSystem
      PhysicsComponents.h   -- RigidBodyComponent, ColliderComponent, CollisionContact
      PhysicsMaterial.h     -- PhysicsMaterial struct + JSON load/save
      PhysicsTypes.h        -- BodyType, CollisionShape enums

  src/
    Physics/
      PhysicsSystem.cpp
      ContactListener.h     -- private, implements JPH::ContactListener
      ContactListener.cpp
      PhysicsLayers.h       -- private, layer constants + filter implementations
      PhysicsUtils.h        -- private, GLM ↔ Jolt conversion helpers
```

---

## What Is Out of Scope (For Now)

- **Debug visualization** — Jolt's `DebugRenderer` interface can draw wireframe shapes in the editor viewport. Collider wireframes are now drawn via the engine's own `DebugDraw`; constraint anchor/axis/limit visualization is still deferred (see Constraints → Roadmap).
- **Compound shapes** — multiple colliders per entity (Jolt `CompoundShape`). Deferred.
- **Joints and constraints** — see the [Constraints (Joints)](#constraints-joints) section. The hinge is implemented (with entity-to-entity targeting and angular limits); motors, soft springs, and the other joint types are on the roadmap there.
- **Character controller** — Jolt has `CharacterVirtual`; a natural follow-on after rigid bodies work.
- **Custom collision layers** — the two-layer setup covers all use cases for now.
- **ConvexHull / TriangleMesh building** — requires feeding vertex data from the mesh asset pipeline; depends on Milestone 3 asset registry.
