# Gameplay Foundation Design

Four systems that take DiamondEngine from a renderer/editor to a game engine: Input, Scripting, Collision + Physics, and Play Mode.

---

## 1. Input System

### Philosophy
Named actions mapped to keys — scripts never call GLFW directly. Bindings live in one place; scripts call semantically meaningful names.

### Action States
Every named action exposes three boolean queries and one float axis:

```cpp
Input::IsPressed("Jump")   // true only on the frame the key went down
Input::IsHeld("Fire")      // true every frame the key is held
Input::IsReleased("Fire")  // true only on the frame the key came up
Input::GetAxis("MoveX")    // returns [-1, 1] float (WASD or thumbstick)
```

State is diffed each frame after `glfwPollEvents()`:
- GLFW event callbacks write into a "current frame" state table
- At the end of each frame, current state is copied to "last frame" state
- `IsPressed` = current down AND NOT last down
- `IsReleased` = NOT current down AND last down

This guarantees no missed presses even at low framerates, since GLFW queues all events between polls.

### Binding Map
Bindings are defined once, either in code or loaded from a config file:

```cpp
Input::BindAction("Jump",   Key::Space);
Input::BindAction("Fire",   Key::MouseLeft);
Input::BindAxis("MoveX",    Key::D, Key::A);   // positive key, negative key
Input::BindAxis("MoveY",    Key::W, Key::S);
```

### Variable Framerate Note
Axis values are snapshots — they are framerate-independent by nature. Scripts are responsible for multiplying axis-driven movement by `dt`:

```cpp
void PlayerScript::OnUpdate(float dt) {
    float x = Input::GetAxis("MoveX");
    transform.position.x += x * m_speed * dt;
}
```

### Direct Polling vs Callbacks — Decision
Scripts poll directly. Callbacks were considered but rejected: execution order is implicit, callbacks can outlive their owning object, and polling is easier to trace in a debugger. For UI events (button clicks), callbacks remain appropriate — this decision applies to gameplay input only.

---

## 2. Scripting

### Mental Model
Gameplay behavior is split into two distinct parts: **data** and **behavior**. Data lives in plain ECS components; behavior lives in systems that iterate those components. There is no monolithic "script object" per entity — a developer defines a component struct to hold state and a system class to act on it.

```
Entity (ID)
├── TransformComponent   ← engine component
├── MeshComponent        ← engine component
└── HealthComponent      ← user-defined data component

HealthSystem             ← user-defined system, iterates all HealthComponents
```

This keeps the full ECS cache benefit: systems iterate tight contiguous component pools with no pointer chasing or virtual dispatch per entity.

### Defining a Script (Component + System pair)
A "script" is always a paired `.h` file containing one component struct and one system class. Both live in `Sandbox/src/Scripts/`.

```cpp
// Scripts/Health.h

struct HealthComponent {
    float max     = 100.0f;
    float current = 100.0f;
    bool  isDead  = false;
};

class HealthSystem : public GameSystem {
    DECLARE_SYSTEM(HealthSystem, 300)
public:
    void OnStart(Scene& scene)            override {}
    void OnUpdate(Scene& scene, float dt) override {
        auto view = scene.View<HealthComponent>();
        for (auto [entity, health] : view.each()) {
            if (health.current <= 0.0f && !health.isDead) {
                health.isDead = true;
                scene.DestroyEntity(entity);
            }
        }
    }
    void OnDestroy(Scene& scene)          override {}
};
```

The editor exposes `HealthComponent` as an attachable component. `HealthSystem` ticks automatically — the developer never touches the application file.

### System Registration and Execution Order
Systems self-register via the `DECLARE_SYSTEM(ClassName, Priority)` macro, which creates a static initializer that inserts the system into a global sorted registry before `main` runs. Priority determines tick order; registration order is irrelevant.

Suggested priority bands:

| Priority | Purpose |
|---|---|
| 0–99 | Core engine (transform hierarchy sync, etc.) |
| 100–199 | Physics |
| 200–299 | Movement / AI |
| 300–399 | Gameplay (health, weapons, etc.) |
| 400–499 | Camera |
| 500+ | Late update |

Leave gaps between systems so new ones can be inserted without renumbering.

### Inter-System Communication
Systems communicate by reading and writing components directly, or through free functions in a named namespace:

```cpp
namespace Health {
    inline void TakeDamage(Scene& scene, entt::entity e, float amount) {
        auto& h = scene.Get<HealthComponent>(e);
        h.current = std::max(0.0f, h.current - amount);
    }
    inline void Heal(Scene& scene, entt::entity e, float amount) {
        auto& h = scene.Get<HealthComponent>(e);
        h.current = std::min(h.max, h.current + amount);
    }
}
```

```cpp
// In GunSystem::OnUpdate
if (Input::IsPressed("Fire")) {
    auto hit = scene.Raycast(muzzlePos, forward, 100.0f);
    if (hit && scene.Has<HealthComponent>(hit.entity))
        Health::TakeDamage(scene, hit.entity, 25.0f);
}
```

An event bus is deferred — free functions are sufficient and easier to trace in a debugger. The two can coexist when the need for decoupled broadcast events arises.

### Spawning and Destroying Entities
Systems have full scene access to spawn and destroy entities:

```cpp
entt::entity bullet = scene.CreateEntity("Bullet");
scene.Add<TransformComponent>(bullet, spawnTransform);
scene.Add<BulletComponent>(bullet);

scene.DestroyEntity(enemy);
```

### GameSystem Base Class

```cpp
class GameSystem {
public:
    virtual ~GameSystem() = default;
    virtual void OnStart(Scene& scene)            {}
    virtual void OnUpdate(Scene& scene, float dt) {}
    virtual void OnDestroy(Scene& scene)          {}
};
```

### Editor Integration
The editor component panel lists all registered component types. Attaching a component to an entity implicitly activates the paired system's behavior for that entity — no additional editor step required.

---

## 3. Collision + Physics (Jolt)

> **Full design moved to [`physics-design.md`](physics-design.md).** We are implementing full rigid body simulation directly (Static, Dynamic, Kinematic bodies; all collision shapes; triggers; collision callbacks; fixed timestep). The Phase 1 / Phase 2 split described below is superseded.

Summary of what is implemented:
- `RigidBodyComponent` — body type, mass, damping, gravity scale, `std::function` collision/trigger callbacks
- `ColliderComponent` — Box, Sphere, Capsule, ConvexHull, TriangleMesh shapes; trigger flag; `PhysicsMaterial`
- `PhysicsSystem` as a `GameSystem` at priority 200 (after pre-physics scripts, before post-physics scripts)
- Fixed 60 Hz step via accumulator; at most one step per frame when fps < 60
- Raycasts and overlap queries via `PhysicsSystem`

---

## 4. Play Mode

### Two Windows
The editor has two viewports:
- **Scene View** — always visible, shows editor state with gizmos, always ticks editor systems
- **Game View** — separate panel, renders the game camera, only active in Play mode

The Game View uses click-to-focus. Mouse/keyboard input only routes to the game when the Game View panel is focused (matches Unity behavior). Editor hotkeys (W/E/R gizmos, etc.) are suppressed while the Game View is focused.

### Play / Stop Lifecycle

```
[Edit Mode]
    ↓  user clicks Play
Serialize full scene snapshot to memory (JSON, same format as save)
Switch to Play mode
    → Initialize all ScriptComponents (call OnStart)
    → Initialize Jolt physics world from ColliderComponents
    → Begin ticking: scripts, physics, input

[Play Mode]
    ↓  user clicks Stop
Destroy all ScriptComponents (call OnDestroy)
Destroy Jolt physics world
Deserialize scene snapshot back into registry (full reload)
Switch to Edit mode — scene is exactly as it was before Play
```

### What Ticks Where

| System | Edit Mode | Play Mode |
|---|---|---|
| Renderer | Yes | Yes |
| Editor gizmos / ImGuizmo | Yes | No |
| Script OnUpdate | No | Yes |
| Physics simulation | No | Yes |
| Input routing to game | No | Yes |
| Undo/redo | Yes | No |

### Scene Snapshot
The snapshot uses the existing JSON serializer. It serializes the full registry state before Play starts. On Stop, the registry is cleared and reloaded from the snapshot. This means:
- Entity positions moved by physics revert
- Entities spawned by scripts are destroyed
- Any editor changes made during Play (if any UI is exposed) are discarded

### State Preservation Guarantee
The scene the user sees after Stop is byte-for-byte identical to the scene before Play. No dirty state, no partial rollback.

---

## Implementation Order

1. **Input** — no dependencies, unblocks scripting immediately
2. **Scripting** — depends on Input for meaningful test scripts
3. **Play Mode** — depends on Scripting (needs OnStart/OnUpdate/OnDestroy to be meaningful)
4. **Collision Phase 1** — can be done in parallel with Play Mode, depends on Scripting for trigger callbacks
5. **Physics Phase 2 (Rigid Body)** — after all of the above
