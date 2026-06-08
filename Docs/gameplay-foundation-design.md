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
A script is a component with behavior. It attaches to an entity, has lifecycle methods, and can hold private state. Shared/structural data belongs in ECS components; private behavior state belongs on the script.

```
Entity (ID)
├── TransformComponent     ← shared, queryable by any system
├── MeshComponent          ← shared
├── HealthComponent        ← shared, so other systems can query it
└── ScriptComponent        ← holds a NativeScript (private state + behavior)
```

### NativeScript Base Class

```cpp
class NativeScript {
public:
    virtual void OnStart()          {}
    virtual void OnUpdate(float dt) {}
    virtual void OnDestroy()        {}

protected:
    Entity  entity;   // handle to the owning entity
    Scene*  scene;    // access to spawn, destroy, and query components
};
```

### ScriptComponent

```cpp
struct ScriptComponent {
    std::unique_ptr<NativeScript> instance;
};
```

One `ScriptComponent` per entity. Multiple behaviors on one entity require either one script that composes behavior, or multiple `ScriptComponent`-style slots — decide when the need arises.

### Attaching Scripts
Always go through a scene-level API, never raw EnTT calls:

```cpp
// Code attachment
scene.AddScript<PlayerController>(entity);

// Removal
scene.RemoveScript(entity);
```

The editor will present a registered script dropdown in the Inspector, calling the same API internally.

### Writing a Script

```cpp
class PlayerController : public NativeScript {
public:
    void OnStart() override {
        m_speed = 5.0f;
    }

    void OnUpdate(float dt) override {
        auto& transform = scene->Get<TransformComponent>(entity);
        float x = Input::GetAxis("MoveX");
        transform.position.x += x * m_speed * dt;

        if (Input::IsPressed("Fire")) {
            scene->SpawnEntity(...);
        }
    }

    void OnDestroy() override {}

private:
    float m_speed;
};
```

### Spawning and Destroying Entities
Scripts have full access to the scene to spawn and destroy entities. This is intentional — guns spawning projectiles, enemies dying and spawning loot, etc.

```cpp
Entity bullet = scene->CreateEntity("Bullet");
scene->Add<TransformComponent>(bullet, ...);
scene->AddScript<BulletScript>(bullet);

scene->DestroyEntity(enemy);
```

### Performance Note
Script objects are heap-allocated and pointer-chased on iteration. This is acceptable because gameplay script counts are low (50–200 entities). The ECS cache benefits — tight contiguous pools — still apply fully to `TransformComponent`, `PhysicsComponent`, `MeshComponent`, etc. Scripts orchestrate those components; they are not the hot path.

### Registration (for Editor)
User-defined scripts must be registered so the editor can list them:

```cpp
ScriptRegistry::Register<PlayerController>("PlayerController");
ScriptRegistry::Register<BulletScript>("BulletScript");
```

---

## 3. Collision + Physics (Jolt)

### Scope — Phase 1
Phase 1 covers collision queries and trigger volumes only. Rigid body simulation (crates falling, ragdolls, forces) is deferred to Phase 2.

Phase 1 deliverables:
- Raycasts (`scene->Raycast(origin, direction, maxDist)`)
- Overlap queries (`scene->OverlapSphere(center, radius)`)
- Trigger volumes — `OnTriggerEnter / OnTriggerExit` called on scripts
- Static colliders for world geometry

Phase 2 (later):
- `RigidBodyComponent` — mass, velocity, forces, constraints
- Kinematic character controller

### PhysicsComponent (ECS)
Physics is an optional component — not every entity needs it. `TransformComponent` remains the source of truth for position/rotation; physics writes back to it each frame.

```
Entity
├── TransformComponent    ← always present, owns the canonical position
└── ColliderComponent     ← optional, shape + trigger flag
    └── (RigidBodyComponent added in Phase 2)
```

```cpp
struct ColliderComponent {
    ColliderShape shape;   // Box, Sphere, Capsule, Mesh
    glm::vec3     offset;
    bool          isTrigger;
};
```

### Jolt Integration
Jolt runs its own internal body representation. Each frame:
1. Jolt simulation steps (Phase 2 only in Phase 1 this is a no-op)
2. Jolt body transforms are written back to `TransformComponent`

Jolt bodies are created/destroyed when `ColliderComponent` is added/removed from an entity (EnTT `on_construct` / `on_destroy` listeners).

### Raycasts and Queries from Scripts

```cpp
void GunScript::OnUpdate(float dt) {
    if (Input::IsPressed("Fire")) {
        auto hit = scene->Raycast(muzzlePos, forward, 100.0f);
        if (hit && scene->Has<HealthComponent>(hit.entity)) {
            scene->Get<HealthComponent>(hit.entity).value -= 25.0f;
        }
    }
}
```

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
