# Scene System Design

## Overview

This document captures the architectural decisions for DiamondEngine's scene and entity system. The goal is a CPU-friendly ECS-backed scene that integrates cleanly with the existing RenderGraph and editor panels.

---

## Core Decisions

### Entity Representation

Entities are raw `entt::entity` handles inside the renderer and engine hot paths. A thin `Entity` wrapper (`handle + Scene*`) lives in `Sandbox/Editor/` for use in editor and scripting code only. The wrapper adds ergonomic helpers (`Get<T>()`, `Has<T>()`, `IsValid()`) without touching data layout or cache behavior.

The wrapper is **not** in `Engine/` — engine code uses raw handles to keep renderer iteration clean.

### Stable Identity (UUID)

`entt::entity` integers are recycled and meaningless across save/load sessions. Every entity gets an `IDComponent` holding a `uint64_t` UUID, generated at creation from `std::mt19937_64` seeded with `std::chrono::steady_clock`. This UUID is stable across sessions and is the canonical cross-reference key for serialization.

```cpp
struct IDComponent {
    uint64_t uuid;
};
```

### Entity Names

Names are editor metadata, not game data. They are stored in a `std::unordered_map<entt::entity, std::string>` on `Scene` rather than as a `NameComponent` in the registry. This keeps names completely out of component iteration hot paths.

### TransformComponent

Rotation is stored as `glm::quat` internally. The inspector converts to Euler angles on demand for display — this is the industry standard to avoid gimbal lock and lossiness from editing Euler values directly.

```cpp
struct TransformComponent {
    glm::vec3 position { 0, 0, 0 };
    glm::quat rotation { 1, 0, 0, 0 };  // identity
    glm::vec3 scale    { 1, 1, 1 };

    glm::mat4 GetLocalMatrix() const;
    glm::vec3 GetEulerAngles() const;  // inspector only
};
```

### Parent-Child Hierarchy

**Deferred.** The scene is flat for this iteration. When added, the hierarchy will be stored as a separate tree structure on `Scene` (a `std::unordered_map<entt::entity, std::vector<entt::entity>>`) rather than as components on entities. This keeps hierarchy traversal simple for UI and avoids polluting the component registry with structural metadata.

---

## Scene Class

Lives in `Engine/` so both the runtime and the editor can use it. Owns the registry, name map, and UUID index.

**Responsibilities:**
- Create and destroy entities (generating UUID and name at creation)
- Expose the `entt::registry` for system and renderer access
- Produce a `DrawList` for the `RenderGraph` to consume

**Does not own:**
- Render resources (meshes, textures, shaders — those live in the renderer)
- Editor state (selection, gizmo mode — those live in `EditorContext`)

---

## Render Integration

`Scene` produces a `DrawList` — a plain array of `{ mesh, material, worldMatrix }` POD structs. The `RenderGraph` consumes this list without knowing anything about ECS or components. Neither system depends on the other's internals.

This keeps the renderer generic and reusable and keeps scene iteration (culling, sorting, batching) inside `Scene::BuildDrawList()` where it has direct access to component data.

---

## Editor Context

`EditorContext` is a plain struct owned by `EditorLayer` and passed by pointer to every panel. It holds editor-only transient state that is not part of the scene or renderer.

```cpp
struct EditorContext {
    Scene*       activeScene    = nullptr;
    entt::entity selectedEntity = entt::null;
};
```

Panels read and write `EditorContext` — the Hierarchy sets `selectedEntity`, the Inspector reads it.

As the editor grows (multi-selection, undo/redo, clipboard, play/pause mode, gizmo state), those fields accumulate on `EditorContext` rather than on `EditorLayer` directly, preventing the layer from becoming a god object.

---

## Inspector Reflection

Components are registered into a callback table at startup. The inspector iterates the table, checks if the selected entity has each component, and calls the draw function if so.

```cpp
// Registration (once at startup):
ComponentRegistry::Register<TransformComponent>("Transform", DrawTransformUI);

// Inspector loop:
for (auto& entry : ComponentRegistry::All())
    if (entry.has(registry, selectedEntity))
        entry.draw(registry, selectedEntity);
```

EnTT's meta/reflection system is not used for now — it adds significant boilerplate and the callback table covers the only current consumer (the inspector). If serialization or scripting later need the same type data, migration to EnTT meta is straightforward.

---

## File Layout

```
Engine/
  include/
    Scene/
      Scene.h          -- registry, name map, CreateEntity/DestroyEntity, BuildDrawList
      Components.h     -- TransformComponent, IDComponent

  src/
    Scene/
      Scene.cpp
      UUIDGenerator.cpp  -- private, not exposed in include/

Sandbox/
  src/
    Editor/
      EditorContext.h        -- selectedEntity, activeScene*
      Entity.h               -- thin wrapper (handle + Scene*), editor-only
      Panels/
        ComponentRegistry.h  -- callback registration table
```

---

## What Is Explicitly Out of Scope

- Parent-child transform propagation (deferred)
- Scene serialization / deserialization (UUIDs are ready for it, but the serializer is not)
- EnTT meta / full reflection (callback table is sufficient for now)
- Script/behavior components
- Multiple scenes / scene streaming
