# Undo / Redo System Design

## Goals

- `Ctrl+Z` / `Ctrl+Y` undo and redo any editor action
- Works across the Hierarchy, Inspector, and Content Browser panels
- One undo per logical gesture (not per frame for continuous drags)
- Stack clears on save so history never refers to a stale scene state
- Multi-action operations (e.g. delete 5 entities) collapse into one undo step

---

## Architecture

### Command Base

```cpp
class Command {
public:
    virtual ~Command() = default;
    virtual void Execute() = 0;
    virtual void Undo() = 0;
};
```

Non-templated, purely polymorphic. All commands are heap-allocated and owned by `CommandStack` via `unique_ptr`.

### CommandStack

```cpp
class CommandStack {
    static constexpr int kMaxDepth = 25;
    std::deque<std::unique_ptr<Command>> m_Undo;
    std::deque<std::unique_ptr<Command>> m_Redo;
public:
    void Push(std::unique_ptr<Command>);  // Execute() + clear redo stack
    void Undo();
    void Redo();
    void Clear();  // called on scene save
};
```

- Owned by `EditorLayer`, passed by pointer/reference to each panel constructor.
- `Push` calls `Execute()` on the command before pushing — callers do not call `Execute()` themselves.
- When `m_Undo` exceeds `kMaxDepth`, the oldest entry is popped from the front.
- Any `Push` while the redo stack is non-empty clears the redo stack (standard behavior).

### Keyboard Handling

Handled in `EditorLayer::OnImGuiRender()`:

```cpp
if (ImGui::IsKeyChordPressed(ImGuiMod_Ctrl | ImGuiKey_Z))
    m_CommandStack.Undo();
if (ImGui::IsKeyChordPressed(ImGuiMod_Ctrl | ImGuiKey_Y))
    m_CommandStack.Redo();
```

---

## ValueChangeCommand (Templated)

Used for all continuous inspector edits (drag sliders, color pickers, float inputs, etc.).

```cpp
template<typename T>
class ValueChangeCommand : public Command {
    std::function<void(T)> m_Setter;
    T m_OldValue;
    T m_NewValue;
public:
    ValueChangeCommand(std::function<void(T)> setter, T oldVal, T newVal)
        : m_Setter(std::move(setter)), m_OldValue(oldVal), m_NewValue(newVal) {}
    void Execute() override { m_Setter(m_NewValue); }
    void Undo()    override { m_Setter(m_OldValue); }
};
```

The setter is a lambda that captures entity UUID and looks up the live entity handle — **never stores a raw `entt::entity`** directly, since handles are invalidated by delete/restore cycles.

Example setter construction:
```cpp
auto setter = [scene, uuid](glm::vec3 v) {
    auto e = scene->FindByUUID(uuid);
    scene->GetRegistry().get<TransformComponent>(e).position = v;
};
```

### Capturing the drag gesture

Since only one ImGui widget is active at a time, one `T m_DragOldValue` member per panel is sufficient:

```cpp
// In InspectorPanel draw code (per-field):
ImGui::DragFloat3("Position", &tc.position.x);
if (ImGui::IsItemActivated())
    m_DragOld_Position = tc.position;        // snapshot before drag
if (ImGui::IsItemDeactivatedAfterEdit())
    m_Stack->Push(make_unique<ValueChangeCommand<glm::vec3>>(setter, m_DragOld_Position, tc.position));
```

---

## Entity Identity

`entt::entity` handles are recycled — they are **not** stable across delete/restore cycles. All commands refer to entities by UUID (`uint64_t` from `IDComponent`).

### Required: `Scene::FindByUUID`

```cpp
entt::entity Scene::FindByUUID(uint64_t uuid) const;
```

Implemented via a `std::unordered_map<uint64_t, entt::entity>` maintained on `Scene`. The map is updated in `CreateEntity` and `DestroyEntity`. This is O(1) and avoids linear registry scans.

---

## Concrete Commands

### Structural (entity lifecycle)

| Command | Execute | Undo |
|---------|---------|------|
| `AddEntityCommand` | `scene->CreateEntity(snapshot)` | `scene->DestroyEntity(uuid)` |
| `DeleteEntityCommand` | `scene->DestroyEntity(uuid)` | `scene->RestoreEntity(snapshot)` |
| `ReparentCommand` | set new parent | restore old parent |
| `RenameEntityCommand` | `scene->SetName(uuid, newName)` | `scene->SetName(uuid, oldName)` |

`DeleteEntityCommand` serializes the full entity state to JSON at the moment of deletion (using the existing `SceneSerializer` logic). `Undo` deserializes it back, restoring the same UUID so any cross-references remain valid.

### Component lifecycle

| Command | Execute | Undo |
|---------|---------|------|
| `AddComponentCommand<T>` | `registry.emplace<T>(e)` | `registry.remove<T>(e)` |
| `RemoveComponentCommand<T>` | `registry.remove<T>(e)` | `registry.emplace<T>(e, snapshot)` |

`RemoveComponentCommand` copies the component value before removal so it can be restored.

### Content Browser

| Command | Execute | Undo |
|---------|---------|------|
| `MoveFileCommand` | `fs::rename(src, dest)` | `fs::rename(dest, src)` |
| `RenameFileCommand` | `fs::rename(oldPath, newPath)` | `fs::rename(newPath, oldPath)` |

These are filesystem operations — they do **not** need UUID lookup.

---

## MacroCommand

Wraps multiple commands into a single undo step. Used whenever a user action affects more than one entity or component at once (e.g. delete selected entities, paste a group).

```cpp
class MacroCommand : public Command {
    std::vector<std::unique_ptr<Command>> m_Commands;
public:
    void Add(std::unique_ptr<Command> cmd) { m_Commands.push_back(std::move(cmd)); }
    void Execute() override { for (auto& c : m_Commands) c->Execute(); }
    void Undo()    override { for (auto it = m_Commands.rbegin(); it != m_Commands.rend(); ++it) (*it)->Undo(); }
};
```

Undo iterates in **reverse** so the commands unwind in the correct order.

---

## Integration Points

| Panel | Change needed |
|-------|---------------|
| `EditorLayer` | Own `CommandStack`; handle `Ctrl+Z`/`Y`; call `m_Stack.Clear()` on save |
| `InspectorPanel` | Add `m_DragOld*` members per draggable field; push `ValueChangeCommand` on `IsItemDeactivatedAfterEdit` |
| `HierarchyPanel` | Push structural commands for add/delete/reparent/rename |
| `ContentPanel` | Push `MoveFileCommand` / `RenameFileCommand` instead of applying directly |
| `Scene` | Add `FindByUUID` with UUID→entity map; add `RestoreEntity(json)` |

---

## Console Panel

A reusable `ConsolePanel` that displays timestamped log messages. The undo/redo system will print a line whenever an action is undone or redone so the user always knows what changed.

### Design

```cpp
struct LogMessage {
    std::string text;
    // expandable later: severity, timestamp, source
};

class ConsolePanel : public Panel {
public:
    void OnImGuiRender() override;
    void Log(const std::string& msg);
    void Clear();
private:
    std::vector<LogMessage> m_Messages;
    bool m_ScrollToBottom = true;
};
```

`ConsolePanel` is owned by `EditorLayer` alongside the other panels. Any system that wants to log passes a pointer to it (or it can be a lightweight global since it holds no scene state).

### Undo/Redo integration

`Command` gets an optional description string:

```cpp
virtual std::string GetDescription() const { return ""; }
```

`CommandStack::Undo()` and `Redo()` call `m_Console->Log(...)` after applying the command:

```cpp
void CommandStack::Undo() {
    // ...apply undo...
    if (m_Console)
        m_Console->Log("Undo: " + cmd->GetDescription());
}
```

Example output:
```
Undo: Move "Sword" → Models/
Redo: Delete entity "PointLight_3"
Undo: Rotate "Player" (0°, 45°, 0°)
```

### Reuse

The same panel can later receive messages from the asset pipeline, scene serializer, script errors, and physics warnings — making it the single observability surface in the editor.

---

## What Is Out of Scope (for now)

- Per-field undo (undo only the last slider change, not the whole gesture)
- Redo across save/load (stack is intentionally cleared on save)
- Undo of viewport camera movement
