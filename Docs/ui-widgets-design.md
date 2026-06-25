# UI Widget Components Design

Part of **Milestone 5 — Game Layer**, building on the UI foundation (glyph-atlas
`Font` + backend-agnostic `Renderer2D` batcher) and the canvas/anchor layer
(`CanvasComponent`, `RectTransformComponent`, `UISystem`). This step attaches the
**widget components** that actually paint something into a resolved rect: Image,
Text, Progress Bar, and Button.

---

## Architecture overview (the whole system at a glance)

The UI is a **retained-mode, ECS-integrated, backend-agnostic** layer that borrows
the engine's existing bones rather than inventing parallel ones. It stacks in
three layers, each depending only on the one below:

**1. Rendering foundation — knows nothing about UI.**
- `Font` ([Engine/include/Renderer/Font.h]) bakes a TTF into a glyph-atlas texture
  (stb_truetype).
- `Renderer2D` ([Engine/include/Renderer/Renderer2D.h]) batches quads + text in
  pixel space. `Renderer2D::Create()` dispatches on the active backend exactly like
  `Shader`/`Mesh`/`Texture`, so a Vulkan backend brings the whole UI stack with it.

**2. UI data + logic — the ECS layer.**
- Components (pure data, in [Engine/include/Scene/Components.h] beside the 3D
  components): `CanvasComponent`, `RectTransformComponent`, `UIImageComponent`,
  `UITextComponent`, `UIProgressBarComponent`, `UIButtonComponent`.
- Systems (stateless functions over the registry):
  `UISystem::Resolve` (anchors → pixel rects), `UIInputSystem::Update` (pointer
  hit-test → button state + `onClick`), `UIRenderSystem::Render` (draw each widget
  through `Renderer2D` in z-order).

**3. Editor integration.** Serialization (assets by path), inspector sections with
drag-drop slots + undo, the content-browser `Font` asset type, and the
viewport→UI mouse mapping in `EditorContext`.

### Per-frame flow (play mode)
The UI is a thin layer that runs **after** the existing 3D pipeline, into the same
framebuffer:

```text
[existing 3D pipeline]  → renders scene into gameViewportFBO
  → UISystem::Resolve(scene registry, screenSize)   // compute pixel rects
  → UIInputSystem::Update(registry, pointer, down)   // hover / press / click
  → UIRenderSystem::Render(registry, renderer2D)     // draw on top (depth-off, alpha-blend)
```

It composites over the 3D frame in the **same FBO** — no separate render target or
pass.

### Seams with existing systems
| Existing system | How the UI plugs in |
|---|---|
| EnTT / Scene | UI entities are plain scene entities; reuse the **same `HierarchyComponent`** as 3D transforms — no separate UI tree. A Canvas is implicitly full-screen (no RectTransform). |
| Render pipeline / FBO | Draws into `gameViewportFBO` after `cullAndExecute`, next to the deferred/bloom/FXAA passes. |
| Serialization | Widgets round-trip like any component; textures/fonts serialize **by path** (mesh/material convention). `onClick` is never saved. |
| Inspector / ContentPanel / Command | Inspector sections, drag-drop asset slots, and undo via the existing `ValueChangeCommand`/`FunctionCommand`. Fonts are a first-class `AssetType`. |
| Input | `UIInputSystem` takes a pointer **already in UI-space**; the editor maps the OS mouse via `EditorContext::viewportMouseNorm` / `gameViewportMouseNorm`. Gamepad focus will plug in here as a synthetic cursor. |
| Scripting | Behavior lives in scripts: `UI::SetButtonCallback(scene, "Name", fn)` from a `GameSystem::OnStart`. Scene data stays declarative. |

---

## The core idea

`UISystem::Resolve` already walks every canvas top-down and writes each
`RectTransformComponent::resolvedPos / resolvedSize` in screen pixels. Today
nothing consumes those rects — `main.cpp` hand-draws demo quads. The missing
piece is a **`UIRenderSystem`**: after `Resolve`, iterate the widgets in draw
order and dispatch one `Renderer2D` call per component. That dispatch loop is what
turns "structs on entities" into actual ECS widgets, so it is the **first** task,
not the last.

Widgets are **data-only components**; a widget entity carries a
`RectTransformComponent` (where) plus one or more widget components (what to
paint). Composition is first-class — a button is just an entity that happens to
have an image, a text, *and* a button-behavior component.

---

## Draw order

`UIRenderSystem::Render(reg, renderer2d, screenSize)` runs after
`UISystem::Resolve`:

1. Collect canvases, sort by `CanvasComponent::sortOrder`.
2. Within a canvas, collect descendant widget entities and sort by
   `RectTransformComponent::zOrder` (stable, so hierarchy/sibling order breaks
   ties).
3. For each entity, draw its widget components **back-to-front**: Image →
   Progress Bar → Text. (A button's background image draws before its label, so
   composition gives correct layering for free.)
4. One `Renderer2D::Begin/End` pair wraps the whole UI pass with the ortho
   projection from `Renderer2D::OrthoProjection(w, h)`.

Everything batches through the existing `Renderer2D` — no per-widget shaders.

---

## Components

### `UIImageComponent` — trivial
The simplest widget, and it needs no new rendering code.

```
shared_ptr<Texture> texture;   // null => draw a solid quad (white image default)
glm::vec4           tint {1};  // straight RGBA: rgb = color multiply, a = opacity
glm::vec2           uvMin {0}, uvMax {1};
```

- `tint` *is* the user's "opacity + RGB tweak" — alpha is opacity, rgb multiplies.
- `texture == null` → `DrawQuad(rect, tint)`; otherwise
  `DrawTexturedQuad(pos, size, *texture, tint, uvMin, uvMax)` — both already exist.

### `UITextComponent`
```
std::string text;
shared_ptr<Font> font;
float            sizePx;          // target pixel height; scale = sizePx / font baked height
glm::vec4        color {1};
enum class HAlign { Left, Center, Right } hAlign = Left;
enum class VAlign { Top, Middle, Bottom } vAlign = Top;
float            lineSpacing = 1.0f;
bool             wrap        = true;   // word-wrap to the rect width
bool             underline   = false;
```

- **In scope now:** H/V alignment within the resolved rect (via `Font::Measure`),
  word-wrap to box width, and underline (a `DrawQuad` line at the baseline +
  descent under each laid-out line).
- **Deliberately skipped:** bold & italic. `Font` bakes a single TTF face at one
  pixel height; styling would mean separate font files or faux-styling. Revisit
  only if a real need shows up.
- Wrapping/alignment likely need a small layout helper (break `text` into lines
  that fit `resolvedSize.x`, then position each line) feeding `Renderer2D::DrawText`.

### `UIProgressBarComponent` — two quads, no shader
```
float     progress {0..1};
glm::vec4 backgroundColor;
glm::vec4 fillColor;
shared_ptr<Texture> fillTexture;   // optional; UV sub-rect clips instead of squashing
enum class Direction { LeftToRight, RightToLeft, BottomToTop, TopToBottom };
```

Draw the background quad over the full rect, then a fill quad whose extent along
`Direction` is `progress * fullExtent`. A textured fill passes a `uvMin/uvMax`
sub-rect so the texture **clips** rather than stretches. No fragment shader —
that would break batching and buys nothing for a linear fill. (A radial/circular
fill *would* justify a shader; out of scope.)

### `UIButtonComponent` — behavior only, composed with Image + Text
A button entity carries `UIImageComponent` (background, default white) +
`UITextComponent` (label) + `UIButtonComponent`, which holds **only interaction
state**, not its own texture/string:

```
glm::vec4 normalTint, hoverTint, pressedTint;   // applied to the sibling image
std::function<void()> onClick;                  // simple callback (see below)
// runtime state:
enum class State { Normal, Hover, Pressed } state;
```

- **Rendering** is just the Image + Text the entity already has; the button only
  swaps the image's effective tint by `state`.
- **Interaction** is a separate pointer pass (hit-test the pointer against
  `resolvedPos/resolvedSize`, track press→release-inside) that sets `state` and
  fires `onClick` on a completed click.
- **Dispatch:** start with a `std::function<void()>` callback (or a polled
  "clicked this frame" flag). The deferred general-purpose UI **event bus**
  (roadmap issue #27) is *not* a prerequisite — wire it in later if needed.
- The future **gamepad-focus** milestone plugs into this same pass (focused
  widget = synthetic hover/press), so input stays in one place.

---

## Editor / inspector

Each new component needs an inspector section in `InspectorPanel.cpp` and
serialization in `SceneSerializer.cpp` — including "assign a texture in the
inspector," which is an inspector concern, not a component one.

**Button presentation:** even though a button is three components under the hood,
the inspector should present image + text + button fields **together under a
single "Button" section** so it reads as one widget, rather than three unrelated
component panels. (Implementation detail for the inspector step; the ECS data
stays composed.)

---

## Build order

1. **`UIRenderSystem` + `UIImageComponent`** ✓ — proves the dispatch loop
   end-to-end with the easiest widget; replaces the hand-drawn demo in `main.cpp`.
2. **`UITextComponent`** ✓ — alignment + word-wrap + underline (no bold/italic).
3. **`UIProgressBarComponent`** ✓ — two quads; falls out of step 1.
4. **`UIButtonComponent`** ✓ — pointer hit-test pass (`UIInputSystem`) + callback
   dispatch; the only widget needing input.
5. **Serializer + inspector** wiring as each widget lands (button fields grouped). ✓
   Texture/font assets round-trip by **path** (`texturePath`/`fillTexturePath`/
   `fontPath` on the components; `LoadCached`/`LoadFontCached` rehydrate). Fonts
   bake at the shared `kUIFontBakeHeight`. `onClick` is **not serialized** —
   assigned from scripts via `UI::SetButtonCallback(scene, "Name", fn)`. The
   inspector folds a button's image + text + tints under one "Button" header
   (standalone Image/Text sections hide when a button is present). MSVC needs
   `/bigobj` for `InspectorPanel.cpp` after this.

   **Known gap (next step):** UI authored on *scene* entities isn't rendered yet —
   `main.cpp` still drives only the standalone `uiDemo` registry. Wiring
   `UISystem::Resolve` + `UIRenderSystem::Render` (+ `UIInputSystem::Update`) over
   the scene registry (ideally into the game-viewport FBO) is what makes
   inspector-authored UI actually appear.

### Input plumbing (step 4 note)
`UIInputSystem::Update(reg, pointer, pointerDown)` takes the pointer **already in
UI-pixel space** — it is agnostic to where that came from (mouse today, gamepad
virtual cursor later). The editor demo maps the OS mouse into the full-window UI
framebuffer via `EditorContext::viewportMouseNorm` (written by `ViewportPanel`,
read in `main.cpp`), because the UI renders into the Scene-viewport FBO at full
window resolution but is displayed scaled inside an ImGui panel.

### Decisions locked
| Question | Decision |
|----------|----------|
| Bold / italic | **Skipped** — single baked face; revisit on real need. |
| Word-wrap | **In scope now** (not deferred). |
| Underline | **In scope now.** |
| Button structure | **Composition** — Image + Text + behavior-only component. |
| Button inspector | All three shown **together** under one "Button" section. |
| Progress bar fill | **Two quads** (+ optional UV-clipped texture), **no shader**. |
| Button events | **Callback / polled flag** first; event bus (#27) deferred. |
