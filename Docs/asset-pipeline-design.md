# Asset Pipeline Design

Milestone 6: four systems that make asset loading reliable and efficient — Asset Registry, Hot Reload, Prefabs, and Texture Compression — plus a stretch fifth: Script Hot Reload via a game-code DLL.

Recommended build order: **Registry → Hot Reload → Prefabs → Compression → Script DLL**. The registry is the foundation the other loaders hang off; prefabs are independent (pure serializer/editor work) and can be built in parallel. The script DLL is the largest single item and depends on nothing else here — but do it last; it's plumbing-heavy and the asset-side items pay off sooner.

---

## 1. Asset Registry

### Problem
The engine already caches assets — in three disconnected places:

- `SceneSerializer::FromJson` builds local `texCache` / `fontCache` / `meshCache` / `skinnedCache` maps that **die when the load returns**. Anything loaded after scene load (thumbnails, material edits, a second scene) re-loads from disk.
- `MaterialLibrary::Get` (MaterialAsset.h) caches `.mat` instances via `weak_ptr` — but `ApplyMaterialJson` calls `Texture::Create` uncached, so two materials sharing an albedo upload the same texture to VRAM twice.
- `ShaderLibrary` caches shaders by name.

### Design
One engine-owned `AssetRegistry` with per-type maps of **normalized absolute path → `weak_ptr<T>`**:

```cpp
auto tex  = Assets::Load<Texture>("Assets/Textures/rock_albedo.png");
auto mat  = Assets::Load<PBRMaterial>("Assets/Materials/rock.mat");
auto mesh = Assets::Load<MeshAsset>("Assets/Models/rock.glb");
```

- **Lookup hit** → return the live `shared_ptr`. **Miss or expired** → load from disk, store a `weak_ptr`, return.
- Internally one `unordered_map<string, weak_ptr<T>>` per asset type (texture, material, mesh/model, font). No type erasure needed at first — a struct of typed maps is fine.

### Decisions
- **Key normalization.** `Assets/foo.png`, `Assets\foo.png`, and `./Assets/foo.png` must map to one entry. Use `std::filesystem::weakly_canonical` on the way in. (`NormalizeMaterialPath` exists because this bug already happened once — the registry centralizes the fix.)
- **`weak_ptr`, not `shared_ptr`.** The registry deduplicates; it does not own. Ownership stays with materials/components, so unloading a scene frees its assets naturally. `MaterialLibrary` already proves this policy works.
- **Lives in Sandbox** (`Sandbox/src/AssetPipeline/`), not Engine. *Correction: this section originally said "Lives in Engine... matching `MaterialLibrary`" — that was wrong. `MaterialLibrary` has always lived in `Sandbox/src/Editor/MaterialAsset.h`, not Engine; it was never the Engine-owned precedent this doc claimed. `ShaderLibrary` is the one genuinely Engine-owned cache (`Engine/src/Platform/OpenGL/Resources/ShaderLibrary.h`, name-keyed, renderer-owned) and stays as-is.* Since Sandbox links `MyEngine` one-directionally, an Engine-owned registry would be reachable from Engine-internal systems too; a Sandbox-owned one is not — only editor/game-shell code can call `Assets::Load<T>()`. Accepted for v1 because every current caller (`SceneSerializer`, `InspectorPanel`, `MaterialLibrary`) already lives in Sandbox; revisit if an Engine-internal system ever needs the dedup.
- **Path-keyed for now.** Asset GUIDs (stable IDs that survive file moves/renames) are a real upgrade but a separate project — the registry API shouldn't preclude them, and shouldn't wait for them.
- **v1 types: `Texture`, `PBRMaterial`, `MeshAsset` (static, via `ModelImporter`), `ImportedModel` (skinned, via `GltfImporter`).** Meshes needed a wrapper because `ModelImporter::Load` returns raw `MeshData` by value — `Assets::MeshAsset` holds the sub-mesh list behind one shared_ptr per file. `Font` is a natural add later (its loader takes a pixel-height parameter, so it needs either a fixed bake height or a composite key).
- **Shared-asset rules.** Registry assets are shared and must be treated as immutable: callers copy what they need (the Inspector's skinned-mesh drop used to `std::move` the skeleton out of its private copy — through the registry that would gut the shared asset). And because the registry holds only `weak_ptr`s while mesh CPU data is dropped right after GPU upload, a multi-entity pass (scene load, future prefab stamp) pins the loaded model shared_ptrs for the duration of the pass, replacing the old per-load cache maps.
- **Migrated (2026-07-10).** `MaterialLibrary::Get` forwards to `Assets::Load<PBRMaterial>`; `ApplyMaterialJson` loads slot textures through the registry (fixes the shared-albedo double VRAM upload); `SceneSerializer::FromJson`'s `texCache`/`meshCache`/`skinnedCache` are gone (fontCache stays); InspectorPanel drag-drops, ContentPanel mesh thumbnails, and the particle emitter texture load all route through `Assets::Load<T>`. The `PBRMaterial` specialization lives in `MaterialAsset.h` (it needs the .mat JSON loader; the registry header can't include editor code). Not migrated: `main.cpp`'s hardcoded demo-scene materials (one-shot startup code) and all `CreateFromPixels` textures (no file identity — correctly outside the registry).

### Also buys us
- Hot reload: the registry is the definitive "every live asset + its source path" list to watch.
- Compression: the registry's texture load path is the single place to check for a cooked `.dds`.
- Debug: an editor panel listing every loaded asset, its refcount, and VRAM estimate is a `for` loop.

---

## 2. Hot Reload (textures + materials)

### Why shaders already work — and what generalizes
Shader hot reload rests on two properties:

1. **Change detection**: throttled mtime polling (`ShaderLibrary::ReloadChanged`, driven from the editor backend each frame on a timer).
2. **Reload in place**: `OpenGLShader::Reload()` swaps the GL program id *inside the same object*. Every reference handed out stays valid; on compile failure the old program is kept. Nobody downstream is invalidated.

Only (2) was shader-specific, and it generalizes directly.

### Textures
Everything holds `shared_ptr<Texture>`. Add `OpenGLTexture::Reload()`: re-decode the source file, upload into the same wrapper (new GL id internally, delete old, update width/height/byte-size). Every material pointing at the texture picks it up automatically — zero invalidation logic.

Prerequisite: `OpenGLTexture` must **store its source path, flip flag, and HDR flag** (today it doesn't — `OpenGLShader` stores its paths, which is exactly what makes its `Reload()` possible). Textures created via `CreateFromPixels` (font atlases, GLB-embedded) have no file and are not watched.

On decode failure (file mid-write, deleted): keep the old texture, log, retry on next mtime change — same policy as shaders.

### Materials
`MaterialLibrary` is shared-instance: every mesh referencing a `.mat` holds the *same* `PBRMaterial`. When a `.mat` file changes on disk, re-run `ApplyMaterialJson` on the existing instance — every mesh updates live. Texture loads inside it go through the registry, so unchanged textures are cache hits.

### Detection: polling, not OS watchers
Keep the proven mtime-poll pattern, driven by the registry (it already knows every path):

```
every 0.5s: for each live registry entry → stat → mtime changed? → Reload()
```

OS watchers (`ReadDirectoryChangesW` / FSEvents / inotify) are per-platform, and editors that save via rename-temp-file produce messy event sequences. A stat sweep over a few hundred assets twice a second is negligible — and it's the same sweep the cooked-texture staleness check (§4) wants.

### Scope
- **v1: textures + `.mat` materials.** Shaders are done. This covers the art iteration loop.
- **Deferred: meshes/models.** Mesh data is baked into `MeshComponent`s and physics at load; reload means re-touching components, not swapping inside one object. Different problem, low payoff — punt.
- **Vulkan backend (2026-07-10): done.** In-flight frames and cached descriptor sets mean an in-place GL-style swap isn't enough on its own — `VulkanTexture2D::Reload()` still swaps in place (new `RHITexture`, old one freed), but the driver (`main.cpp`'s frame loop, backend-agnostic) must call `EditorBackend::WaitIdle()` *before* `Assets::ReloadChanged()/ReloadAll()` runs, not just before invalidating caches afterward — otherwise a prior frame's in-flight command buffer can still be sampling the `VkImage` that `Reload()` just freed. Sequence: cheap mtime precheck (`Assets::HasPendingReloads()`, stat-only, no stall) → `WaitIdle()` → reload sweep → `InvalidateSceneCaches()`. GL's `WaitIdle()`/`InvalidateSceneCaches()` are no-ops, so the same driver code costs it nothing.
  - **Future: narrow the invalidation.** `InvalidateSceneCaches()` is a coarse hammer — it drops *every* baked material/mesh descriptor set on any texture or material change, not just the ones actually affected (same tradeoff shader `DoReload` already makes, batching a poll's changes into one stall). Fine at a 0.5s poll cadence; worth revisiting — e.g. tracking which materials reference a reloaded texture and calling the existing per-material `InvalidateMaterial()` instead — if hot reload ever needs to run at higher frequency or scenes get large enough that a full-cache drop becomes a visible hitch.

---

## 3. Prefabs

### Mental model
A `.prefab` is **the scene format restricted to one entity's subtree**. `SceneSerializer` already has everything hard: per-entity component JSON, `ComponentRegistry` for script components, UUID-based parent/child remapping on load. Prefabs are a refactor of that code plus editor UX — not a new serializer.

### File format
Same JSON schema as `.scene`, with a designated root:

```json
{ "prefab": { "root": 1234567890, "entities": [ ...same as scene... ] } }
```

Refactor `ToJson`/`FromJson` to expose per-entity-subtree entry points (serialize entity + descendants / instantiate a list of entity blobs) shared by both scene and prefab paths.

### Operations (v1)
- **Create**: drag an entity from the hierarchy into the ContentPanel (drag-drop plumbing exists — `CONTENT_ITEM_PATH` etc.) → serialize its subtree → write `Name.prefab`.
- **Instantiate**: drag a `.prefab` from the ContentPanel into the viewport/hierarchy → deserialize with **fresh UUIDs per instance**, remapping internal parent/child references consistently (the `uuidToEntity` remap in `FromJson`, applied to generated-not-stored UUIDs). Root spawns at the drop location.
- **Edit**: edit an instance in the scene, drag back / "Save to Prefab" to overwrite. No separate editing context in v1.

Asset loads during instantiation go through the registry, so stamping 50 copies of a prefab shares textures/meshes.

### Deliberately punted
- **Live instance↔asset linking** — instances updating when the prefab file changes, per-instance property overrides, nested prefabs. This is the hardest feature in Unity's editor; v1 instantiation is a plain copy. **Do** add a small `PrefabInstanceComponent { std::string sourcePath; }` stamped at instantiation so the link exists when we want it (also enables "select prefab source" and a future re-apply).
- **v1 shipped (2026-07-10).** `SceneSerializer` refactored into shared per-entity entry points (`SerializeEntity` / `InstantiateEntities` with a `preserveUuids` flag); `PrefabSerializer::Save/Instantiate` in SceneSerializer.h. Instantiation keeps CreateEntity's fresh UUIDs and remaps internal references through the stored ones — including `ConstraintComponent.targetUuid`, which persists as a uuid and needed an explicit rewrite (identity on scene loads). Create = drag entity onto the ContentPanel (window-wide `HIERARCHY_ENTITY` target → callback in EditorLayers names/uniquifies the file and stamps the source entity); Instantiate = drag `.prefab` onto a hierarchy node (parented, deferred past the tree iteration), the hierarchy blank area (root), or the viewport (ray→y=0 plane, fallback 10 units along the ray) — all undoable; Edit = hierarchy context menu "Save to Prefab" on entities carrying `PrefabInstanceComponent`. The component is scene-serialized and captured by the hierarchy's `EntitySnapshot`, so the link survives save/load, play-mode restore, duplicate, and delete-undo. Prefab files strip the root's `parentUuid` and `prefabInstance` (the link is stamped fresh with the actual load path at instantiation).
- **Isolated prefab-editing panel** (own empty scene context, like ParticlePreviewPanel) — good v2, unnecessary for the core workflow.

---

## 4. Texture Compression (BCn / DDS)

### Necessity — three wins, not one
- **VRAM**: PNG/JPEG must be decoded to raw RGBA8 for the GPU — 4 B/px + ~33% for mips. BCn stays compressed *in VRAM*, sampled directly by hardware: BC1 = 8:1, BC7 = 4:1. A 4K albedo: ~90 MB → ~11–22 MB resident.
- **Bandwidth**: smaller texels → better texture-cache behavior → faster sampling (visible in the GPU profiler).
- **Load time**: no stb decode, no runtime mip generation. Loading = read file, `glCompressedTexImage2D` per mip. Scene loads get dramatically faster.

### Pipeline shape: cooked cache, sources stay authoritative
```
Assets/Textures/rock_albedo.png      ← source of truth, what artists edit
Assets/Cache/Textures/rock_albedo.dds ← cooked BCn + full mip chain (gitignored)
```

1. **Cook step** — invoke `texconv` (DirectXTex) per source texture, offline. Trigger: an editor "Cook Textures" action and/or automatic on import in the ContentPanel; a standalone script also works. Skip if the `.dds` is newer than the source (mtime — same check as hot reload).
2. **Load step** — the registry's texture loader checks for a fresh cooked file first; hit → DDS path, miss/stale → existing PNG path. Callers never know the difference.

Hot reload composes: source PNG changes → cooked file is stale → reload from source PNG immediately, re-cook in the background or on next cook pass.

### Format mapping (usage must drive format)
| Usage | Format | Notes |
|---|---|---|
| Albedo / emissive | BC7 **sRGB** (BC1 if quality allows) | sRGB flag must live in the DDS + GL internal format |
| Normal maps | **BC5** (2-channel) | reconstruct Z in shader; BC1/BC7 normals artifact badly |
| Roughness / metallic / AO masks | BC4 (single channel) | consider channel-packing R/M/AO later |
| HDR (env maps) | BC6H | later; IBL path keeps float textures for now |

The cook step needs per-texture usage info. v1: filename convention (`*_n`, `*_normal` → BC5, etc.) or the `.mat` slot the texture is referenced from; a `.meta` sidecar is the eventual home.

### Runtime loading
- stb_image does not read DDS — add a small DDS header parser (the format is simple) or a single-header lib (`dds_ktx.h`). Upload each mip with `glCompressedTexImage2D` using `GL_COMPRESSED_RGBA_BPTC_UNORM` / `_SRGB_ALPHA_BPTC` (BC7), `GL_COMPRESSED_RG_RGTC2` (BC5), `GL_COMPRESSED_RED_RGTC1` (BC4), S3TC enums for BC1/BC3.
- **macOS caveat**: GL 4.1 has no BPTC (BC7/BC6H — core in 4.2). BC1–BC5 (S3TC/RGTC) are fine. The PNG fallback path stays load-bearing on macOS; don't delete it.
- `texconv` is Windows-only; fine since cooking is a dev-machine pre-process and cooked files are per-machine cache. (Cross-platform cooking later: NVTT or basis_universal.)

---

## 5. Script Hot Reload (gameplay DLL)

### Idea
Move `Sandbox/src/Scripts/` into its own CMake target built as `Scripts.dll`. Watch the script sources; on save, recompile the DLL and swap it into the running editor. This is the standard native-C++ approach (Unreal Live Coding, `cr.h`, Handmade Hero).

### Why the architecture is already well-shaped for it
- **State/behavior split is already correct.** Systems hold behavior and are created fresh per play session (`SystemRegistry::CreateSystems`, Scene.cpp); all state lives in components in the host's EnTT registry. Reload = throw away code objects, keep data, recreate systems from the new DLL.
- **Per-component JSON serializers already exist** (`SerializeComponent` / `DeserializeComponent` via `ComponentRegistry`). They are exactly the state re-hydration mechanism a reload needs: snapshot script components to JSON, reload, re-add + deserialize. Field-name-keyed JSON means struct layout changes (added/removed fields) degrade gracefully via `j.value` defaults.
- **Scripts are already isolated** in one folder with an auto-managed manifest (`AllScripts.h`) — a clean target boundary, and the editor already has script tooling (Scripts > New Script).

### The four hard problems
1. **Duplicate singletons.** `MyEngine` is a *static* lib; if `Scripts.dll` links it, the DLL gets its own copies of every static — `SystemRegistry`, `ComponentRegistry`, `Input`, spdlog, the ImGui context. `DECLARE_SYSTEM` in the DLL would register into a registry the host never reads. Fix is an explicit handshake, not linker tricks: the DLL exports
   - `DiamondScripts_Init(HostAPI*)` — receives host pointers; calls `ImGui::SetCurrentContext` + `SetAllocatorFunctions` (the documented cross-DLL ImGui pattern) and wires any other shared services;
   - `DiamondScripts_Register(ComponentRegistry&, SystemRegistry&)` — drains the DLL-local static registrars into the host's registries. `DECLARE_COMPONENT`/`DECLARE_SYSTEM` macros stay unchanged.
2. **Dangling pointers on unload.** Everything pointing into DLL code/memory must be destroyed before `FreeLibrary`: live system instances, the `std::function` lambdas inside `ComponentRegistry` descriptors, and the profiler's scope keys — `GetName()` returns a string literal *in the DLL* and the profiler keys scopes by pointer (Scripting.h). The reload procedure clears these in order; the host tags which registry entries came from the DLL so only those are removed.
3. **EnTT across the module boundary.** Component pools live in the host; pool types are defined in the DLL. EnTT documents shared-library support (`ENTT_API`) for consistent type identity across modules. The snapshot/restore approach sidesteps most of it: script-component pools are destroyed entirely before unload and recreated after.
4. **Build/tooling plumbing.** Windows locks a loaded DLL and its PDB — load a renamed copy (`Scripts_v{n}.dll`) each time. Detection is the same mtime polling as §2. Swap only on successful link; compile errors keep the old DLL running (same keep-old-on-failure policy as shaders/textures). Same-toolchain + `/MD` on both sides (already forced project-wide — see the Jolt CRT note in CLAUDE.md) makes passing C++ types across the boundary acceptable for a dev-only feature.

### Reload procedure
```
detect change → cmake --build --target Scripts → link ok?
  snapshot script components to JSON (old descriptors)
  destroy live systems (if playing) → destroy script component pools
  unregister DLL-tagged registry entries → clear profiler scopes → FreeLibrary
  LoadLibrary(new copy) → Init(host api) → Register(registries)
  re-add + deserialize components → recreate systems (if playing)
```
Components exist in edit mode too (placed on entities, drawn by inspectors), so the snapshot dance is required even when not playing.

### Scope
- **v1: reload allowed outside play mode only** — skips live-system teardown/rebuild and makes the failure story trivial. Reload-while-playing is v2.
- **Iteration-speed caveat**: the win is bounded by the script TU's compile time (script headers pull in Scene, ImGui, nlohmann, glm, spdlog — expect seconds, not shader-reload-instant). A PCH for the Scripts target is the lever if it feels slow.
- **Considered and rejected**: embedding a scripting language (Lua, C#/Mono) — solves reload but abandons the C++ systems-first scripting model; a much bigger detour than a DLL boundary.

---

## Dependency graph

```
Registry ──→ Hot reload (registry provides the watch list + in-place reload targets)
   │              │
   │              └─ (same mtime-poll pattern drives §5 script DLL rebuilds)
   │
   └──────→ Compression (registry loader is where the cooked-file check lives)

Prefabs — independent; builds on SceneSerializer + ContentPanel only
Script DLL — independent of the asset items; largest single piece, do last
```
