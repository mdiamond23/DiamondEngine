# Particle System Design

Status: **Phases 1-5 built & build-verified** (billboard renderer, simulation,
emission shapes + bursts, inspector + serialization, presets). Roadmap item:
"Particle system — hit sparks, dust, environmental effects."

## Goals

A GPU-light particle system for hit sparks, dust, smoke, fire, and environmental
effects. Authored per-entity via a component, simulated on the CPU, drawn as
camera-facing billboards that composite into the existing HDR/bloom pipeline.

## Architecture: two decoupled layers

Mirrors how Unity (Shuriken) and Unreal (Niagara) separate concerns — one
simulation, a swappable renderer.

```
ParticleEmitterComponent (config)        ──┐
ParticlePool (runtime, per emitter)        ├─ Simulation  (renderer-agnostic)
ParticleSimSystem (emit / integrate / kill)┘
                    │  produces std::vector<RenderParticle>
                    ▼
ParticleRenderer (interface)             ──┐
  └─ OpenGLParticleRenderer (billboards)   ├─ Rendering  (sim-agnostic)
     [future] mesh renderer, ribbon renderer┘
```

The **`RenderParticle`** struct is the contract between the layers. The simulation
fills arrays of it; the renderer consumes them and knows nothing about emission.
This is what lets a mesh or ribbon renderer drop in later without touching the sim
— and it is where true GPU **instancing** will eventually earn its place (mesh
particles), not in the billboard path.

## Data model

`ParticleEmitterComponent` — serialized config (emission rate, max count, shape,
lifetime/velocity/size ranges, start/end color & size, gravity, texture, blend
mode, render mode, world vs. local space).

`Particle` runtime pool — **not serialized** (transient, like the bone palettes):
`{ vec3 pos, vel; vec4 color; float size, age, lifetime, rotation; }`. Stored in a
side buffer keyed by entity so save/load never touches live particles.

## Rendering: where it hooks in

The deferred render graph in `Sandbox/src/main.cpp` already has the right slot. A
`Particles` pass is added **after `Transparency`** and before bloom:

- Reads `gViewPos` + `hdrBuffer`; no writes → sink pass, always alive. By insertion
  order it sorts after Transparency and before BloomComposite (which waits on
  BloomBlur), so particles land in `hdrBuffer` before bloom samples it — bright
  sparks bloom for free.
- `hdrBuffer` is declared `needsDepth = true`, and Transparency blits the G-buffer
  depth into it, so the pass depth-tests against opaque scene geometry.
- GL state: depth-test **on**, depth-write **off** (`glDepthMask(GL_FALSE)`), blend
  per emitter (additive `SRC_ALPHA,ONE` / alpha `SRC_ALPHA,ONE_MINUS_SRC_ALPHA`).
- The pass runs inside both `cullAndExecute` invocations, so particles appear in the
  editor viewport and the game viewport automatically.

### Billboarding (v1: CPU)

Mirrors the `Renderer2D` dynamic-VBO batch (build verts on CPU, one
`glBufferData` + `glDrawArrays` per batch). Camera right/up are pulled from the
view matrix rows:

```
right = vec3(view[0][0], view[1][0], view[2][0]);
up    = vec3(view[0][1], view[1][1], view[2][1]);
```

Each particle expands to a quad: `corner = pos + (±right*cos±up*sin) * size*0.5`,
rotation applied by spinning the right/up basis. The vertex shader is then trivial
(`gl_Position = uVP * vec4(aPos,1)`); the fragment shader modulates the texture by
per-vertex color. Handles tens of thousands of particles in one draw call. GPU
vertex-shader billboarding and instancing are deferred.

## Phasing

1. **Billboard renderer + static test data** — *done.* `ParticleRenderer`
   interface + `OpenGLParticleRenderer`; a `Particles` pass drew a hardcoded test
   array so billboarding/depth/blend were verified before any simulation existed.
2. **Simulation** — *done.* `ParticleEmitterComponent` (config + non-serialized
   pool) + `ParticleSystem` (GameSystem, priority 500): fractional-accumulator
   emit, integrate under gravity, lerp color/size over lifetime, swap-remove kill.
   Render pass now iterates emitters; a code-created "Particle Fountain" entity in
   main.cpp drives verification. Pools reset on play start/stop. Runs in play only.
3. **Emission shapes + bursts** — *done.* `EmissionShape` (Point/Cone/Sphere/Box):
   shape supplies a local spawn offset + emission direction (axis = local +Y),
   oriented by the emitter's world rotation; `speedRange` scalar replaced the old
   per-axis `velocityMin/Max`. Bursts: `burstCount` + `burstInterval` (>0 repeats,
   <=0 one-shot at start), via a transient `burstTimer`. Continuous + burst share a
   `SpawnOne` helper. Test scene: cone fountain + repeating sphere-burst explosion.
4. **Serialization + inspector** — *done.* Registered via `ComponentRegistry`
   (`DECLARE_COMPONENT` + `SerializeComponent`/`DeserializeComponent`/
   `DrawComponentInspector` specializations in `Sandbox/src/Scripts/
   ParticleEmitterEditor.h`, included through `AllScripts.h`). The generic registry
   path drives the inspector, Add Component menu, save/load, and play-mode
   snapshot/restore with no edits to InspectorPanel/SceneSerializer. Added a
   `texturePath` field so the texture round-trips (drag a texture asset onto the
   emitter; empty = default dot). **This fixes the "no particles on 2nd play" bug**
   — the emitter now survives the snapshot/restore round-trip on Stop.
5. **Presets** — *done.* `ParticlePresets::SparkBurst/DustPuff/Fire` (header-only
   inline factories in `Engine/include/Scene/ParticlePresets.h`) each return a
   fully-configured `ParticleEmitterComponent`. A preset is authoring-only: it
   overwrites the component's config fields, which already serialize — so presets
   need **zero** serialization of their own and no runtime state (the component
   never remembers which preset produced it). Surfaced as a "Preset" button row
   at the top of the inspector (`c = ParticlePresets::Fire();`); also usable by
   gameplay code via direct emplace. The three span both blend modes: Additive
   for sparks/fire, Alpha for dust — a final end-to-end validation of the
   renderer path.

## Future renderers (deferred)

- **Mesh particles** — instanced 3D geometry (debris, shells, leaves). Where GPU
  instancing pays off.
- **Ribbon/trail particles** — geometry generated between successive positions
  (swooshes, tracers, smoke trails).

Both reuse the simulation unchanged; only the render step differs.
