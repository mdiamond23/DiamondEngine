# Global Illumination Design — DDGI + SSGI

Replaces the current "IBL ambient × SSAO" stand-in for indirect light with real
GI: **DDGI** (Dynamic Diffuse Global Illumination, Majercik et al. 2019) for
far-field/offscreen irradiance, **SSGI** for the near-field bounce probes are too
coarse to resolve. Vulkan-only — the GL backend is in retirement and gains no new
features.

## Decisions

| Question | Decision |
|---|---|
| Probe ray source | `VK_KHR_ray_tracing_pipeline` — real RT, raygen/closest-hit/miss + SBT in the RHI |
| Acceleration structure scope | Static opaque meshes only (slice 1); skinned/dynamic excluded |
| SSGI implementation | New half-res **compute** pass, separate from SSR, sharing the march via a GLSL include |
| Indirect injection | DDGI inside `deferred_lighting.frag`; SSGI composited after lighting, DDGI as its miss fallback |
| SSAO | Survives as near-field occlusion on the **indirect term only**, not a blunt ambient multiplier |
| Probe volumes | `DDGIVolumeComponent` — placed, inspected, serialized like any other component |

## Hardware tiers

RT is required only by DDGI. SSGI is plain compute over the G-buffer and runs
anywhere the engine already runs, so the fallback is a real tier, not a cliff:

| Tier | Requires | Far-field indirect | Near-field indirect | Occlusion |
|---|---|---|---|---|
| 2 — Full | RT extensions | DDGI probes | SSGI | SSAO on indirect only |
| 1 — Screen-space | nothing new | IBL irradiance | SSGI | SSAO on indirect only |
| 0 — Baseline | nothing new | IBL irradiance | none | SSAO on ambient (today) |

Tier 0 is the current shipping path, unmodified — it stays as the floor. Tier 1
is a genuine upgrade for non-RT hardware, not a consolation prize.

Selection: query the RT extensions at device creation to pick a default, and
expose an explicit override in settings. The override matters — extension
presence is **not** proof of RT hardware. GTX 10/16-series expose
`VK_KHR_ray_tracing_pipeline` through driver emulation and are far too slow to
use it, so those users need to be able to force tier 1.

Mechanism: **no shader permutations.** The far-field term is one line in
`deferred_lighting.frag` (`texture(irradianceMap, worldN).rgb`). Tier selection
is a `giMode` field in the existing lighting UBO branched on at that line, with a
dummy 1×1 probe atlas bound when probes don't exist. The branch is uniform across
the whole fullscreen draw, so it costs nothing measurable, and it avoids building
permutation machinery the shader-compile loop doesn't currently have.

The ongoing cost of keeping tiers is a test matrix, not a second renderer: every
GI change has to be eyeballed in tier 1 as well as tier 2.

## Current state this builds on

- `deferred_lighting.frag:210,275` — SSAO's only job today is attenuating the
  ambient IBL term. That is exactly the slot probe irradiance takes over.
- `ssr.frag` — the view-space march, `ProjectToUV` y-flip, thickness/bias
  rejection and edge fade are all directly reusable by SSGI.
- `ssr_composite.frag` — the confidence-weighted resolve-into-a-new-target
  pattern is the model for the SSGI composite, including the reason it can't
  blend in place (graph readers bind a texture's *last* writer → cycle).
- RHI compute pipelines, storage buffers/images, graph compute nodes and
  persistent/imported textures with `ReadHistory` all landed already — SSGI's
  temporal history and the probe atlases need no new graph machinery.
- `VulkanContext.cpp:29,301` — only `VK_KHR_SWAPCHAIN` is requested today, on a
  Vulkan 1.3 device. 1.3 makes `buffer_device_address`, `descriptor_indexing`
  and SPIR-V 1.4 core, so RT costs three extensions, not a feature audit.

## Architecture

### 1. Ray-tracing layer (new, largest piece)

Extensions: `VK_KHR_acceleration_structure`, `VK_KHR_ray_tracing_pipeline`,
`VK_KHR_deferred_host_operations`, chained as feature structs into the existing
`VkPhysicalDeviceFeatures2` at device creation, behind a capability query.

- `RHIBufferUsage` gains `ShaderDeviceAddress` + `AccelStructInput`; mesh vertex
  and index buffers (`SceneRenderer.cpp:226`) take both, since BLAS builds read
  them by device address.
- BLAS: one per registered mesh, built once at mesh registration.
- TLAS: rebuilt from static `MeshComponent` transforms. The static-set hash
  already used for static shadow-caster dirty tracking is the rebuild trigger.
- Hit shading needs per-instance geometry access: an SSBO of
  `{vbAddress, ibAddress, materialIndex}` plus a material array, read in the
  closest-hit shader via descriptor indexing.
- New RHI surface: acceleration-structure resource, RT pipeline kind, shader
  binding table allocation/alignment, `TraceRays`.

### 2. DDGI

`DDGIVolumeComponent`: origin, extent, probe counts, rays/probe, hysteresis,
normal bias, energy — registered through `ComponentRegistry`, serialized, with
probe wireframe viz under the existing Debug Draw toggle.

Per volume, persistent graph resources:
- irradiance atlas — RGBA16F, octahedral 8×8 + 1px border per probe
- visibility atlas — RG16F, octahedral 16×16 + border (depth + depth², for the
  paper's Chebyshev occlusion test)
- probe offset + probe state buffers (relocation and classification)

Per frame: raygen traces `raysPerProbe` rays per probe under a random rotation;
closest-hit returns direct light at the hit **plus the previous frame's probe
irradiance** — that recursive term is what buys infinite bounces for one ray
depth. Two compute passes then blend radiance and depth into the atlases with
hysteresis, followed by relocation/classification. Probe updates round-robin
against a per-frame budget.

Sampling in `deferred_lighting.frag`: trilinear probe weighting + Chebyshev
visibility + normal/view bias, replacing the IBL irradiance term. Specular IBL
stays as-is.

### 3. SSGI

`ssgi.comp`, half resolution: N cosine-distributed hemisphere rays per pixel
marched through `gViewPos` with the same thickness/bias/edge-fade rules as
`ssr.frag`, factored into a shared include. Hit radiance is fetched from
`hdrLit` — which already contains the DDGI term, so screen-space multi-bounce
comes free (needs a clamp to prevent feedback blowout).

Output is RGBA16F at half res: rgb = incoming indirect radiance, a = confidence.
Then temporal reprojection through `gVelocity` using the graph's `ReadHistory`,
with neighborhood clamping, and an edge-aware (depth + normal) bilateral upsample
to full res.

### 4. Composite and double-counting

Because DDGI is applied inside lighting, the SSGI composite must **replace** the
probe term where SSGI has data, not stack on it:

```
hdrGI = hdrLit + M * (ssgiIrradiance - farIrradiance) * confidence
M     = kD * albedo * ao * ssao        // the multiplier lighting already applied
```

`farIrradiance` comes from **`gIndirect`**, a second MRT output added to
`deferred_lighting.frag` carrying the far-field irradiance that pass used,
*before* the `M` multiplier. Writing it out beats re-deriving it in the
composite: the subtraction is then exact rather than a reconstruction of
`kD`/`ao`/`occlusion`, and it makes the composite tier-agnostic — in tier 2 the
same target simply carries probe irradiance instead of the IBL sample and
nothing in the composite changes. Cost is one RGBA16F full-res write.

Where confidence is 0 the expression collapses to `hdrLit`, i.e. pure far-field,
exactly like SSR's miss path.

Units: both irradiances follow `irradiance_convolution.frag`, which stores
**E/π**, not E. The cosine-pdf Monte Carlo estimator for E is `(π/N)·ΣL`, so the
matching E/π estimate is `(1/N)·ΣL` — SSGI must *not* multiply by π. The trace
divides by the hit count rather than the ray count, which makes the confidence
lerp exactly the "unknown directions look like the far field" assumption.

Pass order becomes: G-buffer → SSAO → lighting (DDGI) → skybox → **SSGI
composite** → SSR → TAA → transparency → particles → bloom → tonemap. SSR reads
`hdrGI` instead of `hdrLit`.

## Slices

Ordered SSGI-first, because SSGI has no RT dependency: it delivers a visible
improvement without the acceleration-structure subsystem, and it *is* tier 1, so
the fallback gets built and proven before the RT work rather than bolted on after.

1. **SSGI** — half-res compute trace, temporal reprojection + bilateral upsample,
   composite pass with the IBL far-field term subtracted. Ships tier 1 on its own.
   **IMPLEMENTED 2026-08-12** — `VulkanSSGIPass` (4 graph passes: trace, temporal,
   history copy, composite), `gIndirect` MRT out of lighting, editor toggle +
   rays/intensity/distance sliders. Full solution builds clean and the Vulkan
   editor runs with zero validation output; visual play-verify still pending.
2. **RT foundation** — extensions/features/capability query, device addresses,
   BLAS/TLAS from static meshes, RT pipeline + SBT, and a throwaway debug pass
   that visualizes ray hit distance fullscreen. Independently verifiable.
3. **DDGI core** — volume component, atlases, raygen/rchit/rmiss, blend +
   relocation/classification compute, probe debug viz. Not yet lighting the scene.
4. **DDGI into lighting** — probe irradiance replaces IBL ambient behind `giMode`;
   SSAO demoted to indirect-only near-field; composite switches its subtracted
   term to the probe lookup. Tier 2 complete, A/B toggle against tier 1.
5. **Polish** — probe budget tuning, Tracy/profiler zones per new pass, editor
   toggles, tier override in settings, Sponza perf baseline per tier.

## Known traps

- The shader compile loop (`Engine/CMakeLists.txt:426`) invokes
  `glslangValidator -V` bare, which targets Vulkan 1.0. RT stages need
  `--target-env vulkan1.3` (SPIR-V ≥ 1.4). Bumping the whole loop needs a
  re-verify of the existing shaders.
- A shared GLSL include needs `GL_GOOGLE_include_directive` + `-I`, and CMake's
  `DEPENDS` won't track the include — add it explicitly or edits won't rebuild.
- `ProjectToUV`'s y-flip in `ssr.frag:37` must be copied verbatim into SSGI. This
  codebase has been bitten by the negative-viewport UV convention before.
- **Compute shaders must use `textureLod(s, uv, 0.0)`, never `texture(s, uv)`.**
  Implicit-LOD sampling needs derivatives, which only fragment shaders have.
  `screen_trace.glsl` is shared with fragment shaders, so it uses `textureLod`
  throughout for both.
- An MRT slot left unwritten on any early-return path is undefined. The
  background early-out in `deferred_lighting.frag` has to write `outIndirect`
  too.
- Skinned characters are absent from the TLAS. They still rasterize and are lit
  normally, and they *receive* GI (probe sampling is per-pixel and doesn't care
  what's in the AS) — they just don't occlude or bounce probe rays, so no
  character-tinted bounce and no probe-side indirect shadow. SSGI recovers both
  while they're on screen; off-screen characters contribute nothing. Excluding
  them also keeps probes from flickering as they move.
- SSGI is temporally accumulated *and* feeds TAA, which already discards history
  aggressively on reflective pixels. Expect to tune the two together.
