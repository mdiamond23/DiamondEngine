# Ray-Traced Reflections Design

Hardware-RT reflections as a **fallback under SSR**, not a replacement for it.
SSR already resolves most on-screen reflections cheaply and well; it fails on
off-screen geometry, backfaces, and rays that leave the frame. Those failures are
exactly the pixels this traces. Vulkan-only, tier 2 (RT hardware) — tiers 1 and 0
keep today's SSR + prefiltered-IBL behaviour untouched.

Builds on the ray-tracing layer from [gi-design.md](gi-design.md) slices 2-3.
Nothing in the RT stack is new work: BLAS/TLAS, RT pipelines, SBT, the bindless
geometry table and the bindless albedo array all ship already.

## Decisions

| Question | Decision |
|---|---|
| Relationship to SSR | Fallback. SSR runs first; RT fills only where its confidence alpha is 0 |
| Where the ray goes | `traceRayEXT` against the existing scene TLAS — probes play no part in reflection visibility |
| Hit-point shading | Direct light (ray-query shadowed) + **DDGI irradiance at the hit**, reusing `ddgi_probe_trace.rchit` almost verbatim |
| Rough reflections | Not traced. Above `ROUGHNESS_FADE_BEGIN` the existing IBL fade already owns the pixel |
| Denoising | None. Gate to low roughness instead — a 1 spp denoiser is its own project |
| Compositing | RT writes a *separate* target; `ssr_composite.frag` is re-pointed at it and its blend math is unchanged |
| Dynamic geometry | Widen the TLAS to dynamic rigid bodies (slice 5.3). Skinned meshes stay out |

## Where it slots

Between the SSR trace and the SSR composite:

```
… → SSGI (→ hdrGI) → SSR (→ ssrColor) → [RTReflect (→ rtReflection)] → SSRComposite (→ hdrSSR) → TAA → …
```

The RT pass reads `ssrColor` sampled and writes `rtReflection` as a storage
image, passing SSR's result straight through where confidence is high. The
composite then samples one unified reflection buffer and never learns RT exists.

`RHIRenderGraph::Compile` topologically sorts (Kahn), so declaration order
doesn't matter — only that the composite reads `rtReflection`. No need to split
`VulkanSSRPass::AddToGraph`.

Reading the DDGI irradiance atlas costs **no new ordering constraint**: a plain
`.Read(irradiance)` makes the pass depend on `DDGIBlendIrradiance`, which is
already upstream of deferred lighting → SSGI → SSR → here.

## What's new

| File | Notes |
|---|---|
| `rt_reflection.rgen` | New logic. Three early-outs (SSR confidence, background, roughness), then reflect and trace |
| `rt_reflection.rchit` | ~90% a copy of `ddgi_probe_trace.rchit` — same geometry-by-device-address, same bindless albedo, same ray-query shadows. Swap the recursive previous-frame term for a probe lookup at the hit |
| `rt_reflection.rmiss` | Sample the equirect environment. `ddgi_probe_trace.rmiss` is the template |
| `VulkanRTReflectionPass.{h,cpp}` | Modelled on `VulkanRTDebugPass`: inert `SupportsRayTracing()` ctor guard, `SetTLAS` dirty flag, deferred set creation inside `SetExecute` |

Edits:

- `SceneRenderer` — declare `rtReflection` (`storage: true`), construct/own the
  pass per view, feed it `SetTLAS` / `SetGeometryBuffer` / `SetAlbedoTextures`
  alongside DDGI, and re-point the composite's source handle.
- `VulkanSSRPass::AddToGraph` — one extra parameter for the texture the composite
  samples, defaulting to `ssrColor`.
- `RenderSettings` — enable toggle, roughness cutoff, max ray distance, half-res
  flag. Per house rule, these are LOOK knobs and belong in `RenderSettings`.
- `Engine/CMakeLists.txt` — the three new shaders join `_vk_rt_shaders`
  (line 456), the list that already gets `--target-env vulkan1.3`.

## Slices

**5.1 — Skeleton.** RT pipeline + SBT + graph node; rgen writes a flat colour and
ignores the TLAS; composite re-pointed at `rtReflection`. Verifies bindings,
storage-image barriers, pass ordering and the composite handoff in isolation.
Success = the screen tints wherever SSR confidence is 0, and validation is
silent.

**IMPLEMENTED 2026-08-14.** Full solution builds clean, the Vulkan editor runs
20 s with ZERO validation output, and two things were machine-checked with
temporary probes (both stripped): the compiled pass order is
`… > SSGIComposite > SSR > **RTReflect** > SSRComposite > TAAResolve > …`,
exactly the designed slot, and the pass takes its **trace** path rather than
silently falling back. Visual play-verify PENDING — nobody has looked for the
magenta yet.

`rt_reflection.{rgen,rmiss,rchit}` + `VulkanRTReflectionPass` landed as
designed; the rmiss/rchit are payload-only stubs (never invoked, since 5.1 does
not trace) so 5.2 fleshes them out in place instead of adding files. The rgen
declares `uTLAS` and the pass writes that descriptor even though 5.1 ignores it
— the set layout comes from the explicit `resourceBindings`, not shader
reflection, so 5.2 is a shader-only change. Editor gains an "RT Reflections"
checkbox and `RenderSettings::rtReflections`; the remaining knobs stay in 5.3.

**ADDITION NOT IN THE ORIGINAL DESIGN — `rt_reflection_passthrough.comp`.**
Re-pointing the composite makes `rtReflection` *mandatory*: it must be written
every frame, on every pixel. But the RT pass cannot always write it. The TLAS is
null in any scene with no static opaque meshes, and reflections are
user-toggleable. Either case would have left the composite sampling whatever the
pooled texture last held. So the pass owns a trivial compute copy of `ssrColor`
and picks between the two pipelines at execute time; the graph still sees
exactly one writer. Choosing in C++ rather than branching in the raygen matters
for 5.2: once the rgen actually calls `traceRayEXT`, the TLAS descriptor becomes
*statically used* and must be valid, so a "no TLAS" branch inside the shader
would not have been legal.

**5.2 — Real shading.** Port the rchit and rmiss, trace the reflection ray, add
the DDGI irradiance lookup at the hit point (with `ddgi.fallback` when the scene
has no volume). Success = a mirror floor shows off-screen geometry, correctly
shadowed and with no black holes in shadowed regions.

**IMPLEMENTED 2026-08-14.** Full solution builds clean; the Vulkan editor runs
**30 s with zero validation output and no device loss** — the long run is the
point, since this slice adds exactly the shadow-ray light loops that produced a
silent `VK_ERROR_DEVICE_LOST` in slice 4 of gi-design.md. The `clamp(int(...),
0, 4)` on both light counts came across with the port. A temporary probe (since
stripped) confirmed the trace set actually builds and the RT path runs rather
than silently falling back — worth checking, because 5.2 has many more ways to
fall back than 5.1 did. Visual play-verify PENDING.

- `rt_reflection.rgen` — three early-outs in cost order (SSR confidence,
  background, roughness), then reflect the view ray about the G-buffer normal,
  transform to world and trace. Every traced pixel reports confidence 1.0: a
  miss returns the environment, which is a legitimate reflection rather than a
  failure.
- `rt_reflection.rchit` — ~90% `ddgi_probe_trace.rchit`. Three deliberate
  differences: the indirect term is a DDGI lookup **at the hit point** rather
  than the probe's own previous-frame value; a backface **flips the normal**
  instead of reporting a negative distance (a probe treats backfaces as evidence
  it is buried, a reflection just needs the visible side lit — returning black
  would put holes in reflections of thin geometry); and there is no
  relocation/classification signalling to feed.
- `rt_reflection.rmiss` — the baked radiance cube × `skyIntensity`, matching the
  skybox so a reflected sky and the real sky agree.
- Roughness cutoff defaults to **0.30**, which is `ROUGHNESS_FADE_BEGIN` in
  `ssr_composite.frag`. Above it the composite is already fading to prefiltered
  IBL, so a mirror-sharp ray there would be wrong as well as wasted.
- One `ReflectBlock` UBO carries `view`/`invView` plus the four packed DDGI
  volume fields, built exactly as `deferred_lighting.frag` builds its local
  `DDGIVolume`. `ApplyGITier` now feeds the reflection pass alongside the
  lighting pass, so the two can never disagree about the grid.

**Limitation worth stating.** With no `DDGIVolumeComponent` in the scene,
`giMode` is not 2 and reflection hits get **no indirect term at all** — a
reflection of anything the sun cannot see goes black. The atlases are still
bound (`ddgi.fallback`) purely to keep the descriptor set complete. The success
criterion above therefore holds only in a scene with a probe volume. Giving the
no-volume case a real fallback would mean binding the IBL irradiance cubemap as
a second raw descriptor write; deliberately not done here.

**5.3 — Gating and cost.** Confidence/roughness early-outs tuned, optional
half-res trace + bilateral upsample, shadow-ray light loop trimmed, dynamic
rigid bodies added to the TLAS, `RenderSettings` knobs exposed. Success = the
per-pass profiler row is affordable in Sponza and reflections track a moving
crate.

Non-goals: any denoiser, skinned geometry in the AS, cone-traced rough
reflections, GL backend.

## Performance

At 1080p, per frame. The shadow multiplier is the term that surprises people —
the rchit fires one ray query *per light* per hit, so "1 spp" is really ~10 rays
per pixel with a full light set.

| Scenario | Primary rays | × shadow | Total |
|---|---|---|---|
| Typical, ~10% of pixels reflective and SSR-failed | ~200k | ~10 | ~2M |
| Mirror floor over 60% of screen | ~1.25M | ~10 | ~12M |
| Same, half-res + sun-only shadow rays | ~310k | 1 | ~310k |

Already paid for: TLAS build/refit (DDGI's cost), BLAS builds (once, at mesh
registration), the `WaitIdle` on descriptor rebuild (only on TLAS pointer
change). The marginal cost is the trace alone.

New memory: one full-res RGBA16F target, 1920×1080×8 ≈ **16.6 MB** (4.1 MB at
half res). SBT is negligible.

Levers by value: half-res trace (4×), roughness cutoff, ray max distance, then
trimming the shadow-ray light loop. The pass gets a per-pass profiler row and a
Tracy GPU zone for free, so the `tracy-capture` → `csvexport` pipeline measures
it next to the DDGI trace directly.

## Known traps

- **Never write back into `ssrColor`.** Reading a texture and writing it in the
  same chain cycles the graph — readers depend on a texture's *last* writer. This
  is documented at `VulkanSSRPass.cpp:52` because SSR already hit it once.
- `rtReflection` must be declared `storage: true`. An RT pass writes via
  `imageStore`.
- **The TLAS is narrower than "static".** `SceneRenderer` gates instances on
  `mc.staticShadowCaster` *and* `BodyType::Static`, and the instance walk covers
  `MeshComponent` only — so dynamic props and every skinned character are absent.
  Reflections show the static world until 5.3 widens this. The inconsistency is
  visible: a character reflects via SSR on screen, then vanishes as the RT
  fallback takes over off screen.
- **No UV flip anywhere in the rgen.** Read the G-buffer with
  `texelFetch(g, ivec2(gl_LaunchIDEXT.xy), 0)` and write with `imageStore` at the
  same coord — launch ID and texel row agree, and the composite's `vUV` from
  `fullscreen.vert` reads row 0 at `v = 0`. The flip only exists in the UV↔NDC
  conversion (`rt_debug.rgen:30`), and reading view-space position straight out of
  `gViewPos` means this pass never converts. Do not copy that line.
- **Clamp any UBO-sourced loop bound before it drives a ray query.** An
  out-of-range light count is harmless arithmetic in a raster pass and a silent
  `VK_ERROR_DEVICE_LOST` once each iteration costs a BVH traversal. See
  gi-design.md's open trap — the root cause is still unfound, so the clamp comes
  along with the ported rchit.
- A scene with no `DDGIVolumeComponent` still needs a bound atlas. Reuse
  `ddgi.fallback` exactly as `VulkanDeferredLightingPass.cpp:91` does.
- SSR confidence is a *fade*, not a boolean. Thresholding it too high double-
  counts (both sources contribute); too low leaves a visible seam where the
  screen-space march gives up. Expect to tune the threshold against
  `EdgeFade`'s 0.1 border ramp.
