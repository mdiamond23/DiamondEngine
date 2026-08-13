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
| 2 — Full | RT extensions | DDGI probes | SSGI | SSAO on indirect diffuse, ×0.5 |
| 1 — Screen-space | nothing new | IBL irradiance | SSGI | SSAO on indirect diffuse |
| 0 — Baseline | nothing new | IBL irradiance | none | SSAO on indirect diffuse |

Tier 1 is a genuine upgrade for non-RT hardware, not a consolation prize, and
tier 0 stays as the floor: it needs nothing beyond what the engine already ran.

As of slice 4 the occlusion column is the same code in all three tiers, varying
only by strength — SSAO on indirect diffuse, `SpecularOcclusion` on indirect
specular. The pre-slice-4 "SSAO multiplies the whole ambient" behaviour is gone
everywhere, including tier 0. Applying a hemispherical AO term to a narrow
reflection lobe was wrong at every tier; preserving it for the low tiers would
have preserved a bug.

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
   **IMPLEMENTED 2026-08-12** — `VulkanContext::RayTracingSupported()` + RT/AS
   property queries; RHI gains accel structures, RT pipelines, `TraceRays` and
   accel-struct descriptor bindings; `VulkanRHIAccelStruct` (BLAS per registered
   mesh, TLAS over static opaque instances, rebuilt off the static-shadow hash);
   `VulkanRTDebugPass` + `rt_debug.{rgen,rmiss,rchit}` behind an editor toggle.
   Runs clean under validation on an RTX 5070 (tier 2 detected, SBT handle 32 B /
   base align 64 B) with the trace active; the hit-distance image itself is not
   yet eyeballed. Known gap: the rebuild trigger is the static *shadow-caster*
   hash, so a static non-casting mesh that moves won't re-trigger a TLAS build.
3. **DDGI core** — volume component, atlases, raygen/rchit/rmiss, blend +
   relocation/classification compute, probe debug viz. Not yet lighting the scene.
   **IMPLEMENTED 2026-08-12** — `DDGIVolumeComponent` (serialized, inspector,
   Add Component); `VulkanDDGIPass` with five graph passes (RT trace → blend
   irradiance → blend visibility → relocate/classify → probe viz);
   `ddgi_probe_trace.{rgen,rmiss,rchit}`, `ddgi_blend_{irradiance,visibility}.comp`,
   `ddgi_relocate.comp`, `ddgi_probe_debug.{vert,frag}` and the shared
   `include/ddgi_common.glsl`. Full solution builds clean, the Vulkan editor runs
   with ZERO validation output, and the probe viz shows spheres shaded from the
   atlas. Also fixed slice 2's known gap: the TLAS now rides its own instance
   hash rather than the static shadow-caster hash.

   Three decisions taken during implementation, all deliberately scoped:
   - ~~**Albedo is `BaseColorFactor`, not a texture sample.**~~ **RESOLVED
     2026-08-13 (slice-4 prep).** `RHIResourceBinding` gained a `count` field and
     `RHITextureBinding` an `arrayIndex`, which is the whole of the RHI change;
     `ddgi_probe_trace.rchit` now samples a 256-slot bindless base-color array
     with `nonuniformEXT` (rays in a subgroup hit different instances, so the
     index is non-uniform by nature — needs
     `shaderSampledImageArrayNonUniformIndexing`, now enabled). Every slot is
     written at set creation, padded with 1×1 white, so no partially-bound
     feature is involved. `RTGeometry` grew to 64 B carrying the texture slot,
     the UV scale and the vertex stride. Verified by pointing the probe viz at
     the downward irradiance and normalizing out magnitude: probes over the lava
     floor read salmon, not white.
   - ~~**Sun is CSM-shadowed; point and spot lights are unshadowed.**~~
     **RESOLVED 2026-08-13 — every light is now shadowed by an inline ray
     query.** The original reasoning (multiple miss shaders + recursion depth 2)
     assumed recursive `traceRayEXT`. `VK_KHR_ray_query` traverses inline from
     inside the closest-hit shader instead: no second miss shader, no extra SBT
     record, no recursion — `maxRecursionDepth` stays at 1. Cost was one
     extension + one feature bit in `VulkanContext`, and `rayQuery` joined the
     tier-2 capability test (the chit's SPIR-V declares the capability, so a
     device without it could not create the pipeline anyway).

     This mattered far more than "probes beyond CSM range". CSM is fitted to the
     **camera** frustum while probes sit wherever the volume is, and
     `SampleCascade` returns 0 (unshadowed) outside the cascade — so most probes,
     most frames, baked **full unshadowed sun** into the atlas, including probes
     indoors under a roof. With the Sponza test scene's sun at intensity 19 that
     turned the probe field into a uniform fill light, which is the flat look
     that prompted the whole investigation. It also drifted as the camera turned,
     since which probes fall inside the cascades is view-dependent.

     Bindings 8-11 (the cascades) are no longer declared by the chit. The
     descriptor set still carries them — unused descriptors are legal, and
     leaving them avoided touching the set layout. Worth cleaning up in slice 5.
   - **One volume, main view only.** First `DDGIVolumeComponent` wins (the
     `SkyLightComponent` rule). Probes are view-independent, so running the
     update in both view graphs would double the ray cost for an identical
     result. **The atlases were hoisted out of the per-view graph 2026-08-13
     (slice-4 prep):** `SceneRenderer` owns the three textures and every view
     graph brings them in with `ImportTexture`, so both views can sample one
     probe set while only the main view runs the update. `ddgiRayData` stays a
     per-graph transient.

   **Relocation shake — FIXED 2026-08-13, four separate defects.** Probes near
   and inside geometry visibly jittered every frame. Every one of these is the
   same class of mistake: a control loop reading a randomly-rotated ray set,
   with either a hard threshold to flip across or too few samples to be stable.
   - Steering by the SINGLE nearest ray. Its direction jitters frame to frame
     because the ray set is randomly rotated. → average the push over every
     too-close ray.
   - Discontinuous push/relax branches: a probe near a surface limit-cycled
     across the boundary. → fade the push weight smoothly to zero at the
     comfort distance.
   - No dead band: the relax term fired the instant the push faded, i.e.
     exactly at the equilibrium, which is where the push is computed from the
     fewest rays and is noisiest. → a `holdRadius` much larger than `comfort`;
     between them the probe just holds still.
   - **The buried branch, which was the worst.** `normalize(backfaceDirSum)`
     where `backfaceDirSum` sums ~64 *uniformly distributed* directions: it
     averages to near zero but not zero — a random walk of magnitude
     ~sqrt(rayCount) — so normalizing produced a fresh RANDOM escape direction
     every frame at half a cell. Probes inside a mesh thrashed. → head for the
     nearest exit (proximity-weighted toward the closest backfaces), gated by a
     smoothstep on directional COHERENCE, so a probe centred in a slab (where
     up and down are equally close and there is no preferred way out) holds
     still instead of random-walking.

   Classification also became a Schmitt trigger (enter buried at the threshold,
   leave at half of it) so the backface ratio can't re-classify every frame; and
   the backface test moved off `gl_HitKindEXT` onto the authored vertex normal,
   because winding is unreliable in imported .obj/glTF content and a
   winding-based test lies about different triangles as the ray set rotates.

   Matters because near-surface probes carry the most trilinear weight, and
   because Chebyshev depth moments are stored relative to the probe position —
   a drifting probe is a leaking probe. Residual jitter on a probe INSIDE a mesh
   is cosmetic only: buried probes are classified inactive, which excludes them
   from both the blend and `DDGISampleIrradiance`, so their position has no
   effect on the lit result.

   MEASUREMENT NOTE: the screenshot-diff harness used to quantify this is
   unreliable — the editor window opens at a cascading position each launch, and
   raising it to grab pixels can move the editor camera. The before/after diff
   IMAGES (filled probe discs vs none) are sound; the pixel counts are not
   comparable across runs.

   **Convergence ramp — ADDED 2026-08-13.** Hysteresis now starts at 0 after a
   reset and climbs as `N/(N+1)` until it reaches the user's value, i.e. a
   progressive average over the opening frames. Converges as fast as the ray
   budget allows with no magic window length; invisible in slice 3, but once
   lighting depends on probes a scene load would otherwise ramp on screen.

   Other slice-3 notes: atlases are allocated for the MAXIMUM grid (16³ probes,
   128 rays) because graph textures are declared once while probe counts are
   live-editable; the irradiance atlas stores **E/π**, matching
   `irradiance_convolution.frag`, so slice 4's swap needs no rescaling; the blend
   passes read and write their atlas in place inside one pass, which is legal
   precisely because the graph never sees it (the trace takes last frame's value
   through `ReadHistory`); `active` is a GLSL reserved word.
4. **DDGI into lighting** — probe irradiance replaces IBL ambient behind `giMode`;
   SSAO demoted to indirect-only near-field; composite switches its subtracted
   term to the probe lookup. Tier 2 complete, A/B toggle against tier 1.

   **IMPLEMENTED 2026-08-13.** Full solution builds clean, the Vulkan editor
   runs with ZERO validation output, and the compiled pass order was machine-
   checked (see below). Visual play-verify PENDING — nobody has looked at the
   lit result yet.

   - `deferred_lighting.frag` includes `ddgi_common.glsl` and branches the
     far-field term on `giMode` (`ambient.y`): 2 = `DDGISampleIrradiance`, else
     the IBL cubemap. Three new samplers (bindings 22-24) plus the four volume
     fields the lookup actually reads — no matrices, the lookup is
     view-independent. The `-I .../include` flag was already global on the
     shader-compile loop, so the include cost nothing.
   - **Sky intensity is NOT applied to the probe path.** The IBL sample is
     scaled by `ambient.x`; probe irradiance already has it, folded in by the
     trace's miss shader. Scaling again would double-apply it.
   - **No graph reorder was needed.** `RHIRenderGraph::Compile` is a Kahn
     topological sort, so registration order decides nothing: adding the three
     atlas `Read()`s to the lighting pass is what pulls the update in front of
     it. Verified by logging `m_SortedIndices` — `DDGIProbeTrace > … >
     DDGIBlendIrradiance > DDGIBlendVisibility > DDGIRelocate > DeferredLighting`,
     with `DDGIProbeDebug` still after `Tonemap`. Those reads are also what
     keeps the update alive through dead-pass culling now that the viz is no
     longer its only consumer.
   - Volume→grid derivation moved to `VulkanDDGIPass::ComputeGrid`, shared by
     the blend and the sampler. They MUST agree exactly — a grid they disagree
     about reads irradiance from the wrong probes — and the sampling view may
     have no DDGI pass instance at all.
   - `SceneRenderer::ApplyGITier` picks the tier per frame: atlases exist (RT
     only) AND a volume AND the toggle. Editor gains a tier readout and an
     "SSAO on indirect" slider; the DDGI checkbox is now the tier 2/1 A/B.

   **SSAO demotion.** Three things darkened the same crevice: lighting's ambient
   multiply, probe Chebyshev visibility, and SSGI's own gather. Now:
   - SSAO attenuates the indirect DIFFUSE term only, at `ssaoStrength`, scaled by
     a further 0.5 under tier 2 (a tuned constant, not a derivation — the honest
     factor depends on probe density).
   - Specular indirect takes Lagarde's `SpecularOcclusion(NdotV, ao*ssao,
     roughness)` instead of the raw AO, which blacks out sharp reflections in any
     crevice. **This changes tiers 0/1 too** — deliberate; keeping a raw-AO path
     for the old tiers would preserve a bug, not a feature.
   - `outIndirect.a` now carries the exact diffuse multiplier the pass applied,
     and `ssgi_composite.frag` reads it instead of re-deriving `ao × ssao`. Same
     reasoning as `gIndirect` itself: reconstruction drifts, a written value
     can't. Its `ssaoTex` binding is now unused (left declared, harmless).

   NOT verified: the second view's path (imported atlas with no producer in that
   graph). The game view is off by default, so only the main view's graph ran.
4.5. **GTAO + bent normals** — replace `ssao.frag` with a GTAO compute pass and
   feed the bent normal + cone aperture into `DDGISampleIrradiance` in place of
   the geometric normal, so the probe lookup is *directionally* occluded rather
   than uniformly multiplied down. That removes the double-count by construction
   instead of by a tuned 0.5, and it upgrades all three tiers at once because it
   is still plain screen-space compute. Deliberately its own slice: it changes
   tier 0's output, so it wants eyeballing separately from the DDGI swap.
   (RTAO off the existing TLAS was considered and rejected — 1-2 rays/pixel
   needs a real denoiser, it is tier-2 only, and it re-derives occlusion the
   probes and SSGI already carry.)
5. **Polish** — probe budget tuning, Tracy/profiler zones per new pass, editor
   toggles, tier override in settings, Sponza perf baseline per tier.

## Known traps

- **A loop bound read from a UBO must be clamped before it drives a ray query.**
  The chit's light loops ran `for (i < int(u.counts.x))` for two slices without
  trouble — an out-of-range count only wasted arithmetic. The moment each
  iteration cost a BVH traversal, the same out-of-range count became a
  **VK_ERROR_DEVICE_LOST** a few seconds in, with validation completely silent.
  Bisecting proved it: sun-only shadow rays were stable, adding the local-light
  loops killed the device, and `clamp(int(u.counts.x), 0, 4)` fixed it with every
  ray enabled. A clamp is a no-op on an in-range value, so this is proof the
  count really does leave [0,4] sometimes.
  **OPEN:** *why* it leaves that range is not root-caused. `RHIBuffer::Update`
  writes only the current frame-in-flight slot and descriptor sets bind the
  matching per-frame handle, so a never-written slot shouldn't be reachable.
  `deferred_lighting.frag` loops on the same `counts` and would also be reading
  garbage on those frames — worth chasing.
- The skybox must not go through the env cubemap. A cube face's corners cover
  ~3x the solid angle per texel that its centre does, so sharpness varies across
  every face and the edges become visible on a large smooth sky. The background
  samples the equirect source directly; the cube still feeds the irradiance /
  prefilter bakes and DDGI's miss shader, where the variation is irrelevant.
  Related: nothing feeding `hdrLit` may display-map — the skybox used to apply
  Reinhard + gamma in-shader (a verbatim port of a GL quirk) and the tonemap
  then mapped it again, compressing the whole sky into a 0.13-wide grey wedge.

- The shader compile loop (`Engine/CMakeLists.txt:426`) invokes
  `glslangValidator -V` bare, which targets Vulkan 1.0. RT stages need
  `--target-env vulkan1.3` (SPIR-V ≥ 1.4). Bumping the whole loop needs a
  re-verify of the existing shaders.
- A shared GLSL include needs `GL_GOOGLE_include_directive` + `-I`, and CMake's
  `DEPENDS` won't track the include — add it explicitly or edits won't rebuild.
  The editor's runtime hot-reload (`RecompileSpirv`) needed the same `-I`; fixed
  in slice 3.
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
