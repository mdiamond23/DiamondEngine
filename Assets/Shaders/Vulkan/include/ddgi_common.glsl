#ifndef DDGI_COMMON_GLSL
#define DDGI_COMMON_GLSL

// Shared DDGI math (Docs/gi-design.md slice 3): probe grid addressing, the
// octahedral atlas layout, the ray direction set, and the irradiance lookup.
//
// Included by the RT probe trace (vulkan1.3 target), the blend/relocation
// compute passes and the probe debug viz, so nothing here may use stage-specific
// syntax. Every sampler is a function parameter for the same reason — the
// bindings differ per shader.
//
// Both atlases are sized for the MAXIMUM grid, not the live one: graph textures
// are declared once at graph-build time, and probe counts are live-editable in
// the inspector. A smaller grid simply uses the top-left sub-rect.

const float DDGI_PI = 3.14159265359;

// Octahedral resolution per probe, excluding the 1px border. Irradiance is
// low-frequency and cheap; visibility carries the Chebyshev depth/depth² pair
// and needs the extra angular resolution to resolve a wall's edge.
const int DDGI_IRRADIANCE_RES = 8;
const int DDGI_VISIBILITY_RES = 16;

// Hard caps the atlases are allocated against. 16³ probes and 128 rays/probe.
const int DDGI_MAX_PROBES_PER_AXIS = 16;
const int DDGI_MAX_PROBES          = 4096;
const int DDGI_MAX_RAYS            = 128;

// Rays [0, DDGI_FIXED_RAYS) of every probe are traced along a FIXED, unrotated
// Fibonacci set and are read only by relocation and classification. The rest
// carry the per-frame random rotation and are the only ones the two blend
// passes integrate.
//
// The split exists because relocation is a feedback loop: it reads this frame's
// rays and writes an offset the next frame traces from. Feeding it the rotated
// set gave that loop a different input every frame, so a probe in a perfectly
// good position still computed a non-zero correction and kept moving — there
// was no position at which it could come to rest. Every probe therefore jittered
// forever, and because the shading-time Chebyshev term is cubed, sub-percent
// jitter in probe position showed up as visible flicker across whole surfaces.
//
// With a fixed set the input is identical frame to frame for static geometry,
// so a settled probe computes a correction of exactly zero and stops dead.
// Mirrored by VulkanDDGIPass::kFixedRays, which is also what forces the live
// ray count to at least 2x this so the blend set never runs empty.
const int DDGI_FIXED_RAYS          = 32;

// std140. Mirrored by VulkanDDGIPass::DDGIUBO — keep the two in step.
struct DDGIVolume {
    mat4  view;         // world -> view; direct lighting reuses the view-space LightingUBO
    mat4  viewProj;     // probe debug billboards
    mat4  rayRotation;  // per-frame random rotation of the Fibonacci ray set
    vec4  origin;       // xyz = world position of probe (0,0,0)
    vec4  spacing;      // xyz = probe spacing, w = min spacing
    ivec4 counts;       // xyz = probe counts, w = total probes
    vec4  params;       // x hysteresis, y normalBias, z energy, w maxRayDistance
    vec4  params2;      // x skyIntensity, y viewBias, z depthSharpness, w rayCount
    vec4  params3;      // x historyValid, y backfaceThreshold, z debugRadius, w debugExposure
};

// What a probe ray brings back. 'hitT' is NEGATIVE on a backface hit — the
// signal relocation/classification use to detect a probe stuck inside geometry,
// and the reason the irradiance blend skips those rays.
struct DDGIPayload {
    vec3  radiance;
    float hitT;
};

// ── Probe addressing ─────────────────────────────────────────────────────────

ivec3 DDGIProbeCoord(int index, ivec3 counts) {
    int x = index % counts.x;
    int y = (index / counts.x) % counts.y;
    int z = index / (counts.x * counts.y);
    return ivec3(x, y, z);
}

int DDGIProbeIndex(ivec3 coord, ivec3 counts) {
    return coord.x + coord.y * counts.x + coord.z * counts.x * counts.y;
}

// Grid position before relocation. Callers add the probe's stored offset.
vec3 DDGIProbeGridPos(DDGIVolume v, ivec3 coord) {
    return v.origin.xyz + vec3(coord) * v.spacing.xyz;
}

// Row 0, one texel per probe: rgb = relocation offset, a = ACTIVATION WEIGHT in
// [0,1]. The weight is a ramp, not a flag — see DDGISampleIrradianceBent.
vec4 DDGIProbeData(sampler2D probeData, int index) {
    return texelFetch(probeData, ivec2(index, 0), 0);
}

// Row 1, x = classification state: >0.5 means the probe is inside geometry.
//
// This is kept separate from the activation ramp in row 0 precisely so the ramp
// can never feed back into the decision that drives it. Deriving "was this probe
// buried last frame?" from the ramp instead would let a probe whose backface
// ratio sits between the classifier's two thresholds flip state every time the
// ramp crossed 0.5 — reintroducing, at ramp frequency, exactly the oscillation
// the Schmitt trigger in ddgi_relocate.comp exists to prevent.
//
// Consumers that must act on the STATE (the two blend passes, which skip a
// buried probe's rays entirely) read this. Consumers that want the smooth weight
// (the irradiance lookup) read row 0's alpha.
bool DDGIProbeBuried(sampler2D probeData, int index) {
    return texelFetch(probeData, ivec2(index, 1), 0).x > 0.5;
}

// ── Octahedral mapping ───────────────────────────────────────────────────────
// Unit sphere <-> [-1,1]², the standard equal-area-ish octahedron unwrap. The
// seam is why every probe tile carries a 1px border.

vec2 DDGIOctEncode(vec3 d) {
    d /= (abs(d.x) + abs(d.y) + abs(d.z));
    if (d.z < 0.0) d.xy = (1.0 - abs(d.yx)) * vec2(d.x >= 0.0 ? 1.0 : -1.0,
                                                   d.y >= 0.0 ? 1.0 : -1.0);
    return d.xy;
}

vec3 DDGIOctDecode(vec2 f) {
    vec3 d = vec3(f.x, f.y, 1.0 - abs(f.x) - abs(f.y));
    if (d.z < 0.0) d.xy = (1.0 - abs(d.yx)) * vec2(d.x >= 0.0 ? 1.0 : -1.0,
                                                   d.y >= 0.0 ? 1.0 : -1.0);
    return normalize(d);
}

// ── Atlas layout ─────────────────────────────────────────────────────────────
// Probes tile the atlas as (x + y*countX) columns by z rows, each tile
// res+2 square. Sized for the max grid, so the column stride uses the LIVE
// countX — a shrunken grid just leaves the right side unused.

ivec2 DDGIProbeTile(int index, ivec3 counts) {
    ivec3 c = DDGIProbeCoord(index, counts);
    return ivec2(c.x + c.y * counts.x, c.z);
}

// Top-left texel of a probe's tile INCLUDING its border.
ivec2 DDGIProbeTileOrigin(int index, ivec3 counts, int res) {
    return DDGIProbeTile(index, counts) * (res + 2);
}

// UV for sampling a probe's octahedral map along 'dir'. Lands strictly inside
// the interior [1, res+1] so bilinear taps only ever reach the border texels,
// never the neighbouring probe.
vec2 DDGIProbeUV(int index, vec3 dir, ivec3 counts, int res, vec2 atlasSize) {
    vec2 oct   = DDGIOctEncode(normalize(dir)) * 0.5 + 0.5;
    vec2 texel = vec2(DDGIProbeTileOrigin(index, counts, res)) + 1.0 + oct * float(res);
    return texel / atlasSize;
}

// Border texels mirror the octahedron's wrap so bilinear filtering is seamless.
// 'local' is a tile-local coord in [0, res+1]; returns the interior coord to
// copy from, or 'local' itself when it is already interior.
ivec2 DDGIBorderSource(ivec2 local, int res) {
    bool left  = local.x == 0;
    bool right = local.x == res + 1;
    bool top   = local.y == 0;
    bool bot   = local.y == res + 1;

    // Corners wrap to the diagonally opposite interior corner.
    if ((left || right) && (top || bot))
        return ivec2(left ? res : 1, top ? res : 1);
    if (top)   return ivec2(res + 1 - local.x, 1);
    if (bot)   return ivec2(res + 1 - local.x, res);
    if (left)  return ivec2(1, res + 1 - local.y);
    if (right) return ivec2(res, res + 1 - local.y);
    return local;
}

// ── Ray directions ───────────────────────────────────────────────────────────
// Spherical Fibonacci: a near-uniform sphere cover for any ray count. The trace
// and its consumers MUST agree exactly on every direction, which is the reason
// this lives here.

vec3 DDGISphericalFibonacci(float i, float n) {
    const float phi = 2.0 * DDGI_PI * fract(i * 0.618033988749895);
    float cosTheta = 1.0 - (2.0 * i + 1.0) / n;
    float sinTheta = sqrt(clamp(1.0 - cosTheta * cosTheta, 0.0, 1.0));
    return vec3(cos(phi) * sinTheta, sin(phi) * sinTheta, cosTheta);
}

// The relocation set: rays [0, DDGI_FIXED_RAYS), deliberately NOT rotated, and
// spread over their own count so they cover the sphere on their own. Depends on
// nothing per-frame, which is the entire point — see DDGI_FIXED_RAYS above.
vec3 DDGIFixedRayDirection(int rayIndex) {
    return DDGISphericalFibonacci(float(rayIndex), float(DDGI_FIXED_RAYS));
}

// The integration set: rays [DDGI_FIXED_RAYS, rayCount), re-indexed from zero so
// they too cover the sphere on their own, then rotated per frame so the pattern
// decorrelates across frames instead of baking into the atlas.
//
// The blend passes must skip the fixed rays rather than fold them in: those 32
// directions never move, so including them would stamp a permanent 32-lobe
// pattern into every probe's octahedral map that no amount of temporal
// accumulation can average away.
vec3 DDGIBlendRayDirection(DDGIVolume v, int rayIndex) {
    const float n = max(v.params2.w - float(DDGI_FIXED_RAYS), 1.0);
    vec3 d = DDGISphericalFibonacci(float(rayIndex - DDGI_FIXED_RAYS), n);
    return normalize(mat3(v.rayRotation) * d);
}

// What the trace uses: one call site that owns the whole [0, rayCount) range and
// hands each half to the right generator.
vec3 DDGIRayDirection(DDGIVolume v, int rayIndex) {
    return rayIndex < DDGI_FIXED_RAYS ? DDGIFixedRayDirection(rayIndex)
                                      : DDGIBlendRayDirection(v, rayIndex);
}

// ── Irradiance lookup ────────────────────────────────────────────────────────
// Trilinear over the 8 surrounding probes, each weighted by a smooth backface
// term and the paper's Chebyshev visibility test. Returns E/pi, matching what
// irradiance_convolution.frag stores — so it is a drop-in for the IBL
// irradiance sample in deferred_lighting.frag (slice 4).
//
// Two normals (slice 4.5). 'bentNormal' is the direction the octahedral map is
// FETCHED along — GTAO's average unoccluded direction, so the probe lookup is
// directionally occluded rather than fetched down the geometric normal and
// scaled. 'normal' stays geometric and drives the two things that must respect
// the actual surface: the normal-bias push-off, and the backface weight that
// rejects probes behind the shading point. A bent normal lying nearly along a
// wall would otherwise push the sample point straight into it.
//
// Callers still multiply by the visibility scalar. The exact treatment would
// narrow the cosine lobe to the visibility cone's aperture, which this atlas
// cannot express — it stores one fixed hemispherical convolution.

vec3 DDGISampleIrradianceBent(DDGIVolume v,
                              sampler2D irradianceAtlas, sampler2D visibilityAtlas,
                              sampler2D probeData,
                              vec3 worldPos, vec3 normal, vec3 bentNormal,
                              vec3 viewDir)
{
    const vec2 irrSize = vec2(textureSize(irradianceAtlas, 0));
    const vec2 visSize = vec2(textureSize(visibilityAtlas, 0));

    // Push the sample point off the surface along the normal (self-shadowing)
    // and toward the viewer (grazing angles), the paper's two bias terms.
    vec3 biased = worldPos + normal * v.params.y + viewDir * v.params2.y;

    vec3  gridF = (biased - v.origin.xyz) / v.spacing.xyz;
    ivec3 base  = ivec3(floor(gridF));
    vec3  alpha = clamp(gridF - vec3(base), vec3(0.0), vec3(1.0));

    vec3  sum  = vec3(0.0);
    float wSum = 0.0;

    for (int i = 0; i < 8; ++i) {
        ivec3 off = ivec3(i & 1, (i >> 1) & 1, (i >> 2) & 1);
        ivec3 c   = clamp(base + off, ivec3(0), v.counts.xyz - 1);
        int   idx = DDGIProbeIndex(c, v.counts.xyz);

        vec4 pd = DDGIProbeData(probeData, idx);
        // Clamped, not trusted: on the opening frames the lighting pass can read
        // this texture before relocation has ever written it, and an unbounded
        // value here would let one garbage probe dominate the normalized sum.
        const float activation = clamp(pd.w, 0.0, 1.0);
        if (activation <= 0.0) continue;   // fully faded out — inside geometry

        vec3  probePos = DDGIProbeGridPos(v, c) + pd.xyz;
        vec3  toProbe  = probePos - biased;
        float dist     = length(toProbe);
        vec3  dirP     = dist > 1e-5 ? toProbe / dist : normal;

        // Trilinear weight of this corner.
        vec3  tri    = mix(1.0 - alpha, alpha, vec3(off));
        float weight = tri.x * tri.y * tri.z;

        // Activation ramp. Classification is binary, but dropping a probe out of
        // the interpolation the instant it flips is a step change in wSum, and
        // it applies at once to every surface the probe trilinearly reaches — a
        // whole wall visibly jumping brightness on one frame. Relocation ramps
        // this over ~16 frames instead, turning that step into a fade.
        weight *= activation;

        // Smooth backface term: a probe behind the shading surface is wrong, but
        // hard-rejecting it makes the interpolation pop. Squared+shifted cosine.
        float backface = dot(dirP, normal) * 0.5 + 0.5;
        weight *= backface * backface + 0.2;

        // Chebyshev: how likely this probe can actually see the shading point,
        // from the mean/mean² depth pair stored along the probe->point direction.
        vec2  vis  = texture(visibilityAtlas,
                             DDGIProbeUV(idx, -dirP, v.counts.xyz,
                                         DDGI_VISIBILITY_RES, visSize)).rg;
        float mean = vis.x;
        if (dist > mean) {
            float variance  = abs(mean * mean - vis.y);
            float d         = dist - mean;
            float chebyshev = variance / (variance + d * d);
            weight *= max(chebyshev * chebyshev * chebyshev, 0.0);
        }

        // Crush near-zero weights so a barely-contributing probe can't dominate
        // once the whole set is normalized (paper §4.2).
        const float crush = 0.2;
        if (weight < crush) weight *= (weight * weight) / (crush * crush);

        vec3 E = texture(irradianceAtlas,
                         DDGIProbeUV(idx, bentNormal, v.counts.xyz,
                                     DDGI_IRRADIANCE_RES, irrSize)).rgb;
        sum  += E * weight;
        wSum += weight;
    }

    if (wSum <= 1e-6) return vec3(0.0);
    // RAW irradiance — the 'energy' multiplier is deliberately NOT applied here.
    // This function is called both by the final lighting AND by the probe trace's
    // recursive bounce, which writes back into the atlas this reads. Folding a
    // gain in here put it inside that feedback loop: the loop gain becomes
    // energy * albedo, so any energy above ~1/albedo makes probe irradiance grow
    // every frame until it saturates. Callers that want the artistic gain apply
    // it themselves, outside the loop.
    return sum / wSum;
}

// Geometric-normal lookup — bent normal == shading normal. What the probe
// trace's recursive bounce uses: it has no G-buffer and therefore no bent
// normal, and a second bounce does not need one.
vec3 DDGISampleIrradiance(DDGIVolume v,
                          sampler2D irradianceAtlas, sampler2D visibilityAtlas,
                          sampler2D probeData,
                          vec3 worldPos, vec3 normal, vec3 viewDir)
{
    return DDGISampleIrradianceBent(v, irradianceAtlas, visibilityAtlas, probeData,
                                    worldPos, normal, normal, viewDir);
}

#endif // DDGI_COMMON_GLSL
