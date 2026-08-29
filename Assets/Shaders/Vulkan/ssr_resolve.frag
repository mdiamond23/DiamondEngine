#version 450
#extension GL_GOOGLE_include_directive : require

// Resolves the reduced-resolution SSR trace (ssr.frag) to full resolution, and
// writes the SAME contract the old full-res trace wrote — rgb = reflected scene
// radiance, a = hit confidence — so ssr_composite.frag and the RT-reflection
// merge downstream are untouched by the resolution split.
//
// TWO PATHS, PICKED BY ROUGHNESS. This is the part that matters.
//
//   roughness < SHARP_BEGIN  — a mirror. The reflected image carries detail no
//     quarter-density ray set can represent, so the ray is re-traced HERE, per
//     full-res pixel. Mirrors stay pixel-exact; they are also a small minority
//     of pixels in any real scene, so the cost is close to nothing.
//
//   roughness > SHARP_END    — the material's own GGX lobe is already far wider
//     than the ~2-texel spread between neighbouring low-res rays, so reusing
//     them is invisible. This is where the 4x saving comes from.
//
// The reuse is NOT a bilateral upsample of colour. Blending reflected colour
// between pixels is what made the previous attempt look like mush: depth and
// normal say "same surface", but two pixels on one flat wall reflect entirely
// different things. Instead each of the four candidate low-res rays is turned
// back into a DIRECTION — from THIS pixel to the point that ray hit — and
// weighted by this pixel's own GGX distribution around its own reflection
// vector. A ray that hit something this pixel could not plausibly see gets ~0
// weight, whatever its depth and normal say. The colour is then fetched from
// the FULL-res scene at the winning hit UV, so the reflected image is never
// downsampled, only the choice of where to look.
//
// Hit UVs arrive through an RGBA16F texture, stored RELATIVE to the low-res
// pixel that traced them (see ssr.frag) and re-absolutised here. Absolute UVs
// would carry only ~2 full-res texels of precision at 4K, and the damage from
// that is not blur, it is SNAPPING: a half float cannot represent the
// in-between values, so a smoothly moving camera makes the sampled point jump
// in steps instead of sliding, and the reflected image glitters. The mirror
// path never round-trips a UV through memory at all.

#include "screen_trace.glsl"

layout(location = 0) in  vec2 vUV;
layout(location = 0) out vec4 FragColor;

layout(set = 0, binding = 0) uniform sampler2D ssrHit;       // LOW res: rg = hit UV, b = confidence
layout(set = 0, binding = 1) uniform sampler2D gViewPos;
layout(set = 0, binding = 2) uniform sampler2D gViewNormal;
layout(set = 0, binding = 3) uniform sampler2D gMaterial;    // g = roughness
layout(set = 0, binding = 4) uniform sampler2D sceneColor;
// Linear view depth (positive, 0 = sky) — depth_pyramid.comp level 0, what the
// mirror path's march taps. gViewPos stays bound for the per-pixel ray setup
// and for reconstructing where a reused ray landed.
layout(set = 0, binding = 6) uniform sampler2D linearDepth;
// Min-reduced level of the same pyramid — the march's conservative reject
// filter. See TraceScreenRay.
layout(set = 0, binding = 7) uniform sampler2D coarseDepth;

layout(set = 0, binding = 5) uniform SSRUBO {
    mat4 projection;
    vec2 fullSize;
    vec2 traceSize;
} ubo;

const float PI = 3.14159265359;

// Roughness band over which the mirror trace hands off to ray reuse. A hard
// switch would draw a visible seam wherever a roughness map crosses it; the
// band costs both paths only for the pixels actually inside it.
const float SHARP_BEGIN = 0.10;
const float SHARP_END   = 0.16;

// Matches ssr.frag — above this ssr_composite has already faded to prefiltered
// IBL, so neither path could contribute.
const float ROUGHNESS_CUTOFF = 0.9;

// Identical march parameters to ssr.frag, so a pixel inside the handoff band
// gets the same geometry from both paths.
const int   MAX_STEPS    = 24;
const float MAX_DISTANCE = 20.0;
const float THICKNESS    = 0.6;
const float BIAS         = 0.05;
const int   REFINE_STEPS = 6;

// A neighbour's ray is only reusable if it left a surface facing roughly the
// way this one does. Without this the reuse silently mixes rays fired off
// completely different normals.
const float ORIGIN_NORMAL_GATE = 0.9;

// How far down the GGX lobe a candidate may sit and still count. Below this it
// is rejected outright rather than contributing a tiny weight.
const float MIN_LOBE_FRACTION  = 0.05;

// Reuse the four low-res rays whose footprints cover this pixel. Returns
// rgb = reflected radiance, a = confidence, and sets 'valid' false when not one
// of the four turned out to be reusable — the caller then traces its own ray
// rather than inventing an answer out of rays that do not apply here.
vec4 ResolveReusedRays(vec3 P, vec3 N, vec3 V, float roughness, out bool valid) {
    valid = false;
    vec2 lowSize = vec2(textureSize(ssrHit, 0));
    vec2 coord   = vUV * lowSize - 0.5;
    vec2 base    = floor(coord);
    vec2 f       = coord - base;

    // Bilinear footprint weights — a tie-break between rays the BRDF weight
    // below rates equally, not a filter in their own right.
    float bw[4] = float[4]((1.0 - f.x) * (1.0 - f.y),
                                  f.x  * (1.0 - f.y),
                           (1.0 - f.x) *        f.y,
                                  f.x  *        f.y);
    ivec2 offs[4] = ivec2[4](ivec2(0, 0), ivec2(1, 0), ivec2(0, 1), ivec2(1, 1));

    float a  = max(roughness * roughness, 1e-3);   // GGX alpha
    float a2 = a * a;

    // D at perfect alignment. Weights are judged against this, so "every
    // candidate is far outside my lobe" is a state the loop can DETECT rather
    // than one it silently averages through.
    float dPeak = 1.0 / (PI * a2);

    vec3  colSum  = vec3(0.0);
    float confSum = 0.0;
    float wSum    = 0.0;
    float keptBw  = 0.0;   // footprint fraction that produced a USABLE ray

    for (int i = 0; i < 4; ++i) {
        ivec2 t = clamp(ivec2(base) + offs[i], ivec2(0), ivec2(lowSize) - 1);
        vec4  h = texelFetch(ssrHit, t, 0);
        if (h.z <= 0.0) continue;                       // that ray missed

        // ── Gate on the ray's ORIGIN, not just on where it ended up ─────────
        // A neighbour's ray is only reusable if it left a surface like this
        // one. On anything with a high-frequency normal map — the diamond plate
        // is the worst case — the normal swings hard between adjacent texels,
        // so a neighbour's reflection vector points somewhere completely
        // unrelated and its result is not an approximation of ours, it is a
        // different question's answer. Nothing downstream can recover from
        // mixing those, so they are rejected here.
        vec2  srcUV = (vec2(t) + 0.5) / lowSize;
        ivec2 src   = clamp(ivec2(srcUV * ubo.fullSize),
                            ivec2(0), ivec2(ubo.fullSize) - 1);

        vec3 sp = texelFetch(gViewPos, src, 0).xyz;
        if (sp.z == 0.0) continue;
        if (abs(sp.z - P.z) > 0.05 * abs(P.z)) continue;

        vec3  sn     = texelFetch(gViewNormal, src, 0).xyz;
        float snLen2 = dot(sn, sn);
        if (snLen2 <= 1e-6) continue;
        if (dot(sn * inversesqrt(snLen2), N) < ORIGIN_NORMAL_GATE) continue;

        // Re-absolutise: the trace stored the offset from its OWN pixel centre.
        vec2 hitUV = h.xy + srcUV;

        // Where the ray landed, in view space. Reading it back from the
        // G-buffer rather than storing a distance keeps the low-res target at
        // four channels and costs one tap. texelFetch, not a filtered sample: a
        // hit UV that lands on a silhouette would otherwise return a position
        // interpolated between the two surfaces and skew the weight below.
        ivec2 hitTexel = clamp(ivec2(hitUV * ubo.fullSize),
                               ivec2(0), ivec2(ubo.fullSize) - 1);
        vec3  H        = texelFetch(gViewPos, hitTexel, 0).xyz;
        if (H.z >= 0.0) continue;                       // no geometry there any more

        vec3  d   = H - P;
        float dl2 = dot(d, d);
        if (dl2 < 1e-8) continue;
        vec3 L = d * inversesqrt(dl2);

        // How much THIS pixel's lobe would weight radiance arriving from L.
        vec3  Hv  = normalize(V + L);
        float NoH = max(dot(N, Hv), 0.0);
        float den = NoH * NoH * (a2 - 1.0) + 1.0;
        float D   = a2 / max(PI * den * den, 1e-8);

        // Out in the tail of the lobe is a REJECTION, not a small weight. The
        // previous version added an epsilon to every weight so the sum could
        // never be zero; when all four candidates were far off-lobe that
        // epsilon became the ONLY term, and four unrelated reflections got
        // averaged with equal weight at full confidence — the exact colour
        // mush the GGX weighting exists to prevent, produced by the guard
        // meant to keep it safe.
        if (D < dPeak * MIN_LOBE_FRACTION) continue;

        float w = bw[i] * D;

        colSum  += textureLod(sceneColor, hitUV, 0.0).rgb * w;
        confSum += h.z * w;
        wSum    += w;
        keptBw  += bw[i];
    }

    if (wSum <= 0.0) return vec4(0.0);   // valid stays false — caller traces

    valid = true;
    // keptBw (the bilinear weights sum to 1) fades confidence where only part
    // of the footprint produced a usable ray, which keeps the boundary of a
    // reflected object soft instead of stair-stepped at the low-res grid.
    return vec4(colSum / wSum, (confSum / wSum) * keptBw);
}

void main() {
    ivec2 id = ivec2(gl_FragCoord.xy);

    vec3 P = texelFetch(gViewPos, id, 0).xyz;
    if (P.z == 0.0) { FragColor = vec4(0.0); return; }   // background

    vec3  normal = texelFetch(gViewNormal, id, 0).xyz;
    float nLenSq = dot(normal, normal);
    if (nLenSq <= 1e-6) { FragColor = vec4(0.0); return; }
    vec3 N = normal * inversesqrt(nLenSq);

    float roughness = texelFetch(gMaterial, id, 0).g;
    if (roughness > ROUGHNESS_CUTOFF) { FragColor = vec4(0.0); return; }

    vec3 V = normalize(-P);
    vec3 R = reflect(-V, N);

    // Rays bending back toward the camera march into space the screen never
    // captured — fade them out rather than show false hits.
    float dirFade = smoothstep(0.0, 0.15, -R.z);
    if (dirFade <= 0.0) { FragColor = vec4(0.0); return; }

    float sharp = 1.0 - smoothstep(SHARP_BEGIN, SHARP_END, roughness);

    bool reuseValid = false;
    vec4 reused     = vec4(0.0);
    if (sharp < 1.0)
        reused = ResolveReusedRays(P, N, V, roughness, reuseValid);

    // Trace this pixel's OWN ray when it is a mirror — and also whenever reuse
    // came up empty. That fallback is what makes the resolution cut adaptive
    // rather than a gamble: a smooth surface keeps the 4x saving because its
    // neighbours' rays genuinely apply to it, while a violently normal-mapped
    // one pays full price and stays correct. The alternative is reusing rays
    // that do not describe this pixel, which is where the flickering blur came
    // from.
    float traceW = reuseValid ? sharp : 1.0;

    vec4 mirror = vec4(0.0);
    if (traceW > 0.0) {
        vec2 hitUV;
        if (TraceScreenRay(linearDepth, coarseDepth, ubo.projection, P, R,
                           MAX_DISTANCE, MAX_STEPS, REFINE_STEPS,
                           THICKNESS, BIAS, hitUV))
            mirror = vec4(textureLod(sceneColor, hitUV, 0.0).rgb, EdgeFade(hitUV));
    }

    // Cross-fade in PREMULTIPLIED space. Mixing the colours directly would let a
    // miss on one path (rgb 0, confidence 0) darken the other path's hit.
    vec3  num  = mix(reused.rgb * reused.a, mirror.rgb * mirror.a, traceW);
    float conf = mix(reused.a, mirror.a, traceW);

    FragColor = vec4(conf > 1e-5 ? num / conf : vec3(0.0), conf * dirFade);
}
