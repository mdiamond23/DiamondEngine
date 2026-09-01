#version 450
#extension GL_GOOGLE_include_directive : require

// SSR march diagnostic. Re-runs the SAME ray ssr_resolve.frag's mirror path
// runs — same origin push, same constants, same TraceScreenRayDbg walk — and
// false-colours what the march did instead of what it saw.
//
// It draws OVER the tonemapped image, after everything. That is the point: a
// diagnostic routed through the composite, the RT merge, TAA and tonemap would
// be blended, reprojected and curve-remapped before it reached the screen, and
// the numbers on it would no longer be the numbers the march produced.
//
// Reading the modes, against the artifact this was built for (hard lines
// slicing through reflections, worst at grazing angles):
//
//   HitMiss    green = the march found a crossing, red = it did not. If the
//              slices are red, they are MISSES and the tolerance is rejecting
//              real geometry. If they are green, the march thinks it hit
//              something — go to HitDistance.
//   HitDistance how far the hit landed from the pixel that fired the ray, in
//              screen space. THE decisive one: a self-intersection hits
//              essentially at its own origin, so it reads black/near-zero.
//              Black lines through an otherwise bright field = the ray is
//              hitting the surface it started on.
//   Steps      which step index accepted the crossing, cycled through a ramp.
//              Slices that line up with step bands mean the artifact is step
//              QUANTISATION, not self-intersection, and the fix is stride, not
//              tolerance.
//   Overshoot  how far behind the surface the ray was when accepted, scaled by
//              the tolerance in force. Near 0 = the crossing was pinned
//              tightly; near 1 = it was accepted right at the edge of the
//              tolerance, which is where flicker comes from.
//   Slope      the measured per-step surface depth span — the slope-scaled part
//              of the tolerance. Shows directly where the grazing-angle term is
//              taking over, and whether it saturates at THICKNESS (white).

#include "screen_trace.glsl"

layout(location = 0) in  vec2 vUV;
layout(location = 0) out vec4 FragColor;

layout(set = 0, binding = 0) uniform sampler2D gViewPos;
layout(set = 0, binding = 1) uniform sampler2D gViewNormal;
layout(set = 0, binding = 2) uniform sampler2D gMaterial;
layout(set = 0, binding = 4) uniform sampler2D linearDepth;
layout(set = 0, binding = 5) uniform sampler2D coarseDepth;

layout(set = 0, binding = 3) uniform SSRDebugUBO {
    mat4  projection;
    vec2  fullSize;
    int   mode;          // SSRDebugMode
    float roughnessMax;  // pixels rougher than this are not traced (dimmed out)
} ubo;

// Must MATCH ssr_resolve.frag's mirror path. If these drift the view stops
// describing the thing it is meant to be describing.
const int   MIN_STEPS       = 24;
const int   MAX_STEPS       = 128;
const float PIXELS_PER_STEP = 8.0;
const float MAX_DISTANCE = 20.0;
const float THICKNESS    = 0.6;
const float BIAS         = 0.05;
const int   REFINE_STEPS = 6;
const float NORMAL_PUSH  = 0.05;

const int MODE_HITMISS     = 1;
const int MODE_HITDISTANCE = 2;
const int MODE_STEPS       = 3;
const int MODE_OVERSHOOT   = 4;
const int MODE_SLOPE       = 5;

// MISSES ARE MAGENTA, in every mode, and nothing in the ramp below can produce
// magenta. The first version of this view drew misses black while the ramp's own
// zero end was near-black, so a dark streak was either "the ray hit at zero
// distance" (a self-intersection) or "the ray found nothing" — two states that
// call for opposite fixes, rendered in indistinguishable colours.
const vec3 kMiss = vec3(1.0, 0.0, 1.0);

// A miss that ran the WHOLE segment on screen is a different fact from a miss
// whose ray left the frame. The first means geometry was there to be found and
// the march did not find it; the second is screen-space reflection working as
// designed, with nothing to reflect. Orange vs magenta keeps them apart.
const vec3 kMissOffscreen = vec3(1.0, 0.5, 0.0);

// Perceptually ordered ramp so magnitude reads without a legend:
// deep blue -> blue -> cyan -> green -> yellow -> red -> white. Starts at a
// VISIBLY blue floor, never black, so "hit at distance ~0" cannot be mistaken
// for "no hit".
vec3 Heat(float t) {
    t = clamp(t, 0.0, 1.0);
    vec3 c = mix(vec3(0.15, 0.15, 0.55), vec3(0.0, 0.4, 1.0), smoothstep(0.00, 0.20, t));
    c = mix(c, vec3(0.0, 1.0, 1.0), smoothstep(0.20, 0.40, t));
    c = mix(c, vec3(0.0, 1.0, 0.0), smoothstep(0.40, 0.60, t));
    c = mix(c, vec3(1.0, 1.0, 0.0), smoothstep(0.60, 0.75, t));
    c = mix(c, vec3(1.0, 0.0, 0.0), smoothstep(0.75, 0.90, t));
    c = mix(c, vec3(1.0, 1.0, 1.0), smoothstep(0.90, 1.00, t));
    return c;
}

void main() {
    const ivec2 id = ivec2(gl_FragCoord.xy);

    // Untraced pixels are drawn dark grey rather than black, so the shape of
    // the traced REGION stays visible and a miss inside it is distinguishable
    // from a pixel the march never considered.
    const vec3 kUntraced = vec3(0.06);

    vec3 P = texelFetch(gViewPos, id, 0).xyz;
    if (P.z == 0.0) { FragColor = vec4(kUntraced, 1.0); return; }

    float roughness = texelFetch(gMaterial, id, 0).g;
    if (roughness > ubo.roughnessMax) { FragColor = vec4(kUntraced, 1.0); return; }

    vec3  normal = texelFetch(gViewNormal, id, 0).xyz;
    float nLenSq = dot(normal, normal);
    if (nLenSq <= 1e-6) { FragColor = vec4(kUntraced, 1.0); return; }
    vec3 N = normal * inversesqrt(nLenSq);

    vec3 V = normalize(-P);
    vec3 R = reflect(-V, N);

    // Same rejection the resolve applies before it ever marches.
    if (-R.z <= 0.0) { FragColor = vec4(kUntraced, 1.0); return; }

    vec3 origin = P + N * (NORMAL_PUSH * max(1.0, abs(P.z) / 25.0));

    vec2             hitUV;
    ScreenTraceDebug dbg;
    bool hit = TraceScreenRayDbg(linearDepth, coarseDepth, ubo.projection, origin, R,
                                 MAX_DISTANCE, MIN_STEPS, MAX_STEPS,
                                 PIXELS_PER_STEP, REFINE_STEPS, THICKNESS,
                                 BIAS, hitUV, dbg);

    // tExit is the fraction of the ray that was on screen. At ~1 the march
    // walked its whole budget inside the frame, so a miss there is the march's
    // own doing; below 1 the ray ran out of screen first.
    const vec3 missColor = (dbg.tExit >= 0.99) ? kMiss : kMissOffscreen;

    vec3 c;
    if (ubo.mode == MODE_HITMISS) {
        c = hit ? vec3(0.1, 0.9, 0.2) : missColor;
    } else if (ubo.mode == MODE_HITDISTANCE) {
        // Normalised against a fifth of the screen: reflections that travel
        // further than that are rare, and the interesting signal is all down at
        // the near-zero end where self-intersections live.
        c = hit ? Heat(length(hitUV - vUV) / 0.2) : missColor;
    } else if (ubo.mode == MODE_STEPS) {
        c = hit ? Heat(float(dbg.step) / float(max(dbg.stepCount, 1))) : missColor;
    } else if (ubo.mode == MODE_OVERSHOOT) {
        c = hit ? Heat(dbg.overshoot / max(dbg.eps, 1e-6) * 0.25) : missColor;
    } else if (ubo.mode == MODE_SLOPE) {
        c = hit ? Heat(dbg.slope / THICKNESS) : missColor;
    } else {
        c = kUntraced;
    }

    FragColor = vec4(c, 1.0);
}
