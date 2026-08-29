#version 450
#extension GL_GOOGLE_include_directive : require

// Screen-space reflection TRACE, at a REDUCED resolution (half by default — the
// target's size is what decides; nothing here assumes a ratio).
//
// This pass no longer produces reflection colour. It produces, per low-res
// pixel, the SCREEN UV ITS MIRROR RAY LANDED ON. ssr_resolve.frag then fetches
// the lit scene at that UV per FULL-res pixel. That split is the whole point of
// the resolution cut: what gets traced at low resolution is the ray, which is a
// smooth function of the surface; what stays at full resolution is the fetch of
// the reflected image, which is not. An earlier attempt traded a half-res trace
// for a bilateral upsample OF THE COLOUR and looked like mush — a depth+normal
// filter reads two pixels on the same flat wall as "same surface, safe to
// blend" when the two things they reflect are completely unrelated. Nothing
// here ever blends reflected colour across pixels; see the resolve for how the
// four candidate rays are weighted instead.
//
// The march itself is the shared screen_trace.glsl walk (uniform steps in
// SCREEN space, two projections total), the same one SSGI uses. The old
// hand-rolled loop in this file projected once per step and stepped uniformly in
// VIEW space, which clustered its taps near the origin.
//
// out: rg = hit UV RELATIVE TO THIS PIXEL, b = edge-fade confidence (0 = no
// usable hit), a = unused.
//
// Relative, because the target is RGBA16F and half floats have RELATIVE
// precision. An absolute UV near 1.0 resolves to about 2^-11, which at 4K is
// nearly two full-res texels — so as the camera moved, a hit point would not
// slide, it would SNAP between representable values, and the reflected image
// would visibly jump every few frames. Reflection offsets are small (a hit is
// usually within a fraction of the screen of its own pixel), and at 0.05 a half
// float resolves to about 3e-5, a tenth of a texel. Same storage, same
// bandwidth, roughly twenty times the precision where it is actually needed.
// The view-direction fade is deliberately NOT folded in here — it belongs to the
// full-res pixel's own reflection vector, so the resolve applies its own.

#include "screen_trace.glsl"

layout(location = 0) in  vec2 vUV;
layout(location = 0) out vec4 FragColor;

layout(set = 0, binding = 0) uniform sampler2D gViewPos;      // FULL res
layout(set = 0, binding = 1) uniform sampler2D gViewNormal;   // FULL res
layout(set = 0, binding = 2) uniform sampler2D gMaterial;     // FULL res, g = roughness
// Linear view depth (positive, 0 = sky) — depth_pyramid.comp level 0. What the
// march taps; gViewPos above is only read for the per-pixel ray setup.
layout(set = 0, binding = 4) uniform sampler2D linearDepth;
// Min-reduced level of the same pyramid — the march's conservative reject
// filter. See TraceScreenRay.
layout(set = 0, binding = 5) uniform sampler2D coarseDepth;

layout(set = 0, binding = 3) uniform SSRUBO {
    mat4 projection;
    vec2 fullSize;    // G-buffer resolution — this pass reads it at full res
    vec2 traceSize;   // this target's resolution
} ubo;

// 24 rather than the 16 the full-res version ran: the trace now covers a
// quarter of the pixels, so steps are the cheap axis again. Steps buy REACH
// (each one is MAX_DISTANCE/steps of view-space travel), which is exactly what
// the earlier 32 -> 16 cut spent to pay for full-res tracing.
const int   MAX_STEPS    = 24;
const float MAX_DISTANCE = 20.0;   // view-space units the ray may travel
const float THICKNESS    = 0.6;    // how "deep" a surface counts as solid
const float BIAS         = 0.05;   // self-intersection guard at the ray origin
const int   REFINE_STEPS = 6;      // SSR needs the hit pinned — it picks a pixel

// Above this no traced hit can survive ssr_composite's roughness fade, so the
// march would be pure waste. Held ABOVE that fade's end (0.85) on purpose: a
// full-res pixel just under the fade still needs its low-res neighbours to
// carry data, and those neighbours sample a different texel's roughness.
const float ROUGHNESS_CUTOFF = 0.9;

void main() {
    // Trace from an EXACT full-res texel, never a filtered one. vUV is this
    // low-res pixel's centre, so this lands on a real texel inside its
    // footprint; sampling gViewPos bilinearly instead would interpolate
    // position and normal ACROSS silhouettes and fire rays off surfaces that do
    // not exist.
    const ivec2 src = clamp(ivec2(vUV * ubo.fullSize),
                            ivec2(0), ivec2(ubo.fullSize) - 1);

    const vec3 fragPos = texelFetch(gViewPos, src, 0).xyz;
    if (fragPos.z == 0.0) { FragColor = vec4(0.0); return; }   // background

    if (texelFetch(gMaterial, src, 0).g > ROUGHNESS_CUTOFF) {
        FragColor = vec4(0.0);
        return;
    }

    vec3  normal = texelFetch(gViewNormal, src, 0).xyz;
    float nLenSq = dot(normal, normal);
    if (nLenSq <= 1e-6) { FragColor = vec4(0.0); return; }
    const vec3 N = normal * inversesqrt(nLenSq);

    // Camera sits at the view-space origin, so the frag position doubles as the
    // incident ray direction.
    const vec3 reflected = reflect(normalize(fragPos), N);

    // Rays bending back toward the camera march into space the screen never
    // captured. The resolve fades these smoothly; here they are simply dropped.
    if (reflected.z >= 0.0) { FragColor = vec4(0.0); return; }

    vec2 hitUV;
    if (!TraceScreenRay(linearDepth, coarseDepth, ubo.projection, fragPos, reflected,
                        MAX_DISTANCE, MAX_STEPS, REFINE_STEPS,
                        THICKNESS, BIAS, hitUV)) {
        FragColor = vec4(0.0);
        return;
    }

    FragColor = vec4(hitUV - vUV, EdgeFade(hitUV), 0.0);
}
