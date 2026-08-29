#ifndef SCREEN_TRACE_GLSL
#define SCREEN_TRACE_GLSL

// Shared view-space screen march, factored out so SSGI and SSR can't drift
// apart. Everything is VIEW space: camera at the origin, -z forward.
//
// The march reads the LINEAR DEPTH PYRAMID's level 0 (depth_pyramid.comp), not
// gViewPos, and reads it with texelFetch. Both halves of that matter, and
// together they are the single largest lever on this loop:
//
//  - R16F is 2 bytes per texel against gViewPos's 8. At 4K that is a 16 MB
//    working set instead of 66 MB — the difference between a tap loop that
//    lives in L2 and one that goes to VRAM for essentially every tap. Screen
//    traces scatter by construction (every ray walks its own direction), so
//    the hit rate IS the performance.
//  - texelFetch, not textureLod: a filtered fetch pulls four texels, i.e. up
//    to four cache sectors, to produce a depth that is WRONG anyway — a
//    bilinear blend across a silhouette is a surface that exists nowhere, and
//    this loop only ever compares against it.
//
// Depth here is POSITIVE and increases away from the camera (-viewZ), and 0
// means sky, matching depth_pyramid.comp's convention.
//
// Sampling is textureLod/texelFetch throughout, never texture() — implicit-LOD
// sampling is a fragment-only operation and this header is included by compute
// shaders.

// View space -> screen UV. The v flip is mandatory: the negative-height
// viewport writes NDC y=+1 into texel row 0.
vec2 ProjectToUV(mat4 proj, vec3 p) {
    vec4 clip = proj * vec4(p, 1.0);
    clip.xy /= clip.w;
    return vec2(0.5 * clip.x + 0.5, 0.5 - 0.5 * clip.y);
}

// 1 in the screen interior, 0 at the borders — hides the hard cutoff where
// rays leave the frame.
float EdgeFade(vec2 uv) {
    vec2 f = smoothstep(0.0, 0.1, min(uv, 1.0 - uv));
    return f.x * f.y;
}

// Point-sampled linear depth at a screen UV. 0 = sky.
float SampleLinearDepth(sampler2D linearDepth, vec2 uv) {
    ivec2 size = textureSize(linearDepth, 0);
    return texelFetch(linearDepth,
                      clamp(ivec2(uv * vec2(size)), ivec2(0), size - 1), 0).r;
}

// Screen-space march to the first depth crossing, then a short binary
// refinement. Returns true on hit and writes the hit's screen UV.
//
// 'refineSteps' is per-caller on purpose. SSR needs the hit pinned precisely —
// it decides which pixel of the reflected image is shown. SSGI does not: the
// hit UV only selects whose radiance to add into a cosine-weighted diffuse
// gather, where being a few pixels off is invisible, so it pays for two
// bisections instead of six. On a scene where most rays hit, those four taps
// are a quarter of the whole loop.
//
// The projection happens TWICE — once per endpoint — instead of once per step.
// A straight line in view space stays straight under a projective transform, so
// screen position is affine in a screen-space parameter, and 1/w is affine in
// that same parameter. Interpolating both is exact, not an approximation, and
// it removes a mat4 multiply from the innermost loop.
//
// It also samples BETTER: stepping uniformly in view space clusters taps near
// the origin in screen space and strides over distant geometry. Uniform screen
// steps put one tap every few pixels along the whole ray.
// 'coarseDepth' is a MIN-reduced level of the same pyramid (nearest surface
// over its footprint). It is a conservative reject filter, not a second march:
// if the nearest surface anywhere in a coarse texel is still in FRONT of the
// ray, then the level-0 texel under the ray — which lies inside that footprint —
// is in front of it too, so there is provably no crossing and the fine tap can
// be skipped. Every candidate the coarse level does report is confirmed against
// level 0 before it counts, so the hit this returns is bit-identical to marching
// level 0 alone. What changes is only which taps go to VRAM: at 4K level 2 is
// about 1 MB against level 0's 16 MB, and rays spend most of their steps in
// empty space where the coarse test alone answers.
//
// (This is not a full HiZ walk. That needs a mip chain down to ~1x1 so empty
// space is crossed in O(log n) cell steps; with this pyramid stopping at level 3
// a cell-stepping DDA would take far MORE iterations than the fixed step budget
// above, not fewer. The reject filter is the part of the idea that pays off at
// four levels.)
bool TraceScreenRay(sampler2D linearDepth, sampler2D coarseDepth,
                    mat4 proj, vec3 origin, vec3 dir,
                    float maxDistance, int steps, int refineSteps,
                    float thickness, float bias, out vec2 hitUV)
{
    hitUV = vec2(0.0);

    // Clip the segment to stay in front of the camera BEFORE projecting. With a
    // per-step projection a ray crossing the near plane produced garbage UVs
    // that the bounds test happened to reject; interpolated endpoints have no
    // such safety net, so a segment straddling w = 0 must be shortened here.
    const float kMinViewZ = -0.05;   // just in front of the camera
    float endT = maxDistance;
    if (origin.z + dir.z * endT > kMinViewZ) {
        if (dir.z <= 1e-6) return false;              // parallel or receding
        endT = (kMinViewZ - origin.z) / dir.z;
        if (endT <= 1e-4) return false;               // origin already at the plane
    }

    vec4 c0 = proj * vec4(origin, 1.0);
    vec4 c1 = proj * vec4(origin + dir * endT, 1.0);
    if (c0.w <= 0.0 || c1.w <= 0.0) return false;

    const vec2  uv0 = vec2(0.5 * c0.x / c0.w + 0.5, 0.5 - 0.5 * c0.y / c0.w);
    const vec2  uv1 = vec2(0.5 * c1.x / c1.w + 0.5, 0.5 - 0.5 * c1.y / c1.w);
    // w == -viewZ for a standard perspective matrix, so 1/w recovers the same
    // positive linear depth the pyramid stores.
    const float iw0 = 1.0 / c0.w;
    const float iw1 = 1.0 / c1.w;

    float prevT = 0.0;
    float hitT  = -1.0;

    for (int i = 1; i <= steps; ++i) {
        float t  = float(i) / float(steps);
        vec2  uv = mix(uv0, uv1, t);
        if (uv.x < 0.0 || uv.x > 1.0 || uv.y < 0.0 || uv.y > 1.0) return false;

        float rayD = 1.0 / mix(iw0, iw1, t);

        // Conservative reject. 0 means the whole footprint is sky (the
        // downsample only writes 0 when every child was sky), and a nearest
        // surface still in front of the ray means no crossing anywhere in the
        // footprint. Either way the fine tap is provably unnecessary.
        float coarseD = SampleLinearDepth(coarseDepth, uv);
        if (coarseD <= 0.0 || coarseD > rayD - bias) { prevT = t; continue; }

        float sceneD = SampleLinearDepth(linearDepth, uv);
        if (sceneD <= 0.0) { prevT = t; continue; }   // ray is over a sky pixel

        // The ray is behind stored geometry once its depth passes the surface's.
        // thickness rejects crossings far behind thin geometry.
        if (sceneD <= rayD - bias && rayD - sceneD < thickness) { hitT = t; break; }
        prevT = t;
    }

    if (hitT < 0.0) return false;

    // Bisect between the last miss and the hit, in the same interpolated space.
    float lo = prevT;
    float hi = hitT;
    for (int i = 0; i < refineSteps; ++i) {
        float mid    = 0.5 * (lo + hi);
        float rayD   = 1.0 / mix(iw0, iw1, mid);
        float sceneD = SampleLinearDepth(linearDepth, mix(uv0, uv1, mid));
        if (sceneD > 0.0 && sceneD <= rayD - bias) hi = mid;
        else                                       lo = mid;
    }

    hitUV = mix(uv0, uv1, hi);
    return true;
}

#endif // SCREEN_TRACE_GLSL
