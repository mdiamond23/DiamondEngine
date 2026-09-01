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

// Smallest depth difference that MEANS anything at this depth.
//
// The pyramid is R16F (VulkanDepthPyramidPass). A half float carries a 10-bit
// mantissa, so consecutive representable depths near d sit about d * 2^-11
// apart — the stored depth is not a smooth function of distance, it is a
// STAIRCASE, and on a floor receding from the camera the treads of that
// staircase are wide horizontal strips of exactly-equal depth.
//
// A fixed tolerance is therefore wrong at both ends of the range. At d = 100 it
// takes about 0.05 just to reach one quantisation step, so a fixed 0.05 is
// comparing two numbers inside their own representation error: 'rayD' varies
// smoothly along the ray while 'sceneD' snaps between treads, and the
// comparison flips sign at every tread boundary. Each flip is a false
// self-intersection right at the ray's origin, which returns the surface's own
// colour instead of what it should reflect — one bright horizontal line per
// tread, slicing across the reflection. That is a screen-space artifact with no
// RT component at all, which is why turning RT reflections off leaves it
// untouched.
//
// 4x the quantum clears the staircase with margin for the interpolated ray
// depth's own error. Below about d = 25 this stays under the caller's fixed
// bias and nothing changes, so near-field behaviour is exactly as tuned.
float DepthEpsilon(float d, float bias) {
    const float kR16FQuantum = 4.0 / 2048.0;   // 4 * 2^-11
    return max(bias, d * kR16FQuantum);
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
// 'thickness' is NOT a depth tolerance on the hit — the bracket test in the
// loop bounds overshoot by itself. It is slack on the bracket's ENTRY half; see
// there.
// What the march DID, for ssr_debug.frag. Filled on every path, hit or miss.
// There is one implementation of the walk below and the plain TraceScreenRay is
// a wrapper over it — a debug trace that ran different code from the shipping
// one would be worse than no debug trace at all.
struct ScreenTraceDebug {
    int   step;        // 1-based step that hit; 0 = no hit
    int   stepCount;   // projected-length-adaptive budget used by this ray
    float overshoot;   // rayD - sceneD where the crossing was accepted
    float eps;         // tolerance actually in force there
    float slope;       // the measured per-step surface depth span at the hit
    float tExit;       // fraction of the segment that was on screen
};

bool TraceScreenRayDbg(sampler2D linearDepth, sampler2D coarseDepth,
                       mat4 proj, vec3 origin, vec3 dir,
                       float maxDistance, int minSteps, int maxSteps,
                       float pixelsPerStep, int refineSteps,
                       float thickness, float bias, out vec2 hitUV,
                       out ScreenTraceDebug dbg)
{
    hitUV = vec2(0.0);
    dbg = ScreenTraceDebug(0, 0, 0.0, 0.0, 0.0, 0.0);

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

    // Shorten the segment to the part that is ON SCREEN before the step budget
    // is spread over it. The march can only ever hit inside the frame, so a
    // segment that leaves after a third of its length was spending two thirds
    // of its steps on taps that are guaranteed misses — and, worse, sizing
    // EVERY step to the full length. A floor reflecting upward is the usual
    // case: its ray exits the top of the frame almost immediately, so the old
    // fixed march landed taps tens of pixels apart over the span it actually
    // cared about. Clipping first spends the adaptive budget only in frame.
    // Exact, not an approximation: uv is affine in t, so this is a
    // Liang-Barsky exit parameter, and 1/w is affine in the SAME t, so
    // rescaling t leaves the depth interpolation below untouched.
    float tExit = 1.0;
    if (all(greaterThanEqual(uv0, vec2(0.0))) && all(lessThanEqual(uv0, vec2(1.0)))) {
        // uv0 is this pixel, hence inside the box: on each axis one root is
        // behind the origin and the other ahead, so max() picks the exit.
        vec2 d = uv1 - uv0;
        if (abs(d.x) > 1e-8) tExit = min(tExit, max(-uv0.x / d.x, (1.0 - uv0.x) / d.x));
        if (abs(d.y) > 1e-8) tExit = min(tExit, max(-uv0.y / d.y, (1.0 - uv0.y) / d.y));
        tExit = clamp(tExit, 0.0, 1.0);
    }
    dbg.tExit = tExit;
    if (tExit <= 1e-4) return false;   // origin on the border, ray leaves at once

    // Spend samples according to projected work, not view-space distance. A
    // short reflection should not pay the same 64/128 taps as a ray spanning
    // half the viewport, while a long 4K ray cannot be represented faithfully
    // by one tap every few dozen pixels. Callers opt out by passing identical
    // min/max counts (SSGI keeps its existing fixed budget).
    const vec2 depthSize = vec2(textureSize(linearDepth, 0));
    const float spanPixels = length((uv1 - uv0) * tExit * depthSize);
    int steps = maxSteps;
    if (minSteps < maxSteps && pixelsPerStep > 0.0) {
        steps = clamp(int(ceil(spanPixels / pixelsPerStep)), minSteps, maxSteps);
    }
    dbg.stepCount = steps;

    float prevT     = 0.0;
    float prevRayD  = 1.0 / iw0;       // depth at the origin
    float prevSceneD = -1.0;           // surface depth at the last FINE tap, <0 = none
    float hitT      = -1.0;

    for (int i = 1; i <= steps; ++i) {
        float t  = tExit * float(i) / float(steps);
        vec2  uv = clamp(mix(uv0, uv1, t), vec2(0.0), vec2(1.0));

        float rayD = 1.0 / mix(iw0, iw1, t);

        // Conservative reject. 0 means the whole footprint is sky (the
        // downsample only writes 0 when every child was sky), and a nearest
        // surface still in front of the ray means no crossing anywhere in the
        // footprint. Either way the fine tap is provably unnecessary.
        float eps = DepthEpsilon(rayD, bias);

        float coarseD = SampleLinearDepth(coarseDepth, uv);
        if (coarseD <= 0.0 || coarseD > rayD - eps) {
            prevT = t; prevRayD = rayD; prevSceneD = -1.0; continue;
        }

        float sceneD = SampleLinearDepth(linearDepth, uv);
        if (sceneD <= 0.0) {                                  // over a sky pixel
            prevT = t; prevRayD = rayD; prevSceneD = -1.0; continue;
        }

        // NO slope term here. A previous version widened eps by the measured
        // per-step change in surface depth, to chase a grazing-angle
        // self-intersection theory. The Hit Distance view killed that theory —
        // there were no near-zero-distance hits anywhere — and the term did
        // real harm: consecutive taps straddling a SILHOUETTE differ hugely, so
        // eps pinned to its cap at exactly the edges where a reflected object
        // begins, and the object's reflection went missing.

        // A crossing is a BRACKET, not a proximity test: the ray must have been
        // in FRONT of this surface where the step began and BEHIND it where the
        // step ended. Both halves carry their weight.
        //
        //  - Testing only "ended up behind, within some tolerance of the
        //    surface" is what a fixed 'thickness' did, and it fails in BOTH
        //    directions once steps get long. Too small and a step that jumps
        //    the ray clean past a surface reports no hit — and since the ray is
        //    behind the geometry from then on, every later step fails
        //    identically and the WHOLE ray misses, which put hard edges through
        //    reflections of thin and distant geometry. Too large (a tolerance
        //    scaled to the step) and a ray legitimately travelling behind an
        //    object gets claimed by it, which smears reflections into bands.
        //
        //  - The bracket has neither failure because it is self-bounding. If
        //    the ray was in front at the start and behind at the end, then
        //    rayD - sceneD is at most the depth the step covered, BY
        //    CONSTRUCTION. A long step therefore admits a proportionally larger
        //    overshoot automatically — which is exactly the slack the coarse
        //    grid needs — while a ray already behind at the step's start is
        //    rejected outright no matter how long the step was.
        //
        // 'thickness' survives as slack on the ENTRY half only. sceneD is
        // sampled at this step's uv, not the previous one, so on a surface
        // slanted away from the ray the previous step can read slightly behind
        // a surface it was really still in front of; thickness absorbs that
        // without ever licensing an unbounded overshoot.
        // Two ways to accept, and the ray only has to satisfy ONE.
        //
        //   shallow  — the ray is behind the surface by less than 'thickness'.
        //              The original test, and it carries the common case.
        //   bracketed— the ray was in front where the step began. This catches
        //              a coarse step that jumps the ray CLEAN PAST a thin or
        //              distant surface, where the overshoot exceeds thickness
        //              and 'shallow' alone would reject it — the chop this
        //              whole line of work started from.
        //
        // Making the bracket a REPLACEMENT for shallow, rather than an
        // alternative to it, is what put holes in the reflections it was
        // supposed to repair. sceneD at this step is frequently a DIFFERENT
        // surface than at the last one — floor, then torus — so 'was the ray in
        // front of it last step' compares the ray's old depth against a surface
        // it had not reached yet, and rejects a crossing that genuinely
        // happened. Either condition alone is evidence of a real hit; requiring
        // both is evidence of neither.
        bool shallow   = rayD - sceneD < thickness;
        bool bracketed = prevRayD <= sceneD + thickness;
        if (rayD > sceneD + eps && (shallow || bracketed)) {
            hitT = t;
            dbg.step      = i;
            dbg.overshoot = rayD - sceneD;
            dbg.eps       = eps;
            // Still reported: it is the per-step surface depth change, which is
            // worth SEEING even though it no longer feeds the tolerance.
            dbg.slope     = (prevSceneD > 0.0) ? abs(sceneD - prevSceneD) : 0.0;
            break;
        }
        prevT      = t;
        prevRayD   = rayD;
        prevSceneD = sceneD;
    }

    if (hitT < 0.0) return false;

    // Bisect between the last miss and the hit, in the same interpolated space.
    float lo = prevT;
    float hi = hitT;
    for (int i = 0; i < refineSteps; ++i) {
        float mid    = 0.5 * (lo + hi);
        float rayD   = 1.0 / mix(iw0, iw1, mid);
        float sceneD = SampleLinearDepth(linearDepth, mix(uv0, uv1, mid));
        // Same depth-relative epsilon as the march — a bisection run against a
        // tighter tolerance than the loop that found the bracket would converge
        // to a boundary the loop never agreed existed.
        if (sceneD > 0.0 && sceneD <= rayD - DepthEpsilon(rayD, bias)) hi = mid;
        else                                                           lo = mid;
    }

    hitUV = mix(uv0, uv1, hi);
    return true;
}

// The shipping entry point. Identical walk — it just drops the diagnostics.
bool TraceScreenRay(sampler2D linearDepth, sampler2D coarseDepth,
                    mat4 proj, vec3 origin, vec3 dir,
                    float maxDistance, int steps, int refineSteps,
                    float thickness, float bias, out vec2 hitUV)
{
    ScreenTraceDebug dbg;
    return TraceScreenRayDbg(linearDepth, coarseDepth, proj, origin, dir,
                             maxDistance, steps, steps, 0.0, refineSteps,
                             thickness, bias,
                             hitUV, dbg);
}

// SSR variant: target a screen-space stride while retaining hard cost bounds.
bool TraceScreenRayAdaptive(sampler2D linearDepth, sampler2D coarseDepth,
                            mat4 proj, vec3 origin, vec3 dir,
                            float maxDistance, int minSteps, int maxSteps,
                            float pixelsPerStep, int refineSteps,
                            float thickness, float bias, out vec2 hitUV)
{
    ScreenTraceDebug dbg;
    return TraceScreenRayDbg(linearDepth, coarseDepth, proj, origin, dir,
                             maxDistance, minSteps, maxSteps, pixelsPerStep,
                             refineSteps, thickness, bias, hitUV, dbg);
}

// Confidence in the hit surface itself, independent of reflected colour. A
// hit next to a depth discontinuity is where a one-pixel change in the march
// swaps SSR for RT. Sampling only depth continuity produces a soft ownership
// handoff without filtering the reflected image.
float ScreenHitConfidence(sampler2D linearDepth, vec2 hitUV) {
    const ivec2 size = textureSize(linearDepth, 0);
    const ivec2 c = clamp(ivec2(hitUV * vec2(size)), ivec2(0), size - 1);
    const float center = texelFetch(linearDepth, c, 0).r;
    if (center <= 0.0) return 0.0;

    const ivec2 offsets[8] = ivec2[8](
        ivec2(-1,  0), ivec2( 1,  0), ivec2( 0, -1), ivec2( 0,  1),
        ivec2(-2,  0), ivec2( 2,  0), ivec2( 0, -2), ivec2( 0,  2));

    float continuity = 0.0;
    const float tolerance = max(DepthEpsilon(center, 0.05) * 4.0,
                                center * 0.01);
    for (int i = 0; i < 8; ++i) {
        const float d = texelFetch(linearDepth,
                                   clamp(c + offsets[i], ivec2(0), size - 1), 0).r;
        if (d > 0.0)
            continuity += 1.0 - smoothstep(tolerance, tolerance * 4.0,
                                            abs(d - center));
    }

    // Eight discrete agreements become a useful 0..1 feather: thin geometry
    // and silhouettes hand ownership to RT; coherent interiors remain SSR.
    return smoothstep(0.25, 1.0, continuity * 0.125);
}

#endif // SCREEN_TRACE_GLSL
