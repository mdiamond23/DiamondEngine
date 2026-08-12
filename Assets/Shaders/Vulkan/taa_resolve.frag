#version 450

// Temporal anti-aliasing resolve. The camera jitters sub-pixel each frame
// (Halton 2,3 in SceneRenderer::RenderView); this pass accumulates those
// samples over time: reproject last frame's resolved image through the
// G-buffer motion vector, rectify it against the current frame's local color
// range, and blend.
//
//   binding 0  uScene     current lit HDR frame (post-SSR composite);
//                         .a = that pass's reflection weight, see below
//   binding 1  uVelocity  gVelocity — UV-space motion, historyUV = uv - v
//   binding 2  uHistory   last frame's TAA output (pass-owned, bound raw)
//
// MRT: outColor feeds the rest of the frame (transparency/particles blend into
// it in place, then tonemap); outHistory is the SAME value into a target
// nothing else touches — the post-graph history copy reads it, so translucents
// and particles (which have no motion vectors) never enter the accumulation.
//
// pc.alpha is the blend toward the current frame: ~0.1 steady-state, 1.0 when
// the history is invalid (first frame, TAA just enabled, resize) — which makes
// this pass a pure passthrough, so the graph never needs rebuilding to toggle.

layout(location = 0) in  vec2 vUV;
layout(location = 0) out vec4 outColor;
layout(location = 1) out vec4 outHistory;

layout(set = 0, binding = 0) uniform sampler2D uScene;
layout(set = 0, binding = 1) uniform sampler2D uVelocity;
layout(set = 0, binding = 2) uniform sampler2D uHistory;

layout(push_constant) uniform Push {
    float alpha;   // 1.0 = ignore history entirely
} pc;

// Clip-box half-width in standard deviations. Lower = tighter = less ghosting
// but more temporal noise; 1.0 is the usual starting point.
const float VARIANCE_GAMMA = 1.0;

// Blend toward the current frame on a pure mirror. gVelocity is the SURFACE's
// motion, but a reflection travels with the virtual image behind the surface,
// so reprojecting SSR through it is never exactly right. Parked at the default
// alpha = neutral; raise it (0.5 is a reasonable ceiling) only if reflections
// still trail once the velocity buffer itself is correct.
const float SSR_ALPHA = 0.1;

vec3 RGBToYCoCg(vec3 c) {
    return vec3( 0.25 * c.r + 0.50 * c.g + 0.25 * c.b,
                 0.50 * c.r               - 0.50 * c.b,
                -0.25 * c.r + 0.50 * c.g - 0.25 * c.b);
}

vec3 YCoCgToRGB(vec3 c) {
    return vec3(c.x + c.y - c.z, c.x + c.z, c.x - c.y - c.z);
}

// Clip the history toward the box center rather than clamping per channel: a
// clamp slides the color along a box face and shifts its hue, a clip keeps
// chromaticity and only shortens the offset.
vec3 ClipToAABB(vec3 boxMin, vec3 boxMax, vec3 q) {
    vec3  center = 0.5 * (boxMax + boxMin);
    vec3  extent = 0.5 * (boxMax - boxMin) + 1e-5;
    vec3  v      = q - center;
    vec3  unit   = abs(v / extent);
    float m      = max(unit.x, max(unit.y, unit.z));
    return m > 1.0 ? center + v / m : q;
}

vec3 Resolve(vec3 current, float ssrWeight) {
    // Disabled / no valid history: passthrough. Uniform branch — free.
    if (pc.alpha >= 1.0) return current;

    // Stale reprojection (e.g. an object whose cached prev-transform is old
    // enough to land behind the previous camera) can divide through w <= 0 and
    // write Inf/NaN velocities — never trust the raw value.
    vec2 velocity = texture(uVelocity, vUV).rg;
    if (any(isnan(velocity)) || any(isinf(velocity))) velocity = vec2(0.0);

    // Where this pixel's surface point was last frame. Off-screen = no history
    // (disocclusion at the frame edge) — fall back to the current sample.
    vec2 histUV = vUV - velocity;
    if (any(lessThan(histUV, vec2(0.0))) || any(greaterThan(histUV, vec2(1.0))))
        return current;

    // History rectification. Mean/variance of the 3x3 neighborhood in YCoCg:
    // separating luma from chroma keeps the box tight on smooth gradients,
    // which is exactly where an RGB min/max AABB grows loose enough to let
    // mis-reprojected history slide through as a ghost.
    vec2 texel = 1.0 / vec2(textureSize(uScene, 0));
    vec3 m1 = vec3(0.0);
    vec3 m2 = vec3(0.0);
    for (int y = -1; y <= 1; ++y) {
        for (int x = -1; x <= 1; ++x) {
            vec3 c = RGBToYCoCg(texture(uScene, vUV + vec2(x, y) * texel).rgb);
            m1 += c;
            m2 += c * c;
        }
    }
    vec3 mean  = m1 / 9.0;
    vec3 sigma = sqrt(max(m2 / 9.0 - mean * mean, vec3(0.0)));

    vec3 history = RGBToYCoCg(texture(uHistory, histUV).rgb);
    history = YCoCgToRGB(ClipToAABB(mean - VARIANCE_GAMMA * sigma,
                                    mean + VARIANCE_GAMMA * sigma,
                                    history));

    // Reflective pixels lean on the current frame (see SSR_ALPHA).
    float alpha = mix(pc.alpha, SSR_ALPHA, clamp(ssrWeight, 0.0, 1.0));

    // The YCoCg round-trip can land a hair below zero; RGBA16F would carry that
    // negative radiance into bloom and the tonemap.
    return max(mix(history, current, alpha), vec3(0.0));
}

void main() {
    vec4 scene    = texture(uScene, vUV);
    vec3 resolved = Resolve(scene.rgb, scene.a);
    outColor   = vec4(resolved, 1.0);
    outHistory = vec4(resolved, 1.0);
}
