#version 450

// Temporal anti-aliasing resolve. The camera jitters sub-pixel each frame
// (Halton 2,3 in SceneRenderer::RenderView); this pass accumulates those
// samples over time: reproject last frame's resolved image through the
// G-buffer motion vector, rectify it against the current frame's local color
// range, and blend. The output doubles as next frame's history (copied into
// the pass-owned single-buffered history texture after the graph runs —
// see VulkanTAAPass::RecordHistoryCopy).
//
//   binding 0  uScene     current lit HDR frame (post-SSR composite)
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

vec3 Resolve(vec3 current) {
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

    // History rectification — the step that kills ghosting. Build the 3x3
    // neighborhood's color AABB in the CURRENT frame and clamp the reprojected
    // history into it: history that no longer resembles anything near this
    // pixel (disocclusion, lighting change, bad velocity) gets pulled to the
    // closest plausible color instead of smearing in. RGB min/max clamp is the
    // v1 baseline; YCoCg variance clipping is the known upgrade.
    vec2 texel = 1.0 / vec2(textureSize(uScene, 0));
    vec3 nmin = current;
    vec3 nmax = current;
    for (int y = -1; y <= 1; ++y) {
        for (int x = -1; x <= 1; ++x) {
            if (x == 0 && y == 0) continue;
            vec3 c = texture(uScene, vUV + vec2(x, y) * texel).rgb;
            nmin = min(nmin, c);
            nmax = max(nmax, c);
        }
    }
    vec3 history = clamp(texture(uHistory, histUV).rgb, nmin, nmax);

    return mix(history, current, pc.alpha);
}

void main() {
    vec3 resolved = Resolve(texture(uScene, vUV).rgb);
    outColor   = vec4(resolved, 1.0);
    outHistory = vec4(resolved, 1.0);
}
