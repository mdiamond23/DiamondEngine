#version 450

// Auto-exposure, stage 1: sample the HDR scene down to a fixed-size square of
// LOG2 luminance. Averaging in log space is what makes the result a geometric
// mean — a handful of very bright pixels can't drag the average the way they
// would in linear space, which is exactly the failure that makes naive
// auto-exposure pump when a light source enters frame.
//
// Paired with fullscreen.vert. set 0, binding 0 = the HDR scene. Output is a
// single channel (R16F target); log2 of the epsilon floor is about -13, well
// inside half-float range.

layout(location = 0) in  vec2 vUV;
layout(location = 0) out float outLogLuma;

layout(set = 0, binding = 0) uniform sampler2D uScene;

void main() {
    vec3  c = texture(uScene, vUV).rgb;
    float l = dot(max(c, vec3(0.0)), vec3(0.2126, 0.7152, 0.0722));
    outLogLuma = log2(max(l, 1e-4));
}
