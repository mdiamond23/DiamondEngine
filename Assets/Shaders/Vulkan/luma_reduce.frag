#version 450

// Auto-exposure, stage 2: 4x box reduction of the log-luminance image, run
// repeatedly until it reaches 1x1 (256 -> 64 -> 16 -> 4 -> 1).
//
// Four taps, not sixteen. Each output texel covers a 4x4 source block; because
// the source is exactly 4x the destination, the output texel centre lands on a
// source texel CORNER, and so does every offset of a whole texel from it. A
// bilinear fetch at a corner returns the mean of the four texels around it, so
// four taps at (+/-1, +/-1) texels cover the full 4x4 block exactly — the same
// average as sixteen point samples, at a quarter of the cost. This relies on
// the render target's Linear filter (RHITextureDesc::filter, the default).
//
// Paired with fullscreen.vert. set 0, binding 0 = the previous stage's R16F.

layout(location = 0) in  vec2 vUV;
layout(location = 0) out float outLogLuma;

layout(set = 0, binding = 0) uniform sampler2D uSource;

void main() {
    vec2 texel = 1.0 / vec2(textureSize(uSource, 0));

    float s = texture(uSource, vUV + vec2(-1.0, -1.0) * texel).r
            + texture(uSource, vUV + vec2( 1.0, -1.0) * texel).r
            + texture(uSource, vUV + vec2(-1.0,  1.0) * texel).r
            + texture(uSource, vUV + vec2( 1.0,  1.0) * texel).r;

    outLogLuma = s * 0.25;
}
