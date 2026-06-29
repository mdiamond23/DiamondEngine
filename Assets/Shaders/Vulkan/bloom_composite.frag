#version 450

// Bloom composite: add the blurred bright-pass result back onto the HDR scene.
// Kept additive and HDR-only — tonemapping stays in the dedicated tonemap pass
// that runs after this (the GL renderer folds both into bloomfinal.frag; splitting
// them keeps each post stage single-purpose). Paired with fullscreen.vert.
// set 0, binding 0 = HDR scene color; binding 1 = blurred bloom.

layout(location = 0) in  vec2 vUV;
layout(location = 0) out vec4 outColor;

layout(set = 0, binding = 0) uniform sampler2D uScene;
layout(set = 0, binding = 1) uniform sampler2D uBloom;

const float kBloomIntensity = 1.0;

void main() {
    vec3 hdr   = texture(uScene, vUV).rgb;
    vec3 bloom = texture(uBloom, vUV).rgb;
    outColor   = vec4(hdr + bloom * kBloomIntensity, 1.0);
}
