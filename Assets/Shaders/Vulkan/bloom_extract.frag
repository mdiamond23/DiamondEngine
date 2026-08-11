#version 450

// Bloom bright-pass: keep only fragments whose luminance is above a threshold;
// everything else goes black. The GL renderer extracts bright pixels via an MRT
// attachment in the deferred lighting pass; this forward Vulkan demo has no such
// MRT, so bloom gets a dedicated extract pass over the HDR scene color.
// Paired with fullscreen.vert. set 0, binding 0 is the HDR scene color.

layout(location = 0) in  vec2 vUV;
layout(location = 0) out vec4 outColor;

layout(set = 0, binding = 0) uniform sampler2D uScene;
// 1x1 auto-exposure multiplier — the same value the tonemap applies.
layout(set = 0, binding = 1) uniform sampler2D uExposure;

layout(push_constant) uniform Push {
    // Cutoff in POST-EXPOSURE luminance: the test runs on the scene as the
    // tonemap will see it, so 1.0 means "blooms once it reaches display white"
    // no matter how the scene is lit or how the exposure has adapted. Keeping
    // this in pre-exposure linear HDR would make it a per-scene magic number.
    float threshold;
    float manualExposure;   // mirrors the tonemap's manual term
} pc;

float luma(vec3 c) {
    return dot(c, vec3(0.2126, 0.7152, 0.0722));
}

void main() {
    vec3  c        = texture(uScene, vUV).rgb;
    float exposure = pc.manualExposure * texture(uExposure, vec2(0.5)).r;

    // Threshold against the exposed value, but emit the RAW scene colour: the
    // composite adds this back into the HDR scene, which the tonemap exposes
    // later. Emitting the exposed colour would apply exposure twice to bloom.
    outColor = (luma(c * exposure) > pc.threshold) ? vec4(c, 1.0)
                                                   : vec4(0.0, 0.0, 0.0, 1.0);
}
