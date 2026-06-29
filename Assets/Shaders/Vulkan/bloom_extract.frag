#version 450

// Bloom bright-pass: keep only fragments whose luminance is above a threshold;
// everything else goes black. The GL renderer extracts bright pixels via an MRT
// attachment in the deferred lighting pass; this forward Vulkan demo has no such
// MRT, so bloom gets a dedicated extract pass over the HDR scene color.
// Paired with fullscreen.vert. set 0, binding 0 is the HDR scene color.

layout(location = 0) in  vec2 vUV;
layout(location = 0) out vec4 outColor;

layout(set = 0, binding = 0) uniform sampler2D uScene;

const float kThreshold = 1.0;   // HDR luminance cutoff for what blooms

float luma(vec3 c) {
    return dot(c, vec3(0.2126, 0.7152, 0.0722));
}

void main() {
    vec3 c = texture(uScene, vUV).rgb;
    outColor = (luma(c) > kThreshold) ? vec4(c, 1.0) : vec4(0.0, 0.0, 0.0, 1.0);
}
