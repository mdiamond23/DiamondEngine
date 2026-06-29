#version 450

// Post-process tonemap: ACES filmic curve + gamma, sampling the HDR scene color.
// Vulkan port of Engine/Shaders/PostProcess/tonemap.frag. Paired with
// fullscreen.vert (vertex-less triangle). set 0, binding 0 is the HDR color
// target, in SHADER_READ_ONLY after the graph's read barrier.

layout(location = 0) in  vec2 vUV;
layout(location = 0) out vec4 outColor;

layout(set = 0, binding = 0) uniform sampler2D uHDR;

vec3 ACESFilmic(vec3 x)
{
    const float a = 2.51;
    const float b = 0.03;
    const float c = 2.43;
    const float d = 0.59;
    const float e = 0.14;
    return clamp((x * (a * x + b)) / (x * (c * x + d) + e), 0.0, 1.0);
}

void main()
{
    vec3 hdr    = texture(uHDR, vUV).rgb;
    vec3 mapped = ACESFilmic(hdr);
    vec3 gamma  = pow(mapped, vec3(1.0 / 2.2));
    outColor    = vec4(gamma, 1.0);
}
