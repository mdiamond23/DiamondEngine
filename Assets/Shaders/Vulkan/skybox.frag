#version 450

// Skybox background — Vulkan port of Engine/Shaders/IBL/background.frag. Samples the
// radiance environment cubemap by world direction. Unlike the GL version it does NOT
// tonemap/gamma: this feeds the HDR scene target, and the dedicated tonemap pass at
// the end of the chain maps HDR→display (the same split the deferred/bloom passes use).

layout(location = 0) in  vec3 vDir;
layout(location = 0) out vec4 outColor;

layout(set = 0, binding = 1) uniform samplerCube environmentMap;

void main() {
    vec3 env = textureLod(environmentMap, vDir, 1.2).rgb;
    outColor = vec4(env, 1.0);
}
