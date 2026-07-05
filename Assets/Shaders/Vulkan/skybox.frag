#version 450

// Skybox background — Vulkan port of Engine/Shaders/IBL/background.frag, kept
// VERBATIM including its in-shader Reinhard + gamma. That looks redundant (this
// feeds the HDR target, which the tonemap pass maps again), but GL does exactly
// the same — background.frag writes display-mapped sky into hdrBuffer and the
// tonemap re-maps it. Reproducing the quirk is what makes the Vulkan sky match
// GL's muted look; writing raw HDR here instead had ACES pushing the sky to
// near-white.

layout(location = 0) in  vec3 vDir;
layout(location = 0) out vec4 outColor;

layout(set = 0, binding = 1) uniform samplerCube environmentMap;

void main() {
    vec3 env = textureLod(environmentMap, vDir, 1.2).rgb;
    env = env / (env + vec3(1.0));          // Reinhard (matches GL background.frag)
    env = pow(env, vec3(1.0 / 2.2));        // gamma (matches GL background.frag)
    outColor = vec4(env, 1.0);
}
