#version 450

// Point-shadow depth pass (Vulkan port of Shadows/point_depth.frag). The cube
// stores the normalized radial distance to the light instead of projected depth,
// so the lighting pass can compare against length(frag - light) directly without
// unprojecting per-face.

layout(location = 0) in vec3 vWorldPos;
layout(location = 1) flat in vec4 vLightPosFar;   // .xyz light position, .w farPlane

void main() {
    gl_FragDepth = length(vWorldPos - vLightPosFar.xyz) / vLightPosFar.w;
}
