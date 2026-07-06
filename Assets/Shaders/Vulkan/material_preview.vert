#version 450

// Material-ball preview — Vulkan port of Engine/Shaders/Thumbnail/material_preview.vert.
// Same block as material_preview.frag; model is identity so mat3(model) stands in
// for the GL normalMatrix uniform.

layout(location = 0) in vec3 aPos;
layout(location = 1) in vec3 aNormal;
layout(location = 2) in vec2 aUV;

layout(set = 0, binding = 0) uniform PreviewUBO {
    mat4 projection;
    mat4 view;
    mat4 model;
    vec4 cameraPos;        // xyz
    vec4 albedoFallback;   // rgb; w = uvScale
    vec4 scalars;          // x metallicFallback, y roughnessFallback, z emissiveStrength
    vec4 hasMaps;          // x albedo, y metallic, z roughness, w ao  (0/1)
    vec4 hasMaps2;         // x emissive
} u;

layout(location = 0) out vec3 vWorldPos;
layout(location = 1) out vec3 vNormal;
layout(location = 2) out vec2 vUV;

void main() {
    vWorldPos   = vec3(u.model * vec4(aPos, 1.0));
    vNormal     = normalize(mat3(u.model) * aNormal);
    vUV         = aUV;
    gl_Position = u.projection * u.view * vec4(vWorldPos, 1.0);
}
