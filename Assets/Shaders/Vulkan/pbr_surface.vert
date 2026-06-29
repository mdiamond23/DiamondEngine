#version 450

// Forward PBR surface — Vulkan port of Engine/Shaders/PBR/pbr_surface.vert. An
// alternative to the deferred path: opaque geometry shaded forward with the full
// Cook-Torrance + IBL model in world space (the deferred lighting pass works in view
// space; this one keeps the GL forward shader's world-space formulation verbatim).
// The normal matrix is derived from the model push constant (transpose-inverse) so
// non-uniform scale stays correct without a second push range.

layout(location = 0) in vec3 aPos;
layout(location = 1) in vec3 aNormal;
layout(location = 2) in vec2 aTexCoords;

layout(location = 0) out vec2 vTexCoords;
layout(location = 1) out vec3 vWorldPos;
layout(location = 2) out vec3 vNormal;

layout(set = 0, binding = 0) uniform FrameUBO {
    mat4 view;
    mat4 projection;
    vec4 camPos;            // .xyz
    vec4 albedo;            // .rgb albedo, .w metallic
    vec4 material;          // .x roughness, .y ao
    vec4 lightPositions[4]; // .xyz
    vec4 lightColors[4];    // .rgb
} ubo;

layout(push_constant) uniform Push {
    mat4 model;
} pc;

void main() {
    vTexCoords  = aTexCoords;
    vec4 world  = pc.model * vec4(aPos, 1.0);
    vWorldPos   = world.xyz;
    mat3 normalMatrix = transpose(inverse(mat3(pc.model)));
    vNormal     = normalMatrix * aNormal;
    gl_Position = ubo.projection * ubo.view * world;
}
