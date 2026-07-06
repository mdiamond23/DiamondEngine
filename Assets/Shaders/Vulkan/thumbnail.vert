#version 450

// Mesh-thumbnail preview — Vulkan port of Engine/Shaders/Thumbnail/thumbnail.vert.
// The GL shader took a separate normalMatrix uniform; thumbnails always draw with
// an identity model matrix, so mat3(model) is exact and the UBO stays lean.

layout(location = 0) in vec3 aPos;
layout(location = 1) in vec3 aNormal;

layout(set = 0, binding = 0) uniform ThumbUBO {
    mat4 projection;
    mat4 view;
    mat4 model;
    vec4 cameraPos;   // xyz used
} u;

layout(location = 0) out vec3 vWorldPos;
layout(location = 1) out vec3 vNormal;

void main() {
    vWorldPos   = vec3(u.model * vec4(aPos, 1.0));
    vNormal     = normalize(mat3(u.model) * aNormal);
    gl_Position = u.projection * u.view * vec4(vWorldPos, 1.0);
}
