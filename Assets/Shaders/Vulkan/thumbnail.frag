#version 450

// Mesh-thumbnail preview — Vulkan port of Engine/Shaders/Thumbnail/thumbnail.frag.
// Three-light studio look (warm key, cool fill, rim) over a flat mesh color.

layout(location = 0) in vec3 vWorldPos;
layout(location = 1) in vec3 vNormal;

layout(set = 0, binding = 0) uniform ThumbUBO {
    mat4 projection;
    mat4 view;
    mat4 model;
    vec4 cameraPos;
} u;

layout(location = 0) out vec4 FragColor;

void main() {
    vec3 N = normalize(vNormal);
    vec3 V = normalize(u.cameraPos.xyz - vWorldPos);

    // Key light — warm, upper front-right
    vec3  keyDir  = normalize(vec3(1.0, 1.5, 1.0));
    float keyDiff = max(dot(N, keyDir), 0.0);
    vec3  key     = vec3(1.0, 0.93, 0.82) * keyDiff * 0.75;

    // Fill light — cool, lower back-left
    vec3  fillDir  = normalize(vec3(-1.0, 0.4, -0.8));
    float fillDiff = max(dot(N, fillDir), 0.0);
    vec3  fill     = vec3(0.35, 0.45, 0.65) * fillDiff * 0.3;

    // Rim — blue-white edge glow
    float rim    = pow(1.0 - max(dot(N, V), 0.0), 3.0);
    vec3  rimCol = vec3(0.4, 0.5, 0.7) * rim * 0.45;

    vec3 ambient   = vec3(0.12);
    vec3 meshColor = vec3(0.72, 0.72, 0.78);

    vec3 color = meshColor * (key + fill + ambient) + rimCol;
    FragColor  = vec4(color, 1.0);
}
