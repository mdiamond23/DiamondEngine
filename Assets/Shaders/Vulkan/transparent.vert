#version 450

// Forward transparency — Vulkan port of Engine/Shaders/Forward/transparent.vert.
// Reads only position + UV (location 1/normal is skipped, matching the GL shader);
// the pipeline declares just those two attributes over the shared mesh vertex.

layout(location = 0) in vec3 aPos;
layout(location = 2) in vec2 aTexCoords;

layout(location = 0) out vec2 vTexCoords;

layout(set = 0, binding = 0) uniform CameraUBO {
    mat4 view;
    mat4 projection;
} cam;

layout(push_constant) uniform Push {
    mat4 model;
} pc;

void main() {
    vTexCoords  = aTexCoords;
    gl_Position = cam.projection * cam.view * pc.model * vec4(aPos, 1.0);
}
