#version 450

// M1 binding-model probe. Three things are validated here together:
//   * per-frame UBO at set 0, binding 0 (shared with the fragment stage),
//   * a per-draw push constant (the model matrix), and
//   * a real vertex buffer (position + colour attributes).

layout(location = 0) in vec2 inPos;
layout(location = 1) in vec3 inColor;

layout(location = 0) out vec3 vColor;

layout(set = 0, binding = 0) uniform FrameUBO {
    mat4  viewProj;
    float time;
} ubo;

layout(push_constant) uniform Push {
    mat4 model;
} pc;

void main() {
    gl_Position = ubo.viewProj * pc.model * vec4(inPos, 0.0, 1.0);
    vColor = inColor;
}
