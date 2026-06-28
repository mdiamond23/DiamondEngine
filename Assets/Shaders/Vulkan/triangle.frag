#version 450

// Reads the same per-frame UBO as the vertex stage so the descriptor set is
// exercised from both stages (VERTEX | FRAGMENT stage flags). 'time' tints the
// triangle, proving the UBO contents actually reach the shader rather than the
// pipeline merely binding a set that goes unread.

layout(location = 0) in vec3 vColor;

layout(location = 0) out vec4 outColor;

layout(set = 0, binding = 0) uniform FrameUBO {
    mat4  viewProj;
    float time;
} ubo;

void main() {
    float pulse = 0.5 + 0.5 * sin(ubo.time * 2.0);
    outColor = vec4(vColor * pulse, 1.0);
}
