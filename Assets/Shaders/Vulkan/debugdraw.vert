#version 450

// Immediate-mode debug-line renderer — the Vulkan counterpart of DebugDraw's GL
// Flush(). Positions are world-space; the view-proj rides in a push constant
// (no per-frame UBO, mirroring renderer2d.vert's projection-only push).
layout(location = 0) in vec3 aPos;
layout(location = 1) in vec3 aColor;

layout(push_constant) uniform Push { mat4 uViewProj; } pc;

layout(location = 0) out vec3 vColor;

void main() {
    vColor      = aColor;
    gl_Position = pc.uViewProj * vec4(aPos, 1.0);
}
