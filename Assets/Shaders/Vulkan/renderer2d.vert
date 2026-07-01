#version 450

// 2D overlay batcher — screen-space quads/text. Port of OpenGLRenderer2D's inline
// vertex shader; the projection rides in a push constant (no per-frame UBO).
layout(location = 0) in vec2  aPos;
layout(location = 1) in vec2  aUV;
layout(location = 2) in vec4  aColor;
layout(location = 3) in float aMode;

layout(push_constant) uniform Push { mat4 uProj; } pc;

layout(location = 0) out vec2  vUV;
layout(location = 1) out vec4  vColor;
layout(location = 2) out float vMode;

void main() {
    vUV     = aUV;
    vColor  = aColor;
    vMode   = aMode;
    gl_Position = pc.uProj * vec4(aPos, 0.0, 1.0);
}
