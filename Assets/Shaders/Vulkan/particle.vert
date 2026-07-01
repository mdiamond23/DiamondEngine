#version 450
layout(location = 0) in vec3 aPos;
layout(location = 1) in vec2 aUV;
layout(location = 2) in vec4 aColor;
layout(push_constant) uniform Push { mat4 uViewProj; } pc;
layout(location = 0) out vec2 vUV;
layout(location = 1) out vec4 vColor;
void main() {
    vUV = aUV; vColor = aColor;
    gl_Position = pc.uViewProj * vec4(aPos, 1.0);
}