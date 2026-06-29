#version 450

// Shared cube-capture vertex shader for the IBL bake (equirect→cubemap,
// irradiance convolution, prefilter). Port of Engine/Shaders/IBL/cubemap.vert:
// the cube's object-space position doubles as the world-space sample direction
// the fragment shader convolves. The per-face projection*view arrives through a
// push constant (the bake renders one cube face per draw); 'roughness' rides in
// the same block so the prefilter fragment shader can read it without a second
// range — the equirect/irradiance fragment shaders just ignore it.

layout(location = 0) in vec3 aPos;

layout(location = 0) out vec3 vWorldPos;

layout(push_constant) uniform Push {
    mat4  viewProj;
    float roughness;
} pc;

void main() {
    vWorldPos   = aPos;
    gl_Position = pc.viewProj * vec4(aPos, 1.0);
}
