#version 450

// M3 first-real-geometry probe. A textured, lit mesh uploaded from MeshData:
//   * per-frame UBO at set 0, binding 0 (camera viewProj + directional light),
//   * a combined image-sampler at set 0, binding 1 (read in the fragment stage),
//   * a per-draw push constant (the model matrix), and
//   * a full vertex (position + normal + UV).

layout(location = 0) in vec3 inPos;
layout(location = 1) in vec3 inNormal;
layout(location = 2) in vec2 inUV;

layout(location = 0) out vec3 vNormalWS;
layout(location = 1) out vec2 vUV;

layout(set = 0, binding = 0) uniform FrameUBO {
    mat4 viewProj;
    vec4 lightDir;   // xyz: world-space direction the light travels
    float time;
} ubo;

layout(push_constant) uniform Push {
    mat4 model;
} pc;

void main() {
    vec4 worldPos = pc.model * vec4(inPos, 1.0);
    gl_Position = ubo.viewProj * worldPos;

    // Uniform scale assumed (cube/sphere) → the model's upper 3x3 transforms the
    // normal correctly without a separate normal matrix.
    vNormalWS = mat3(pc.model) * inNormal;
    vUV = inUV;
}
