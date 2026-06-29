#version 450

// Cascaded-shadow-map depth pass (Vulkan port of Shadows/depth.vert). Renders scene
// geometry from the sun's point of view, once per cascade, writing only depth.
//
// The GL pass set `lightSpaceMatrix` (per cascade) and `model` (per draw) as two
// uniforms. Here the host folds them into one push-constant matrix
// (lightSpaceMatrix * model): a single cascade renders the whole scene in one
// command buffer, so a per-cascade UBO can't hold four distinct values at once —
// a push constant sidesteps that and needs no descriptors at all.

layout(location = 0) in vec3 inPos;   // position only; normal/uv ignored by the depth pass

layout(push_constant) uniform Push {
    mat4 lightModel;   // lightSpaceMatrix * model
} pc;

void main() {
    gl_Position = pc.lightModel * vec4(inPos, 1.0);
}
