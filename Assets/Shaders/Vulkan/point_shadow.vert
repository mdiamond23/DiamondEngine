#version 450

// Point-shadow depth pass (Vulkan port of Shadows/point_depth.vert). Renders the
// scene once per cube face per point light; the fragment stage overwrites depth
// with length(frag - light) / farPlane, so the vertex stage must hand down the
// WORLD-space position (a folded faceVP * model push like the CSM pass would
// lose it).
//
// GL renders all six faces in one draw through a geometry shader; here the host
// records six per-face render scopes instead, so the face's view-projection is
// looked up from a UBO holding every light's six matrices by a pushed index —
// a per-face UBO can't hold 24 distinct values in one command buffer.

layout(location = 0) in vec3 inPos;   // position only; the shared mesh vertex stride is set by the pipeline

layout(set = 0, binding = 0) uniform PointShadowUBO {
    mat4 faceVP[24];        // [light * 6 + face] — light clip from world
    vec4 lightPosFar[4];    // .xyz world-space light position, .w farPlane
} u;

layout(push_constant) uniform Push {
    mat4 model;   // per draw
    uint idx;     // per face: light * 6 + face
} pc;

layout(location = 0) out vec3 vWorldPos;
layout(location = 1) flat out vec4 vLightPosFar;

void main() {
    vec4 world   = pc.model * vec4(inPos, 1.0);
    vWorldPos    = world.xyz;
    vLightPosFar = u.lightPosFar[pc.idx / 6u];
    gl_Position  = u.faceVP[pc.idx] * world;
}
