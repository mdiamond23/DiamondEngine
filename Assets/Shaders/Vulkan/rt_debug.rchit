#version 460
#extension GL_EXT_ray_tracing : require

// Closest hit: distance only. Slice 3 replaces this with real shading, which is
// where the per-instance geometry table and descriptor indexing come in.

layout(location = 0) rayPayloadInEXT float hitT;

void main() {
    hitT = gl_HitTEXT;
}
