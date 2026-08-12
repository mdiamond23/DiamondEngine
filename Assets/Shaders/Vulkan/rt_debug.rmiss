#version 460
#extension GL_EXT_ray_tracing : require

// Miss: a negative distance, which the raygen renders as black.

layout(location = 0) rayPayloadInEXT float hitT;

void main() {
    hitT = -1.0;
}
