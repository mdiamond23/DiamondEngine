#version 450
#extension GL_GOOGLE_include_directive : require

#include "ddgi_common.glsl"

// Probe visualization: one camera-facing quad per probe, six vertices, no vertex
// buffer. The fragment shader turns each quad into a sphere impostor and shades
// it with that probe's own irradiance — the whole point of the viz is seeing
// what the atlas actually contains, not where the probes are.

layout(set = 0, binding = 0, std140) uniform DDGIBlock { DDGIVolume v; } ddgi;
layout(set = 0, binding = 1) uniform sampler2D uProbeData;

layout(location = 0) out vec2 vLocal;    // quad coord in [-1,1], the impostor disc
layout(location = 1) flat out int vProbe;
layout(location = 2) flat out float vActive;

const vec2 kCorners[6] = vec2[](
    vec2(-1.0, -1.0), vec2( 1.0, -1.0), vec2( 1.0,  1.0),
    vec2(-1.0, -1.0), vec2( 1.0,  1.0), vec2(-1.0,  1.0)
);

void main() {
    vProbe = gl_InstanceIndex;
    vLocal = kCorners[gl_VertexIndex];

    const ivec3 coord = DDGIProbeCoord(vProbe, ddgi.v.counts.xyz);
    const vec4  pd    = DDGIProbeData(uProbeData, vProbe);
    vActive = pd.w;

    const vec3 center = DDGIProbeGridPos(ddgi.v, coord) + pd.xyz;

    // Camera basis straight out of the view matrix's rotation rows.
    const vec3 right = vec3(ddgi.v.view[0][0], ddgi.v.view[1][0], ddgi.v.view[2][0]);
    const vec3 up    = vec3(ddgi.v.view[0][1], ddgi.v.view[1][1], ddgi.v.view[2][1]);

    const float radius = ddgi.v.params3.z;
    const vec3 world = center + right * (vLocal.x * radius) + up * (vLocal.y * radius);

    gl_Position = ddgi.v.viewProj * vec4(world, 1.0);
}
