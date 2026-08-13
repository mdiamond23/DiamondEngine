#version 450
#extension GL_GOOGLE_include_directive : require

#include "ddgi_common.glsl"

// Sphere impostor over the probe quad: reconstruct the hemisphere normal facing
// the camera, look that direction up in the probe's octahedral irradiance tile,
// and tonemap it for display.
//
// This draws AFTER the tonemap pass, over the LDR image, so it applies its own
// curve rather than feeding the HDR chain — a debug viz must never become a
// radiance source for SSGI or SSR.

layout(set = 0, binding = 0, std140) uniform DDGIBlock { DDGIVolume v; } ddgi;
layout(set = 0, binding = 1) uniform sampler2D uProbeData;
layout(set = 0, binding = 2) uniform sampler2D uIrradianceAtlas;

layout(location = 0) in vec2 vLocal;
layout(location = 1) flat in int vProbe;
layout(location = 2) flat in float vActive;

layout(location = 0) out vec4 outColor;

void main() {
    const float r2 = dot(vLocal, vLocal);
    if (r2 > 1.0) discard;   // outside the disc — keeps the quad from showing

    // Probes classified inside geometry render flat red: the fastest way to see
    // whether relocation is working without reading any numbers.
    if (vActive < 0.5) {
        outColor = vec4(0.35, 0.0, 0.0, 1.0);
        return;
    }

    const vec3 right = vec3(ddgi.v.view[0][0], ddgi.v.view[1][0], ddgi.v.view[2][0]);
    const vec3 up    = vec3(ddgi.v.view[0][1], ddgi.v.view[1][1], ddgi.v.view[2][1]);
    const vec3 back  = vec3(ddgi.v.view[0][2], ddgi.v.view[1][2], ddgi.v.view[2][2]);

    const vec3 normal = normalize(right * vLocal.x + up * vLocal.y
                                + back * sqrt(max(1.0 - r2, 0.0)));

    const vec2 uv = DDGIProbeUV(vProbe, normal, ddgi.v.counts.xyz,
                                DDGI_IRRADIANCE_RES,
                                vec2(textureSize(uIrradianceAtlas, 0)));

    // The atlas holds E/pi; multiply back to irradiance so the sphere reads at
    // the same brightness as the surfaces around it.
    vec3 c = texture(uIrradianceAtlas, uv).rgb * DDGI_PI * ddgi.v.params3.w;
    c = c / (1.0 + c);                       // Reinhard
    outColor = vec4(pow(c, vec3(1.0 / 2.2)), 1.0);
}
