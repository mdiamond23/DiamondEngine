#version 460
#extension GL_EXT_ray_tracing        : require
#extension GL_GOOGLE_include_directive : require

#include "ddgi_common.glsl"

// Probe ray miss: the sky. Samples the same baked radiance cubemap the skybox
// draws, scaled by SkyLightComponent::intensity — so the sun-to-sky ratio the
// user tunes for direct lighting drives probe irradiance identically.
//
// A miss reports the full ray length as its hit distance, which is what makes
// the visibility atlas treat open sky as "nothing occludes this direction".

layout(set = 0, binding = 2, std140) uniform DDGIBlock { DDGIVolume v; } ddgi;
layout(set = 0, binding = 12) uniform samplerCube uEnvMap;

layout(location = 0) rayPayloadInEXT DDGIPayload payload;

void main() {
    payload.radiance = textureLod(uEnvMap, gl_WorldRayDirectionEXT, 0.0).rgb
                     * ddgi.v.params2.x;
    payload.hitT     = ddgi.v.params.w;
}
