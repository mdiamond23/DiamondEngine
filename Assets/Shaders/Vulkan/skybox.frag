#version 450

// Skybox background — raw HDR radiance, like everything else feeding hdrLit.
// Dropped GL's in-shader Reinhard+gamma, which double-tonemapped the sky into a
// 0.65-0.78 grey wedge. Not scaled by SkyLightComponent::intensity — that knob
// scales the sky's CONTRIBUTED light; exposure controls how bright it reads.
//
// Samples the EQUIRECT, not the env cube: a cube face's corners cover ~3x the
// solid angle per texel that its centre does, so sharpness varies across every
// face and the cube's edges show on a big smooth sky. This also gets the full
// 4k source instead of one 512 face per 90 degrees. The cube still feeds the
// irradiance/prefilter bakes and DDGI's miss shader, where none of that matters.

layout(location = 0) in  vec3 vDir;
layout(location = 0) out vec4 outColor;

layout(set = 0, binding = 1) uniform sampler2D equirectMap;

// Must match equirect_to_cubemap.frag exactly, or the background and the baked
// lighting would disagree about where the sun is.
const vec2 invAtan = vec2(0.1591, 0.3183);
vec2 SampleSphericalMap(vec3 v) {
    vec2 uv = vec2(atan(v.z, v.x), asin(v.y));
    uv *= invAtan;
    uv += 0.5;
    return uv;
}

void main() {
    vec3 dir = normalize(vDir);
    // atan wraps hard at +/-180, which spikes the implicit-LOD derivative right
    // along that meridian. The source has no mip chain, so LOD 0 is both free
    // and the only correct level; the sampler's REPEAT in u blends the wrap.
    outColor = vec4(textureLod(equirectMap, SampleSphericalMap(dir), 0.0).rgb, 1.0);
}
