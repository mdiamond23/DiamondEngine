#version 450

// Equirectangular HDR → cubemap face. Vulkan port of
// Engine/Shaders/IBL/equirect_to_cubemap.frag: the interpolated cube direction is
// mapped to spherical UVs and sampled from the loaded equirect environment, one
// face per draw. Paired with cubemap.vert.

layout(location = 0) in  vec3 vWorldPos;
layout(location = 0) out vec4 outColor;

layout(set = 0, binding = 0) uniform sampler2D equirectangularMap;

const vec2 invAtan = vec2(0.1591, 0.3183);
vec2 SampleSphericalMap(vec3 v) {
    vec2 uv = vec2(atan(v.z, v.x), asin(v.y));
    uv *= invAtan;
    uv += 0.5;
    return uv;
}

void main() {
    vec2 uv    = SampleSphericalMap(normalize(vWorldPos));
    vec3 color = texture(equirectangularMap, uv).rgb;
    outColor   = vec4(color, 1.0);
}
