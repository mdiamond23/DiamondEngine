#version 450

// Material-ball preview — Vulkan port of Engine/Shaders/Thumbnail/material_preview.frag.
// Lightweight PBR: two analytic lights + flat ambient (no IBL). The GL has* bool
// uniforms + scalar fallbacks become UBO flags; unassigned slots still bind 1x1
// neutral textures so every descriptor is valid.

layout(location = 0) in vec3 vWorldPos;
layout(location = 1) in vec3 vNormal;
layout(location = 2) in vec2 vUV;

layout(set = 0, binding = 0) uniform PreviewUBO {
    mat4 projection;
    mat4 view;
    mat4 model;
    vec4 cameraPos;        // xyz
    vec4 albedoFallback;   // rgb; w = uvScale
    vec4 scalars;          // x metallicFallback, y roughnessFallback, z emissiveStrength
    vec4 hasMaps;          // x albedo, y metallic, z roughness, w ao  (0/1)
    vec4 hasMaps2;         // x emissive
} u;

layout(set = 0, binding = 1) uniform sampler2D albedoMap;
layout(set = 0, binding = 2) uniform sampler2D metallicMap;
layout(set = 0, binding = 3) uniform sampler2D roughnessMap;
layout(set = 0, binding = 4) uniform sampler2D aoMap;
layout(set = 0, binding = 5) uniform sampler2D emissiveMap;

layout(location = 0) out vec4 FragColor;

const float PI = 3.14159265359;

float D_GGX(float NdotH, float a) {
    float a2 = a * a;
    float d  = (NdotH * NdotH * (a2 - 1.0) + 1.0);
    return a2 / max(PI * d * d, 1e-5);
}
float G_Smith(float NdotV, float NdotL, float a) {
    float k  = (a * a) * 0.5;
    float gv = NdotV / (NdotV * (1.0 - k) + k);
    float gl = NdotL / (NdotL * (1.0 - k) + k);
    return gv * gl;
}
vec3 F_Schlick(float ct, vec3 F0) {
    return F0 + (1.0 - F0) * pow(1.0 - ct, 5.0);
}

vec3 lightContrib(vec3 L, vec3 radiance, vec3 N, vec3 V,
                  vec3 albedo, float metallic, float rough, vec3 F0) {
    vec3  H     = normalize(V + L);
    float NdotL = max(dot(N, L), 0.0);
    float NdotV = max(dot(N, V), 0.0);
    float NdotH = max(dot(N, H), 0.0);
    float a     = max(rough * rough, 0.001);

    float D = D_GGX(NdotH, a);
    float G = G_Smith(NdotV, NdotL, a);
    vec3  F = F_Schlick(max(dot(H, V), 0.0), F0);

    vec3 spec = (D * G * F) / max(4.0 * NdotV * NdotL, 0.001);
    vec3 kd   = (vec3(1.0) - F) * (1.0 - metallic);
    return (kd * albedo / PI + spec) * radiance * NdotL;
}

void main() {
    vec2  uv        = vUV * u.albedoFallback.w;
    vec3  albedo    = u.hasMaps.x > 0.5 ? pow(texture(albedoMap, uv).rgb, vec3(2.2))
                                        : u.albedoFallback.rgb;
    float metallic  = u.hasMaps.y > 0.5 ? texture(metallicMap,  uv).r : u.scalars.x;
    float roughness = u.hasMaps.z > 0.5 ? texture(roughnessMap, uv).r : u.scalars.y;
    float ao        = u.hasMaps.w > 0.5 ? texture(aoMap,        uv).r : 1.0;

    vec3 N  = normalize(vNormal);
    vec3 V  = normalize(u.cameraPos.xyz - vWorldPos);
    vec3 F0 = mix(vec3(0.04), albedo, metallic);

    vec3 Lo = vec3(0.0);
    Lo += lightContrib(normalize(vec3( 1.0, 1.5,  1.0)), vec3(3.0),                 N, V, albedo, metallic, roughness, F0);
    Lo += lightContrib(normalize(vec3(-1.0, 0.4, -0.8)), vec3(1.0, 1.2, 1.6) * 0.6, N, V, albedo, metallic, roughness, F0);

    vec3 ambient  = vec3(0.10) * albedo * ao;
    vec3 emissive = u.hasMaps2.x > 0.5
        ? pow(texture(emissiveMap, uv).rgb, vec3(2.2)) * u.scalars.z
        : vec3(0.0);

    vec3 color = ambient + Lo + emissive;
    color = color / (color + vec3(1.0));   // reinhard tonemap
    color = pow(color, vec3(1.0 / 2.2));   // gamma
    FragColor = vec4(color, 1.0);
}
