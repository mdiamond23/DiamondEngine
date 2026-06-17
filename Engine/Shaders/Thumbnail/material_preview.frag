#version 410 core
// Lightweight PBR for material-ball thumbnails. Two analytic lights + flat
// ambient (no IBL) — enough to read a material at a glance. Maps that aren't
// assigned fall back to scalar defaults via the has* flags.
in vec3 WorldPos;
in vec3 Normal;
in vec2 UV;
out vec4 FragColor;

uniform vec3  cameraPos;
uniform float uvScale;

uniform sampler2D albedoMap;     uniform bool  hasAlbedo;     uniform vec3  albedoFallback;
uniform sampler2D metallicMap;   uniform bool  hasMetallic;   uniform float metallicFallback;
uniform sampler2D roughnessMap;  uniform bool  hasRoughness;  uniform float roughnessFallback;
uniform sampler2D aoMap;         uniform bool  hasAO;
uniform sampler2D emissiveMap;   uniform bool  hasEmissive;   uniform float emissiveStrength;

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
    vec2  uv        = UV * uvScale;
    vec3  albedo    = hasAlbedo    ? pow(texture(albedoMap, uv).rgb, vec3(2.2)) : albedoFallback;
    float metallic  = hasMetallic  ? texture(metallicMap,  uv).r : metallicFallback;
    float roughness = hasRoughness ? texture(roughnessMap, uv).r : roughnessFallback;
    float ao        = hasAO        ? texture(aoMap,         uv).r : 1.0;

    vec3 N  = normalize(Normal);
    vec3 V  = normalize(cameraPos - WorldPos);
    vec3 F0 = mix(vec3(0.04), albedo, metallic);

    vec3 Lo = vec3(0.0);
    Lo += lightContrib(normalize(vec3( 1.0, 1.5,  1.0)), vec3(3.0),               N, V, albedo, metallic, roughness, F0);
    Lo += lightContrib(normalize(vec3(-1.0, 0.4, -0.8)), vec3(1.0, 1.2, 1.6) * 0.6, N, V, albedo, metallic, roughness, F0);

    vec3 ambient  = vec3(0.10) * albedo * ao;
    vec3 emissive = hasEmissive ? pow(texture(emissiveMap, uv).rgb, vec3(2.2)) * emissiveStrength : vec3(0.0);

    vec3 color = ambient + Lo + emissive;
    color = color / (color + vec3(1.0));   // reinhard tonemap
    color = pow(color, vec3(1.0 / 2.2));    // gamma
    FragColor = vec4(color, 1.0);
}
