#version 450

// Forward PBR surface — Vulkan port of Engine/Shaders/PBR/pbr_surface.frag. The
// Cook-Torrance BRDF + split-sum IBL ambient are ported verbatim (world space). Two
// deliberate differences from the GL shader:
//   * material params + lights arrive in one UBO instead of loose uniforms;
//   * no internal tonemap/gamma — this writes HDR for the dedicated tonemap pass.

layout(location = 0) in vec2 vTexCoords;
layout(location = 1) in vec3 vWorldPos;
layout(location = 2) in vec3 vNormal;

layout(location = 0) out vec4 outColor;

layout(set = 0, binding = 0) uniform FrameUBO {
    mat4 view;
    mat4 projection;
    vec4 camPos;            // .xyz
    vec4 albedo;            // .rgb albedo, .w metallic
    vec4 material;          // .x roughness, .y ao
    vec4 lightPositions[4]; // .xyz
    vec4 lightColors[4];    // .rgb
} ubo;

layout(set = 0, binding = 1) uniform samplerCube irradianceMap;
layout(set = 0, binding = 2) uniform samplerCube prefilterMap;
layout(set = 0, binding = 3) uniform sampler2D   brdfLUT;

const float PI = 3.14159265359;

float DistributionGGX(vec3 N, vec3 H, float roughness) {
    float a = roughness * roughness;
    float a2 = a * a;
    float NdotH = max(dot(N, H), 0.0);
    float NdotH2 = NdotH * NdotH;
    float denom = (NdotH2 * (a2 - 1.0) + 1.0);
    denom = PI * denom * denom;
    return a2 / denom;
}

float GeometrySchlickGGX(float NdotV, float roughness) {
    float r = (roughness + 1.0);
    float k = (r * r) / 8.0;
    return NdotV / (NdotV * (1.0 - k) + k);
}

float GeometrySmith(vec3 N, vec3 V, vec3 L, float roughness) {
    float ggx2 = GeometrySchlickGGX(max(dot(N, V), 0.0), roughness);
    float ggx1 = GeometrySchlickGGX(max(dot(N, L), 0.0), roughness);
    return ggx1 * ggx2;
}

vec3 fresnelSchlick(float cosTheta, vec3 F0) {
    return F0 + (1.0 - F0) * pow(clamp(1.0 - cosTheta, 0.0, 1.0), 5.0);
}

vec3 fresnelSchlickRoughness(float cosTheta, vec3 F0, float roughness) {
    return F0 + (max(vec3(1.0 - roughness), F0) - F0) * pow(clamp(1.0 - cosTheta, 0.0, 1.0), 5.0);
}

void main() {
    vec3  albedo    = ubo.albedo.rgb;
    float metallic  = ubo.albedo.w;
    float roughness = ubo.material.x;
    float ao        = ubo.material.y;

    vec3 N = normalize(vNormal);
    vec3 V = normalize(ubo.camPos.xyz - vWorldPos);
    vec3 R = reflect(-V, N);

    vec3 F0 = vec3(0.04);
    F0 = mix(F0, albedo, metallic);

    vec3 Lo = vec3(0.0);
    for (int i = 0; i < 4; ++i) {
        vec3  L = normalize(ubo.lightPositions[i].xyz - vWorldPos);
        vec3  H = normalize(V + L);
        float distance    = length(ubo.lightPositions[i].xyz - vWorldPos);
        float attenuation = 1.0 / (distance * distance);
        vec3  radiance    = ubo.lightColors[i].rgb * attenuation;

        float NDF = DistributionGGX(N, H, roughness);
        float G   = GeometrySmith(N, V, L, roughness);
        vec3  F   = fresnelSchlick(max(dot(H, V), 0.0), F0);

        vec3  numerator   = NDF * G * F;
        float denominator = 4.0 * max(dot(N, V), 0.0) * max(dot(N, L), 0.0) + 0.0001;
        vec3  specular    = numerator / denominator;

        vec3 kS = F;
        vec3 kD = vec3(1.0) - kS;
        kD *= 1.0 - metallic;

        float NdotL = max(dot(N, L), 0.0);
        Lo += (kD * albedo / PI + specular) * radiance * NdotL;
    }

    // IBL ambient (split-sum).
    vec3 F  = fresnelSchlickRoughness(max(dot(N, V), 0.0), F0, roughness);
    vec3 kS = F;
    vec3 kD = 1.0 - kS;
    kD *= 1.0 - metallic;

    vec3 irradiance = texture(irradianceMap, N).rgb;
    vec3 diffuse    = irradiance * albedo;

    const float MAX_REFLECTION_LOD = 4.0;
    vec3 prefilteredColor = textureLod(prefilterMap, R, roughness * MAX_REFLECTION_LOD).rgb;
    vec2 brdf     = texture(brdfLUT, vec2(max(dot(N, V), 0.0), roughness)).rg;
    vec3 specular = prefilteredColor * (F * brdf.x + brdf.y);

    vec3 ambient = (kD * diffuse + specular) * ao;

    outColor = vec4(ambient + Lo, 1.0);
}
