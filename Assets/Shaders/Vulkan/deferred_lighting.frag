#version 450

// Deferred lighting — Vulkan port of Engine/Shaders/Deferred/lighting.frag. A
// fullscreen pass that consumes the G-buffer + SSAO + cascaded shadow maps and
// resolves Cook-Torrance PBR into one HDR color target. Paired with
// fullscreen.vert; the bloom pass extracts its own bright pixels downstream, so
// this stage writes a single HDR attachment (no BrightColor MRT).
//
// Everything is evaluated in VIEW space (the G-buffer stores view-space position
// + normal): the camera sits at the origin so V = normalize(-viewPos), and CSM
// reuses the lightFromView = lightMatrix * inverse(view) trick the debug shader
// proved out. World-space reconstruction (and the camPos uniform the GL shader
// needs) is therefore unnecessary.
//
// Slice scope: directional sun with CSM shadows + point lights (unshadowed —
// point cube-shadows aren't ported yet) + image-based lighting (diffuse irradiance
// + specular prefilter/BRDF) replacing the old constant-ambient placeholder.
//
// The IBL maps are world-space cubemaps, but everything else here is view space, so
// invView rotates the view-space normal/reflection into world space to sample them.

layout(location = 0) in  vec2 vUV;
layout(location = 0) out vec4 outColor;

layout(set = 0, binding = 0) uniform sampler2D gViewPos;
layout(set = 0, binding = 1) uniform sampler2D gViewNormal;
layout(set = 0, binding = 2) uniform sampler2D gAlbedo;
layout(set = 0, binding = 3) uniform sampler2D gMaterial;   // r=metallic g=roughness b=ao
layout(set = 0, binding = 4) uniform sampler2D ssaoTex;
layout(set = 0, binding = 5) uniform sampler2D gEmissive;
layout(set = 0, binding = 6) uniform sampler2D shadowCascade0;
layout(set = 0, binding = 7) uniform sampler2D shadowCascade1;
layout(set = 0, binding = 8) uniform sampler2D shadowCascade2;
layout(set = 0, binding = 9) uniform sampler2D shadowCascade3;

layout(set = 0, binding = 10) uniform LightingUBO {
    mat4 lightFromView[4];   // per-cascade: view space → light clip
    mat4 invView;            // view space → world space (for IBL cubemap sampling)
    vec4 cascadeSplits;      // x..w = cascade far view-depths (positive)
    vec4 sunDirView;         // .xyz sun direction in view space (normalized)
    vec4 sunColor;           // .xyz
    vec4 pointPos[4];        // .xyz view-space position
    vec4 pointColor[4];      // .xyz radiant intensity
    vec4 counts;             // x = numPointLights, y = prefilter max LOD
} u;

// IBL maps (world space). Bound after the resource set is created (bake-time views).
layout(set = 0, binding = 11) uniform samplerCube irradianceMap;
layout(set = 0, binding = 12) uniform samplerCube prefilterMap;
layout(set = 0, binding = 13) uniform sampler2D   brdfLUT;

const float PI = 3.14159265359;

// ── Cook-Torrance terms (ported verbatim from the GL lighting shader) ─────────
float DistributionGGX(vec3 N, vec3 H, float roughness) {
    float a  = roughness * roughness;
    float a2 = a * a;
    float d  = (dot(N, H) * dot(N, H)) * (a2 - 1.0) + 1.0;
    return a2 / (PI * d * d);
}
float GeometrySchlickGGX(float NdotV, float roughness) {
    float k = (roughness + 1.0) * (roughness + 1.0) / 8.0;
    return NdotV / (NdotV * (1.0 - k) + k);
}
float GeometrySmith(vec3 N, vec3 V, vec3 L, float roughness) {
    return GeometrySchlickGGX(max(dot(N, V), 0.0), roughness)
         * GeometrySchlickGGX(max(dot(N, L), 0.0), roughness);
}
vec3 fresnelSchlick(float cosTheta, vec3 F0) {
    return F0 + (1.0 - F0) * pow(clamp(1.0 - cosTheta, 0.0, 1.0), 5.0);
}
// Roughness-aware Fresnel for the IBL ambient term (ported from the GL shader).
vec3 fresnelSchlickRoughness(float cosTheta, vec3 F0, float roughness) {
    return F0 + (max(vec3(1.0 - roughness), F0) - F0)
              * pow(clamp(1.0 - cosTheta, 0.0, 1.0), 5.0);
}

// Single Cook-Torrance light contribution given an incoming radiance.
vec3 BRDF(vec3 N, vec3 V, vec3 L, vec3 albedo, vec3 F0,
          float metallic, float roughness, vec3 radiance) {
    vec3  H   = normalize(V + L);
    float NDF = DistributionGGX(N, H, roughness);
    float G   = GeometrySmith(N, V, L, roughness);
    vec3  F   = fresnelSchlick(max(dot(H, V), 0.0), F0);

    vec3  spec  = NDF * G * F
                / (4.0 * max(dot(N, V), 0.0) * max(dot(N, L), 0.0) + 0.0001);
    vec3  kD    = (vec3(1.0) - F) * (1.0 - metallic);
    float NdotL = max(dot(N, L), 0.0);
    return (kD * albedo / PI + spec) * radiance * NdotL;
}

// ── Cascaded shadow (3×3 PCF on the selected cascade) ─────────────────────────
float PCF(sampler2D smap, vec2 suv, float current, float bias) {
    if (suv.x < 0.0 || suv.x > 1.0 || suv.y < 0.0 || suv.y > 1.0) return 0.0;
    if (current > 1.0) return 0.0;
    vec2 texel = 1.0 / vec2(textureSize(smap, 0));
    float shadow = 0.0;
    for (int x = -1; x <= 1; ++x)
        for (int y = -1; y <= 1; ++y)
            shadow += (current - bias >
                       texture(smap, suv + vec2(x, y) * texel).r) ? 1.0 : 0.0;
    return shadow / 9.0;
}
float SunShadow(vec3 viewPos, vec3 N, vec3 L) {
    float depth = -viewPos.z;   // view space looks down -z → positive distance
    int c = 3;
    if      (depth < u.cascadeSplits.x) c = 0;
    else if (depth < u.cascadeSplits.y) c = 1;
    else if (depth < u.cascadeSplits.z) c = 2;

    vec4  lc      = u.lightFromView[c] * vec4(viewPos, 1.0);
    vec3  p       = lc.xyz / lc.w;          // ortho → w == 1
    vec2  suv     = p.xy * 0.5 + 0.5;       // light NDC → [0,1]
    float current = p.z;                    // [0,1] (GLM_FORCE_DEPTH_ZERO_TO_ONE)
    float bias    = max(0.0025 * (1.0 - dot(N, L)), 0.0005);

    if      (c == 0) return PCF(shadowCascade0, suv, current, bias);
    else if (c == 1) return PCF(shadowCascade1, suv, current, bias);
    else if (c == 2) return PCF(shadowCascade2, suv, current, bias);
    else             return PCF(shadowCascade3, suv, current, bias);
}

void main() {
    vec3 viewPos = texture(gViewPos, vUV).xyz;
    // Background pixel — no geometry written (gViewPos cleared to 0).
    if (viewPos.z == 0.0) { outColor = vec4(0.0, 0.0, 0.0, 1.0); return; }

    vec3  N         = normalize(texture(gViewNormal, vUV).xyz);
    vec3  albedo    = texture(gAlbedo,   vUV).rgb;
    vec3  matData   = texture(gMaterial, vUV).rgb;
    float metallic  = matData.r;
    float roughness = matData.g;
    float ao        = matData.b;
    float occlusion = texture(ssaoTex, vUV).r;

    vec3 V  = normalize(-viewPos);                    // camera at origin in view space
    vec3 F0 = mix(vec3(0.04), albedo, metallic);

    vec3 Lo = vec3(0.0);

    // Directional sun + cascaded shadow.
    {
        vec3  L      = normalize(-u.sunDirView.xyz);
        float shadow = SunShadow(viewPos, N, L);
        Lo += BRDF(N, V, L, albedo, F0, metallic, roughness, u.sunColor.xyz)
              * (1.0 - shadow);
    }

    // Point lights (inverse-square falloff, unshadowed).
    int np = int(u.counts.x);
    for (int i = 0; i < np; ++i) {
        vec3  d    = u.pointPos[i].xyz - viewPos;
        vec3  L    = normalize(d);
        float dist = length(d);
        vec3  rad  = u.pointColor[i].xyz / (dist * dist);
        Lo += BRDF(N, V, L, albedo, F0, metallic, roughness, rad);
    }

    // Image-based ambient: diffuse irradiance + specular prefilter/BRDF. The maps
    // are world-space cubemaps, so rotate the view-space normal/reflection to world
    // (dot(N,V) is rotation-invariant, so the BRDF lookup stays in view space).
    mat3 V2W   = mat3(u.invView);
    vec3 worldN = normalize(V2W * N);
    vec3 worldR = normalize(V2W * reflect(-V, N));
    float NdotV = max(dot(N, V), 0.0);

    vec3 F  = fresnelSchlickRoughness(NdotV, F0, roughness);
    vec3 kD = (1.0 - F) * (1.0 - metallic);

    vec3 diffuse         = texture(irradianceMap, worldN).rgb * albedo;
    vec3 prefilteredColor = textureLod(prefilterMap, worldR, roughness * u.counts.y).rgb;
    vec2 brdf            = texture(brdfLUT, vec2(NdotV, roughness)).rg;
    vec3 specular        = prefilteredColor * (F * brdf.x + brdf.y);

    // Both baked AO and SSAO attenuate the ambient term.
    vec3 ambient  = (kD * diffuse + specular) * ao * occlusion;
    vec3 emissive = texture(gEmissive, vUV).rgb;

    outColor = vec4(ambient + Lo + emissive, 1.0);
}
