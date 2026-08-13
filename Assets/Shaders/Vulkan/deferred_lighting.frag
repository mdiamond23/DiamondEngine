#version 450
#extension GL_GOOGLE_include_directive : require

#include "include/ddgi_common.glsl"

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
// Scope: directional sun with CSM shadows + point lights with distance-cubemap
// shadows + spot lights with perspective shadow maps + image-based lighting
// (diffuse irradiance + specular prefilter/BRDF).
//
// The IBL maps and the point-shadow cubes are world-space cubemaps, but everything
// else here is view space, so invView rotates view-space vectors to world space to
// sample them (and reconstructs the world position for the cube-shadow lookups).

layout(location = 0) in  vec2 vUV;
layout(location = 0) out vec4 outColor;
// Far-field diffuse irradiance this pass used, before the kD/albedo/AO
// multiplier — ssgi_composite.frag subtracts it to replace the term rather
// than stack on it. Carries DDGI probe irradiance once probes exist.
layout(location = 1) out vec4 outIndirect;

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
    mat4 invView;            // view space → world space (IBL + cube-shadow sampling)
    vec4 cascadeSplits;      // x..w = cascade far view-depths (positive)
    vec4 sunDirView;         // .xyz sun direction in view space (normalized)
    vec4 sunColor;           // .xyz
    vec4 pointPos[4];        // .xyz view-space position, .w = radius (falloff window)
    vec4 pointColor[4];      // .xyz radiant intensity
    vec4 pointPosWorld[4];   // .xyz world-space position (cube shadows sample by world direction)
    mat4 spotFromView[4];    // per-spot: view space → spot light clip
    vec4 spotPosView[4];     // .xyz view-space position, .w = cos(outer cone)
    vec4 spotDirView[4];     // .xyz view-space direction (normalized), .w = cos(inner cone)
    vec4 spotColor[4];       // .xyz radiant intensity, .w = range (falloff window)
    vec4 counts;             // x = numPointLights, y = prefilter max LOD,
                             // z = numSpotLights, w = point-shadow far plane
    vec4 ambient;            // x = sky/IBL intensity (SkyLightComponent)
                             // y = giMode: 0/1 = IBL far field, 2 = DDGI probes
                             // z = SSAO strength on the indirect diffuse term
                             // w unused
    // DDGI volume, only the fields DDGISampleIrradiance reads (it is
    // view-independent, so no matrices). Ignored unless giMode == 2.
    vec4  ddgiOrigin;        // xyz = world position of probe (0,0,0)
    vec4  ddgiSpacing;       // xyz = probe spacing
    ivec4 ddgiCounts;        // xyz = probe counts per axis
    vec4  ddgiParams;        // x = normalBias, y = energy, z = viewBias
} u;

// IBL maps (world space). Bound after the resource set is created (bake-time views).
layout(set = 0, binding = 11) uniform samplerCube irradianceMap;
layout(set = 0, binding = 12) uniform samplerCube prefilterMap;
layout(set = 0, binding = 13) uniform sampler2D   brdfLUT;

// Spot shadow maps (graph-owned perspective depth targets, one per spot light).
layout(set = 0, binding = 14) uniform sampler2D spotShadow0;
layout(set = 0, binding = 15) uniform sampler2D spotShadow1;
layout(set = 0, binding = 16) uniform sampler2D spotShadow2;
layout(set = 0, binding = 17) uniform sampler2D spotShadow3;

// Point shadow cubes (raw views bound like the IBL maps — one distance cubemap per
// point light, storing length(frag - light) / farPlane).
layout(set = 0, binding = 18) uniform samplerCube pointShadow0;
layout(set = 0, binding = 19) uniform samplerCube pointShadow1;
layout(set = 0, binding = 20) uniform samplerCube pointShadow2;
layout(set = 0, binding = 21) uniform samplerCube pointShadow3;

// DDGI probe atlases (SceneRenderer-owned, shared by every view). Bound to 1x1
// dummies when there is no volume or no ray-tracing support, so the set is
// always complete and giMode is the only thing that decides whether they matter.
layout(set = 0, binding = 22) uniform sampler2D ddgiIrradiance;
layout(set = 0, binding = 23) uniform sampler2D ddgiVisibility;
layout(set = 0, binding = 24) uniform sampler2D ddgiProbeData;

const float PI = 3.14159265359;

// Windowed inverse-square falloff (Frostbite/Karis). Plain 1/d² never reaches
// zero, but GatherLights culls each light by its radius sphere — a light whose
// glow extended past that sphere would pop off wholesale when the sphere left
// the camera frustum (the "moving darkening" bug). The window forces the
// contribution to zero exactly at the radius, so culling is invisible.
float Falloff(float dist, float radius) {
    float w = clamp(1.0 - pow(dist / radius, 4.0), 0.0, 1.0);
    return (w * w) / max(dist * dist, 1e-4);
}

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

// Lagarde's specular occlusion. A hemispherical AO term is the wrong occluder
// for a narrow reflection lobe — applying it raw blacks out mirror surfaces in
// any crevice — so it is widened toward 1 as roughness falls.
float SpecularOcclusion(float NdotV, float ao, float roughness) {
    return clamp(pow(NdotV + ao, exp2(-16.0 * roughness - 1.0)) - 1.0 + ao, 0.0, 1.0);
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
    // Light NDC → [0,1]; v flipped: the negative-height viewport wrote light
    // NDC y=+1 to shadow-map row 0, which the sampler addresses as v=0.
    vec2  suv     = vec2(0.5 * p.x + 0.5, 0.5 - 0.5 * p.y);
    float current = p.z;                    // [0,1] (GLM_FORCE_DEPTH_ZERO_TO_ONE)
    float bias    = max(0.0025 * (1.0 - dot(N, L)), 0.0005);

    if      (c == 0) return PCF(shadowCascade0, suv, current, bias);
    else if (c == 1) return PCF(shadowCascade1, suv, current, bias);
    else if (c == 2) return PCF(shadowCascade2, suv, current, bias);
    else             return PCF(shadowCascade3, suv, current, bias);
}

// ── Spot shadow (3×3 PCF on the light's perspective depth map) ───────────────
float SpotShadow(int i, vec3 viewPos, vec3 N, vec3 L) {
    vec4 lc = u.spotFromView[i] * vec4(viewPos, 1.0);
    if (lc.w <= 0.0) return 0.0;               // behind the light's near plane
    vec3  p    = lc.xyz / lc.w;
    vec2  suv  = vec2(0.5 * p.x + 0.5, 0.5 - 0.5 * p.y);   // v flipped like the CSM lookup
    float bias = max(0.0025 * (1.0 - dot(N, L)), 0.0005);
    if      (i == 0) return PCF(spotShadow0, suv, p.z, bias);
    else if (i == 1) return PCF(spotShadow1, suv, p.z, bias);
    else if (i == 2) return PCF(spotShadow2, suv, p.z, bias);
    else             return PCF(spotShadow3, suv, p.z, bias);
}

// ── Point shadow (20-tap disk PCF, ported from the GL lighting shader) ───────
// The cubes store length(frag - light) / farPlane and follow the IBL bake's face
// orientation, so a world-space direction samples them exactly like GL.
const vec3 g_SampleDisk[20] = vec3[](
    vec3( 1, 1, 1), vec3( 1,-1, 1), vec3(-1,-1, 1), vec3(-1, 1, 1),
    vec3( 1, 1,-1), vec3( 1,-1,-1), vec3(-1,-1,-1), vec3(-1, 1,-1),
    vec3( 1, 1, 0), vec3( 1,-1, 0), vec3(-1,-1, 0), vec3(-1, 1, 0),
    vec3( 1, 0, 1), vec3(-1, 0, 1), vec3( 1, 0,-1), vec3(-1, 0,-1),
    vec3( 0, 1, 1), vec3( 0,-1, 1), vec3( 0,-1,-1), vec3( 0, 1,-1)
);
float PointDepth(int i, vec3 dir) {
    if      (i == 0) return texture(pointShadow0, dir).r;
    else if (i == 1) return texture(pointShadow1, dir).r;
    else if (i == 2) return texture(pointShadow2, dir).r;
    else             return texture(pointShadow3, dir).r;
}
float PointShadow(int i, vec3 worldPos, vec3 camPosW) {
    vec3  fragToLight  = worldPos - u.pointPosWorld[i].xyz;
    float currentDepth = length(fragToLight);
    float far          = u.counts.w;
    float bias         = 0.15;
    float diskRadius   = (1.0 + length(camPosW - worldPos) / far) / 25.0;
    float shadow = 0.0;
    for (int s = 0; s < 20; ++s) {
        float closest = PointDepth(i, fragToLight + g_SampleDisk[s] * diskRadius) * far;
        if (currentDepth - bias > closest) shadow += 1.0;
    }
    return shadow / 20.0;
}

void main() {
    vec3 viewPos = texture(gViewPos, vUV).xyz;
    // Background pixel — no geometry written (gViewPos cleared to 0).
    if (viewPos.z == 0.0) {
        outColor    = vec4(0.0, 0.0, 0.0, 1.0);
        // Never leave an MRT slot unwritten. The composite early-outs on sky, so
        // the .a occlusion factor here is only ever a placeholder.
        outIndirect = vec4(0.0, 0.0, 0.0, 1.0);
        return;
    }

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

    // World-space position + camera for the cube-shadow lookups (the cubes are
    // world-space, like the IBL maps).
    vec3 worldPos = vec3(u.invView * vec4(viewPos, 1.0));
    vec3 camPosW  = u.invView[3].xyz;

    // Point lights (windowed inverse-square falloff + distance-cubemap shadow).
    int np = int(u.counts.x);
    for (int i = 0; i < np; ++i) {
        vec3  d      = u.pointPos[i].xyz - viewPos;
        float dist   = length(d);
        if (dist >= u.pointPos[i].w) continue;   // beyond the light's radius
        vec3  L      = normalize(d);
        vec3  rad    = u.pointColor[i].xyz * Falloff(dist, u.pointPos[i].w);
        float shadow = PointShadow(i, worldPos, camPosW);
        Lo += BRDF(N, V, L, albedo, F0, metallic, roughness, rad) * (1.0 - shadow);
    }

    // Spot lights (windowed inverse-square falloff × smooth cone attenuation + 2D shadow).
    int ns = int(u.counts.z);
    for (int i = 0; i < ns; ++i) {
        vec3  d        = u.spotPosView[i].xyz - viewPos;
        float dist     = length(d);
        if (dist >= u.spotColor[i].w) continue;   // beyond the light's range
        vec3  L        = normalize(d);
        float cosTheta = dot(-L, u.spotDirView[i].xyz);
        float cosInner = u.spotDirView[i].w;
        float cosOuter = u.spotPosView[i].w;
        float ang      = clamp((cosTheta - cosOuter) / max(cosInner - cosOuter, 1e-4), 0.0, 1.0);
        if (ang <= 0.0) continue;
        vec3  rad    = u.spotColor[i].xyz * Falloff(dist, u.spotColor[i].w) * ang;
        float shadow = SpotShadow(i, viewPos, N, L);
        Lo += BRDF(N, V, L, albedo, F0, metallic, roughness, rad) * (1.0 - shadow);
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

    // Far-field diffuse irradiance — the tier branch. Uniform across the whole
    // fullscreen draw, so it costs nothing measurable and needs no permutation.
    //
    // Sky intensity scales the IBL path only. The probe path already has it: the
    // trace's miss shader folds skyIntensity in when a ray escapes, so scaling
    // again here would apply it twice. Either way it must be folded in before
    // outIndirect is written, or SSGI would subtract an unscaled irradiance and
    // over- or under-shoot by exactly this factor.
    vec3 irradiance;
    if (u.ambient.y > 1.5) {
        DDGIVolume vol;
        vol.origin  = u.ddgiOrigin;
        vol.spacing = u.ddgiSpacing;
        vol.counts  = u.ddgiCounts;
        vol.params  = vec4(0.0, u.ddgiParams.x, 1.0, 0.0);   // energy applied below
        vol.params2 = vec4(0.0, u.ddgiParams.z, 0.0, 0.0);
        // The lookup biases toward the viewer, so this is surface → camera.
        vec3 worldV = normalize(camPosW - worldPos);
        // Energy is applied HERE, not inside the lookup: the probe trace calls
        // the same function for its recursive bounce, and a gain in there sits
        // inside the feedback loop and diverges. See ddgi_common.glsl.
        irradiance  = DDGISampleIrradiance(vol, ddgiIrradiance, ddgiVisibility,
                                           ddgiProbeData, worldPos, worldN, worldV)
                    * u.ddgiParams.y;
    } else {
        irradiance = texture(irradianceMap, worldN).rgb * u.ambient.x;
    }

    vec3 diffuse         = irradiance * albedo;
    vec3 prefilteredColor = textureLod(prefilterMap, worldR, roughness * u.counts.y).rgb * u.ambient.x;
    vec2 brdf            = texture(brdfLUT, vec2(NdotV, roughness)).rg;
    vec3 specular        = prefilteredColor * (F * brdf.x + brdf.y);

    // SSAO now attenuates the indirect DIFFUSE term only, at a tier-dependent
    // strength. Probe irradiance already carries coarse occlusion (Chebyshev
    // visibility) and the SSGI composite gathers its own, so a full-strength
    // ambient multiply here darkens the same crevice three times over. Baked AO
    // is a material property and stays at full strength.
    float aoDiffuse = ao * mix(1.0, occlusion, u.ambient.z);
    float aoSpec    = SpecularOcclusion(NdotV, ao * occlusion, roughness);

    vec3 ambient  = kD * diffuse * aoDiffuse + specular * aoSpec;
    vec3 emissive = texture(gEmissive, vUV).rgb;

    outColor    = vec4(ambient + Lo + emissive, 1.0);
    // .a carries the exact diffuse multiplier this pass applied, so the SSGI
    // composite can reproduce it instead of re-deriving ao × ssao and drifting
    // out of step with whatever this branch decided.
    outIndirect = vec4(irradiance, aoDiffuse);
}
