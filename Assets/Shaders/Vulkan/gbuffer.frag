#version 450

// Deferred G-buffer fragment stage — full port of Engine/Shaders/Deferred/
// gbuffer.frag: the complete 6-texture PBR material (albedo/normal/metallic/
// roughness/ao/emissive) with tangent-space normal mapping. Writes five color
// attachments in one pass; each `location` maps to one target:
//   0 gViewPos    RGBA16F  view-space position
//   1 gViewNormal RGBA16F  view-space normal (normal-mapped)
//   2 gAlbedo     RGBA8    base color (linear)
//   3 gMaterial   RGBA8    r=metallic g=roughness b=ao
//   4 gEmissive   RGBA16F  emissive (HDR)
// Bindings 1-6 + the params UBO live in the per-material descriptor set the
// SceneRenderer's MaterialCache builds (binding 0 is the shared camera UBO).

layout(location = 0) in vec3 vViewPos;
layout(location = 1) in vec2 vUV;
layout(location = 2) in mat3 vTBN;

layout(location = 0) out vec4 gViewPos;
layout(location = 1) out vec4 gViewNormal;
layout(location = 2) out vec4 gAlbedo;
layout(location = 3) out vec4 gMaterial;
layout(location = 4) out vec4 gEmissive;

layout(set = 0, binding = 1) uniform sampler2D albedoMap;
layout(set = 0, binding = 2) uniform sampler2D normalMap;
layout(set = 0, binding = 3) uniform sampler2D metallicMap;
layout(set = 0, binding = 4) uniform sampler2D roughnessMap;
layout(set = 0, binding = 5) uniform sampler2D aoMap;
layout(set = 0, binding = 6) uniform sampler2D emissiveMap;

layout(set = 0, binding = 7) uniform MaterialUBO {
    float uvScale;
    float emissiveStrength;
} mtl;

void main() {
    vec2 uv = vUV * mtl.uvScale;

    vec3  albedo    = pow(texture(albedoMap, uv).rgb, vec3(2.2));   // sRGB → linear (matches GL)
    float metallic  = texture(metallicMap,  uv).r;
    float roughness = texture(roughnessMap, uv).r;
    float ao        = texture(aoMap,        uv).r;

    // Normal map (tangent space) → view space. Only RG is read and Z is
    // reconstructed unconditionally: tangent-space normals always have z > 0,
    // so this is exactly as correct for an uncooked RGBA8 normal map as for a
    // cooked BC5 one (which only stores RG) — no per-texture flag needed.
    vec2 nXY = texture(normalMap, uv).rg * 2.0 - 1.0;
    vec3 N   = vec3(nXY, sqrt(max(0.0, 1.0 - dot(nXY, nXY))));
    N = normalize(vTBN * N);

    // Emissive — already in linear space, scaled by per-material strength
    vec3 emissive = texture(emissiveMap, uv).rgb * mtl.emissiveStrength;

    gViewPos    = vec4(vViewPos, 1.0);
    gViewNormal = vec4(N, 0.0);
    gAlbedo     = vec4(albedo, 1.0);
    gMaterial   = vec4(metallic, roughness, ao, 1.0);
    gEmissive   = vec4(emissive, 1.0);
}
