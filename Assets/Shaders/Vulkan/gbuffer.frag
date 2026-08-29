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
//   5 gVelocity   RG16F    screen-space motion vector (UV space, TAA)
// Bindings 1-6 + the params UBO live in the per-material descriptor set the
// SceneRenderer's MaterialCache builds (binding 0 is the shared camera UBO).

layout(location = 0) in vec3 vViewPos;
layout(location = 1) in vec2 vUV;
layout(location = 2) in mat3 vTBN;
layout(location = 5) in vec4 vCurrClip;   // undivided — see the divide in main()
layout(location = 6) in vec4 vPrevClip;

layout(location = 0) out vec4 gViewPos;
layout(location = 1) out vec4 gViewNormal;
layout(location = 2) out vec4 gAlbedo;
layout(location = 3) out vec4 gMaterial;
layout(location = 4) out vec4 gEmissive;
layout(location = 5) out vec2 gVelocity;

layout(set = 0, binding = 1) uniform sampler2D albedoMap;
layout(set = 0, binding = 2) uniform sampler2D normalMap;
layout(set = 0, binding = 3) uniform sampler2D metallicMap;
layout(set = 0, binding = 4) uniform sampler2D roughnessMap;
layout(set = 0, binding = 5) uniform sampler2D aoMap;
layout(set = 0, binding = 6) uniform sampler2D emissiveMap;

layout(set = 0, binding = 7) uniform MaterialUBO {
    vec4  baseColorFactor;
    float uvScale;
    float emissiveStrength;
    float alphaCutoff;       // 0 for Opaque/Blend materials — test never fires
    // Scalar metallic/roughness. The CPU sends 1.0 for a slot that has a real
    // map (so the multiply is a no-op and the texture passes through), and the
    // material's constant for a slot that does not — where the map sampled here
    // is a 1x1 white default, making the multiply the value itself.
    float metallicFactor;
    float roughnessFactor;
} mtl;

void main() {
    vec2 uv = vUV * mtl.uvScale;

    // Alpha-MASK materials (decals) render deferred like opaques but discard
    // below the cutoff — glTF defines the tested alpha as texture.a * factor.a.
    vec4 baseSample = texture(albedoMap, uv);
    if (baseSample.a * mtl.baseColorFactor.a < mtl.alphaCutoff) discard;

    // sRGB → linear (matches GL), then modulated by the constant factor — glTF's
    // baseColorFactor is already linear, not sRGB, so it multiplies in after.
    vec3  albedo    = pow(baseSample.rgb, vec3(2.2)) * mtl.baseColorFactor.rgb;
    float metallic  = texture(metallicMap,  uv).r * mtl.metallicFactor;
    float roughness = texture(roughnessMap, uv).r * mtl.roughnessFactor;
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

    // Perspective divide HERE, not in the vertex shader: NDC is affine in screen
    // space, so interpolating a pre-divided value through the rasterizer's
    // perspective-correct path re-applies a 1/w weighting and skews it wherever
    // w varies across the triangle — worst on large flat quads spanning depth.
    //
    // NDC → UV-space delta. x: uv = ndc*0.5+0.5, so Δuv = Δndc*0.5. y is
    // NEGATED: the backend's negative-height viewport puts NDC y=+1 at texel
    // row 0 and sampler v=0 reads row 0 (see fullscreen.vert), so v runs
    // opposite to NDC y. Stored as a true UV offset: historyUV = uv - gVelocity.
    if (vPrevClip.w <= 0.0) {
        gVelocity = vec2(0.0);   // was behind the previous camera — no history
    } else {
        vec2 dNdc = vCurrClip.xy / vCurrClip.w - vPrevClip.xy / vPrevClip.w;
        gVelocity = vec2(dNdc.x, -dNdc.y) * 0.5;
    }
}
