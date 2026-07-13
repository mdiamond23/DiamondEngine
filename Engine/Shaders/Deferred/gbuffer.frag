#version 410 core
layout (location = 0) out vec4 gViewPos;
layout (location = 1) out vec4 gViewNormal;
layout (location = 2) out vec4 gAlbedo;
layout (location = 3) out vec4 gMaterial;   // r=metallic, g=roughness, b=ao
layout (location = 4) out vec4 gEmissive;   // rgb=emissive color (HDR)

in vec2 TexCoords;
in vec3 ViewPos;
in mat3 TBN_view;

uniform sampler2D albedoMap;
uniform sampler2D normalMap;
uniform sampler2D metallicMap;
uniform sampler2D roughnessMap;
uniform sampler2D aoMap;
uniform sampler2D emissiveMap;
uniform float     emissiveStrength;
uniform float     uvScale;

void main()
{
    vec2 uv = TexCoords * uvScale;

    vec3  albedo    = pow(texture(albedoMap,    uv).rgb, vec3(2.2));
    float metallic  = texture(metallicMap,  uv).r;
    float roughness = texture(roughnessMap, uv).r;
    float ao        = texture(aoMap,        uv).r;

    // Normal map (tangent space) → view space. Only RG is read and Z is
    // reconstructed unconditionally: tangent-space normals always have z > 0,
    // so this is exactly as correct for an uncooked RGBA8 normal map as for a
    // cooked BC5 one (which only stores RG) — no per-texture flag needed.
    vec2 nXY = texture(normalMap, uv).rg * 2.0 - 1.0;
    vec3 N   = vec3(nXY, sqrt(max(0.0, 1.0 - dot(nXY, nXY))));
    N = normalize(TBN_view * N);

    // Emissive — already in linear space, scaled by per-material strength
    vec3 emissive = texture(emissiveMap, uv).rgb * emissiveStrength;

    gViewPos    = vec4(ViewPos, 1.0);
    gViewNormal = vec4(N, 0.0);
    gAlbedo     = vec4(albedo, 1.0);
    gMaterial   = vec4(metallic, roughness, ao, 1.0);
    gEmissive   = vec4(emissive, 1.0);
}
