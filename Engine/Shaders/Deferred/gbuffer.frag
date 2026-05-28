#version 410 core
layout (location = 0) out vec4 gViewPos;
layout (location = 1) out vec4 gViewNormal;
layout (location = 2) out vec4 gAlbedo;
layout (location = 3) out vec4 gMaterial;   // r=metallic, g=roughness, b=ao

in vec2 TexCoords;
in vec3 ViewPos;
in mat3 TBN_view;

uniform sampler2D albedoMap;
uniform sampler2D normalMap;
uniform sampler2D metallicMap;
uniform sampler2D roughnessMap;
uniform sampler2D aoMap;

void main()
{
    vec3  albedo    = pow(texture(albedoMap,    TexCoords).rgb, vec3(2.2));
    float metallic  = texture(metallicMap,  TexCoords).r;
    float roughness = texture(roughnessMap, TexCoords).r;
    float ao        = texture(aoMap,        TexCoords).r;

    // Normal map (tangent space) → view space
    vec3 N = texture(normalMap, TexCoords).rgb * 2.0 - 1.0;
    N = normalize(TBN_view * N);

    gViewPos    = vec4(ViewPos, 1.0);
    gViewNormal = vec4(N, 0.0);
    gAlbedo     = vec4(albedo, 1.0);
    gMaterial   = vec4(metallic, roughness, ao, 1.0);
}
