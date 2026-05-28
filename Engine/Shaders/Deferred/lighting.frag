#version 410 core
layout (location = 0) out vec4 FragColor;
layout (location = 1) out vec4 BrightColor;

in vec2 TexCoords;

// G-buffer (slots 0-3)
uniform sampler2D gViewPos;
uniform sampler2D gViewNormal;
uniform sampler2D gAlbedo;
uniform sampler2D gMaterial;    // r=metallic  g=roughness  b=ao

// SSAO (slot 4)
uniform sampler2D ssaoTex;

// IBL (slots 5-7, bound by BindIBL)
uniform samplerCube irradianceMap;
uniform samplerCube prefilterMap;
uniform sampler2D   brdfLUT;

// Directional shadow (slot 8)
uniform sampler2D shadowMap;
uniform mat4      lightSpaceMatrix;

// Point shadows (slots 9-12)
uniform samplerCube pointShadowMaps[4];
uniform float       shadowFarPlane;

// Lights
uniform vec3 lightPositions[4];
uniform vec3 lightColors[4];
uniform vec3 sunDirection;
uniform vec3 sunColor;

// Camera / transforms
uniform vec3 camPos;
uniform mat4 view;

const float PI = 3.14159265359;

// ----------------------------------------------------------------------------
float DistributionGGX(vec3 N, vec3 H, float roughness)
{
    float a  = roughness * roughness;
    float a2 = a * a;
    float d  = (dot(N, H) * dot(N, H)) * (a2 - 1.0) + 1.0;
    return a2 / (PI * d * d);
}
float GeometrySchlickGGX(float NdotV, float roughness)
{
    float k = (roughness + 1.0) * (roughness + 1.0) / 8.0;
    return NdotV / (NdotV * (1.0 - k) + k);
}
float GeometrySmith(vec3 N, vec3 V, vec3 L, float roughness)
{
    return GeometrySchlickGGX(max(dot(N, V), 0.0), roughness)
         * GeometrySchlickGGX(max(dot(N, L), 0.0), roughness);
}
vec3 fresnelSchlick(float cosTheta, vec3 F0)
{
    return F0 + (1.0 - F0) * pow(clamp(1.0 - cosTheta, 0.0, 1.0), 5.0);
}
vec3 fresnelSchlickRoughness(float cosTheta, vec3 F0, float roughness)
{
    return F0 + (max(vec3(1.0 - roughness), F0) - F0)
              * pow(clamp(1.0 - cosTheta, 0.0, 1.0), 5.0);
}
// ----------------------------------------------------------------------------
float ShadowCalculation(vec4 fragPosLightSpace, vec3 N, vec3 L)
{
    vec3 proj = fragPosLightSpace.xyz / fragPosLightSpace.w;
    proj = proj * 0.5 + 0.5;
    if (proj.z > 1.0) return 0.0;
    float bias = max(0.005 * (1.0 - dot(N, L)), 0.0005);
    float shadow = 0.0;
    vec2 texelSize = 1.0 / textureSize(shadowMap, 0);
    for (int x = -1; x <= 1; ++x)
        for (int y = -1; y <= 1; ++y)
            shadow += (proj.z - bias) > texture(shadowMap, proj.xy + vec2(x, y) * texelSize).r
                      ? 1.0 : 0.0;
    return shadow / 9.0;
}
const vec3 g_SampleDisk[20] = vec3[](
    vec3( 1, 1, 1), vec3( 1,-1, 1), vec3(-1,-1, 1), vec3(-1, 1, 1),
    vec3( 1, 1,-1), vec3( 1,-1,-1), vec3(-1,-1,-1), vec3(-1, 1,-1),
    vec3( 1, 1, 0), vec3( 1,-1, 0), vec3(-1,-1, 0), vec3(-1, 1, 0),
    vec3( 1, 0, 1), vec3(-1, 0, 1), vec3( 1, 0,-1), vec3(-1, 0,-1),
    vec3( 0, 1, 1), vec3( 0,-1, 1), vec3( 0,-1,-1), vec3( 0, 1,-1)
);
float ShadowCalculationPoint(vec3 fragPos, vec3 lightPos, samplerCube depthMap)
{
    vec3  fragToLight  = fragPos - lightPos;
    float currentDepth = length(fragToLight);
    float shadow       = 0.0;
    float bias         = 0.15;
    float diskRadius   = (1.0 + length(camPos - fragPos) / shadowFarPlane) / 25.0;
    for (int i = 0; i < 20; ++i)
    {
        float closestDepth = texture(depthMap, fragToLight + g_SampleDisk[i] * diskRadius).r
                             * shadowFarPlane;
        if (currentDepth - bias > closestDepth) shadow += 1.0;
    }
    return shadow / 20.0;
}
// ----------------------------------------------------------------------------
void main()
{
    vec3  viewPos    = texture(gViewPos,    TexCoords).xyz;
    vec3  viewNormal = texture(gViewNormal, TexCoords).xyz;
    vec3  albedo     = texture(gAlbedo,     TexCoords).rgb;
    vec3  mat3Data   = texture(gMaterial,   TexCoords).rgb;
    float metallic   = mat3Data.r;
    float roughness  = mat3Data.g;
    float ao         = mat3Data.b;
    float occlusion  = texture(ssaoTex, TexCoords).r;

    // Background pixel — no geometry written
    if (viewPos.z == 0.0) { FragColor = vec4(0.0); BrightColor = vec4(0.0); return; }

    // Reconstruct world space: worldPos = R^T * viewPos + camPos  (view is rigid)
    mat3 V2W      = transpose(mat3(view));
    vec3 WorldPos = V2W * viewPos + camPos;
    vec3 N        = normalize(V2W * viewNormal);

    vec4 FragPosLightSpace = lightSpaceMatrix * vec4(WorldPos, 1.0);

    vec3 V  = normalize(camPos - WorldPos);
    vec3 R  = reflect(-V, N);
    vec3 F0 = mix(vec3(0.04), albedo, metallic);

    // --- Point lights ---
    vec3 Lo = vec3(0.0);
    for (int i = 0; i < 4; ++i)
    {
        vec3  L     = normalize(lightPositions[i] - WorldPos);
        vec3  H     = normalize(V + L);
        float dist  = length(lightPositions[i] - WorldPos);
        vec3  rad   = lightColors[i] / (dist * dist);

        float NDF = DistributionGGX(N, H, roughness);
        float G   = GeometrySmith(N, V, L, roughness);
        vec3  F   = fresnelSchlick(max(dot(H, V), 0.0), F0);

        vec3  spec  = NDF * G * F / (4.0 * max(dot(N, V), 0.0) * max(dot(N, L), 0.0) + 0.0001);
        vec3  kD    = (vec3(1.0) - F) * (1.0 - metallic);
        float NdotL = max(dot(N, L), 0.0);
        float shad  = ShadowCalculationPoint(WorldPos, lightPositions[i], pointShadowMaps[i]);

        Lo += (kD * albedo / PI + spec) * rad * NdotL * (1.0 - shad);
    }

    // --- Directional sun ---
    {
        vec3  L     = normalize(-sunDirection);
        vec3  H     = normalize(V + L);
        float NdotL = max(dot(N, L), 0.0);

        float NDF = DistributionGGX(N, H, roughness);
        float G   = GeometrySmith(N, V, L, roughness);
        vec3  F   = fresnelSchlick(max(dot(H, V), 0.0), F0);

        vec3 spec = NDF * G * F / (4.0 * max(dot(N, V), 0.0) * max(dot(N, L), 0.0) + 0.0001);
        vec3 kD   = (vec3(1.0) - F) * (1.0 - metallic);
        float shad = ShadowCalculation(FragPosLightSpace, N, L);

        Lo += (kD * albedo / PI + spec) * sunColor * NdotL * (1.0 - shad);
    }

    // --- IBL ambient ---
    vec3 F  = fresnelSchlickRoughness(max(dot(N, V), 0.0), F0, roughness);
    vec3 kD = (1.0 - F) * (1.0 - metallic);

    vec3 diffuse         = texture(irradianceMap, N).rgb * albedo;
    vec3 prefilteredColor = textureLod(prefilterMap, R, roughness * 4.0).rgb;
    vec2 brdf            = texture(brdfLUT, vec2(max(dot(N, V), 0.0), roughness)).rg;
    vec3 specular        = prefilteredColor * (F * brdf.x + brdf.y);

    // Baked AO × SSAO both attenuate ambient
    vec3 ambient = (kD * diffuse + specular) * ao * occlusion;

    vec3 color = ambient + Lo;

    FragColor = vec4(color, 1.0);
    BrightColor = dot(color, vec3(0.2126, 0.7152, 0.0722)) > 1.0
                  ? vec4(color, 1.0) : vec4(0.0, 0.0, 0.0, 1.0);
}
