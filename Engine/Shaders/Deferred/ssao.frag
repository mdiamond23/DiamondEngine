#version 410 core
layout (location = 0) out float FragColor;

in vec2 TexCoords;

uniform sampler2D gViewPos;
uniform sampler2D gViewNormal;
uniform sampler2D texNoise;

uniform vec3  samples[64];
uniform mat4  projection;
uniform vec2  noiseScale;    // vec2(screenW / 4.0, screenH / 4.0)

const float radius = 0.5;
const float bias   = 0.025;

void main()
{
    vec3 fragPos = texture(gViewPos,    TexCoords).xyz;
    vec3 normal  = normalize(texture(gViewNormal, TexCoords).xyz);

    // Background pixel — no geometry written here
    if (fragPos.z == 0.0) { FragColor = 1.0; return; }

    // Random rotation vector (tiles the 4x4 noise over the screen)
    vec3 randVec = normalize(texture(texNoise, TexCoords * noiseScale).xyz);

    // Build a TBN that reorients the kernel hemisphere to the surface normal
    vec3 tangent   = normalize(randVec - normal * dot(randVec, normal));
    vec3 bitangent = cross(normal, tangent);
    mat3 TBN       = mat3(tangent, bitangent, normal);

    float occlusion = 0.0;
    for (int i = 0; i < 64; ++i)
    {
        // Kernel sample → view space, offset from fragment
        vec3 samplePos = fragPos + TBN * samples[i] * radius;

        // Project sample to get the UV where we read the actual geometry depth
        vec4 offset = projection * vec4(samplePos, 1.0);
        offset.xyz /= offset.w;
        offset.xyz  = offset.xyz * 0.5 + 0.5;

        float sampleDepth = texture(gViewPos, offset.xy).z;

        // sampleDepth >= samplePos.z means the surface is in front of (or at) the sample
        // point — it's occluding it. (View-space z is negative; "closer" means larger value.)
        float rangeCheck = smoothstep(0.0, 1.0, radius / abs(fragPos.z - sampleDepth));
        occlusion += (sampleDepth >= samplePos.z + bias ? 1.0 : 0.0) * rangeCheck;
    }

    FragColor = 1.0 - (occlusion / 64.0);
}
