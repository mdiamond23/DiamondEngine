#version 410 core
layout (location = 0) in vec3  aPos;
layout (location = 1) in vec3  aNormal;
layout (location = 2) in vec2  aTexCoords;
layout (location = 3) in vec3  aTangent;
layout (location = 4) in vec3  aBitangent;
layout (location = 5) in ivec4 aBoneIDs;
layout (location = 6) in vec4  aWeights;

out vec2 TexCoords;
out vec3 ViewPos;
out mat3 TBN_view;

uniform mat4 model;
uniform mat4 view;
uniform mat4 projection;
uniform mat3 normalMatrix;

// Bone-matrix palette: palette[i] = boneWorld(i) * inverseBind(i). Sized to match
// kMaxBoneInfluence's host-side MAX_BONES; humanoid skeletons stay well under this.
const int MAX_BONES = 100;
uniform mat4 uBones[MAX_BONES];

void main()
{
    // Blend up to 4 bone transforms by weight. Fall back to identity for
    // unweighted vertices so they don't collapse to the origin.
    mat4 skin = aWeights.x * uBones[aBoneIDs.x]
              + aWeights.y * uBones[aBoneIDs.y]
              + aWeights.z * uBones[aBoneIDs.z]
              + aWeights.w * uBones[aBoneIDs.w];
    if (aWeights.x + aWeights.y + aWeights.z + aWeights.w < 1e-4)
        skin = mat4(1.0);

    vec4 skinnedPos     = skin * vec4(aPos, 1.0);
    vec3 skinnedNormal  = mat3(skin) * aNormal;
    vec3 skinnedTangent = mat3(skin) * aTangent;

    vec4 worldPos = model * skinnedPos;
    ViewPos   = vec3(view * worldPos);
    TexCoords = aTexCoords;

    // Build world-space TBN then rotate into view space (matches gbuffer.vert).
    vec3 T = normalize(normalMatrix * skinnedTangent);
    vec3 N = normalize(normalMatrix * skinnedNormal);
    T = normalize(T - dot(T, N) * N);
    vec3 B = cross(N, T);
    TBN_view = mat3(view) * mat3(T, B, N);

    gl_Position = projection * vec4(ViewPos, 1.0);
}
