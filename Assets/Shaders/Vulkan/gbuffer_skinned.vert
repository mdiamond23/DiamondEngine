#version 450

// Skinned variant of gbuffer.vert — GPU vertex skinning for glTF characters
// (port of Engine/Shaders/Deferred/gbuffer_skinned.vert). Blends up to four bone
// transforms by weight into a skin matrix applied before the model matrix, then
// emits the same view-space position + view-space TBN the static G-buffer does,
// so the shared gbuffer.frag lights it identically.
//
// Set 0 is the unchanged material/camera set (bindings 0-7); the per-entity bone
// palette lives in set 1 so a skinned draw reuses its material's set-0 descriptor
// and only swaps set 1.

layout(location = 0) in vec3  inPos;
layout(location = 1) in vec3  inNormal;
layout(location = 2) in vec2  inUV;
layout(location = 3) in vec3  inTangent;
layout(location = 4) in ivec4 inBoneIDs;
layout(location = 5) in vec4  inWeights;

layout(location = 0) out vec3 vViewPos;
layout(location = 1) out vec2 vUV;
layout(location = 2) out mat3 vTBN;   // consumes locations 2, 3, 4

layout(set = 0, binding = 0) uniform FrameUBO {
    mat4 view;
    mat4 viewProj;
} ubo;

// Palette: bones[i] = boneWorld(i) * inverseBind(i). MAX_BONES matches the GL
// shader (humanoid skeletons stay well under it).
const int MAX_BONES = 100;
layout(set = 1, binding = 0) uniform BoneUBO {
    mat4 bones[MAX_BONES];
} skin;

layout(push_constant) uniform Push {
    mat4 model;
} pc;

void main() {
    // Weighted blend of up to four bone matrices; identity fallback for unweighted
    // vertices so they don't collapse to the origin (matches the GL shader).
    mat4 skinMat = inWeights.x * skin.bones[inBoneIDs.x]
                 + inWeights.y * skin.bones[inBoneIDs.y]
                 + inWeights.z * skin.bones[inBoneIDs.z]
                 + inWeights.w * skin.bones[inBoneIDs.w];
    if (inWeights.x + inWeights.y + inWeights.z + inWeights.w < 1e-4)
        skinMat = mat4(1.0);

    vec4 skinnedPos     = skinMat * vec4(inPos, 1.0);
    vec3 skinnedNormal  = mat3(skinMat) * inNormal;
    vec3 skinnedTangent = mat3(skinMat) * inTangent;

    vec4 worldPos = pc.model * skinnedPos;
    vViewPos = vec3(ubo.view * worldPos);
    vUV      = inUV;

    mat3 normalMatrix = transpose(inverse(mat3(pc.model)));
    vec3 T = normalize(normalMatrix * skinnedTangent);
    vec3 N = normalize(normalMatrix * skinnedNormal);
    T = normalize(T - dot(T, N) * N);
    vec3 B = cross(N, T);
    vTBN = mat3(ubo.view) * mat3(T, B, N);

    gl_Position = ubo.viewProj * worldPos;
}
