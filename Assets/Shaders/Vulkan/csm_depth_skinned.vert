#version 450

// Skinned depth pass — CSM cascades and spot shadows both use it (the static
// csm_depth pipeline's skinned sibling). Applies the same bone-weighted skin as
// gbuffer_skinned.vert before the pushed lightSpace * model matrix, so a skinned
// caster's shadow silhouette follows its animated pose.
//
// The depth pass has no set-0 resources, so the bone palette sits at set 1
// (uniformly with every other skinned pipeline) and set 0 is an empty layout.

layout(location = 0) in vec3  inPos;
layout(location = 4) in ivec4 inBoneIDs;
layout(location = 5) in vec4  inWeights;

const int MAX_BONES = 100;
layout(set = 1, binding = 0) uniform BoneUBO {
    mat4 bones[MAX_BONES];
} skin;

layout(push_constant) uniform Push {
    mat4 lightModel;   // lightSpaceMatrix * model
} pc;

void main() {
    mat4 skinMat = inWeights.x * skin.bones[inBoneIDs.x]
                 + inWeights.y * skin.bones[inBoneIDs.y]
                 + inWeights.z * skin.bones[inBoneIDs.z]
                 + inWeights.w * skin.bones[inBoneIDs.w];
    if (inWeights.x + inWeights.y + inWeights.z + inWeights.w < 1e-4)
        skinMat = mat4(1.0);

    gl_Position = pc.lightModel * skinMat * vec4(inPos, 1.0);
}
