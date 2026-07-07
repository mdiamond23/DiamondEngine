#version 450

// Skinned variant of point_shadow.vert — distance-cube depth for point lights with
// GPU skinning. Skins the vertex into model space, then transforms by the model
// matrix to WORLD space (which the fragment stage needs for length(frag - light))
// and by the pushed face view-projection to clip space.
//
// Set 0 keeps the shared face-matrix UBO (binding 0); the per-entity bone palette
// is at set 1, matching every other skinned pipeline.

layout(location = 0) in vec3  inPos;
layout(location = 4) in ivec4 inBoneIDs;
layout(location = 5) in vec4  inWeights;

layout(set = 0, binding = 0) uniform PointShadowUBO {
    mat4 faceVP[24];        // [light * 6 + face] — light clip from world
    vec4 lightPosFar[4];    // .xyz world-space light position, .w farPlane
} u;

const int MAX_BONES = 100;
layout(set = 1, binding = 0) uniform BoneUBO {
    mat4 bones[MAX_BONES];
} skin;

layout(push_constant) uniform Push {
    mat4 model;   // per draw
    uint idx;     // per face: light * 6 + face
} pc;

layout(location = 0) out vec3 vWorldPos;
layout(location = 1) flat out vec4 vLightPosFar;

void main() {
    mat4 skinMat = inWeights.x * skin.bones[inBoneIDs.x]
                 + inWeights.y * skin.bones[inBoneIDs.y]
                 + inWeights.z * skin.bones[inBoneIDs.z]
                 + inWeights.w * skin.bones[inBoneIDs.w];
    if (inWeights.x + inWeights.y + inWeights.z + inWeights.w < 1e-4)
        skinMat = mat4(1.0);

    vec4 world   = pc.model * skinMat * vec4(inPos, 1.0);
    vWorldPos    = world.xyz;
    vLightPosFar = u.lightPosFar[pc.idx / 6u];
    gl_Position  = u.faceVP[pc.idx] * world;
}
