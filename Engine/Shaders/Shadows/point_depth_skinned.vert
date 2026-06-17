#version 410 core
layout (location = 0) in vec3  aPos;
layout (location = 5) in ivec4 aBoneIDs;
layout (location = 6) in vec4  aWeights;

uniform mat4 model;

const int MAX_BONES = 100;
uniform mat4 uBones[MAX_BONES];

// Emits world-space positions; point_depth.geom expands them to the six cube
// faces. Matches point_depth.vert but applies GPU skinning first.
void main()
{
    mat4 skin = aWeights.x * uBones[aBoneIDs.x]
              + aWeights.y * uBones[aBoneIDs.y]
              + aWeights.z * uBones[aBoneIDs.z]
              + aWeights.w * uBones[aBoneIDs.w];
    if (aWeights.x + aWeights.y + aWeights.z + aWeights.w < 1e-4)
        skin = mat4(1.0);

    gl_Position = model * skin * vec4(aPos, 1.0);
}
