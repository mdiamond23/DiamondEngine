#pragma once

#include <memory>
#include <vector>
#include <string>
#include <glm/glm.hpp>
#include "Renderer/MeshData.h"
#include "Renderer/Material.h"
#include "Renderer/Frustum.h"
#include "Animation/Skeleton.h"
#include "Animation/AnimationClip.h"

// Skinned-character components. Kept in the global namespace to match the other
// ECS components in Scene/Components.h. Populated from GltfImporter::LoadModel.

// Render data for a skinned model: GPU meshes (one per glTF primitive) sharing a
// skeleton, plus its animation clips. The animated pose lives on AnimatorComponent.
struct SkinnedMeshComponent {
    std::vector<std::shared_ptr<Diamond::Mesh>> meshes;
    std::shared_ptr<Diamond::PBRMaterial>       material;
    Diamond::Skeleton                           skeleton;
    std::vector<Diamond::AnimationClip>         clips;
    Diamond::AABB                               localBounds;

    bool visible     = true;
    bool castsShadow = true;
    std::string meshPath;       // for future serialization
};

// Playback state + the per-frame bone-matrix palette produced by the animation
// system. `palette` is uploaded to the skinning shader as `uBones`.
struct AnimatorComponent {
    int   clip    = 0;       // index into SkinnedMeshComponent::clips (-1 = bind pose)
    float time    = 0.0f;    // seconds
    float speed   = 1.0f;
    bool  loop    = true;
    bool  playing = true;

    std::vector<glm::mat4> palette;
};
