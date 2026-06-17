#pragma once

#include <string>
#include <vector>
#include <memory>
#include "Renderer/MeshData.h"
#include "Renderer/Material.h"
#include "Animation/Skeleton.h"
#include "Animation/AnimationClip.h"

namespace Diamond {

// A fully imported glTF model: geometry plus (when present) the skeleton and
// animation clips needed for a skinned character. `skeleton.bones` is empty for
// a static model. `material` is the first glTF material found (single-material
// assumption) — null if the file has none.
struct ImportedModel {
    std::vector<MeshData>         meshes;
    Skeleton                      skeleton;
    std::vector<AnimationClip>    animations;
    std::shared_ptr<PBRMaterial>  material;
};

// Loads a glTF / GLB file via cgltf. glTF is the format our animated characters
// come through, so skinning/skeleton support lives here (vs. the Assimp path in
// ModelImporter, used for OBJ/FBX static meshes).
class GltfImporter {
public:
    // Geometry only — one MeshData per primitive. Used by the static-mesh path.
    static std::vector<MeshData> Load(const std::string& path);

    // Geometry + skeleton + animations + per-vertex skinning weights.
    static ImportedModel LoadModel(const std::string& path);
};

} // namespace Diamond
