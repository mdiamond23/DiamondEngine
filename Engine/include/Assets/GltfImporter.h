#pragma once

#include <string>
#include <vector>
#include <memory>
#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>
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

// One glTF node that carries geometry, flattened to a world-space TRS (empty
// intermediate nodes don't survive import — their transforms are baked in).
// `primitives` indexes into ImportedScene::meshes; two nodes sharing a glTF
// mesh both list the same indices (glTF instancing).
struct SceneNodeInstance {
    std::string       name;
    glm::vec3         position { 0.0f };
    glm::quat         rotation { 1.0f, 0.0f, 0.0f, 0.0f };
    glm::vec3         scale    { 1.0f };
    std::vector<int>  primitives;
};

// A multi-material static scene import (level files like Sponza). `meshes` is
// one MeshData per triangle primitive in *file order* — index-compatible with
// Load()'s output, so a MeshComponent::meshSubIndex means the same submesh
// through either path. Materials are deduped: only ones a surviving primitive
// references get loaded; the rest stay null.
struct ImportedScene {
    std::vector<MeshData>                      meshes;
    std::vector<int>                           primitiveMaterial;    // per mesh: index into materials, -1 = none
    std::vector<std::shared_ptr<PBRMaterial>>  materials;
    std::vector<std::string>                   materialNames;
    std::vector<uint8_t>                       materialTransparent;  // glTF alphaMode != OPAQUE
    std::vector<SceneNodeInstance>             nodes;
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

    // Full static-scene import: per-primitive materials plus the node graph
    // (flattened to world TRS). The way to bring in a level file whose
    // primitives use many different materials — LoadModel's single-material
    // assumption is wrong for those.
    static ImportedScene LoadScene(const std::string& path);

    // Cheap rigged-vs-static check: parses only the glTF JSON structure (no
    // buffers/images loaded) and reports whether the file declares a skin. Used
    // by the editor to badge rigged characters distinctly in the content browser.
    static bool HasSkeleton(const std::string& path);
};

} // namespace Diamond
