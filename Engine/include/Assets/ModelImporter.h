#pragma once

#include <string>
#include <vector>
#include "Renderer/MeshData.h"

namespace Diamond {

// Loads a 3D model file (OBJ, FBX, etc.) via Assimp and returns one MeshData per sub-mesh.
// Pass each MeshData to Mesh::Create() to upload it to the GPU.
class ModelImporter {
public:
    static std::vector<MeshData> Load(const std::string& path);
};

} // namespace Diamond