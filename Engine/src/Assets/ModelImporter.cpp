#include "Assets/ModelImporter.h"

#include <assimp/Importer.hpp>
#include <assimp/scene.h>
#include <assimp/postprocess.h>
#include <spdlog/spdlog.h>

namespace Diamond {

static MeshData ProcessMesh(aiMesh* mesh)
{
    MeshData data;
    data.Vertices.reserve(mesh->mNumVertices);

    for (uint32_t i = 0; i < mesh->mNumVertices; ++i) {
        Vertex v{};
        v.Position = { mesh->mVertices[i].x, mesh->mVertices[i].y, mesh->mVertices[i].z };

        if (mesh->HasNormals())
            v.Normal = { mesh->mNormals[i].x, mesh->mNormals[i].y, mesh->mNormals[i].z };

        if (mesh->mTextureCoords[0]) {
            v.TexCoords = { mesh->mTextureCoords[0][i].x, mesh->mTextureCoords[0][i].y };
            if (mesh->HasTangentsAndBitangents()) {
                v.Tangent   = { mesh->mTangents[i].x,   mesh->mTangents[i].y,   mesh->mTangents[i].z };
                v.Bitangent = { mesh->mBitangents[i].x, mesh->mBitangents[i].y, mesh->mBitangents[i].z };
            }
        }

        data.Vertices.push_back(v);
    }

    for (uint32_t i = 0; i < mesh->mNumFaces; ++i) {
        const aiFace& face = mesh->mFaces[i];
        for (uint32_t j = 0; j < face.mNumIndices; ++j)
            data.Indices.push_back(face.mIndices[j]);
    }

    return data;
}

static void ProcessNode(aiNode* node, const aiScene* scene, std::vector<MeshData>& out)
{
    for (uint32_t i = 0; i < node->mNumMeshes; ++i)
        out.push_back(ProcessMesh(scene->mMeshes[node->mMeshes[i]]));

    for (uint32_t i = 0; i < node->mNumChildren; ++i)
        ProcessNode(node->mChildren[i], scene, out);
}

std::vector<MeshData> ModelImporter::Load(const std::string& path)
{
    Assimp::Importer importer;
    const aiScene* scene = importer.ReadFile(
        path,
        aiProcess_Triangulate | aiProcess_FlipUVs | aiProcess_CalcTangentSpace
    );

    if (!scene || scene->mFlags & AI_SCENE_FLAGS_INCOMPLETE || !scene->mRootNode) {
        spdlog::error("ModelImporter: {}", importer.GetErrorString());
        return {};
    }

    std::vector<MeshData> meshes;
    ProcessNode(scene->mRootNode, scene, meshes);
    return meshes;
}

} // namespace Diamond