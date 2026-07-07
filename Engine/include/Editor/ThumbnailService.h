#pragma once

#include <cstdint>
#include <string>
#include <vector>

namespace Diamond {

struct MeshData;
struct PBRMaterial;

// Editor asset previews behind the active render backend (editor-wiring step
// 4): the panels ask for a thumbnail and get back an ImTextureID-compatible
// handle (a GL texture id or a Vulkan descriptor set) they can hand straight
// to ImGui. 0 means the preview failed — the panel draws its flat icon.
//
// The service owns every handle it returns: Release() frees one early (a
// re-baked material preview); anything still live is freed when the backend
// shuts down. Handles are only valid while the backend that created them lives.
class ThumbnailService {
public:
    virtual ~ThumbnailService() = default;

    // Decode an image asset and upload it as a preview texture.
    virtual uint64_t CreateTextureThumbnail(const std::string& path) = 0;

    // Studio-lit render of a (multi-submesh) model, auto-framed by its AABB.
    virtual uint64_t CreateMeshThumbnail(const std::vector<MeshData>& meshes) = 0;

    // Material-ball preview: the shared UV sphere shaded with `mat`.
    virtual uint64_t CreateMaterialThumbnail(const PBRMaterial& mat) = 0;

    // Frees the thumbnail behind `id`. Safe to call with 0 / unknown ids.
    virtual void Release(uint64_t id) = 0;
};

} // namespace Diamond
