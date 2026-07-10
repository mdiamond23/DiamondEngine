#pragma once
#include <Renderer/TextureData.h>
#include <Assets/ModelImporter.h>
#include <Assets/GltfImporter.h>
#include <spdlog/spdlog.h>
#include <filesystem>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

// Path-keyed dedup point for engine assets: normalized absolute path -> weak_ptr<T>,
// one map per asset type. Load<T>() returns the live shared_ptr on a cache hit;
// on a miss (or an expired entry, meaning nothing else references the asset
// anymore) it loads from disk and caches a weak_ptr. The registry never owns an
// asset outright -- callers (materials, components) do, so an asset frees itself
// once nothing references it and a later Load() picks it up fresh from disk.
// See Docs/asset-pipeline-design.md, section 1.
//
// Two rules for callers:
//  - Assets are SHARED. Never mutate or std::move out of a registry-loaded
//    asset -- copy what you need into your component.
//  - Textures pin themselves (materials hold them), but mesh/model CPU data is
//    normally dropped right after GPU upload. A caller loading many entities in
//    one pass (scene load, prefab stamp) must keep the shared_ptrs alive for the
//    duration of the pass or entities sharing a file will re-parse it.
//
// Sandbox-owned (not Engine) to match where MaterialLibrary already lives --
// Sandbox links MyEngine one-directionally, so this is reachable from editor and
// game-shell code but not from inside Engine itself.
//
// Specializations here cover Texture, MeshAsset, and ImportedModel; the
// PBRMaterial one lives in Editor/MaterialAsset.h next to its JSON code (it
// loads its textures through this registry, so it must include us, not vice
// versa).
namespace Assets {

// CPU-side result of a static-model import (one MeshData per sub-mesh), wrapped
// so the registry can hand out one shared, immutable copy per file. Feed entries
// to Mesh::Create() for GPU upload.
struct MeshAsset {
    std::vector<Diamond::MeshData> subMeshes;
};

namespace Detail {

// std::filesystem::weakly_canonical resolves ".."/"." segments and slash
// direction and makes the path absolute, without requiring the file to exist
// (unlike canonical). Two spellings of the same file always collapse to one key.
inline std::string NormalizePath(const std::string& path)
{
    std::error_code ec;
    auto canonical = std::filesystem::weakly_canonical(path, ec);
    return ec ? path : canonical.string();
}

template<typename T>
std::unordered_map<std::string, std::weak_ptr<T>>& Cache()
{
    static std::unordered_map<std::string, std::weak_ptr<T>> cache;
    return cache;
}

template<typename T, typename LoadFn>
std::shared_ptr<T> LoadCached(const std::string& path, LoadFn&& loadFn)
{
    if (path.empty()) return nullptr;

    std::string key = NormalizePath(path);
    auto& cache = Cache<T>();

    auto it = cache.find(key);
    if (it != cache.end())
    {
        if (auto sp = it->second.lock())
            return sp;
    }

    // One line per actual disk read makes dedup observable in the console: the
    // same path printing twice within one scene load means a cache/pinning bug.
    spdlog::info("AssetRegistry: disk load '{}'", key);

    auto asset = loadFn(key);
    if (asset) cache[key] = asset;
    else {
        spdlog::warn("AssetRegistry: failed to load '{}'", key);
        cache.erase(key);
    }
    return asset;
}

} // namespace Detail

// No generic definition -- every asset type must provide its own load path
// below (or in MaterialAsset.h for PBRMaterial), so calling
// Load<SomeUnsupportedType>() is a compile error rather than silently doing
// nothing useful.
template<typename T>
std::shared_ptr<T> Load(const std::string& path) = delete;

// flipVertically = false matches every existing Sandbox call site (scene
// textures, material slots, particle textures) -- the project's assets are
// authored for that convention. Don't load the same path with a different flip
// elsewhere: the cache is keyed on path alone, so first load wins.
template<>
inline std::shared_ptr<Diamond::Texture> Load<Diamond::Texture>(const std::string& path)
{
    return Detail::LoadCached<Diamond::Texture>(path, [](const std::string& p) {
        return Diamond::Texture::Create(p, /*flipVertically=*/false);
    });
}

// Static meshes (OBJ/FBX via Assimp; glTF routes through the cgltf geometry path).
template<>
inline std::shared_ptr<MeshAsset> Load<MeshAsset>(const std::string& path)
{
    return Detail::LoadCached<MeshAsset>(path, [](const std::string& p) -> std::shared_ptr<MeshAsset> {
        auto subMeshes = Diamond::ModelImporter::Load(p);
        if (subMeshes.empty()) return nullptr;
        auto asset = std::make_shared<MeshAsset>();
        asset->subMeshes = std::move(subMeshes);
        return asset;
    });
}

// Skinned models: full cgltf import (geometry + skeleton + clips + material).
// Empty meshes means the file was missing or had no geometry -> treated as a
// failed load so the registry doesn't cache a dud.
template<>
inline std::shared_ptr<Diamond::ImportedModel> Load<Diamond::ImportedModel>(const std::string& path)
{
    return Detail::LoadCached<Diamond::ImportedModel>(path, [](const std::string& p) -> std::shared_ptr<Diamond::ImportedModel> {
        auto model = std::make_shared<Diamond::ImportedModel>(Diamond::GltfImporter::LoadModel(p));
        if (model->meshes.empty()) return nullptr;
        return model;
    });
}

} // namespace Assets
