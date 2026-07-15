#pragma once
#include <Renderer/Material.h>
#include <Renderer/TextureData.h>
#include <AssetPipeline/AssetRegistry.h>
#include "AssetPathUtils.h"
#include <nlohmann/json.hpp>
#include <array>
#include <fstream>
#include <filesystem>
#include <memory>
#include <string>

// Load/save for the .mat asset — a reusable PBR material (texture paths + scalars)
// stored on disk, like PhysicsMaterialAsset / AnimStateMachineAsset. Editor-side
// because JSON lives in Sandbox; the data model is Diamond::PBRMaterial.

inline std::string NormalizeMaterialPath(const std::string& path)
{
    return std::filesystem::path(path).make_preferred().string();
}

// JSON encoding for AlphaMode — glTF's names, lowercased to match our key style.
inline const char* AlphaModeToString(Diamond::AlphaMode m)
{
    switch (m) {
        case Diamond::AlphaMode::Mask:  return "mask";
        case Diamond::AlphaMode::Blend: return "blend";
        default:                        return "opaque";
    }
}

inline Diamond::AlphaMode AlphaModeFromString(const std::string& s)
{
    if (s == "mask")  return Diamond::AlphaMode::Mask;
    if (s == "blend") return Diamond::AlphaMode::Blend;
    return Diamond::AlphaMode::Opaque;
}

// Populate a material's paths + scalars from JSON and (re)load its textures.
// Textures come through the asset registry, so two materials sharing an albedo
// share one GPU texture (empty path -> nullptr, handled by the registry).
inline void ApplyMaterialJson(Diamond::PBRMaterial& m, const nlohmann::json& j)
{
    m.AlbedoPath       = AssetPaths::Resolve(j.value("albedoPath",       std::string{}));
    m.NormalPath       = AssetPaths::Resolve(j.value("normalPath",       std::string{}));
    m.MetallicPath     = AssetPaths::Resolve(j.value("metallicPath",     std::string{}));
    m.RoughnessPath    = AssetPaths::Resolve(j.value("roughnessPath",    std::string{}));
    m.AOPath           = AssetPaths::Resolve(j.value("aoPath",           std::string{}));
    m.EmissivePath     = AssetPaths::Resolve(j.value("emissivePath",     std::string{}));
    m.EmissiveStrength = j.value("emissiveStrength", 0.0f);
    m.UVScale          = j.value("uvScale",          1.0f);
    auto bc = j.value("baseColorFactor", std::array<float, 4>{ 1.0f, 1.0f, 1.0f, 1.0f });
    m.BaseColorFactor  = glm::vec4(bc[0], bc[1], bc[2], bc[3]);
    m.Mode             = AlphaModeFromString(j.value("alphaMode", std::string{ "opaque" }));
    m.AlphaCutoff      = j.value("alphaCutoff", 0.5f);

    m.Albedo    = Assets::Load<Diamond::Texture>(m.AlbedoPath);
    m.Normal    = Assets::Load<Diamond::Texture>(m.NormalPath);
    m.Metallic  = Assets::Load<Diamond::Texture>(m.MetallicPath);
    m.Roughness = Assets::Load<Diamond::Texture>(m.RoughnessPath);
    m.AO        = Assets::Load<Diamond::Texture>(m.AOPath);
    m.Emissive  = Assets::Load<Diamond::Texture>(m.EmissivePath);
}

inline nlohmann::json MaterialToJson(const Diamond::PBRMaterial& m)
{
    return {
        { "albedoPath",       m.AlbedoPath       },
        { "normalPath",       m.NormalPath       },
        { "metallicPath",     m.MetallicPath     },
        { "roughnessPath",    m.RoughnessPath    },
        { "aoPath",           m.AOPath           },
        { "emissivePath",     m.EmissivePath     },
        { "emissiveStrength", m.EmissiveStrength },
        { "uvScale",          m.UVScale          },
        { "baseColorFactor",  std::array<float, 4>{ m.BaseColorFactor.x, m.BaseColorFactor.y,
                                                     m.BaseColorFactor.z, m.BaseColorFactor.w } },
        { "alphaMode",        AlphaModeToString(m.Mode) },
        { "alphaCutoff",      m.AlphaCutoff             }
    };
}

// Fresh load from disk — no caching. Returns null if the file is missing/invalid.
inline std::shared_ptr<Diamond::PBRMaterial> LoadMaterialAsset(const std::string& path)
{
    std::ifstream f{std::filesystem::path(path)};
    if (!f.is_open()) return nullptr;
    auto j = nlohmann::json::parse(f, nullptr, /*allow_exceptions=*/false);
    if (j.is_discarded()) return nullptr;
    auto m = std::make_shared<Diamond::PBRMaterial>();
    ApplyMaterialJson(*m, j);
    return m;
}

inline bool SaveMaterialAsset(const std::string& path, const Diamond::PBRMaterial& m)
{
    std::ofstream f{std::filesystem::path(path)};
    if (!f.is_open()) return false;
    nlohmann::json j = MaterialToJson(m);
    AssetPaths::MakePortable(j);
    f << j.dump(4);
    return true;
}

// Re-parse a .mat into an ALREADY-SHARED PBRMaterial instance (hot reload) in
// place of a fresh LoadMaterialAsset -- every mesh referencing this exact
// instance picks up the change live. Leaves the material untouched if the file
// is missing or mid-write, so a save-in-progress never leaves it half-applied;
// the caller retries on the next mtime change (same policy as shader/texture
// hot reload).
inline bool ReloadMaterialInPlace(Diamond::PBRMaterial& m, const std::string& path)
{
    std::ifstream f{std::filesystem::path(path)};
    if (!f.is_open()) return false;
    auto j = nlohmann::json::parse(f, nullptr, /*allow_exceptions=*/false);
    if (j.is_discarded()) return false;
    ApplyMaterialJson(m, j);
    return true;
}

// Registry load path for .mat materials. Lives here rather than in
// AssetRegistry.h because it needs LoadMaterialAsset above; any TU that calls
// Assets::Load<PBRMaterial> must include this header (the base template is
// deleted, so forgetting is a compile error, not a silent miss).
namespace Assets {

    template<>
    inline std::shared_ptr<Diamond::PBRMaterial> Load<Diamond::PBRMaterial>(const std::string& path)
    {
        return Detail::LoadCached<Diamond::PBRMaterial>(path, [](const std::string& p) {
            return LoadMaterialAsset(p);
        });
    }

    // Hot reload: poll every live .mat's mtime and re-apply it onto the SAME
    // PBRMaterial instance (not a fresh Load) -- every mesh sharing the
    // material updates live. Texture slots resolve through Assets::Load
    // inside ApplyMaterialJson, so an unchanged slot is a cache hit; a slot
    // whose image changed on disk (same path) is picked up separately by
    // ReloadChangedTextures/ReloadAllTextures below.
    inline void ReloadChangedMaterials()
    {
        Detail::SweepReload<Diamond::PBRMaterial>(/*forceAll=*/false, [](Diamond::PBRMaterial& mat, const std::string& path) {
            if (ReloadMaterialInPlace(mat, path)) spdlog::info("AssetRegistry: reloaded material '{}'", path);
        });
    }

    inline void ReloadAllMaterials()
    {
        Detail::SweepReload<Diamond::PBRMaterial>(/*forceAll=*/true, [](Diamond::PBRMaterial& mat, const std::string& path) {
            if (ReloadMaterialInPlace(mat, path)) spdlog::info("AssetRegistry: reloaded material '{}'", path);
        });
    }

    // Umbrella entry points for the editor's hot-reload driver (main.cpp's
    // frame loop): every asset type with a Reload path today. Meshes/models
    // are deliberately out of scope -- see Docs/asset-pipeline-design.md, §2.
    inline void ReloadChanged()
    {
        ReloadChangedTextures();
        ReloadChangedMaterials();
    }

    inline void ReloadAll()
    {
        ReloadAllTextures();
        ReloadAllMaterials();
    }

    // Cheap precheck for the hot-reload driver: true if ReloadChanged() would
    // actually reload something right now. Pure mtime stats, no GPU/file work,
    // so a caller can skip paying for a WaitIdle (Vulkan's Reload() safety
    // requirement) on ticks where nothing changed.
    inline bool HasPendingReloads()
    {
        return Detail::AnyChanged<Diamond::Texture>() || Detail::AnyChanged<Diamond::PBRMaterial>();
    }

} // namespace Assets

// Shared-instance access: every mesh referencing the same .mat gets the SAME
// PBRMaterial, so editing it (and saving) updates all of them live. Held weakly
// by the registry, so a material is reloaded from disk once nothing references
// it (e.g. after a scene reload), picking up any external edits. Kept as the
// editor-facing name; the cache itself moved into the AssetRegistry.
namespace MaterialLibrary {

    inline std::shared_ptr<Diamond::PBRMaterial> Get(const std::string& path)
    {
        return Assets::Load<Diamond::PBRMaterial>(path);
    }

} // namespace MaterialLibrary
