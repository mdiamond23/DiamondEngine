#pragma once
#include <Renderer/Material.h>
#include <Renderer/TextureData.h>
#include <AssetPipeline/AssetRegistry.h>
#include <nlohmann/json.hpp>
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

// Populate a material's paths + scalars from JSON and (re)load its textures.
// Textures come through the asset registry, so two materials sharing an albedo
// share one GPU texture (empty path -> nullptr, handled by the registry).
inline void ApplyMaterialJson(Diamond::PBRMaterial& m, const nlohmann::json& j)
{
    m.AlbedoPath       = j.value("albedoPath",       std::string{});
    m.NormalPath       = j.value("normalPath",       std::string{});
    m.MetallicPath     = j.value("metallicPath",     std::string{});
    m.RoughnessPath    = j.value("roughnessPath",    std::string{});
    m.AOPath           = j.value("aoPath",           std::string{});
    m.EmissivePath     = j.value("emissivePath",     std::string{});
    m.EmissiveStrength = j.value("emissiveStrength", 0.0f);
    m.UVScale          = j.value("uvScale",          1.0f);

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
        { "uvScale",          m.UVScale          }
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
    f << MaterialToJson(m).dump(4);
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
