#pragma once
#include <Renderer/Material.h>
#include <Renderer/TextureData.h>
#include <nlohmann/json.hpp>
#include <fstream>
#include <filesystem>
#include <memory>
#include <string>
#include <unordered_map>

// Load/save for the .mat asset — a reusable PBR material (texture paths + scalars)
// stored on disk, like PhysicsMaterialAsset / AnimStateMachineAsset. Editor-side
// because JSON lives in Sandbox; the data model is Diamond::PBRMaterial.

inline std::string NormalizeMaterialPath(const std::string& path)
{
    return std::filesystem::path(path).make_preferred().string();
}

// Populate a material's paths + scalars from JSON and (re)load its textures.
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

    auto load = [](const std::string& p) -> std::shared_ptr<Diamond::Texture> {
        return p.empty() ? nullptr : Diamond::Texture::Create(p, false);
    };
    m.Albedo    = load(m.AlbedoPath);
    m.Normal    = load(m.NormalPath);
    m.Metallic  = load(m.MetallicPath);
    m.Roughness = load(m.RoughnessPath);
    m.AO        = load(m.AOPath);
    m.Emissive  = load(m.EmissivePath);
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

// Shared-instance cache: every mesh referencing the same .mat gets the SAME
// PBRMaterial, so editing it (and saving) updates all of them live. Held weakly,
// so a material is reloaded from disk once nothing references it (e.g. after a
// scene reload), picking up any external edits.
namespace MaterialLibrary {

    inline std::shared_ptr<Diamond::PBRMaterial> Get(const std::string& path)
    {
        static std::unordered_map<std::string, std::weak_ptr<Diamond::PBRMaterial>> cache;
        std::string key = NormalizeMaterialPath(path);
        auto it = cache.find(key);
        if (it != cache.end())
            if (auto sp = it->second.lock()) return sp;
        auto m = LoadMaterialAsset(key);
        if (m) cache[key] = m;
        return m;
    }

} // namespace MaterialLibrary
