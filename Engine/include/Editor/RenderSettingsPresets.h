#pragma once

#include "Renderer/SceneRenderer.h"   // AutoExposureSettings, Tonemapper
#include "Assets/AssetPathUtils.h"    // ProjectRoot

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <algorithm>
#include <filesystem>
#include <fstream>
#include <string>
#include <vector>

// Named render-settings presets, saved to <project root>/RenderSettings.json.
//
// Everything on the renderer-settings panel that is a LOOK rather than a
// diagnostic: exposure, tonemapper, TAA, the GI knobs, auto-exposure and bloom.
// Deliberately NOT here: anything owned by the scene (sky intensity, the DDGI
// volume's grid) — those serialize with the scene, and duplicating them would
// give two sources of truth for one value.
//
// Header-only for the same reason MaterialAsset.h is: it is a struct plus its
// JSON mapping, and a new .cpp would have to be hand-added to the CMake source
// list. Editor-only — nothing in a packaged runtime reads this file.
namespace Diamond {

// One saved look. Defaults MUST match the renderer's own, so a preset file that
// predates a new field loads that field at the renderer default rather than 0.
struct RenderSettings {
    float exposure   = 1.0f;
    int   tonemapper = static_cast<int>(Tonemapper::ACES);
    bool  taa        = true;

    // Screen-space occlusion. 'ssaoStrength' is how much GTAO visibility
    // attenuates the indirect diffuse; the rest is GTAO's own quality/look.
    float ssaoStrength     = 1.0f;
    float gtaoRadius       = 0.8f;
    int   gtaoSlices       = 3;
    int   gtaoSteps        = 6;
    bool  gtaoBentNormals  = true;

    bool  ssgiEnabled     = true;
    // Rays per half-res pixel per frame. The trace's cost is linear in this and
    // it is one of the two most expensive passes in the frame; the temporal pass
    // accumulates across frames, so halving it mostly costs convergence time
    // rather than steady-state quality. Slider goes to 32 if you want it back.
    int   ssgiRays        = 4;
    float ssgiIntensity   = 1.0f;
    float ssgiMaxDistance = 8.0f;

    bool  ddgiEnabled = true;
    // RT reflections fill SSR's misses; off leaves SSR's result untouched.
    bool  rtReflections = true;

    float bloomThreshold = 1.0f;
    float bloomIntensity = 1.0f;

    AutoExposureSettings autoExposure{};
};

inline nlohmann::json ToJson(const RenderSettings& s)
{
    return nlohmann::json{
        { "exposure",        s.exposure },
        { "tonemapper",      s.tonemapper },
        { "taa",             s.taa },
        { "ssaoStrength",    s.ssaoStrength },
        { "gtaoRadius",      s.gtaoRadius },
        { "gtaoSlices",      s.gtaoSlices },
        { "gtaoSteps",       s.gtaoSteps },
        { "gtaoBentNormals", s.gtaoBentNormals },
        { "ssgiEnabled",     s.ssgiEnabled },
        { "ssgiRays",        s.ssgiRays },
        { "ssgiIntensity",   s.ssgiIntensity },
        { "ssgiMaxDistance", s.ssgiMaxDistance },
        { "ddgiEnabled",     s.ddgiEnabled },
        { "rtReflections",   s.rtReflections },
        { "bloomThreshold",  s.bloomThreshold },
        { "bloomIntensity",  s.bloomIntensity },
        { "autoExposure", {
            { "enabled",         s.autoExposure.enabled },
            { "keyValue",        s.autoExposure.keyValue },
            { "minLogLuminance", s.autoExposure.minLogLuminance },
            { "maxLogLuminance", s.autoExposure.maxLogLuminance },
            { "speed",           s.autoExposure.speed },
        } },
    };
}

// Every field goes through j.value(key, default), so a file written by an older
// build simply leaves the new fields at their renderer defaults.
inline RenderSettings RenderSettingsFromJson(const nlohmann::json& j)
{
    RenderSettings s;
    s.exposure        = j.value("exposure",        s.exposure);
    s.tonemapper      = j.value("tonemapper",      s.tonemapper);
    s.taa             = j.value("taa",             s.taa);
    s.ssaoStrength    = j.value("ssaoStrength",    s.ssaoStrength);
    s.gtaoRadius      = j.value("gtaoRadius",      s.gtaoRadius);
    s.gtaoSlices      = j.value("gtaoSlices",      s.gtaoSlices);
    s.gtaoSteps       = j.value("gtaoSteps",       s.gtaoSteps);
    s.gtaoBentNormals = j.value("gtaoBentNormals", s.gtaoBentNormals);
    s.ssgiEnabled     = j.value("ssgiEnabled",     s.ssgiEnabled);
    s.ssgiRays        = j.value("ssgiRays",        s.ssgiRays);
    s.ssgiIntensity   = j.value("ssgiIntensity",   s.ssgiIntensity);
    s.ssgiMaxDistance = j.value("ssgiMaxDistance", s.ssgiMaxDistance);
    s.ddgiEnabled     = j.value("ddgiEnabled",     s.ddgiEnabled);
    s.rtReflections   = j.value("rtReflections",   s.rtReflections);
    s.bloomThreshold  = j.value("bloomThreshold",  s.bloomThreshold);
    s.bloomIntensity  = j.value("bloomIntensity",  s.bloomIntensity);

    if (j.contains("autoExposure") && j["autoExposure"].is_object()) {
        const nlohmann::json& a = j["autoExposure"];
        s.autoExposure.enabled         = a.value("enabled",         s.autoExposure.enabled);
        s.autoExposure.keyValue        = a.value("keyValue",        s.autoExposure.keyValue);
        s.autoExposure.minLogLuminance = a.value("minLogLuminance", s.autoExposure.minLogLuminance);
        s.autoExposure.maxLogLuminance = a.value("maxLogLuminance", s.autoExposure.maxLogLuminance);
        s.autoExposure.speed           = a.value("speed",           s.autoExposure.speed);
    }
    return s;
}

// The whole preset file: named entries plus which one was last applied, so the
// editor reopens on the look you left it in.
class RenderSettingsStore {
public:
    struct Entry {
        std::string    name;
        RenderSettings settings;
    };

    static std::filesystem::path FilePath()
    {
        return AssetPaths::ProjectRoot() / "RenderSettings.json";
    }

    // Missing or unparseable file is not an error — it just means no presets
    // yet. A corrupt file is logged and treated as empty rather than throwing
    // through the editor's startup path.
    void Load()
    {
        m_Entries.clear();
        m_LastUsed.clear();

        const std::filesystem::path path = FilePath();
        std::ifstream in(path);
        if (!in.is_open()) return;

        nlohmann::json j;
        try {
            in >> j;
        } catch (const std::exception& e) {
            spdlog::warn("[RenderSettings] '{}' is not valid JSON ({}) — ignoring",
                         path.generic_string(), e.what());
            return;
        }

        m_LastUsed = j.value("lastUsed", std::string{});
        if (j.contains("presets") && j["presets"].is_object()) {
            for (const auto& [name, body] : j["presets"].items())
                m_Entries.push_back({ name, RenderSettingsFromJson(body) });
        }
        std::sort(m_Entries.begin(), m_Entries.end(),
                  [](const Entry& a, const Entry& b) { return a.name < b.name; });
    }

    bool Save() const
    {
        nlohmann::json presets = nlohmann::json::object();
        for (const Entry& e : m_Entries) presets[e.name] = ToJson(e.settings);

        nlohmann::json j;
        j["lastUsed"] = m_LastUsed;
        j["presets"]  = presets;

        const std::filesystem::path path = FilePath();
        std::ofstream out(path);
        if (!out.is_open()) {
            spdlog::error("[RenderSettings] could not write '{}'", path.generic_string());
            return false;
        }
        out << j.dump(2) << '\n';
        return true;
    }

    const std::vector<Entry>& Entries() const { return m_Entries; }

    const RenderSettings* Find(const std::string& name) const
    {
        for (const Entry& e : m_Entries)
            if (e.name == name) return &e.settings;
        return nullptr;
    }

    // Upsert — saving over an existing name overwrites it, which is what a
    // "Save" button on a selected preset should do.
    void Set(const std::string& name, const RenderSettings& settings)
    {
        if (name.empty()) return;
        for (Entry& e : m_Entries) {
            if (e.name == name) { e.settings = settings; return; }
        }
        m_Entries.push_back({ name, settings });
        std::sort(m_Entries.begin(), m_Entries.end(),
                  [](const Entry& a, const Entry& b) { return a.name < b.name; });
    }

    void Remove(const std::string& name)
    {
        m_Entries.erase(std::remove_if(m_Entries.begin(), m_Entries.end(),
                                       [&](const Entry& e) { return e.name == name; }),
                        m_Entries.end());
        if (m_LastUsed == name) m_LastUsed.clear();
    }

    const std::string& LastUsed() const      { return m_LastUsed; }
    void SetLastUsed(const std::string& n)   { m_LastUsed = n; }

private:
    std::vector<Entry> m_Entries;
    std::string        m_LastUsed;
};

} // namespace Diamond
