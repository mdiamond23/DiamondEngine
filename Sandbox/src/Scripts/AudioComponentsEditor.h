#pragma once
// Editor bindings for the engine's audio components (defined in Scene/Components.h).
// Registering them with the ComponentRegistry drives the inspector, the Add
// Component menu, scene serialization, and play-mode snapshot/restore generically —
// no edits to InspectorPanel / SceneSerializer. Mirrors ParticleEmitterEditor.h.
#include "Scene/Components.h"
#include "Scene/ComponentRegistry.h"
#include "Audio/AudioAPI.h"
#include <imgui.h>
#include <nlohmann/json.hpp>
#include <algorithm>

// ===========================================================================
// AudioListenerComponent
// ===========================================================================

template<>
inline void DrawComponentInspector<AudioListenerComponent>(AudioListenerComponent& c)
{
    ImGui::Checkbox("Enabled", &c.enabled);
    ImGui::TextDisabled("Drives the 3D listener (usually the camera).");
    ImGui::TextDisabled("Only the first enabled listener is used.");
}

template<>
inline std::string SerializeComponent<AudioListenerComponent>(const AudioListenerComponent& c)
{
    nlohmann::json j;
    j["enabled"] = c.enabled;
    return j.dump();
}

template<>
inline void DeserializeComponent<AudioListenerComponent>(AudioListenerComponent& c, const std::string& data)
{
    auto j = nlohmann::json::parse(data);
    c.enabled = j.value("enabled", true);
}

DECLARE_COMPONENT(AudioListenerComponent, "Audio Listener")

// ===========================================================================
// AudioSourceComponent
// ===========================================================================

template<>
inline void DrawComponentInspector<AudioSourceComponent>(AudioSourceComponent& c)
{
    // -- Clip: drag an AudioClip asset here; double-click-style audition below. --
    std::string clipLabel = c.clipPath.empty() ? "Clip: (none)" : "Clip: " + c.clipPath;
    ImGui::Button(clipLabel.c_str(), ImVec2(-1.0f, 0.0f));
    if (ImGui::BeginDragDropTarget()) {
        if (const ImGuiPayload* p = ImGui::AcceptDragDropPayload("CONTENT_ITEM_PATH")) {
            c.clipPath = std::string(static_cast<const char*>(p->Data));
        }
        ImGui::EndDragDropTarget();
    }
    if (!c.clipPath.empty()) {
        // Edit-mode audition through the dedicated preview voice (separate from play).
        if (Audio::IsPreviewPlaying()) {
            if (ImGui::SmallButton("Stop")) Audio::StopPreview();
        } else {
            if (ImGui::SmallButton("Preview")) Audio::PreviewClip(c.clipPath);
        }
        ImGui::SameLine();
        if (ImGui::SmallButton("Clear Clip")) c.clipPath.clear();
    }

    ImGui::Separator();

    // -- Routing / level --
    const char* buses[] = { "Master", "Music", "SFX", "UI", "Ambience" };
    int busIdx = (int)c.bus;
    if (ImGui::Combo("Bus", &busIdx, buses, IM_ARRAYSIZE(buses)))
        c.bus = (Audio::Bus)busIdx;

    ImGui::DragFloat("Volume", &c.volume, 0.01f, 0.0f, 4.0f);
    ImGui::DragFloat("Pitch",  &c.pitch,  0.01f, 0.01f, 4.0f);

    ImGui::Checkbox("Loop", &c.loop);
    ImGui::SameLine();
    ImGui::Checkbox("Play On Start", &c.playOnStart);
    ImGui::Checkbox("Stream (long tracks)", &c.stream);

    ImGui::Separator();

    // -- 3D spatialization --
    ImGui::Checkbox("3D Spatialized", &c.is3D);
    if (c.is3D) {
        const char* models[] = { "None", "Inverse", "Linear", "Exponential" };
        int modelIdx = (int)c.attenuation;
        if (ImGui::Combo("Attenuation", &modelIdx, models, IM_ARRAYSIZE(models)))
            c.attenuation = (Audio::AttenuationModel)modelIdx;

        ImGui::DragFloat("Min Distance", &c.minDistance, 0.05f, 0.0f, 10000.0f);
        ImGui::DragFloat("Max Distance", &c.maxDistance, 0.5f,  0.0f, 100000.0f);
        ImGui::DragFloat("Rolloff",      &c.rolloff,     0.01f, 0.0f, 100.0f);

        ImGui::DragFloat("Cone Inner Angle", &c.coneInnerAngle, 1.0f, 0.0f, 360.0f);
        ImGui::DragFloat("Cone Outer Angle", &c.coneOuterAngle, 1.0f, 0.0f, 360.0f);
        ImGui::DragFloat("Cone Outer Gain",  &c.coneOuterGain,  0.01f, 0.0f, 1.0f);
    }
}

template<>
inline std::string SerializeComponent<AudioSourceComponent>(const AudioSourceComponent& c)
{
    nlohmann::json j;
    j["clipPath"]       = c.clipPath;
    j["bus"]            = (int)c.bus;
    j["volume"]         = c.volume;
    j["pitch"]          = c.pitch;
    j["loop"]           = c.loop;
    j["playOnStart"]    = c.playOnStart;
    j["stream"]         = c.stream;
    j["is3D"]           = c.is3D;
    j["attenuation"]    = (int)c.attenuation;
    j["minDistance"]    = c.minDistance;
    j["maxDistance"]    = c.maxDistance;
    j["rolloff"]        = c.rolloff;
    j["coneInnerAngle"] = c.coneInnerAngle;
    j["coneOuterAngle"] = c.coneOuterAngle;
    j["coneOuterGain"]  = c.coneOuterGain;
    return j.dump();
}

template<>
inline void DeserializeComponent<AudioSourceComponent>(AudioSourceComponent& c, const std::string& data)
{
    auto j = nlohmann::json::parse(data);
    c.clipPath       = j.value("clipPath", std::string{});
    c.bus            = (Audio::Bus)j.value("bus", (int)Audio::Bus::SFX);
    c.volume         = j.value("volume", 1.0f);
    c.pitch          = j.value("pitch", 1.0f);
    c.loop           = j.value("loop", false);
    c.playOnStart    = j.value("playOnStart", true);
    c.stream         = j.value("stream", false);
    c.is3D           = j.value("is3D", true);
    c.attenuation    = (Audio::AttenuationModel)j.value("attenuation", (int)Audio::AttenuationModel::Inverse);
    c.minDistance    = j.value("minDistance", 1.0f);
    c.maxDistance    = j.value("maxDistance", 100.0f);
    c.rolloff        = j.value("rolloff", 1.0f);
    c.coneInnerAngle = j.value("coneInnerAngle", 360.0f);
    c.coneOuterAngle = j.value("coneOuterAngle", 360.0f);
    c.coneOuterGain  = j.value("coneOuterGain", 0.0f);
}

DECLARE_COMPONENT(AudioSourceComponent, "Audio Source")
