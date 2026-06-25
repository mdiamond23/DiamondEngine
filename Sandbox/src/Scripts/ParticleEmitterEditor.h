#pragma once
// Editor bindings for the engine's ParticleEmitterComponent (defined in
// Scene/Components.h). Registering it with the ComponentRegistry drives the
// inspector, the Add Component menu, scene serialization, and play-mode
// snapshot/restore generically — no edits to InspectorPanel / SceneSerializer.
#include "Scene/Components.h"
#include "Scene/ComponentRegistry.h"
#include "Scene/ParticlePresets.h"
#include "Renderer/TextureData.h"
#include <imgui.h>
#include <nlohmann/json.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <algorithm>

// ---- Inspector UI -----------------------------------------------------------

template<>
inline void DrawComponentInspector<ParticleEmitterComponent>(ParticleEmitterComponent& c)
{
    // -- Presets: overwrite the config with a ready-made look. Applied in edit
    // mode, so the (empty) runtime pool is simply reset along with it. --
    ImGui::TextUnformatted("Preset:");
    ImGui::SameLine();
    if (ImGui::SmallButton("Spark Burst")) c = ParticlePresets::SparkBurst();
    ImGui::SameLine();
    if (ImGui::SmallButton("Dust Puff"))   c = ParticlePresets::DustPuff();
    ImGui::SameLine();
    if (ImGui::SmallButton("Fire"))        c = ParticlePresets::Fire();
    ImGui::Separator();

    // -- Emission shape --
    const char* shapes[] = { "Point", "Cone", "Sphere", "Box" };
    int shapeIdx = (int)c.shape;
    if (ImGui::Combo("Shape", &shapeIdx, shapes, IM_ARRAYSIZE(shapes)))
        c.shape = (EmissionShape)shapeIdx;

    switch (c.shape) {
        case EmissionShape::Cone:
            ImGui::DragFloat("Cone Angle", &c.coneAngle, 0.5f, 0.0f, 90.0f);
            ImGui::DragFloat("Cone Radius", &c.coneRadius, 0.01f, 0.0f, 100.0f);
            break;
        case EmissionShape::Sphere:
            ImGui::DragFloat("Sphere Radius", &c.sphereRadius, 0.01f, 0.0f, 100.0f);
            break;
        case EmissionShape::Box:
            ImGui::DragFloat3("Box Half Extents", glm::value_ptr(c.boxHalfExtents), 0.01f, 0.0f, 100.0f);
            break;
        case EmissionShape::Point:
        default:
            break;
    }
    ImGui::DragFloat2("Speed Range", glm::value_ptr(c.speedRange), 0.05f, 0.0f, 1000.0f);

    ImGui::Separator();

    // -- Rate / cap (size_t edited through int proxies) --
    ImGui::DragFloat("Spawn Rate", &c.spawnRate, 0.5f, 0.0f, 100000.0f);
    int maxP = (int)c.maxParticles;
    if (ImGui::DragInt("Max Particles", &maxP, 1.0f, 0, 1000000))
        c.maxParticles = (std::size_t)std::max(0, maxP);

    // -- Bursts --
    int burst = (int)c.burstCount;
    if (ImGui::DragInt("Burst Count", &burst, 1.0f, 0, 1000000))
        c.burstCount = (std::size_t)std::max(0, burst);
    ImGui::DragFloat("Burst Interval", &c.burstInterval, 0.05f, 0.0f, 1000.0f,
                     "%.3f (<=0 = one shot)");

    ImGui::Separator();

    // -- Per-particle ranges --
    ImGui::DragFloat2("Lifetime Range", glm::value_ptr(c.lifetimeRange), 0.05f, 0.0f, 1000.0f);
    ImGui::DragFloat2("Size Range", glm::value_ptr(c.sizeRange), 0.01f, 0.0f, 1000.0f);
    ImGui::DragFloat("End Size", &c.endSize, 0.01f, 0.0f, 1000.0f);
    ImGui::DragFloat3("Gravity", glm::value_ptr(c.gravity), 0.05f);

    ImGui::Separator();

    // -- Color over lifetime --
    ImGui::ColorEdit4("Start Color", glm::value_ptr(c.startColor));
    ImGui::ColorEdit4("End Color",   glm::value_ptr(c.endColor));

    const char* blends[] = { "Alpha", "Additive" };
    int blendIdx = (int)c.blend;
    if (ImGui::Combo("Blend", &blendIdx, blends, IM_ARRAYSIZE(blends)))
        c.blend = (Diamond::ParticleBlend)blendIdx;

    // -- Texture: drag a texture asset here; empty = the renderer's default dot. --
    std::string texLabel = c.texturePath.empty() ? "Texture: (default dot)"
                                                  : "Texture: " + c.texturePath;
    ImGui::Button(texLabel.c_str(), ImVec2(-1.0f, 0.0f));
    if (ImGui::BeginDragDropTarget()) {
        if (const ImGuiPayload* p = ImGui::AcceptDragDropPayload("CONTENT_ITEM_PATH")) {
            std::string path(static_cast<const char*>(p->Data));
            if (auto tex = Diamond::Texture::Create(path, false)) {
                c.texture     = tex;
                c.texturePath = path;
            }
        }
        ImGui::EndDragDropTarget();
    }
    if (!c.texturePath.empty() && ImGui::SmallButton("Clear Texture")) {
        c.texture.reset();
        c.texturePath.clear();
    }
}

// ---- Serialization ----------------------------------------------------------

template<>
inline std::string SerializeComponent<ParticleEmitterComponent>(const ParticleEmitterComponent& c)
{
    nlohmann::json j;
    j["spawnRate"]      = c.spawnRate;
    j["maxParticles"]   = c.maxParticles;
    j["shape"]          = (int)c.shape;
    j["coneAngle"]      = c.coneAngle;
    j["coneRadius"]     = c.coneRadius;
    j["sphereRadius"]   = c.sphereRadius;
    j["boxHalfExtents"] = { c.boxHalfExtents.x, c.boxHalfExtents.y, c.boxHalfExtents.z };
    j["speedRange"]     = { c.speedRange.x, c.speedRange.y };
    j["burstCount"]     = c.burstCount;
    j["burstInterval"]  = c.burstInterval;
    j["lifetimeRange"]  = { c.lifetimeRange.x, c.lifetimeRange.y };
    j["sizeRange"]      = { c.sizeRange.x, c.sizeRange.y };
    j["gravity"]        = { c.gravity.x, c.gravity.y, c.gravity.z };
    j["startColor"]     = { c.startColor.r, c.startColor.g, c.startColor.b, c.startColor.a };
    j["endColor"]       = { c.endColor.r, c.endColor.g, c.endColor.b, c.endColor.a };
    j["endSize"]        = c.endSize;
    j["blend"]          = (int)c.blend;
    j["texturePath"]    = c.texturePath;
    return j.dump();
}

template<>
inline void DeserializeComponent<ParticleEmitterComponent>(ParticleEmitterComponent& c, const std::string& data)
{
    auto j = nlohmann::json::parse(data);

    c.spawnRate    = j.value("spawnRate", 40.0f);
    c.maxParticles = j.value("maxParticles", std::size_t{512});
    c.shape        = (EmissionShape)j.value("shape", (int)EmissionShape::Cone);
    c.coneAngle    = j.value("coneAngle", 25.0f);
    c.coneRadius   = j.value("coneRadius", 0.0f);
    c.sphereRadius = j.value("sphereRadius", 0.5f);
    if (j.contains("boxHalfExtents")) {
        auto& b = j["boxHalfExtents"];
        c.boxHalfExtents = { b[0], b[1], b[2] };
    }
    if (j.contains("speedRange")) {
        auto& s = j["speedRange"];
        c.speedRange = { s[0], s[1] };
    }
    c.burstCount    = j.value("burstCount", std::size_t{0});
    c.burstInterval = j.value("burstInterval", 0.0f);
    if (j.contains("lifetimeRange")) {
        auto& l = j["lifetimeRange"];
        c.lifetimeRange = { l[0], l[1] };
    }
    if (j.contains("sizeRange")) {
        auto& s = j["sizeRange"];
        c.sizeRange = { s[0], s[1] };
    }
    if (j.contains("gravity")) {
        auto& g = j["gravity"];
        c.gravity = { g[0], g[1], g[2] };
    }
    if (j.contains("startColor")) {
        auto& s = j["startColor"];
        c.startColor = { s[0], s[1], s[2], s[3] };
    }
    if (j.contains("endColor")) {
        auto& e = j["endColor"];
        c.endColor = { e[0], e[1], e[2], e[3] };
    }
    c.endSize     = j.value("endSize", 0.0f);
    c.blend       = (Diamond::ParticleBlend)j.value("blend", (int)Diamond::ParticleBlend::Additive);
    c.texturePath = j.value("texturePath", std::string{});
    c.texture     = c.texturePath.empty() ? nullptr
                                          : Diamond::Texture::Create(c.texturePath, false);
}

// ---- Registration (after the specializations above) -------------------------

DECLARE_COMPONENT(ParticleEmitterComponent, "Particle Emitter")
