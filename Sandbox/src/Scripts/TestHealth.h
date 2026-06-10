#pragma once
#include "Scene/Scripting.h"
#include "Scene/Scene.h"
#include "Scene/ComponentRegistry.h"
#include <imgui.h>
#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

// ---- Data -------------------------------------------------------------------

struct HealthComponent
{
    float max     = 100.0f;
    float current = 100.0f;
    bool  isDead  = false;
};

// ---- Inspector UI -----------------------------------------------------------

template<>
inline void DrawComponentInspector<HealthComponent>(HealthComponent& h)
{
    ImGui::DragFloat("Max HP",     &h.max,     1.0f, 1.0f, 9999.0f);
    ImGui::DragFloat("Current HP", &h.current, 1.0f, 0.0f, h.max);
    ImGui::Checkbox("Dead",        &h.isDead);
}

// ---- Serialization ----------------------------------------------------------

template<>
inline std::string SerializeComponent<HealthComponent>(const HealthComponent& c)
{
    nlohmann::json j;
    j["max"]     = c.max;
    j["current"] = c.current;
    j["isDead"]  = c.isDead;
    return j.dump();
}

template<>
inline void DeserializeComponent<HealthComponent>(HealthComponent& c, const std::string& data)
{
    auto j     = nlohmann::json::parse(data);
    c.max     = j.value("max",     100.0f);
    c.current = j.value("current", 100.0f);
    c.isDead  = j.value("isDead",  false);
}

// ---- Registration -----------------------------------------------------------
// Must come after DrawComponentInspector and SerializeComponent specializations.

DECLARE_COMPONENT(HealthComponent, "Health")

// ---- Behavior ---------------------------------------------------------------

class HealthSystem : public GameSystem
{
    DECLARE_SYSTEM(HealthSystem, 300)
public:
    void OnStart(Scene& scene) override
    {
        spdlog::info("[HealthSystem] OnStart - {} entity/entities with HealthComponent",
            scene.View<HealthComponent>().size());
    }

    void OnUpdate(Scene& scene, float dt) override
    {
        for (auto [entity, health] : scene.View<HealthComponent>().each())
        {
            if (health.isDead) continue;

            health.current -= 10.0f * dt;

            if (health.current > health.max) 
                health.current = health.max;

            if (health.current <= 0.0f)
            {
                health.current = 0.0f;
                health.isDead  = true;
                // Disable mesh
                auto mesh = scene.Get<MeshComponent>(entity);
                if (scene.Has<MeshComponent>(entity)) {
                    auto& mesh = scene.Get<MeshComponent>(entity);
                    mesh.visible = false;
                }
            }

            if (health.current <= health.max * .5f)
            {

            }
        }
    }

    void OnDestroy(Scene& scene) override
    {
        spdlog::info("[HealthSystem] OnDestroy");
    }
};
