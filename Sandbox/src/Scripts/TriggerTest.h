#pragma once
#include "Scene/Scripting.h"
#include "Scene/Scene.h"
#include "Scene/ComponentRegistry.h"
#include "Scene/Physics/Collision.h"   // ColliderComponent
#include <imgui.h>
#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

// ---- Data -------------------------------------------------------------------

struct TriggerTestComponent
{
    // TODO: Add component fields here
};

// ---- Inspector UI -----------------------------------------------------------

template<>
inline void DrawComponentInspector<TriggerTestComponent>(TriggerTestComponent& c)
{
    // TODO: Add ImGui fields here
    ImGui::Text("TriggerTest");
}

// ---- Serialization ----------------------------------------------------------

template<>
inline std::string SerializeComponent<TriggerTestComponent>(const TriggerTestComponent& c)
{
    nlohmann::json j;
    // TODO: j["field"] = c.field;
    return j.dump();
}

template<>
inline void DeserializeComponent<TriggerTestComponent>(TriggerTestComponent& c, const std::string& data)
{
    auto j = nlohmann::json::parse(data);
    // TODO: c.field = j.value("field", defaultValue);
}

// ---- Registration -----------------------------------------------------------

DECLARE_COMPONENT(TriggerTestComponent, "TriggerTest")

// ---- Behavior ---------------------------------------------------------------

class TriggerTestSystem : public GameSystem
{
    DECLARE_SYSTEM(TriggerTestSystem, 101)
public:
    void OnStart(Scene& scene) override {
        for (auto entity : scene.View<TriggerTestComponent>())
        {
            if (!scene.Has<ColliderComponent>(entity)) continue;
            auto& col = scene.Get<ColliderComponent>(entity);

            col.onTriggerEnter = [](entt::entity other) {
                spdlog::info("Trigger ENTER — other entity: {}", (uint32_t)other);
            };
            col.onTriggerExit = [](entt::entity other) {
                spdlog::info("Trigger EXIT — other entity: {}", (uint32_t)other);
            };
            // For collision stay (non-trigger):
            col.onTriggerStay = [](entt::entity other) {
                spdlog::info("Trigger STAY — other entity: {}", (uint32_t)other);
            };
        }
    }

    void OnUpdate(Scene& scene, float dt) override
    {
    }

    void OnDestroy(Scene& scene) override {}
};
