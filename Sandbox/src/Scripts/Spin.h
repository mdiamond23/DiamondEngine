#pragma once
#include "Scene/Scripting.h"
#include "Scene/Scene.h"
#include "Scene/ComponentRegistry.h"
#include <imgui.h>
#include <nlohmann/json.hpp>
#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>

// Constant rotation about a world-space axis. Built for the reflection showcase
// scene, where the point is to keep something moving in the reflections rather
// than to simulate anything.
//
// Note for anything you attach this to: clear MeshComponent::staticShadowCaster.
// The static shadow cache only re-renders when the light moves or the caster set
// changes, so a script-spun mesh left flagged static keeps casting the shadow it
// had on the frame the cache was baked.

// ---- Data -------------------------------------------------------------------

struct SpinComponent
{
    glm::vec3 axis             { 0.0f, 1.0f, 0.0f };   // world space, normalized on use
    float     degreesPerSecond = 45.0f;
};

// ---- Inspector UI -----------------------------------------------------------

template<>
inline void DrawComponentInspector<SpinComponent>(SpinComponent& c)
{
    ImGui::DragFloat3("Axis", &c.axis.x, 0.01f, -1.0f, 1.0f);
    ImGui::DragFloat("Degrees / Second", &c.degreesPerSecond, 1.0f, -720.0f, 720.0f);
}

// ---- Serialization ----------------------------------------------------------

template<>
inline std::string SerializeComponent<SpinComponent>(const SpinComponent& c)
{
    nlohmann::json j;
    j["axisX"]            = c.axis.x;
    j["axisY"]            = c.axis.y;
    j["axisZ"]            = c.axis.z;
    j["degreesPerSecond"] = c.degreesPerSecond;
    return j.dump();
}

template<>
inline void DeserializeComponent<SpinComponent>(SpinComponent& c, const std::string& data)
{
    auto j = nlohmann::json::parse(data);
    c.axis = { j.value("axisX", 0.0f), j.value("axisY", 1.0f), j.value("axisZ", 0.0f) };
    c.degreesPerSecond = j.value("degreesPerSecond", 45.0f);
}

// ---- Registration -----------------------------------------------------------

DECLARE_COMPONENT(SpinComponent, "Spin")

// ---- Behavior ---------------------------------------------------------------

class SpinSystem : public GameSystem
{
    DECLARE_SYSTEM(SpinSystem, 100)
public:
    void OnUpdate(Scene& scene, float dt) override
    {
        for (auto [entity, spin] : scene.View<SpinComponent>().each())
        {
            if (!scene.Has<TransformComponent>(entity)) continue;
            auto& xf = scene.Get<TransformComponent>(entity);

            const float len = glm::length(spin.axis);
            if (len < 1e-4f) continue;   // no axis, no rotation

            // Pre-multiplied, so the axis is world space rather than the
            // object's own — three tori tilted differently then still share one
            // sense of "up" and read as a set.
            const glm::quat step = glm::angleAxis(
                glm::radians(spin.degreesPerSecond * dt), spin.axis / len);
            xf.rotation = glm::normalize(step * xf.rotation);

            // eulerDegrees is only the editor's display cache, but leaving it
            // stale makes the inspector lie about a spinning object.
            xf.eulerDegrees = glm::degrees(glm::eulerAngles(xf.rotation));
        }
    }
};
