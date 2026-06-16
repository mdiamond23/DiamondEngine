#pragma once
#include "Scene/Scripting.h"
#include "Scene/Scene.h"
#include "Scene/ComponentRegistry.h"
#include "Scene/Physics/PhysicsAPI.h"
#include "Scene/Physics/Rigidbody.h"
#include <imgui.h>
#include <nlohmann/json.hpp>
#include "Core/Input.h"
#include <glm/glm.hpp>

// ---- Data -------------------------------------------------------------------
// Tag a bone with this to drive it by keyboard torque — a hands-on way to feel
// constraint limits. Add it to a Dynamic body that also has a ConstraintComponent
// (a hinge or swing-twist) and push it around with the arrow keys + Q/E.

struct ArmControlComponent
{
    float torqueStrength = 10.0f;   // N·m applied per axis at full input
};

// ---- Inspector UI -----------------------------------------------------------

template<>
inline void DrawComponentInspector<ArmControlComponent>(ArmControlComponent& c)
{
    ImGui::DragFloat("Torque Strength", &c.torqueStrength, 0.5f, 0.0f, 1000.0f);
    ImGui::TextDisabled("Arrows = swing, Q/E = twist (play mode)");
}

// ---- Serialization ----------------------------------------------------------

template<>
inline std::string SerializeComponent<ArmControlComponent>(const ArmControlComponent& c)
{
    nlohmann::json j;
    j["torqueStrength"] = c.torqueStrength;
    return j.dump();
}

template<>
inline void DeserializeComponent<ArmControlComponent>(ArmControlComponent& c, const std::string& data)
{
    auto j = nlohmann::json::parse(data);
    c.torqueStrength = j.value("torqueStrength", 10.0f);
}

// ---- Registration -----------------------------------------------------------

DECLARE_COMPONENT(ArmControlComponent, "ArmControl")

// ---- Behavior ---------------------------------------------------------------

class ArmControlSystem : public GameSystem
{
    DECLARE_SYSTEM(ArmControlSystem, 100)   // pre-physics: applies torque before the step
public:
    void OnStart(Scene& scene) override
    {
        // World-space torque axes. Pitch tips the bone forward/back, yaw swings it
        // left/right (both exercise the swing cone); twist rotates it about Z
        // (exercises the twist range on a swing-twist joint).
        Input::BindAxis("ArmPitch", Key::Up,    Key::Down);
        Input::BindAxis("ArmYaw",   Key::Right, Key::Left);
        Input::BindAxis("ArmTwist", Key::E,     Key::Q);
    }

    void OnUpdate(Scene& scene, float dt) override
    {
        glm::vec3 torque {
            Input::GetAxis("ArmPitch"),
            Input::GetAxis("ArmYaw"),
            Input::GetAxis("ArmTwist")
        };
        if (glm::length(torque) < 1e-4f) return;

        for (auto [entity, ctrl] : scene.View<ArmControlComponent>().each())
        {
            if (!scene.Has<RigidBodyComponent>(entity)) continue;
            auto& rb = scene.Get<RigidBodyComponent>(entity);
            if (rb.bodyType != BodyType::Dynamic) continue;
            Physics::AddTorque(rb, torque * ctrl.torqueStrength);
        }
    }

    void OnDestroy(Scene& scene) override {}
};