#pragma once
#include "Scene/Scripting.h"
#include "Scene/Scene.h"
#include "Scene/ComponentRegistry.h"
#include "Scene/Physics/PhysicsAPI.h"
#include "Scene/Physics/Rigidbody.h"
#include "Scene/Physics/Constraint.h"
#include <imgui.h>
#include <nlohmann/json.hpp>
#include "Core/Input.h"
#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>

// ---- Data -------------------------------------------------------------------
// Tag a bone with this to drive it by keyboard torque — a hands-on way to feel
// constraint limits. Add it to a Dynamic body that also has a ConstraintComponent
// (a hinge or swing-twist) and push it around with the arrow keys + Q/E.

struct ArmControlComponent
{
    float torqueStrength = 10.0f;   // N·m applied per axis at full input (no motor)
    float motorRange     = 60.0f;   // if the bone has a motor: Left/Right drives the
                                    // target across +/- this (deg for Position, deg/s for Velocity)
};

// ---- Inspector UI -----------------------------------------------------------

template<>
inline void DrawComponentInspector<ArmControlComponent>(ArmControlComponent& c)
{
    ImGui::DragFloat("Torque Strength", &c.torqueStrength, 0.5f, 0.0f, 1000.0f);
    ImGui::DragFloat("Motor Range",     &c.motorRange,     1.0f, 0.0f, 100000.0f);
    ImGui::TextDisabled("Arrows = swing, Q/E = twist (play mode)");
    ImGui::TextDisabled("With a motor: Left/Right drives the target");
}

// ---- Serialization ----------------------------------------------------------

template<>
inline std::string SerializeComponent<ArmControlComponent>(const ArmControlComponent& c)
{
    nlohmann::json j;
    j["torqueStrength"] = c.torqueStrength;
    j["motorRange"]     = c.motorRange;
    return j.dump();
}

template<>
inline void DeserializeComponent<ArmControlComponent>(ArmControlComponent& c, const std::string& data)
{
    auto j = nlohmann::json::parse(data);
    c.torqueStrength = j.value("torqueStrength", 10.0f);
    c.motorRange     = j.value("motorRange",     60.0f);
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
        float yaw = Input::GetAxis("ArmYaw");
        glm::vec3 torque { Input::GetAxis("ArmPitch"), yaw, Input::GetAxis("ArmTwist") };

        for (auto [entity, ctrl] : scene.View<ArmControlComponent>().each())
        {
            // Motorized joint → steer its target live (release returns to rest).
            if (scene.Has<ConstraintComponent>(entity)) {
                auto& cc = scene.Get<ConstraintComponent>(entity);
                if (cc.motorMode != MotorMode::Off) {
                    if (cc.type == ConstraintType::SwingTwist) {
                        // Build an orientation from all three axes (pitch/yaw/twist)
                        // and drive the swing-twist motor (Physics::SetMotorTargetOrientation).
                        glm::vec3 euler = glm::radians(torque * ctrl.motorRange);
                        Physics::SetMotorTargetOrientation(cc, glm::quat(euler));
                    } else {
                        // Hinge: Left/Right maps to +/- motorRange (Physics::SetMotorTarget).
                        Physics::SetMotorTarget(cc, yaw * ctrl.motorRange);
                    }
                    continue;
                }
            }

            // Otherwise push the bone with raw torque.
            if (glm::length(torque) < 1e-4f) continue;
            if (!scene.Has<RigidBodyComponent>(entity)) continue;
            auto& rb = scene.Get<RigidBodyComponent>(entity);
            if (rb.bodyType != BodyType::Dynamic) continue;
            Physics::AddTorque(rb, torque * ctrl.torqueStrength);
        }
    }

    void OnDestroy(Scene& scene) override {}
};