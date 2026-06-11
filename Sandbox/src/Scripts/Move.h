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
#include <glm/gtc/quaternion.hpp>
#include <spdlog/spdlog.h>

// ---- Data -------------------------------------------------------------------

struct MoveComponent
{
    float moveForce  = 12.0f;  // impulse/force magnitude applied per axis
    float jumpForce  = 6.0f;   // vertical impulse on jump
    bool  airControl = true;   // allow input while airborne
};

// ---- Inspector UI -----------------------------------------------------------

template<>
inline void DrawComponentInspector<MoveComponent>(MoveComponent& c)
{
    ImGui::DragFloat("Move Force",  &c.moveForce,  0.1f, 0.0f, 200.0f);
    ImGui::DragFloat("Jump Force",  &c.jumpForce,  0.1f, 0.0f, 100.0f);
    ImGui::Checkbox("Air Control",  &c.airControl);
}

// ---- Serialization ----------------------------------------------------------

template<>
inline std::string SerializeComponent<MoveComponent>(const MoveComponent& c)
{
    nlohmann::json j;
    j["moveForce"]  = c.moveForce;
    j["jumpForce"]  = c.jumpForce;
    j["airControl"] = c.airControl;
    return j.dump();
}

template<>
inline void DeserializeComponent<MoveComponent>(MoveComponent& c, const std::string& data)
{
    auto j       = nlohmann::json::parse(data);
    c.moveForce  = j.value("moveForce",  12.0f);
    c.jumpForce  = j.value("jumpForce",  6.0f);
    c.airControl = j.value("airControl", true);
}

// ---- Registration -----------------------------------------------------------

DECLARE_COMPONENT(MoveComponent, "Move")

// ---- Behavior ---------------------------------------------------------------

class MoveSystem : public GameSystem
{
    DECLARE_SYSTEM(MoveSystem, 100)
public:
    void OnStart(Scene& scene) override
    {
        Input::BindAxis("Horizontal", Key::D,     Key::A);
        Input::BindAxis("Forward",    Key::W,     Key::S);
        Input::BindAction("Jump", Key::Space);
        Input::BindAction("Ray", MouseButton::Left);
        Input::BindAction("Sphere", MouseButton::Right);
    }

    void OnUpdate(Scene& scene, float dt) override
    {
        float h = Input::GetAxis("Horizontal");
        float f = Input::GetAxis("Forward");
        bool  jump = Input::IsPressed("Jump");
        bool castRay = Input::IsPressed("Ray");
        bool castSphere = Input::IsPressed("Sphere");


        for (auto [entity, comp] : scene.View<MoveComponent>().each())
        {
            if (!scene.Has<RigidBodyComponent>(entity)) continue;
            auto& rb = scene.Get<RigidBodyComponent>(entity);
            if (rb.bodyType != BodyType::Dynamic) continue;

            if (!comp.airControl && !IsGrounded(rb)) continue;

            // Build a flat move direction from the entity's yaw rotation
            glm::vec3 right, forward;
            if (scene.Has<TransformComponent>(entity)) {
                auto& xf = scene.Get<TransformComponent>(entity);
                right   = xf.rotation * glm::vec3( 1, 0,  0);
                forward = xf.rotation * glm::vec3( 0, 0, -1);
                // Flatten — no vertical contribution from tilt
                right.y   = 0.0f; right   = glm::length(right)   > 1e-4f ? glm::normalize(right)   : glm::vec3(1,0,0);
                forward.y = 0.0f; forward = glm::length(forward)  > 1e-4f ? glm::normalize(forward) : glm::vec3(0,0,-1);
            } else {
                right   = glm::vec3(1, 0, 0);
                forward = glm::vec3(0, 0, -1);
            }

            glm::vec3 moveDir = right * h + forward * f;
            if (glm::length(moveDir) > 1e-4f)
                Physics::AddForce(rb, glm::normalize(moveDir) * comp.moveForce);

            if (jump && IsGrounded(rb))
                Physics::AddImpulse(rb, glm::vec3(0, comp.jumpForce, 0));

            if (castRay)
            {
                auto& xf = scene.Get<TransformComponent>(entity);
                auto hits = Physics::RaycastMulti(xf.position, glm::vec3(0, -1, 0), 5.0f, entity, true);
                spdlog::info("[RaycastMulti] hits={}", hits.size());
            }
            if (castSphere)
            {
                auto& xf = scene.Get<TransformComponent>(entity);
                auto hits = Physics::SphereCastMulti(xf.position, 0.5f, glm::vec3(0, -1, 0), 5.0f, entity, true);
                spdlog::info("[SphereCastMulti] hits={}", hits.size());
            }
        }
    }

    void OnDestroy(Scene& scene) override {}

private:
    // Simple ground check: body's vertical velocity is near zero and it is active.
    // Replace with a ray-cast ground probe once raycasting is available.
    static bool IsGrounded(const RigidBodyComponent& rb)
    {
        glm::vec3 vel = Physics::GetLinearVelocity(rb);
        return fabsf(vel.y) < 0.5f;
    }
};
