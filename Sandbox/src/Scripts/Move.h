#pragma once
#include "Scene/Scripting.h"
#include "Scene/Scene.h"
#include "Scene/ComponentRegistry.h"
#include <imgui.h>
#include "Core/Debug.h"
#include "Core/Input.h"
#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>

// ---- Data -------------------------------------------------------------------

struct MoveComponent
{
    float moveSpeed = 5.0f;
};

// ---- Inspector UI -----------------------------------------------------------

template<>
inline void DrawComponentInspector<MoveComponent>(MoveComponent& c)
{
    ImGui::DragFloat("Move Speed", &c.moveSpeed, 0.1f, 0.0f, 1000.0f);
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
        Input::BindAxis("Elevation",  Key::Space, Key::LeftShift);
    }

    void OnUpdate(Scene& scene, float dt) override
    {
        float h = Input::GetAxis("Horizontal");
        float f = Input::GetAxis("Forward");
        float e = Input::GetAxis("Elevation");

        if (h == 0.0f && f == 0.0f && e == 0.0f) return;

        for (auto [entity, comp] : scene.View<MoveComponent>().each())
        {
            if (!scene.Has<TransformComponent>(entity)) continue;
            auto& transform = scene.Get<TransformComponent>(entity);

            glm::vec3 right   = transform.rotation * glm::vec3( 1,  0,  0);
            glm::vec3 forward = transform.rotation * glm::vec3( 0,  0, -1);
            glm::vec3 up      =                      glm::vec3( 0,  1,  0);

            transform.position += (right * h + forward * f + up * e) * comp.moveSpeed * dt;
        }
    }

    void OnDestroy(Scene& scene) override {}
};
