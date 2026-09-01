#pragma once
#include "Scene/Scripting.h"
#include "Scene/Scene.h"
#include "Scene/ComponentRegistry.h"
#include "Core/Input.h"
#include <imgui.h>
#include <nlohmann/json.hpp>
#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>
#include <cmath>

// WASD + mouse look, for touring a scene in play mode.
//
// Attach it to the entity carrying the primary CameraComponent. That camera's
// view matrix is the inverse of its world transform (see SceneRenderer), so this
// drives the TransformComponent and nothing else.

// ---- Data -------------------------------------------------------------------

struct FlyCameraComponent
{
    float moveSpeed   = 6.0f;    // units/second
    float sensitivity = 0.1f;    // degrees per pixel — the editor camera's value

    // Euler state, degrees. Kept here rather than read back off the transform
    // every frame, which accumulates error and goes ambiguous looking straight
    // up or down. Seeded once from the authored rotation, then this owns it.
    float yaw    = 0.0f;
    float pitch  = 0.0f;
    bool  seeded = false;   // runtime only — deliberately not serialized
};

// ---- Inspector UI -----------------------------------------------------------

template<>
inline void DrawComponentInspector<FlyCameraComponent>(FlyCameraComponent& c)
{
    ImGui::DragFloat("Move Speed",  &c.moveSpeed,   0.1f,   0.1f,  500.0f);
    ImGui::DragFloat("Sensitivity", &c.sensitivity, 0.005f, 0.005f, 2.0f, "%.3f");
}

// ---- Serialization ----------------------------------------------------------

template<>
inline std::string SerializeComponent<FlyCameraComponent>(const FlyCameraComponent& c)
{
    nlohmann::json j;
    j["moveSpeed"]   = c.moveSpeed;
    j["sensitivity"] = c.sensitivity;
    return j.dump();
}

template<>
inline void DeserializeComponent<FlyCameraComponent>(FlyCameraComponent& c, const std::string& data)
{
    auto j = nlohmann::json::parse(data);
    c.moveSpeed   = j.value("moveSpeed",   6.0f);
    c.sensitivity = j.value("sensitivity", 0.1f);
    // yaw/pitch aren't stored: the authored transform is the single source of
    // where the camera starts looking, so it is re-seeded on every load.
    c.seeded = false;
}

// ---- Registration -----------------------------------------------------------

DECLARE_COMPONENT(FlyCameraComponent, "Fly Camera")

// ---- Behavior ---------------------------------------------------------------

class FlyCameraSystem : public GameSystem
{
    DECLARE_SYSTEM(FlyCameraSystem, 450)   // camera band: after physics + gameplay
public:
    void OnUpdate(Scene& scene, float dt) override
    {
        for (auto [entity, cam] : scene.View<FlyCameraComponent>().each())
        {
            if (!scene.Has<TransformComponent>(entity)) continue;
            auto& xf = scene.Get<TransformComponent>(entity);

            if (!cam.seeded) {
                // Start looking wherever the scene authored the camera, instead
                // of snapping to identity the moment play begins.
                const glm::vec3 f = glm::normalize(xf.rotation * glm::vec3(0.0f, 0.0f, -1.0f));
                cam.pitch  = glm::degrees(std::asin(glm::clamp(f.y, -1.0f, 1.0f)));
                cam.yaw    = glm::degrees(std::atan2(-f.x, -f.z));
                cam.seeded = true;
            }

            // Screen-space delta: x grows right, y grows DOWN. Both are
            // subtracted so dragging right looks right and dragging up looks up.
            const glm::vec2 d = Input::GetMouseDelta();
            cam.yaw   -= d.x * cam.sensitivity;
            cam.pitch  = glm::clamp(cam.pitch - d.y * cam.sensitivity, -89.0f, 89.0f);

            // Yaw about world up, then pitch about the yawed local X: that order
            // never rolls the camera, and yaw = pitch = 0 is identity, which
            // points down -Z — the direction the view matrix treats as forward.
            const glm::quat rot =
                glm::angleAxis(glm::radians(cam.yaw),   glm::vec3(0.0f, 1.0f, 0.0f)) *
                glm::angleAxis(glm::radians(cam.pitch), glm::vec3(1.0f, 0.0f, 0.0f));
            xf.rotation     = rot;
            xf.eulerDegrees = glm::degrees(glm::eulerAngles(rot));

            glm::vec3 dir(0.0f);
            if (Input::IsKeyHeld(Key::W)) dir += rot * glm::vec3(0.0f, 0.0f, -1.0f);
            if (Input::IsKeyHeld(Key::S)) dir -= rot * glm::vec3(0.0f, 0.0f, -1.0f);
            if (Input::IsKeyHeld(Key::D)) dir += rot * glm::vec3(1.0f, 0.0f,  0.0f);
            if (Input::IsKeyHeld(Key::A)) dir -= rot * glm::vec3(1.0f, 0.0f,  0.0f);

            // Normalized so diagonals aren't faster than the cardinals.
            if (glm::dot(dir, dir) > 1e-8f)
                xf.position += glm::normalize(dir) * cam.moveSpeed * dt;
        }
    }
};
