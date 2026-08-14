#pragma once

#include "Scene/ComponentRegistry.h"
#include "Scene/Scene.h"
#include "Core/Input.h"

#include <glm/glm.hpp>
#include <imgui.h>
#include <nlohmann/json.hpp>

#include <algorithm>
#include <string>

// Assign this component to a playable character. The gameplay systems sample one
// PlayerCommandState from its controller slot instead of sharing global named input
// bindings, so prefab instances can be independently controlled.
struct PlayerInputComponent
{
    int  gamepadId = 0;       // zero-based GLFW gamepad slot
    bool inputEnabled = true;  // per-character input gate; global Input::SetEnabled still applies
};

// One-frame, gameplay-facing command snapshot. Keeping hardware queries here means
// locomotion and combat consume commands rather than knowing how a controller is laid
// out. AI, replay, or network producers can supply the same shape later.
struct PlayerCommandState
{
    bool connected = false;
    glm::vec2 move { 0.0f };

    bool punchLeftPressed  = false;
    bool punchLeftHeld     = false;
    bool punchLeftReleased = false;
    bool punchRightPressed  = false;
    bool punchRightHeld     = false;
    bool punchRightReleased = false;
};

namespace PlayerInput
{
    inline int SanitizeGamepadId(int gamepadId)
    {
        return std::clamp(gamepadId, 0, Input::MaxGamepads - 1);
    }

    inline PlayerCommandState ReadCommand(const PlayerInputComponent& player)
    {
        PlayerCommandState command;
        if (!player.inputEnabled) return command;

        const int gamepadId = SanitizeGamepadId(player.gamepadId);
        command.connected = Input::IsGamepadConnected(gamepadId);
        if (!command.connected) return command;

        command.move = {
            Input::GetGamepadAxis(GamepadAxis::LeftX, gamepadId),
            -Input::GetGamepadAxis(GamepadAxis::LeftY, gamepadId)
        };

        command.punchLeftPressed = Input::IsGamepadButtonPressed(
            GamepadButton::LeftBumper, gamepadId);
        command.punchLeftHeld = Input::IsGamepadButtonHeld(
            GamepadButton::LeftBumper, gamepadId);
        command.punchLeftReleased = Input::IsGamepadButtonReleased(
            GamepadButton::LeftBumper, gamepadId);
        command.punchRightPressed = Input::IsGamepadButtonPressed(
            GamepadButton::RightBumper, gamepadId);
        command.punchRightHeld = Input::IsGamepadButtonHeld(
            GamepadButton::RightBumper, gamepadId);
        command.punchRightReleased = Input::IsGamepadButtonReleased(
            GamepadButton::RightBumper, gamepadId);
        return command;
    }

    // Compatibility path for scenes authored before PlayerInput existed. Such characters
    // remain player one until the component is added in the inspector or prefab.
    inline PlayerCommandState ReadCommandOrDefault(Scene& scene, entt::entity entity)
    {
        if (scene.Has<PlayerInputComponent>(entity))
            return ReadCommand(scene.Get<PlayerInputComponent>(entity));
        return ReadCommand(PlayerInputComponent{});
    }
}

template<>
inline void DrawComponentInspector<PlayerInputComponent>(PlayerInputComponent& c)
{
    int displayedSlot = PlayerInput::SanitizeGamepadId(c.gamepadId) + 1;
    if (ImGui::SliderInt("Gamepad", &displayedSlot, 1, Input::MaxGamepads))
        c.gamepadId = displayedSlot - 1;
    else
        c.gamepadId = PlayerInput::SanitizeGamepadId(c.gamepadId);

    ImGui::Checkbox("Input Enabled", &c.inputEnabled);
    ImGui::TextDisabled("Controller %d: %s", displayedSlot,
        Input::IsGamepadConnected(c.gamepadId) ? "connected" : "not connected");
}

template<>
inline std::string SerializeComponent<PlayerInputComponent>(const PlayerInputComponent& c)
{
    nlohmann::json j;
    j["gamepadId"] = PlayerInput::SanitizeGamepadId(c.gamepadId);
    j["inputEnabled"] = c.inputEnabled;
    return j.dump();
}

template<>
inline void DeserializeComponent<PlayerInputComponent>(PlayerInputComponent& c,
                                                        const std::string& data)
{
    const auto j = nlohmann::json::parse(data);
    c.gamepadId = PlayerInput::SanitizeGamepadId(j.value("gamepadId", 0));
    c.inputEnabled = j.value("inputEnabled", true);
}

DECLARE_COMPONENT(PlayerInputComponent, "PlayerInput")
