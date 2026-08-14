#include "Core/Input.h"

#define GLFW_INCLUDE_NONE
#include <GLFW/glfw3.h>

#include <array>
#include <cmath>
#include <unordered_map>

namespace
{
    // ---- Keyboard / mouse state -------------------------------------------

    constexpr int KeyCount   = 349;
    constexpr int MouseCount = 8;

    std::array<bool, KeyCount>   s_heldKeys{};
    std::array<bool, KeyCount>   s_pressedKeys{};
    std::array<bool, KeyCount>   s_releasedKeys{};

    std::array<bool, MouseCount> s_heldMouse{};
    std::array<bool, MouseCount> s_pressedMouse{};
    std::array<bool, MouseCount> s_releasedMouse{};

    // ---- Gamepad state ----------------------------------------------------

    constexpr int GamepadButtonCount = 15; // GLFW_GAMEPAD_BUTTON_LAST + 1
    constexpr int GamepadAxisCount   = 6;  // GLFW_GAMEPAD_AXIS_LAST  + 1
    constexpr float StickDeadzone   = 0.1f;

    struct GamepadState
    {
        bool connected = false;
        std::array<bool,  GamepadButtonCount> held{};
        std::array<bool,  GamepadButtonCount> pressed{};
        std::array<bool,  GamepadButtonCount> released{};
        std::array<float, GamepadAxisCount>   axes{};
    };

    std::array<GamepadState, Input::MaxGamepads> s_gamepads;

    // ---- Named bindings ---------------------------------------------------

    enum class InputSource { Keyboard, Mouse, Gamepad };

    struct ActionBinding
    {
        InputSource source    = InputSource::Keyboard;
        int         code      = 0;
        int         gamepadId = 0;
    };

    struct AxisBinding
    {
        bool isGamepadAxis = false;
        int  positiveKey   = 0; // keyboard: positive key
        int  negativeKey   = 0; // keyboard: negative key
        int  axisCode      = 0; // gamepad:  axis enum value
        int  gamepadId     = 0; // gamepad:  controller index
    };

    std::unordered_map<std::string, ActionBinding> s_actions;
    std::unordered_map<std::string, AxisBinding>   s_axes;

    // ---- Misc -------------------------------------------------------------

    GLFWwindow* s_window  = nullptr;
    bool        s_enabled = true;

    float s_mouseX = 0.0f,     s_mouseY = 0.0f;
    float s_lastMouseX = 0.0f, s_lastMouseY = 0.0f;
    float s_scrollY = 0.0f;

    // ---- GLFW callbacks ---------------------------------------------------
    // Whoever installed callbacks before Init (e.g. ImGui's GLFW backend, when
    // the editor initializes it first) is chained, not clobbered — GLFW only
    // holds one callback per event, so we forward after recording our state.

    GLFWkeyfun         s_prevKeyCallback         = nullptr;
    GLFWmousebuttonfun s_prevMouseButtonCallback = nullptr;
    GLFWscrollfun      s_prevScrollCallback      = nullptr;

    void KeyCallback(GLFWwindow* window, int key, int scancode, int action, int mods)
    {
        if (s_prevKeyCallback) s_prevKeyCallback(window, key, scancode, action, mods);
        if (key < 0 || key >= KeyCount) return;
        if (action == GLFW_PRESS)   { s_pressedKeys[key] = true;  s_heldKeys[key] = true;  }
        if (action == GLFW_RELEASE) { s_releasedKeys[key] = true; s_heldKeys[key] = false; }
    }

    void MouseButtonCallback(GLFWwindow* window, int button, int action, int mods)
    {
        if (s_prevMouseButtonCallback) s_prevMouseButtonCallback(window, button, action, mods);
        if (button < 0 || button >= MouseCount) return;
        if (action == GLFW_PRESS)   { s_pressedMouse[button] = true;  s_heldMouse[button] = true;  }
        if (action == GLFW_RELEASE) { s_releasedMouse[button] = true; s_heldMouse[button] = false; }
    }

    void ScrollCallback(GLFWwindow* window, double xOffset, double yOffset)
    {
        if (s_prevScrollCallback) s_prevScrollCallback(window, xOffset, yOffset);
        s_scrollY += static_cast<float>(yOffset);
    }
}

namespace Input
{
    void Init(GLFWwindow* window)
    {
        s_window = window;
        s_prevKeyCallback         = glfwSetKeyCallback(window, KeyCallback);
        s_prevMouseButtonCallback = glfwSetMouseButtonCallback(window, MouseButtonCallback);
        s_prevScrollCallback      = glfwSetScrollCallback(window, ScrollCallback);

        double x, y;
        glfwGetCursorPos(window, &x, &y);
        s_mouseX = s_lastMouseX = static_cast<float>(x);
        s_mouseY = s_lastMouseY = static_cast<float>(y);
    }

    void Update()
    {
        // Clear one-frame keyboard / mouse state
        s_pressedKeys.fill(false);
        s_releasedKeys.fill(false);
        s_pressedMouse.fill(false);
        s_releasedMouse.fill(false);

        s_lastMouseX = s_mouseX;
        s_lastMouseY = s_mouseY;
        double x, y;
        glfwGetCursorPos(s_window, &x, &y);
        s_mouseX = static_cast<float>(x);
        s_mouseY = static_cast<float>(y);
        s_scrollY = 0.0f;

        // Poll all gamepads
        for (int gid = 0; gid < Input::MaxGamepads; ++gid)
        {
            GamepadState& gs = s_gamepads[gid];
            gs.pressed.fill(false);
            gs.released.fill(false);

            GLFWgamepadstate state;
            if (glfwGetGamepadState(GLFW_JOYSTICK_1 + gid, &state))
            {
                gs.connected = true;
                for (int b = 0; b < GamepadButtonCount; ++b)
                {
                    bool now    = (state.buttons[b] == GLFW_PRESS);
                    gs.pressed[b]  = now && !gs.held[b];
                    gs.released[b] = !now && gs.held[b];
                    gs.held[b]     = now;
                }
                for (int a = 0; a < GamepadAxisCount; ++a)
                    gs.axes[a] = state.axes[a];
            }
            else
            {
                gs.connected = false;
                gs.held.fill(false);
                gs.axes.fill(0.0f);
            }
        }
    }

    void SetEnabled(bool enabled) { s_enabled = enabled; }

    // ---- Named binding registration ---------------------------------------

    void BindAction(const std::string& name, Key key)
    {
        s_actions[name] = { InputSource::Keyboard, static_cast<int>(key), 0 };
    }

    void BindAction(const std::string& name, MouseButton button)
    {
        s_actions[name] = { InputSource::Mouse, static_cast<int>(button), 0 };
    }

    void BindAction(const std::string& name, GamepadButton button, int gamepadId)
    {
        s_actions[name] = { InputSource::Gamepad, static_cast<int>(button), gamepadId };
    }

    void BindAxis(const std::string& name, Key positiveKey, Key negativeKey)
    {
        s_axes[name] = { false, static_cast<int>(positiveKey), static_cast<int>(negativeKey), 0, 0 };
    }

    void BindAxis(const std::string& name, GamepadAxis axis, int gamepadId)
    {
        s_axes[name] = { true, 0, 0, static_cast<int>(axis), gamepadId };
    }

    // ---- Named binding queries --------------------------------------------

    bool IsPressed(const std::string& name)
    {
        if (!s_enabled) return false;
        auto it = s_actions.find(name);
        if (it == s_actions.end()) return false;
        const auto& b = it->second;
        switch (b.source) {
            case InputSource::Mouse:   return IsMouseButtonPressed(static_cast<MouseButton>(b.code));
            case InputSource::Gamepad: return IsGamepadButtonPressed(static_cast<GamepadButton>(b.code), b.gamepadId);
            default:                   return IsKeyPressed(static_cast<Key>(b.code));
        }
    }

    bool IsHeld(const std::string& name)
    {
        if (!s_enabled) return false;
        auto it = s_actions.find(name);
        if (it == s_actions.end()) return false;
        const auto& b = it->second;
        switch (b.source) {
            case InputSource::Mouse:   return IsMouseButtonHeld(static_cast<MouseButton>(b.code));
            case InputSource::Gamepad: return IsGamepadButtonHeld(static_cast<GamepadButton>(b.code), b.gamepadId);
            default:                   return IsKeyHeld(static_cast<Key>(b.code));
        }
    }

    bool IsReleased(const std::string& name)
    {
        if (!s_enabled) return false;
        auto it = s_actions.find(name);
        if (it == s_actions.end()) return false;
        const auto& b = it->second;
        switch (b.source) {
            case InputSource::Mouse:   return IsMouseButtonReleased(static_cast<MouseButton>(b.code));
            case InputSource::Gamepad: return IsGamepadButtonReleased(static_cast<GamepadButton>(b.code), b.gamepadId);
            default:                   return IsKeyReleased(static_cast<Key>(b.code));
        }
    }

    float GetAxis(const std::string& name)
    {
        if (!s_enabled) return 0.0f;
        auto it = s_axes.find(name);
        if (it == s_axes.end()) return 0.0f;
        const auto& b = it->second;
        if (b.isGamepadAxis)
            return GetGamepadAxis(static_cast<GamepadAxis>(b.axisCode), b.gamepadId);
        float pos = s_heldKeys[b.positiveKey] ? 1.0f : 0.0f;
        float neg = s_heldKeys[b.negativeKey] ? 1.0f : 0.0f;
        return pos - neg;
    }

    // ---- Mouse queries ----------------------------------------------------

    glm::vec2 GetMousePosition() { return { s_mouseX, s_mouseY }; }

    glm::vec2 GetMouseDelta()
    {
        if (!s_enabled) return { 0.0f, 0.0f };
        return { s_mouseX - s_lastMouseX, s_mouseY - s_lastMouseY };
    }

    float GetScrollDelta()
    {
        if (!s_enabled) return 0.0f;
        return s_scrollY;
    }

    // ---- Raw keyboard queries ---------------------------------------------

    bool IsKeyPressed(Key key)
    {
        if (!s_enabled) return false;
        int k = static_cast<int>(key);
        return k >= 0 && k < KeyCount && s_pressedKeys[k];
    }

    bool IsKeyHeld(Key key)
    {
        if (!s_enabled) return false;
        int k = static_cast<int>(key);
        return k >= 0 && k < KeyCount && s_heldKeys[k];
    }

    bool IsKeyReleased(Key key)
    {
        if (!s_enabled) return false;
        int k = static_cast<int>(key);
        return k >= 0 && k < KeyCount && s_releasedKeys[k];
    }

    // ---- Raw mouse queries ------------------------------------------------

    bool IsMouseButtonPressed(MouseButton button)
    {
        if (!s_enabled) return false;
        int b = static_cast<int>(button);
        return b >= 0 && b < MouseCount && s_pressedMouse[b];
    }

    bool IsMouseButtonHeld(MouseButton button)
    {
        if (!s_enabled) return false;
        int b = static_cast<int>(button);
        return b >= 0 && b < MouseCount && s_heldMouse[b];
    }

    bool IsMouseButtonReleased(MouseButton button)
    {
        if (!s_enabled) return false;
        int b = static_cast<int>(button);
        return b >= 0 && b < MouseCount && s_releasedMouse[b];
    }

    // ---- Raw gamepad queries ----------------------------------------------

    bool IsGamepadConnected(int gamepadId)
    {
        return gamepadId >= 0 && gamepadId < MaxGamepads && s_gamepads[gamepadId].connected;
    }

    bool IsGamepadButtonPressed(GamepadButton button, int gamepadId)
    {
        if (!s_enabled || gamepadId < 0 || gamepadId >= MaxGamepads) return false;
        if (!s_gamepads[gamepadId].connected) return false;
        int b = static_cast<int>(button);
        return b >= 0 && b < GamepadButtonCount && s_gamepads[gamepadId].pressed[b];
    }

    bool IsGamepadButtonHeld(GamepadButton button, int gamepadId)
    {
        if (!s_enabled || gamepadId < 0 || gamepadId >= MaxGamepads) return false;
        if (!s_gamepads[gamepadId].connected) return false;
        int b = static_cast<int>(button);
        return b >= 0 && b < GamepadButtonCount && s_gamepads[gamepadId].held[b];
    }

    bool IsGamepadButtonReleased(GamepadButton button, int gamepadId)
    {
        if (!s_enabled || gamepadId < 0 || gamepadId >= MaxGamepads) return false;
        if (!s_gamepads[gamepadId].connected) return false;
        int b = static_cast<int>(button);
        return b >= 0 && b < GamepadButtonCount && s_gamepads[gamepadId].released[b];
    }

    float GetGamepadAxis(GamepadAxis axis, int gamepadId)
    {
        if (!s_enabled || gamepadId < 0 || gamepadId >= MaxGamepads) return 0.0f;
        if (!s_gamepads[gamepadId].connected) return 0.0f;
        int a = static_cast<int>(axis);
        if (a < 0 || a >= GamepadAxisCount) return 0.0f;

        float val = s_gamepads[gamepadId].axes[a];

        // Triggers rest at -1 in GLFW — remap to 0..1
        if (axis == GamepadAxis::LeftTrigger || axis == GamepadAxis::RightTrigger)
            return (val + 1.0f) * 0.5f;

        // Sticks: apply deadzone, then rescale so the live range stays 0..1
        if (std::abs(val) < StickDeadzone) return 0.0f;
        return (val - std::copysign(StickDeadzone, val)) / (1.0f - StickDeadzone);
    }
}
