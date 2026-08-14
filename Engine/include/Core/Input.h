#pragma once
#include <string>
#include <glm/glm.hpp>

struct GLFWwindow;

// ---- Keyboard ---------------------------------------------------------------

enum class Key : int
{
    Space       = 32,
    Apostrophe  = 39,
    Comma       = 44,
    Minus       = 45,
    Period      = 46,
    Slash       = 47,
    Num0 = 48, Num1 = 49, Num2 = 50, Num3 = 51, Num4 = 52,
    Num5 = 53, Num6 = 54, Num7 = 55, Num8 = 56, Num9 = 57,
    Semicolon   = 59,
    Equal       = 61,
    A = 65, B = 66, C = 67, D = 68, E = 69, F = 70,
    G = 71, H = 72, I = 73, J = 74, K = 75, L = 76,
    M = 77, N = 78, O = 79, P = 80, Q = 81, R = 82,
    S = 83, T = 84, U = 85, V = 86, W = 87, X = 88,
    Y = 89, Z = 90,
    Escape      = 256,
    Enter       = 257,
    Tab         = 258,
    Backspace   = 259,
    Insert      = 260,
    Delete      = 261,
    Right       = 262,
    Left        = 263,
    Down        = 264,
    Up          = 265,
    PageUp      = 266,
    PageDown    = 267,
    Home        = 268,
    End         = 269,
    CapsLock    = 280,
    F1  = 290, F2  = 291, F3  = 292, F4  = 293,
    F5  = 294, F6  = 295, F7  = 296, F8  = 297,
    F9  = 298, F10 = 299, F11 = 300, F12 = 301,
    LeftShift   = 340,
    LeftCtrl    = 341,
    LeftAlt     = 342,
    RightShift  = 344,
    RightCtrl   = 345,
    RightAlt    = 346,
};

// ---- Mouse ------------------------------------------------------------------

enum class MouseButton : int
{
    Left   = 0,
    Right  = 1,
    Middle = 2,
};

// ---- Gamepad ----------------------------------------------------------------
// Values match GLFW_GAMEPAD_BUTTON_* constants.
// GLFW maps all controllers (Xbox, PS4/PS5, Switch Pro, etc.) to this layout
// via its built-in SDL gamepad database, so button names are hardware-neutral.

enum class GamepadButton : int
{
    South        = 0,   // A (Xbox) / Cross    (PS) / B (Switch)
    East         = 1,   // B (Xbox) / Circle   (PS) / A (Switch)
    West         = 2,   // X (Xbox) / Square   (PS) / Y (Switch)
    North        = 3,   // Y (Xbox) / Triangle (PS) / X (Switch)
    LeftBumper   = 4,   // LB / L1 / L
    RightBumper  = 5,   // RB / R1 / R
    Back         = 6,   // View / Select / Share / Minus
    Start        = 7,   // Menu  / Options     / Plus
    Guide        = 8,   // Xbox / PS / Home button
    LeftThumb    = 9,   // L3 — left stick click
    RightThumb   = 10,  // R3 — right stick click
    DpadUp       = 11,
    DpadRight    = 12,
    DpadDown     = 13,
    DpadLeft     = 14,
};

// Values match GLFW_GAMEPAD_AXIS_* constants.
// Sticks rest at 0. Triggers rest at -1 and reach +1 when fully pressed;
// Input::GetGamepadAxis remaps triggers to 0..1 automatically.
enum class GamepadAxis : int
{
    LeftX        = 0,
    LeftY        = 1,
    RightX       = 2,
    RightY       = 3,
    LeftTrigger  = 4,
    RightTrigger = 5,
};

// ---- Input namespace --------------------------------------------------------

namespace Input
{
    // GLFW joystick slots polled by the engine. Gamepad ids passed to every API below
    // are zero-based and must be in [0, MaxGamepads).
    inline constexpr int MaxGamepads = 4;

    void Init(GLFWwindow* window);
    void Update();
    void SetEnabled(bool enabled);

    // --- Named bindings (keyboard / mouse) ---
    void BindAction(const std::string& name, Key key);
    void BindAction(const std::string& name, MouseButton button);
    void BindAxis(const std::string& name, Key positiveKey, Key negativeKey);

    // --- Named bindings (gamepad) ---
    // gamepadId is 0-based: 0 = first controller, up to MaxGamepads - 1.
    void BindAction(const std::string& name, GamepadButton button, int gamepadId = 0);
    void BindAxis(const std::string& name, GamepadAxis axis, int gamepadId = 0);

    // --- Named binding queries ---
    bool  IsPressed(const std::string& name);
    bool  IsHeld(const std::string& name);
    bool  IsReleased(const std::string& name);
    float GetAxis(const std::string& name);

    // --- Mouse queries ---
    glm::vec2 GetMousePosition();
    glm::vec2 GetMouseDelta();
    float     GetScrollDelta();

    // --- Raw keyboard queries ---
    bool IsKeyPressed(Key key);
    bool IsKeyHeld(Key key);
    bool IsKeyReleased(Key key);

    // --- Raw mouse queries ---
    bool IsMouseButtonPressed(MouseButton button);
    bool IsMouseButtonHeld(MouseButton button);
    bool IsMouseButtonReleased(MouseButton button);

    // --- Raw gamepad queries ---
    bool  IsGamepadConnected(int gamepadId = 0);
    bool  IsGamepadButtonPressed(GamepadButton button, int gamepadId = 0);
    bool  IsGamepadButtonHeld(GamepadButton button, int gamepadId = 0);
    bool  IsGamepadButtonReleased(GamepadButton button, int gamepadId = 0);
    // Sticks: -1..1 with deadzone applied. Triggers: remapped to 0..1.
    float GetGamepadAxis(GamepadAxis axis, int gamepadId = 0);
}
