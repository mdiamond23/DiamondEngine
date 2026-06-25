#pragma once
#include <entt/entt.hpp>

// Gamepad focus navigation for UI buttons. Runs in the same UI pass as
// UIInputSystem (after UISystem::Resolve, before UIRenderSystem::Render): it
// reads resolvedPos/resolvedSize to move a "focused" button around with the
// d-pad / left stick and fires onClick when the activate button is pressed.
//
// Focus is stored per-registry in reg.ctx(), so each scene keeps its own
// selection with no new component or global state. The focused button is shown
// using its existing hoverTint — gamepad focus and mouse hover look identical.
//
// Like UIInputSystem, Update() is agnostic to where input came from: it takes an
// already-decoded UINavInput. UI::PollGamepadNav() builds one from a real pad.
class UINavigationSystem
{
public:
    enum class Dir { None, Up, Down, Left, Right };

    struct UINavInput {
        Dir  move       = Dir::None;  // one directional step this frame (edge-triggered)
        bool activate   = false;      // activate-button press edge -> fire focused onClick
        bool mouseMoved = false;      // mouse is driving the UI -> release gamepad focus
    };

    // Call once per frame, after UIInputSystem::Update (so mouse hover is applied
    // first and gamepad focus highlight wins when the mouse is idle).
    static void Update(entt::registry& reg, const UINavInput& input);
};

namespace UI {

// Reads a controller's d-pad / left stick into a single debounced directional
// step plus the activate (South / Cross / A) edge. Holds its own stick-latch
// state so a held stick steps one button at a time — call once per frame per pad.
// mouseMoved is left false; the caller sets it from mouse activity if needed.
UINavigationSystem::UINavInput PollGamepadNav(int gamepadId = 0);

} // namespace UI
