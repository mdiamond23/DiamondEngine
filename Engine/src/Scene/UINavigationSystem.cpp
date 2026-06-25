#include "Scene/UINavigationSystem.h"
#include "Scene/Components.h"
#include "Core/Input.h"

#include <glm/glm.hpp>
#include <limits>

using Dir       = UINavigationSystem::Dir;
using UINavInput = UINavigationSystem::UINavInput;

namespace {

// Per-registry focus, stored in reg.ctx(). Keeps each scene's selection without a
// new component or global. Only this translation unit references the type.
struct UIFocusState {
    entt::entity focused = entt::null;
};

UIFocusState& Focus(entt::registry& reg)
{
    if (auto* fs = reg.ctx().find<UIFocusState>())
        return *fs;
    return reg.ctx().emplace<UIFocusState>();
}

glm::vec2 RectCenter(const RectTransformComponent& rt)
{
    return rt.resolvedPos + rt.resolvedSize * 0.5f;
}

// Top-most, then left-most focusable button — the default selection grabbed by
// the first nav input when nothing is focused yet.
entt::entity FirstButton(entt::registry& reg)
{
    entt::entity best = entt::null;
    glm::vec2    bestC{ 0.0f };
    for (auto e : reg.view<UIButtonComponent, RectTransformComponent>()) {
        const glm::vec2 c = RectCenter(reg.get<RectTransformComponent>(e));
        if (best == entt::null || c.y < bestC.y || (c.y == bestC.y && c.x < bestC.x)) {
            best  = e;
            bestC = c;
        }
    }
    return best;
}

// Nearest button from `from` in direction `dir`. Candidates must lie in the
// direction's half-plane; score prefers straight-ahead by weighting the
// perpendicular offset more than the on-axis distance. Coordinate space is
// top-left origin, +Y down (Up == decreasing Y), matching RectTransform.
entt::entity Neighbor(entt::registry& reg, entt::entity from, Dir dir)
{
    if (from == entt::null || !reg.valid(from) ||
        !reg.all_of<UIButtonComponent, RectTransformComponent>(from))
        return entt::null;

    const glm::vec2 cur = RectCenter(reg.get<RectTransformComponent>(from));

    entt::entity best  = entt::null;
    float        bestScore = std::numeric_limits<float>::max();

    for (auto e : reg.view<UIButtonComponent, RectTransformComponent>()) {
        if (e == from) continue;
        const glm::vec2 c = RectCenter(reg.get<RectTransformComponent>(e));
        const glm::vec2 d = c - cur;

        float along = 0.0f, perp = 0.0f;
        switch (dir) {
            case Dir::Up:    along = -d.y; perp = glm::abs(d.x); break;
            case Dir::Down:  along =  d.y; perp = glm::abs(d.x); break;
            case Dir::Left:  along = -d.x; perp = glm::abs(d.y); break;
            case Dir::Right: along =  d.x; perp = glm::abs(d.y); break;
            default: return entt::null;
        }
        if (along <= 0.0f) continue;   // not in the requested direction

        const float score = along + 2.0f * perp;
        if (score < bestScore) {
            bestScore = score;
            best      = e;
        }
    }
    return best;
}

} // namespace

void UINavigationSystem::Update(entt::registry& reg, const UINavInput& input)
{
    using State = UIButtonComponent::State;

    UIFocusState& focus = Focus(reg);

    // Drop focus that points at a dead/renamed entity.
    if (focus.focused != entt::null &&
        (!reg.valid(focus.focused) || !reg.all_of<UIButtonComponent>(focus.focused)))
        focus.focused = entt::null;

    // The mouse is driving the UI: release gamepad focus and let UIInputSystem's
    // pointer pass own button state this frame (avoids the two fighting over it).
    if (input.mouseMoved) {
        focus.focused = entt::null;
        return;
    }

    // Directional move: grab the default button if nothing is focused, else step
    // to the nearest neighbor (ignore the step if there's nothing that way).
    if (input.move != Dir::None) {
        if (focus.focused == entt::null) {
            focus.focused = FirstButton(reg);
        } else if (entt::entity next = Neighbor(reg, focus.focused, input.move);
                   next != entt::null) {
            focus.focused = next;
        }
    }

    if (focus.focused == entt::null)
        return;

    auto& btn = reg.get<UIButtonComponent>(focus.focused);

    if (input.activate) {
        btn.state = State::Pressed;
        if (btn.onClick) btn.onClick();
    } else {
        // Highlight the focused button with the same look as mouse hover.
        btn.state = State::Hover;
    }
}

namespace UI {

UINavInput PollGamepadNav(int gamepadId)
{
    UINavInput out;
    if (!Input::IsGamepadConnected(gamepadId))
        return out;

    // Left stick contributes a direction only when pushed past the threshold, and
    // must return near center before it can step again — so a held stick advances
    // one button per flick instead of racing through the menu. The d-pad is
    // naturally edge-triggered via IsGamepadButtonPressed.
    static bool s_stickLatched = false;
    constexpr float kPush = 0.6f, kRelease = 0.3f;

    const float sx = Input::GetGamepadAxis(GamepadAxis::LeftX, gamepadId);
    const float sy = Input::GetGamepadAxis(GamepadAxis::LeftY, gamepadId);

    Dir stickDir = Dir::None;
    if (glm::max(glm::abs(sx), glm::abs(sy)) >= kPush) {
        if (glm::abs(sx) >= glm::abs(sy)) stickDir = sx > 0.0f ? Dir::Right : Dir::Left;
        else                              stickDir = sy > 0.0f ? Dir::Down  : Dir::Up;
    }
    if (glm::max(glm::abs(sx), glm::abs(sy)) < kRelease)
        s_stickLatched = false;

    Dir stickStep = Dir::None;
    if (stickDir != Dir::None && !s_stickLatched) {
        stickStep      = stickDir;
        s_stickLatched = true;
    }

    if      (Input::IsGamepadButtonPressed(GamepadButton::DpadUp,    gamepadId)) out.move = Dir::Up;
    else if (Input::IsGamepadButtonPressed(GamepadButton::DpadDown,  gamepadId)) out.move = Dir::Down;
    else if (Input::IsGamepadButtonPressed(GamepadButton::DpadLeft,  gamepadId)) out.move = Dir::Left;
    else if (Input::IsGamepadButtonPressed(GamepadButton::DpadRight, gamepadId)) out.move = Dir::Right;
    else                                                                          out.move = stickStep;

    out.activate = Input::IsGamepadButtonPressed(GamepadButton::South, gamepadId);
    return out;
}

} // namespace UI
