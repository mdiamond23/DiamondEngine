#include "Scene/UIInputSystem.h"
#include "Scene/Components.h"
#include "Scene/Scene.h"

void UIInputSystem::Update(entt::registry& reg, const glm::vec2& pointer, bool pointerDown)
{
    using State = UIButtonComponent::State;

    // One OS pointer: remember last frame's held state so we only latch a press on
    // the down-edge (mouse pressed *while* over the button), not when a held mouse
    // is dragged over it.
    static bool s_prevDown = false;
    const bool pressEdge = pointerDown && !s_prevDown;

    for (auto e : reg.view<UIButtonComponent, RectTransformComponent>()) {
        auto&       btn = reg.get<UIButtonComponent>(e);
        const auto& rt  = reg.get<RectTransformComponent>(e);

        const bool inside =
            pointer.x >= rt.resolvedPos.x && pointer.x < rt.resolvedPos.x + rt.resolvedSize.x &&
            pointer.y >= rt.resolvedPos.y && pointer.y < rt.resolvedPos.y + rt.resolvedSize.y;

        if (btn.state == State::Pressed) {
            // A press is latched on this button; hold it until release.
            if (!pointerDown) {
                if (inside && btn.onClick) btn.onClick();   // released inside => click
                btn.state = inside ? State::Hover : State::Normal;
            }
        } else {
            if (inside && pressEdge) btn.state = State::Pressed;
            else                     btn.state = inside ? State::Hover : State::Normal;
        }
    }

    s_prevDown = pointerDown;
}

namespace UI {

bool SetButtonCallback(entt::registry& reg, entt::entity e, std::function<void()> onClick)
{
    if (e == entt::null || !reg.valid(e) || !reg.all_of<UIButtonComponent>(e))
        return false;
    reg.get<UIButtonComponent>(e).onClick = std::move(onClick);
    return true;
}

bool SetButtonCallback(Scene& scene, std::string_view name, std::function<void()> onClick)
{
    for (const auto& [entity, entName] : scene.GetEntityNames())
        if (entName == name)
            return SetButtonCallback(scene.GetRegistry(), entity, std::move(onClick));
    return false;
}

} // namespace UI
