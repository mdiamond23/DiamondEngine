#pragma once
#include <entt/entt.hpp>
#include <glm/glm.hpp>
#include <functional>
#include <string_view>

class Scene;

// Pointer interaction for UI buttons. Runs after UISystem::Resolve (it reads
// resolvedPos/resolvedSize) and before UIRenderSystem::Render (which draws the
// button state it writes).
//
// `pointer` is in the same screen-pixel space as the resolved rects; pass a
// position outside every rect (e.g. {-1,-1}) when the pointer isn't over the UI.
// `pointerDown` is the primary button's held state this frame. A click fires when
// a press that began on a button releases while still over it. The press edge is
// detected against the previous call, so call this once per frame for one pointer.
class UIInputSystem
{
public:
    static void Update(entt::registry& reg, const glm::vec2& pointer, bool pointerDown);
};

namespace UI {

// Assign a button's click handler from script code (e.g. a GameSystem::OnStart).
// onClick is intentionally not serialized — a scene stores what a button looks
// like; what it *does* is behavior and lives in scripts. Returns false if the
// target has no UIButtonComponent (or the name isn't found).
bool SetButtonCallback(entt::registry& reg, entt::entity e, std::function<void()> onClick);
bool SetButtonCallback(Scene& scene, std::string_view name, std::function<void()> onClick);

} // namespace UI
