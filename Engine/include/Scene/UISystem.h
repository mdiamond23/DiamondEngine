#pragma once
#include <entt/entt.hpp>
#include <glm/glm.hpp>

struct CanvasComponent;

// Resolves screen-space UI. Walks every CanvasComponent hierarchy top-down and
// writes RectTransformComponent::resolvedPos / resolvedSize in screen pixels.
// Stateless — call Resolve() once per frame before drawing the UI with Renderer2D.
class UISystem
{
public:
    // screenSize = pixel dimensions of the render target the UI is drawn into.
    static void Resolve(entt::registry& reg, const glm::vec2& screenSize);

private:
    // Canvas-units -> screen-pixels factor (1.0 for ConstantPixelSize).
    static float ScaleFactor(const CanvasComponent& canvas, const glm::vec2& screenSize);

    // Resolves one element against its parent's resolved rect, then recurses.
    static void ResolveElement(entt::registry& reg, entt::entity e,
                               const glm::vec2& parentPos, const glm::vec2& parentSize,
                               float scale);
};
