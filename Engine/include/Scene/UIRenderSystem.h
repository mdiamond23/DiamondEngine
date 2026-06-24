#pragma once
#include <entt/entt.hpp>
#include <glm/glm.hpp>
#include <vector>

namespace Diamond { class Renderer2D; }

// Paints every canvas's widget components into their resolved RectTransform rects.
//
// Runs *after* UISystem::Resolve has filled in resolvedPos/resolvedSize for the
// frame. Canvases are drawn in CanvasComponent::sortOrder; within a canvas,
// widgets are drawn in RectTransformComponent::zOrder (stable, so hierarchy
// pre-order breaks ties). The whole pass is wrapped in one Renderer2D batch.
class UIRenderSystem
{
public:
    // screenSize = pixel dimensions of the render target the UI is drawn into.
    static void Render(entt::registry& reg, Diamond::Renderer2D& renderer,
                       const glm::vec2& screenSize);

private:
    // Pre-order collect of a canvas subtree's widget entities (those carrying a
    // RectTransformComponent), parents before children.
    static void CollectWidgets(entt::registry& reg, entt::entity root,
                               std::vector<entt::entity>& out);

    // Draw one entity's widget components into its resolved rect, back-to-front.
    static void DrawWidget(entt::registry& reg, Diamond::Renderer2D& renderer,
                           entt::entity e);
};
