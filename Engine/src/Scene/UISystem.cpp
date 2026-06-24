#include "Scene/UISystem.h"
#include "Scene/Components.h"

#include <cmath>

float UISystem::ScaleFactor(const CanvasComponent& canvas, const glm::vec2& screen)
{
    if (canvas.scaleMode == CanvasComponent::ScaleMode::ConstantPixelSize)
        return 1.0f;

    const glm::vec2 ref = canvas.referenceResolution;
    if (ref.x <= 0.0f || ref.y <= 0.0f || screen.x <= 0.0f || screen.y <= 0.0f)
        return 1.0f;

    // Unity CanvasScaler-style log2 blend between the width-fit and height-fit
    // ratios, so the UI scales smoothly with the window's aspect ratio.
    const float logW    = std::log2(screen.x / ref.x);
    const float logH    = std::log2(screen.y / ref.y);
    const float match   = glm::clamp(canvas.matchWidthHeight, 0.0f, 1.0f);
    const float blended = glm::mix(logW, logH, match);
    return std::exp2(blended);
}

void UISystem::ResolveElement(entt::registry& reg, entt::entity e,
                              const glm::vec2& parentPos, const glm::vec2& parentSize,
                              float scale)
{
    auto& rt = reg.get<RectTransformComponent>(e);

    glm::vec2 pos(0.0f), size(0.0f);
    for (int ax = 0; ax < 2; ++ax) {
        // Anchor reference edges inside the parent rect (px).
        const float aMin = parentPos[ax] + rt.anchorMin[ax] * parentSize[ax];
        const float aMax = parentPos[ax] + rt.anchorMax[ax] * parentSize[ax];

        // Canvas-unit values become pixels here.
        const float sizeDelta = rt.size[ax]     * scale;  // added to the anchor gap
        const float anchored  = rt.position[ax] * scale;  // offset from the anchor

        // Convert (anchoredPosition, sizeDelta, pivot) -> edge offsets, so the
        // pivot is honoured for both point anchors and stretched anchors.
        const float offMin = anchored - sizeDelta * rt.pivot[ax];
        const float offMax = anchored + sizeDelta * (1.0f - rt.pivot[ax]);

        const float rMin = aMin + offMin;
        const float rMax = aMax + offMax;

        pos[ax]  = rMin;
        size[ax] = rMax - rMin;
    }

    rt.resolvedPos  = pos;
    rt.resolvedSize = size;

    // Children resolve against this element's freshly-computed rect.
    if (reg.all_of<HierarchyComponent>(e)) {
        for (entt::entity child : reg.get<HierarchyComponent>(e).children)
            if (reg.valid(child) && reg.all_of<RectTransformComponent>(child))
                ResolveElement(reg, child, pos, size, scale);
    }
}

void UISystem::Resolve(entt::registry& reg, const glm::vec2& screenSize)
{
    for (auto entity : reg.view<CanvasComponent>()) {
        const auto& canvas = reg.get<CanvasComponent>(entity);
        const float scale  = ScaleFactor(canvas, screenSize);

        // The canvas rect spans the whole render target; its children resolve
        // against (0,0)..(screenSize).
        if (reg.all_of<HierarchyComponent>(entity)) {
            for (entt::entity child : reg.get<HierarchyComponent>(entity).children)
                if (reg.valid(child) && reg.all_of<RectTransformComponent>(child))
                    ResolveElement(reg, child, glm::vec2(0.0f), screenSize, scale);
        }
    }
}
