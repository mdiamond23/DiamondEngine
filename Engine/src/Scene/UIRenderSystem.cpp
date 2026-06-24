#include "Scene/UIRenderSystem.h"
#include "Scene/Components.h"
#include "Renderer/Renderer2D.h"
#include "Renderer/TextureData.h"
#include "Renderer/Font.h"

#include <algorithm>
#include <cmath>
#include <string>
#include <vector>
#include <glm/common.hpp>

namespace {

// Break a text component's string into the lines that will actually be drawn:
// split on explicit '\n', then (if wrapping) greedy word-wrap each piece to
// maxWidth pixels. A word wider than maxWidth is left whole (no char-splitting).
std::vector<std::string> LayoutLines(const UITextComponent& t, const Diamond::Font& font,
                                     float scale, float maxWidth)
{
    std::vector<std::string> lines;

    auto emitParagraph = [&](const std::string& para) {
        if (!t.wrap || maxWidth <= 0.0f) { lines.push_back(para); return; }
        if (para.empty())               { lines.push_back("");   return; }

        std::string line;
        size_t i = 0;
        while (i < para.size()) {
            const size_t wordStart = i;
            while (i < para.size() && para[i] != ' ') ++i;
            std::string word = para.substr(wordStart, i - wordStart);
            while (i < para.size() && para[i] == ' ') ++i;   // collapse run of spaces
            if (word.empty()) continue;

            if (line.empty()) {
                line = word;
            } else {
                const std::string candidate = line + ' ' + word;
                if (font.Measure(candidate).x * scale > maxWidth) {
                    lines.push_back(line);
                    line = word;
                } else {
                    line = candidate;
                }
            }
        }
        lines.push_back(line);
    };

    std::string para;
    for (char c : t.text) {
        if (c == '\n') { emitParagraph(para); para.clear(); }
        else           { para.push_back(c); }
    }
    emitParagraph(para);
    return lines;
}

void DrawText(const RectTransformComponent& rt, const UITextComponent& t,
              Diamond::Renderer2D& renderer)
{
    if (!t.font || t.text.empty() || t.font->PixelHeight() <= 0.0f)
        return;

    const Diamond::Font& font = *t.font;
    const float scale       = t.sizePx / font.PixelHeight();
    const float lineAdvance = font.LineHeight() * scale * t.lineSpacing;
    const float ascent      = font.Ascent() * scale;

    const std::vector<std::string> lines = LayoutLines(t, font, scale, rt.resolvedSize.x);

    // Vertical placement of the whole block within the rect.
    const float blockH = static_cast<float>(lines.size()) * lineAdvance;
    float startY = rt.resolvedPos.y;
    if      (t.vAlign == UITextComponent::VAlign::Middle) startY += (rt.resolvedSize.y - blockH) * 0.5f;
    else if (t.vAlign == UITextComponent::VAlign::Bottom) startY += (rt.resolvedSize.y - blockH);

    const float ulThickness = std::max(1.0f, t.sizePx * 0.06f);
    const float ulOffset    = font.Descent() * scale * -0.4f;   // a little below the baseline

    for (size_t i = 0; i < lines.size(); ++i) {
        const std::string& line = lines[i];
        const float lineW = font.Measure(line).x * scale;

        float x = rt.resolvedPos.x;
        if      (t.hAlign == UITextComponent::HAlign::Center) x += (rt.resolvedSize.x - lineW) * 0.5f;
        else if (t.hAlign == UITextComponent::HAlign::Right)  x += (rt.resolvedSize.x - lineW);

        const float lineTopY = startY + static_cast<float>(i) * lineAdvance;

        if (t.underline && !line.empty()) {
            const float baselineY = lineTopY + ascent;
            renderer.DrawQuad({ x, baselineY + ulOffset }, { lineW, ulThickness }, t.color);
        }
        renderer.DrawText(font, line, { x, lineTopY }, t.color, scale);
    }
}

void DrawProgressBar(const RectTransformComponent& rt, const UIProgressBarComponent& bar,
                     Diamond::Renderer2D& renderer)
{
    const glm::vec2 pos  = rt.resolvedPos;
    const glm::vec2 size = rt.resolvedSize;

    renderer.DrawQuad(pos, size, bar.backgroundColor);   // track spans the whole rect

    const float p = glm::clamp(bar.progress, 0.0f, 1.0f);
    if (p <= 0.0f) return;

    // Size + offset the fill along `direction`; a textured fill clips a matching
    // UV sub-rect (top-left origin, +Y down) so it reveals instead of squashing.
    glm::vec2 fillPos = pos, fillSize = size, uvMin(0.0f), uvMax(1.0f);
    switch (bar.direction) {
        case UIProgressBarComponent::Direction::LeftToRight:
            fillSize.x = size.x * p; uvMax.x = p; break;
        case UIProgressBarComponent::Direction::RightToLeft:
            fillSize.x = size.x * p; fillPos.x = pos.x + size.x * (1.0f - p); uvMin.x = 1.0f - p; break;
        case UIProgressBarComponent::Direction::BottomToTop:
            fillSize.y = size.y * p; fillPos.y = pos.y + size.y * (1.0f - p); uvMin.y = 1.0f - p; break;
        case UIProgressBarComponent::Direction::TopToBottom:
            fillSize.y = size.y * p; uvMax.y = p; break;
    }

    if (bar.fillTexture)
        renderer.DrawTexturedQuad(fillPos, fillSize, *bar.fillTexture, bar.fillColor, uvMin, uvMax);
    else
        renderer.DrawQuad(fillPos, fillSize, bar.fillColor);
}

} // namespace

void UIRenderSystem::CollectWidgets(entt::registry& reg, entt::entity root,
                                    std::vector<entt::entity>& out)
{
    if (!reg.all_of<HierarchyComponent>(root))
        return;

    for (entt::entity child : reg.get<HierarchyComponent>(root).children) {
        if (!reg.valid(child) || !reg.all_of<RectTransformComponent>(child))
            continue;
        out.push_back(child);
        CollectWidgets(reg, child, out);   // depth-first, parents before children
    }
}

void UIRenderSystem::DrawWidget(entt::registry& reg, Diamond::Renderer2D& renderer,
                                entt::entity e)
{
    const auto& rt = reg.get<RectTransformComponent>(e);

    // Image draws first so it sits behind any text/label on the same entity. On a
    // button, the image's tint is modulated by the current interaction state.
    if (const auto* img = reg.try_get<UIImageComponent>(e)) {
        glm::vec4 tint = img->tint;
        if (const auto* btn = reg.try_get<UIButtonComponent>(e))
            tint *= btn->TintForState();

        if (img->texture)
            renderer.DrawTexturedQuad(rt.resolvedPos, rt.resolvedSize, *img->texture,
                                      tint, img->uvMin, img->uvMax);
        else
            renderer.DrawQuad(rt.resolvedPos, rt.resolvedSize, tint);
    }

    // Progress fill sits over the image, under any label.
    if (const auto* bar = reg.try_get<UIProgressBarComponent>(e))
        DrawProgressBar(rt, *bar, renderer);

    // Text (and any label on a composed widget like a button) draws on top.
    if (const auto* txt = reg.try_get<UITextComponent>(e))
        DrawText(rt, *txt, renderer);
}

void UIRenderSystem::Render(entt::registry& reg, Diamond::Renderer2D& renderer,
                            const glm::vec2& screenSize)
{
    // Canvases drawn back-to-front by sortOrder.
    std::vector<entt::entity> canvases;
    for (entt::entity e : reg.view<CanvasComponent>())
        canvases.push_back(e);
    std::stable_sort(canvases.begin(), canvases.end(),
                     [&](entt::entity a, entt::entity b) {
                         return reg.get<CanvasComponent>(a).sortOrder <
                                reg.get<CanvasComponent>(b).sortOrder;
                     });

    renderer.Begin(Diamond::Renderer2D::OrthoProjection(screenSize.x, screenSize.y));

    std::vector<entt::entity> widgets;
    for (entt::entity canvas : canvases) {
        widgets.clear();
        CollectWidgets(reg, canvas, widgets);

        // zOrder over the flattened subtree; stable keeps pre-order on ties.
        std::stable_sort(widgets.begin(), widgets.end(),
                         [&](entt::entity a, entt::entity b) {
                             return reg.get<RectTransformComponent>(a).zOrder <
                                    reg.get<RectTransformComponent>(b).zOrder;
                         });

        for (entt::entity w : widgets)
            DrawWidget(reg, renderer, w);
    }

    renderer.End();
}
