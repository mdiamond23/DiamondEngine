#pragma once
#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <entt/entt.hpp>
#include <memory>
#include <string>
#include <vector>
#include <functional>
#include "Renderer/MeshData.h"
#include "Renderer/Material.h"
#include "Renderer/Frustum.h"
#include <cstdint>

namespace Diamond { class Texture; class Font; }

enum class LightType {
    Sun,
    Point,
    Spot
};

struct TransformComponent
{
    glm::vec3 position     { 0, 0, 0 };
    glm::quat rotation     { 1, 0, 0, 0 };  // identity
    glm::vec3 eulerDegrees { 0, 0, 0 };     // editor display cache; kept in sync with rotation
    glm::vec3 scale        { 1, 1, 1 };

    inline glm::mat4 GetLocalMatrix() const
    {
        glm::mat4 t = glm::translate(glm::mat4(1.0f), position);
        glm::mat4 r = glm::mat4_cast(rotation);
        glm::mat4 s = glm::scale(glm::mat4(1.0f), scale);
        return t * r * s;
    }
};

struct MeshComponent
{
    std::shared_ptr<Diamond::Mesh>        mesh;
    std::shared_ptr<Diamond::PBRMaterial> material;
    Diamond::AABB                         localBounds;

    bool visible        = true;
    bool castsShadow    = true;
    bool receivesShadow = true;

    std::string meshPath;
    int         meshSubIndex = 0;
    std::string materialPath;
};

struct LightComponent
{
    LightType type           = LightType::Point;
    glm::vec3 color          = { 1.0f, 1.0f, 1.0f };
    float     intensity      = 100.0f;
    float     radius         = 10.0f;   // Point/Spot: max influence range
    float     innerConeAngle = 15.0f;   // Spot: inner cone, degrees
    float     outerConeAngle = 30.0f;   // Spot: outer cone, degrees
};

struct CameraComponent
{
    bool  isPrimary = true;
    float fov       = 60.0f;
    float nearClip  = 0.1f;
    float farClip   = 1000.0f;
};

struct IDComponent {
    uint64_t uuid;
};

// Scene hierarchy — parent/children stored as entity handles.
// Always mutate through Scene::SetParent / Scene::UnsetParent so the
// TransformSystem flat arrays stay consistent.
struct HierarchyComponent {
    entt::entity              parent   = entt::null;
    std::vector<entt::entity> children;
};

// ---- In-game UI (screen-space) ---------------------------------------------
// A CanvasComponent marks an entity as the root of a UI tree. Its rect is the
// full render target. Descendant entities carry RectTransformComponent and are
// resolved top-down by UISystem into screen-pixel rects, then drawn by Renderer2D.
struct CanvasComponent {
    // ConstantPixelSize  : 1 UI unit == 1 screen pixel, regardless of resolution.
    // ScaleWithScreenSize: UI authored at referenceResolution, then uniformly
    //                      scaled to fit the actual target (resolution independence).
    enum class ScaleMode { ConstantPixelSize, ScaleWithScreenSize };

    ScaleMode scaleMode           = ScaleMode::ScaleWithScreenSize;
    glm::vec2 referenceResolution { 1920.0f, 1080.0f };
    float     matchWidthHeight    = 0.5f;   // 0 = match width, 1 = match height
    int       sortOrder           = 0;      // draw order between canvases
};

// Anchor/pivot rect, Unity RectTransform-style. Resolved against the parent's
// resolved rect (another RectTransform, or the Canvas = full screen).
//
// Coordinate space: top-left origin, +X right, +Y down (matches Renderer2D).
// Anchors are fractions of the parent rect: (0,0) = top-left, (1,1) = bottom-right.
//   - anchorMin == anchorMax  -> "point" anchor: `size` is the literal box size.
//   - anchorMin != anchorMax  -> "stretch": the box follows the parent's edges;
//                                 `size` is a delta added to the anchor gap
//                                 (size 0 fills the gap exactly, negative insets it).
// `position` offsets the box from its anchor (anchoredPosition).
// `pivot` is the box's local reference point for position/scaling (the "Alignment").
struct RectTransformComponent {
    glm::vec2 anchorMin { 0.5f, 0.5f };
    glm::vec2 anchorMax { 0.5f, 0.5f };
    glm::vec2 pivot     { 0.5f, 0.5f };
    glm::vec2 position  { 0.0f,  0.0f  };   // anchoredPosition, in canvas units
    glm::vec2 size      { 100.0f, 100.0f }; // sizeDelta, in canvas units
    int       zOrder    = 0;

    // --- computed by UISystem each frame; not serialized ---
    glm::vec2 resolvedPos  { 0.0f };   // top-left, screen pixels
    glm::vec2 resolvedSize { 0.0f };   // screen pixels
};

// Paints a quad into the entity's resolved RectTransform rect. The simplest
// widget: a flat color, or a texture modulated by `tint`. Drawn by UIRenderSystem.
//   - texture == null -> solid quad filled with `tint` (the "white image" default).
//   - texture != null -> textured quad, with the texture multiplied by `tint`.
// `tint` is straight (non-premultiplied) RGBA: rgb is a color multiply, a is opacity.
struct UIImageComponent {
    std::shared_ptr<Diamond::Texture> texture;          // null = solid color
    std::string texturePath;                            // asset path; round-trips `texture`
    glm::vec4 tint  { 1.0f, 1.0f, 1.0f, 1.0f };
    glm::vec2 uvMin { 0.0f, 0.0f };                     // sub-rect of the texture
    glm::vec2 uvMax { 1.0f, 1.0f };
};

// UI text/fonts bake at this fixed pixel height; UITextComponent::sizePx then
// scales from it. A single bake height is a known crispness tradeoff (SDF fonts
// are the long-term fix). Shared so serialization and the inspector agree.
inline constexpr float kUIFontBakeHeight = 48.0f;

// Lays out and paints a string inside the entity's resolved RectTransform rect.
// Honors explicit '\n', optional word-wrap to the rect width, horizontal and
// vertical alignment, a line-spacing multiplier, and an optional underline.
// Drawn by UIRenderSystem. Bold/italic are intentionally unsupported (the Font
// is a single baked face) — use a bold/italic font file if needed.
struct UITextComponent {
    enum class HAlign { Left, Center, Right };
    enum class VAlign { Top, Middle, Bottom };

    std::string                    text;
    std::shared_ptr<Diamond::Font> font;
    std::string                    fontPath;   // asset path; round-trips `font`
    float     sizePx      = 24.0f;   // target pixel height; scaled from the font's baked height
    glm::vec4 color       { 1.0f, 1.0f, 1.0f, 1.0f };
    HAlign    hAlign      = HAlign::Left;
    VAlign    vAlign      = VAlign::Top;
    float     lineSpacing = 1.0f;    // multiplier on the font's line height
    bool      wrap        = true;    // word-wrap to the rect width
    bool      underline   = false;
};

// A linear fill bar: a background quad, then a fill quad covering `progress`
// (0..1) of the rect along `direction`. An optional fill texture is UV-clipped to
// the filled fraction so it reveals rather than squashes. Two quads, no shader.
struct UIProgressBarComponent {
    enum class Direction { LeftToRight, RightToLeft, BottomToTop, TopToBottom };

    float     progress        = 0.5f;                       // clamped to 0..1 when drawn
    glm::vec4 backgroundColor { 0.10f, 0.10f, 0.12f, 1.0f };
    glm::vec4 fillColor       { 0.25f, 0.80f, 0.40f, 1.0f };
    std::shared_ptr<Diamond::Texture> fillTexture;          // optional; modulated by fillColor
    std::string fillTexturePath;                            // asset path; round-trips `fillTexture`
    Direction direction       = Direction::LeftToRight;
};

// Interaction-only widget. Buttons are composed: this rides on the same entity as
// a UIImageComponent (background) and usually a UITextComponent (label), and draws
// nothing itself. UIInputSystem hit-tests the pointer against the entity's
// resolved rect and writes `state`; UIRenderSystem multiplies the sibling image's
// tint by the matching state tint. onClick fires on a press that began on the
// button and released while still over it (a completed click).
struct UIButtonComponent {
    enum class State { Normal, Hover, Pressed };

    glm::vec4 normalTint  { 1.00f, 1.00f, 1.00f, 1.0f };
    glm::vec4 hoverTint   { 1.20f, 1.20f, 1.20f, 1.0f };
    glm::vec4 pressedTint  { 0.80f, 0.80f, 0.80f, 1.0f };
    std::function<void()> onClick;

    // --- runtime, written by UIInputSystem; not serialized ---
    State state = State::Normal;

    const glm::vec4& TintForState() const {
        return state == State::Pressed ? pressedTint
             : state == State::Hover   ? hoverTint
             :                           normalTint;
    }
};
