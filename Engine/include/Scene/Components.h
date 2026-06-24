#pragma once
#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <entt/entt.hpp>
#include <memory>
#include <string>
#include <vector>
#include "Renderer/MeshData.h"
#include "Renderer/Material.h"
#include "Renderer/Frustum.h"
#include <cstdint>

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
