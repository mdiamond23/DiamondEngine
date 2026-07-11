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
#include "Renderer/ParticleRenderer.h"
#include "Audio/AudioTypes.h"
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
    // Eligible for the static shadow cache: when true, this mesh's shadow depth
    // is rendered into a light's cached map once and reused across frames instead
    // of re-drawn every frame (the win from unchanged geometry). Leave true for
    // level geometry; clear it for anything moved outside physics — e.g. a mesh
    // spun by a script — since the cache only re-renders when the light moves,
    // this caster's transform changes, or it's added/removed. A moving rigidbody
    // (Dynamic/Kinematic) is treated as dynamic automatically, regardless of this
    // flag; static rigidbodies (level colliders) stay cached.
    bool staticShadowCaster = true;
    // Forward alpha-blended instead of deferred-opaque: the mesh skips the
    // G-buffer and renders in the transparency pass (albedo map's alpha, sorted
    // back-to-front, depth-tested but not written).
    bool transparent    = false;

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

// Stamped on the root of an instantiated prefab — links the instance back to
// its source .prefab file. Scenes serialize such roots as references (path +
// root transform + override diff) and re-instantiate from the file on load, so
// instances always reflect the current asset.
struct PrefabInstanceComponent {
    std::string sourcePath;
    // Set when the source file was missing/unparsable at load: the entity is an
    // empty placeholder that keeps the reference (and its overrides, below)
    // alive so a re-save round-trips instead of silently dropping the instance.
    bool        broken = false;
    std::string pendingOverrides;   // raw override JSON held while broken
};

// Stamped on every entity created by prefab instantiation (root included):
// which entity in the .prefab file this one came from. Keys the override diff
// on scene save and the deterministic per-instance UUID derivation that keeps
// references into an instance's interior stable across reloads.
struct PrefabChildComponent {
    uint64_t prefabUuid = 0;
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

// ---- Particles -------------------------------------------------------------
// Where particles spawn and which way they launch. The shape supplies a local
// spawn-position offset and an emission direction; speedRange gives the velocity
// magnitude along that direction. Local space is oriented by the emitter's world
// rotation, so a tilted emitter emits sideways. Axis convention is local +Y.
enum class EmissionShape {
    Point,   // at the origin, straight up
    Cone,    // apex (+ optional base disc), within coneAngle of +Y
    Sphere,  // inside sphereRadius, radially outward
    Box      // inside boxHalfExtents, straight up
};

// Marks an entity as a particle emitter. ParticleSystem simulates the pool each
// frame during play; the engine's particle render pass draws the live particles
// as camera-facing billboards. Particles are spawned at the emitter's world
// position and then simulated in world space (they ignore later emitter motion).
struct ParticleEmitterComponent {
    // ---- Emission rate / cap ----
    float       spawnRate    = 40.0f;               // continuous particles per second
    std::size_t maxParticles = 512;                 // hard cap / pool size

    // ---- Shape: spawn position + emission direction ----
    EmissionShape shape          = EmissionShape::Cone;
    float         coneAngle      = 25.0f;           // Cone: half-angle off +Y, degrees
    float         coneRadius     = 0.0f;            // Cone: base disc radius at the apex
    float         sphereRadius   = 0.5f;            // Sphere: spawn radius
    glm::vec3     boxHalfExtents { 1.0f, 0.1f, 1.0f }; // Box: half-size of the volume
    glm::vec2     speedRange     { 2.5f, 4.5f };    // initial speed along the emission dir

    // ---- Bursts: spawn a clump at once, independent of spawnRate ----
    std::size_t burstCount    = 0;     // particles per burst (0 = continuous only)
    float       burstInterval = 0.0f;  // seconds between bursts (<=0 = one shot at start)

    // ---- Per-particle randomized ranges ----
    glm::vec2 lifetimeRange { 1.0f, 2.0f };         // seconds
    glm::vec2 sizeRange     { 0.15f, 0.30f };       // start size
    glm::vec3 gravity       { 0.0f, -3.0f, 0.0f };

    // ---- Over-lifetime ----
    glm::vec4 startColor { 1.0f, 0.75f, 0.25f, 1.0f };
    glm::vec4 endColor   { 1.0f, 0.15f, 0.0f,  0.0f };
    float     endSize    = 0.0f;                    // size lerps startSize -> endSize

    // ---- Render ----
    std::shared_ptr<Diamond::Texture> texture;      // null -> renderer's default soft-dot
    std::string texturePath;                        // asset path; round-trips `texture`
    Diamond::ParticleBlend blend = Diamond::ParticleBlend::Additive;

    // ---- Runtime pool: transient, NOT serialized and NOT shown in the inspector.
    // Live particles occupy pool[0, liveCount); dead slots are recycled via
    // swap-remove so the live span stays contiguous and ready to upload. ----
    struct Particle {
        glm::vec3 pos { 0.0f }, vel { 0.0f };
        glm::vec4 color { 1.0f };
        float size = 1.0f, startSize = 1.0f, age = 0.0f, lifetime = 1.0f, rotation = 0.0f;
    };
    std::vector<Particle> pool;
    std::size_t liveCount        = 0;
    float       spawnAccumulator = 0.0f;   // fractional carry for sub-frame spawn rates
    float       burstTimer       = 0.0f;   // counts down to the next burst
};

// ---- Audio -----------------------------------------------------------------
// Marks the entity whose world transform drives the single 3D audio listener
// (typically the camera). During play, AudioSystem syncs its position/forward/up
// to Audio::SetListener; in edit mode the editor camera drives the listener. Only
// one listener is honored — if several exist, the first enabled one wins.
struct AudioListenerComponent {
    bool enabled = true;
};

// Plays a clip from the entity's world position. AudioSystem owns the live voice:
// it starts the clip when play begins (if playOnStart), follows the transform while
// `is3D`, and pushes volume/pitch edits to the running sound each frame. The spatial
// fields map straight onto miniaudio's spatializer. Registered with the
// ComponentRegistry for free serialization + inspector (same path as particles).
struct AudioSourceComponent {
    std::string clipPath;                 // asset path; loaded to a ClipHandle on play
    Audio::Bus  bus         = Audio::Bus::SFX;
    float       volume      = 1.0f;
    float       pitch       = 1.0f;
    bool        loop        = false;
    bool        playOnStart = true;
    bool        stream      = false;      // long tracks: decode from disk on the fly

    // ---- 3D spatialization (ignored when is3D is false) ----
    bool                    is3D           = true;
    Audio::AttenuationModel attenuation    = Audio::AttenuationModel::Inverse;
    float                   minDistance    = 1.0f;    // full volume within this radius
    float                   maxDistance    = 100.0f;
    float                   rolloff        = 1.0f;
    float                   coneInnerAngle = 360.0f;  // degrees; 360 = omnidirectional
    float                   coneOuterAngle = 360.0f;  // degrees
    float                   coneOuterGain  = 0.0f;    // gain beyond the outer cone

    // ---- runtime: transient, NOT serialized and NOT shown in the inspector ----
    Audio::VoiceHandle _voice   = Audio::kInvalidVoice; // active voice, or kInvalidVoice
    bool               _started = false;                // playOnStart has fired
};
