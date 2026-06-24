#pragma once
#include "Scene/Scripting.h"
#include "Scene/Scene.h"
#include "Scene/ComponentRegistry.h"
#include "Scene/Components.h"
#include "Scene/Physics/PhysicsAPI.h"
#include <imgui.h>
#include <nlohmann/json.hpp>
#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <spdlog/spdlog.h>
#include <cmath>

// ---- Data -------------------------------------------------------------------
// Tag a player (or anything that should stay in frame) with CameraTarget. The
// director frames the centroid of every entity wearing it and pulls back as they
// spread apart. See Docs/camera-design.md.

struct CameraTargetComponent {};   // pure tag — no fields

// Attach to the primary camera entity. Holds the fixed framing angle plus the
// distance / smoothing / collision knobs. Orientation is fixed (no orbit): only
// the pivot and distance move, so framing is a spread projected onto two screen
// axes and collision is a single fixed-direction sweep.
struct CameraDirectorComponent
{
    // Fixed orientation (degrees). yaw 0 / pitch 0 looks down −Z; positive pitch
    // tilts the view downward onto the action.
    float pitchDeg = 25.0f;
    float yawDeg   = 0.0f;

    // Frustum-fit distance is clamped to this range, then scaled by margin so the
    // targets don't sit right at the screen edge.
    float minDistance   = 6.0f;
    float maxDistance   = 30.0f;
    float framingMargin = 1.3f;

    // Lift the look point above the centroid so heads aren't at screen center.
    float pivotHeightOffset = 1.0f;

    // Horizontal fov is derived from the camera's vertical fov and this aspect, so
    // the system never needs the live viewport size. Set to your window's aspect.
    float aspect = 16.0f / 9.0f;

    // Exponential ease rates (1 - exp(-s·dt)); higher = snappier.
    float smoothing         = 6.0f;   // pivot + distance follow
    float recoverSmoothing  = 3.0f;   // collision pull-OUT (snap-in is instant)

    // Distance hysteresis: ignore re-targeting smaller than this so the camera
    // doesn't breathe in/out on tiny player movement.
    float distanceDeadzone = 0.25f;

    // Collision avoidance.
    bool  collisionEnabled = true;
    float collisionRadius  = 0.3f;   // ~near-plane half-extent
    float collisionSkin    = 0.2f;   // pull in this far short of the hit
    float minClearance     = 0.5f;   // never seat closer than this to the pivot

    // ---- Runtime state (not serialized) ----
    bool      initialized = false;
    glm::vec3 curPivot { 0.0f };
    float     curDistance = 0.0f;
    float     seatDistance = 0.0f;   // post-collision distance, asymmetrically eased
};

// ---- Inspector UI -----------------------------------------------------------

template<>
inline void DrawComponentInspector<CameraTargetComponent>(CameraTargetComponent&)
{
    ImGui::TextDisabled("Framed by the Camera Director.");
}

template<>
inline void DrawComponentInspector<CameraDirectorComponent>(CameraDirectorComponent& c)
{
    ImGui::SeparatorText("Framing");
    ImGui::DragFloat("Pitch (deg)",   &c.pitchDeg, 0.5f, -89.0f, 89.0f);
    ImGui::DragFloat("Yaw (deg)",     &c.yawDeg,   0.5f, -180.0f, 180.0f);
    ImGui::DragFloat("Min Distance",  &c.minDistance, 0.1f, 0.1f, 1000.0f);
    ImGui::DragFloat("Max Distance",  &c.maxDistance, 0.1f, 0.1f, 1000.0f);
    ImGui::DragFloat("Framing Margin",&c.framingMargin, 0.01f, 1.0f, 3.0f);
    ImGui::DragFloat("Pivot Height",  &c.pivotHeightOffset, 0.05f, -10.0f, 10.0f);
    ImGui::DragFloat("Aspect",        &c.aspect, 0.01f, 0.1f, 4.0f);

    ImGui::SeparatorText("Smoothing");
    ImGui::DragFloat("Follow",        &c.smoothing, 0.1f, 0.1f, 30.0f);
    ImGui::DragFloat("Recover",       &c.recoverSmoothing, 0.1f, 0.1f, 30.0f);
    ImGui::DragFloat("Dist Deadzone", &c.distanceDeadzone, 0.01f, 0.0f, 5.0f);

    ImGui::SeparatorText("Collision");
    ImGui::Checkbox("Enabled",        &c.collisionEnabled);
    ImGui::DragFloat("Probe Radius",  &c.collisionRadius, 0.01f, 0.0f, 5.0f);
    ImGui::DragFloat("Skin",          &c.collisionSkin, 0.01f, 0.0f, 5.0f);
    ImGui::DragFloat("Min Clearance", &c.minClearance, 0.05f, 0.1f, 10.0f);
}

// ---- Serialization ----------------------------------------------------------
// CameraTargetComponent is a tag — default "{}" serialization is fine.

template<>
inline std::string SerializeComponent<CameraDirectorComponent>(const CameraDirectorComponent& c)
{
    nlohmann::json j;
    j["pitchDeg"]          = c.pitchDeg;
    j["yawDeg"]            = c.yawDeg;
    j["minDistance"]       = c.minDistance;
    j["maxDistance"]       = c.maxDistance;
    j["framingMargin"]     = c.framingMargin;
    j["pivotHeightOffset"] = c.pivotHeightOffset;
    j["aspect"]            = c.aspect;
    j["smoothing"]         = c.smoothing;
    j["recoverSmoothing"]  = c.recoverSmoothing;
    j["distanceDeadzone"]  = c.distanceDeadzone;
    j["collisionEnabled"]  = c.collisionEnabled;
    j["collisionRadius"]   = c.collisionRadius;
    j["collisionSkin"]     = c.collisionSkin;
    j["minClearance"]      = c.minClearance;
    return j.dump();
}

template<>
inline void DeserializeComponent<CameraDirectorComponent>(CameraDirectorComponent& c, const std::string& data)
{
    auto j = nlohmann::json::parse(data);
    c.pitchDeg          = j.value("pitchDeg",          25.0f);
    c.yawDeg            = j.value("yawDeg",            0.0f);
    c.minDistance       = j.value("minDistance",       6.0f);
    c.maxDistance       = j.value("maxDistance",       30.0f);
    c.framingMargin     = j.value("framingMargin",     1.3f);
    c.pivotHeightOffset = j.value("pivotHeightOffset", 1.0f);
    c.aspect            = j.value("aspect",            16.0f / 9.0f);
    c.smoothing         = j.value("smoothing",         6.0f);
    c.recoverSmoothing  = j.value("recoverSmoothing",  3.0f);
    c.distanceDeadzone  = j.value("distanceDeadzone",  0.25f);
    c.collisionEnabled  = j.value("collisionEnabled",  true);
    c.collisionRadius   = j.value("collisionRadius",   0.3f);
    c.collisionSkin     = j.value("collisionSkin",     0.2f);
    c.minClearance      = j.value("minClearance",      0.5f);
}

// ---- Registration -----------------------------------------------------------

DECLARE_COMPONENT(CameraTargetComponent,   "CameraTarget")
DECLARE_COMPONENT(CameraDirectorComponent, "CameraDirector")

// ---- Behavior ---------------------------------------------------------------

class CameraDirectorSystem : public GameSystem
{
    DECLARE_SYSTEM(CameraDirectorSystem, 450)   // camera band: after physics + gameplay
public:
    void OnUpdate(Scene& scene, float dt) override
    {
        // Gather target positions once per frame (shared by every director).
        glm::vec3 centroid { 0.0f };
        glm::vec3 targets[kMaxTargets];
        int       count = 0;
        for (entt::entity e : scene.View<CameraTargetComponent>())
        {
            if (!scene.Has<TransformComponent>(e)) continue;
            glm::vec3 p = glm::vec3(scene.GetTransformSystem().GetWorldMatrix(e)[3]);
            centroid += p;
            if (count < kMaxTargets) targets[count++] = p;
        }
        if (count == 0) return;
        centroid /= static_cast<float>(count);

        for (auto [cam, dir] : scene.View<CameraDirectorComponent>().each())
        {
            if (!scene.Has<TransformComponent>(cam)) continue;

            // --- Fixed camera basis from authored angle ---
            const float yr = glm::radians(dir.yawDeg);
            const float pr = glm::radians(dir.pitchDeg);
            const glm::vec3 worldUp { 0.0f, 1.0f, 0.0f };
            glm::vec3 front {
                std::cos(pr) * std::sin(yr),
                -std::sin(pr),
                -std::cos(pr) * std::cos(yr)
            };
            front = glm::normalize(front);
            glm::vec3 right = glm::normalize(glm::cross(front, worldUp));
            glm::vec3 up    = glm::normalize(glm::cross(right, front));

            glm::vec3 pivot = centroid + worldUp * dir.pivotHeightOffset;

            // --- Frustum-fit distance from the targets' spread ---
            float fovDeg = scene.Has<CameraComponent>(cam)
                         ? scene.Get<CameraComponent>(cam).fov : 60.0f;
            float vHalf  = glm::radians(fovDeg * 0.5f);
            float hHalf  = std::atan(dir.aspect * std::tan(vHalf));

            float halfW = 0.0f, halfH = 0.0f;
            for (int i = 0; i < count; ++i)
            {
                glm::vec3 v = targets[i] - pivot;
                halfW = glm::max(halfW, std::fabs(glm::dot(v, right)));
                halfH = glm::max(halfH, std::fabs(glm::dot(v, up)));
            }
            float distW = halfW / std::tan(hHalf);
            float distH = halfH / std::tan(vHalf);
            float wantDist = glm::max(distW, distH) * dir.framingMargin;
            wantDist = glm::clamp(wantDist, dir.minDistance, dir.maxDistance);

            // --- Ease pivot + distance (with a distance deadzone) ---
            if (!dir.initialized)
            {
                dir.curPivot     = pivot;
                dir.curDistance  = wantDist;
                dir.seatDistance = wantDist;
                dir.initialized  = true;
            }

            float t = 1.0f - std::exp(-dir.smoothing * dt);
            dir.curPivot += (pivot - dir.curPivot) * t;
            if (std::fabs(wantDist - dir.curDistance) > dir.distanceDeadzone)
                dir.curDistance += (wantDist - dir.curDistance) * t;

            // --- Collision: sweep pivot -> seat, snap in / ease out ---
            float seat = dir.curDistance;
            if (dir.collisionEnabled)
            {
                HitResult h = Physics::SphereCast(dir.curPivot, dir.collisionRadius,
                                                  -front, dir.curDistance, cam,
                                                  /*drawDebug=*/true);
                // Diagnostic: same direction as a thin ray. If the ray hits but the
                // sphere doesn't, it's the shape-cast initial-overlap quirk; if both
                // miss, the geometry has no registered physics body.
                HitResult hr = Physics::Raycast(dir.curPivot, -front, dir.curDistance, cam);
                if (h.hit)
                    seat = glm::max(h.distance - dir.collisionSkin, dir.minClearance);
            }
            if (seat < dir.seatDistance)
                dir.seatDistance = seat;                       // obstruction: snap in
            else
            {
                float tr = 1.0f - std::exp(-dir.recoverSmoothing * dt);
                dir.seatDistance += (seat - dir.seatDistance) * tr;  // clear: ease out
            }

            // --- Write the camera transform ---
            auto& xf = scene.Get<TransformComponent>(cam);
            xf.position = dir.curPivot - front * dir.seatDistance;
            // World rotation is the transpose of the view's rotation block.
            glm::mat3 viewR { glm::lookAt(xf.position, dir.curPivot, worldUp) };
            xf.rotation = glm::normalize(glm::quat_cast(glm::transpose(viewR)));
        }
    }

private:
    static constexpr int kMaxTargets = 8;
};
