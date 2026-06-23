#pragma once
#include "Scene/Scripting.h"
#include "Scene/Scene.h"
#include "Scene/ComponentRegistry.h"
#include "Core/Input.h"
#include "Animation/IKComponent.h"
#include "Animation/AnimationComponents.h"
#include <imgui.h>
#include <nlohmann/json.hpp>
#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>
#include <string>

// ---- Data -------------------------------------------------------------------
// A dead-simple IK test. Put this on a skinned character that also has an
// IKComponent with a left-arm chain (end effector = the hand bone). Pull the
// LEFT TRIGGER to throw a punch: the hand snaps a target out in the LEFT STICK
// direction and the arm extends toward it (the chain's easeTime smooths the
// reach in). Hold the trigger to hold the pose extended; release to retract
// (weight eases back to 0 and the animated pose takes over again).

struct PunchComponent
{
    std::string handBone = "Skeleton_arm_joint_L__2_"; // IK chain to drive (matched by endEffectorBone)
    float reach          = 0.8f;   // how far in front to plant the target (solver clamps to full extension)
    float shoulderHeight = 1.3f;   // target anchor height above the entity origin (model units)

    // --- runtime (not serialized) ---
    bool      _wasHeld = false;     // trigger state last frame (rising-edge latch)
    glm::vec3 _dir     { 0, 0, -1 };// latched punch direction (world), captured on press
};

// ---- Inspector UI -----------------------------------------------------------

template<>
inline void DrawComponentInspector<PunchComponent>(PunchComponent& c)
{
    char buf[128];
    std::snprintf(buf, sizeof(buf), "%s", c.handBone.c_str());
    if (ImGui::InputText("Hand Bone", buf, sizeof(buf)))
        c.handBone = buf;
    ImGui::DragFloat("Reach",           &c.reach,          0.01f, 0.0f, 5.0f);
    ImGui::DragFloat("Shoulder Height", &c.shoulderHeight, 0.01f, 0.0f, 3.0f);
    ImGui::TextDisabled("Left trigger = punch toward left stick (play mode)");
}

// ---- Serialization ----------------------------------------------------------

template<>
inline std::string SerializeComponent<PunchComponent>(const PunchComponent& c)
{
    nlohmann::json j;
    j["handBone"]       = c.handBone;
    j["reach"]          = c.reach;
    j["shoulderHeight"] = c.shoulderHeight;
    return j.dump();
}

template<>
inline void DeserializeComponent<PunchComponent>(PunchComponent& c, const std::string& data)
{
    auto j           = nlohmann::json::parse(data);
    c.handBone       = j.value("handBone",       std::string("Skeleton_arm_joint_L__2_"));
    c.reach          = j.value("reach",          0.8f);
    c.shoulderHeight = j.value("shoulderHeight", 1.3f);
}

// ---- Registration -----------------------------------------------------------

DECLARE_COMPONENT(PunchComponent, "Punch")

// ---- Behavior ---------------------------------------------------------------

class PunchSystem : public GameSystem
{
    DECLARE_SYSTEM(PunchSystem, 100)
public:
    void OnStart(Scene& scene) override
    {
        Input::BindAxis("PunchX",       GamepadAxis::LeftX);
        Input::BindAxis("PunchY",       GamepadAxis::LeftY);
        Input::BindAxis("PunchTrigger", GamepadAxis::LeftTrigger);
    }

    void OnUpdate(Scene& scene, float dt) override
    {
        const float h    = Input::GetAxis("PunchX");
        const float f    = Input::GetAxis("PunchY");
        const bool  held = Input::GetAxis("PunchTrigger") > 0.5f;

        for (auto [entity, punch] : scene.View<PunchComponent>().each())
        {
            // The IK chain is normally authored in the inspector, but IKComponent is
            // not serialized — so for this test we lazily provision it (and a chain for
            // the configured hand bone) the first time we see the entity in play mode.
            IKChain* chain = EnsureChain(scene, entity, punch);
            if (!chain) continue;

            // Latch the punch direction on the rising edge of the trigger, from the
            // current left-stick tilt mapped through the entity's facing. A centered
            // stick punches straight forward.
            if (held && !punch._wasHeld)
                punch._dir = PunchDir(scene, entity, h, f);
            punch._wasHeld = held;

            if (held) {
                glm::vec3 anchor = glm::vec3(0.0f);
                if (scene.Has<TransformComponent>(entity))
                    anchor = scene.Get<TransformComponent>(entity).position;
                anchor.y += punch.shoulderHeight;

                chain->targetEntity   = entt::null;
                chain->targetWorldPos = anchor + punch._dir * punch.reach;
                chain->weight         = 1.0f;
            } else {
                chain->weight = 0.0f;   // eased back to the animated pose by the chain's easeTime
            }
        }
    }

    void OnDestroy(Scene& scene) override {}

private:
    // Make sure the entity has an IKComponent with a chain for the punch hand bone,
    // creating them if needed. Returns the chain to drive, or nullptr if the entity
    // can't run IK at all (no skinned mesh / animator). IKComponent isn't serialized,
    // so the chain is provisioned here rather than authored in the scene.
    IKChain* EnsureChain(Scene& scene, entt::entity entity, PunchComponent& punch)
    {
        // UpdateIK only runs on entities with a skinned mesh + animator; without them
        // a chain would never be solved.
        if (!scene.Has<SkinnedMeshComponent>(entity) || !scene.Has<AnimatorComponent>(entity))
            return nullptr;

        if (!scene.Has<IKComponent>(entity))
            scene.Add<IKComponent>(entity);
        auto& ik = scene.Get<IKComponent>(entity);

        for (auto& c : ik.chains)
            if (c.endEffectorBone == punch.handBone) return &c;

        // No chain for this bone yet — make one. easeTime gives the extend/retract feel.
        IKChain c;
        c.endEffectorBone = punch.handBone;
        c.weight          = 0.0f;
        c.easeTime        = 0.10f;
        ik.chains.push_back(c);
        return &ik.chains.back();
    }

    // World-space punch direction from the left-stick tilt, oriented by the entity's
    // yaw (same flatten convention as MoveSystem). Defaults to forward when centered.
    static glm::vec3 PunchDir(Scene& scene, entt::entity entity, float h, float f)
    {
        glm::vec3 right(1, 0, 0), forward(0, 0, -1);
        if (scene.Has<TransformComponent>(entity)) {
            auto& xf = scene.Get<TransformComponent>(entity);
            right   = xf.rotation * glm::vec3(1, 0,  0);
            forward = xf.rotation * glm::vec3(0, 0, -1);
            right.y   = 0.0f; right   = glm::length(right)   > 1e-4f ? glm::normalize(right)   : glm::vec3(1, 0, 0);
            forward.y = 0.0f; forward = glm::length(forward) > 1e-4f ? glm::normalize(forward) : glm::vec3(0, 0, -1);
        }
        glm::vec3 dir = right * h + forward * f;
        return glm::length(dir) > 1e-3f ? glm::normalize(dir) : forward;
    }
};
