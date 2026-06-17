#pragma once
#include <entt/entt.hpp>
#include <string>

class Scene;

// Gameplay-facing facade for driving an entity's animation state machine from
// game systems / scripts — e.g. a movement system feeding "speed" so the SM
// blends Idle/Walk/Run, or firing a "hit" trigger. Parameters are matched by name
// against the entity's .animsm graph.
//
// Every call is a silent no-op (getters return a default) if the entity has no
// AnimStateMachineComponent, so callers don't need to guard with Has<>. Mirrors
// the Physics:: API style. The underlying data is just AnimStateMachineComponent,
// so scene.Get<AnimStateMachineComponent>(e).SetFloat(...) works too — this is the
// convenient, null-safe front door.
namespace Animator {

    void SetFloat  (Scene& scene, entt::entity e, const std::string& name, float value);
    void SetBool   (Scene& scene, entt::entity e, const std::string& name, bool  value);
    void SetTrigger(Scene& scene, entt::entity e, const std::string& name);

    float GetFloat (Scene& scene, entt::entity e, const std::string& name);   // 0.0f if absent
    bool  GetBool  (Scene& scene, entt::entity e, const std::string& name);   // false if absent

    // Name of the entity's current state, or "" if it has no machine / hasn't
    // entered a state yet (i.e. before play begins).
    std::string CurrentState(Scene& scene, entt::entity e);

} // namespace Animator
