#include "Animation/AnimatorAPI.h"
#include "Animation/AnimationComponents.h"
#include "Scene/Scene.h"

namespace Animator {

void SetFloat(Scene& scene, entt::entity e, const std::string& name, float value)
{
    if (auto* sm = scene.GetRegistry().try_get<AnimStateMachineComponent>(e))
        sm->SetFloat(name, value);
}

void SetBool(Scene& scene, entt::entity e, const std::string& name, bool value)
{
    if (auto* sm = scene.GetRegistry().try_get<AnimStateMachineComponent>(e))
        sm->SetBool(name, value);
}

void SetTrigger(Scene& scene, entt::entity e, const std::string& name)
{
    if (auto* sm = scene.GetRegistry().try_get<AnimStateMachineComponent>(e))
        sm->SetTrigger(name);
}

float GetFloat(Scene& scene, entt::entity e, const std::string& name)
{
    if (auto* sm = scene.GetRegistry().try_get<AnimStateMachineComponent>(e))
        return sm->GetFloat(name);
    return 0.0f;
}

bool GetBool(Scene& scene, entt::entity e, const std::string& name)
{
    if (auto* sm = scene.GetRegistry().try_get<AnimStateMachineComponent>(e))
        return sm->GetBool(name);
    return false;
}

std::string CurrentState(Scene& scene, entt::entity e)
{
    auto* sm = scene.GetRegistry().try_get<AnimStateMachineComponent>(e);
    if (!sm || !sm->machine) return {};
    const auto& states = sm->machine->states;
    if (sm->currentState < 0 || sm->currentState >= (int)states.size()) return {};
    return states[sm->currentState].name;
}

} // namespace Animator
