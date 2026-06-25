#pragma once
#include <entt/entt.hpp>
 
// A UI button completed a click (mouse release-inside or gamepad activate).
// Lets game logic react to any button globally without per-button onClick
// wiring; the existing UIButtonComponent::onClick continues to work alongside.
struct UIButtonFired {
    entt::entity button = entt::null;
};
