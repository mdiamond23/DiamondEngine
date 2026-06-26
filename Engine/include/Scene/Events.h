#pragma once
#include <entt/entt.hpp>
 
// A UI button completed a click (mouse release-inside or gamepad activate).
// Lets game logic react to any button globally without per-button onClick
// wiring; the existing UIButtonComponent::onClick continues to work alongside.
struct UIButtonFired {
    entt::entity button = entt::null;
};

// A component-owned AudioSourceComponent voice began playing (AudioSystem started
// it — e.g. playOnStart). Fire-and-forget one-shots do NOT emit this: they have no
// owning entity. Published deferred (enqueue), drained at end of frame.
struct SoundStarted {
    entt::entity source = entt::null;
};

// A component-owned AudioSourceComponent voice finished — a non-looping source that
// played to its end and was reclaimed. Like SoundStarted, scoped to source voices
// only (one-shots are reclaimed inside the engine, which has no entity context).
struct SoundFinished {
    entt::entity source = entt::null;
};

// A UI-bus sound was played in response to a UIButtonFired, because the button
// entity carried an AudioSourceComponent. Lets game logic know a click sounded
// without wiring each button.
struct UISoundPlayed {
    entt::entity button = entt::null;
};
