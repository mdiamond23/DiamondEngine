#pragma once

#include <entt/entt.hpp>

namespace Diamond {

// Writes every (SkinnedMeshComponent, AnimatorComponent) pair's bone-matrix
// palette for this frame, and advances playback time when `advance` is true.
// Call once per frame before building draw calls. The pose is always sampled
// (so the mesh stays correctly skinned), but time only moves forward in play
// mode — pass `scene.IsPlaying() && !scene.IsPaused()`. With advance=false the
// character holds its current frame, which is also what lets the inspector Time
// slider scrub a pose in the editor.
void UpdateAnimators(entt::registry& reg, float dt, bool advance = true);

// Drives each entity's AnimatorComponent from its AnimStateMachineComponent: on
// first tick it enters the machine's entry state; thereafter (only when `advance`
// is true) it evaluates the current state's outgoing transitions against the live
// parameters and, on the first whose conditions all pass, cross-fades to the
// target state. Triggers tested by a firing transition are consumed. Call once per
// frame BEFORE UpdateAnimators so the animator advances the clip the SM selected.
void UpdateStateMachines(entt::registry& reg, float dt, bool advance = true);

} // namespace Diamond
