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

} // namespace Diamond
