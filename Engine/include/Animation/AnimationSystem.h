#pragma once

#include <entt/entt.hpp>

namespace Diamond {

// Advances every (SkinnedMeshComponent, AnimatorComponent) pair by dt and writes
// each animator's bone-matrix palette for this frame. Call once per frame before
// building draw calls. Runs in the editor too (not gated on play mode) so poses
// can be previewed live.
void UpdateAnimators(entt::registry& reg, float dt);

} // namespace Diamond
