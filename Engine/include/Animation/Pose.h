#pragma once

#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>
#include <vector>

namespace Diamond {

// One bone's local (parent-relative) transform, kept decomposed rather than as a
// mat4 so two poses can be blended component-wise: lerp translation/scale, slerp
// rotation. Composing to matrices too early would make blending meaningless.
struct BoneTransform {
    glm::vec3 translation { 0.0f };
    glm::quat rotation    { 1.0f, 0.0f, 0.0f, 0.0f };
    glm::vec3 scale       { 1.0f };
};

// A full skeletal pose in local space, indexed 1:1 with Skeleton::bones. This is
// the intermediate the animation system samples, blends (cross-fade, state
// machine), and finally composes into the GPU skinning palette. It is also the
// hand-off point a future active ragdoll will read to drive joint motors toward
// an animated target.
using Pose = std::vector<BoneTransform>;

} // namespace Diamond
