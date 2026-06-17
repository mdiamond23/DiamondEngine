#pragma once

#include <glm/glm.hpp>
#include <vector>
#include "Animation/Skeleton.h"
#include "Animation/AnimationClip.h"

namespace Diamond {

// Produces the bone-matrix palette uploaded to the GPU skinning shader:
//   palette[i] = worldTransform(bone i) * bone[i].inverseBind
// `out` is sized to skel.bones.size(). Both functions rely on the skeleton being
// topologically sorted (parent before child) so world transforms resolve in one
// forward pass.

// Rest/bind pose with no clip applied. When the model's rest pose matches its
// bind pose (the normal case) every matrix comes out ~identity — a handy sanity
// check on the skeleton/inverse-bind data.
void ComputeBindPose(const Skeleton& skel, std::vector<glm::mat4>& out);

// Samples `clip` at `time` seconds (clamped to each track's range). Bones the
// clip does not animate keep their rest pose. Interpolation is LINEAR (lerp T/S,
// shortest-arc slerp R).
void SamplePose(const Skeleton& skel, const AnimationClip& clip, float time,
                std::vector<glm::mat4>& out);

} // namespace Diamond
