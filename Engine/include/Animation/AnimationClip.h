#pragma once

#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>
#include <string>
#include <vector>

namespace Diamond {

// A single sampled value at a point in time. Times are in seconds.
template<typename T>
struct Keyframe {
    float time = 0.0f;
    T     value{};
};

// Animation for one bone: three independent tracks because glTF stores
// translation, rotation, and scale as separate samplers that may have different
// keyframe counts and timings. Empty tracks fall back to the bone's rest pose.
struct BoneChannel {
    int boneIndex = -1;
    std::vector<Keyframe<glm::vec3>> positions;
    std::vector<Keyframe<glm::quat>> rotations;
    std::vector<Keyframe<glm::vec3>> scales;
};

// One animation (e.g. "Walk", "Idle"). Duration is in seconds (glTF is already
// time-based, so no ticks-per-second conversion). Sampling at a time t produces a
// local TRS per animated bone; the animation system then composes world matrices
// and the bone palette. Interpolation is assumed LINEAR for now (the common glTF
// case); a per-channel mode can be added if an asset needs STEP/CUBICSPLINE.
struct AnimationClip {
    std::string              name;
    float                    duration = 0.0f;
    std::vector<BoneChannel> channels;
};

} // namespace Diamond
