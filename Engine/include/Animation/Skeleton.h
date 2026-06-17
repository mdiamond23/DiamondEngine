#pragma once

#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>
#include <string>
#include <vector>
#include <unordered_map>

namespace Diamond {

// One joint in a skeleton. Bones live in a flat array (see Skeleton) and refer to
// their parent by index, not pointer — this keeps the data copyable/serializable,
// maps 1:1 onto the GPU bone-matrix palette, and lets world transforms be computed
// in a single forward pass instead of recursive pointer chasing.
struct Bone {
    std::string name;
    int         parent = -1;   // index into Skeleton::bones; -1 marks a root

    // Mesh space -> this bone's space in the rest (bind) pose. The skinning matrix
    // for a bone is animatedWorld * inverseBind.
    glm::mat4 inverseBind { 1.0f };

    // Rest-pose transform relative to the parent, kept decomposed as TRS because
    // animation interpolates the three independently (lerp T/S, slerp R). When a
    // clip animates this bone these are overridden per-channel; otherwise they are
    // the default pose.
    glm::vec3 localT { 0.0f };
    glm::quat localR { 1.0f, 0.0f, 0.0f, 0.0f };
    glm::vec3 localS { 1.0f };
};

// A skeleton is a flat, topologically sorted list of bones (every bone appears
// after its parent) plus a name->index lookup used to bind animation channels and
// to map ragdoll rigid bodies onto bones later.
struct Skeleton {
    std::vector<Bone>                    bones;
    std::unordered_map<std::string, int> nameToIndex;

    int Find(const std::string& name) const {
        auto it = nameToIndex.find(name);
        return it == nameToIndex.end() ? -1 : it->second;
    }
};

} // namespace Diamond
