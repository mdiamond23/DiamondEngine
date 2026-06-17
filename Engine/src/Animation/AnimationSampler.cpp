#include "Animation/AnimationSampler.h"

#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/quaternion.hpp>
#include <algorithm>

namespace Diamond {

// --- Track interpolation ---------------------------------------------------
// Keyframes are sorted by time. Out-of-range times clamp to the first/last key.

static glm::vec3 SampleVec3(const std::vector<Keyframe<glm::vec3>>& keys,
                            float t, const glm::vec3& fallback)
{
    if (keys.empty())            return fallback;
    if (t <= keys.front().time)  return keys.front().value;
    if (t >= keys.back().time)   return keys.back().value;

    for (size_t i = 0; i + 1 < keys.size(); ++i) {
        if (t < keys[i + 1].time) {
            float span = keys[i + 1].time - keys[i].time;
            float a    = span > 0.0f ? (t - keys[i].time) / span : 0.0f;
            return glm::mix(keys[i].value, keys[i + 1].value, a);
        }
    }
    return keys.back().value;
}

static glm::quat SampleQuat(const std::vector<Keyframe<glm::quat>>& keys,
                            float t, const glm::quat& fallback)
{
    if (keys.empty())            return fallback;
    if (t <= keys.front().time)  return keys.front().value;
    if (t >= keys.back().time)   return keys.back().value;

    for (size_t i = 0; i + 1 < keys.size(); ++i) {
        if (t < keys[i + 1].time) {
            float span = keys[i + 1].time - keys[i].time;
            float a    = span > 0.0f ? (t - keys[i].time) / span : 0.0f;
            glm::quat q0 = keys[i].value;
            glm::quat q1 = keys[i + 1].value;
            // Shortest-arc: flip one quaternion if they're in opposite hemispheres,
            // otherwise slerp can take the long way around.
            if (glm::dot(q0, q1) < 0.0f) q1 = -q1;
            return glm::normalize(glm::slerp(q0, q1, a));
        }
    }
    return keys.back().value;
}

static glm::mat4 TRS(const glm::vec3& t, const glm::quat& r, const glm::vec3& s)
{
    return glm::translate(glm::mat4(1.0f), t) * glm::mat4_cast(r) * glm::scale(glm::mat4(1.0f), s);
}

// --- stage 1: sample -------------------------------------------------------

void BindPose(const Skeleton& skel, Pose& out)
{
    const size_t n = skel.bones.size();
    out.resize(n);
    for (size_t i = 0; i < n; ++i) {
        const Bone& b = skel.bones[i];
        out[i] = { b.localT, b.localR, b.localS };
    }
}

void SampleLocalPose(const Skeleton& skel, const AnimationClip& clip, float time,
                     Pose& out)
{
    // Start from the rest pose, then let the clip's channels override per bone.
    BindPose(skel, out);
    const int n = (int)skel.bones.size();
    for (const BoneChannel& ch : clip.channels) {
        if (ch.boneIndex < 0 || ch.boneIndex >= n) continue;
        BoneTransform& bt = out[ch.boneIndex];
        bt.translation = SampleVec3(ch.positions, time, bt.translation);
        bt.rotation    = SampleQuat(ch.rotations, time, bt.rotation);
        bt.scale       = SampleVec3(ch.scales,    time, bt.scale);
    }
}

// --- stage 2: blend --------------------------------------------------------

void BlendPoses(const Pose& a, const Pose& b, float w, Pose& out)
{
    const size_t n = std::min(a.size(), b.size());
    out.resize(n);
    for (size_t i = 0; i < n; ++i) {
        out[i].translation = glm::mix(a[i].translation, b[i].translation, w);
        out[i].scale       = glm::mix(a[i].scale,       b[i].scale,       w);
        glm::quat qa = a[i].rotation;
        glm::quat qb = b[i].rotation;
        if (glm::dot(qa, qb) < 0.0f) qb = -qb;   // shortest arc
        out[i].rotation = glm::normalize(glm::slerp(qa, qb, w));
    }
}

// --- stage 3: compose ------------------------------------------------------
// Walks the (sorted) skeleton turning local transforms into the skinning palette
// in a single forward pass.

void ComputePalette(const Skeleton& skel, const Pose& pose, std::vector<glm::mat4>& out)
{
    const size_t n = skel.bones.size();
    std::vector<glm::mat4> world(n);
    out.resize(n);
    for (size_t i = 0; i < n; ++i) {
        const BoneTransform& bt = pose[i];
        glm::mat4 local = TRS(bt.translation, bt.rotation, bt.scale);
        int parent = skel.bones[i].parent;
        world[i] = (parent < 0) ? local : world[parent] * local;
        out[i]   = world[i] * skel.bones[i].inverseBind;
    }
}

// --- convenience wrappers --------------------------------------------------

void ComputeBindPose(const Skeleton& skel, std::vector<glm::mat4>& out)
{
    Pose pose;
    BindPose(skel, pose);
    ComputePalette(skel, pose, out);
}

void SamplePose(const Skeleton& skel, const AnimationClip& clip, float time,
                std::vector<glm::mat4>& out)
{
    Pose pose;
    SampleLocalPose(skel, clip, time, pose);
    ComputePalette(skel, pose, out);
}

} // namespace Diamond
