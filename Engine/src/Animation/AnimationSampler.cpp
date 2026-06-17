#include "Animation/AnimationSampler.h"

#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/quaternion.hpp>

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

// Walks the (sorted) skeleton turning per-bone local matrices into the skinning
// palette in a single forward pass.
static void ComputePalette(const Skeleton& skel,
                           const std::vector<glm::mat4>& locals,
                           std::vector<glm::mat4>& out)
{
    const size_t n = skel.bones.size();
    std::vector<glm::mat4> world(n);
    out.resize(n);
    for (size_t i = 0; i < n; ++i) {
        int parent = skel.bones[i].parent;
        world[i] = (parent < 0) ? locals[i] : world[parent] * locals[i];
        out[i]   = world[i] * skel.bones[i].inverseBind;
    }
}

void ComputeBindPose(const Skeleton& skel, std::vector<glm::mat4>& out)
{
    const size_t n = skel.bones.size();
    std::vector<glm::mat4> locals(n);
    for (size_t i = 0; i < n; ++i) {
        const Bone& b = skel.bones[i];
        locals[i] = TRS(b.localT, b.localR, b.localS);
    }
    ComputePalette(skel, locals, out);
}

void SamplePose(const Skeleton& skel, const AnimationClip& clip, float time,
                std::vector<glm::mat4>& out)
{
    const size_t n = skel.bones.size();

    // Start from the rest pose, then let the clip's channels override.
    std::vector<glm::vec3> T(n), S(n);
    std::vector<glm::quat> R(n);
    for (size_t i = 0; i < n; ++i) {
        T[i] = skel.bones[i].localT;
        R[i] = skel.bones[i].localR;
        S[i] = skel.bones[i].localS;
    }
    for (const BoneChannel& ch : clip.channels) {
        if (ch.boneIndex < 0 || ch.boneIndex >= (int)n) continue;
        int i = ch.boneIndex;
        T[i] = SampleVec3(ch.positions, time, T[i]);
        R[i] = SampleQuat(ch.rotations, time, R[i]);
        S[i] = SampleVec3(ch.scales,    time, S[i]);
    }

    std::vector<glm::mat4> locals(n);
    for (size_t i = 0; i < n; ++i) locals[i] = TRS(T[i], R[i], S[i]);
    ComputePalette(skel, locals, out);
}

} // namespace Diamond
