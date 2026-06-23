#include "Animation/IKSolver.h"

#include <glm/gtc/constants.hpp>
#include <cmath>

namespace Diamond {

// Shortest-arc rotation taking unit `from` onto unit `to`, robust at the parallel
// and antiparallel ends (glm::rotation lives in the experimental gtx headers, so we
// spell it out and dodge GLM_ENABLE_EXPERIMENTAL).
static glm::quat RotationFromTo(const glm::vec3& from, const glm::vec3& to)
{
    const glm::vec3 f = glm::normalize(from);
    const glm::vec3 t = glm::normalize(to);
    const float d = glm::dot(f, t);
    if (d >= 1.0f - 1e-6f) return glm::quat(1, 0, 0, 0);   // already aligned
    if (d <= -1.0f + 1e-6f) {                              // opposite: 180° about any perpendicular
        glm::vec3 axis = glm::cross(f, glm::vec3(1, 0, 0));
        if (glm::dot(axis, axis) < 1e-12f) axis = glm::cross(f, glm::vec3(0, 1, 0));
        return glm::angleAxis(glm::pi<float>(), glm::normalize(axis));
    }
    const glm::vec3 axis = glm::normalize(glm::cross(f, t));
    return glm::angleAxis(std::acos(glm::clamp(d, -1.0f, 1.0f)), axis);
}

// Any unit vector perpendicular to `v` (for the degenerate pole fallback).
static glm::vec3 AnyPerp(const glm::vec3& v)
{
    glm::vec3 p = glm::cross(v, glm::vec3(0, 1, 0));
    if (glm::dot(p, p) < 1e-8f) p = glm::cross(v, glm::vec3(1, 0, 0));
    return glm::normalize(p);
}

void SolveTwoBone(const glm::vec3& a, const glm::vec3& b, const glm::vec3& c,
                  const glm::quat& rootR, const glm::quat& midR,
                  const glm::vec3& target, const glm::vec3& pole,
                  glm::quat& outRootR, glm::quat& outMidR)
{
    constexpr float kEps = 1e-5f;

    const float l1 = glm::length(b - a);   // upper bone length
    const float l2 = glm::length(c - b);   // lower bone length
    if (l1 < kEps || l2 < kEps) {          // degenerate rig: leave the pose untouched
        outRootR = rootR; outMidR = midR;
        return;
    }

    // Reach: can't span past fully extended (nor fold tighter than the bone-length
    // gap). Clamping here makes an out-of-range target an extended-toward-target reach.
    const glm::vec3 toTarget = target - a;
    if (glm::dot(toTarget, toTarget) < kEps * kEps) { outRootR = rootR; outMidR = midR; return; }
    const glm::vec3 dirAT = glm::normalize(toTarget);
    const float lat = glm::clamp(glm::length(toTarget),
                                 glm::abs(l1 - l2) + kEps, l1 + l2 - kEps);
    const glm::vec3 tipPos = a + dirAT * lat;

    // Solve the bent triangle directly (pose-independent — works from a straight or an
    // already-bent start): the elbow sits in the plane spanned by the root->target line
    // and the pole, offset off that line toward the pole. `x` is its projection along
    // the line, `h` the perpendicular height (law of cosines), so both bone lengths are
    // satisfied exactly.
    glm::vec3 bendDir = (pole - a) - dirAT * glm::dot(pole - a, dirAT);  // pole side, perpendicular to the line
    if (glm::dot(bendDir, bendDir) < kEps) bendDir = AnyPerp(dirAT);     // pole colinear with the line
    bendDir = glm::normalize(bendDir);

    const float x = (lat * lat + l1 * l1 - l2 * l2) / (2.0f * lat);
    const float h = std::sqrt(glm::max(0.0f, l1 * l1 - x * x));
    const glm::vec3 elbowPos = a + dirAT * x + bendDir * h;

    // Rotate each bone from its current direction onto the solved one. The lower bone is
    // first carried by the upper bone's rotation, then swung from there onto the tip.
    const glm::quat qUpper = RotationFromTo(b - a, elbowPos - a);
    const glm::vec3 lowerCarried = qUpper * (c - b);
    const glm::quat qLower = RotationFromTo(lowerCarried, tipPos - elbowPos);

    outRootR = glm::normalize(qUpper * rootR);
    outMidR  = glm::normalize(qLower * qUpper * midR);
}

} // namespace Diamond
