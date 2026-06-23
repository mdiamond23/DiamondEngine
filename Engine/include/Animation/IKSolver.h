#pragma once

#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>

namespace Diamond {

// Analytic two-bone inverse kinematics — the workhorse for hand reach and foot
// placement. A two-bone chain (root joint -> mid joint -> tip; e.g. upper arm ->
// forearm -> wrist) has a closed-form solution via the law of cosines: no
// iteration, exact, deterministic. See Docs/ik-design.md.
//
// Everything is MODEL space (the entity transform divided out, as in the ragdoll
// readback) so import scale doesn't distort bone lengths. Inputs:
//   a, b, c   current joint positions: root, mid, tip.
//   rootR     current model-space rotation of the upper bone (a->b).
//   midR      current model-space rotation of the lower bone (b->c).
//   target    where the tip should land. Clamped to reach (|root..target| is held
//             below the fully-extended length), so an out-of-range target just
//             extends the limb straight toward it.
//   pole      bend-plane hint: biases which way the mid joint (elbow/knee) points.
//             Pass a point on the desired bend side, e.g. the current animated
//             elbow position, so the bend stays natural. A degenerate hint falls
//             back to the chain's current plane.
// Outputs the new model-space rotations of the two moving bones; bone lengths are
// taken from |b-a| and |c-b|. Tip orientation (palm-to-handle) is the caller's job.
void SolveTwoBone(const glm::vec3& a, const glm::vec3& b, const glm::vec3& c,
                  const glm::quat& rootR, const glm::quat& midR,
                  const glm::vec3& target, const glm::vec3& pole,
                  glm::quat& outRootR, glm::quat& outMidR);

} // namespace Diamond
