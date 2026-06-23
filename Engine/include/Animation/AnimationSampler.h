#pragma once

#include <glm/glm.hpp>
#include <vector>
#include "Animation/Skeleton.h"
#include "Animation/AnimationClip.h"
#include "Animation/Pose.h"

namespace Diamond {

// The animation pipeline is three stages so poses can be blended:
//   sample (clip -> Pose)  ->  blend (Pose,Pose -> Pose)  ->  compose (Pose -> palette)
// The palette is what the GPU skinning shader consumes:
//   palette[i] = worldTransform(bone i) * bone[i].inverseBind
// All compose steps rely on the skeleton being topologically sorted (parent
// before child) so world transforms resolve in one forward pass.

// ---- stage 1: sample --------------------------------------------------------

// The skeleton's rest pose as local transforms (no clip applied).
void BindPose(const Skeleton& skel, Pose& out);

// Samples `clip` at `time` seconds into local transforms (clamped to each track's
// range). Bones the clip does not animate keep their rest pose. Interpolation is
// LINEAR (lerp T/S, shortest-arc slerp R).
void SampleLocalPose(const Skeleton& skel, const AnimationClip& clip, float time,
                     Pose& out);

// ---- stage 2: blend ---------------------------------------------------------

// Per-bone blend of two equally-sized poses: out = lerp(a, b, w) for T/S and
// shortest-arc slerp for R. w=0 yields `a`, w=1 yields `b`. Used for cross-fades
// and (later) state-machine transitions.
void BlendPoses(const Pose& a, const Pose& b, float w, Pose& out);

// ---- stage 3: compose -------------------------------------------------------

// Composes a local-space pose into per-bone MODEL-space transforms (out sized to
// bone count) in one forward pass: out[i] = out[parent] * TRS(pose[i]). This is the
// shared step under both skinning and procedural passes (ragdoll readback, IK) that
// need bone positions/orientations rather than the inverse-bind-folded palette.
void ComputeWorldTransforms(const Skeleton& skel, const Pose& pose, std::vector<glm::mat4>& out);

// Composes a local-space pose into the skinning palette (out sized to bone count).
void ComputePalette(const Skeleton& skel, const Pose& pose, std::vector<glm::mat4>& out);

// ---- convenience wrappers (sample + compose in one call) --------------------

// Equivalent to BindPose + ComputePalette. When rest pose matches bind pose (the
// normal case) every matrix comes out ~identity — a handy skeleton sanity check.
void ComputeBindPose(const Skeleton& skel, std::vector<glm::mat4>& out);

// Equivalent to SampleLocalPose + ComputePalette.
void SamplePose(const Skeleton& skel, const AnimationClip& clip, float time,
                std::vector<glm::mat4>& out);

} // namespace Diamond
