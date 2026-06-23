#pragma once

class Scene;

namespace Diamond {

// Per-frame inverse-kinematics pass. For every entity carrying an IKComponent (plus
// a SkinnedMeshComponent + AnimatorComponent), each chain pulls its end-effector
// bone toward its target and the solved rotations are blended into the animator's
// local pose by the chain weight; the bone palette is then recomposed.
//
// Runs AFTER UpdateAnimators (it needs the built pose and a placed chain root) and
// BEFORE the palette is consumed by skinning. An entity whose ragdoll currently owns
// the pose (Limp/Powered) is skipped — the ragdoll readback wins. See Docs/ik-design.md.
//
// `dt` is the frame delta; it drives the per-chain weight/target easing so reaches
// ramp in/out instead of snapping (set a chain's easeTime to 0 to disable easing).
void UpdateIK(Scene& scene, float dt);

} // namespace Diamond
