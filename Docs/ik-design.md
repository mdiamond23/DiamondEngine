# Inverse Kinematics System Design

Builds on the skeletal animation pipeline (`Engine/Animation/`) and reuses the
same model-space → local-space machinery the ragdoll readback uses
(`ragdoll-design.md`). This document covers **v1: two-bone analytic IK** — the
workhorse for **hand reach** and **foot placement**: a procedural pass that, given
a world-space target for an end-effector bone (a hand or a foot), solves the joint
rotations that put it there and blends the result into the animated `Pose`.

IK is the next domino in the Milestone-4 chain (GLTF → skeletal anim → constraints
→ ragdoll → **IK** → Grab). It unlocks Grab and pairs with active ragdoll
(hit-react = IK target + motors).

---

## The core idea (and the question it answers)

Normal animation flows **root → leaves**: the hand's world position is just
*whatever falls out* of the spine/shoulder/elbow rotations. IK **reverses the
dependency** — you are *given* a desired world position for a leaf bone (the hand)
and you solve for the joint rotations that land it there.

So the target is **not derived from the skeleton** — it is an input from outside
the animation system. The skeleton is the constraint; the target is a world point
you get from:

- **Hand reach / grab:** the world translation of the thing being reached for —
  `scene.GetTransformSystem().GetWorldMatrix(targetEntity)[3]`, or a named socket /
  offset on a weapon, ledge, or door handle.
- **Foot placement:** a downward `Physics::Raycast` from under each foot; the hit
  point (+ surface normal for foot tilt) is the target.
- **Look-at / aim** (later): the world point being aimed at.

"Start with the end placement" therefore means: pick the **end-effector bone**,
get a **world target** from gameplay or a raycast, walk `Bone::parent` up the chain
to collect the bones allowed to move, and solve.

---

## Goals (v1)

- An `IKComponent` (plain data) holding one or more **chains**, each describing an
  end-effector bone, how many links move, a target (entity or world point), an
  elbow **pole hint**, and a **blend weight**.
- A **two-bone analytic solver** (law of cosines) — closed-form, no iteration,
  exact, stable. Drives a 2-bone chain (upper arm + forearm → wrist; thigh + shin
  → ankle).
- IK runs as a **post-pose pass**: after `UpdateAnimators` builds
  `AnimatorComponent::pose`, before the skinning palette is composed — the same
  slot the ragdoll readback occupies.
- Results written back as **local (parent-relative) rotations** into `pose`, so the
  existing `ComputePalette` path skins the reached limb for free.
- A **weight** per chain (0..1) blended against the animated pose, with target +
  weight **smoothing** so reaches ease in/out instead of snapping.

### Out of scope for v1
- Iterative solvers (FABRIK / CCD) for long chains (spine, tail, look-at). The data
  model leaves `IKSolver` open, but only `TwoBone` ships.
- Full-body IK / balance (multi-chain coupling, COM constraints).
- Editor target handles / gizmos — chains are authored in the inspector as fields;
  in-viewport manipulators are a fast-follow.
- Stretchy bones, soft IK (length-preserving only).

---

## Why these choices

| Decision | Choice | Rationale |
|----------|--------|-----------|
| Solver | **Two-bone analytic** (cosine law), not FABRIK/CCD | An arm/leg is 2 bones → closed-form exists: cheap, deterministic, exact, no iteration or convergence tuning. FABRIK only earns its keep on long chains we don't have yet. |
| Target source | **External** (entity world pos, socket, or raycast) | The target is gameplay/world data, never derived from the pose. Keeps the solver pure: skeleton in, target in, rotations out. |
| Chain definition | End-effector bone + link count, walk `Bone::parent` | Skeleton is topologically sorted with parent indices already; no separate chain authoring graph needed. |
| Pose representation | Solve in **model space**, write back **local** to `Pose` | `Pose` is local TRS (`Pose.h`); converting the solved model rotations back to local lets `ComputePalette` run unchanged. Mirrors the ragdoll local-space hand-off. |
| Animation blend | Per-chain **weight**, slerp against the animated local pose | IK rarely fully overrides animation; a weight gives partial reach and clean enable/disable. |
| Elbow disambiguation | **Pole hint** vector | A 2-bone chain bends in infinitely many planes about the root→target axis; the hint picks one and stops the elbow flipping. |

---

## How the two-bone solve works

A 2-bone chain — root joint `A` (shoulder/hip), mid joint `B` (elbow/knee), tip `C`
(wrist/ankle) — with fixed bone lengths `L1 = |AB|`, `L2 = |BC|` and a target `T`,
has a closed-form solution:

1. **Reach clamp.** Let `d = |T - A|`. If `d ≥ L1 + L2` the target is out of range
   → fully extend the limb straight toward `T` (and stop). If `d ≤ |L1 - L2|`,
   clamp to the minimum-fold distance. Otherwise:
2. **Interior angles via law of cosines.**
   - Elbow bend: `cos(B) = (L1² + L2² − d²) / (2·L1·L2)`.
   - Shoulder lift toward target: `cos(α) = (L1² + d² − L2²) / (2·L1·d)`.
3. **Build the rotations in model space.**
   - Aim the upper bone `A→B` at `T`, then rotate it back by `α` within the bend
     plane.
   - The **bend plane** normal comes from the **pole hint**: project the hint so the
     elbow points the chosen way (typically the current animated elbow direction, so
     it stays natural). `axis = normalize(cross(T - A, poleHint))`.
   - Rotate the lower bone `B→C` by `(π − B)` about the same axis so the tip meets
     `T`.
4. **Optional tip orientation.** If `matchTargetRotation`, after position is solved
   slerp the end-effector bone toward the target's orientation (palm-to-handle),
   weighted.

All of this is `glm::quat` / `glm::vec3` arithmetic — no solver loop.

---

## Where it slots in (the pass)

Structurally identical to the ragdoll get-up blend (decompose → modify →
recompose). For each active chain:

```
1. FK the chain to model space.
   world[i] = (parent<0) ? TRS(pose[i]) : world[parent] * TRS(pose[i])
   (parents-first forward loop — skeleton is topo-sorted, same as ComputePalette)

2. Resolve the target in model space.
   targetModel = inverse(entityWorld) * targetWorld     // entity matrix cancels, as in the ragdoll readback

3. Solve (two-bone math above) → new model-space rotations for the root + mid bone
   (+ tip if matchTargetRotation).

4. Convert back to LOCAL and blend by weight:
   solvedLocalR = conj(parentModelR) * solvedModelR
   pose[i].rotation = slerp(pose[i].rotation, solvedLocalR, weight)   // shortest-arc guarded

5. ComputePalette(skeleton, pose) as usual → skinning sees the reached limb.
```

Because the write-back is local and re-composed through the hierarchy, the limb
stays rigid (bones keep their lengths and joints) and disabling a chain
(`weight → 0`) returns exactly to the animated pose.

---

## Concerns / gotchas (must handle while coding)

1. **Bone lengths from the bind pose, not live.** `L1`, `L2` are constant rig data —
   read them once from the bind transforms (`inverse(Bone::inverseBind)` child
   offsets), not from the animated pose each frame, or a squashed pose corrupts the
   cosine law.
2. **Pole flip / elbow popping.** Without a stable hint the bend plane can snap as
   the target crosses the shoulder→target axis. Derive the hint from the *current
   animated* mid-joint direction (and optionally smooth it) so the elbow stays where
   the animation already had it.
3. **Snapping targets / weights.** A hard target jump or a `weight` step reads as a
   pop (cf. the get-up blend). Filter the resolved target position and ramp `weight`
   over a few frames; expose ease-in/out time per chain.
4. **Solve order vs. the body.** Run arm/leg IK **after** the spine/root is posed,
   so the chain root (shoulder/hip) is already in its final place before the chain
   hangs off it. Within a frame: animators → (spine done) → IK chains → palette.
5. **Model space vs. entity scale.** Solve in **model space** (entity matrix divided
   out), exactly like the ragdoll readback, so import scale and the entity transform
   don't distort lengths. Convert the target into model space once up front.
6. **Unreachable / degenerate targets.** Target coincident with the shoulder, or a
   zero-length hint, must fall back gracefully (extend toward target / keep the
   animated bend) rather than emit NaNs into the pose.
7. **No model-space world helper exists yet.** `ComputePalette` computes `world[]`
   internally and discards it. Factor out a `ComputeWorldTransforms(skel, pose, out)`
   sibling (proposed in `ragdoll-design.md`) so the IK FK pass and the palette
   compose share one source of truth instead of recomputing.

---

## Code layout (Engine vs Sandbox)

Mirrors the established split (`AnimationStateMachine.h` Engine vs
`AnimStateMachineAsset.h` Sandbox; `RagdollConfig.h` Engine vs `RagdollAsset.h`
Sandbox):

| File | Where | Contents |
|------|-------|----------|
| `Engine/include/Animation/IKComponent.h` | Engine | `IKSolver`, `IKChain`, `IKComponent` (plain data, no JSON) |
| `Engine/include/Animation/IKSolver.h` / `.cpp` | Engine | `SolveTwoBone(...)` — pure math, no ECS |
| `Engine/src/Animation/IKSystem.cpp` (or fold into AnimationSystem) | Engine | the per-frame pass: resolve targets, FK, solve, blend into `pose` |
| `Engine/src/Animation/AnimationSampler.cpp` | Engine | add `ComputeWorldTransforms` (concern #7) |
| `Sandbox/src/Editor/...` (inspector) | Sandbox | chain editing UI; later, viewport target handles |

JSON/serialization (if chains are saved with the scene) stays in Sandbox, as with
the ragdoll asset.

---

## Data model (Engine, plain data)

```cpp
// Engine/include/Animation/IKComponent.h
#pragma once
#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>
#include <string>
#include <vector>
#include <entt/entt.hpp>

enum class IKSolver { TwoBone };   // FABRIK / CCD later

struct IKChain {
    // --- chain ---
    std::string  endEffectorBone;          // e.g. "hand_R"; chain walks Bone::parent up
    int          boneCount = 2;            // links that move (2 = forearm + upper arm)
    IKSolver     solver    = IKSolver::TwoBone;

    // --- target (one source) ---
    entt::entity targetEntity = entt::null; // follow this entity's world transform...
    glm::vec3    targetWorldPos { 0.0f };   // ...or an explicit world point (entity == null)
    glm::vec3    targetOffset   { 0.0f };   // offset in target space
    bool         matchTargetRotation = false; // also align the end-effector orientation

    // --- shaping ---
    glm::vec3    poleHint { 0, 0, 1 };      // elbow/knee bend direction (defaults to animated dir)
    float        weight   = 1.0f;           // 0..1 blend against the animated pose
    float        easeTime = 0.15f;          // weight/target smoothing (s); 0 = snap

    // --- runtime (resolved/cached, not authored) ---
    int          _tipBone   = -1;           // resolved from endEffectorBone
    float        _curWeight = 0.0f;         // smoothed weight
    glm::vec3    _curTarget { 0.0f };       // smoothed model-space target
};

struct IKComponent {
    std::vector<IKChain> chains;
};
```

```cpp
// Engine/include/Animation/IKSolver.h  — pure math, no ECS
// Solves a 2-bone chain in MODEL space. a/b/c are the current model-space joint
// positions (shoulder, elbow, wrist); target is model space; pole disambiguates
// the bend plane. Outputs new model-space rotations for the two moving bones.
void SolveTwoBone(const glm::vec3& a, const glm::vec3& b, const glm::vec3& c,
                  const glm::vec3& target, const glm::vec3& pole,
                  glm::quat& outRootDeltaR, glm::quat& outMidDeltaR);
```

---

## System ordering

```
UpdateStateMachines
UpdateAnimators                       // writes pose + animation palette
IKSystem (post-anim, pre-skinning)    // resolve targets → FK → two-bone solve → blend into pose → recompose palette
RagdollSystem (pre/post physics)      // unchanged
skinning upload (uBones = palette)
```

IK must run **after** the pose is built (so it has an animated pose to blend
against and a placed chain root) and **before** the palette is uploaded. When both
IK and a Limp/Powered ragdoll are active on the same entity, the ragdoll readback
owns the pose — IK is skipped for that entity (or, later, layered as a target for
the active-ragdoll motors instead of writing the pose directly).

---

## Build order (PRs)

1. **`SolveTwoBone` + `ComputeWorldTransforms`** — pure math + the shared model-space
   FK helper. Unit-verifiable in isolation (target on a known chain → expected
   wrist position within epsilon, reach-clamp at the extremes).
2. **`IKComponent` + `IKSystem` pass** — resolve `endEffectorBone`, FK the chain,
   solve, write blended local rotations into `pose`. Verify with a static target
   entity that a character's hand tracks it.
3. **Target sources + smoothing** — entity-follow vs. world point, weight/target
   easing, pole hint from the animated bend (concerns #2, #3).
4. **Foot placement** — same solver fed by a downward `Physics::Raycast`; ankle
   target = hit point, optional foot tilt to the surface normal.
5. **Editor** — inspector chain editing, then in-viewport target handles.

---

## Open details (to settle during implementation)

- **Pole hint authoring** — fixed vector vs. always-from-animated-bend vs. a hint
  entity. Default to animated bend; allow override.
- **Hip/pelvis adjustment for feet** — pure leg IK can leave a foot reaching past a
  locked hip; foot placement often also lowers the pelvis. v1 may solve legs only
  and defer pelvis drop.
- **Multi-chain priority** — two arms reaching one object, or arm + spine lean. v1
  solves chains independently in list order; coupling is later.
- **Interaction with active ragdoll** — feed the IK target to joint motors
  (`Physics::SetMotorTargetOrientation`) instead of overwriting `pose`, so a powered
  rig *reaches physically*. The hit-react pairing noted in `ragdoll-design.md`.

---

## Beyond v1

- **FABRIK / CCD** for long chains (spine bend, tail, tentacles, look-at) — same
  pass, different `IKSolver`.
- **Look-at / aim** as a 1-bone constraint (head/eyes) reusing the target plumbing.
- **Grab** (the milestone endpoint): hand-reach IK + an attach/socket so the hand
  locks to and carries an object — the chain target becomes the grab point.
