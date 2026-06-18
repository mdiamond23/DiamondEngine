# Ragdoll System Design

Builds on the constraint + motor system (`physics-design.md`) and the skeletal
animation pipeline (`Engine/Animation/`). This document covers **v1: passive
(limp) ragdolls** — a skinned character whose skeleton can hand control from the
animation system over to physics and flop. Active (motor-driven, fights toward an
animation pose) ragdolls are a v2 layer; the constraint motors it needs already
exist (`Physics::SetMotorTargetOrientation`).

---

## Goals (v1)

- A `.ragdoll` asset describing how a skeleton maps onto a chain of rigid bodies
  and joints, auto-generated from the skeleton and then editable per-bone.
- A `RagdollComponent` (plain data) whose physics bodies + joints are owned
  internally by `PhysicsSystem` — bones are **not** ECS entities, and Jolt types
  stay out of the public header.
- Bodies always present: built **Kinematic** at play start, following the
  animation each frame; flipped to **Dynamic** to go limp (inheriting velocity).
- The animation→physics hand-off implemented as a palette override in the
  skinning path, so the simplified ~15-bone physics skeleton drives the full
  render skeleton for free.
- Triggering via a manual API (`Physics::SetRagdollMode`) and a minimal
  auto-on-impact threshold.

### Out of scope for v1
- Active / powered ragdoll (motors fighting toward an animation pose).
- Get-up / recovery blend (ragdoll → standing animation).
- Partial / per-bone ragdoll (upper body limp while legs animate). The data model
  leaves the door open (per-bone mapping), but no blend weights in v1.

---

## Why these choices

| Decision | Choice | Rationale |
|----------|--------|-----------|
| Body representation | `RagdollComponent` (plain data); bodies+joints owned internally by `PhysicsSystem` | Cheaper and cleaner than ~15 child entities; keeps the hierarchy uncluttered. Parallel to — not built on — the entity/UUID constraint system. |
| Body+joint impl | **Route B — reuse existing primitives** (see below) | `BuildConstraint`, `MoveKinematic`, motors, the group filter and contact `bodyMap` already exist and are tuned. Jolt's `Ragdoll` helper would duplicate them via a parallel body path. |
| Authoring | Auto-generate → editable `.ragdoll` asset | Fast start (no hand-authoring per character), reversible (asset is editable), and makes the "how many bones?" question data-driven rather than baked into the engine. |
| Per-bone shapes | Authored in the asset | Collision feel needs hand-tuning; auto-gen only seeds a default. |
| Granularity | ~15 core bones to start, scalable via the asset | Engine is body-count agnostic; unmapped bones ride their physics parent for free (see Readback). |
| Lifecycle | Always present, Kinematic-follow → flip to Dynamic | Instant, physically correct transition: a body that was following animation already has the right velocity when it goes dynamic. |

---

## Implementation route — Route B (reuse existing primitives)

An earlier draft said "lean on Jolt's `Ragdoll`/`RagdollSettings` helper." After
reading `PhysicsSystem.cpp`, **that's reversed**: every hard primitive a ragdoll
needs already exists in the engine, so we reuse them rather than introduce a
parallel body-management path.

| Need | Reuse |
|------|-------|
| A joint (swing-twist / hinge, with motors for v2) | `BuildConstraint(cc, body1, body2)` — takes **raw Jolt bodies**, world-space anchor+axis. `RegisterConstraint` tracks teardown. |
| "Drive to pose using kinematics" | `MoveKinematic(id, pos, rot, FIXED_DT)` — already the idiom in `SyncKinematicBodies`. Gives correct velocity on the flip for free. |
| Offset capsule from the bone frame | `RotatedTranslatedShapeSettings` (already used in `CreateShape`). |
| Self-collision off | the per-body `GroupFilter` (same group → bones ignore each other). |
| Contacts resolve to the character | `mUserData = entity` + the `bodyMap` the contact listener reads. |
| Motor drive (v2) | `Physics::SetMotorTargetOrientation` — already implemented. |

What Jolt's helper would have added (`DriveToPoseUsingKinematics` /
`DriveToPoseUsingMotors`) reduces to "`MoveKinematic` each body" and
"`SetMotorTargetOrientation` each joint" — both of which we already have. So the
new surface is: one build function, two per-frame passes, and a mode flag.

---

## Concerns / gotchas (must handle while coding)

1. **Velocity inheritance requires `MoveKinematic`, not teleport.** Setting a
   body's position each frame (`SetPositionAndRotation`) records *zero* velocity,
   so the flip to Dynamic flops from rest — defeating the whole lifecycle. Use
   `MoveKinematic(id, target, dt)` (Jolt derives the velocity). `dt` must be the
   **physics step `FIXED_DT`**, not the variable frame dt.
2. **Entity transform / bounds / gameplay drift when limp.** The skin renders
   correctly regardless (the entity matrix cancels in the readback:
   `entityWorld · modelFromWorld · bodyWorld = bodyWorld`). But the mesh **AABB**
   (frustum culling) and any **gameplay query** reading `TransformComponent` see
   the stale pre-limp origin. So while limp, re-root the entity transform to the
   hips body each frame — for bounds/gameplay, **not** for the skin.
3. **The body's reference frame must equal the *bone* frame, not the capsule
   center.** The readback does `bodyWorld · inverseBind`, which only reconstructs
   the skin if `bodyWorld` is the bone's transform. Place the body at the bone
   origin and offset the *shape* along the bone via `RotatedTranslatedShape`.
4. **Scale.** glTF models carry an import scale and bind poses can have non-unit
   bone scale; Jolt bodies are rigid. Bake the entity/import scale into capsule
   dimensions and placement, or the ragdoll ends up giant/tiny/offset.
5. **Auto-trigger on a kinematic body is awkward** (infinite mass → the reported
   impulse isn't a clean "hit hard" signal). v1 ships **manual
   `SetRagdollMode` only**; auto-on-impact is a fast-follow once bodies are
   Dynamic and the signal is trustworthy. (Reduced from the original v1 scope.)

---

## Code layout (Engine vs Sandbox)

Mirrors the established split (`AnimationStateMachine.h` in Engine vs
`AnimStateMachineAsset.h` in Sandbox; `PBRMaterial` vs `MaterialAsset.h`):

| File | Where | Contents |
|------|-------|----------|
| `Engine/include/Scene/Physics/RagdollConfig.h` | Engine | `RagdollBodyDef`, `RagdollConfig` (plain data, no JSON) + `BuildDefaultRagdoll(skeleton)` decl |
| `Engine/src/Scene/Physics/RagdollConfig.cpp` | Engine | `BuildDefaultRagdoll` (auto-gen from skeleton) |
| `Engine/include/Scene/Physics/RagdollComponent.h` | Engine | `RagdollComponent` (holds `shared_ptr<RagdollConfig>` + mode + `_ragdollId`) — **PR2** |
| `Sandbox/src/Editor/RagdollAsset.h` | Sandbox | `LoadRagdoll` / `SaveRagdoll` (nlohmann, `inline`, enum↔string) |

`nlohmann_json` is linked to **Sandbox, not MyEngine** — keep all JSON in the
Sandbox header.

---

## Data model (Engine, plain data)

```cpp
// Engine/include/Scene/Physics/RagdollConfig.h
#pragma once
#include <glm/glm.hpp>
#include <string>
#include <vector>
#include "Constraint.h"   // reuse ConstraintType
namespace Diamond { struct Skeleton; }

struct RagdollBodyDef {
    std::string    boneName;
    std::string    parentBoneName;            // "" = simulation root (the hips)

    enum class Shape { Capsule, Box, Sphere };
    Shape          shape       = Shape::Capsule;
    float          radius      = 0.05f;       // capsule / sphere
    float          halfHeight  = 0.10f;       // capsule cylinder half-length
    glm::vec3      halfExtents { 0.08f };     // box
    float          mass        = 1.0f;

    // Joint to parent (ignored for the root). Maps onto ConstraintComponent fields.
    ConstraintType jointType   = ConstraintType::SwingTwist;
    glm::vec3      twistAxisLocal { 0, 1, 0 };// along the bone, in the bone's frame
    float          swingNormalDeg = 30.0f, swingPlaneDeg = 30.0f;
    float          twistMinDeg = -15.0f, twistMaxDeg = 15.0f;
    float          hingeMinDeg = 0.0f,  hingeMaxDeg = 150.0f;
};

struct RagdollConfig {
    std::vector<RagdollBodyDef> bodies;
    float impactThreshold = 0.0f;
};

// Best-effort auto-generation (topology + role hints). The editable asset is the
// real authority; this only seeds it.
RagdollConfig BuildDefaultRagdoll(const Diamond::Skeleton& skel);
```

```cpp
// Engine/include/Scene/Physics/RagdollComponent.h  (PR2)
#pragma once
#include <memory>
#include <string>
#include <cstdint>
#include "RagdollConfig.h"

enum class RagdollMode { Animated, Limp };   // v1; Powered (motors) = v2

struct RagdollComponent {
    std::string                    assetPath;       // the .ragdoll file
    std::shared_ptr<RagdollConfig> config;          // loaded by the editor (like AnimStateMachineComponent::machine)
    RagdollMode                    mode = RagdollMode::Animated;
    uint32_t                       _ragdollId = 0xFFFFFFFFu;  // -> PhysicsSystem::Impl, internal
};
```

---

## Auto-generation from skeleton (topology, not names)

CesiumMan (our test asset) has idiosyncratic bone names
(`Skeleton_arm_joint_L__4_`, `leg_joint_R_5`, …) with inconsistent L/R numbering,
which kills any universal name table. `BuildDefaultRagdoll` is therefore
**topology + coarse role hints + best-effort**, and the asset/inspector is the
real authority:

- **Role by substring** (lowercased): `torso|spine|hip|pelvis|chest` → spine;
  `neck|head` → neck/head; `arm|shoulder|hand|forearm` → arm;
  `leg|thigh|shin|calf|foot` → leg. Bones with no role hint are skipped (the user
  can add them in the asset).
- **Joint type by chain depth, not name:** the root of a limb → swing-twist; the
  next joint down (elbow/knee) → hinge. Reading the hierarchy sidesteps the
  inconsistent numbering. (Keyword overrides like `elbow|knee` still apply.)
- **Parent** = nearest *included* ancestor bone (intermediate skipped bones still
  skin off their physics parent via the readback).
- **Shape / size / twist axis** are derived from the bind pose: each bone's bind
  world transform is `inverse(Bone::inverseBind)`; the direction to its child
  gives the capsule length, a heuristic radius, and the local twist axis.
- **Mass** roughly by role (torso heavy, hands/feet light).

> Expect to hand-fix a few joints for CesiumMan — that's the point of the editable
> asset, not a failure of auto-gen.

---

## The animation → physics hand-off (the readback)

This is the heart of v1. `ComputePalette`
(`Engine/src/Animation/AnimationSampler.cpp`) builds each bone's **model-space**
world matrix, then `palette[i] = world[i] * inverseBind`. The ragdoll supplies
`world[i]` from physics instead of the local pose chain:

```
modelFromWorld = inverse(entity world matrix)   // bodies are world-space; palette is model-space

for each bone i (skeleton order, parents first):
    if bone i is mapped to a ragdoll body:
        world[i] = modelFromWorld * bodyWorldTransform[mapped(i)]
    else:
        world[i] = world[parent(i)] * TRS(pose[i])     // hangs off its physics-driven parent
    palette[i] = world[i] * inverseBind
```

**Why the simplified skeleton works:** unmapped bones (fingers, toes, twist
bones) take the `else` branch and ride their physics-driven parent automatically.
No special handling, no popping.

**Mode behaviour:**
- *Animated:* skip the readback entirely; keep the animation palette from
  `UpdateAnimators`. The kinematic bodies just shadow the animation.
- *Limp:* run the readback after the physics step; it overwrites the palette
  `UpdateAnimators` produced.

**Shared hook:** add `ComputeWorldTransforms(skel, pose, out)` — a small sibling
of `ComputePalette` that returns the model-space `world[]` it already computes
internally — so build, the kinematic drive, and the readback all read one source
of truth.

---

## Setup & per-frame code (Route B)

### Internal instance (Jolt stays in PhysicsSystem.cpp)

```cpp
struct RagdollInstance {
    entt::entity              entity = entt::null;
    std::vector<JPH::BodyID>  bodies;        // bodies[k]
    std::vector<int>          boneIndex;     // bodies[k] drives skeleton bone boneIndex[k]
    std::vector<uint32_t>     joints;        // ids into impl.constraintMap
    uint32_t                  group = 0;     // per-ragdoll self-collision group
    RagdollMode               mode  = RagdollMode::Animated;
};
// impl: std::unordered_map<uint32_t /*ragdollId*/, RagdollInstance> ragdolls;
```

### Build at play start

Per included body: find the bone, compute its world frame, build a capsule
offset toward its child, create a **Kinematic** body (userData=entity, per-ragdoll
collision group, registered in `bodyMap`). Then per non-root body, build the
joint by feeding an in-memory `ConstraintComponent` (world-space anchor at the
child bone origin, axis = world twist axis) through `BuildConstraint` +
`RegisterConstraint`. While kinematic the joints are inert; they hold the skeleton
together the instant we go limp. (See chat transcript / PR2 for the full body.)

### Per frame — Animated mode (before the physics step)

```cpp
for each ragdoll inst with mode == Animated:
    world = ComputeWorldTransforms(skeleton, currentPose)
    for k in inst.bodies:
        glm::mat4 w = entityXf * world[inst.boneIndex[k]];
        bi.MoveKinematic(inst.bodies[k], pos(w), rot(w), FIXED_DT);   // concern #1
```

### Trigger → Limp

```cpp
for each id in inst.bodies:
    bi.SetMotionType(id, Dynamic, Activate);   // velocity already correct from the follow
inst.mode = Limp;
```

### Readback — Limp mode (after the physics step, overwrites the palette)

```cpp
modelFromWorld = inverse(entityXf);
for i in skeleton.bones (parents first):
    if boneToBody[i] >= 0: world[i] = modelFromWorld * bodyWorld(inst.bodies[boneToBody[i]]);
    else:                  world[i] = (parent<0) ? TRS(bone) : world[parent] * TRS(bone);
    anim.palette[i] = world[i] * bones[i].inverseBind;
// also: re-root entity TransformComponent to the hips body (concern #2)
```

---

## Lifecycle

| Event | Action |
|-------|--------|
| Play start | Build kinematic bodies + joints from the config (`BuildConstraint`), per-ragdoll collision group. |
| Each frame, Animated mode | `MoveKinematic` each body to its bone's model→world transform. |
| Trigger → Limp | `SetMotionType` all bodies → Dynamic. Velocity inherited from the follow. Skinning switches to the readback path; entity re-roots to hips. |
| Play stop | Tear down bodies + joints with the same deferred-destruction safety as constraints (no destruction mid-step). |

---

## System ordering

```
UpdateStateMachines
UpdateAnimators                         // writes the animation palette
RagdollSystem (pre-physics)             // Animated: MoveKinematic each body
── physics step ──
RagdollSystem (post-physics)            // Limp: readback → overwrite palette + re-root entity
skinning upload (uBones = palette)
```

The readback must run **after** the physics step and **before** the palette is
uploaded.

---

## Triggering

- **Manual (v1):** `Physics::SetRagdollMode(entt::entity, RagdollMode)` —
  free-function style matching the rest of the `Physics::` API.
- **Auto-on-impact (fast-follow, not v1):** once bodies are Dynamic, detect a
  hard contact (relative velocity / other body's impulse, not the kinematic
  body's) above `RagdollConfig::impactThreshold` and flip to Limp. Deferred per
  concern #5.

---

## Build order (PRs)

1. **`.ragdoll` config + I/O** — `RagdollConfig` (Engine), `BuildDefaultRagdoll`
   (Engine), `LoadRagdoll`/`SaveRagdoll` (Sandbox). Pure data; verifiable in
   isolation.
2. **`RagdollComponent` + build/destroy lifecycle** — kinematic bodies + joints
   via `BuildConstraint`, following the animation. Verify with
   `Physics::DrawColliders` that capsules track the walk cycle. Add
   `ComputeWorldTransforms`.
3. **Trigger → Dynamic flip + readback palette override + entity re-root** — the
   actual flop (concerns #1, #2).
4. **Asset inspector panel** — per-bone shape/mass/limit editing. (Auto-on-impact
   later.)

---

## Open details (to settle during implementation)

- Joint-limit defaults per bone role (tune against CesiumMan).
- Mass distribution heuristic — role-proportional vs. authored tables.
- Root-bone handling — the hips body as simulation root + entity re-root on limp
  (concern #2).
- Solver iterations for the ~15-joint chain (slow convergence / mass-ratio
  instability — same concern as long constraint chains in `physics-design.md`).

---

## v2 preview — active ragdoll

The motor primitives already exist. The active loop is: sample/blend the `Pose`
(animation system, done) → per mapped bone, feed its local rotation as the
swing-twist motor target via `Physics::SetMotorTargetOrientation`. `RagdollMode`
gains `Powered`; bodies stay Dynamic with motors enabled instead of Kinematic.
Motor strength (`motorMaxTorque`) becomes the dial between "tracks animation
tightly" and "floppy struggle" — the Gang Beasts feel.
