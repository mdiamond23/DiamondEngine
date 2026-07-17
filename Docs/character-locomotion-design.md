# Character Locomotion Design (Procedural Gait + Powered Ragdoll)

The player controller for **Milestone 5 — Game Layer**: a Gang Beasts–style
character that walks with a **fully procedural gait** — no authored animation
clips. The legs are animated by generated foot targets; the upper body is
deliberately under-powered so it **lags behind the hips and flops from real
inertia**. Sloppiness is the aesthetic, and it comes from genuine dynamics, not
faked wobble.

Home: `Sandbox/src/Scripts/LocamotionController.h`
(`LocamotionControllerComponent` + `LocamotionControllerSystem`, priority
**100** — the pre-physics gameplay band, same as `PunchSystem`, so the pose is
written before the physics step consumes it).

---

## The core idea

Everything hangs off one existing hook: **`SyncRagdollPowered` drives every
ragdoll joint motor toward whatever is in `AnimatorComponent::pose`**
(PhysicsSystem.cpp). Today that pose comes from sampling a clip. Nothing
downstream cares where it comes from.

> **Procedural animation = a system that writes a generated `Pose` instead of a
> sampled one.** The motor pipeline, per-joint torque budgets, `strength` knob,
> flinch / knockdown / get-up reactions, and the physics→palette skinning
> readback all work unchanged.

Three pieces cooperate each frame:

1. **Root drive** — the hips body is moved *kinematically* (`MoveKinematic`)
   along the path the controller integrates from stick input. This is the
   proven mechanism from the get-up heave: joint motors cannot support or
   balance the pelvis (it is the suspension point for the whole chain), but a
   kinematic root reliably drags the dynamic body along and looks right.
2. **Gait generator** — computes per-leg foot targets from a speed-driven phase
   accumulator, solves them with the existing **two-bone IK** solver, and
   writes the resulting leg rotations (plus hip bob and an acceleration lean on
   the spine target) into `AnimatorComponent::pose`.
3. **Powered ragdoll** — set to `RagdollMode::Powered`; motors chase the pose.
   The per-joint torque distribution in the `.ragdoll` asset makes legs track
   crisply while the torso/arms trail: **the floppy upper body is a tuning
   preset, not code.**

Balance is **fake by design**: the kinematic root cannot be knocked over by
contacts — only the reaction system decides to hand control to physics (flinch
/ knockdown flip the hips dynamic, exactly as today). True self-balancing
active ragdolls are out of scope; this is also effectively what the genre
ships.

---

## Goals (v1)

- Left-stick locomotion: **camera-relative** direction, stick **magnitude →
  speed** past a deadzone, smooth accelerate / decelerate / turn-toward-motion.
- A complete procedural walk: alternating plant/swing stepping with no foot
  slide, ground-conforming via raycast, stride rate proportional to speed.
- The "drunk marionette" look: stiff legs, medium spine, weak arms/shoulders/
  head — upper body visibly lags on acceleration and swings passively.
- Idle = standing pose + subtle sway (the weak motors provide most of it free).
- Clean hand-off to the existing hit-react lifecycle (flinch → knockdown →
  get-up → back to locomotion).

### Out of scope for v1
- Jumping
- True dynamic balance / stumble recovery from physics alone.
- Runtime per-*section* strength API (asset-authored per-joint torque covers
  v1; the runtime knob is a known follow-up in the ragdoll plan).
- Slopes steeper than "gentle", stairs, moving platforms.
- Networked / second player (component is per-entity, so nothing blocks it
  later).

---

## Data model

`LocamotionControllerComponent` (all authored fields serialized + inspector):

| Field | Default | Meaning |
|---|---|---|
| `maxSpeed` | 2.5 | m/s at full stick deflection |
| `acceleration` | 10 | m/s² toward desired velocity |
| `deceleration` | 14 | m/s² when stick released (slightly snappier stop) |
| `turnRate` | 8 | slerp rate of hips yaw toward movement direction |
| `deadzone` | 0.2 | stick magnitude below which input is ignored |
| `strideLength` | 0.55 | meters per step at `maxSpeed` (phase rate = speed / stride) |
| `stepHeight` | 0.12 | swing-arc apex above ground |
| `dutyFactor` | 0.55 | fraction of the cycle each foot is planted (>0.5 = walk overlap) |
| `hipHeight` | — | standing pelvis clearance; **captured from the bind pose at start**, not authored |
| `bobAmplitude` | 0.03 | vertical hip bob synced to 2× gait phase |
| `leanGain` | 0.25 | radians of spine-target lean per m/s² of acceleration |
| `groundRayLength` | 1.0 | downward probe distance for root height + foot placement |

Runtime scratch (not serialized): `_velocity`, `_gaitPhase`, per-leg
`{plantedPos, swingStartPos, phaseOffset}`, `_desiredYaw`, cached bone indices
for the two leg chains, `_hipsBodyActive` (whether the controller currently
owns the root — false while a reaction owns it).

Input bindings (registered in `OnStart`, mirroring `Punch.h`'s
`Input::BindAxis` pattern): `MoveX` / `MoveY` → `GamepadAxis::LeftX/LeftY`,
plus a WASD fallback for deskbound testing.

> **Input conflict (must resolve in Phase 1):** `Punch.h` currently binds the
> **left stick** to arm steering (`ArmX`/`ArmY`) and the left trigger to grab.
> Movement takes the left stick; arm control moves to the **right stick**
> (grab stays on the trigger). This touches `PunchComponent` bindings only —
> the grab logic itself is unchanged.

---

## Per-frame flow (inside `OnUpdate`, priority 100 → before physics 200)

1. **Input → desired velocity.** Read stick, apply deadzone, rotate into
   camera-relative world space using the primary camera's yaw (position-only —
   the `CameraDirector`'s fixed pitch must not tilt movement into the ground).
   Desired speed = magnitude × `maxSpeed`.
2. **Integrate root.** Ease `_velocity` toward desired (accel/decel), slerp
   `_desiredYaw` toward the velocity heading, raycast down from the hips for
   ground height (`EntityIgnoreFilter` already excludes the whole ragdoll —
   the IK foot-ray fix covers this), and compute the new hips transform at
   `ground + hipHeight + bob`.
3. **Drive root.** `MoveKinematic` the hips body to that transform (fixed dt,
   never `SetPosition` — teleporting zeroes velocity and kills the drag on the
   chain). Skip entirely while `_hipsBodyActive == false`.
4. **Advance gait.** `_gaitPhase += horizontalSpeed / strideLength * dt`. Legs
   run at phase and phase + 0.5. Planted foot: target pinned at
   `plantedPos`. Swinging foot: target follows a raised arc from lift-off
   point to the next plant point (a step ahead along `_velocity`, raycast to
   ground); on plant, pin it. Speed ≈ 0 → phase freezes, both feet settle
   under the hips (idle).
5. **Write the pose.** Start from the bind/stand pose; solve each leg with the
   existing two-bone solver (`SolveTwoBone`, pole hints = animated-knee
   forward) against the foot targets; apply the acceleration lean to the spine
   target rotations; leave arms/head at bind — their motion comes entirely
   from weak motors + inertia. Write into `AnimatorComponent::pose`.
   The `AnimatorComponent` stays clipless (`clip = -1`); `UpdateAnimators`
   must not overwrite the procedural pose (gate on a flag or on the entity
   having a `LocamotionControllerComponent`).
6. Physics steps (priority 200): `SyncRagdollPowered` reads the pose and sets
   motor targets; the kinematic hips move; the body follows.

**Known latency:** IK ordering and the physics-before-animators main-loop
order mean motors may chase a one-frame-old pose. The existing
kinematic-follow already tolerates exactly this; at 60 Hz fixed-step it is
invisible. Do not restructure the main loop for it.

---

## The sloppiness preset (`.ragdoll` tuning, not code)

Per-joint `motorMaxTorque` in the asset (editable in the ragdoll inspector,
"Save .ragdoll"):

- **Hips→legs (hips, knees, ankles): strong** — they must track the gait pose.
- **Spine: medium** — lags a beat behind hip acceleration, recovers.
- **Shoulders, elbows, neck: weak** — trail, swing, and settle passively.

Global `RagdollComponent::strength` stays the master drunkenness slider
(`Physics::SetRagdollStrength`). Lower motor `frequency` = mushier tracking =
drunker; tune per section. Expect this phase to be almost entirely in-editor
knob-turning against feel.

---

## Reaction hand-off

The existing `RagReaction` machinery stays the owner of hit-react:

- **Flinch / knockdown:** `AutoTriggerRagdollImpacts` fires as today. On any
  reaction that takes the hips dynamic, the controller sets
  `_hipsBodyActive = false` (stops `MoveKinematic` + freezes gait) and lets
  the reaction run.
- **Resume:** when the rig returns to its rest mode, the controller re-captures
  the hips transform as the new path origin, re-plants both feet under the
  pelvis, and resumes. Rest mode for a controlled character is **Powered**
  (not Animated) — the controller is the pose source, so the old
  "hand back to Animated" endpoint becomes "hand back to
  Powered-with-locomotion". The get-up blend's known world-space pop
  (GH #25) matters less here since the target pose is the stand pose the
  get-up already drives toward.

---

## Build order (PRs)

1. **Root drive + input** — bindings (incl. the Punch right-stick migration),
   camera-relative velocity integration, kinematic hips follow, ground ray.
   *Verify:* rig in Powered mode dangles from moving hips and is dragged
   around the floor. Should already be funny.
2. **Gait generator** — phase, plant/swing targets, leg IK into the pose, idle
   settle. *Verify:* no foot slide at any stick deflection; stops cleanly.
3. **Sloppiness pass** — asset torque distribution, hip bob, accel lean,
   strength/frequency tuning session in the editor.
4. **Reaction integration** — `_hipsBodyActive` hand-off, resume-from-get-up,
   shove-while-walking test with the existing arm-swing rig.

Test scene: `Assets/Scenes/CharacterTest.scene` (CesiumMan + `.ragdoll` already
authored). CesiumMan's single walk clip is not used — which is the point.
