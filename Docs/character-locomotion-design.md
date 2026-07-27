# Character Locomotion Design

Last updated: 2026-07-27

The walk is a SIMBICON controller (Yin, Loken, van de Panne, SIGGRAPH 2007) running on
the powered ragdoll. The previous foothold-planner/IK gait was removed; what replaced it
is the paper's section 3 in full, plus a bounded IK terrain layer taken from section 8.

## Why the previous approach was replaced

The old controller was architecturally inverted against the paper in three ways:

| SIMBICON | Previous controller |
|---|---|
| Fixed-time / contact transitions; balance comes *from* stepping | `WeightShift` blocked stepping *until* balanced |
| Internal joint torques; the legs move the body | Pelvis velocity force, leash and COM-over-base forces moved the body |
| Target = joint angles, always reachable | Target = world foothold -> IK -> often unreachable -> pole search, bisection, clamping |

The third row is why the old file was 1400 lines: `ProjectSwingTarget`, the 19-candidate
pole search and the stride/lift bisection all existed to work around world-space foot
targets that the hip cone could not reach. Target angles are reachable by construction, so
that machinery is gone.

The second row is why the pelvis reached 0.8-1.1 m/s against a gait that could only
support 0.257 m/s. Section 7.3 also predicts the resulting fall directly: "when `cv` is
below the lower limit, the velocity of the character accumulates until it falls."

## Runtime pipeline

1. Read camera-relative input, update requested yaw, enter powered-ragdoll mode.
2. Advance the two-state FSM: state 0 for `state0Time`, state 1 until swing-foot contact.
3. Measure `d` (stance ankle -> COM) and `v` (COM velocity), split sagittal/coronal.
4. Swing hip target = state angle + `cd*d + cv*v`, independently in both planes.
5. Write swing hip and torso as WORLD-frame targets, knees/ankles parent-relative.
6. Clamp every written target to the authored Jolt swing/twist envelope.
7. Powered ragdoll motors chase the pose; physics owns every body.

`LocamotionControllerSystem` runs at priority 100, before physics consumes the pose.

## The body frame is MEASURED, not configured

`facingOffsetDeg` was guessed three times and was wrong every time. Sagittal/coronal are now
derived from the rig itself:

```text
right = normalize(horizontal(rightHipPos - leftHipPos))
fwd   = normalize(cross(worldUp, right))
```

The line between the hip joints is an unambiguous lateral axis on any rig. When the offset
was wrong by 90 deg, `right` was actually the body's FORWARD axis, so the swing rotation
lifted the leg sideways instead of stepping — "the legs kick up and don't go forward". The
envelope clamp used to mask this by folding the bad target into the cone; raw virtual torque
does not, which is why the symptom got louder rather than better when tau_A landed.

Two other places stopped depending on a guessed frame:

- **Hip targets are a femur DIRECTION**, not a composed frame. `worldHip` builds the desired
  thigh direction from the measured axes and swings the bind orientation onto it
  (`RotationBetween`), so no bind/entity/heading rotation can leak into the target.
- **Upright targets correct TILT ONLY** — take the body's current orientation and rotate its
  up-axis back to world up, with no yaw term. A yaw-locked pelvis is something the legs have
  to push against; there is now nothing to fight.

`facingOffsetDeg` still feeds `locomotionTargetRot` for the standing/non-SIMBICON path.
Turning during gait is unimplemented (paper section 8 does it by modulating desired facing and
letting the stance hip achieve it).

## Section 3.2 - the virtual torques (the part that was missing)

Three play-test rounds all ended identically: tilt 99-102 deg, `dSag` ~0.61, speed 0. Five
runs, three numbers. That is a switch flipping, not a control law degrading — and the switch
was `tiltDeg < locomotionFallenTilt` guarding an external pelvis drive.

The root drive was writing `SetAngularVelocity` and `SetLinearVelocity` on the pelvis every
substep. Those are not forces; they OVERWRITE velocity state. Consequences:

- Any angular momentum the stance leg imparts to the pelvis was destroyed 60x a second.
  The stance hip torque rotating the body over the stance foot IS SIMBICON's propulsion
  mechanism, and it cannot act through a velocity overwrite.
- The pelvis was yaw-locked to `_yaw + facingOffsetDeg`, so the legs fought a rotational
  constraint they could not move. (This is also why 0 vs 90 changed the startup state but
  never the outcome.)
- All of it inside the 55 deg gate, so the rig went rigid-rigid-rigid then instantly limp.

Now implemented as the paper specifies:

```text
tau_torso = worldPD(pelvis -> upright at heading)       virtual, never applied directly
tau_B     = worldPD(swing femur -> its world target)    applied to the swing hip
tau_A     = -tau_torso - tau_B                          applied to the stance hip
```

Each hip torque is applied with its equal-and-opposite reaction on the root, so the net
torque the pelvis sees is exactly `-tau_A - tau_B = tau_torso`. The stance hip is the free
variable; the residual is what pushes the body forward using only internal torque.

Both hip joint MOTORS are muted while this is active (`locomotionHipBones` in
`SyncRagdollPowered`) — a position motor on the same joint just fights the virtual PD. The
hips are therefore no longer written as pose targets at all; knees, ankles and torso still
are. Jolt's constraint cones still bound the joint physically, so `ClampToEnvelope` is no
longer needed on the hips.

Torques are computed script-side per frame but applied engine-side per SUBSTEP
(`DriveRagdollLocomotionRoot`), because `AddTorque` is cleared each step — applying from the
frame loop would land inconsistently at any frame rate other than 60.

The vertical support spring became a FORCE under SIMBICON rather than a velocity write (a
velocity write destroys the vertical momentum a gait needs) and is scalable to zero via
`supportScale` once the stance leg genuinely carries the body.

## Standing pose and the two blends

Two weights gate the pose write:

- `_poseBlend` — is the controller driving at all. Ramps up once powered, grounded and past
  `postPoweredGrace`; drops when tilt exceeds `locomotionFallenTilt`.
- `_gaitWeight` — 0 = standing, 1 = full FSM pose. Folded into the write weight, so at 0 the
  controller writes nothing and the motors hold the BIND pose.

An earlier attempt gave standing its own explicit symmetric pose. It made things worse — the
character toppled while merely standing (`gait=0.00`, tilt 8 -> 14.8 -> 28.3 -> 36.6 in one
second) where holding bind had been stable for 21 s. Reverted. If a tuned standing posture is
wanted later, verify it against the 21 s bind baseline before trusting it.

## The fallen-tilt gate is NOT on the FSM

`canGait` used to require `tiltDeg < locomotionFallenTilt`. That froze the FSM mid-stumble,
latching a stale one-legged pose at exactly the moment the balance law wanted its biggest
catch step — a recoverable trip became an unrecoverable face-plant. The FSM now keeps running
regardless of tilt; only `_poseBlend` fades, and the engine's own root drive still bails at
`locomotionFallenTilt` so a prone rig is never dragged.

`_timeSincePowered` initialises to -1 as a sentinel for the Animated->Powered flip. A scene
that authors the ragdoll as Powered (`"mode": 2`, which `CharacterTest.scene` does) never
runs that flip, so the sentinel doubled the grace period to 1.4 s. Clamped to 0 on first use.

## Section 3.1 - Finite state machine

Two states, mirrored by swapping which leg is stance:

```text
state 0  lift    fixed duration (state0Time, 0.30 s)
state 1  plant   exits on swing-foot contact, then legs swap
```

State 1 has a `minSwingTime` floor so the just-lifted foot cannot immediately re-trigger
contact, and a `maxSwingTime` ceiling so a missed step cannot hang the gait. Neither
exists in the paper; both are cheap guards, not balance logic.

Target poses are a **bias, not a goal**. The paper is explicit that they "are typically not
actually achieved" — in state 1 the swing leg is physically forward while its target is
backward, and that mismatch is what drives the foot down into contact. There is
deliberately no landing verification, no landing tolerance and no settled/landed
distinction; all of that contradicted the method and is gone.

## Section 3.2 - Torso and swing-hip control

The swing hip and torso track in the world frame; everything else is parent-relative.

A world-frame target becomes a joint target by composing against the **physical** parent
body rather than the animated one:

```text
pose[bone].rotation = conjugate(physicalParentWorldRot) * desiredWorldRot
```

`SyncRagdollPowered` reads `pose[bone].rotation` straight through as the joint's
child-relative-to-parent motor target, so this lands exactly. Reading the physical parent
is what `Physics::GetRagdollBoneRotation` was added for — the animated and physical
parents diverge precisely when balance feedback matters.

`tau_A = -tau_torso - tau_B` is **not yet implemented**. The stance hip is currently driven
to a near-vertical world target (`stanceHipDeg`) instead of being a free torque variable,
and the pelvis is still righted by the engine's upright spring. `locomotionUprightScale`
fades that spring out and `Physics::AddRagdollBoneTorque` is in place, so wiring the real
free-variable stance hip is the next step once the limit cycle holds. This is the paper's
one external-torque cheat; it is avoided in the paper for physical realizability on robots,
not because motion quality requires it.

## Section 3.3 - Balance feedback

```text
theta_d = theta_d0 + cd*d + cv*v
```

applied to the swing hip in both planes, where `d` is the horizontal stance-ankle-to-COM
distance and `v` is COM velocity. This is the entire balance mechanism and the only thing
deciding where the foot lands. There is no foothold planner.

`d` and `v` come from the engine's mass-weighted `_locomotionCOM` / `_locomotionCOMVel`.
`useHipCOMProxy` swaps in the hip position instead, which the paper endorses as a proxy in
both 2D and 3D; it is worth testing because the swing leg still pulls the true COM around.

Coronal `theta_d0` is `stanceWidthDeg`, applied outward per side. Section 8 notes that
symmetric lateral hip displacements are exactly how stance width is authored.

Stable ranges from section 7.3, wired into the inspector sliders as their bounds:

| Gain | Sagittal | Coronal |
|---|---|---|
| `cd` | -0.71 .. 1.4 | -1.29 .. 1.13 |
| `cv` | 0.03 .. 0.59 | -0.06 .. 0.48 |

Nominal is 0.5 / 0.2 in both planes.

## The IK layer

IK survives, demoted from decision-maker to correction layer. Section 8's extension
mechanism for terrain is a displacement on the target angles
(`delta_theta_hip = k * theta_slope`), which is exactly IK-shaped work, and it fills a hole
the paper admits to: "stairs can cause problems because the controllers cannot see an
upcoming step."

`TerrainKneeOffsetDeg` is deliberately **differential**: it solves the leg twice, once for
the nominal foot position and once for the ground-adjusted one, and contributes the
difference. On flat ground the two solves are identical and the contribution is exactly
zero, so the layer cannot destabilise the base gait. It is clamped to `ikMaxOffsetDeg`,
applies only to the swing leg during state 1, and is **off by default** (`ikTerrainEnabled`)
until the flat-ground limit cycle holds.

Blending an absolute IK pose against the FSM pose would put two controllers on the same
joint. Do not do that; keep this layer differential.

## What was salvaged

`ClampToEnvelope` is the constraint-space projection extracted from the old leg solver: it
expresses a local target in Jolt's authored swing/twist basis, scores elliptical swing and
twist violation, and pulls the target back to the nearest legal orientation. It is worth
keeping regardless of who generates the target — Table 1's angles plus a large `cd*d` term
can leave a 60-degree cone on a rig the paper never saw. Hinges skip it; Jolt clamps those
itself.

## Rig changes

The paper's method has physical preconditions the rig was violating. `CesiumMan.ragdoll`
was changed accordingly:

| Change | Before | After | Why |
|---|---|---|---|
| Leg mass share | 52% (37 of 71 kg) | 34% (24.2 of 70.4 kg) | Swing-hip feedback assumes placing the swing leg does not dominate trunk dynamics |
| Waist torque | 28 N-m | 250 N-m | Balance runs through the torso; the paper uses its highest gain (kp=1000) at the waist. Gravity alone on the trunk at 20 degrees of lean was already 22 N-m |
| Upper spine torque | 28 N-m | 200 N-m | Same |
| Hip torque | 160 N-m | 250 N-m | Binding constraint once the hip carries residual torque |
| Knee torque | 90 N-m | 150 N-m | Paper's 2D walk fails below 90 N-m |
| Right hip twist axis | `(0.980, -0.034, 0.194)` | `(0.786, -0.034, 0.617)` | Was not a mirror of the left (~48 deg apart in Z) |

The hip axes were the documented directional asymmetry, and it was a data bug rather than
something the solver should compensate for. The knees mirror as `(x, -y, -z)` and the hips'
`y` components already matched that convention exactly, so the left axis was mirrored onto
the right. Left is the side that previously walked better. If right-favouring runs turn out
worse, mirror the other direction instead — that is the whole fix.

Feet were already fine: `CreateFootSole` overrides the config capsule with an 18 x 5 x 28 cm
box, which is a real base of support.

## Engine additions

- `Physics::AddRagdollBoneTorque` — actuator for virtual PD controllers.
- `Physics::GetRagdollBoneRotation` / `GetRagdollBoneAngularVelocity` — world-frame targets
  must compose against the physical parent.
- `RagdollComponent::locomotionSimbicon` — set per-frame by the controller. While true,
  `DriveRagdollLocomotionRoot` suppresses the pelvis velocity force, the leash and the
  COM-over-base forces. The vertical support spring and upright spring stay.
- `RagdollComponent::locomotionUprightScale` — fades the upright spring out as the torso
  joint takes over.

The COM-over-base force was **not** deleted. It is the correct controller when there is no
swing leg to place — the paper says nothing about balancing in place, and section 7.2 notes
that non-stepping balance needs a different approach entirely. It stays scoped to standing.

## Verification status

**Standing: PLAY-VERIFIED 2026-07-27.** Held a fixed equilibrium for 21 s (velocity 0.000,
`dSag` 0.176 -> 0.170) and recovered from a 33 deg tilt excursion back to ~12 deg. The rig
rebalance did its job.

**Walking: partially verified, still falls.** Best run reached 4 alternating steps with the
FSM cycling, contact detection firing and knee commands tracking the state table to within
a few degrees — and tilt briefly dropped to 6.8 deg mid-gait, so the control law does work.
It has not yet held a limit cycle.

Bugs found and fixed from those runs, in order of how much they mattered:

1. **World-frame rest dropped the entity rotation.** `worldHip` built rest as
   `headingRot * BindModelRot(bone)`, omitting `entityRot`. CesiumMan is Z-up, so a 33 deg
   request reached the motor as 70 deg with 42 deg of spurious hip twist, on BOTH hips
   identically. Motor logs showed command 69.7 / actual 68.7 — the motors were tracking
   perfectly, the targets were wrong. Cross-check: the engine's own upright target is
   `locomotionTargetRot * rootBindRot`, and `rootBindRot` is exactly the omitted term.
2. **No standing pose** — see above.
3. **`_timeSincePowered` sentinel** — see above.
4. **Fallen-tilt gate froze the FSM** — see above.

Success is still a limit cycle: step length and period stable over 10+ steps, and the
character walking 60 s without falling. "Verified landings" is not a SIMBICON metric.

Diagnostic key for the debug line:

| Signal | Meaning |
|---|---|
| `t` stuck at 0.00 | FSM not running — `walking` is false, look at the gates, not the gains |
| `gait` / `pose` | the two blend weights; `gait` 0 with `pose` 1 = holding the stand |
| `steps` incrementing | contact transitions firing |
| `t` repeatedly hitting `maxSwingTime` | contact never detected — foot grounding, not balance |
| `hip` pinned at `swingHipLimitDeg` | feedback saturating; check tilt first, it is usually downstream of a fall |
| `tilt` past `locomotionFallenTilt` | engine support is off; below this line you are watching a corpse fall |
| `tau=(T,B,A)` | virtual torque magnitudes. A pinned at `maxVirtualTorque` means the stance hip is saturating — raise the limit or lower `torsoKp` |
| identical terminal numbers across runs | a gate is firing, not a control law failing |

## Known gaps

`tau_A` is now wired, so the remaining external assists are the vertical support force
(scalable to 0 via `supportScale`) and the COM-over-base force while standing. Neither acts
during gait except the vertical one.

## Tuning

Section 4: these controllers were authored interactively against a running sim, and only
three parameters per state really drive style — `state0Time`, swing hip and swing knee.
Start there. All FSM angles and both gain pairs are live inspector sliders.

Table 1's numbers were tuned on ODE at 5 ms against the paper's own rigs and will not
transfer verbatim; section 7.2 notes that even moving between the paper's 2D and 3D models
needed adjustment. Treat them as a starting point and the section 7.3 ranges as the search
box.

Sections 5 (mocap-derived controllers) and 6 (feedback error learning) are deliberately
skipped. FEL exists to reduce stiffness and torso oscillation on a controller that already
works.

## Known gaps

1. `tau_A = -tau_torso - tau_B` not wired; stance hip is a driven target and the pelvis
   upright spring is still an external torque.
2. Foot heading is uncontrolled — the ankle tracks a pitch angle only, not yaw.
3. Turning mid-swing is untested; the heading frame is sampled live, so a large direction
   change during state 1 moves the target the swing leg is chasing.
4. No airborne state.

## Relevant files

- `Sandbox/src/Scripts/LocamotionController.h`
- `Engine/src/Scene/Physics/PhysicsSystem.cpp` (`DriveRagdollLocomotionRoot`, `SyncRagdollPowered`)
- `Engine/include/Scene/Physics/RagdollComponent.h`
- `Engine/include/Scene/Physics/PhysicsAPI.h`
- `Assets/Models/CesiumMan.ragdoll`
- `Assets/Scenes/CharacterTest.scene`
