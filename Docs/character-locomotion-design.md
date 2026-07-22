# Character Locomotion Design

The controller is an intentionally small active-ragdoll gait. Physics owns the
pelvis; the script chooses alternating foot placements and supplies leg motor poses.

## Runtime pipeline

1. Read camera-relative movement input.
2. Write a target heading to `RagdollComponent`.
3. Start one fixed swing step before enabling horizontal pelvis movement.
4. Alternate left and right steps with a short double-support interval.
5. Convert the desired sole point into a foot-body point, then an ankle target.
6. Drive only the swinging hip-knee-ankle chain.
7. Confirm the real sole reached its target before accepting the landing.
8. Ramp pelvis target velocity after both feet complete an initial placement.

`LocamotionControllerSystem` runs at priority 100, before physics consumes the
pose and root targets.

## Ownership

- `LocamotionController`: input, heading, step timing, foot targets, leg IK.
- `DriveRagdollLocomotionRoot`: pelvis support, velocity force, COM balance,
  upright control and the lean leash.
- `SyncRagdollPowered`: joint motor targets.
- `RagReaction`: knockdown, limp and get-up.
- `Punch`: arms.

There is no second controller-side COM correction.

## CesiumMan leg chain

```text
Skeleton_torso_joint_1
  -> leg_joint_*_1  hip, swing-twist
  -> leg_joint_*_2  knee, hinge
  -> leg_joint_*_3  ankle, swing-twist
  -> leg_joint_*_5  sole/contact body
```

The analytic two-bone chain ends at `_3`, not `_5`. Each leg measures the live
height between `_5`'s body origin and the ground. A desired sole point is raised by
that clearance to recover the desired foot-body position, then converted to an
ankle target. The `_5` foot motor retains its authored pose.

The solver writes only `_1` and `_2`:

- Hip: world-space swing converted to a parent-relative pose, with bind twist held.
- Knee: positive bend about the ragdoll's authored hinge axis.

## Gait

Only one leg may swing. A step target is:

```text
hips + moveDirection * stepLength + right * sideOffset
```

`sideOffset` is negative for the left leg and positive for the right leg, keeping
a useful support width. The target is raycast onto the ground once when the swing
starts and is not retargeted in flight.

The sole follows a smooth interpolation plus a sine lift arc. Completing the timer
only enters the landing hold; the state changes to planted after the real sole is
within `landingTolerance` of the target. Until then, the other leg and root wait.

## Movement sequencing

The first two steps run with zero requested horizontal pelvis velocity. Once both
feet have established the new support area, pelvis velocity ramps toward the input
speed over `moveRampTime`. Falling immediately cancels gait IK and root drive.

When input stops, gait IK and root velocity blend out. Idle therefore returns to the
authored standing pose instead of continuously replacing it with procedural IK.

## Current tunables

| Field | Default | Purpose |
|---|---:|---|
| `maxSpeed` | 1.0 m/s | Full-input speed |
| `moveRampTime` | 0.75 s | Root velocity ramp |
| `stepLength` | 0.25 m | Forward foot placement |
| `stanceHalfWidth` | 0.10 m | Per-side lateral offset |
| `stepDuration` | 0.40 s | Swing duration |
| `stepLift` | 0.10 m | Swing height |
| `doubleSupportTime` | 0.08 s | Delay between steps |
| `gaitBlendTime` | 0.20 s | IK blend in/out |
| `kneePoleDist` | 0.50 m | Forward knee bias |
| `maxReachFraction` | 0.95 | Prevents full extension |
| `swingPoseWeight` | 0.35 | Limits swing motor disturbance |
| `landingTolerance` | 0.10 m | Required real-sole proximity |

## Deferred recovery features

Capture-point stepping, shove recovery, tilt-triggered steps, hip-outside triggers,
and mid-swing retargeting are intentionally absent. Reintroduce recovery only after
ordinary alternating walking is stable and measurable.

## Validation order

1. Idle remains stable with gait weight zero.
2. One swing leg completes a visible 0.25 m step while the stance pose is untouched.
3. First landing occurs before horizontal drive rises.
4. Repeated steps retain stance width and advance at 0.4 m/s.
5. Increase toward 1.0 m/s.
6. Add capture/tilt recovery behavior if needed.
