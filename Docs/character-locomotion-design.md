# Character Locomotion Design

Last updated: 2026-07-22

This is the continuation point for the Gang Beasts-inspired active-ragdoll walk.
The character can now produce convincing alternating steps and travel in the
requested direction for several seconds. It is not yet game-ready: startup,
directional symmetry, overspeed, turning and long-term balance still need work.

## Runtime pipeline

1. Read camera-relative movement input and update the requested yaw.
2. Enter powered-ragdoll mode and blend procedural gait IK in.
3. Enter `WeightShift` and bias physical COM support toward the future stance foot.
4. Start the swing only after that foot is grounded and COM remains inside its
   support region for a short stable interval.
5. Place the swing target ahead of the opposite planted foot.
6. Animate a smooth lifted sole trajectory to that target.
7. Convert the sole target into an ankle target and solve a hinge-aware hip/knee
   pose in world space.
8. Let powered ragdoll motors chase that pose while physics owns every body.
9. Verify the physical foot reached the target, or settle it at its actual grounded
   position after a timeout, before alternating legs.
10. Drive the pelvis toward a gait-limited velocity while applying upright and
   support-base balance.

`LocamotionControllerSystem` runs at priority 100, before physics consumes the
pose and locomotion command.

## Ownership

- `LocamotionController`: input, heading, gait phase, foot placement, swing arc,
  leg IK and safe root-velocity command.
- `DriveRagdollLocomotionRoot`: pelvis velocity force, vertical support, COM/base
  balance, upright control and the support leash.
- `SyncRagdollPowered`: converts animation-pose joints into physical motor targets.
- `RagReaction`: knockdown, limp and get-up.
- `Punch`: arms.

Physics is authoritative. Debug targets are intentions, not proof that a physical
foot reached them.

## CesiumMan leg chain

```text
Skeleton_torso_joint_1
  -> leg_joint_*_1  hip, swing-twist
  -> leg_joint_*_2  knee, hinge
  -> leg_joint_*_3  ankle stub, swing-twist
  -> leg_joint_*_5  sole/contact body
```

The analytic chain ends at `_3`. The controller measures the live `_5`-to-ground
clearance, converts the desired sole point into a foot-body point, and then offsets
that point to obtain the desired ankle position.

The controller currently writes `_1` and `_2` only. The ankle/foot remains passive,
so foot yaw is not aligned to movement yet.

## Hinge-aware leg solve

The original free two-bone IK generated knee rotations that were not rotations
about the physical hinge. Jolt discarded that component, leaving the leg straight.
The current solve is compatible with the ragdoll constraints:

1. Cache each knee's authored `twistAxisLocal` from the ragdoll configuration.
2. Clamp the ankle target to `maxReachFraction` of total leg length.
3. Compute knee bend with the law of cosines.
4. Apply that bend about the authored knee hinge axis.
5. Build the resulting bent chain in hip-local space.
6. Orient the hip so the exact chain reaches the ankle target and its bend plane
   points toward the movement direction.

Motor logs confirmed that knee `commandTwist` is now nonzero and `actualTwist`
generally follows it. This was the change that made visible stepping possible.

## Gait and foot placement

The gait state machine is:

```text
Idle -> WeightShift -> Swing -> DoubleSupport -> WeightShift
```

`Idle` clears any one-foot support bias and is the only entry into a fresh gait.
During `WeightShift`, both feet remain planted while physics pulls projected COM
toward a point slightly inboard of the future stance foot. A swing can begin only
after the stance foot is physically grounded and COM remains inside
`weightShiftRadius` for `weightShiftStableTime`. A failed transfer times out to
`DoubleSupport` and retries instead of lifting an unloaded foot.

Only one leg may swing at once. The current forward target is:

```text
stanceFootPlant + moveDirection * stepLength
```

Its lateral coordinate is placed on the swing leg's side of the stance foot using
twice `stanceHalfWidth`. This is important: the previous target advanced each foot
from its own old position, causing one foot to lead while the other merely caught
up. The stance-relative target makes every new footfall lead the other foot.

The sole follows smoothstep interpolation plus a sine lift. The target is grounded
once at swing start and is not retargeted in flight.

At the end of the timer:

- `landed`: the physical foot is within `landingTolerance` and grounded;
- `settled`: after a short grounded timeout, the commanded plant is moved to the
  physical foot's actual position so the gait cannot deadlock.

A settled step resets the consecutive verified-landing count, but alternating can
continue.

## Root drive and balance

Root drive now begins as soon as gait is requested. Waiting for two successful
landings allowed leg reaction forces to scoot the pelvis backward during startup.

The requested root speed is conservatively capped from current gait timing:

```text
stepLength / (2 * (stepDuration + weightShiftTime + doubleSupportTime))
```

With current values this is about `0.257 m/s`. `moveRampTime` blends into that
command. During weight transfer the velocity command is additionally scaled to
15 percent so support can move laterally without restoring the old no-propulsion
startup behavior. Propulsion is smoothly attenuated from 20 degrees of tilt and
reaches zero at 30 degrees; the normal fallen threshold remains separate.

This cap limits the target, not the physical velocity. Leg and balance impulses can
still accelerate the character to `0.8-1.1 m/s`, which is one cause of eventual
falls.

## Current tuning

`CharacterTest.scene` overrides are the values that matter for the current test.

| Field | Current value | Purpose |
|---|---:|---|
| `maxSpeed` | 1.0 m/s | Input request before gait limiting |
| `moveRampTime` | 0.75 s | Root command ramp |
| `facingOffsetDeg` | 0 deg | No model-facing correction |
| `stepLength` | 0.35 m | Distance each footfall leads the stance foot |
| `stanceHalfWidth` | 0.10 m | Half the desired foot separation |
| `stepDuration` | 0.40 s | Swing duration |
| `stepLift` | 0.10 m | Swing arc height |
| `doubleSupportTime` | 0.08 s | Delay between swings |
| `weightShiftTime` | 0.20 s | Minimum transfer time between ordinary steps |
| `firstWeightShiftTime` | 0.25 s | Longer startup transfer time |
| `weightShiftTimeout` | 0.40 s | Return to double support if transfer cannot verify |
| `weightShiftRadius` | 0.08 m | Maximum projected COM error before lifting |
| `weightShiftStableTime` | 0.05 s | Required continuous stable support interval |
| `weightShiftDriveScale` | 0.15 | Forward drive multiplier during transfer |
| `weightShiftInboard` | 0.20 | Support target inset from stance foot toward midpoint |
| `gaitBlendTime` | 0.20 s | Procedural pose blend |
| `maxReachFraction` | 0.95 | Prevents full leg extension |
| `swingPoseWeight` | 1.0 | Scene override for swing leg IK |
| `stancePoseWeight` | 0.45 | Driven support-leg IK weight |
| `landingTolerance` | 0.10 m | Physical landing radius |

`kneePoleDist`, capture-point triggers, tilt-triggered steps, hip-outside triggers,
mid-swing retargeting and the experimental stride fields were removed. Do not
reintroduce them until the base gait is stable and measured.

## Debug visualization and logs

Current debug markers:

- green sphere: planted sole;
- white sphere: current point on the commanded swing arc;
- orange sphere: final swing target;
- blue sphere: clamped ankle IK target;
- red sphere and line: physical ankle and its error from the blue target.
- cyan sphere: active weight-shift support target;
- yellow sphere: physical projected COM.

Important log fields:

- `aheadOfStance`: target lead over the opposite planted foot; should be about
  `+0.35` for a straight step;
- `swingTravel`: target distance from that swing foot's physical start; normally
  larger than `stepLength` once alternating correctly;
- `actualAlongMove`: physical foot travel along its captured swing direction;
- `target`: tilt-scaled, gait-capped pelvis speed;
- `along`: signed physical pelvis velocity along current input; negative is real
  backward movement;
- `steps`: consecutive verified landings, reset by a settled step or fall;
- `[LocoMotor] command/actual/error`: total motor rotation;
- `[LocoMotor] commandTwist/actualTwist`: rotation usable by a hinge.

## Verified fixes

- Camera-relative targets and body facing now use the same zero-degree convention.
- Swing targets advance in the requested direction; target direction is not negated.
- A stance leg remains procedurally driven instead of going passive during swing.
- Knee motor commands use the physical hinge axis.
- Hip and knee targets describe the same bent chain.
- Real physical foot positions, rather than only IK targets, determine landing.
- A failed landing no longer freezes the gait permanently.
- Swing targets are stance-relative, producing true alternating lead steps.
- Root propulsion starts with gait and is capped/tilt-faded.
- The Debug configuration builds successfully with the current implementation.

Good runs now show repeated alternating landings with roughly `0.5-0.75 m` physical
swing travel. A recent leftward run completed eight verified steps before failing.

## Known problems

### 1. Startup weight-shift tuning

The explicit `WeightShift` phase is implemented. Physics now exposes mass-weighted
COM and per-foot grounding, and can blend normal support-base balance toward a
controller-provided support target. The controller waits for verified, stable
support before lifting and safely retries after a timeout.

The phase still needs four-direction runtime testing. Watch `comError`, stance-foot
grounding, timeout frequency, signed startup velocity and whether the first physical
foot now travels toward its target. Tune the support radius/inset and balance force
from those measurements rather than shortening the readiness gate blindly.

### 2. Directional asymmetry

Up/left generally works better than down/right. CesiumMan's authored hip twist axes
are strongly asymmetric:

```text
left hip:  ( 0.786,  0.034, -0.617)
right hip: ( 0.980, -0.034,  0.194)
```

The hips also have unequal plane/normal cone limits. A movement direction can
therefore map into a permissive part of one cone and a restrictive part of the
other. The observation that angled feet/legs sometimes work better supports this
constraint-space diagnosis. Passive foot yaw and contact friction may contribute.

Do not blindly mirror the axes. First draw/log the axes in world space at bind and
measure swing-hip command error for four controlled cardinal runs.

### 3. Eventual overspeed and forward fall

During good walking, actual pelvis speed has frequently reached `0.8-1.1 m/s`,
well above the previous `0.365 m/s` target and the new weight-shift-aware
`0.257 m/s` target. Eventually a swing foot misses, support disappears and tilt
rises rapidly. The current controller fades future propulsion but cannot cancel
existing momentum or place a recovery step.

Likely fixes after direction/startup work:

- brake signed overspeed explicitly;
- use actual pelvis velocity in foot placement;
- add capture-point placement or limited mid-swing prediction;
- inhibit a new swing when stance support or COM position is unsafe;
- later add tilt-triggered recovery stepping.

### 4. Foot heading is uncontrolled

The passive ankle/foot does not face the travel direction. Add swing-only ankle/foot
yaw later, blending to a stable planted orientation before contact. Driving planted
foot yaw directly would inject unwanted ground torque.

### 5. Turning during a swing

Foot position and `swingDirection` are captured at swing start, while the knee bend
plane currently follows live input. Large direction changes can make the target,
solver plane and landing measurement disagree. Decide whether a swing should retain
its captured direction or support limited retargeting.

## Next-session plan

1. Commit this checkpoint before further experiments.
2. Add diagnostics only: world hip axes/cones, per-foot grounding, COM/hip distance
   from the stance foot, forward/lateral pelvis velocity and tipping angular velocity.
3. From a fresh scene start, hold exactly one direction for each test: up, down,
   left and right. Do not turn during a run. Record verified/settled steps and peak
   hip motor error per leg.
4. Correct the rig axes or make the hip solve cone-aware based on that evidence.
5. Tune the explicit weight-transfer phase from the four-direction measurements.
6. Add overspeed braking and velocity-aware foot placement.
7. Add swing-foot heading only after the four-direction gait is stable.
8. Reintroduce capture/tilt recovery last.

## Relevant files

- `Sandbox/src/Scripts/LocamotionController.h`
- `Engine/include/Scene/Physics/RagdollComponent.h`
- `Engine/src/Scene/Physics/PhysicsSystem.cpp`
- `Assets/Models/CesiumMan.ragdoll`
- `Assets/Scenes/CharacterTest.scene`
- `logs/loco_debug.log`
