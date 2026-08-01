# Character Locomotion Design

Last updated: 2026-07-28

## Goal

Build a fully procedural, physically interactive active-ragdoll character with a loose,
responsive style similar to Gang Beasts. Physics owns the body at all times: pushes,
collisions, contacts, momentum, and falls must remain real. No animation clips are sampled
and no transform is teleported to make a step succeed.

SIMBICON remains the conceptual basis for alternating stance/swing states, contact-driven
transitions, balance feedback, and velocity-dependent foot placement. We are not attempting
to reproduce the paper's controller literally on a different rig and physics solver.

The immediate objective is not continuous walking. It is to prove the physical foundations
one at a time, in the order defined below.

## Control ownership

The controller must have one owner for each physical task:

| Task | Owner |
|---|---|
| Rigid-body motion and collision response | Jolt |
| Gait phase and support-leg selection | Locomotion FSM |
| Desired swing-foot trajectory | Footstep planner |
| Swing-leg joint pose | Two-bone IK -> powered-ragdoll motors |
| Torso attitude and broad COM balance | Root/torso force controller |
| Stance-foot world position | One stance-contact controller only |
| Fall/recovery mode | Latched state with hysteresis |

The following combinations are forbidden:

- A planted leg must not use world-space IK and an external foot-lock servo to correct the
  same translation simultaneously.
- The pelvis controller and stance controller must not independently pull the body toward
  different world-space targets.
- A gait target must not be rebuilt every frame from the error it caused on the previous
  frame. Step and plant targets are latched for the phase.
- Control must not repeatedly turn off and on near one tilt threshold. Falling and recovery
  require separate thresholds and a minimum dwell time.
- Increasing force limits is not a fix for two saturated controllers fighting each other.

Joint motors may hold a local support pose while a foot is planted, but only one controller
may own the foot's world-space translation. Any competing plant correction must be disabled
during that test.

## Validation rules

Only one new capability is enabled per test. A test must pass repeatedly before work starts
on the next one. If a later test fails, return to the first failed invariant instead of
adding a compensating controller downstream.

The numerical limits below are initial engineering acceptance criteria, not permanent style
settings. They may be revised once if the rig establishes a repeatable physical baseline;
they must not be relaxed run-by-run to call an unstable result successful.

Every run should record:

- gait phase and phase transition reason;
- body tilt and angular velocity;
- COM position/velocity and active support side;
- left/right contact state, foot velocity, and plant error;
- requested and applied balance/lock forces, including saturation;
- requested and measured hip, knee, and ankle angles;
- fall/recovery state transitions.

Visual debug drawing should show the COM, support point/area, planned foothold, swing
trajectory, planted-foot target, knee pole, and applied balance forces. Logs diagnose control
state; a video or debug draw is required to diagnose twisted geometry.

## Test 1 - Standing equilibrium and disturbance

**Purpose:** prove that the rig, contacts, joint motors, and torso balance can maintain a
coherent powered-ragdoll stand before either leg is asked to step.

**Enabled:** bind/standing motor pose, physical contacts, torso/root balance.

**Disabled:** gait FSM advancement, swing IK, footstep planning, world-space foot locks.

**Procedure:** stand for 30 seconds, then apply the same small lateral impulse from each
side in separate runs.

**Runtime controls:** select `Test 1 - standing/shove` under `Validation`. WASD is ignored
while it is active. After the inspector reports `baseline=ready`, press F6 for an impulse
toward the character's left or F7 for an impulse toward its right. The 25 N*s default is
applied to the configured torso body at its center of mass; it changes momentum without
teleporting any body or bypassing Jolt contacts.

**Pass:**

- no fall or control-mode chatter;
- both feet remain in physical contact during undisturbed standing;
- undisturbed foot drift remains below 4 cm;
- after the impulse, tilt returns below 15 degrees and horizontal speed below 0.15 m/s
  within 2 seconds;
- no actuator remains saturated for more than 0.25 seconds.

**Status: PASSED 2026-07-27.** In the clean bidirectional run, both feet retained contact,
foot drift finished at 1.6/2.6 cm, foot-lock authority remained exactly zero, peak tilt was
16.2 degrees left and 13.6 degrees right, and recovery completed in 0.37-0.40 seconds.

### Grounded motor-isolation diagnostic

Test 1's coarse pass criteria did not measure visible high-frequency sole chatter. Before
continuing Test 3, select `Diagnostic - grounded motor isolation`. The diagnostic waits for the
normal settled standing condition, captures and freezes both legs' local motor pose, and records
foot translation, rotation, linear/angular speed, contact transitions, exact oriented-box sole
minimum Y, contact-point Y, and Jolt's signed penetration depth.

Use F6 to advance and F7 to move backward through these four combinations; let each stage run
for at least five seconds without other input. F8 discards the capture and waits for a fresh
settled baseline.

0. fixed soles, ankle motors enabled, explicit COM support enabled;
1. fixed soles, ankle motors disabled, explicit COM support enabled;
2. fixed soles, ankle motors enabled, explicit COM support disabled;
3. fixed soles, ankle motors disabled, explicit COM support disabled.

`[LocoGround]` reports current/peak linear and angular speed, displacement and rotation from the
start of the stage, contact-transition counts, `soleY`, `contactY`, and `penetration`. Positive
penetration is real overlap; a negative value is a speculative Jolt contact before the shapes
touch. The first stage whose speed and contact-transition peaks collapse identifies the feedback
loop. Compare 0 with 1 to measure the ankle motors while COM support is held constant, 0 with 2
to measure COM support while ankle motors remain enabled, and 2 with 3 to recheck the ankle effect
without COM support. If none are quiet while penetration oscillates materially, investigate the
contact solver/shape pair rather than IK.

**2026-07-27 finding and topology correction:** the original run retained both ground contacts
with zero transitions, disabling the terminal foot motors did not remove right-foot chatter, and
disabling both terminal and ankle motors made motion substantially worse (including 185/104 deg/s
angular spikes). The rig had two consecutive loose swing/twist joints below each shin. The
`leg_joint_*_5` terminal sole is now fixed to `leg_joint_*_3` with no motor; `leg_joint_*_3` is the
only articulated ankle, limited to a 30-degree swing cone and +/-10 degrees of twist.

**Status: PASSED after topology correction.** In the powered stage, both contacts remained active
with zero transitions for 17 seconds. Left/right sole drift was 0.5/1.2 mm, rotation stayed below
0.83/0.77 degrees, and peak angular speed fell to 15.8/13.5 deg/s. The reported backward slide was
isolated to the deliberate support-removal stages: disabling COM support produced 4.0 cm right-foot
drift, while disabling both COM support and ankle motors allowed roughly 0.8 m of passive drift.

## Test 2 - Controlled weight shift

**Purpose:** prove that the character can deliberately load either support leg without
lifting a foot or sliding the base across the floor.

**Enabled:** everything from Test 1 plus a slowly moving lateral COM target.

**Disabled:** leg lift, swing trajectory, step transitions.

**Procedure:** shift left, settle, return to center, shift right, and settle.

**Runtime controls:** select `Test 2 - weight shift`, wait for `baseline=ready`, press F6 to
request left support, F8 to return to the captured center, and F7 to request right support.
Each target is latched in the original support frame and ramps over `test2ShiftTime`.

**Pass:**

- COM projection moves toward the selected support side;
- both feet retain contact and each drifts less than 4 cm;
- tilt remains below 25 degrees;
- the controller settles on both sides without force or motor saturation;
- left and right results are directionally mirrored rather than using separate fixes.

**Status: PASSED 2026-07-27.** The COM moved 4.1 cm left and 5.8 cm right, tracking its
latched targets within 2-5 mm before returning to zero at center. Both contacts remained,
maximum foot drift was 1.2 cm, peak tilt was 12.5 degrees, support never saturated, and
foot-lock authority remained zero.

## Test 3 - Single-leg lift and hold

**Purpose:** isolate support-leg balance from swing-leg IK.

**Enabled:** validated weight shift, a brief internal toe-off actuator, then IK for the swing
leg only.

**Disabled:** forward swing, touchdown transition, support transfer, second step.

**Procedure:** load one leg, lift the other foot 8-12 cm, hold for 0.5 seconds, and lower it
to its original location. Repeat with the legs swapped.

**Runtime controls:** `Test 3 - single-leg lift` is the default validation mode. Wait for
`baseline=ready`. F6 runs an automatic left-support/right-lift sequence; F7 mirrors it. Each
sequence shifts the COM close to the chosen stance sole, captures the swing leg's live position
and knee plane, unloads the sticky swing contact until the collision manifold is absent for at
least 0.05 seconds with 4-6 cm of measured clearance, then hands exclusive control to swing-leg
IK. It lifts, holds for 0.5 seconds, lowers to the captured point, confirms settled contact, and
recenters. Do not press both keys or begin a second sequence until `phase=0`.

`[LocoTest3]` reports phase, COM target/tracking, both contacts, lift clearance, swing-target
error, knee-pole dot product, maximum stance drift, tilt, support-force saturation, foot-lock
weights, and the lift-assist bone/target/applied-force/cap. Support must remain unsaturated and
locks must remain zero. The one-sided lift spring remains active through TAKEOFF, LIFT, and HOLD;
its target follows the continuous foot trajectory and its force naturally reaches zero when the
foot reaches that target. It fades with the trajectory during LOWER and is off for CONTACT and
RECENTER. A takeoff that cannot prove a real contact exit within its timeout aborts and recenters.
Phases 3-5 also abort immediately if the swing contact returns, stance drift exceeds 4 cm, or
tilt reaches 30 degrees, instead of continuing into a known fall.

`[LocoKnee]` independently reports, in millimeters, upper/lower rigid-body segment stretch and
the thigh/calf capsule-tip gaps at the knee/ankle for both legs. Near-zero values prove the
physics bodies and collider geometry remain connected; a visible mesh seam with those values
near zero belongs to skinning/readback rather than the ragdoll constraint topology.

**Pass:**

- the correct foot lifts without first kicking backward or sideways;
- stance contact is continuous and stance-foot drift remains below 4 cm;
- the knee stays on its captured anatomical bend side with no branch flip;
- tilt remains below 30 degrees;
- the character returns to the Test 1 standing state after lowering the foot.

**Status: PASSED 2026-07-27.** Three clean sequences completed TAKEOFF, LIFT, HOLD, LOWER,
CONTACT, and RECENTER without an abort, including both support directions. Swing clearance
reached 9.0-9.3 cm, maximum stance-foot drift was 1 mm, peak tilt was 7.7-8.9 degrees, and the
knee-pole dot product remained +1.00 with no branch flip. Support remained unsaturated, both
foot-lock weights stayed zero, and every sequence returned to stable double contact. Correcting
the right-thigh capsule axis also reduced its settled knee collider gap from 123.9 mm to about
0.01 mm.

## Test 4 - One forward step and settle

**Purpose:** validate a complete swing trajectory and touchdown without support transfer.

**Enabled:** validated single support plus one latched forward foothold and continuous
swing-foot trajectory.

**Disabled:** automatic leg alternation and second step.

**Procedure:** move one foot 15-25 cm forward, touch down, and hold double support for at
least 0.3 seconds.

**Runtime controls:** select `Test 4 - one forward step`, wait for `baseline=ready`, then
press F6 for left support/right step or F7 for the mirrored run. The controller reuses Test
3's validated weight shift and contact-release phases, latches a 22.5 cm forward foothold once,
and follows a zero-end-velocity swing arc to a 5 cm hover target. The foot must remain within
2 cm horizontally for 0.1 seconds during a bounded arrival phase before a separate smooth
vertical descent. After a valid touchdown it continues solving landing IK until both contacts
remain present and swing-foot speed stays at or below 0.05 m/s for 0.1 seconds. It then captures
the achieved local leg pose and holds the original support side; acquisition has a bounded
0.6-second timeout so a moving plant cannot stall the test indefinitely.
support transfer, automatic alternation,
and all world-space foot locks remain disabled. F8 discards the current capture and waits for
a new settled baseline; it never moves a body, so reload the scene before the mirrored run when
an identical starting stance is required.

If the requested foothold exceeds the captured swing-leg reach, Test 4 shortens only its
horizontal displacement and raycasts the shortened location again. The ground-derived foot
height is never moved toward the hip by the reach clamp. The planner may use up to 99% of the
measured anatomical thigh-plus-shin length, capped below 99.5% to retain an anti-singularity
margin; a captured physical pose already beyond that cap remains valid but receives no further
extension.

`[LocoTest4]` reports the latched foothold, desired and measured foot positions, trajectory
progress, clearance, forward travel, contact edge, touchdown normal, API and finite-difference
vertical speeds, forward/lateral/vertical target-error components, sole-up direction, contact
point in foot-local coordinates, both plant drifts, initial/peak/final tilt, support and lift
force, lock weights/forces, and direct Jolt motor applied-torque/limit saturation readback.
`TOUCHDOWN_CHECK` names every passing and failing predicate. The touchdown gate requires an
upward normal, at most 0.25 m/s vertical speed, and no more than 4 cm horizontal target error;
completion additionally requires at least 15 cm of retained forward travel.

**Pass:**

- the foot clears the floor, travels forward, and descends without a discontinuity;
- touchdown is a real upward-facing contact with low vertical foot speed;
- neither plant drifts more than 4 cm after settling;
- tilt stays below 30 degrees and finishes within 10 degrees of its pre-step value;
- no lock force or motor remains saturated after the settling interval.

**Status: PASSED 2026-07-28.** Both mirrored runs completed without an abort. F6 stepped the right foot
15.1 cm with 0.8 cm final plant drift; F7 stepped the left foot 20.5 cm with 0.2 cm final plant
drift. Both touchdown gates accepted a real upward contact below the vertical-speed limit, and
both runs finished within the tilt, drift, lock-force, and motor-saturation limits.

## Test 5 - Support transfer

**Purpose:** prove that weight can move onto the newly planted foot without initiating the
next swing or fighting two plant controllers.

**Enabled:** validated first step plus a time-bounded COM transfer to the new support side.

**Disabled:** opposite-leg lift and automatic alternation.

**Procedure:** after Test 4 touchdown, transfer support to the new foot and hold for 0.5
seconds.

**Runtime controls:** select `Test 5 - support transfer`, wait for `baseline=ready`, then
press F6 for left support/right step and transfer or F7 for the mirror. Test 5 runs the
validated Test 4 sequence unchanged. After its stable plant capture, the support target follows
a smooth one-second curve from the old stance-side target to 92% of the measured distance from
the current COM to the new plant. The target velocity is supplied to the COM controller, both
foot contacts remain required, and the captured landing pose is held throughout. Opposite-foot
lift, foot locks, and automatic role alternation remain disabled. F8 recaptures the baseline.
Because F8 does not move any body back to the original stance, reload the scene before the
mirrored run; recapturing after a completed step creates a staggered-foot condition belonging
to Test 6 rather than an isolated Test 5 baseline.

`[LocoTest5]` reports transfer progress, requested and measured COM position error, horizontal
COM speed, COM distance from the old and new support feet, both contacts, both plant drifts,
tilt, support force/saturation, and motor saturation. A per-foot normal impulse is not currently
available, so old-leg unloading is tested geometrically: the COM must enter the new sole's
support region and be at least 2 cm closer to it than to the old sole. Test 6 will provide the
stronger functional proof by lifting that old support foot for the mirrored second step.

**Pass:**

- COM projection moves into the new support region;
- the old leg unloads without losing control of the body;
- both contacts remain stable during the transfer;
- plant drift stays below 4 cm and tilt below 30 degrees;
- the final state is stable enough to perform Test 3 with the opposite leg.

**Status: PASSED 2026-07-28.** Fresh-scene runs completed in both directions after adding the
safe anatomical reach reserve. F6 retained an 18.0 cm step and transferred within 0.5 cm of its
COM target, with 2.6/3.0 cm maximum plant drift and 15.7 degrees peak tilt. F7 retained 20.4 cm,
finished within 0.5 cm of its COM target, and held with 2.3/0.2 cm maximum drift and 16.4 degrees
peak tilt. Both runs held the COM 1.5-1.9 cm from the new support foot for at least 0.5 seconds,
kept both contacts, and reported no support or motor saturation.

## Test 6 - Second step

**Purpose:** validate phase handoff and expose accumulated error before continuous walking
can hide it.

**Enabled:** one automatic leg swap after a successful Test 5 transfer.

**Disabled:** a third step.

**Procedure:** complete one step, transfer support, complete the mirrored second step, then
settle in double support.

**Runtime controls:** select `Test 6 - second step`, wait for `baseline=ready`, then press F6
for left support/right-first or F7 for the mirrored sequence. One press runs both steps. After
the first validated Test 5 transfer and 0.5-second hold, the controller requires an additional
0.25 seconds of both contacts, low foot/COM speed, sub-30-degree tilt, and unsaturated support.
It then captures both legs' current physical local poses, preserves the original world
right/forward basis, swaps support roles exactly once, and starts the second weight shift. The
second step and transfer use the same physical touchdown and support gates as the first. A third
step is categorically disabled; F8 discards the sequence and recaptures the current pose but does
not restore the scene.

`[LocoTest6]` reports `STEP1_COMPLETE`, the measured `ROLE_SWAP`, both transfer phases,
`COMPLETE_CHECK`, and final `COMPLETE`. It records each step's retained forward travel, maximum
plant drift, peak tilt, and maximum motor-saturation ratio, plus left/right contact-transition
counts. Completion requires exactly two steps, exactly one airborne/contact cycle per foot,
final tilt within 10 degrees of the original baseline, no more than 1.5 cm drift growth on the
second step, and no more than 0.05 motor-ratio growth.

**Pass:**

- exactly two alternating steps complete without state resets or fall-gate chatter;
- the second swing begins from a settled support state rather than a timeout;
- no visible jitter, IK branch change, or prolonged saturation occurs;
- tilt after the second step is within 10 degrees of the initial standing tilt;
- foot drift and motor tracking error do not grow from the first step to the second.

**Status: PASSED 2026-07-28.** Fresh-scene F6 and F7 runs each completed exactly two alternating
steps and one measured role swap without an abort. F6 retained 16.4/20.0 cm and F7 retained
20.2/20.8 cm on the first/second steps. Second-step plant drift stayed at 0.3/0.7 cm, drift and
motor-ratio growth remained within tolerance, both feet produced exactly one airborne/contact
cycle, and final tilt remained within 10 degrees of the initial standing tilt.

## Test 7 - Continuous gait

**Status: COMPLETE (2026-07-29).**

**Purpose:** turn a validated two-step sequence into a repeatable limit cycle.

**Enabled:** repeated alternation and desired-speed foot-placement feedback.

**Completion procedure:** complete 10 steps at constant low speed from both mirrored starting
supports and return cleanly to standing. The optional 60-second endurance mode remains a regression
and tuning tool; it no longer blocks extraction of the validated gait into the gameplay controller.
Turning, slopes, uneven terrain, and recovery remain separate later milestones.

**Runtime controls:** select `Test 7 - continuous gait`, wait for `baseline=ready`, then press
F6 for left support/right-first or F7 for the mirror. The default run stops automatically after
10 completed steps. Enable `Test 7 60-Second Run` in the inspector for the endurance run; F8 may
request an early controlled stop, which finishes the active step before recentering.

Test 7 repeats Test 6's measured role-swap gate without resetting the physical pose. After each
settled transfer it measures the step period and COM advance, then applies bounded desired-speed
feedback to the next support-to-support advance. Unlike the isolated Test 4-6 footholds, every
Test 7 target is placed ahead of the current stance foot while retaining the swing foot's lateral
lane. This prevents alternating catch-up steps and makes the feedback operate on actual body
advance. The 10 cm nominal advance is bounded to 6-14 cm, and the default 0.020 m/s speed target
matches the deliberately slow validation cadence; neither is the eventual gameplay walking
speed. The reach clamp must retain the minimum advance plus a target-tracking reserve before
takeoff. Admission subtracts the filtered loss measured from prior physical landings instead of
using an unrelated fixed margin, and plant acquisition verifies the same retained support advance
before capturing the contact pose. Before each takeoff, the weight-shift gate requires the forward
COM error to be below 1.5 cm, the lateral error below 1 cm, and forward COM speed below 1 cm/s.
The forward position band deliberately accommodates the nonzero equilibrium error of the physical
1.5 Hz PD support spring. A shift that remains blocked for 4.5 seconds with the default settings
logs `WEIGHT_SHIFT_TIMEOUT` with every readiness predicate and aborts instead of waiting silently.
During the walking sequence the dynamic-root height target
ramps into a small 1.8 cm reach crouch and the Test 7 planner limits nominal usable leg reach to
97.5% of the anatomical chain (without shortening an already longer captured pose). This preserves
knee bend so small radial motor lag is not magnified into centimetres of horizontal landing error.
The initial settled orientation of each sole is also kept as a separate flat-ground reference.
Per-step physical state still seeds each new swing trajectory, but settled motor lag is not copied
back into the support commands at plant acquisition or role swap. Swing blends from the measured
takeoff orientation to the invariant sole reference over 0.35 seconds. A bounded horizontal
task-space correction adds 45% of measured sole-position error plus 80 ms of target-velocity lead
before the analytic ankle solve, capped at 4.5 cm. The correction therefore compensates joint-servo
latency while converging back to the admitted foothold as the physical sole catches up.
The velocity estimate is taken from the previous frame's nominal foot target before that history
is advanced, so the configured 80 ms lead remains active during the moving swing rather than
collapsing to zero. Late swing also constructs the local ankle target under the measured knee
orientation and requires the physical sole to come within 10 degrees of its invariant ground
orientation before descent. A toe/heel recontact at hover raises the desired sole center by 3 cm
and receives a bounded 0.35 second angular-convergence window; it is never accepted as touchdown.
After the transfer settles, Test 7 uses the double-support interval to recenter before the next
role swap. It captures both the current support target and the midpoint of the two physical soles
once at inter-step entry. A 0.35-second smoothstep then moves between those immutable endpoints,
and its analytic derivative supplies the support-target velocity. The target is never rebuilt from
the vibrating feet each frame, so contact noise cannot become a velocity command and feed back
through the COM damping term. There is no separate posture-recovery torque and no phase-specific
hip weakening. Test 7 selects a single physical upright spring for its entire lifetime, including
baseline settling. A world-to-root SixDOF constraint leaves all translation free. Two force-limited
position motors drive pitch/roll toward the bind-upright frame, while the same constraint uses a
lower-authority yaw motor to preserve the heading captured when F6/F7 starts. The heading target is
never recaptured from the rotated pelvis after a landing. Physical stiffness and damping are sized
for the whole character rather than normalized from the pelvis body's inertia.
The initial 700 N*m/rad stiffness is above the approximately 450 N*m/rad small-angle gravity slope
of the 70.4 kg test ragdoll at standing hip height, while the 250 N*m per-axis cap bounds recovery.
Heading begins at 180 N*m/rad stiffness, 50 N*m*s/rad damping, and an 80 N*m cap so stance-contact
yaw impulses are rejected without turning the heading hold into a hard foot-scrubbing lock.
The motor impulse is applied inside the same iterative solve as the articulated joints and ground
contacts, so those constraints respond during the correction instead of after an external velocity
write. This mode bypasses both direct `AddTorque` PD control and the legacy
`SetAngularVelocity` spring, so no phase boundary can run multiple upright controllers. Tests 1-6
retain the already-validated legacy spring. Handoff cannot occur until recentering is complete, the
COM is within 4 cm of the fixed midpoint, tilt is inside the configured gate (15 degrees by default),
heading error is within 8 degrees, and the orientation, support, and joint-motor controllers are
unsaturated. Raw root angular velocity remains diagnostic rather than a handoff gate. These rules
prevent a new step from starting in the
middle of a saturated correction without recreating the dual-controller recovery shake.
On stopping, the controller holds the final stable support target from the completed transfer;
it does not pull the COM toward the midpoint of the staggered soles. Over the configured stop
duration, the captured walking leg targets blend smoothly into the bind/standing targets while
the reach crouch unwinds. The physical sole positions at stop entry remain diagnostic references
for the commanded transition displacement; they are never force targets. Once the pose and crouch
release finishes, both physical soles are recaptured exactly once and subsequent drift is measured
from those settled-pose references through the standing handoff. Only that post-release drift must
remain below 4 cm. `STOP_CHECK` reports both measurements, support error, pose/reference state, and
lock weights/forces (which must remain zero), so the validator distinguishes intentional stance
geometry change from continued sliding. Once the released pose is quiet, support and walking-pose
ownership pass to the validated Test 1 standing controller for another 0.5-second stability check.

`[LocoTest7]` reports `START`, every `STEP_COMPLETE` and `ROLE_SWAP`, periodic `STATUS` and
`CONTROL_STATUS`, the
`STOPPING`/`RETURN_STAND` transition, and a final `COMPLETE_CHECK`. The completion check verifies
the requested step count or duration, exactly one airborne/contact cycle per step, positive COM
travel, bounded drift/tilt/motor effort, convergence of the final two support advances and step
periods, and a stable return to standing.
`CONTROL_STATUS` decomposes the causal control state into signed root pitch/roll/yaw velocity,
horizontal tilt rate, the active upright mode, legacy angular-velocity delta, the solver motor's
signed applied torque, per-axis torque cap and saturation, support-target velocity, and the COM
support controller's position (P) and damping (D) force components. It also reports current/peak
heading error plus yaw-motor saturation. `INTERSTEP_CHECK` reports each handoff
gate separately so a timeout identifies the branch that failed rather than only its final tilt
symptom. The final `COMPLETE_CHECK` reports shutdown drift separately from the maximum active-gait
drift so a stopping failure cannot be mistaken for a failed step cycle.

**Pass:**

- 10 alternating steps complete from both F6/F7 starting supports with bounded tilt, plant drift,
  and tracking error;
- step period and length converge instead of growing or shrinking each cycle;
- stopping returns to the validated Test 1 standing state.

**Completion evidence (2026-07-29):** both mirrored ten-step runs emitted
`COMPLETE_CHECK result=PASS`. They completed 20/20 expected contact edges with 8 mm maximum active-
gait drift, 11 mm maximum post-release stop drift, 6.2 degrees peak tilt, 0.07 maximum motor ratio,
and a stable standing return. Forward travel was 0.625 m and 0.636 m; the final support advances
converged to 66/66 mm and 65/65 mm respectively.

## Current status

Tests 1-7 have passed. The locomotion foundation now covers stable standing, weight shift, physical
swing and landing, support transfer, mirrored alternation, a repeatable continuous limit cycle, and
a controlled return to standing. Test 7 remains the regression oracle while its validated control
laws are simplified and moved out of the validation-only F6/F7 path.

## Next milestone - gameplay character controller

The next task is to turn the validated Test 7 behavior into a smaller input-driven runtime
controller. Preserve the proven physical invariants while removing validation-specific sequencing,
duplicated state, and test-only gates:

- map player input direction and magnitude to heading, desired speed, and bounded footholds instead
  of using fixed F6/F7 forward sequences;
- retain contact-driven phase changes, COM/velocity foot-placement feedback, velocity lead, reach
  admission, invariant sole orientation, and the single solver-based upright/heading controller;
- reuse the stable stop transition when input returns to zero, with a deliberate closing step added
  later if gameplay requires a symmetric final stance;
- keep one owner per physical task and never combine foot locks, competing upright controllers, or
  independently moving support targets;
- keep Test 7 runnable as a deterministic regression until the gameplay controller reproduces both
  mirrored passes, then remove only scaffolding proven to be redundant;
- establish flat-ground start, move, turn, direction-change, and stop parity before adding slopes,
  uneven terrain, recovery, animation styling, or higher gameplay speeds.

### Recommended implementation sequence

1. Extract Test 7 into a reusable runtime gait core without changing its validated behavior. The
   gait consumes a command containing start/stop intent, initial support side, desired world-space
   travel direction, desired speed, and an optional run limit. Keyboard, gamepad, AI, replay, and
   the Test 7 harness are command producers rather than branches inside the physical controller.
2. Feed camera-relative player movement into that core. Initially, desired travel direction and
   desired facing are the same; independent facing and strafing remain a later extension.
3. Establish flat-ground parity for standing start, straight movement, direction changes, and the
   validated controlled stop.
4. Split the validation harness and diagnostics away from the runtime component after the runtime
   path reproduces Test 7. Keep Test 7 runnable until the replacement passes both mirrored runs.
5. Add turning through bounded heading and per-step planning, then validate it in both directions.
6. Add and validate jumping without weakening the standing, contact, or stop invariants.
7. Validate variable-speed input and tune responsiveness using acceleration, cadence, stride
   bounds, and yaw rate together. Do not treat a larger foothold or a higher force limit as an
   isolated responsiveness fix.

**Implementation status (2026-07-30):** steps 1 and 2 are implemented. The currently scoped
flat-ground portion of step 3 has passed gameplay validation for standing starts, sustained straight
movement, controlled stops, and direction changes. Test 7 and normal
gameplay produce the same `GaitCommand`; normal gameplay maps camera-relative WASD direction and
input magnitude into the validated continuous gait, uses hysteresis at the walking threshold, and
requests Test 7's controlled stop when input returns to zero. The commanded direction is latched
for each run. A live direction change finishes the active step and controlled stop, performs the
temporary settled turn described below, then restarts in the held direction. It never rotates an
active planted-foot frame.

**Gameplay validation checkpoint (2026-07-30):** fresh-scene W/A/S/D starts passed; sustained
straight travel in each commanded direction passed; release during the first step, between steps,
and after several steps returned through the controlled stop; adjacent 90-degree changes and
opposite 180-degree reversals completed stop, eased reorientation, and restart in the held direction.
Variable-speed behavior is deliberately deferred until after controlled turning and jumping. This
checkpoint is sufficient to begin structural refactoring, provided the Test 7 regression path and
these runtime scenarios remain behaviorally unchanged throughout the split.

### Runtime control contract

The first gameplay controller faces its direction of travel. This keeps one horizontal command
vector authoritative and makes camera-relative input unambiguous. A future strafe/aim controller
may provide separate desired-velocity and desired-facing vectors, but it must not be introduced
implicitly while basic turning is being validated.

Input magnitude should pass through a deadzone and acceleration limit before becoming desired
speed. Start and stop require separate thresholds or equivalent hysteresis so small analog-input
noise cannot chatter between standing and walking. A rapid reversal is initially handled by
finishing the active step, performing the validated stop, snapping the settled physical rig to the
new heading, and restarting in that direction. A collision-aware in-place pivot is a separate gait
capability.

### Turning ownership and validation

Turning must not rotate the entire control frame freely against planted feet. Maintain a smoothly
rate-limited desired heading, but latch the active swing foothold and its touchdown orientation for
the step. The stance foot remains fixed in world translation under the existing single-owner rule.
The next step receives a new heading basis at the phase boundary, and its lateral lane must be
constructed in that basis with a minimum separation that prevents leg crossing.

The flat-ground sole invariant changes slightly for turning: preserve alignment to the ground
normal (pitch and roll), while latching a permitted yaw for each new touchdown. Reusing Test 7's
complete initial world-space sole quaternion would prevent the feet from following a turn and
would force the heading motor to scrub them across the floor.

Validate turning as another incremental ladder:

- gentle constant-curvature walking, left and right;
- 45- and 90-degree direction changes initiated in each gait phase;
- stopping while turning;
- a rapid 180-degree reversal through controlled stop and restart;
- only then consider in-place pivots, crossover steps, side steps, or other stylistic IK motion.

Each run retains Test 7's contact, drift, tilt, reach, heading-error, and saturation checks and
adds minimum foot separation plus requested-versus-achieved travel direction. Larger gameplay
steps are accepted only if cadence and phase gates allow the COM to advance correspondingly; step
length by itself is not the gameplay-speed controller.

**Directional-start implementation (2026-07-30):** runtime gait now owns one authoritative
horizontal `forward/right/heading` frame. Settled sole rotations and anatomical knee-pole vectors
are stored relative to the heading that produced the standing pose, then reconstructed beneath the
latched heading for each swing. This preserves flat-ground pitch/roll without forcing every landing
back to the original W-facing yaw. Once idle, a requested heading change enters a short eased turn
transition. Each update applies only that frame's incremental yaw to every live ragdoll body around
the midpoint of the settled feet. A smoothstep curve provides ease-in/ease-out timing; the default
540 degrees/second produces about 0.17 seconds for 90 degrees and 0.33 seconds for 180 degrees, with
the duration clamped to 0.12-0.40 seconds. Body-relative transforms, orientations, and linear/angular
momentum are preserved and the physics contact cache is invalidated. Releasing input stops at the
current partial heading, while changing direction retargets from that heading. After completion the
gait stays idle through one physics refresh, then held input starts the ordinary straight gait.

This gameplay turn does **not** discard the valid settled baseline or wait through the one-second
baseline-acquisition gate. Cached world-space foot, COM, and support references rotate with the
rig. Slow recapture remains reserved for initial startup, explicit reset, and physical recovery.
The eased whole-rig rotation is intentionally temporary: it is not swept through collision geometry
and therefore must be replaced by the bounded, collision-aware turning work in step 5 before it is
used near crowded obstacles as a final shipping solution.

An aborted runtime step cannot restart while movement remains held; input must return to neutral
before another start, preventing a failed physical pose from being recaptured in a retry loop.
Runtime stopping still requires correct contact edges, bounded drift/tilt/motor effort, and a stable
standing return, but it does not apply Test 7's minimum total-forward-travel criterion when input is
released after only one step. `[LocoRuntime] TURN_BLEND` reports begin, retarget, cancel, failure,
and completion with yaw, duration, and heading; `[LocoDirection]` reports requested/active axes, swing side,
world/projected foothold displacement, and actual versus target sole yaw. Validate W, A, D, and S
as four independent fresh-scene runs, then test held-input changes W-to-A, A-to-S, S-to-D, D-to-W,
and 180-degree reversals. No direction may start during the previous direction's stop or abort
recovery.

### Structural refactor checkpoint (2026-07-30)

The validation-to-gameplay refactor is complete at the source-organization level. The primary
`LocamotionController.h` is now 848 lines (down from roughly 7,700) and contains the runtime
component, gameplay inspector/serialization, input command production, standing-pose ownership,
and top-level system update. The detailed physical state machine is isolated in
`LocomotionPhysicalGait.inl`; this keeps the controller readable without concealing that the gait
algorithm itself still has substantial complexity.

The embedded Test 1-7 selector, test input paths, diagnostic state, grounded-motor test, validation
serialization, and validation inspector output have been removed. Former Test 4 concepts now use
physical-step names, and former Test 7 concepts use gait names. `GaitCommand` is production-only:
start, stop, initial support side, desired direction, and desired speed. Test-only step-count,
endurance, and reset commands no longer exist.

The inspector now exposes only gameplay-relevant movement, step-planning, gait-stabilization, pose,
and IK settings. Serialization writes the same production settings; deserialization accepts the old
Test 2-7 and procedural key names as migration fallbacks so existing scenes retain their authored
values. Logging was reduced to runtime lifecycle events, compact opt-in debug status, and failure
warnings.

Footstep planning remains part of the physical gait because foothold choice, phase admission,
contact transfer, and stop policy are locomotion responsibilities. The existing engine `IKSolver`
remains a general model-space analytic two-bone solver; the gait's constrained physical leg command
is still coupled to ragdoll joint limits, motor targets, sole orientation, and measured physics
state. Moving that block into the generic solver before defining a clean data-only interface would
mix gameplay policy into an animation primitive. A later extraction should first introduce a small
constrained-leg input/output type, then move only the reusable solve math.

Both `Sandbox` and `Runtime` Debug targets compile at this checkpoint. Gameplay behavior still needs
the same W/A/S/D start, sustained movement, direction-change, stop, and fresh-scene smoke pass after
the structural split.

### Next refinement - gait phase continuity and responsiveness

**Planned 2026-08-01.** The next locomotion refinement is to remove the visibly rigid pauses and
target changes inside an otherwise stable physical step. This work comes before continuous turning
and before the full gameplay-speed increase. A small temporary speed adjustment may make iteration
less tedious, but cadence and stride are not to be tuned aggressively until the phase behavior is
continuous.

This is not a missing Jolt or physics-API feature. `LocomotionPhysicalGait.inl` already evaluates a
local cubic smoothstep, uses `glm::mix` for task-space targets, and sends the resulting pose and
support commands through the existing analytic IK, ragdoll joint motors, support controller, and
Jolt constraint solve. Interpolation belongs on the command-generation side of that boundary:

1. the FSM determines contact state, support ownership, and which failures are admissible;
2. small trajectory functions evaluate continuous foot, sole, COM, crouch, and assistance targets;
3. the existing IK and physics controllers pursue those targets;
4. measured contacts and physical tracking decide whether the FSM may cross a safety milestone.

No new engine system is required. Smoothstep is sufficient for the first pass; a local quintic
smoothstep or cubic Hermite helper may be introduced only if measured target velocity still has an
objectionable discontinuity. Quaternion targets continue to use normalized interpolation or
`glm::slerp`. A general spline library would add abstraction without solving the present gating
problem.

The current implementation is already partially continuous. The swing target follows a curved,
eased path to its hover point, inter-step support motion uses smoothstep, and landing verification
runs concurrently with support transfer after plant acquisition. Those mechanisms should be
preserved. The remaining rigidity comes primarily from stopping the commanded motion at phase
boundaries, rebuilding a target from a new starting sample, and serially waiting for predicates
whose control work could begin safely before the previous validation has completely finished.

#### What the implementation needs

1. **Phase-residency diagnostics before behavioral changes.** Record time spent in each cadence
   phase, the configured minimum, the actual exit time, and the predicate that delayed exit. Also
   measure commanded foot/support position and velocity immediately before and after each phase
   boundary. This separates a genuinely necessary physical settle from a timer or discontinuous
   target that merely looks like one.
2. **One continuous swing sample.** Evaluate the nominal swing from the measured takeoff point to
   the immutable admitted foothold using one normalized progress value and explicit lift, apex,
   approach, and descent landmarks. The evaluator should return at least desired foot position and
   trajectory progress; target velocity may be analytic or measured from the previous nominal
   sample. Sole leveling and lift-assist weight should be functions of the same progress rather
   than resetting their clocks at visual sub-phase boundaries.
3. **Milestones rather than pose holds.** `SWING`, `ARRIVAL`, `DESCENT`, and
   `TOUCHDOWN_WAIT` may remain named FSM states for diagnostics and contact policy, but crossing a
   name must not itself jump the desired pose or reset target velocity. `ARRIVAL` should represent
   permission to descend, not an unconditional visible stop at hover. An early valid touchdown may
   still end the trajectory immediately through the existing measured-contact gate.
4. **Continuous support intent.** Preserve the single support-target owner, but blend its command
   toward the upcoming support throughout the portions of the step where both balance and contact
   invariants permit it. Plant acquisition remains the ownership boundary for treating the new
   foot as support. Landing verification may continue in parallel with transfer, as it does now.
   The next weight shift can be prepared while double support is stable, without allowing the old
   stance foot to lift before the transfer and handoff predicates pass.
5. **Continuous boundary handoff.** When a measured physical sample must seed a recovery segment,
   capture it once and construct a curve whose first position matches the previous command. Do not
   rebuild endpoints from noisy measured feet every frame. Position continuity is mandatory;
   velocity continuity is the next priority where it does not conflict with collision response.
6. **Independent clocks for control and validation.** A trajectory clock describes nominal motion;
   contact-debounce, stable-hold, and timeout clocks validate the physical result. A validation
   delay should not freeze every unrelated control channel. Conversely, reaching normalized
   trajectory time 1.0 never substitutes for a real contact or stable-support predicate.
7. **An incremental validation ladder.** First reproduce the current low-speed straight gait with
   unchanged footholds and safety limits. Then remove the arrival pause, then smooth the
   post-touchdown/support handoff, and only afterward reduce the whole-cycle target period. At each
   stage repeat mirrored starts, sustained travel, release in every major phase, and controlled
   return to standing.

#### What the implementation does not need

- no Jolt modification or new `PhysicsAPI` trajectory interface;
- no new ECS system, animation graph, blend tree, or parallel execution/threading;
- no general-purpose spline subsystem for the first iteration;
- no removal of the contact-driven FSM, abort paths, reach admission, or timeout diagnostics;
- no second owner for foot placement, support motion, root orientation, or leg IK;
- no relaxation of touchdown, drift, tilt, reach, foot-separation, or motor-saturation limits just
  to make a phase finish sooner;
- no continuous turning, strafing, jumping, uneven-terrain work, or high-speed tuning in the same
  behavioral change.

#### Acceptance criteria

- the desired foot and support positions have no phase-boundary jump attributable to changing FSM
  state, and any recovery re-seed is visible in diagnostics;
- the actual per-phase dwell report explains every interval beyond its configured cadence budget;
- mirrored continuous runs retain the existing contact-edge, support-advance, drift, tilt, reach,
  heading-error, and motor-saturation bounds;
- release during weight shift, swing, descent, transfer, and inter-step still reaches the validated
  controlled stop without a new restart loop;
- the measured step period falls because unnecessary holds were removed, not because safety
  predicates or timeouts were bypassed;
- only after these checks pass should bounded per-step turning be added, followed by coordinated
  cadence-and-stride speed tuning.

**First implementation slice (2026-08-01):** the ordinary, non-cancelled swing now keeps one
immutable normalized trajectory from measured takeoff through hover and descent to the admitted
foothold. The existing lift/apex/arrival shape is preserved, but `ARRIVAL` no longer resets the
nominal target from a new measured start before descent. Arrival confidence begins accumulating
during the last portion of swing, so a well-tracked foot can pass through the milestone after at
most the normal frame boundary instead of paying a fresh unconditional settle delay. A lagging,
misaligned, or recontacting sole still uses the existing bounded arrival gate and abort policy.

Task-space foot correction now remains continuous across `ARRIVAL -> DESCENT` and eases to zero at
the admitted foothold. Lift assistance and sole leveling are also evaluated from the shared
trajectory progress rather than selecting unrelated phase constants. Early/late commanded-stop
paths remain explicitly separate recovery trajectories and may capture one measured start; they
were not silently folded into the ordinary curve. When locomotion debug output is enabled, each
phase transition now reports its residence time, nominal cadence budget, excess time, foot/support
target step, and trajectory progress. The periodic status line includes phase time, trajectory
progress, last completed step period, and measured speed.

The implementation compiles in the `Sandbox` and `Runtime` Debug targets. Physical gameplay
validation remains intentionally short for this refinement:

1. hold forward for four completed steps, release, and confirm a controlled standing return;
2. repeat for four steps so the runtime's next-start support selection exercises the mirror;
3. perform one release during swing and one during transfer;
4. inspect the resulting transition records for an unexplained phase excess, a target jump, or an
   abort.

This should take well under a minute and replaces the old 60-second endurance run for the current
iteration. Longer endurance testing is reserved for a later checkpoint after phase continuity,
turning, and gameplay cadence have each passed their short validation ladder.

**First visual review result (2026-08-01): not accepted.** Although the unified swing curve removed
one target reset, the character still visibly stopped at landing, load transfer, inter-step, and the
next weight shift. The review log also recorded repeated recoverable old-support drift, three
transfer aborts, and restart blocking while forward input remained held. This showed that smoothing
the airborne foot alone was insufficient: the serial support-ownership gates were still presenting
the FSM as a sequence of poses, and the abort policy was leaking into normal input handling.

**Second implementation slice (2026-08-01): continuous support handoff.** The named FSM states are
retained as contact and safety milestones, but the ordinary walking path no longer commands a stop
between them:

- touchdown now needs credible contact, trajectory progress, low vertical speed, and a sole within
  15 degrees of the intended ground-relative orientation;
- plant acquisition requires the loaded sole to be within 12 degrees, while support transfer begins
  during the plant-acquisition interval instead of waiting for a separate settled pose;
- because the physics facade does not currently expose per-foot normal impulse, support ownership is
  estimated by projecting the COM along the measured old-foot-to-new-foot support span. The new foot
  must own at least 68 percent of that span before the old foot is considered unloaded;
- the achieved support target is preserved through transfer, hold, inter-step, and role swap. After
  the first standing start, the next swing is planned directly from that handoff and bypasses the
  former recenter-and-weight-shift restart;
- continuous-gait landing and handoff no longer require both feet and the COM to become nearly
  motionless. Contact, sole alignment, load ownership, drift, tilt, saturation, and bounded motion
  remain mandatory;
- a recoverable abort while movement is still held now waits for stable abort recovery and queues an
  automatic restart. Two consecutive retries are allowed with a 0.35-second cooldown; exceeding the
  limit still requires neutral input so a persistent physical failure cannot loop forever.

The load projection is intentionally a controller-local proxy, not a new physics system. If later
terrain or dynamic-contact work needs true force allocation, the narrow follow-up is to expose
per-contact impulse through the physics facade and replace this proxy without changing the gait
state contract.

The short visual validation for this slice is:

1. hold forward for four to six steps and confirm there is no full-body stop and second lateral
   weight shift between consecutive steps;
2. confirm the transition log passes through `TRANSFER`, `HOLD`, `INTER_STEP`, and the subsequent
   `WEIGHT_SHIFT` without a target jump or a long residence in those milestone states;
3. confirm touchdown occurs with a level sole and neither support foot drifts beyond the existing
   four-centimeter guard;
4. if one recoverable abort occurs, keep forward held and confirm `AUTO_RETRY_QUEUED` is followed by
   a new start without releasing and pressing the key again.

This is again a short four-to-six-step visual check, not the previous long endurance test. Cadence
and stride-speed tuning remain separate until this handoff is visually accepted.

**Second visual review result (2026-08-01): improved but not accepted.** The character appeared to
take three steps before requiring a new key press, but the transition log showed three failed
landing/transfer attempts rather than completed alternating steps. Each attempt reached `HOLD`,
dragged the newly planted sole 4.0-5.1 cm, aborted, and consumed one bounded automatic retry. The
third consecutive failure correctly activated the retry limit. Visually, the remaining command
sequence was still foot stop at landing, delayed pelvis shift, fast transfer, and another stop.

**Third implementation slice (2026-08-01): anchored contact and velocity-continuous support.** This
slice changes the command ownership responsible for those stops:

- the swing foot now retains horizontal travel throughout descent rather than reaching its complete
  forward position at `ARRIVAL` and dropping vertically;
- a sole-aligned arrival crosses the diagnostic milestone immediately, while the existing contact,
  clearance, leveling, and timeout policies continue to guard descent and touchdown;
- the support target begins one cubic Hermite curve during descent, before touchdown. Its start and
  end velocities are explicit, and its forward velocity continues after lateral transfer so phase
  changes do not reduce the pelvis command to zero;
- credible touchdown captures an immutable world-space sole anchor. The pre-contact support curve is
  re-targeted to the measured plant with matching position and incoming velocity, rather than
  starting a new zero-velocity transfer segment;
- after contact, analytic leg IK continues solving the planted leg against that world anchor every
  update while the pelvis moves. On subsequent steps the stance leg receives the same treatment;
  fixed touchdown joint angles no longer drag the foot with the hips;
- `TRANSFER` and `HOLD` still validate contact, sole angle, load ownership, drift, tilt, and motor
  health, but neither state freezes the support command.

The Hermite evaluator is controller-local command math. It does not add a spline subsystem or alter
Jolt. Transition diagnostics now include curve progress, commanded horizontal support speed, and
plant drift. The next short visual check remains four to six held-forward steps; the decisive log
criteria are a `SUPPORT_CURVE_BEGIN` before each touchdown, nonzero support speed through landing and
`HOLD`, plant drift below 4 cm, and a real `INTER_STEP` role swap rather than an automatic retry.

**Third visual review result (2026-08-01): rear-foot drag and touchdown regression.** The new support
curve was continuous but not physically paced. Its endpoint tangents were limited to 0.1 m/s, while
the middle of the Hermite polynomial reached 0.79-0.89 m/s because it attempted almost the complete
support-span displacement in 0.336 seconds. It began at only 4-5 percent incoming-foot load, pulled
the sole accepted rear support 1.7-3.2 cm, and left the descending foot 8.6-13.2 cm from its target.
Contact existed, but touchdown correctly rejected that horizontal/sole error and timed out. No
attempt reached a real support handoff; the apparent following weight shifts were abort retries.

**Fourth implementation slice (2026-08-01): bounded preload and first-step rear anchor.** The support
curve now uses two velocity-continuous intentions without transferring ownership prematurely:

- descent targets only 20 percent of the predicted support span as an anticipatory preload;
- credible touchdown re-seeds the curve at the current commanded position and velocity, then targets
  the measured old-foot-to-new-foot support span over at least 0.45 seconds;
- the evaluated command—not merely its Hermite endpoint tangents—is limited to 0.30 m/s and
  1.75 m/s^2. A bounded position-error term lets the command finish the physical handoff without a
  mid-curve velocity spike;
- both planted legs now capture complete world-anchor IK references at gait start. The original
  stance foot is therefore solved against its world-space plant during the first weight shift,
  rather than receiving anchored treatment only after it has previously been a swing leg;
- after a validated role swap, the old support anchor is released immediately into `TAKEOFF`.
  Continuous gait requires only 25 ms of airborne confirmation before `SWING`, down from the former
  50 ms, while contact loss and clearance remain required;
- logs distinguish `SUPPORT_CURVE_BEGIN` preload from `SUPPORT_CURVE_CONTACT`, report commanded
  support speed, and include both rear baseline drift and direct rear-anchor error at every phase
  transition. `REAR_RELEASE` records anchor error and new-foot load at toe-off.

The short visual/log criterion for this slice is not step count. First confirm descent support speed
stays at or below 0.30 m/s, rear-anchor error remains below the existing four-centimeter boundary,
and contact advances through `SUPPORT_CURVE_CONTACT`. Only then evaluate whether `REAR_RELEASE` and
`TAKEOFF -> SWING` remove the visible back-foot drag.

**Fourth visual review result (2026-08-01): opening motion accepted, second plant failed.** From a
clean start, the first right-foot plant completed the full `HOLD -> INTER_STEP` handoff with only
1.6-1.8 cm of plant drift, and rear-anchor release entered the next swing correctly. The following
left-foot plant repeatedly accumulated 4.7-5.5 cm of drift during `SETTLE`; it was then admitted as
kinematically settled and aborted 13 ms into `TRANSFER`. The rear/right anchor remained within
1.1-1.8 cm and all leg motors remained far below saturation. The failure was therefore neither the
new toe-off nor insufficient motor authority: full load was accelerating before the incoming plant
had demonstrated positional stability.

**Fifth implementation slice (2026-08-01): drift-aware plant acquisition.** The load and planted-leg
controllers now cooperate during the contact-acquisition window:

- while touchdown is accepted but plant ownership is not, support speed is limited to 0.15 m/s and
  acceleration to 0.85 m/s^2;
- measured drift progressively lowers that ceiling to 0.08 m/s and 0.55 m/s^2. Lateral velocity is
  reduced as far as 0.03 m/s while up to 0.10 m/s of forward flow remains, avoiding a whole-body
  stop during recovery;
- deceleration may use up to 2.5 m/s^2 so an emerging slide is arrested promptly. After acquisition,
  the ordinary 0.30 m/s and 1.75 m/s^2 limits ramp back over 0.15 seconds instead of changing in one
  frame;
- planted-leg IK now adds a bounded restoring task-space bias equal to 65 percent of measured anchor
  error, capped at 2 cm, and uses a response time no slower than 0.05 seconds;
- acquisition requires plant drift at or below 3 cm and filtered outward drift rate no greater than
  0.02 m/s for the complete stable interval. A sliding foot remains in recoverable acquisition
  rather than being admitted and immediately failing the unchanged 4 cm transfer guard;
- `PLANT_RECOVERY` reports the drift, growth rate, and support speed at the first corrective response.
  Phase logs now include the filtered plant-drift rate, and acquisition timeout diagnostics expose
  both new predicates.

The next short run should preserve the accepted opening motion. On the second landing, either no
`PLANT_RECOVERY` should be needed, or it should be followed by falling plant drift, a non-positive or
small drift rate, and `SETTLE -> TRANSFER` below 3 cm. The 4 cm emergency boundary is intentionally
unchanged.

**Rejected handoff experiment (2026-08-01): moving HOLD gate and early rear release.** Replacing the
fixed COM predicate with contact, load, plant drift, and moving-speed checks did not remove the
second-step pause and looked worse visually, so the experiment was reverted. The second handoff
still remained in `HOLD` for 1.01 seconds even though new-foot load was 82 percent, drift was only
2.3 cm and shrinking, sole angle was 5.5 degrees, and neither controller was saturated. The new gate
failed solely because total planted-foot speed was 0.278 m/s against a 0.180 m/s threshold. This
demonstrates that neither a fixed COM endpoint nor a low total-foot-speed requirement is the right
completion signal for a continuously correcting plant. The implementation is restored to the
fifth-slice behavior pending a handoff design that does not introduce another stop condition.

## Relationship to SIMBICON

Keep from the paper:

- explicit left/right stance states;
- contact-driven phase changes;
- torso and swing-leg feedback;
- foot placement based on COM position and velocity;
- target poses that remain procedural and imperfectly tracked.

Adapt for this engine:

- validate the 3D ragdoll and contact foundation before seeking a limit cycle;
- use bounded analytic IK only for the swing leg during the relevant tests;
- use a single stance-contact authority;
- use force-limited physical assistance suitable for a responsive game character;
- preserve real collision response and fall behavior rather than forcing a paper-pure
  torque controller onto a different rig.

## Workflow

1. Implement or enable only the current test.
2. Run the same reproducible scenario several times.
3. Compare measurements with that test's pass criteria.
4. Fix the earliest violated invariant.
5. Record the result here and advance only after the test passes.

During controller extraction, preserve the completed Test 7 path as a regression baseline. Do not
add terrain, stylistic secondary motion, or recovery logic until the input-driven flat-ground
controller matches its start, gait, mirrored-direction, and stopping invariants.

## Relevant files

- `Sandbox/src/Scripts/LocamotionController.h`
- `Sandbox/src/Scripts/LocomotionPhysicalGait.inl`
- `Sandbox/src/Scripts/LocomotionGaitRuntime.h`
- `Engine/src/Scene/Physics/PhysicsSystem.cpp`
- `Engine/include/Scene/Physics/RagdollComponent.h`
- `Engine/include/Scene/Physics/PhysicsAPI.h`
- `Assets/Models/CesiumMan.ragdoll`
- `Assets/Scenes/CharacterTest.scene`
- `logs/loco_debug.log`
