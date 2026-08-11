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

**2026-08-03 boundary review: swing landmark discontinuity corrected.** The latest compact phase log
contained one genuine two-step handoff followed by a third-plant failure. Across that window the
one-frame desired-foot change at `SWING -> ARRIVAL` grew from 2.8 cm to 6.3 cm to 8.6 cm, incoming
touchdown load rose from roughly 21 percent to 32 percent, and the third plant slid 3.9 cm before
acquisition timed out. The cause was an implementation mismatch in the nominal unified trajectory:
the swing evaluated `trajectoryPoint(0.70)`, then the transition frame replaced it with
`hoverTarget`, whose horizontal position was already the final foothold; `ARRIVAL` returned to
`trajectoryPoint(0.70)` on the next frame. This forward/back target impulse also entered the
task-space velocity-lead term immediately before descent. The transition now retains
`trajectoryPoint(0.70)`, preserving the intended horizontal continuation into descent. Plant
acquisition timeout output also reports the previously hidden loaded-sole predicate, so a level-angle
miss can no longer appear as a failure where every printed gate passed.

The same run exposed a separate `HOLD` timing race. Three transfer failures had 97-100 percent
new-foot load, both contacts, 0.056-0.075 m/s COM speed, 1.2-2.2 degree tilt, and no support or motor
saturation. They failed the COM predicate because it still referenced the Hermite curve's static
endpoint even though the controller intentionally continued the support command beyond that point.
`HOLD` completion now measures COM tracking error against the live support command. The fixed
endpoint and new-foot radius remain diagnostic, while contact, 68-percent load ownership, sole
alignment, plant drift, bounded motion, tilt, lift release, lock ownership, and saturation remain
mandatory gates. This removes dependence on landing verification becoming ready during the brief
instant that the moving COM crosses an obsolete point.

The immediate validation is the same short held-forward run in the compact log format. A passing
run should show a near-frame-scale `targetStep.foot` at both `SWING -> ARRIVAL` and
`ARRIVAL -> DESCENT`, rather than a stride-scaled jump; plant recovery should then converge below
3 cm with non-growing drift through at least six real `INTER_STEP` role swaps. Support-curve and
plant thresholds remain unchanged for this check.

The support-command ceiling is now the serialized inspector property
`gaitSupportMaxSpeed`, with a compatibility default of 0.30 m/s. `CharacterTest.scene` uses
0.15 m/s for the next diagnostic run; the cap applies to curve entry, plant acquisition, and the
post-acquisition transfer ramp.

**2026-08-03 staged touchdown transfer.** The first run under the 0.15 m/s diagnostic cap produced
seven support curves, three completed handoffs, and four plant-acquisition timeouts, with no
transfer, stance-drift, or touchdown failures. Completed handoffs left `HOLD` at 0.047-0.095 m/s,
confirming that the cap removed the previous 0.21-0.30 m/s handoff escalation. It did not remove the
root contact problem: every touchdown still entered `PLANT_RECOVERY`, and the visible standing
interruptions remained the 0.6-second acquisition timeout followed by `ABORT -> IDLE` auto-retry.

Touchdown therefore no longer re-targets the support curve directly to the full new-foot handoff.
It C1-reseeds from the live command toward 20 percent of the remaining current-to-plant span while
the incoming sole proves stable. Successful acquisition performs a second C1 re-seed from that exact
position and velocity to the ordinary full transfer target. A plant already within 2.5 cm, with a
level sole, low foot speed, both contacts, and drift growth no greater than 0.05 m/s receives up to
0.20 seconds of bounded grace instead of a whole-body abort; unsafe or worsening plants retain the
base timeout. `SUPPORT_CURVE_ACQUIRED` and `PLANT_ACQUIRE_GRACE` expose both decisions, and failure
logs now identify the incoming foot explicitly.

The 0.15 m/s scene value is a temporary stability envelope, not the intended gait-speed ceiling.
The component default remains 0.30 m/s, the staged transfer preserves nonzero velocity at both
boundaries, and later speed/turning work can raise the scene cap without removing the touchdown
stability state.

**2026-08-03 contact-pivot plant correction.** The staged-transfer run made the remaining invariant
visible: six first plants acquired, four reached a real handoff, but no second plant acquired. The
five failures covered both incoming feet, ruling out a left-only calibration fault. Failed second
plants had support command speed as low as 0.03 m/s, while the foot body reached 0.18-0.20 m/s,
translated about 4 cm, and rolled to 13-16 degrees. Their touchdown contact was repeatedly near
`+/-0.11 m` on the sole's long edge, matching the box half-extent. Every landing entering
`PLANT_RECOVERY` was therefore consistent with the controller counting the center translation
required to rock an edge contact flat as plant slip.

Touchdown now captures one immutable world contact point and its foot-local material coordinate.
Plant drift measures motion of that material point rather than motion of the rigid body's center.
During planted-leg sole leveling, IK derives the compatible foot-center target around the contact
pivot and applies restoring correction to pivot error; it no longer commands both a fixed center
and a fixed edge while changing orientation. The derived level-center target persists when the new
plant becomes the next stance foot. New `TOUCHDOWN_ANCHOR`, `anchorDrift`, and `centerTravel`
diagnostics distinguish real patch slip from the expected center compensation.

The same run exposed two unrelated commanded stops at 5.9/6.0 cm and 5.1/6.0 cm achieved advance.
Landing advance now has 1 cm of hysteresis, matching the scale of accepted physical tracking error,
and logs when that margin preserves continuation. Truly short landings still request the bounded
standing recovery.

**2026-08-03 trend-aware acquisition and retry recovery.** The first contact-pivot validation
completed ten full handoffs and reached touchdown on step eleven. The eleventh plant retained both
contacts, safe foot speed, a 6.8-degree sole, and only 2.5 cm of anchor drift, but its filtered drift
growth of 0.058 m/s narrowly missed the former 0.050 m/s grace predicate. A later failed plant was
already stationary at 0.009 m/s with -0.001 m/s drift growth, but remained 3.6 cm from its immutable
first-impact patch. One separate 4.0 cm plant was still growing at 0.096 m/s and remained a genuine
slip. The correction system was active in all three cases; the remaining problem was classifying
its outcome at the fixed timeout.

The strict acquisition proof remains unchanged at no more than 3 cm of anchor drift and no more
than 0.02 m/s outward growth for the complete stable interval. Safe plants within 3 cm may now use
the existing 0.20-second grace while growth is no greater than 0.08 m/s, giving bounded correction
time to a converging near-miss without declaring it stable early. A plant between 3 and 4 cm may
capture one fresh contact anchor only after both contacts remain present, the sole is level, foot
speed is at most 0.03 m/s, and absolute drift growth is at most 0.01 m/s for 0.10 seconds. This
one-time rebase recognizes a new stationary contact patch; it cannot ratchet repeatedly through a
slide. A plant beyond 4 cm and still growing faster than 0.05 m/s must remain in that state for
0.18 seconds before the emergency guard aborts. This persistence distinguishes an ordinary
first-impact excursion from a slide that correction is not arresting.

Plant correction still uses the validated 65-percent restoring bias and 2 cm cap. Acquisition,
recovery, rebase, and failure logs now report peak requested/applied correction and whether that cap
was reached, so correction authority can be raised later only from measured saturation evidence.
Automatic retries also retain the 0.35-second cooldown but now require 0.25 seconds of stable double
support after the abort has completed and the FSM has entered `IDLE` before held movement can start
another gait. `AUTO_RETRY_READY` exposes that proof. Stability accumulated while the abort pose was
still active no longer permits a next-frame `IDLE -> WEIGHT_SHIFT` restart. The scene's temporary
0.15 m/s support cap is unchanged.

**2026-08-03 two-stage plant-anchor ownership.** The trend-aware run reached nine steps, but the
incoming feet became visibly wobblier over the sequence. The log showed a strong signature on the
right plants: requested pivot corrections repeatedly reached roughly 2.5-3.0 cm and saturated at
the 2 cm safety cap, while the collision-local contact on one failed plant moved from approximately
`+0.10 m` to `-0.11 m` along the sole. The rigid body retained contact and the character remained
upright; the controller was fighting an obsolete first-impact edge after the load-bearing patch had
migrated across the rolling sole. A separate transfer reached the 68-percent load boundary but lost
its stable window to a very small projection fluctuation.

Plant ownership is now explicitly two-stage. The first-impact material point remains authoritative
only during initial rocking. Contact-local migration begins fading that pivot's position and
restoring authority between 3 and 8 cm, preventing heel/toe manifold changes from injecting more
rocking. Once both feet remain in contact and the incoming sole stays within 8 degrees, below
0.75 rad/s angular speed, and below 0.12 m/s horizontal speed for 0.08 seconds, the controller
captures the measured sole center. It blends from the pivot-compatible center to that frozen center
over 0.09 seconds, then measures and corrects center drift for the rest of support. This is an
ownership handoff rather than a moving rebase: the center target is captured once and cannot chase
a sliding foot. `PLANT_CONTACT_MIGRATION` and `PLANT_ANCHOR_HANDOFF` expose the transition and its
measured quiet conditions.

The COM projection used as the load-bearing proxy now has hysteresis. New-support ownership latches
at 0.68 and remains valid until the projection falls below 0.64, including through `INTER_STEP`.
`SUPPORT_LOAD_LATCH` records acquire/release edges. The 65-percent planted correction gain, 2 cm
correction cap, strict acquisition limits, emergency slip guard, and temporary 0.15 m/s support
speed envelope remain unchanged, isolating this test to anchor ownership and load-gate chatter.

**2026-08-03 anchor-handoff blocker telemetry.** The first two-stage validation showed that the
feature was not applied symmetrically: all four left plants reached `PLANT_ANCHOR_HANDOFF`, while
none of five right plants did. Every right plant remained under impact-pivot ownership, repeatedly
reached the 2 cm correction cap, and step nine eventually aborted at 4 cm of new-support drift.
The load latch acquired cleanly on each completed transfer and did not explain that failure. One
right contact also migrated 0.254 m in foot-local space immediately before transfer, confirming a
collision-manifold flip but not whether pitch rocking, threshold chatter, or controller-induced
translation prevented the 0.08-second quiet proof.

`PLANT_ANCHOR_TRACE` now samples each pivot-owned `SETTLE` at 0.10-second intervals. It records all
four handoff gates, current and maximum contiguous quiet time, heading-relative pitch/roll/yaw
angular velocity, forward/lateral/vertical foot velocity, pivot and center error components,
contact-local migration, support-command velocity, and correction saturation. Handoff, acquisition,
and transfer-abort summaries also report cumulative time blocked by contact, sole angle, angular
speed, and linear speed. These signals are diagnostic only; no quiet threshold or correction
authority changed.

The same run exposed a separate deterministic deadline error. A left center handoff occurred at
0.623 seconds, reset the acquisition proof as required by the new drift invariant, and then aborted
at 0.663 seconds against the original 0.600-second timeout despite only 4 mm of center drift. A
handoff now receives a bounded post-transition deadline of its required stable proof plus 0.06
seconds. This prevents the ownership transition itself from guaranteeing failure while leaving the
ordinary and recoverable acquisition deadlines unchanged.

**2026-08-03 correction-overload pivot release.** The blocker telemetry separated the remaining
right-foot wobble from general gait instability. The validation completed 20 steps before failing
on right step 21. Ordinary right plants repeatedly commanded the full 2 cm pivot correction while
the foot reversed between roughly 0.2-0.4 m/s translation and 2-4 rad/s rotation. By contrast,
right steps 7 and 15 experienced large collision-contact migration, which naturally faded pivot
authority; their correction peaks stayed near 2-3 mm, the feet quieted, and center handoff
succeeded. Left plants were generally quiet without needing that release. The support command was
only about 0.03-0.07 m/s, so increasing support damping or permanently reducing gait speed would
not address the measured excitation source.

The controller now treats sustained correction saturation plus excessive planted-foot motion as a
failed ownership mode rather than a request for more authority. If the correction is at its 2 cm
cap while horizontal motion exceeds 0.12 m/s or angular motion exceeds 0.75 rad/s for 0.032 seconds,
it latches a side-neutral pivot release. Over 0.06 seconds, the same smooth weight fades both the
impact-pivot position target and its restoring correction to passive measured-foot ownership. The
normal 0.08-second quiet proof then captures the sole center. Once this recovery has latched,
support acquisition and transfer cannot proceed until that center handoff is complete; the nearby
settled-contact rebase is also disabled so it cannot restore obsolete pivot ownership.

`PLANT_PIVOT_RELEASE` records the correction and motion that caused the latch. `PLANT_ANCHOR_TRACE`
now includes per-frame correction demand, cap state, trigger duration, and release weight, while
handoff, acquisition, and acquisition-failure summaries identify recovery-release ownership. The
existing contact requirements, 8-degree sole gate, 4 cm persistent-slip abort, 30-degree tilt abort,
65-percent correction gain, 2 cm correction cap, and temporary 0.15 m/s support-speed test envelope
remain unchanged.

**2026-08-03 straight-line continuous-walking acceptance.** The first validation after adding the
correction-overload release completed 44 consecutive alternating plants. All 44 touchdowns reached
support acquisition and the load latch; the first 43 completed the following rear-foot release. The
final step entered `HOLD -> STOPPING` from a healthy state after input was released, with 1 mm rear
drift, 1 mm plant drift, and 0.69 normalized new-support load. There were no locomotion warnings,
failed checks, aborts, or automatic retries. This is the first long runtime run in the current
continuous controller that ended because the player stopped rather than because the gait failed.

The new recovery behaved deterministically rather than merely postponing a failure. It activated on
21 of 22 right plants and one left plant. Each recovery faded the saturated pivot correction to
zero, established the required quiet interval, handed ownership to the sole center, and continued
the alternating cycle without accumulated drift. Across the run, 38 plants used center handoff,
one used the bounded settled-contact rebase, and the remaining five acquired safely within the
existing drift gates. The strong right-side activation rate confirms that a plant-impact asymmetry
still exists, but the controller now contains it repeatably. Keep that asymmetry visible in turning
telemetry rather than weakening the recovery or creating side-specific gains.

**Decision:** straight-line continuous walking V1 is complete at the current low-speed test
envelope. This does not complete cadence or higher-speed tuning: the temporary 0.15 m/s support cap
remains in place, and several hold phases still exceed their nominal timing budgets. Preserve this
44-step run as the regression baseline while developing later features.

**Next session (2026-08-04): bounded continuous turning.** Begin with small per-step heading changes
at the current speed. Rotate the latched heading and foothold-planning basis through one deliberate
owner; do not directly spin the root or rotate an active planted-foot frame. Preserve the plant
anchor, pivot-release, load-hysteresis, slip, contact, and tilt systems unchanged. Validate mirrored
left and right arcs first, then increase allowed heading change per step before revisiting cadence
or walking speed.

### Bounded continuous turning - authoritative implementation plan

**Design decision (2026-08-04): implement and validate this in small slices.** The final behavior is
one unified step planner, not separate forward-step and turn-step state machines. A step command has
two independent quantities: horizontal advance and heading change. Straight walking has nonzero
advance and nearly zero yaw; turning in place has nearly zero advance and nonzero yaw; an ordinary
arc has both. The existing contact-driven phases, support ownership, swing trajectory, plant
acquisition, and transfer remain the executor for all three cases.

The work may live on one branch, but each slice below must remain buildable and must pass its own
short physical check before enabling the next slice. A one-shot behavioral switch would change
stop routing, foothold construction, body-heading control, and success metrics simultaneously. A
failure would then be ambiguous between steering geometry, joint reach, support motion, and contact
ownership. The staged sequence is therefore part of the design, not merely a preferred workflow.

This section supersedes the temporary settled `TURN_BLEND` as the intended runtime direction-change
behavior. Keep the whole-ragdoll blend available only as a guarded fallback while the early slices
are being validated. It must not run during an admitted physical turn step, and it can be removed
after the final validation matrix passes. Continuous turning requires no new `PhysicsAPI` operation
and must never call `RotateRagdollYaw` on the ordinary path.

#### Command and state contract

The first implementation continues using `GaitCommand::desiredForward` as both travel direction and
desired facing. Separate movement and aim/facing vectors remain deferred with strafing. Unlike the
current runtime, a live change in `desiredForward` does not automatically request a controlled stop.
The gait stores three distinct headings:

- **desired heading:** the latest normalized horizontal command and the final direction the player
  is asking for;
- **committed heading:** the heading physically established by completed support transfers and used
  as the start of the next plan;
- **active step heading:** immutable start/end headings admitted for the current swing, plus a
  normalized turn-progress value used to move the body-heading target between them.

Retargeting the input changes the desired heading immediately, but it does not rewrite an already
committed late-swing foothold. The next step consumes the newest error. Early-swing replanning may be
added after the latched version passes; it is not required for continuous multi-step turning V1.
For V1, successful foothold admission immediately before `TAKEOFF` is the commit point.

Add explicit per-step telemetry/state rather than overloading `_physicalStepForward` or
`_gaitHeadingTargetRot` with several meanings. The implementation needs equivalents of:

- requested and admitted yaw for the step;
- start, midpoint, and end forward/right bases;
- active-step start/end heading rotations;
- requested and admitted horizontal advance;
- planned and achieved angular progress;
- candidate versus admitted foot position and yaw;
- the constraint that limited admission, when any.

`_physicalStepForward` and `_physicalStepRight` currently feed placement, error projection,
adaptation, and diagnostics. Keep a step's measurement basis immutable until its transfer completes.
Drive the physical heading through the separate active-step rotation, then publish the end basis as
the next committed basis at `INTER_STEP`. This prevents changing projections underneath the current
step while the pelvis is turning.

#### Per-step steering geometry

At the existing foothold-planning point, compute the shortest signed horizontal yaw error:

```text
yawError = atan2(cross(committedForward, desiredForward).y,
                  dot(committedForward, desiredForward))
requestedStepYaw = clamp(yawError, -maxYawPerStep, +maxYawPerStep)
endHeading = rotateY(requestedStepYaw) * committedHeading
midHeading = rotateY(0.5 * requestedStepYaw) * committedHeading
```

`maxYawPerStep` is a conservative controller/cadence limit, not an alias for the swing hip's twist
limit. Begin at 5 degrees for the first physical slice. Increase it only after mirrored runs show
joint-limit reserve, stable support transfer, and bounded planted-foot drift. The final allowed yaw
is the smaller of this controller limit and the whole-step feasible yaw described below.

Construct the requested footprint as a full pose. The current support foot remains the world-space
origin for the relative step, the midpoint heading supplies the arc tangent, and the end heading
supplies the final lateral stance axis:

```text
requestedPosition = stanceFoot
                  + forward(midHeading) * requestedAdvance
                  + right(endHeading) * signedFootSeparation
requestedRotation = endHeadingRotation
                  * swingFootHeadingLocalSettledRotation
```

The signed separation preserves the existing anatomical left/right lane and minimum runtime width.
Using the end-heading right axis rotates the new foot pair toward the requested facing; using the
mid-heading forward axis makes nonzero advance follow the chord of a small arc instead of stepping
fully along either the old or new direction. Terrain raycasts continue grounding the resulting
position and pitch/roll. Ground alignment and yaw are separate: pitch/roll follow the hit normal,
while the admitted turn determines the sole's touchdown yaw.

For gentle turns, retain the validated straight-line advance. A later turn-dominant slice reduces
advance according to the remaining heading error: small error gives a normal curved step, large
error approaches zero advance, and advance returns smoothly as the committed heading aligns. Do
not encode this as a new FSM mode. It is only a scalar input to the same footprint formula.

Alternating zero-advance steps rotate the swing footprint about the current stance foot. The body
naturally shifts over each real support in turn, so no synthetic root pivot is needed for V1. If a
later style requires a mathematically fixed turn center, add a persistent gait-reference pose above
this planner; do not reintroduce direct root rotation.

#### Whole-step feasibility and yaw admission

Admission evaluates a foot **pose**, not just horizontal distance. First clamp the requested yaw by
the authored per-step limit, build the candidate, and test the complete predicted end-of-step
configuration. If it is not safe, reduce the requested yaw and rebuild the pose until it is safe or
until the turn falls below a small no-progress threshold. A descending bounded search is sufficient
for V1; a general optimizer is unnecessary.

The feasibility result must include:

- the existing swing-leg anatomical and configured reach with the candidate sole rotation included
  in the ankle offset;
- swing-hip cone and twist margin after the constrained leg solve, not merely whether the final
  command was silently clamped;
- predicted stance-hip twist margin while its sole remains planted and the pelvis follows the
  active-step heading;
- knee and ankle envelopes for both legs;
- minimum left/right foot separation, correct anatomical lane sign, and a swing path that does not
  pass through the stance leg;
- valid grounded position and acceptable surface normal;
- retained reach and joint-limit reserve for physical tracking and balance correction.

The current reach binary search may still shorten an otherwise valid horizontal candidate, but a
turn plan must re-evaluate separation, lane sign, and angular progress after that shortening. The
planner records requested versus clamped hip/ankle commands. If a joint target loses more than the
admitted margin, shrink step yaw and solve again rather than accepting a footprint that no longer
represents the plan.

The admission loop is conceptually:

```text
admittedYaw = requestedStepYaw
repeat:
    candidate = buildFootPose(admittedYaw, requestedAdvance)
    candidate = groundCandidate(candidate)
    result = predictBothLegsAndCheckConstraints(candidate)
    if result.safe: admit immutable candidate and stop
    admittedYaw *= reductionFactor
until abs(admittedYaw) < minimumUsefulYaw
```

Failure to admit a useful yaw does not immediately weaken limits. Hold the current committed
heading and use the existing stable stop/fallback path, logging the limiting constraint and side.

#### Physical heading ownership

The heading torque remains the only owner of pelvis/root orientation during a physical step. Latch
the current physical/committed heading as the start target and the admitted heading as the end
target. Evaluate a monotonic turn curve from takeoff through support transfer:

- weight shift performs no or only a small authored pre-turn while both feet are planted;
- swing advances most of the heading target while the released leg can reposition;
- touchdown and transfer finish the remaining yaw as the new foot acquires load;
- `INTER_STEP` commits the end heading only after the existing heading, contact, load, drift, tilt,
  and saturation gates pass.

Use shortest-path normalized quaternion interpolation for the target, driven by the gait trajectory
clock rather than a new wall-clock animation. A delayed contact may delay commitment, but it must
not make the heading target jump or restart from a new measured yaw. The stance leg continues using
its frozen planted world anchor. The swing sole and anatomical knee plane interpolate from their
captured takeoff references toward the admitted end-heading references. Existing plant pivot,
center handoff, correction-overload release, and slip ownership do not rotate with the desired
heading after contact; they remain world-space physical facts.

The inter-step heading-error limit continues protecting the next role swap. It compares the physical
root against the **incremental admitted end heading**, never against a distant final 90- or
180-degree input heading. Otherwise a safe multi-step turn would stall after its first step.

#### Generalizing forward-only control and validation

A turn-in-place step intentionally has little forward support advance, so the current minimum-
advance admission and landing checks cannot remain the universal definition of success. Classify
the admitted plan by its nonzero objectives without adding a separate FSM:

- translation objective: validate planned versus achieved horizontal advance;
- angular objective: validate planned versus achieved root/sole yaw and decreasing remaining
  heading error;
- combined objective: validate both with their respective tolerances.

Forward gait adaptation must not interpret zero advance during a deliberate turn as stride loss or
stress. Freeze its forward correction integrators during a pure turn, or feed them only the admitted
translation objective. Preserve lateral/drift/contact/motor feedback. Add angular tracking telemetry
before considering adaptive yaw correction.

The support curves already carry vector positions and velocities, but several endpoint tangents are
derived from a nonnegative projection onto `_physicalStepForward`. During turning, use the actual
horizontal direction from the current support target toward the planned transfer target. A pure
turn may have primarily lateral/tangential support motion; forcing its feed-forward velocity along
the old forward axis would create a pause or a sideways disturbance. Position and velocity remain
continuous across the existing support-ownership handoffs.

Stopping remains contact-driven. Releasing input during a turn finishes or safely cancels the
active swing using the existing early/late cancellation rules, transfers to a valid double-support
state, and enters the validated standing return. It does not invoke the settled whole-ragdoll blend.

#### Implementation slices and verification gates

1. **State and diagnostics only.** Add the desired/committed/active heading distinction, planned-yaw
   fields, constraint-result telemetry, and debug visualization of start/end bases and candidate
   footprint pose. Leave runtime routing and footholds unchanged. Build `Sandbox` and `Runtime`, run
   one straight four-step start/stop, and confirm every new yaw value remains zero with no target
   or support-command change.
2. **Gentle curved steps at 5 degrees per step.** Stop treating an ordinary direction change as a
   stop request when its path is allowed by the new planner. Admit at most 5 degrees, build the
   curved footprint with the existing full advance, and phase the heading target through the step.
   Permit desired errors through 45 degrees to converge over repeated steps; keep errors above that
   on the temporary fallback during this slice. Run mirrored W-to-diagonal left/right changes for at
   least six completed steps each. Heading error must decrease after every successful transfer; no
   planted reference may rotate or exceed existing drift gates.
3. **Feasibility-backed yaw — accepted at the conservative 5-degree runtime baseline
   (2026-08-05).** Route the candidate end pose through both-leg reach/envelope checks,
   add descending yaw admission, and expose the limiting constraint. Raise the cap gradually toward
   the smallest value that produces responsive motion without routine joint clamping; do not choose
   the value from the authored hip limit alone. Repeat mirrored arcs at each increase and retain a
   meaningful hip/ankle/reach reserve.
   The accepted baseline keeps feasibility observational, the cap fixed at 5 degrees, and records
   authoritative routing, descending-yaw reconstruction, and cap expansion as deferred extensions.
4. **Turn-dominant and zero-advance steps — accepted (2026-08-05).** Add smooth advance reduction for large remaining heading
   errors, replace forward-only admission/landing success with objective-relative checks, and make
   support-curve tangents vector-relative. Validate fresh-scene 45- and 90-degree changes in both
   directions, including changes requested during weight shift, swing, descent, transfer, and
   inter-step. No case may abort solely because a deliberate turn produced less than the straight-
   gait minimum forward advance.
5. **Accepted (2026-08-05) — Retarget, stop, and reversal behavior.** Allow the desired heading to change at any time while
   keeping the admitted active step immutable after its commit point. Test left-to-right retargets,
   input release in every major phase, and 180-degree requests. Initially, 180 degrees may use the
   validated controlled stop if multi-step turn admission cannot retain separation or joint margin;
   enable full physical multi-step reversal only after both turn directions pass symmetrically.
6. **Accepted (2026-08-05) — Remove the temporary blend and run regressions.** Disable ordinary `TURN_BLEND` routing, remove
   dead state only after no caller remains, and repeat the accepted 44-step straight run. Then run
   at least 20 completed plants on a constant left arc and 20 on a constant right arc, followed by
   the phase-by-phase 45/90-degree matrix and controlled stops. Only after this gate may cadence,
   stride, or gameplay-speed tuning resume.

**Slice 1 implementation and acceptance (2026-08-04): accepted.**
`LocamotionControllerComponent::TurnPlanDiagnostics` now keeps desired, committed, and immutable
active-step start/mid/end bases separate. It also carries requested/admitted/achieved yaw, angular
progress, requested/admitted advance, candidate/admitted sole poses, reach reserve, foot separation,
ground/admission status, and an extensible limiting-constraint result. All angular planning fields
are initialized to zero and are observational only in this slice. The existing straight planner
copies its already-computed candidate and admitted results into this structure after the existing
reach projection; no new field is read by command routing, foothold construction, support control,
phase transitions, or the constrained leg solve.

With locomotion debug enabled, each admission emits one `[LocomotionTurnPlan]` record. Debug drawing
shows the active start basis in blue, the end basis in magenta, and the candidate sole pose with a
small position marker and local right/up/forward axes. The vertically separated basis origins make
the expected zero-yaw overlap visible without changing either physical frame.

Both dependency-inclusive Debug builds passed for `Sandbox` and `Runtime`. The accepted manual
straight run is the isolated session from 12:11:32 through 12:11:41 in `logs/loco_debug.log`; an
earlier process-automation launch that visibly slid was discarded rather than counted. The accepted
session contains exactly four turn-plan records. Every record reports desired error, requested yaw,
admitted yaw, and achieved yaw as `+0.000` degrees; all four candidates are grounded and accepted
with `constraint=none`; and requested/admitted advance, reach, and separation are identical. There
are no aborts, failed checks, retries, or restart blocks in the session. Ordinary phase-transition
target deltas after gait activation remain at most 1.6 mm for the foot and 1.2 mm for support, with
no command discontinuity. The commanded stop completes with `steps=4`, returns to `IDLE`, and
reports 1.6 cm stop-settle drift, 1.4-degree final tilt, correct contact edges, and no adaptive
recovery.
The historical stride-length diagnostic is `variable`, but it is not a stop-safety failure and is
outside this state-only slice. Slice 1 therefore passes its build, zero-yaw, unchanged-routing, and
four-step start/stop gates.

**Slice 2 implementation and validation (2026-08-04): mixed-frame validation failure.**
Runtime direction changes are now compared with the last successfully committed heading. Desired
errors through the configurable 45-degree fallback threshold remain in the live physical gait;
larger errors retain the temporary settled whole-ragdoll blend. Each admitted footprint clamps its
yaw to `gaitMaxTurnStepDeg` (5 degrees by default), keeps the existing full step advance along the
midpoint heading, and places the anatomical lane along the end-heading right axis. Reach projection
uses the end-heading sole rotation. The first implementation deliberately retained the accepted
straight-gait forward admission, landing checks, and support tangents; the mirrored run proved that
those dependencies cannot remain deferred until Slice 4 even at the conservative 5-degree cap.

The active heading target follows a shortest-path quaternion interpolation across swing, descent,
settle, and support transfer. The planned end heading is immutable after foothold admission. The
committed heading, physical gait basis, runtime yaw, and next-step frame advance only after the
existing successful-transfer gate. The old stance sole continues to use its captured world-space
rotation throughout the step; only the released foot receives the admitted end-heading rotation,
and that rotation becomes its new planted reference at touchdown. A cancelled early return retains
its previous sole rotation. Zero-admitted-yaw steps still report exactly zero requested, admitted,
and achieved turn even when ordinary heading hold has a small physical tracking error.

`[LocomotionTurnPlan]` reports the bounded candidate as before. Each successful transfer now also
emits `[LocomotionTurnCommit]` with admitted/achieved yaw, planned progress, remaining heading error
before and after commit, a monotonic-decrease result, both existing translational drift maxima, and
the old-stance/new-plant reference-rotation deltas. The reference deltas should be zero to logging
precision; they distinguish immutable command references from ordinary physical sole tracking
error. Dependency-inclusive Debug builds pass for both `Sandbox` and `Runtime`, and `git diff
--check` is clean.

The mirrored validation did establish the new heading ownership: both directions eventually
completed all nine 5-degree commits, every successful transfer reduced remaining heading error,
achieved yaw stayed near the admitted yaw, and both planted-reference rotation deltas remained zero.
It did not establish a usable gait. The negative turn required ten abort recoveries (five minimum-
advance rejections, two plant slips, one transfer-drift abort, and two acquisition timeouts); the
positive turn required five (three minimum-advance rejections, one plant slip, and one transfer-
drift abort). A plan could log 10.1 cm of admitted midpoint-frame advance and then fail the old-
forward minimum-advance gate. Accepted failing plants touched down with roughly 14-15 degrees of
sole error, accelerated horizontally to 0.28-0.43 m/s while angular speed reached as much as
2.54 rad/s, saturated the 2 cm planted correction, and crossed the 4 cm slip boundary. Once the
heading error reached zero, the original straight gait became stable again. Slice 2 therefore
failed because the curved footprint, straight objective projections, and straight support tangents
did not share one frame; the authored yaw cap, scalar reach, and heading convergence were not the
limiting failures.

**Slice 2b implementation (2026-08-04): frame-consistent gentle turns; validation pending.**
An admitted nonzero turn now owns one immutable translation coordinate system for its entire step.
Because midpoint forward and end right are slightly oblique, advance and lane are recovered as the
coefficients of that pair rather than as raw projections:

```text
advance = dot(delta, endForward) / dot(midForward, endForward)
lane    = dot(delta, midRight)   / dot(endRight, midRight)
```

The requested/admitted planner values, minimum-advance admission, physical support-to-support
advance, target error, landing retention, settled tracking loss, and gait adaptation now use those
coefficients. At 5 degrees, a 10 cm advance plus an 18 cm lane previously appeared in the start
frame as alternating approximately 11.6 cm and 8.4 cm steps; both now remain the authored 10 cm.
`[LocomotionTurnPlan]` reports `legacyStart` beside the authoritative admitted advance so the removed
alternation remains visible. Zero-yaw and cancelled steps take the original straight projections
exactly.

Preload, contact-acquisition, and full-transfer Hermite curves preserve their incoming C1 velocity,
but an active turn derives its outgoing tangent from the actual horizontal start-to-end support
span. The plant-drift speed limiter decomposes motion in midpoint forward/right instead of the stale
step-entry frame. Straight steps retain their previous forward tangent. This makes support motion
follow the curved footprint without changing its accepted 0.15 m/s speed or 1.75 m/s2 acceleration
limits.

Turn steps must also establish an angularly credible sole before accepting support ownership.
Unlike straight arrival, turn arrival cannot use the 30 ms milestone bypass while sole error exceeds
10 degrees and receives at most 0.25 seconds for convergence. A contacting turn sole is not admitted
until error is at most 10 degrees, angular speed at most 0.75 rad/s, and horizontal speed at most
0.12 m/s; touchdown receives the same bounded 0.25-second window. The body heading remains at its
80-percent touchdown target while waiting, and the eventual anchor captures the then-current quiet
material contact instead of the unstable first impact. `[LocomotionTurnTouchdown]` logs the first
blocked readiness sample; touchdown and commit records carry the accepted sole, angular-speed, and
horizontal-speed values.

Dependency-inclusive Debug builds pass for both `Sandbox` and `Runtime`, and `git diff --check` is
clean.

**Slice 2c implementation (2026-08-04): explicit support ownership; validation pending.** The first
2b run proved that the angular touchdown gate was functioning, but exposed a later invariant
violation. One recorded turn waited 351 ms and accepted contact at 9.9 degrees sole error,
0.032 rad/s angular speed, and 0.005 m/s horizontal speed. Only 33 ms later, ordinary plant
acquisition began transfer with `centerAnchor=no` and `maxQuiet=0.000/0.080s`. Repeated failures then
reported `TRANSFER_DRIFT_ABORT reason=new_support`, `anchorStage=pivot`, no handoff time, and
4.0-4.3 cm new-support drift. The controller had treated a safe impact as permission to bear load.

Planting now has three explicitly different authorities:

1. `TOUCHDOWN_WAIT` accepts a collision only after the relevant impact-readiness gates pass.
2. `SETTLE` owns the immutable material-point pivot. It holds the exact support command present at
   impact and may roll, recovery-release, or rebase that pivot, but it cannot advance load or body
   heading. Only the existing 80 ms quiet proof may capture the sole-center anchor.
3. `SUPPORT_READY` owns that captured center. It completes the fixed 90 ms center blend and then
   proves the ordinary contact, speed, sole, drift, and drift-rate gates for an independent plant-
   acquisition interval. Only that proof may enter `TRANSFER`.

The pre-contact preload remains bounded and may reach its former partial-load position before
impact. At accepted touchdown its curve is frozen at the current commanded position with zero
feed-forward velocity. `beginTransfer()` re-seeds the full handoff from that exact stationary state
only after `[LocomotionGait] SUPPORT_OWNERSHIP_READY`. Consequently
`SUPPORT_CURVE_ACQUIRED` must always report `centerAnchor=yes` and `maxQuiet>=0.080/0.080s`; a pivot
that cannot establish center ownership aborts in `SETTLE`, while a captured center that cannot stay
stable aborts in `SUPPORT_READY`. The turn target remains at 80 percent through both states and
finishes its remaining 20 percent during physical transfer.

Dependency-inclusive Debug builds pass for both `Sandbox` and `Runtime`, and `git diff --check` is
clean. Runtime validation must confirm the new phase sequence
`TOUCHDOWN_WAIT -> SETTLE -> SUPPORT_READY -> TRANSFER`, zero support speed in both ownership-proof
states, and no `TRANSFER_DRIFT_ABORT` whose anchor stage is still `pivot`.

**Slice 2d implementation (2026-08-04): objective-relative landing and closed-loop handoff;
validation pending.** The first run with explicit ownership proved that the state split worked:
successful plants reached `SUPPORT_READY` with a center anchor and a complete quiet interval. The
same visible stand-and-retry behavior remained because three later policies deliberately routed a
physically stable turn back to recovery.

First, the straight minimum-forward-advance check no longer requests `STOPPING` solely because an
active angular step lands short. A quiet center-owned turn plant logs
`LANDING_ADVANCE_CHECK result=TURN_REPLAN`, retains its actual physical footprint, completes the
ordinary transfer gates, and seeds the next plan from the measured feet. Straight translation
retains its existing controlled-stop behavior. Forward stride/loss adaptation is skipped while an
angular objective is active so an intentional curved step cannot inflate the following stride.

Second, support transfer is closed around the same live old-to-new foot span used by the load latch.
The command now targets at least 0.74 normalized new-support load and corrects that component each
frame as either physical foot settles. The existing physical ownership latch remains 0.68 acquire /
0.64 release; it has not been weakened. Hermite continuity, the 0.15 m/s support-speed ceiling, and
the acceleration limiter remain authoritative. `TRANSFER_CHECK` now reports actual load, commanded
load, command target, latch threshold, and every boolean HOLD gate. The fixed distance to the new
sole is explicitly labeled telemetry because it is not a completion gate.

Third, impact-pivot release and loaded-support validation now use one 12-degree sole-ownership
tolerance. The prior 8-degree pivot gate could hold a motionless 9-11-degree sole until timeout even
though the immediately following loaded-support state already accepted it at 12 degrees. Contact,
0.75 rad/s angular speed, 0.12 m/s horizontal speed, and the full 80 ms quiet proof remain required,
so this removes an unreachable transition rather than bypassing physical ownership.

Finally, foothold reach admission predicts contact-time hip position from the actual pre-contact
support path plus the admitted heading's hip orbit. Its displacement is bounded by support speed and
the current cadence horizon instead of a fixed 1.5 cm. Reach must be valid from both the current hip
and the predicted contact hip, preventing a favorable prediction from admitting a target that is
unreachable before support motion completes.

Repeat the same two fresh W-to-diagonal sessions through nine successful commits. No curved plan may
fail minimum advance solely because `legacyStart` is smaller than admitted advance. Every accepted
turn touchdown must satisfy the 10-degree/0.75-rad/s/0.12-m/s readiness limits; support target speed
must remain at most 0.15 m/s; all commits must decrease error with zero reference-rotation deltas;
and neither run may abort, cross lanes, exceed 4 cm drift, or visibly scrub a planted foot. Follow
each completed turn with at least four zero-yaw steps and the validated stop to confirm the turn did
not leave a poisoned straight-gait state.

**Slice 2e implementation (2026-08-04): committed transport and dynamic swing admission;
validation pending.** The 2d logs showed that explicit plant ownership and live transfer load had
removed the former post-landing reset path: transfer gates passed and turn steps committed. The
remaining failures happened earlier, overwhelmingly on the outside foot. Those plans passed a
static reach test with only 3-24 mm reserve, then asked the physical sole to cover a longer curved
path while the support command continued cruising along the preceding foot-to-foot transfer chord.
Arrival misses were therefore not another contact timeout: the planner and the next swing were
using different future pelvis motion, and no gate checked whether the foot arc fit the available
trajectory time.

Successful transfer commitment now C1-reseeds a dedicated support-transport Hermite segment. Its
start position and velocity are the exact live support command at commitment; its endpoint velocity
is projected onto the newly committed forward direction and its endpoint lies on that same tangent.
The former lateral transfer component is smoothly removed inside the bounded segment instead of
becoming the next swing's indefinite cruise direction. `[LocomotionGait]
SUPPORT_TRANSPORT_REBASE` records both velocity vectors and the discarded old-frame lateral speed.
The existing 0.15 m/s runtime speed cap and 1.75 m/s2 acceleration cap remain authoritative.

Contact-hip prediction now evaluates that exact active support segment through takeoff, swing, and
arrival, then seeds the same candidate-dependent 20-percent descent preload used at runtime. A
fixed 120 Hz forward simulation applies the runtime position gain, speed cap, acceleration cap, and
incoming command velocity. The resulting support position, support velocity, and 80-percent heading
hip orbit produce the reach-test contact hip. This replaces the old displacement clamp, which could
hide precisely the lateral transport responsible for the failures.

Turn footholds also receive dynamic target-space admission before anatomical reach projection. The
trajectory's actual smoothstep peak derivative converts swing-start-to-target distance into required
linear speed. Inside feet are limited to 0.78 m/s and outside feet to 0.62 m/s; if a candidate is
too fast, binary search removes forward advance while preserving its anatomical lane and admitted
yaw. A turn-dominant plan may consequently admit less than the straight 6 cm minimum advance and is
judged by its angular objective; the already implemented `TURN_REPLAN` landing path retains the
measured result. Straight zero-yaw footholds bypass this policy unchanged. Dynamic angular admission
measures the complete released-sole rotation (normally two committed yaw increments), not merely the
current step delta, and bounds it by the 2.25 rad/s peak sole-leveling rate. A future increase above
the current five-degree step cap therefore cannot silently exceed the available convergence time.

`[LocomotionTurnPlan]`, `[LocomotionTurnContact]`, `[LocomotionTurnAbort]`, and turn-commit records
now identify the swing side and whether it is the outside foot. They report requested/admitted swing
distance and peak target speed, speed limits, achieved physical linear/angular peaks, predicted and
actual contact hip, their error vector, and predicted contact support velocity. The arrival and
touchdown timeouts remain safety gates and were not enlarged.

Dependency-inclusive Debug builds pass for `Sandbox` and `Runtime`, and `git diff --check` is clean.
Runtime validation should first confirm that four straight start/stop steps remain unchanged. Then
repeat both turn directions: every outside plan above 0.62 m/s must show
`constraint=swing-linear-speed` and a reduced admitted distance/advance; contact-hip error should
shrink materially from the former 5.5-15.7 cm lateral range; support rebase output must preserve its
incoming velocity at the segment start and converge to the committed tangent; and no arrival or
touchdown abort should occur. If a failure remains, the new side/speed/hip record is the evidence
for adjusting a physical budget rather than another phase timeout.

**Slice 2f implementation (2026-08-04): paired turn budgets and inside-foot convergence;
validation pending.** The first 2e run validated outside-foot dynamic admission: ten of eleven
outside plans reduced advance at the 0.62 m/s target-speed limit and none aborted. All six remaining
nonzero-yaw aborts belonged to the inside foot. Four stopped in `ARRIVAL` with 8-11 cm remaining
horizontal error and 22-28 degrees of sole error; two reached `TOUCHDOWN_WAIT` near the admitted
foothold but did not satisfy an unrecorded final motion gate. Static reach was not the separator:
contact-hip prediction was within roughly 0.4-4 cm and failed inside plans were below the independent
0.78 m/s target-speed ceiling. The failed pair instead combined a heavily shortened outside step
with an immediately full-advance, full-yaw inside step while the pelvis resumed its 0.10 m/s
transport. `ARRIVAL` then held the trajectory at `t=0.70`, so the controller asked the sole to settle
while deliberately retaining the final 30 percent of its trajectory.

An accepted outside turn plan now publishes one persistent budget for the immediately following
inside plan. Its advance scale is the outside plan's admitted/requested advance ratio, bounded to
0.25-1.0. Its yaw scale is `0.55 + 0.45 * advanceScale`, bounded to 0.55-1.0. The next inside plan
applies both scales before dynamic speed and reach admission, reports `constraint=turn-pair-budget`
unless a stricter linear-speed or reach constraint wins, and consumes the budget exactly once. A
direction reversal, unexpected foot role, stop, abort, or fresh start clears the pending pair, so a
stale budget cannot leak into another turn. Straight zero-yaw planning never reads it.

When that paired inside plan is admitted, the active support transport is C1-reseeded from its exact
current command position and velocity toward a 0.05 m/s committed-forward tangent. The horizon
covers predicted takeoff plus swing and arrival, and the existing support speed/acceleration limiter
remains authoritative. Contact-hip prediction therefore evaluates the same slower incoming pelvis
curve that runtime will use. `[LocomotionGait] SUPPORT_TRANSPORT_PAIR` records the scale pair and
incoming/outgoing velocity; turn-plan and abort records carry the applied/latched pair flags,
scales, conditioned support speed, and final arrival trajectory coordinate.

Inside-turn `ARRIVAL` now performs a bounded hover-height crawl from trajectory `t=0.70` to at most
`t=0.82` over the existing timeout. It does not enlarge the timeout or change sole readiness. If
readiness succeeds, descent continues from the reached coordinate and blends height continuously
from hover to the foothold, avoiding a target jump. Outside and straight trajectories retain the
existing path exactly. Finally, a turn touchdown timeout emits one complete
`[LocomotionTurnTouchdown] action=TIMEOUT` snapshot for contact, normal, vertical speed, horizontal
and vertical target error, sole error, angular speed, and horizontal speed. The 10-degree,
0.75-rad/s, 0.12-m/s, contact-normal, vertical-speed, and target-error thresholds are unchanged.

Dependency-inclusive Debug builds pass for `Sandbox` and `Runtime`, and `git diff --check` is clean.
Runtime validation should repeat both mirrored W-to-diagonal turns from a fresh settled launch. An
outside plan with reduced advance must show `latched=yes`; the immediately following inside plan
must show `applied=yes`, proportionally reduced advance/yaw, `support=0.050mps`, and no intervening
reset. Every such inside step must pass `ARRIVAL` and touchdown. If a touchdown still times out, the
new final gate snapshot—not a longer timeout—selects the next fix. After both turns complete with
zero stand-ups, run at least four zero-yaw steps and the validated stop before advancing to Slice 3.

**Slice 2g implementation (2026-08-04): measured-state inside-foot command governor; validation
pending.** The 2f rerun ruled out paired geometry, predicted hip position, and joint reach as the
remaining primary failure. Seven recurring paired inside-foot aborts all applied the reduced pair
budget and 0.05 m/s support transport. Their planned swing speeds were only 0.40-0.54 m/s, yet the
physical sole reached 1.09-1.22 m/s; planned angular speeds were 0.43-2.25 rad/s while measured peaks
reached 10.46-14.44 rad/s. Arrival and touchdown hip prediction was generally within 2 cm, reach
shortfall and hip-envelope clamp were zero, and the knee retained roughly 45-47 degrees of bend.
The failed touchdown snapshots were marginal and single-gate: 10.6 versus 10 degrees sole error,
0.122 versus 0.120 m/s horizontal speed, or 0.196 versus 0.120 m/s horizontal speed. The remaining
problem was therefore command tracking and braking, not another foothold budget.

The inside turn now owns a persistent measured-state trajectory clock from `SWING` through
`DESCENT`. Wall-clock phase progress is only a ceiling. The command accelerates at no more than
2.5 m/s2, respects the already admitted swing-speed budget up to a hard 0.80 m/s ceiling, and brakes
toward the next stopping endpoint. It advances normally while the measured sole is within 2.5 cm
of the current command, slows continuously between 2.5 and 7 cm, and stops advancing beyond 7 cm so
the physical foot can catch up. A bounded binary search converts the per-frame distance budget to
trajectory progress. The position-command speed is retained across phase boundaries instead of
being recreated from wall-clock progress.

Inside `ARRIVAL`, the 2f hover crawl remains, but its command is governed by that same clock. The
transition to descent now requires the complete already-computed stable-pose proof--horizontal
foothold error, hover height, sole alignment, no contact, and the configured stable interval--rather
than one frame of sole alignment. During descent, touchdown progress is derived from governed
trajectory progress. Position feedback remains active through touchdown wait; only velocity
feed-forward fades during descent. This fixes the former behavior where all task-space correction
vanished exactly when the lagging sole needed to brake onto the foothold. Ordinary straight,
outside-turn, and cancellation trajectories retain their prior wall-clock path exactly.

Sole orientation now has an equivalent world-space governor for the inside foot. Its angular
command accelerates at no more than 12 rad/s2, is limited to 0.75-2.25 rad/s according to the
admitted plan, brakes against remaining command angle, and slows when the measured sole trails the
world command by 8-20 degrees. The rate-limited world command is converted each frame through the
measured knee into a local ankle target. That compensated local target is applied directly rather
than filtered a second time; hip and knee commands retain their existing response filter. Ankle
envelope clamping remains authoritative and is now measured explicitly.

The existing arrival, descent, and touchdown safety windows are not enlarged. A governed swing or
descent that cannot reach its milestone inside its nominal duration plus the existing relevant
safety window aborts with `[LocomotionTurnGovernor]`. Successful contact and abort records now
include trajectory progress, linear command speed, measured tracking error, angular command speed,
world-sole tracking error, and ankle-envelope clamp. Runtime validation must show that paired inside
steps no longer exceed their admitted physical speed by the former factor, reach full arrival
stability before descent, and satisfy touchdown without a stand-up. Repeat both mirrored turns,
then four zero-yaw steps and the validated stop before advancing to Slice 3.

Dependency-inclusive Debug builds pass for both `Sandbox` and `Runtime`, and `git diff --check` is
clean. This is compile-time acceptance only; Slice 2g remains runtime-validation pending.

**Slice 2h implementation (2026-08-04): bounded ankle compensation and path-consistent timing;
validation pending.** The 2g run produced twelve aborts, all on the inside foot: six governed
`SWING` timeouts and six `ARRIVAL` stability timeouts. The visible shake was controller chatter,
not a weak or unreachable leg. Failed inside ankles repeatedly carried 12-22 degrees of motor error
while the opposite ankle was generally within 2-4 degrees. Their commanded twist reversed during a
monotonic swing, every swing timeout lost 7.5-12.6 degrees to the ankle envelope, and measured sole
angular speed reached 9-14.5 rad/s while the world command was only about 0.1-0.45 rad/s. Hip clamp
and reach shortfall remained zero, with 44-48 degrees of knee bend. The terminal ankle control was
therefore the first inconsistent layer.

The exact measured-knee cancellation introduced an algebraic high-bandwidth loop: knee motion
changed the child-local ankle target, the 16 Hz position motor moved the light ankle/foot bodies,
and that movement changed the next measured-parent correction. Slice 2g then applied this moving
local target directly. Slice 2h instead constructs the nominal ankle target beneath the filtered
hip/knee command that the motor will actually receive, compares it with exact measured-parent
compensation, and admits at most six degrees of that correction. The resulting target still passes
through the authored ankle envelope. The final local ankle command accelerates at no more than
20 rad/s2, is rate-limited to 1.75-3.25 rad/s according to the admitted sole budget, brakes against
its remaining local angle, and is clamped once more after interpolation. Hip and knee filtering,
motor frequency, damping, torque, and all authored joint limits remain unchanged.

After local rate and envelope limits, the controller reconstructs the sole orientation that the
current physical parent plus bounded local motor command can actually produce. That reachable
orientation becomes the next frame's world-governor state, so the outer loop cannot integrate
through an ankle boundary and repeatedly demand an impossible pose. Thresholded
`[LocomotionTurnAnkle]` records report requested/admitted parent compensation, envelope loss,
per-frame local command step, local command rate, reachable-world residual, and physical sole
tracking. Contact and abort records retain the same values.

The six swing timeouts also exposed a separate clock mismatch. Their final physical tracking error
was only 1.8-3.1 cm, but the deadline was derived from cadence while the new governor limited motion
along the full vertical-plus-horizontal arc. The governed deadline now samples that 3D path at 24
segments, solves its acceleration-limited minimum traversal time using the same 2.5 m/s2 acceleration
and 0.45-0.80 m/s command-speed bounds, then takes the larger of that derived deadline plus an
80 ms tracking reserve and the existing safety deadline. `[LocomotionTurnGovernor]` reports sampled
path length, minimum duration, and final deadline on failure.

Finally, the arrival crawl previously first reached `t=0.82` on the same frame as its unchanged
timeout, making the 30 ms full-pose stability proof structurally unavailable. The crawl now reserves
at least 60 ms inside the existing arrival window, including two frames beyond the configured
stability duration. No arrival threshold or timeout was enlarged. Validation must show materially
smaller inside-ankle motor error and sole angular peaks, no repeated twist reversals, and no swing or
arrival abort. Repeat both mirrored turns before the four zero-yaw steps and controlled stop.

Dependency-inclusive Debug builds pass for both `Sandbox` and `Runtime`, and `git diff --check` is
clean. This is compile-time acceptance only; Slice 2h remains runtime-validation pending.

**Rejected Slice 2i experiment (2026-08-04): command-space leg shaping and moving-leg motor
profile; reverted.** The 2h rerun kept the final local ankle command inside its intended bounds,
but the remaining failures still coincided exactly with physical inside-leg shake. In a representative
failure, the local ankle moved only 0.39-0.94 degrees per diagnostic interval, ankle-envelope loss was
zero after TAKEOFF, and reachable-world residual stayed near 0.3-1.6 degrees. The physical sole still
alternated through 12.4, 7.4, 18.5, 7.0, and 26.3 degrees of tracking error. At the same time, a hip
command moving from 26.3 to 32.6 to 30.9 degrees produced physical poses of 15.2, 38.0, and 20.2
degrees, while ankle motor error reached 16.3 degrees. Hip and ankle torque ratios remained below
roughly five and one percent respectively, with no reach shortfall or terminal envelope clamp. The
governed path had a 0.372-second calculated minimum inside a 0.680-second deadline but reached only
`trajectoryT=0.163`. The stand-up was therefore downstream recovery from a swing timeout: coupled
servo ringing repeatedly left the tracking tube and stopped the command clock.

The forward-walking horizontal task-space correction was already active on the inside turn. The
runtime scene uses 0.60 proportional gain, 100 ms of target-velocity lead, and a 6 cm cap, so adding
another measured-joint correction would have duplicated the powered-ragdoll servos. Slice 2i instead
keeps the same low-frequency correction objective but shapes it for the governed inside foot. Its
proportional term is scaled to 65 percent of the configured gain, its admitted magnitude is capped at
4 cm, and a persistent command state follows the requested bias with at most 0.12 m/s velocity and
0.80 m/s2 acceleration. Direction reversal first removes the old correction velocity. Straight,
outside-turn, planted-foot, and cancellation correction paths retain their existing behavior.

Hip, knee, and ankle commands now carry persistent angular-velocity vectors for the governed inside
leg from TAKEOFF through TOUCHDOWN_WAIT. Shortest-arc target velocity is acceleration limited; a
moving target cannot instantaneously reverse the integrated pose command. Hip command speed and
acceleration are bounded at 2.0 rad/s and 10 rad/s2, knee at 3.0 rad/s and 18 rad/s2, and the terminal
ankle at 1.25-2.25 rad/s according to the admitted yaw budget with 12 rad/s2 acceleration. The
measured-parent ankle correction remains capped at six degrees, but it is now its own slow bias with
0.75 rad/s speed and 6 rad/s2 acceleration instead of an algebraic local target. Starting these
governors in TAKEOFF removes the controller-mode seam at the first SWING frame.

The persistent world-space sole trajectory is no longer overwritten by an orientation reconstructed
from the raw measured knee every frame. Reachability is projected through the commanded hip/knee
chain for residual telemetry, while physical sole error may slow the outer governor without becoming
the outer governor's state. This preserves the admitted world trajectory as the equilibrium and
breaks the remaining measured-parent feedback loop.

Finally, only the active inside swing leg blends over 100 ms from the authored 16 Hz, damping-ratio
1.0 springs to a moving-leg profile: 10 Hz hip, 12 Hz knee, 8 Hz ankle, and damping ratio 1.2. Torque
limits and joint envelopes are unchanged. The engine reapplies authored spring settings to every
other joint every physics substep, so the profile cannot leak into stance, straight walking,
standing, or get-up control. `[LocoMotor]` now reports live spring frequency, damping, profile
weight, and measured child-to-parent angular speed. `[LocomotionTurnAnkle]`, successful contact, and
abort records report requested/admitted foot correction, correction rate, hip/knee/ankle command
rates, and motor-profile blend.

Runtime acceptance requires both mirrored turn directions to complete without an inside-foot abort,
without repeated sole-error excursions above the 8-20 degree governor band, and without alternating
hip/ankle tracking reversals. The governed trajectory must reach its arrival landmark before the
unchanged deadline. Four zero-yaw steps and the validated stop must then confirm that the transient
profile and correction state return to zero. Debug builds and static checks are recorded after the
implementation below; physical validation remains user-run.

The runtime result rejected this experiment. The inside foot stopped completing the step at least as
often as before, while the new command governors obscured the earlier validated 2h behavior. More
importantly, the added logs disproved the premise: low command rates coexisted with 5.5-7.4 rad/s
physical ankle motion, 6-21 degrees of envelope loss, and 5-16 degrees of reachable-sole residual.
The worst samples combined 9.5-10.2 cm position tracking error with 53-59 degrees of sole error and
early ground contact. The missing layer was not more temporal filtering. The point-position IK chose
one knee plane first, then assigned all remaining world-sole orientation to a terminal ankle whose
authored safe envelope is only about 27 degrees of swing and seven degrees of twist after margin.
All Slice 2i command-space shaping, motor-profile routing, and transient state has therefore been
removed. Slice 2h's bounded measured-parent compensation and local ankle governor are restored.
Measured child-to-parent angular-rate telemetry remains because it is diagnostic-only.

**Rejected Slice 2j experiment (2026-08-04): orientation-aware whole-leg allocation; reverted.**
The analytical two-bone solve used its redundant knee-bend plane instead of fixing it
to the nominal pole. For an active inside turn, it samples 25 swivel angles over plus/minus 60 degrees,
then performs two local refinement passes. Every candidate solves the hip position, clamps the hip to
its authored swing/twist envelope, composes the resulting knee world orientation, and measures the
terminal ankle orientation lost to its own envelope. Candidate cost strongly prioritized eliminating
ankle loss, then hip loss, branch motion, and unnecessary swivel. The selected swivel persisted across
frames and was limited to 240 degrees/second, preventing analytical branch flips. Straight walking,
outside turns, stance solves, cancellation, torque limits, and authored joint limits retain their 2h
behavior.

Sole orientation was explicitly secondary only while the inside foot had airborne clearance. From
trajectory `t=0.25` to `t=0.55`, its authority ramps smoothly from the released sole orientation to the
exact admitted world orientation. The ankle endpoint is constructed after that blend, so the position
and orientation tasks describe one consistent terminal transform. Exact landing orientation has full
authority before arrival or descent can begin; no touchdown threshold was relaxed.

Admission tested more than point reach. At trajectory `t=0.60`, `0.80`, and `1.00`, it predicted the
hip, reconstructed the two-bone geometry, searched the same redundant knee plane, clamped both hip and
ankle, and recorded the worst exact-sole ankle loss. If the requested footprint exceeded 0.75 degrees of
loss, a binary projection reduced horizontal advance until all three late-swing samples fit. If even
the zero-advance projection was not feasible, the plan was rejected with `constraint=ankle-envelope`
instead of entering a swing that can only fail at arrival. The plan record includes sampled ankle
clamp and contact swivel.

Runtime records separated requested and applied knee swivel, exact versus currently relaxed ankle
loss, orientation priority, relaxed orientation angle, normalized ankle swing reserve, and signed
twist margin. `[LocoMotor]` retains relative angular speed and additionally reports Jolt's actual
twist/swing joint-limit lambda converted to torque. That distinguishes a motor tracking error from a
physical limit impulse during the next run.

Runtime acceptance required both mirrored inside steps to show a smooth swivel command, positive
ankle swing/twist reserve before descent, no repeated joint-limit impulse sign reversal, no violent
sole shake, and no swing/arrival abort. Then repeat four zero-yaw steps and the controlled stop to
confirm the new redundancy path is dormant outside turning. Dependency-inclusive Debug builds pass
for `Sandbox` and `Runtime`, and `git diff --check` is clean; physical validation remains user-run.

The runtime result rejected both the admission proof and the live allocation. Nine of thirteen turn
plans were denied in `WEIGHT_SHIFT` with `constraint=ankle-envelope`, so the foot never entered
`TAKEOFF`. The 0.75-degree admission gate compared a hypothetical settled configuration with a
controller that begins from a measured pose and filters hip, knee, ankle, and swivel at different
rates. A plan predicted at 0.2 degrees of ankle loss and 25 degrees of swivel began its real swing at
only 1.9 degrees of applied swivel and 9.2 degrees of ankle loss. The predictor was therefore not a
valid routing invariant.

The four plans admitted by that gate also disproved ankle feasibility as the root cause. Two failed
with zero final ankle-envelope loss and positive swing reserve, while horizontal sole error remained
5.3-6.1 cm and physical sole angular speed peaked at 6.8-9.2 rad/s. One right ankle was commanded to
-3.0 degrees of twist while the measured ankle crossed to +11.5 degrees with 22.8 degrees of complete
motor error. Slice 2j's hard admission, target projection, early orientation relaxation, and applied
knee swivel are removed. Its sampled admission loss and best-swivel calculation remain
counterfactual telemetry only; they cannot change a foothold or any joint target. Slice 2h runtime
behavior is restored.

**Slice 2k implementation (2026-08-04): commanded-FK closure diagnostic; runtime validation
pending.** Before choosing another controller, every active turn frame now reconstructs the sole pose
from the exact hip, knee, and ankle commands written to the powered ragdoll. It records three poses:
the corrected desired sole, that commanded-joint forward-kinematics sole, and the measured physical
sole. Position and orientation error are split into desired-to-commanded `ik`, commanded-to-physical
`motor`, and desired-to-physical `total` terms. `[LocomotionTurnFK]` emits the complete vectors and
magnitudes every 100 ms; successful contact and abort records retain the final split.

This diagnostic is read-only. If `ik` is already large, the analytical solve, frame composition, or
independent command filtering is internally inconsistent. If `ik` remains small while `motor` grows,
the powered-ragdoll chain is failing to follow a valid command and the next controller must address
coupled physical tracking. The retained counterfactual admission clamp/best swivel and Jolt limit
torques allow either result to be distinguished from a genuine joint-limit collision.
Dependency-inclusive Debug builds pass for `Sandbox` and `Runtime`, and `git diff --check` is clean.
Physical classification remains runtime-validation pending.

**Slice 2l implementation (2026-08-04): coherent governed-sole command; runtime validation
pending.** Slice 2k showed that the inside footprint was admitted, but the controller subsequently
created two independent errors. The analytical target was filtered separately at the hip, knee, and
ankle, so the final joint triplet no longer forward-kinematically reproduced the desired sole. The
powered ragdoll then lagged that already-inconsistent command. In the worst recorded TAKEOFF sample,
desired-to-commanded closure reached 12.4 cm while commanded-to-physical error reached 11.5 cm; the
two temporarily cancelled to a misleading 5 cm total error before diverging during swing.

The governed inside swing now uses one authority hierarchy instead of blending competing controller
outputs. Its Cartesian sole trajectory remains acceleration limited and subordinate to measured
physical tracking. TAKEOFF lift and contact-recovery lift ramp continuously in task space rather than
jumping by five to eight centimetres on the first frame. The analytical two-bone solve writes the hip
and knee solution for that admitted intermediate sole pose directly; it no longer applies a second,
independent joint-space response filter. The terminal ankle is derived from the exact commanded knee
orientation, then passed through the existing authored envelope. Measured knee motion is retained as
feedback telemetry for the outer governor but is no longer injected into the ankle motor target, so
the ankle cannot chase physical parent lag while the parent simultaneously chases another command.
Straight walking, outside turns, planted-leg solving, cancellation, joint envelopes, motor torque,
contact gates, and all abort gates retain their existing routes.

`[LocomotionTurnFK]` identifies `controller=coherent`; its `ik` term is the decisive invariant for this
slice. A valid run should keep desired-to-commanded position closure near numerical/envelope residual
rather than the previous 2-12 cm range. `[LocomotionTurnAnkle] controller=coherent` reports measured
parent disagreement explicitly as unapplied feedback, alongside the exact local command step,
command rate, envelope loss, attainable world residual, and physical sole tracking. The physical
`motor` term may initially remain nonzero, but it must converge without the alternating 3-7 rad/s
inside-ankle motion seen previously. Runtime acceptance requires mirrored inside steps to enter
TAKEOFF, complete ARRIVAL/DESCENT without reset, and preserve the existing 2.5 cm/10 degree arrival
gate. Four zero-yaw steps and the controlled stop must remain unchanged.
Dependency-inclusive Debug builds pass for `Sandbox` and `Runtime`, and `git diff --check` is clean;
physical validation remains user-run.

**Slice 2m implementation (2026-08-04): physical-chain IK and shared joint-rate admission;
runtime validation pending.** Slice 2l removed the independently filtered hip/knee targets and
measured-parent ankle cancellation, which made the remaining conflict measurable. The first coherent
TAKEOFF samples closed within 5-11 mm, but later samples accumulated 5-6.5 cm of desired-to-commanded
position error despite zero hip/knee command lag and zero hip-envelope loss. At the same time, the
terminal ankle was commanded through 6.7-7.0 degrees in one frame (approximately 15 rad/s) to retain
zero world-orientation residual after an upstream parent change. The powered ankle subsequently
oscillated at 4-10 rad/s with 14-23 degrees of physical tracking error. Mirrored logs showed the same
failure, and outside turn ankles also exhibited it even though their less strict arrival route did not
immediately abort.

The imported skeleton and the ragdoll are deliberately not rewritten to match one another. Skeleton
`localT` is animation/skin geometry; changing it at runtime would alter the rendered hierarchy without
rebaking inverse bind matrices, constraints, or animation. Instead, each leg capture records the live
ragdoll hip-to-knee vector in the hip body's local frame and knee-to-ankle vector in the knee body's
local frame. Analytical lengths, hip direction reconstruction, admission telemetry, and commanded FK
now use those physical constraint-chain vectors consistently. Skeleton translations remain an explicit
fallback only when a complete live ragdoll capture is unavailable.

Turn orientation is no longer terminal-ankle-only. The existing redundant knee-plane search evaluates
the unfiltered coherent chain, and its optimum is admitted at no more than 120 degrees/second. This
allows hip orientation and knee plane to take a continuous share of the sole task without the previous
sampled branch jump. The old Slice 2j hard feasibility gate remains removed: knee-plane allocation can
change execution commands but cannot reject or project a footprint.

Finally, the coherent route measures the requested hip, knee, and ankle delta from the previous command
and admits one shared fraction bounded by 240, 300, and 240 degrees/second respectively. Applying one
fraction to all three joints replaces the independent temporal layers that made the ankle race ahead of
its parents. The temporary desired-to-commanded residual is therefore an explicit rate-admission result,
not an accidental IK disagreement; the existing measured-state trajectory governor waits for it to
converge. `[LocomotionTurnFK]` reports `geometry=(ragdoll,upper,lower)` and
`jointAdmission=(scale,requested hip/knee/ankle delta)`. `[LocomotionTurnAnkle]` and abort records retain
the same shared admission data. Runtime acceptance requires the raw ankle request to stop alternating,
the admitted local ankle step to remain within its 240-degree/second bound, physical sole angular error
to converge below 10 degrees, and mirrored inside steps to complete without weakening arrival safety.
Dependency-inclusive Debug builds pass for `Sandbox` and `Runtime`, and `git diff --check` is clean;
physical validation remains user-run.

**Slice 2n implementation (2026-08-04): position-primary leg closure and bounded stance width;
runtime validation pending.** Slice 2m's first physical run separated the remaining failure cleanly.
The shared joint admission reached `scale=1.000` with sub-degree requests, but the commanded FK sole
was still 4-12 cm from the Cartesian target. Error grew with the magnitude of the applied knee-plane
swivel and was predominantly lateral. On the worst inside step, a -25-degree swivel produced 11.5 cm
of desired-to-commanded position error; the measured-state governor correctly stopped the trajectory
at `t=0.197`, after which the swing timed out. A later -13.8-degree swivel retained approximately
4 cm of command-space error through ARRIVAL. The powered-ragdoll motor error was smaller than the
command-space error in both cases. Slice 2m's rate admission and physical segment capture are therefore
retained, while its runtime swivel application is rejected.

The underlying construction aligned only the physical upper-link direction with `RotationBetween`.
That shortest-arc rotation left hip twist about the upper segment unspecified, so the actual knee
hinge and lower physical link did not occupy the sampled knee plane. Slice 2n instead constructs
orthonormal frames from two vectors: the physical hip-to-knee link and the hinged knee-to-ankle link.
Mapping both source vectors to the requested upper/lower directions determines the missing axial hip
rotation. Every sampled candidate is then reconstructed through full FK, including the bounded hip,
bounded ankle, terminal-foot relation, and ankle-to-sole offset. Position is a hard primary invariant:
a candidate exceeding 3 mm of sole-position closure error is discarded before ankle margin, hip
margin, branch continuity, or knee-pole preference can affect its score.

Corrected swivel search remains counterfactual for this validation pass. Runtime explicitly commands
zero swivel, but the zero-swivel pose itself uses the corrected two-vector construction. This removes
the proven lateral command displacement while preserving `bestSwivel`, closure, and accepted/rejected
telemetry for a later controlled re-enable. `[LocomotionTurnFK]` now includes
`closure`, `accepted`, `appliedSwivel=0`, and `runtimeSwivel=disabled`; contact and abort records retain
the same candidate proof. A nonzero counterfactual swivel must never influence a joint command in this
slice. Debug drawing adds a magenta commanded-FK marker and orange-to-magenta closure segment, followed
by a cyan physical marker and magenta-to-cyan motor-tracking segment, so the two error sources remain
visually separable from the orange desired trajectory and green foothold.

Planner width is independently bounded so a wide abort/reset pose cannot perpetuate an expanding gait.
The signed anatomical lane still has the existing 10 cm minimum, while the new serialized
`gaitMaxFootSeparation` begins at 18 cm. `[LocomotionTurnPlan]` reports raw, candidate, admitted, maximum,
and limited separation, and uses `constraint=foot-separation` when this clamp is the limiting admission
result. This bound addresses the observed occasional 22 cm plan, but it is not treated as the primary
fix because the same inside-foot failure occurred with admitted separations of only 10-12.4 cm.

Runtime acceptance for Slice 2n is one straight four-step start/stop followed by mirrored diagonal
turns. Straight plans must remain inside the 18 cm bound without new aborts. During inside turns,
`appliedSwivel` must remain zero, counterfactual accepted candidates must report at most 3 mm closure,
and desired-to-commanded FK position error must converge through shared rate admission rather than
settling at a lateral offset. The red desired-trajectory marker must reach the green foothold, and no
inside step may reset through SWING or ARRIVAL. Dependency-inclusive Debug builds pass for `Sandbox`
and `Runtime`, and `git diff --check` is clean; physical validation remains user-run.

**Slice 2 runtime acceptance (2026-08-05): accepted.** The position-primary, zero-runtime-swivel
chain and bounded stance width passed the straight four-step/start-stop regression and mirrored
W-to-diagonal turns. Slice 3 therefore inherits the current 5-degree turn as its behavioral
baseline. The retained Slice 2j orientation sampler is not part of that acceptance proof: passing
steps can still report `counterfactual accepted=no` or a 180-degree hypothetical clamp before later
committing successfully. It remains diagnostic-only and must never be promoted directly into a
routing gate.

**Slice 3 rollout sequence (2026-08-05): feasibility-backed yaw begins in shadow mode.** Keep the
accepted executor and the 5-degree cap unchanged while admission is introduced in four explicit
gates:

1. Evaluate the final grounded pose for both physical constraint chains in read-only shadow mode.
   Use captured ragdoll segment geometry, the current position-primary two-vector reconstruction,
   zero applied swivel, actual authored envelopes, the fixed stance sole, and the predicted end
   heading. Report swing/stance reach, knee margin, hip and ankle clamp/reserve, FK closure, lane,
   separation, ground validity, and the first limiting constraint.
2. Compare each shadow decision with the unchanged executor. A committed step that shadow would
   reject is a false rejection; a runtime abort after a shadow-safe decision is a false acceptance.
   Either result blocks routing activation and requires the predictor to be corrected, not the
   physical gait, motors, contact gates, or joint limits to be retuned.
3. Only after the 5-degree straight and mirrored matrix contains no unexplained mismatches may the
   shadow result become authoritative. Enable it at the same 5-degree cap first, then add descending
   yaw admission while rebuilding the full grounded pose and both-leg proof at every candidate yaw.
4. Raise the cap in small increments. At every value, run fresh mirrored arcs through complete
   outside/inside step pairs and retain the zero-yaw four-step/start-stop regression. Stop increasing
   at the first routine clamp, lost reserve, crossover, anchor/drift regression, or directional
   asymmetry; responsiveness alone cannot override the weakest measured reserve.

The first implementation stage emits `[LocomotionTurnFeasibilityShadow]` once per planned turn.
`[LocomotionTurnFeasibilityMismatch]` distinguishes a shadow rejection of a step that later commits
from a shadow-safe step that later aborts, and records the action to keep shadow routing disabled.
This telemetry is observational: it cannot change `candidateAccepted`, `activeHeadingPlan`, the
foothold, a joint target, or any phase transition.

Dependency-inclusive Debug builds pass for both `Sandbox` and `Runtime`, and `git diff --check` is
clean. Runtime shadow classification remains user-run; no Slice 3 feasibility result is yet allowed
to alter admission.

**Slice 3 shadow refinement (2026-08-05): safety-buffer consumption and dynamic tracking reserve.**
The first mirrored shadow run produced 35 evaluated turn plans: 29 shadow-safe, six shadow-unsafe,
five committed false rejections, and one aborted false acceptance. Both directions also contained
one ARRIVAL abort and automatic retry. The apparent stand-up between turn segments was therefore the
validated recovery path, not only a visual cadence seam.

All five false rejections were caused by only 0.31-1.42 degrees of predicted ankle loss against the
controller's already-inset three-degree safety envelope; the corresponding authored hard envelope
was not violated. Shadow evaluation now records both values separately. An ankle pose remains unsafe
if it clamps against the authored hard limit or consumes more than 1.5 degrees (half) of the
three-degree safety buffer. Lesser consumption remains visible as reduced reserve but does not reject
an otherwise exact physical-chain pose.

The initial two physical ARRIVAL failures were inside-foot turns whose admitted target speed used
98.7 and 100 percent of the 0.78 m/s target-speed ceiling, so the first dynamic refinement required
five percent admitted-speed headroom. A follow-up mirrored run showed that this was too conservative:
of 23 shadow records, 20 were safe and all three unsafe results were tracking-reserve decisions. The
predictor had zero false acceptances and correctly anticipated the only runtime abort, but still
falsely rejected two committed steps. One of those clean steps succeeded at 0.766/0.780 m/s, proving
that admitted speed near the ceiling is not itself a sufficient failure condition. The ankle change
did behave as intended: up to 1.21 degrees of safety-buffer consumption committed with no authored
hard-limit clamp or ankle-envelope mismatch.

The next mirrored run invalidated discarded request as a gate as well. Of 35 shadow records, 13 were
classified unsafe: twelve committed false rejections, one correctly anticipated abort, and one
additional abort was a false acceptance with zero clamp loss. Intentional pair-budget and
descending-yaw target reshaping produced 0.129-0.354 m/s of loss on successful steps, so the value
does not isolate physical tracking danger. Requested, admitted, and limit speed plus clamp loss and
the former eight-percent reference remain in telemetry, explicitly marked `gate=disabled`.
Clamp loss can no longer set `constraint=swing-tracking-reserve` or change shadow safety.

That run also separated the visible posture seam from abort recovery. On both completed rotations,
the final nonzero-yaw step committed normally and was followed immediately by an unrestricted
zero-yaw step. The transition touchdown rose from 0.069-0.103 m/s on the final turning step to
0.564-0.596 m/s, then triggered plant recovery, pivot release, and a zero-support-velocity settle.
This is the observed stand-up before straight travel resumes. A committed nonzero-yaw step now arms
one latched turn-exit step. The latch is consumed only when that zero-yaw step commits; while active,
the step retains the 0.78 m/s inside-foot admission budget, 0.05 m/s conditioned support transport,
coherent swing command, turn touchdown-readiness gates, and the governed arrival/descent path.
`exitBlend=yes`, `SUPPORT_TRANSPORT_TURN_EXIT`, and the transition touchdown metrics
make the seam directly verifiable without changing the accepted yaw cap.

**Slice 3 pre-activation stabilization (2026-08-05): outside-foot initiation and explicit ankle
reserve.** The turn-exit rerun validated both conditioned exit steps: touchdown linear speed fell
from the former 0.564-0.596 m/s seam to 0.035 and 0.005 m/s, angular speed fell to 0.070 and
0.068 rad/s, and both steps committed without an exit abort or pivot release. Keep this path fixed.

The same run contained two earlier ARRIVAL aborts. Both were fresh positive-yaw attempts beginning
on the anatomical inside foot; the automatic retry that finally began on the outside foot completed
the full arc. A new turn may therefore defer, but not reject, yaw for one already-admitted inside-foot
step. The step retains its zero-yaw straight advance and all ordinary landing invariants, reports
`constraint=turn-initiation-role`, and emits `[LocomotionTurnInitiation] action=DEFER_YAW`. The desired
heading remains unchanged, so the following outside foot must carry the first nonzero admitted yaw.
This rule does not alter an established turn pair, a live turn already containing a committed yaw,
the accepted turn-exit step, or stop handling.

The only remaining false rejection consumed 1.66 degrees of the three-degree ankle safety buffer
and zero degrees of the authored hard envelope, then committed cleanly. Shadow feasibility now
expresses the soft rule as a remaining-reserve invariant: at least one full degree of the configured
safety buffer must remain. With the current three-degree inset, up to two degrees of soft-envelope
consumption is diagnostic-safe; any authored hard-limit clamp remains unsafe. Runtime activation is
still blocked. The next mirrored test must show one deferral only when an inside foot would otherwise
initiate the turn, first nonzero yaw on the following outside foot, zero turn aborts/retries, zero
unexplained shadow mismatches, both quiet exit blends, four subsequent zero-yaw steps, and the
validated controlled stop before shadow feasibility can become authoritative at five degrees.

**Slice 3 lane-floor closure (2026-08-05): final five-degree runtime edge pending validation.** The
next matrix produced 33/33 shadow-safe physical poses, no false rejection, two correctly sequenced
inside-first deferrals, three quiet exit blends, monotonic completed heading progress, and a passing
controlled stop. Its only failure occurred in `WEIGHT_SHIFT`, before takeoff or physical tracking:
the first outside-foot plan requested 1.056 m/s, advance projection reached its zero-forward floor,
and the immutable lateral lane still measured 0.622 m/s against the 0.620 m/s outside budget. The
final gate allowed only 0.001 m/s of closure, so it rejected the otherwise grounded plan and forced
one automatic retry; the later 0.570 m/s outside-foot plan completed.

Admission now measures and reports that zero-forward lane floor explicitly. The binary projector
still targets the exact speed ceiling, but a final floor no more than 0.005 m/s above the ceiling is
accepted as bounded geometric/numerical closure and emits
`[LocomotionTurnAdmission] result=LANE_FLOOR_TOLERANCE`. A larger overshoot remains rejected, emits
`result=LANE_FLOOR_EXCEEDED`, and records `action=await-descending-yaw`; it is not hidden by raising
the outside-foot budget. `[LocomotionTurnPlan]` and abort telemetry also record the floor, tolerance,
and applied/exceeded state. The 0.005 m/s allowance is less than one percent of the 0.620 m/s budget
and covers the observed 0.002 m/s floor without authorizing a materially different trajectory.

**Slice 3 runtime acceptance (2026-08-05): accepted at the fixed 5-degree cap.** The final mirrored
run contained zero turn aborts, automatic retries, restart blocks, shadow false rejections, shadow
false acceptances, or confirmed unsafe outcomes. All 22 evaluated turn poses were shadow-safe. The
inside-first request deferred exactly one zero-yaw step and the following anatomical outside foot
carried the first nonzero yaw. Both completed turns used exactly one conditioned exit step; their
touchdowns measured 0.024/0.007 m/s linear and 0.217/0.060 rad/s angular, with no visible stand-up or
recovery seam. Completed heading error converged and no lane-floor closure or exceedance was needed.
The controlled stop was not repeated in that final session, but passed the immediately preceding
matrix; the intervening patch changes only turn admission and does not alter stop routing.

This acceptance freezes the validated behavior: a 5-degree cap, outside-foot-first initiation,
read-only both-leg feasibility telemetry, bounded pair conditioning, and one quiet turn-exit step.
It does not claim that shadow feasibility is authoritative or that descending yaw and larger caps
have been validated. Those are retained as named follow-up extensions rather than silently entering
Slice 4. The current focus is now Slice 4's turn-dominant runtime validation matrix.
An abort following an unsafe prediction now emits
`[LocomotionTurnFeasibilityOutcome] result=SHADOW_REJECTION_CONFIRMED`, separating a correctly
anticipated runtime failure from the existing false-acceptance mismatch.
Dependency-inclusive Debug builds pass for `Sandbox` and `Runtime`, and `git diff --check` is clean.

**Slice 4 implementation (2026-08-05): build-verified; physical validation pending.** The unified
planner now classifies every admitted step as translation, angular, or combined. Remaining committed
heading error supplies a serialized smooth advance envelope: full advance through 15 degrees,
smooth reduction between 15 and 45 degrees, and zero commanded advance at 45 degrees and above.
The ordinary physical-turn route now remains available through 90 degrees while the accepted
per-step yaw cap stays fixed at 5 degrees. Inside-first initiation deferrals and the conditioned
zero-yaw exit step retain full translation so neither inherited Slice 3 transition is silently
reclassified as a pivot.

The same advance envelope now owns foothold construction, conditioned pair transport, and the
inter-step cruise rebase. Zero-span support curves publish zero endpoint velocity; nonzero turning
curves derive their endpoint tangent from the actual horizontal support span. Landing checks use the
admitted objective: straight translation retains the existing minimum-advance invariant, combined
steps compare against their admitted advance with the existing foot-target tolerance, and angular
steps may accept a measured zero-advance footprint while still requiring the ordinary contact, load,
sole, drift, tilt, support, and saturation gates. HOLD additionally requires signed angular progress
within a two-degree-or-40-percent tolerance before committing an angular objective.

Forward stride filtering updates for translation-bearing steps. Settled-loss reserve learning updates
only for translation-only steps, so turn-dominant combined loss cannot contaminate admission of the
conditioned straight exit. Pure angular steps freeze both forward integrators; combined steps retain
forward-error and stride adaptation while continuing to feed lateral, touchdown, drift, motor, reach,
unload, cadence-slowdown, transfer-bias, stress, and recovery feedback. Turn-plan and commit telemetry
report objective type, nominal advance, heading advance scale, requested,
admitted, and achieved advance, and independent translation/angular satisfaction. Both-leg
feasibility remains read-only shadow telemetry; descending-yaw reconstruction and cap expansion were
not enabled. Dependency-inclusive Debug builds pass for `Sandbox` and `Runtime`, and `git diff
--check` is clean. The fresh-scene mirrored 45/90-degree phase matrix remains the acceptance gate.

**Slice 4 landing-objective stabilization (2026-08-05): implementation complete; rerun pending.**
The first extensive matrix proved the physical 90-degree route but produced eight visible standing
returns. None was an abort, restart block, contact failure, or turn-exit seam. Six combined and two
translation landings finished with stable support but missed the existing 10 mm advance hysteresis by
only 2-7 mm; the landing check requested a controlled stop, and held input restarted after
`RETURN_STAND`. Sub-tolerance admitted translation is now objective-relative: a turning step carries
an independent translation objective only when its admitted advance is at least the larger of 2 cm
and the configured foot-target tolerance plus 1 cm. With the accepted 4 cm tolerance, the two 3.5 cm
plans from the run are angular rather than brittle combined objectives.

Translation-bearing landings retain the 10 mm hysteresis and add a separately named 8 mm
`OBJECTIVE_CLOSURE` band. It is evaluated only after the existing contact, center-ownership, sole,
drift, tilt, motor, and support gates; larger misses still request the controlled recovery. A
historical replay of all eight records classifies two as angular and six as bounded closure, with zero
remaining recovery requests. Landing-triggered stops now latch their cause so `STOP_COMPLETE` reports
`recovery=landing-objective` rather than the misleading `commanded`. No feasibility, heading, anchor,
or physical safety limit changed. Dependency-inclusive builds and the fresh runtime rerun remain the
verification gates.

The follow-up mirrored 90-degree run isolated one remaining positive-yaw failure at the 30-degree
angular-to-combined crossover. Its accepted contact and angular transfer were valid, but the combined
step retained 6 mm against a 30 mm translation requirement, 6 mm below the hysteresis-plus-closure
boundary; the mirrored negative-yaw step retained 16 mm and passed. A combined landing shortfall now
emits `COMBINED_REPLAN`, preserves the measured physical footprint, and retries translation from that
footprint on the next plan while retaining all angular, contact, ownership, sole, drift, tilt, motor,
and support proofs. Translation-only shortfalls remain strict controlled recoveries. This prevents a
single turn-dominant servo miss from manufacturing a stand/restart without weakening straight gait.
Dependency-inclusive Debug builds pass for `Sandbox` and `Runtime`; the fresh mirrored runtime rerun
remains the behavioral verification gate.

**Slice 4 lane-floor yaw descent (2026-08-05): implementation complete; rerun pending.** The next
matrix validated both 90-degree routes and the combined-footprint replan, but one positive 45-degree
outside step reached an immutable 0.626 m/s lateral-lane floor against the 0.620 m/s outside budget
and 0.005 m/s closure. The plan aborted before takeoff with zero achieved swing speed, then completed
only after automatic retry. Increasing the closure would merely move this binary edge.

Outside turn admission now searches downward from the angular-speed-admitted yaw when the zero-
advance lane still exceeds the existing speed boundary. It admits the largest yaw whose rebuilt lane
and released-sole rotation satisfy the unchanged linear and angular budgets, then reconstructs the
mid/end heading basis, foothold, grounding, reach, support prediction, orientation evaluation, and
shadow feasibility from that yaw. The following inside step inherits the descended yaw ratio through
the existing pair contract. `YAW_DESCENT` reports the requested/admitted yaw and before/after lane
floor; `LANE_FLOOR_TOLERANCE` remains the bounded final closure, and a plan still rejects if no
positive-yaw footprint fits. The greater-than-90-degree settled fallback and its cancellation behavior
are explicitly deferred and unchanged. Dependency-inclusive build verification is complete; the
fresh mirrored 45/90-degree runtime matrix remains the acceptance gate.

**Slice 4 exit-reserve isolation (2026-08-05): implementation complete; rerun pending.** The next
positive 45-degree route completed its yaw, but its conditioned zero-yaw exit aborted before takeoff.
Swing-speed admission reduced advance from 134 to 102 mm; the shared settled-loss reserve had reached
its 40 mm cap after an earlier combined-footprint replan, leaving only 62 mm predicted against the
scene's 80 mm minimum. Automatic retry reset that reserve, and the equivalent exit then achieved
85 mm, proving that the rejected pose was not an unsafe physical route.

Combined steps still update forward-error and stride adaptation, but only translation-only landings
may now update `_gaitSettledTrackingLoss`. The straight and conditioned-exit predictor therefore
retains evidence from comparable translation steps instead of inheriting turn-dominant loss. A new
`MINIMUM_ADVANCE_REJECTED` admission record reports planned, predicted, minimum, and reserve values
plus pose feasibility whenever this proof blocks a step. No minimum, speed, reach, landing, or turn
limit changed. Dependency-inclusive builds pass; a fresh direct mirrored 45/90-degree matrix remains
required before Slice 4 acceptance.

**Slice 4 runtime acceptance (2026-08-05): accepted.** The final direct mirrored matrix completed
positive and negative 45-degree turns and positive and negative 90-degree turns with zero turn aborts,
automatic retries, restart blocks, lane-floor exceedances, minimum-advance rejections, landing
recoveries, feasibility mismatches, confirmed shadow rejections, or logged failures. Every completed
turn converged to zero remaining heading error and committed its conditioned zero-yaw exit; exit
touchdown horizontal speeds measured 0.010, 0.013, 0.035, 0.018, and 0.021 m/s across the five tested
sequences.

The matrix exercised the new descending-yaw path on the previously failing positive outside step.
Admission reduced yaw from 5.000 to 2.426 degrees and rebuilt the immutable lane floor from 0.646 to
0.625 m/s, inside the unchanged 0.620 m/s budget plus 0.005 m/s closure. Four `COMBINED_REPLAN` and
the ordinary angular `TURN_REPLAN` records preserved stable measured footprints and continued without
a standing return; eleven `OBJECTIVE_CLOSURE` records likewise completed through the accepted stable-
contact band. These are expected objective-relative replans, not recovery failures.

The controlled stop was not repeated in this final session. It remains inherited from the preceding
validated straight/stop regression because the intervening changes affect yaw reconstruction,
objective-relative landing policy, and settled tracking-reserve ownership, not stop routing. Closing
the final test process while a straight step was active did expose a Vulkan Memory Allocator assertion
for unfreed dedicated allocations after `HealthSystem::OnDestroy`. Track that shutdown/resource-
cleanup defect separately; it did not occur in locomotion control and does not block Slice 4's runtime
acceptance. Slice 5 subsequently accepted retarget, stop, and physical reversal behavior, and Slice 6
removed the fallback before closing the full regression matrix.

**Slice 5 retarget, stop handoff, and physical reversal (2026-08-05): accepted.** Live desired-heading
changes now retain an accumulated 0.5-degree telemetry reference and
emit `[LocomotionRetarget]` records containing the gait phase, immutable-active-step status, routing
decision, old/new desired headings, committed-heading error, admitted yaw, and turn progress. A
retarget remains queued for the next foothold when a step is already admitted, regardless of its
angle. Commands above 90 degrees no longer synthesize a stop request, and fresh large-angle starts no
longer enter `TURN_BLEND`; both remain in the physical step planner under the accepted per-step yaw,
contact, tracking, reach, lane, separation, and support limits.

Input release now has an explicit heading handoff. Release during `WEIGHT_SHIFT` or `INTER_STEP`
enters the controlled stop directly without admitting another foothold. Release during `TAKEOFF` or
the first half of `SWING` returns the released foot to its exact previous plant pose while a separate
smooth heading curve unwinds continuously from the target already issued to the immutable step-entry
heading. Release in the second half of `SWING`, `ARRIVAL`, or `DESCENT` retains the admitted late-step
yaw while using the existing shortened safe landing. Later contact and transfer phases finish their
already-admitted step. Every route publishes the final motor-owned heading as the new standing and
committed basis before `STOPPING`, clears turn-pair/exit/cancellation state, and emits
`[LocomotionTurnHandoff]`; it never rotates a planted reference or calls `RotateRagdollYaw`.

Dependency-inclusive Debug builds pass for `Sandbox` and `Runtime`, and `git diff --check` is clean.
Runtime acceptance now requires release and sub-90-degree retargets in both turn directions during
`WEIGHT_SHIFT`, early/late `SWING`, `ARRIVAL`/`DESCENT`, `TOUCHDOWN_WAIT`/`SETTLE`, `TRANSFER`/`HOLD`,
and `INTER_STEP`. Each release must produce exactly one stop rebase, reach `IDLE` without retry or
restart blocking, and restart from the rebased heading. Each live retarget must leave the admitted
foot pose unchanged and make the following plan consume the newest desired heading. Only after this
matrix passes should exact-180 turn-side latching and physical reversal admission be enabled.

The completed release matrix exercised `WEIGHT_SHIFT`, `TAKEOFF`, early and late `SWING`, `ARRIVAL`,
`DESCENT`, `SETTLE`, `TRANSFER`, `HOLD`, and `INTER_STEP`. The two hard-to-hit double-support routes
were repeated with temporary one-shot phase triggers. Both `WEIGHT_SHIFT` attempts immediately
published `mode=double-support` and completed with 0-1 mm stop-settle drift; both `INTER_STEP`
attempts did the same with 0-1 mm drift. All four reported correct contact edges, 1.1-1.3-degree
final tilt, at most 0.08 motor ratio, and `ready=IDLE`. Across the matrix there were zero locomotion
aborts, automatic retries, restart blocks, stop-check failures, stop-completion failures, or
`TURN_BLEND` calls. The temporary trigger state, inspector controls, command injection, and trigger
telemetry were removed after acceptance; the ordinary retarget and heading-handoff telemetry remains
for reversal validation.

Exact opposites now latch one deterministic yaw sign from the first available outside foot. The latch
survives until the requested target changes or heading converges, removing the `atan2` sign ambiguity
at 180 degrees without changing the immutable admitted-step contract. `[LocomotionReversal]` reports
physical-route begin, exact-opposite latch, every committed yaw-bearing step, convergence, and any
fallback transition. With the accepted 5-degree cap, an exact reversal requires 36 yaw-bearing
commits, plus any conditioned inside-first or zero-yaw exit step required by the existing planner.

During Slice 5 the temporary settled `TURN_BLEND` remained only as a guarded recovery. If the
authoritative planner rejected a foothold while a large-angle physical reversal was active, logical
ownership of the not-yet-released swing sole was restored and the validated double-support stop ran
before that fallback. Shadow envelope feasibility remained diagnostic and could not request it.
Slice 6 removes this last direct-rotation recovery path entirely.

The acceptance run changed a live command by exactly 180 degrees during `TRANSFER`. The pending input-
release stop canceled cleanly, the reversal selected `CW` with `LEFT` as the first outside swing, and
37 yaw-bearing commits reduced remaining error monotonically from -180 to 0 degrees. Thirty-three
commits admitted the full 5-degree cap; tracking and pair conditioning reduced three late commits to
4.122, 4.206, and 1.672 degrees. Every commit reported decreasing error and satisfied its angular
objective, followed by the conditioned zero-yaw exit step and continued straight walking.

Across the reversal there were zero fallbacks, `TURN_BLEND` calls, locomotion aborts, automatic
retries, restart blocks, feasibility mismatches, or saturation failures. Minimum foot separation was
0.100 m, minimum reach reserve was 0.031 m, maximum stance/plant drift was 0.018/0.035 m, and maximum
touchdown horizontal speed was 0.119 m/s. Twenty-seven `TURN_REPLAN` records accepted the actual
turn-dominant footprint, and three `OBJECTIVE_CLOSURE` records accepted stable contact; these are the
intended objective-relative reconciliation paths, not recovery failures. Together with the previously
accepted mirrored 45/90-degree, retarget, and phase-by-phase release matrices, this closes Slice 5.
The opposite exact-180 initial latch and representative mirrored 135-degree commands remain explicit
Slice 6 regression cases rather than Slice 5 blockers.

**Slice 6 fallback removal and final regression (2026-08-05): accepted.** The
settled-turn executor, `RotateRagdollYaw` physics API, runtime turn timing/target state, inspector speed,
serialized tuning, scene value, and reversal-fallback target state have been removed. Exact-180
latching and large-reversal telemetry now use a fixed 90-degree classification boundary and remain
part of the physical planner; no removed fallback setting controls physical admission.

An authoritative reversal-plan rejection now restores the captured swing sole to its existing plant,
logs `[LocomotionReversal] event=PLAN_REJECT`, and enters the validated double-support stop. The
rejected desired direction is recorded as a restart guard: the gait cannot repeatedly retry the same
unadmittable held command after reaching `IDLE`. Releasing input or changing direction by at least
0.5 degrees clears that target-specific guard. If the stop itself aborts, the same guard remains in
force instead of degrading into an automatic retry loop. Ordinary non-reversal abort recovery keeps
its existing bounded retry policy.

Dependency-inclusive Debug builds pass for `Sandbox` and `Runtime`, and `git diff --check` is clean.
Final runtime acceptance required the 44-step straight/controlled-stop regression, at least 20 plants
on each constant arc, mirrored 45/90-degree changes, the opposite exact-180 latch, mirrored 135-degree
changes, and representative phase retarget/releases. No session may emit `TURN_BLEND` or call direct
ragdoll rotation; the code no longer contains either route. A real planner rejection, if encountered,
must stop once and remain idle for the unchanged command until input release or direction change.

The final regression ran 156 committed steps over 244.61 seconds. It included a live exact-180
reversal, a positive 90-degree turn, and two negative 135-degree reversals, for 19 positive and 95
negative yaw-bearing commits. The exact reversal latched `CW` with the left foot outside first and
converged in 37 yaw-bearing commits. Both 135-degree sequences converged in 28 commits. All three
large reversals emitted monotonic `PROGRESS`, reached `heading-converged`, consumed their conditioned
zero-yaw exit, and continued walking. Retargets exercised `HOLD`, `SETTLE`, and `DESCENT`; three
release-to-new-direction transitions canceled their pending stops cleanly before the final commanded
release completed the standing return.

There were zero `PLAN_REJECT` records, locomotion aborts, automatic retries, restart blocks, stop
failures, feasibility mismatches, `TURN_BLEND` calls, or fallback records. Every angular objective
passed and every heading commit reported decreasing remaining error. Seventy-three `TURN_REPLAN`
records accepted the actual turn-dominant footprint. One combined step reported translation closure
shortfall, emitted the intended `COMBINED_REPLAN`, retained its successful 5-degree angular progress,
and recovered translation on the following step without adaptation or stability failure. Minimum
foot separation remained 0.100 m and minimum reach reserve remained 0.018 m.

`STOP_COMPLETE` reported correct contact edges, converged step length and period, 0.014 m stop-settle
drift, 0.018 m historical relevant drift, 3.7-degree peak tilt, 0.07 peak motor ratio, and 1.5-degree
final tilt at `IDLE`. This closes Slice 6 and the bounded continuous-turning implementation plan.

Each slice records requested/admitted/achieved yaw, remaining heading error, candidate/admitted/
actual sole pose, swing and stance joint margins, minimum foot separation, support-target position
and velocity steps, contact edges, plant-anchor ownership, drift, tilt, motor saturation, and the
existing right-side pivot-release asymmetry. Turning work must not relax the accepted contact,
12-degree ownership-sole, 4 cm persistent-slip, 30-degree tilt, correction, load-hysteresis, or temporary
support-speed limits.

#### Final acceptance criteria

- straight input still reproduces the accepted continuous gait and controlled stop, with no change
  attributable to zero-yaw planning;
- ordinary direction changes through 90 degrees remain in the physical gait instead of stopping,
  rotating the complete ragdoll, and restarting;
- ordinary 135- and 180-degree changes likewise remain physical, with exact opposites selecting one
  deterministic outside-first direction and making bounded per-step progress;
- completed-step heading error converges monotonically apart from a logged live retarget;
- left and right turns preserve anatomical lane sign and minimum foot separation with no crossover;
- stance material anchors remain fixed in world space until their existing ownership handoff;
- the swing foot lands with the admitted yaw and ground-aligned pitch/roll without routine joint-
  envelope clamping;
- support position and velocity targets remain continuous at the same phase boundaries validated by
  straight walking;
- turn-dominant steps are judged by their admitted angular objective and do not poison forward gait
  adaptation;
- release or retarget during every major gait phase reaches either continued stable walking or the
  validated standing return without a restart loop;
- a real reversal admission failure restores double-support ownership, completes the validated stop,
  and uses at most one guarded settled fallback for the still-matching target;
- the ordinary turn path contains no direct ragdoll transform rotation and requires no physics-
  engine change.

The geometry follows the future-footprint principle described by Johansen's [*Automated
Semi-Procedural Animation for Character Locomotion*](https://runevision.com/thesis/rune_skovbo_johansen_thesis.pdf),
while the physical division between per-step heading, predictive foot placement, and balance control
follows the compatible structure in Coros, Beaudoin, and van de Panne's [*Generalized Biped Walking
Control*](https://www.cs.ubc.ca/~van/papers/2010-TOG-gbwc/paper.pdf). A later balance refinement may
add an inverted-pendulum or capture-point offset to the nominal steering footprint before feasibility
projection. That is deliberately outside the first turning implementation; current support and
landing feedback remain unchanged until the nominal steering behavior passes.

### Unified assisted turning speed - authoritative implementation plan

**Design decision (2026-08-11): planned.** The bounded physical-turn planner above proves safe
footprint construction, contact ownership, mirrored direction changes, retargeting, cancellation,
and exact-reversal convergence. It does not provide acceptable gameplay response. Increasing the
controller yaw ceiling alone cannot close that gap because heading is still serialized behind a
complete physical step and each anatomical sole must recover the accumulated rotation from its
previous plant.

The first 20-degree-ceiling runtime captured on 2026-08-11 demonstrates the limit. An exact
180-degree command began at `14:00:43.324` and converged at `14:01:08.436`: 25.112 seconds and 15
yaw-bearing commits. The first outside foot admitted 20 degrees, but later plans admitted values
between roughly 6 and 18 degrees under `swing-angular-speed` and `turn-pair-budget`. Completed turn
steps took approximately 1.4-2.6 seconds despite the 0.50-second nominal cadence. A 45-degree request
would therefore be descended by the same admission constraints and would not provide the required
order-of-magnitude improvement.

The new milestone uses one unified scheduled-heading curve for every nontrivial direction change
from 0 through 180 degrees. Physical heading control and physical footsteps remain active, while a
bounded whole-ragdoll yaw correction closes only the lag between the scheduled heading and the
heading that physics has achieved. Assistance increases naturally with tracking lag instead of
switching on at one large-angle threshold. Small arcs should need little correction; a rapid reversal
will use most of the available correction.

This is not a return to the removed settled `TURN_BLEND` behavior:

- it runs concurrently with the physical gait rather than stopping, rotating, and restarting;
- it pivots around the currently loaded stance contact rather than the midpoint of two settled feet;
- it follows one authoritative heading schedule and never adds an independent visual angle to the
  physical turn;
- it retains real swing, touchdown, load transfer, minimum separation, and fall response;
- it rotates every affected world-space gait reference by exactly the same incremental correction;
- it uses one turn beat through 90 degrees and an outside/inside pair above 90 degrees, so the feet
  visibly participate in the direction change.

The assisted path deliberately supersedes the earlier prohibition on direct ragdoll yaw during the
ordinary turn path. The prohibition remains the regression baseline until each slice below passes.
The new permission is narrow: incremental yaw about an owned physical stance pivot, driven by the
scheduled-heading residual, while a turn plan is active. It does not permit arbitrary root snapping,
translation warping, or rotating cached references independently of the bodies.

#### Gameplay timing contract

Heading time and gait-ready time are separate measurements. `heading complete` means the physical
pelvis forward is within 2 degrees of the scheduled target and its yaw rate is below 30 degrees per
second. `gait ready` additionally means the required turn contacts and load handoff are valid and a
step in the new direction may be admitted.

| Requested change | Heading target | Intended foot behavior |
|------------------|----------------|------------------------|
| 0-15 degrees | 0.15-0.20 s | ordinary walking arc; residual assistance only |
| 15-45 degrees | 0.20-0.35 s | one short turning step |
| 45-90 degrees | 0.35-0.55 s | one assisted pivot step |
| 90-120 degrees | 0.50-0.70 s | one large or two overlapping turn beats |
| 120-180 degrees | 0.70-0.90 s | outside-foot then inside-foot assisted turn |

The initial schedule is:

```text
turnDuration = clamp(abs(requestedYawDegrees) / 200.0, 0.15, 0.90)
```

The curve must ease angular velocity at both ends and remain deterministic across render frame
rates. The 180-degree target reserves up to 0.20 seconds after heading completion for the closing
contact, making 1.10 seconds the first full gait-ready target and 1.20 seconds the hard acceptance
limit. Once repeatable, tuning may attempt a sub-one-second full handoff without weakening contact or
load proofs.

These are gameplay targets rather than claims that all humans turn at one fixed rate. Human
180-degree turns use step, pivot, and mixed strategies, commonly expressed with a small number of
turning steps rather than many incremental shuffles. The engine targets a responsive mixed
step-and-pivot style. See [Faria et al.](https://pmc.ncbi.nlm.nih.gov/articles/PMC5088107/) and
[Bonnyaud et al.](https://pmc.ncbi.nlm.nih.gov/articles/PMC5431013/) for the relevant strategy and
step-pattern descriptions.

#### Authoritative heading and assistance contract

The controller must keep four values distinct:

- **desired heading:** latest live player intent;
- **scheduled heading:** the global eased heading for the current turn time;
- **physical heading:** measured pelvis heading after the physics step;
- **assisted yaw:** the bounded incremental correction applied to close scheduled-versus-physical
  lag during that update.

The scheduled heading is the only angular objective. The joint heading motor, foot planner, and
assisted yaw all consume it. Assisted yaw never advances the character beyond the schedule and is
zero when physics already tracks or leads it. Retargeting constructs a new schedule from the current
physical heading; it does not restart from the old plan origin or add the remaining angle twice.

The correction pivot is the measured stance contact/plant center while one foot owns support. During
validated double support, pivot ownership blends continuously from the old stance center to the new
one using the same load handoff that selects support ownership. There must be no midpoint switch or
world-space target jump at the role swap.

Applying one assisted increment must rotate together:

- every ragdoll body position and orientation about the owned pivot;
- linear and angular velocity directions;
- physical foot baselines and plant references;
- COM baseline, support target, support curve endpoints, and support velocities;
- admitted swing start/target/desired positions and turn-plan bases;
- standing ground-reference heading and committed/scheduled heading state;
- any cancellation, stop, or inter-step world-space reference that can survive the frame.

The stance center is not translated by the operation. Contact caches are invalidated for affected
bodies after the incremental transform. Pitch, roll, vertical position, relative articulated pose,
and the existing fall state are preserved.

#### Foot strategy and cadence overlay

Foot behavior changes continuously with requested angle; angle bands select contact count, not a
different heading controller:

- below 15 degrees, preserve ordinary forward walking and curve the footprint;
- from 15 through 90 degrees, suppress forward advance progressively and use one shortened turning
  step around the loaded stance foot;
- above 90 degrees, suppress forward advance, retain the exact-180 outside-first sign latch, and use
  an outside/inside pair;
- after heading completion, admit a closing step only when needed for stance width or load symmetry;
  it must not delay the reported heading time;
- retain the current 20-degree physical-footprint ceiling initially. It controls how much relative
  leg/sole steering the physical planner attempts, not the global turn duration.

Turning uses the existing contact-driven phase names but receives a cadence overlay. The nominal
target for each turn beat is 0.45-0.55 seconds. Weight shift, swing, landing validation, and load
transfer should overlap where their existing predicates allow. A timer may not be bypassed merely to
hit the deadline, but an already-proven contact/load condition must not pay another serialized hold.
Diagnostics must identify whether any excess is caused by contact, tracking, sole orientation, load,
drift, tilt, or a timer.

#### Slice 1 - turn-level clock and shadow scheduler

**Behavior change:** none.

**Accepted for progression (2026-08-11); full runtime matrix deferred.**
`LocamotionControllerComponent::AssistedTurnPlan` now stores the command basis, live target,
duration, elapsed/eased schedule, scheduled and measured yaw/rates, residual, support ownership,
event timestamps, crossing mask, and turn/retarget sequence. `UpdatePhysicalGait` samples this record
after command/start state is finalized. The block writes no existing gait, ragdoll, heading, foot, or
support target, and `_assistedTurnPlan` has no control-path reader.

The `[LocomotionAssistedTurn]` stream reports `COMMAND`, `RETARGET`, `RELEASE`,
`SCHEDULE_COMPLETE`, `HEADING_CROSS`, `HEADING_COMPLETE`, `GAIT_READY`, and 0.25-second `SAMPLE`
records. A formula check at 30, 60, and 144 Hz reached identical signed endpoints monotonically for
mirrored 15/45/90/120/180-degree schedules, the Debug build passes, and a routing audit confirms that
the shadow record cannot affect current locomotion. Because the complete manual matrix was
impractical to exercise, the live mirrored matrix, exact-180 sign trace, retarget/release trace, and
unchanged physical-turn comparison remain deferred regression coverage. They do not block the
diagnostic-only Slice 2, but must pass before scheduled assistance is activated in Slice 3.

Add an angle-independent `AssistedTurnPlan` (or equivalent) containing start/desired heading,
scheduled duration, elapsed time, eased progress, scheduled angular velocity, measured physical
progress, support role, and turn/retarget sequence. Keep the current physical planner authoritative
while evaluating the new schedule in shadow mode.

Add one turn-level telemetry stream with command, heading-crossing, and gait-ready timestamps instead
of reconstructing total latency from per-phase messages. It must record requested angle, scheduled
angle/rate, physical angle/rate, residual, current phase, current support foot, contact count, and the
constraint delaying gait-ready completion.

**Pass gate:** mirrored 15/45/90/120/180-degree commands produce monotonic, frame-rate-independent
schedules; exact 180 keeps one deterministic sign; retarget/release produces no angle discontinuity;
runtime behavior and the accepted physical-turn regression remain unchanged.

#### Slice 2 - bounded yaw primitive and reference coherence

**Behavior change:** diagnostic test path only.

**Accepted (2026-08-11).**
`Physics::ApplyRagdollLocomotionPivotYaw` is restricted to active Powered locomotion and clamps one
call to 20 degrees. It snapshots the complete articulated body set, rotates positions and
orientations about the requested stance pivot, rotates linear and angular velocities without
changing their magnitudes, activates the bodies, and invalidates every affected contact cache. Its
result reports pivot, momentum, relative-pose, tilt, and body-count checks.

`RotatePhysicalGaitWorldReferences` is the single gait-side coherence boundary. It rotates physical
turn bases, foot plants and baselines, swing/landing targets, support curves, COM/stop/cancellation
references, world-space leg references, cached physics readback, and the current physical heading
target in the same call. Live input and the global Slice 1 schedule remain fixed so an applied yaw
reduces their measured residual instead of moving the objective. The inspector exposes nonserialized
5/10/20-degree presets, a diagnostic weight that defaults to zero, and an explicit one-shot Apply
button. `[LocomotionAssistedYawDiagnostic]` reports `ZERO_WEIGHT`, `APPLY`, `REJECT`, and `REFRESH`;
`REFRESH` waits for a real fixed-physics-step serial before evaluating contact preservation.

The Debug build and zero-weight routing audit pass, and the live one-shot diagnostic produced the
expected pivot-yaw behavior without an observed pose, contact, or locomotion regression. Slice 2 is
therefore accepted for progression. Keep the one-shot path available while Slice 3 is introduced;
any later mirrored-turn failure attributable to pivot drift, reference mismatch, momentum change,
or contact refresh reopens this acceptance rather than being hidden by assistance tuning.

Restore a narrowly scoped physics operation that rotates an active ragdoll incrementally about an
explicit world-space pivot while preserving relative pose and momentum. Add one gait-side function
that rotates every surviving world-space reference listed above in the same call site. Do not expose
the operation as a general gameplay teleport.

Exercise 5-, 10-, and 20-degree incremental rotations from settled double support and from each
mirrored single-support stance. Verify the chosen pivot remains fixed, reference deltas equal body
deltas, the upright constraint follows the new frame, and one physics refresh produces valid contacts.

**Pass gate:** pivot translation below 5 mm, no reference discontinuity, no new foot separation or
tilt failure, momentum rotates without magnitude injection, and returning the diagnostic weight to
zero exactly reproduces current behavior.

#### Slice 3 - unified scheduled assistance through 90 degrees

**Behavior change:** enable scheduled residual assistance for 0-90-degree runtime turns while keeping
the existing physical cadence and foothold admission.

**Accepted (2026-08-11).**
The Slice 1 scheduler now marks nontrivial requests through 90 degrees as assistance-eligible while
leaving larger requests in shadow mode. The eased scheduled heading becomes the global root-motor
target for an eligible turn. At the end of each controller update, the measured signed lag is scaled
by `gaitAssistedTurnStrength`, capped by `gaitAssistedTurnMaxSpeedDeg` and the Slice 2 20-degree
primitive limit, and applied only when physics is behind the schedule in the requested direction.
The correction cannot pass the current scheduled sample.

Pivot ownership stays on the current support foot during single support. During validated double
support it blends from the old contact to the new contact over the existing 0.48-0.68 measured-load
handoff, then the ordinary role swap makes the new foot authoritative. Every applied increment uses
the accepted Slice 2 reference-coherence boundary; afterward the fixed global schedule target is
reasserted so rotating the immutable physical step cannot double-add its admitted yaw. Assisted
commits use measured pelvis heading rather than the rotated per-step endpoint.

`Scheduled Turn Assistance` defaults on with strength 1 and a 360-degree/second correction cap. The
fields are serialized and exposed in the locomotion inspector, so setting the strength to zero or
disabling assistance restores the previous physical path. `[LocomotionAssistedTurn]` now labels each
turn `mode=assisted` or `mode=shadow` and reports correction, accumulated assisted yaw, pivot blend,
and application count. Mirrored schedule simulation at 30, 60, and 144 Hz reaches identical
15/45/90-degree endpoints without exceeding a 10-degree incremental correction; the Debug build
passes. Live mirrored timing, overshoot, contact, foot-crossing, abort, and straight/zero-yaw checks
remain the acceptance gate.

The first live acceptance run confirmed the heading response—a 89.945-degree command reported
`HEADING_COMPLETE` in 0.451 seconds—but exposed a straight-walking regression after convergence.
The active schedule continued applying direct corrections until command release, and the physical
foothold planner treated residual yaw as small as 0.106 degrees as a fresh turn. That combination
could invoke an inside-foot role swap, reduce heading advance to zero, and produce a zero-length
planned step after an otherwise successful turn.

The stabilization pass now latches direct assistance off at the existing rate-checked
`HEADING_COMPLETE` event, evaluates the completion rate by magnitude, ignores assisted commands
below 1 degree, and quantizes physical foothold yaw below 1 degree to zero. The scheduled heading
remains the ordinary motor target after assistance stops, so physics may settle naturally without
additional whole-ragdoll transforms. Sub-threshold foothold residuals retain full translation and
cannot trigger outside-foot selection, turn-pair bookkeeping, or an inside-foot role swap. Runtime
acceptance still requires a fresh straight-walk and mirrored-turn trace; this stabilization is not
itself an acceptance claim.

A second live trace confirmed normal 0.139-0.154 m straight advances and no assisted transforms
after `HEADING_COMPLETE`, but exposed two post-heading aborts. A 90-degree run admitted a redundant
-1.9-degree physical cleanup turn and later failed sole ownership on its exit step; a 45-degree
retarget admitted a redundant -6.0-degree inside-foot role swap while the global schedule was still
finishing, then failed descent. Both came from the physical foothold planner comparing its older
committed basis with the final input heading even though scheduled assistance already owned that
same remaining yaw.

The localized ownership correction now rebases each newly admitted foothold to the current scheduled
heading while an eligible assisted schedule is active. The current scheduled basis is both the start
and angular objective for that admission; subsequent assisted increments rotate the immutable plan
through the existing reference-coherence boundary. Superseded physical turn-pair and exit-blend
latches are cleared at that boundary, with `FOOTHOLD_REBASE` telemetry recording any nontrivial
basis change. This prevents the physical planner from replaying scheduler residuals without changing
contact, sole, drift, or landing thresholds.

Heading completion and gait-ready admission now use the same measured pelvis angular velocity shown
by assisted-turn telemetry rather than the root motor's internal rate, which excludes direct
whole-ragdoll correction. Heading error and measured rate must remain inside their existing 2-degree
and 30-degree/second limits for 0.04 seconds before `HEADING_COMPLETE`. This bounded stability proof
prevents the first post-turn foothold from starting while the visible ragdoll is still rotating.

The acceptance trace from `15:32:37.220` through `15:33:02.090` passed the Slice 3 runtime gate. The
89.813-degree retarget reached `HEADING_COMPLETE` in 0.501 seconds, and the 45.387-degree retarget in
0.270 seconds. Additional mirrored live retargets of 68.734 and 84.803 degrees completed in 0.390 and
0.472 seconds. Schedule completion correctly reported `heading-settle` or `yaw-rate` until the
measured 0.04-second stability proof passed, and no assisted increment followed any
`HEADING_COMPLETE` event. Across 28 subsequent foothold admissions, scheduler ownership produced
zero requested physical yaw and full 0.135-0.144 m advances; neither the former -1.9-degree cleanup
turn nor the -6-degree role swap recurred. No locomotion abort occurred in the acceptance interval.
The later 180-degree shadow-mode experiment is outside Slice 3 and is intentionally excluded from
this result. Slice 3 is accepted for progression to the one-beat cadence work in Slice 4.

Each physics update measures pelvis heading, samples the schedule, and applies only the bounded lag
correction about the currently owned stance pivot. The physical heading motor remains enabled.
Assistance must approach zero on gentle arcs that the motor tracks and increase smoothly with lag;
there is no on/off threshold at 15, 45, or 90 degrees.

The admitted swing foothold remains immutable in the corrected world frame. After an assisted
increment, rotate that immutable plan as a whole rather than rebuilding it from measured error.

**Pass gate:** mirrored 15/45/90-degree turns reach their heading targets without overshoot, target
rewrite, foot crossing, or an abort. Heading time meets 0.20/0.35/0.55 seconds respectively, even if
the unchanged physical cadence still delays `gait ready`. Straight walking and zero-yaw plans apply
exactly zero assisted rotation.

#### Slice 4 - one-beat turn cadence through 90 degrees

**Behavior change:** enable the 0.45-0.55-second cadence overlay for 15-90-degree turns.

Use one outside-directed turn step. Begin support preparation as the global heading starts, continue
the support curve through swing and landing, and allow existing touchdown/load validation to overlap.
The stance foot remains the assisted pivot until the normal load-handoff predicate transfers
ownership. Forward advance scales smoothly from ordinary gait below 15 degrees to nearly zero at 90.

**Pass gate:** mirrored 15/45/90-degree commands meet both heading and gait-ready targets, use no more
than one primary turn contact, and preserve the existing contact, 4 cm drift, 30-degree tilt, 10 cm
minimum separation, reach, touchdown, load, and motor-saturation limits. Release and retarget at each
major phase must remain bounded.

#### Slice 5 - two-beat 90-180-degree turn

**Behavior change:** extend the same scheduler and assistance to large direction changes.

Use the existing exact-opposite sign latch and outside-foot selection. The first beat places the
outside foot while heading advances toward the midpoint of the global curve. After measured load
ownership transfers to that foot, pivot ownership moves continuously to it and the inside foot closes
toward the final heading. The second foothold is predicted from the global schedule and physical
support state; it is not derived by adding another per-step yaw to the assisted result.

The controller may finish heading before the closing contact settles. It may not start forward
acceleration in the new direction until `gait ready` passes.

**Pass gate:** mirrored 120/135/180-degree commands complete with two primary contacts, reach heading
within 0.70/0.75/0.90 seconds, and become gait-ready within 1.20 seconds. Exact reversals retain one
direction, no turn uses more than two primary contacts, and no run emits plan rejection, abort,
automatic retry, crossover, or a standing restart.

#### Slice 6 - active-phase handoff, release, and retarget

**Behavior change:** allow the unified schedule to begin from every gait phase.

- `WEIGHT_SHIFT` and `INTER_STEP` enter the schedule immediately;
- early swing retains the existing safe return while scheduled yaw decelerates or retargets from the
  achieved physical heading;
- late swing, arrival, and descent finish their immutable foothold while the global schedule proceeds;
- touchdown, settle, transfer, and hold preserve the admitted contact and rebase the pivot only after
  measured load ownership changes;
- release decelerates to the achieved heading, while a live direction change constructs one new
  schedule without a stop/restart cycle.

**Pass gate:** mirrored release and retarget tests in every major phase produce one continuous heading
record, no double-applied yaw, no stale pivot, no target jump, and either continued stable gait or the
validated controlled stop. The 0-180 timing targets apply from command receipt, not only from the next
step boundary.

#### Slice 7 - final tuning, presentation, and regression

Tune scheduled angular rate/acceleration, maximum residual-assist rate, and the turn cadence together.
Do not tune the 20-degree footprint ceiling, foot angular-speed limit, heading torque, or assistance
cap in isolation. A parameter increase requires telemetry showing that its corresponding limit is the
active cause of lateness and that physical reserve remains.

After physical timing passes, an optional presentation-only refinement may lead the pelvis by a small
torso/head target. It must not own root heading, foot placement, or support and is not required for the
first accepted implementation.

**Final acceptance matrix:**

- mirrored 5/15/30/45/60/90/120/135/180-degree commands from both initial support roles;
- sustained gentle arcs and repeated alternating medium turns;
- command, release, and retarget during every major gait phase;
- exact reversal followed immediately by walking in the new direction;
- 30, 60, and 120 Hz render rates with the same physics timestep;
- the accepted straight 44-step/controlled-stop regression;
- the previous physical-only turning matrix with assisted weight forced to zero.

The accepted implementation must meet the timing table at the 95th percentile across repeated runs,
apply zero assistance to zero-yaw gait, preserve all existing fall/contact/reach/drift/tilt/separation/
load/motor safety bounds, and contain no unreported fallback. Once this matrix passes, the physical-
only route remains a diagnostic mode rather than the default gameplay turn.

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
