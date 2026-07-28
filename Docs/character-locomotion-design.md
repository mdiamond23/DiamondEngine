# Character Locomotion Design

Last updated: 2026-07-27

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

**Pass:**

- the foot clears the floor, travels forward, and descends without a discontinuity;
- touchdown is a real upward-facing contact with low vertical foot speed;
- neither plant drifts more than 4 cm after settling;
- tilt stays below 30 degrees and finishes within 10 degrees of its pre-step value;
- no lock force or motor remains saturated after the settling interval.

## Test 5 - Support transfer

**Purpose:** prove that weight can move onto the newly planted foot without initiating the
next swing or fighting two plant controllers.

**Enabled:** validated first step plus a time-bounded COM transfer to the new support side.

**Disabled:** opposite-leg lift and automatic alternation.

**Procedure:** after Test 4 touchdown, transfer support to the new foot and hold for 0.5
seconds.

**Pass:**

- COM projection moves into the new support region;
- the old leg unloads without losing control of the body;
- both contacts remain stable during the transfer;
- plant drift stays below 4 cm and tilt below 30 degrees;
- the final state is stable enough to perform Test 3 with the opposite leg.

## Test 6 - Second step

**Purpose:** validate phase handoff and expose accumulated error before continuous walking
can hide it.

**Enabled:** one automatic leg swap after a successful Test 5 transfer.

**Disabled:** a third step.

**Procedure:** complete one step, transfer support, complete the mirrored second step, then
settle in double support.

**Pass:**

- exactly two alternating steps complete without state resets or fall-gate chatter;
- the second swing begins from a settled support state rather than a timeout;
- no visible jitter, IK branch change, or prolonged saturation occurs;
- tilt after the second step is within 10 degrees of the initial standing tilt;
- foot drift and motor tracking error do not grow from the first step to the second.

## Test 7 - Continuous gait

**Purpose:** turn a validated two-step sequence into a repeatable limit cycle.

**Enabled:** repeated alternation and desired-speed foot-placement feedback.

**Procedure:** first complete 10 steps at constant low speed, then walk for 60 seconds.
Turning, slopes, uneven terrain, and recovery are separate later milestones.

**Pass:**

- 10 alternating steps complete with bounded tilt, plant drift, and tracking error;
- step period and length converge instead of growing or shrinking each cycle;
- the 60-second run does not fall;
- stopping returns to the validated Test 1 standing state.

## Current status

Tests 1 and 2 have passed. Test 3 is the only active target. The first Test 3 run exposed a
specific closed-chain failure: the high-friction swing sole retained contact while its hip and
knee motor commands tracked, so the leg folded against the planted foot, dragged the pelvis,
and made the other foot slide. F6/F7 role selection and COM direction were correct in the log;
the missing invariant was swing-contact release.

Test 3 now moves the COM 92% of the way toward the stance-foot center, applies a capped internal
toe-off force between the swing foot and pelvis until both clearance and a sustained contact exit
are measured, and then disables that force. During takeoff/lift/hold/lower/contact, the normal standing pose writer
skips the swing leg; the stance leg retains its validated local standing pose. Walking, forward
foot planning, plant acquisition, support transfer, and foot locks remain disabled. This does
not pin the foot or add net force to the character: the toe-off force has an equal-and-opposite
pelvis reaction and exists only to open the sticky floor contact before IK owns the airborne leg.

The next Test 3 inspection exposed a ragdoll construction defect below the controller. Foot
support boxes were hard-coded to 18 x 5 x 28 cm, centered on terminal foot-bone pivots that lie
on the ground plane, and drawn with a different transform than the real Jolt shape. The boxes
therefore straddled the floor and their long rotating edges retained contact during toe-off.
Foot bodies now take their dimensions and bind-world vertical center offset from the ragdoll
asset. CesiumMan uses 13 x 5 x 22 cm boxes offset upward 2.7 cm, and debug drawing stores the
exact Jolt wrapper transform. Test 3 also recaptures both foot positions before every F6/F7 run,
so a prior landing offset cannot make the mirrored run abort before takeoff.

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

Do not tune continuous walking while an earlier test is failing. Do not add terrain,
turning, stylistic secondary motion, or recovery logic until Test 7 is stable.

## Relevant files

- `Sandbox/src/Scripts/LocamotionController.h`
- `Engine/src/Scene/Physics/PhysicsSystem.cpp`
- `Engine/include/Scene/Physics/RagdollComponent.h`
- `Engine/include/Scene/Physics/PhysicsAPI.h`
- `Assets/Models/CesiumMan.ragdoll`
- `Assets/Scenes/CharacterTest.scene`
- `logs/loco_debug.log`
