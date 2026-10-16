# Left Elbow Joint-Space Pregrasp

## Latest Reach-First Request (2026-10-09)

The latest request supersedes the fixed two-stage 80 mm insertion recipe below:
recognize the left-elbow surface once, target the grasp center 50 mm farther
along the demonstrated approach direction, prioritize reaching it, and prefer
a shorter grasp-center path without mandating a 400 mm waypoint. This is
flange +Z approach, approximately world downward, not robot-base negative Z.

Entry point:

```bash
/home/arm/xCoreSDK-v0.5.1/example/Test/plan_left_elbow_reach.sh
```

It compares direct Cartesian interpolation, direct joint interpolation, and
joint-transit/linear-approach candidates with 50, 100, 150, 200, 300 and 400 mm
surface offsets. These are a finite candidate set, NOT a global shortest-path
algorithm. The SDK supplies IK, FK and Jacobians. Every candidate must pass
limit, workspace, continuity, requested insertion-plane and singularity checks.
Among passing candidates it selects minimum sampled grasp-center path length,
then joint travel for a numerical tie. It does not silently relax orientation,
change the detected target, or fall back to historical recognition.

`kLeftElbowReachInsertionM = 0.050` is the latest mode's parameter. The older
`kLeftElbowSurfaceInsertionM` belongs only to the legacy staged recipe. Results
are `.reach_plan.json` plus `.reach_path.csv`, with one or two selected legs,
all candidate outcomes, both surface and grasp-center coordinates, start/end
joint/pose data, and sample-derived path costs. `validate_reach_plan.py` checks
the report offline. First-run evidence: `reports/reach_plan/run_71ug0LO3`.

The operator reports that technical staff resolved collision concerns. This
is recorded as an onsite report, not falsely marked as software collision
verification. No old 50/200 mm support-gap gate or positive-payload gate is used
by this candidate search. Controller soft limits and existing incident holds
are retained. Both CLI layers reject execution before hardware access; this
mode does not dispatch commands or repurpose the fixed two-stage supervisor.
The selected MoveL interpolation is a local planning assumption, not a verified
controller trace. Exact tool/link swept geometry and controller trajectory
acceptance remain outside this kinematic result. Do not directly feed the CSV
to a motion loop or treat a passing plan as permission to bypass the hold.

The first fresh run found seven feasible candidates. Direct linear planning
failed at sample 2 with local SDK IK -6; this is not proof the physical robot
cannot ever follow a straight route. Direct joint motion had a sampled grasp
path of 2.168 m; the selected 400 mm transit plus 450 mm linear approach was
1.748 m. Selecting 400 mm here was a measured comparison result, not a fixed
requirement. These numbers describe that observation only.

### Controller Check of the Saved Candidate

`check_reach_controller.cpp` is a read-only diagnostic for a saved reach
manifest. It validates the active zero tool/reference and base frame, checks
stored joint/FK agreement, and calls the SDK's `checkPath` for MoveL legs with
their planned start joints. It never uses the current arm joints as the
second leg's seed. It does not refresh the target or send motion commands.

```bash
LD_LIBRARY_PATH=/home/arm/.local/xcore-vision/lib \
  build/bin/check_reach_controller PATH_TO_PLAN.reach_plan.json
```

Actual result for `reports/reach_plan/run_71ug0LO3/controller_check.log`:
the 450 mm second leg returned code 0 (`success`), and the maximum endpoint
joint difference from the sampled plan was 5.39456e-08 rad. The arm remained
idle and unchanged during the diagnostic. This confirms the controller can
check that Cartesian leg from the virtual handover state. It is not a trace
of actual execution or proof that every intermediate controller joint sample
equals the local interpolation. MoveAbsJ is not covered by this Cartesian
check, and neither leg has a whole-arm collision certificate. The original
manifest remains unchanged as a record of the earlier planning operation;
this later check is recorded in the separate log. Execution remains held.

## Current Surface-to-Grasp Target (2026-10-09)

The user clarified that depth identifies the visible surface, while the desired
grasp center is approximately 80 mm farther along the approach direction. New
staged left-elbow plans use `kLeftElbowSurfaceInsertionM = 0.080` in
`../function_config/visual_servo_config.hpp`. This is planning-only and does NOT
release the incident execution hold or certify insertion/contact.

Let P be the observed surface and d the final flange +Z approach axis:

- First-stage grasp center: P - 0.400*d (unchanged).
- Final grasp center: P + 0.080*d.
- Final flange: P + 0.080*d - R*[0,0,0.300].
- Second-stage line length: 0.480 m, not 0.400 m.
- Reported finger-tip axial extension from the surface: 0.080 + 0.100 = 0.180 m.

"Down" here means the existing demonstrated approach direction, NOT robot-base
Z (the installed base/world axes differ). New schema v3 manifests distinguish
`target_point_base_m` (observed surface) from `grasp_target_base_m`, record the
insertion direction/depth, and use surface-relative `end_gap_m = -0.080`.
The validator, supervisor, timeout and support-plane diagnostic all include
this insertion. Historical schema v1/v2 files retain their zero-insertion
semantics; do not reinterpret old reports or compare their endpoints as equal.
The older 40 cm second-stage descriptions below refer to that earlier setup.

The user subsequently reported 200 mm of approach-direction clearance behind
the elbow, superseding the earlier 50 mm report for the current scene. With
80 mm insertion and 100 mm finger extension, nominal residual clearance is
20 mm before perception, calibration, geometry and motion errors. This is an
onsite-reported clearance, not a machine-verified swept-volume certificate.

## Execution Hold After Unintended Contact (2026-10-07)

The zero-standoff joint alignment contacted another body area, and the onsite
operator emergency-stopped it. Both Python and C++ now reject grasp-alignment
execution BEFORE any robot connection; read-only planning remains available.
Do not reset/re-enable or automatically return the robot while contact remains.

The failed trajectory replaced the entire earlier pregrasp path, rather than
appending a local approach. J1 travel increased from about 107 to 175 degrees,
and J4 from about 70 to 137 degrees. FK, soft-limit and target-plane checks did
not model finger/link swept volumes or collision with the mannequin/other arm.
Sampled kinematic feasibility must not be described as collision clearance.
Execution must stay blocked pending collision-aware route redesign and review.

Original incident log was `/tmp/xmc_left_elbow_300mm_align_execution_20261007.log`.
It was read on 2026-10-07 but was no longer present on 2026-10-08. See
`INCIDENT_20261007.md` for the preserved findings, not a full trajectory log.

The demonstrated-orientation mode now defaults to joint motion (`MoveAbsJ`),
not a Cartesian straight line. It always captures a fresh left-camera burst
paired with the current left-arm pose. No old recognition result is reused.

Build once after editing C++:

```bash
cd /home/arm/xCoreSDK-v0.5.1
cmake --build build --target mannequin_approach_plan mannequin_snapshot -j2
```

Read-only capture and planning:

```bash
env LD_LIBRARY_PATH=/home/arm/.local/xcore-vision/lib \
    PYTHONPATH=/home/arm/.local/share/xmc-pose-python \
    /home/arm/论文/大语言模型/qwen_env/bin/python \
    /home/arm/xCoreSDK-v0.5.1/example/Test/left_shoulder_micro_approach.py \
    --demonstrated-pregrasp --joint left_elbow --motion joint
```

Only after confirming the entire new swept volume is clear and an onsite
operator can emergency-stop, add `--execute`. Execution recaptures/replans;
it does not execute a previously saved recognition result.
Use `--motion linear` to explicitly select the earlier straight-line mode.

## Planning and Execution

- Solve the endpoint with controller IK, choose nearby legal equivalent turns,
  and verify the target by FK. No Cartesian intermediate-point IK is required.
- Sample joint interpolation at at most 0.005 rad per axis. Check controller
  soft-limit margins, flange workspace, provisional grasp-center standoff,
  FK and Jacobian thresholds. Existing protection thresholds are unchanged.
- Left arm only, 1% joint speed, zero blending. Monitor actual joint-path
  agreement, workspace, speed, target standoff, controller events, and the
  stationary right arm. Stop/cleanup is attempted on error or interruption.
- The synchronized joint interpolation is a planning assumption; deviations
  during execution cause a stop. Planning is not a certified controller trace.
- The non-contact mode uses the saved manually demonstrated orientation,
  user-measured flange +Z tool length 0.300 m (updated 2026-10-07), and 0.40 m
  non-contact clearance. No gripper IO
  or contact action is performed, and no automatic short final approach is added.
- These checks are NOT whole-arm self/environment collision detection. A valid
  endpoint and sampled path do not establish collision clearance or success rate.
  Do not run another robot-control program concurrently.

If joint planning fails, the program holds; it does not relax safeguards or
automatically improvise intermediate poses. A different route must be planned
and its swept space checked before execution.

## Two-Stage Planning Trial (Read-Only)

Complete one-observation planning entry point, from any working directory:

```bash
/home/arm/xCoreSDK-v0.5.1/example/Test/plan_left_elbow_full.sh
```

This builds the two required targets, takes ONE initial observation round
(10 frames for confidence/depth consistency while stationary), plans BOTH
stages before returning, and checks the resulting manifest/CSV offline. There
is no recapture or person-keypoint detection between the two virtual stages.
The launcher accepts no execution flag and sends no motion command.

Each run has its own `example/Test/reports/full_plan/run_*` directory with the
paired images/pose, detected point, pregrasp geometry, planning log, sampled CSV,
and `.full_plan.json` manifest. A complete manifest is committed only after both
stages pass. Any failure makes the launch fail, with no first-stage motion.

The manifest binds one base-frame target to both stages, records the start and
handover joints/poses, calibrated camera transform, measured 300 mm tool offset,
right-arm reference, final grasp pose, and controller-vs-model IK outcome.
`robot_motion_permitted: 0` and `collision_validation: not_performed` are
intentional: a complete kinematic plan is not a collision certificate or a
hardware-commissioned executor. Target/base/mount/scene changes, unexpected
robot movement, contact or E-stop invalidate a later attempt to use the plan.
Do not feed the original observation to a fresh detector after stage one or
substitute another run's target. The staged supervisor consumes the same in-memory
plan and monitors its handover; the SDK dispatch branch remains behind the early
incident hold, not available as a runnable motion mode.

Offline consistency check (no hardware access):

```bash
/home/arm/论文/大语言模型/qwen_env/bin/python \
  /home/arm/xCoreSDK-v0.5.1/example/Test/validate_full_plan.py /absolute/path/to/plan.full_plan.json
```

Use `--staged-plan --joint left_elbow` instead of `--demonstrated-pregrasp`.
This is a new planning-only mode, not an executable workaround for the incident
hold. Both Python and C++ reject `--execute` before hardware access.

1. Plan joint transit to a grasp center 0.40 m in front of the freshly detected
   elbow, with the demonstrated final tool orientation and 0.300 m tool offset.
2. Starting at that exact joint endpoint, plan a 0.40 m straight approach with
   constant orientation along flange +Z until the grasp center reaches the point.

This removes the last 0.40 m from the large joint-space arc; it does NOT prove
the first-stage arc or the final insertion is collision-free. Stage one is
freshly computed, not an exact replay of the older 0.35 m tool experiment.
IK/FK, workspace, joint limits, Jacobian conditioning and continuity checks
apply to both stages. A failure in either stage makes the entire trial fail;
no first-stage motion is sent while planning the second stage.

Stage two now uses the SDK's local `getJointPos` for EVERY new sample, with the
previous virtual joint state as seed (principal angles for the solver, then
unwrapped near the previous physical joint values). Its initial state is exactly
the stage-one joint endpoint. A boundary IK/FK round-trip first verifies that
the solver preserves that endpoint. All subsequent results must pass FK, soft
limit, continuity and Jacobian checks. A local failure fails the whole plan;
there is no fallback to `calcIk` with the live observation posture as reference.

Manifest schema v2 records `linear_ik_solver: seeded_local_sdk`, the seed policy,
boundary check, and the local solve count. Stage-two controller IK is NOT queried
or marked as passed. The offline validator accepts historical v1 reports but
checks the different v2 semantics explicitly. This fixes the virtual seed chain,
not the controller's internal implementation, and does not certify a future
MoveL command or collision clearance. No `setDefaultConfOpt` call is made: its
`confData` selection option changes controller configuration and is separate
from a numerical previous-joint seed. Actual execution remains disabled.

For a repeatable solver diagnostic on a historical paired observation, the C++
entry point accepts `SAVED_CAPTURE.yml --diagnose-staged-ik SAVED_GEOMETRY.yml`.
Use the same absolute capture path embedded in the geometry metadata. This mode
does not access a camera, but reads the robot/model and requires its current
joints to still match the saved observation. It can accept an old timestamp ONLY
because execution is rejected before any hardware access. It is not a scene
refresh or permission to reuse a historical target for motion.

Diagnostic output uses a separate `.seeded_diagnostic` suffix and schema v2
`status: diagnostic_kinematic_complete`, `diagnostic_replay: 1`, and
`observation_rounds: 0`; it must not be confused with a new scene plan. Original
plan files are not overwritten. The offline validator checks these labels.

The geometry filename plus `.staged.csv` holds sampled joints, flange position,
grasp-center position, tool axis and signed gap for both phases. A file without
the final `# COMPLETE` marker is partial/failed. Even a complete file is only a
kinematic plan, not collision clearance or permission to execute. Use a durable
`--output-prefix` directory to preserve images and the report after reboot.
If the robot is still stopped in contact or reports a non-idle state, the trial
holds; it never clears E-stop, enables power, or automatically returns the arm.

## Staged Supervisor Commissioning (2026-10-08)

`../task_library/staged_motion.hpp` implements the two-stage supervision sequence:
start-pose check, first-stage command, matching arrival event plus at least 0.3 s
of idle endpoint confirmation, another boundary check, second-stage command,
and final idle/power-off verification. Any failure after preparation triggers
cleanup and prevents subsequent stages. It monitors joint route/branch, limits,
Cartesian line/orientation, speed, power/state, reference frames, other-arm
stationarity, interruption, telemetry gaps and timeouts. No gripper IO or
automatic recovery/return is included.

`staged_motion_replay.hpp` supplies a virtual clock and synthetic telemetry.
Every completed staged planning run now replays that supervisor against the
local SDK FK model, writing `.supervisor_replay.txt` on success. This is NOT a
controller/dynamics simulation, timing certification, collision check or proof
that the controller accepts MoveL. The simulator does not access the network;
the enclosing planner still connects to read robot state/model.

`staged_motion_sdk.hpp` is the compiled SDK adapter. Its dispatch branch is
unreachable while the existing early execution hold remains in force. It has
NOT been exercised on hardware. It validates finite nonnegative payload readback
(zero is permitted), checks readback changes and cancellation, issues one command at a time,
and attempts stop, power-off, queue reset, idle and manual mode on cleanup.
SDK success/error and cleanup behavior still require commissioning; none of
these host-side checks replaces the robot's independent safety system.

Offline supervisor tests (no controller/camera access):

```bash
cmake -S . -B build
cmake --build build --target test_staged_motion -j2
build/bin/test_staged_motion
```

Measured tool facts are in `../function_config/left_gripper_geometry.json`:
flange-to-center 300 mm, open outer width 150 mm, maximum thickness 20 mm,
and finger tips 100 mm beyond the grasp center. These are user-reported
dimensions, NOT an exact finger/camera/adapter collision mesh. In particular,
moving the grasp center to the detected surface puts the finger tips 100 mm
beyond that point. The inner opening and contact geometry are not established
by the outer width alone.

The user initially reported configured payload/center of mass and enabled collision
detection. An SDK readback on 2026-10-08 returned mass 0 kg, zero
center of mass/inertia and zero TCP. Per `robot.h`, the SDK toolset is distinct
from an RL project's tools; do not overwrite parameters with guessed values or
conclude that the pendant's tool database is empty. `tool_load_readback --left`
queries the SDK active group and currently loaded project's tool list without
explicit mode, power or motion commands. It does NOT query all unloaded projects
or prove that collision detection is enabled. Note that SDK robot construction
itself is documented to reset its motion queue; use only with the robot idle.

On 2026-10-09 the user clarified, after consulting technical support, that this
controller version does not support configuring effective payload parameters.
Treat this as the current user-reported capability, not as independent SDK
verification. The previous positive-mass admission requirement was our added
assumption, not a demonstrated requirement for this controller, and has been
removed. Do not request repeated payload setup, write guessed payload values,
or interpret zero readback as proof of unsafe/missing configuration. Numeric
validity and unexpected readback-change checks remain. This change neither
certifies collision detection nor removes the separate incident execution hold.

The robot was subsequently disconnected by the user. No reconnection or motion
is automatic. Reconnection requires a new stationary observation and plan; old
images and completed replay reports must never act as a resume command.

## Local Support-Plane Diagnostic

`inspect_gripper_support_plane.py PLAN --roi X0 Y0 X1 Y1` reads an existing
paired depth image and plan offline, fits a manually selected visible surface
patch, and estimates finger-tip height relative to that extrapolated plane.
It uses the reported 100 mm extension beyond the grasp center, and bounds
transverse placement by a disk enclosing the reported width/thickness. It does
not know the exact finger solids/cavity, hidden surfaces or entire robot.

The diagnostic never connects to hardware and always reports
`robot_motion_permitted: false` and `collision_clearance_established: false`.
A good local plane fit is NOT a bound on extrapolation error. Compare nearby
patches instead of selecting a favorable result; inconsistent estimates require
onsite geometry clarification, not execution. The 2026-10-09 example review is
`reports/full_plan/run_V0hsRb1d/SUPPORT_PLANE_REVIEW.md`.

## Grasp-Center Target (Read-Only)

Use `--grasp-plan --joint left_elbow` instead of `--demonstrated-pregrasp` to
plan the actual grasp-center target with zero added standoff. This mode still
captures a fresh observation; it does not reuse the last approach detection.
Given the detected base-frame point `P`, demonstrated orientation `R`, and
tool offset `t = [0, 0, 0.300]` m, the target flange is `P - R*t`.
The target grasp center is `P`, not a point 0.40 m in front of it.

Both the Python entry point and C++ planner reject `--execute` in this mode.
This is a feasibility plan, not a contact/gripping command: tool calibration,
keypoint/depth error, finger geometry and collision clearance still need to be
validated. A visual surface point is not automatically an anatomical center.
Close-up depths down to 0.35 m are accepted for this read-only plan; executable
approach depth/clearance guards are unchanged. No gripper IO is added.

## Explicit Grasp-Center Alignment

`--grasp-align --tool-length-mm 300 --joint left_elbow` uses the measured
300 mm coaxial tool offset and zero standoff. It defaults to read-only.
This separate mode previously allowed `--execute`, but execution is now blocked
after the unintended contact described above. Onsite clearance confirmation
alone is not a substitute for validating the actual swept volume.
The measurement is not a six-DOF TCP calibration or a collision certificate.

The Python and C++ entry points require the explicitly supplied measured tool
length to match the configured 0.300 m. Old geometry files without that field
cannot authorize alignment. Both grasp modes now reject execution.
Each Python invocation takes a fresh paired capture; no old target is reused.

The disabled execution implementation used MoveAbsJ at 0.5% joint speed
(linear override: 5 mm/s).
Joint motion has a 12-minute timeout, a 0.05 rad/s measured joint-speed guard,
and a 2 mm estimated signed-standoff monitoring tolerance. Existing pose,
limit, stationary-other-arm, FK/Jacobian, and trajectory guards remain active.
Completion means commanded alignment, NOT independently verified physical
grasp accuracy or successful gripping. There is no gripper IO/closure command.

Argument regression tests (no robot connection or image capture):

```bash
env PYTHONPATH=/home/arm/.local/share/xmc-pose-python \
    /home/arm/论文/大语言模型/qwen_env/bin/python \
    /home/arm/xCoreSDK-v0.5.1/example/Test/test_mannequin_approach_args.py
```
