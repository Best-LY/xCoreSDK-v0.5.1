# XMC18 Checkerboard Visual Servo

Implementation:

- Runnable program: `../Test/visual_servo_follow.cpp`
- Tunable parameters: `../function_config/visual_servo_config.hpp`
- Hand-eye producer: `../function_config/drag_hand_eye_calibration.cpp`

## Coordinate Chain

The calibration key `H_Flange2Cam` is `flange_T_camera`. The legacy 0.4.1b
files used as input to axis identification are:

```text
LeftArm_interactive_calib.yml  -> camera 236422072262
RightArm_interactive_calib.yml -> camera 236422070786
```

Any calibration result is valid only while its camera mount is unchanged. For
every valid camera frame, the program reads `base_T_flange` and computes
`base_T_camera` as:

Right-arm realtime execution now uses
`RightArm_interactive_calib_axis_verified.yml`, produced by the controlled
camera-axis identification test. Both arms automatically prefer an
`_axis_verified.yml` file when one exists. The left arm currently has only the
legacy file, so `--left --execute` stops before connecting hardware until its
axis identification has been completed. Observe-only and `--identify-axes`
can still read the legacy file; the latter defaults to the legacy input to
avoid repeatedly correcting an already corrected matrix. Realtime execution
also checks that the verified file names the selected camera serial and
checkerboard dimensions.

```text
base_T_camera = base_T_flange * flange_T_camera
```

The checkerboard center is estimated from its sub-pixel image center and aligned
RealSense depth, then deprojected directly into the camera frame. PnP is retained
as a checker-size diagnostic, but does not scale translation tracking. The camera
translation is corrected until that center is `[0, 0, 0.60]` metres in the camera frame. The
flange orientation is held fixed during this first version; this avoids wrist
flips caused by planar PnP ambiguity while following board translations.

For a fixed flange orientation, the relative flange correction depends on the
hand-eye rotation but the hand-eye translation cancels algebraically. This is
why the existing right-arm relative following can work despite an uncertain
legacy hand-eye translation. Absolute base-frame grasp/placement coordinates
still require a verified flange-to-camera translation.

On 2026-09-29 the user confirmed a 28.6 mm checker square. The shared config
and legacy drag calibration source now use 0.0286 m. Earlier results computed
with 20 mm for this board are invalid; both arms must be recalibrated.
The pre-collision axis-verified result is archived and must not be reused.
Enter 28.6 mm in the updated drag calibration program; it also
checks central-square size against aligned depth at each capture. The correct
right or left camera serial is required; the program no longer falls back to
the other connected camera. Run controlled axis identification again on the
new hand-eye result before enabling realtime following with it.

## Filtering And Hold State

The measurement path uses aligned RealSense depth and RGB, sub-pixel checkerboard
corners, depth deprojection, a five-sample median, and position EMA.
Large frame-to-frame jumps and implausible board speed are rejected.
If a rejected new position remains clustered for five consecutive frames, the
filter treats it as a real repositioning and relocks there. This prevents a
permanent hold after the board is moved quickly and then becomes stable again.

Following starts or resumes only after five consecutive valid and safe frames.
It stops when the board is lost, the measurement is rejected, IK fails, a soft
limit is predicted, or the current/target Jacobian is too close to singular.
When the target becomes valid again, the program starts a new FollowPosition
session from the current pose.

## Tool Clearance And Working Distance

The fixed camera/gripper mounting was depth-probed at the verified 0.60 m
working pose. Visible gripper surfaces extend approximately 0.248 m and 0.279 m
from the camera along optical Z. A 0.25 m camera-to-board target would therefore
place the gripper at or beyond the board plane. The project default is 0.60 m,
and command-line distances below 0.35 m are rejected to preserve at least about
70 mm of conservative tool-to-board clearance. A live board measurement below
0.35 m also triggers HOLD, even if the configured target distance is valid.
Re-measure and update these
limits if the camera, adapter plate, or gripper mounting changes.

The legacy home posture contains principal joint angles at approximately
`+/-180 deg`. The 0.5.1 model inverse kinematics uses an internal range of about
`+/-175 deg`, even though the controller allows multiple turns on J1/J4/J5/J6.
Consequently, visual following intentionally remains blocked at that home
posture. `--safety-scan` computes a read-only candidate that brings only those
principal angles back to `+/-160 deg` and scans all six camera directions in
8 mm increments before any real movement is considered.

## Safety Gates

Before each target update:

1. Hold when the target error exceeds 0.18 m; otherwise limit each command
   update to 8 mm.
2. Check the configured workspace envelope.
3. Sample the short Cartesian move every 4 mm and run
   `xMateModel::getJointPos()` along that path.
4. Reject unreachable, soft-limit, large-joint-step, and singular IK results
   at any path sample.
5. Check minimum Jacobian singular value and condition number at current and
   all predicted joints; report the worst value in the CSV log.

`FollowPosition` performs its internal real-time interpolation. The vision loop
updates targets at 15 Hz with a 0.20 speed scale and never performs camera work
in a 1 ms callback.

Commissioning runs can write one CSV row per camera frame with `--log file.csv`.
The fields include filtered board XYZ, detection/measurement/safety/following
states, Jacobian metrics, and the hold reason. A verified 50-second static run at
the 0.60 m working pose produced 754 valid frames, mean Z 0.60147 m, Z standard
deviation 0.00032 m, and Z range 0.60061--0.60213 m.

## Validation Sequence

```bash
cmake -S /home/arm/xCoreSDK-v0.5.1 \
      -B /home/arm/xCoreSDK-v0.5.1/build \
      -DCMAKE_BUILD_TYPE=Release \
      -DXCORE_USE_XMATE_MODEL=ON
cmake --build /home/arm/xCoreSDK-v0.5.1/build \
      --target visual_servo_follow --parallel 4

/home/arm/xCoreSDK-v0.5.1/build/bin/visual_servo_follow --self-test

# List and verify the configured cameras.
/home/arm/xCoreSDK-v0.5.1/build/bin/visual_servo_follow --list-devices

# Read-only model scan from the current pose and the proposed servo-ready pose.
/home/arm/xCoreSDK-v0.5.1/build/bin/visual_servo_follow \
  --right --safety-scan
/home/arm/xCoreSDK-v0.5.1/build/bin/visual_servo_follow \
  --left --safety-scan

# Compute the servo-ready target without motion. After physically clearing the
# workspace, add --execute to perform the low-speed NRT joint move.
/home/arm/xCoreSDK-v0.5.1/build/bin/visual_servo_follow \
  --right --move-ready
/home/arm/xCoreSDK-v0.5.1/build/bin/visual_servo_follow \
  --right --move-ready --execute

# No motion: verify signs, distance, IK, and Jacobian logs first. An available
# axis-verified file is selected automatically; otherwise the legacy file is
# used only for observation and axis identification.
/home/arm/xCoreSDK-v0.5.1/build/bin/visual_servo_follow \
  --right

/home/arm/xCoreSDK-v0.5.1/build/bin/visual_servo_follow \
  --left

# After clearing the left-arm workspace and placing a stationary board in
# view, create LeftArm_interactive_calib_axis_verified.yml. This commands
# small controlled base-axis moves and requires an operator present.
/home/arm/xCoreSDK-v0.5.1/build/bin/visual_servo_follow \
  --left --identify-axes --execute

# Motion is enabled only with this explicit flag.
/home/arm/xCoreSDK-v0.5.1/build/bin/visual_servo_follow \
  --right --execute

# Record dynamic left/right/up/down/near/far commissioning data.
/home/arm/xCoreSDK-v0.5.1/build/bin/visual_servo_follow \
  --right --execute --log /tmp/right_dynamic.csv
```

Do not use an identity or copied calibration for motion. The final Jacobian
thresholds and workspace envelope must be confirmed from observe-only logs at
the actual XMC18 working poses before the first `--execute` run.

The current physical board was depth-estimated at about 28.6 mm per square while
the legacy calibration source was configured for 20 mm. A 70 cm commissioning
test on the right arm confirmed correct depth approach and safe hold/resume, but
the camera-frame X error increased while Y decreased. Treat the legacy right-arm
rotation as unverified: do not continue toward 25 cm until the physical square
size is measured and the right hand-eye rotation is recalibrated or independently
identified with controlled axis motions.

The controlled identification mode does not depend on square size. It translates
the flange along base X/Y/Z by `+/-12 mm`, collects 15 depth-deprojected board
centres at each endpoint, verifies a return-to-origin drift below 5 mm, and
projects the measured camera/base mapping onto SO(3). It writes a new
`*_axis_verified.yml` file and never overwrites the legacy calibration:

```bash
# Read-only endpoint safety check.
/home/arm/xCoreSDK-v0.5.1/build/bin/visual_servo_follow \
  --right --identify-axes

# Keep the checkerboard fixed and clear the robot workspace before execution.
/home/arm/xCoreSDK-v0.5.1/build/bin/visual_servo_follow \
  --right --identify-axes --execute
```
