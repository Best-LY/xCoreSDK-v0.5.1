# Board Pick Demo

Executable: `build/bin/board_pick_demo`

Source: `example/Test/board_pick_demo.cpp`

Configuration: `example/function_config/board_pick_demo_config.hpp`

## Cycle

1. Select the closest soft-limit-valid equivalent of the configured right-arm
   initial joints, validate the path, and move there.
2. Wait for Enter or `S` in the camera window. `Q` exits.
3. Require 10 consecutive checkerboard observations with valid aligned depth
   and no more than 12 mm spread around their median position.
4. Center the camera above the board at a 0.60 m coarse viewing distance.
5. Detect the board again before it can leave the close-range field of view.
   Reject the cycle if the two base-frame board estimates differ by more than
   40 mm.
6. Precheck the approach, lift, and return paths; then move to an estimated
   0.20 m grasp-point-to-board stand-off.
7. Call the gripper-close placeholder.
8. Lift 0.20 m along base-frame positive Z.
9. Hold for 3 seconds.
10. Validate the return path again from measured joints, return to the initial
    joints, and wait for the next trigger.

Every Cartesian segment is sampled at approximately 5 mm intervals. The demo
rejects model IK failures, controller soft-limit proximity, excessive joint
changes, configured workspace violations, and singularity risk before sending
the segment. Each SDK motion waits for its own path ID to report `ReachTarget`
and checks the measured joints or flange pose before the next state begins.
The lift path is checked again from the measured approach pose before the
gripper-close placeholder is called.

## Distance Assumption

The final camera-to-board distance is currently calculated as:

```text
0.28 m estimated camera-to-grasp-point offset
+ 0.20 m requested grasp-point stand-off
= 0.48 m final camera-to-board distance
```

The 0.28 m value is an installation estimate, not a calibrated TCP. It assumes
the grasp point lies on the camera optical axis; lateral grasp-point offset is
not yet calibrated. Replace
`kCameraBehindGraspPointM` after measuring the final flange, camera, and gripper
geometry. The coarse 0.60 m observation prevents loss of the complete board
before the final open-loop approach is calculated.

## Gripper IO

`requestGripperClose()` is the only gripper integration point. It currently
prints a request and writes no IO. Replace its body after the controller output,
active level, timeout, and feedback input are known.

## Running

Camera preview and dry-run only:

```bash
./build/bin/board_pick_demo --self-test
./build/bin/board_pick_demo
```

Real motion after clearing the complete workspace:

```bash
./build/bin/board_pick_demo --execute
```

Press `Ctrl+C` to request a stop. The cleanup path calls the controller's
`stop()` command, then requests idle mode, power-off, and manual mode. These
transitions still need to be verified on the physical controller. A failed
model path check terminates the cycle without bypassing that check.

The default calibration is
`function_config/RightArm_interactive_calib_axis_verified.yml`. Its camera
serial and checkerboard dimensions must match the selected right camera. The
older right calibration file lacks this verification metadata and is rejected.
The open-loop final approach assumes the board remains still after the second
observation. Controller/model path checks do not detect external obstacles or
the gripper's exact collision geometry.
