# AI Prompt - Task Code Library

This folder is the reference library for future large-language-model task synthesis.

The user's long-term task:
- Use a depth camera to identify grasp points and target placement points.
- Transform camera observations into robot/base coordinates.
- Transmit grasp/target data to the robot control program.
- Let an AI model compose the final integrated robot task code by referencing this folder and the SDK examples.

Use this folder for:
- Architecture notes.
- Task prompts.
- Data contracts.
- Coordinate transform conventions.
- Pseudocode.
- Reusable reference snippets that may not be standalone executables.
- Notes about perception-to-robot workflows.

Rules:
- Files here are for AI reference and task synthesis, not automatic compilation.
- Put runnable task programs in `../Test` and reusable calibration/configuration
  utilities in `../function_config`; both directories are auto-scanned by CMake.
- When generating final executable code, pull validated patterns from:
  - `../original_cases` for official SDK usage.
  - `../function_config` for tested robot/camera utilities.
  - `../Test/board_pick_demo.cpp` for a signal-triggered board approach,
    gripper-close placeholder, 20 cm base-Z lift, and return cycle.
- Always respect the calibrated transform output key `H_Flange2Cam` unless the user changes the convention.
- Prefer explicit safety states: connect, check mode, power, drag/realtime/non-realtime mode, execute, stop, cleanup.

Default synthesis request:
Generate a C++17 xCoreSDK v0.5.1 program for XMC18 that uses the configured
RealSense depth/RGB camera to detect a grasp point and a target point, converts
them through the hand-eye calibration result into the robot base frame,
validates reachability, and commands the robot with conservative speed and
safe cleanup.

For checkerboard visual-servo tasks, reuse the coordinate chain, filtering,
state transitions, IK and Jacobian gates in `VISUAL_SERVO_DESIGN.md` and
`../Test/visual_servo_follow.cpp`. Keep perception outside the 1 ms robot loop.

For one-cycle pick demonstrations, read `BOARD_PICK_DEMO.md` and
`../Test/board_pick_demo.cpp`. Its gripper-close function currently writes no
IO; do not claim that a physical grasp has occurred until a real IO command
and closure feedback have been integrated. Preserve the camera-to-grasp-point
offset as an explicit measured configuration parameter.

For the mannequin joint-transit plus straight-approach workflow, read
`../Test/mannequin_joint_approach.md` and `../Test/INCIDENT_20261007.md` first.
`staged_motion.hpp` is a tested supervisory state machine, and
`staged_motion_replay.hpp` is a synthetic backend, not a physics/collision
simulator. `staged_motion_sdk.hpp` is compiled but NOT hardware commissioned.
Preserve the existing early execution hold in `mannequin_approach_plan.cpp`.
Do not bypass it by directly instantiating the SDK backend. A successful
kinematic plan or replay is not authorization/certification to dispatch motion.
The two stages share one initial observation; do not re-detect at the close-up
handover, resume historical plans, reset E-stop, or invent gripper IO.
