# Perception To Robot Contract

This document defines the data contract future AI-generated task code should follow.

Inputs:
- RGB image from RealSense D455.
- Depth image from RealSense D455.
- Camera intrinsics from librealsense.
- Hand-eye calibration file produced by `function_config/drag_hand_eye_calibration.cpp`.

Calibration file:
- Expected key: `H_Flange2Cam`
- Fixed grasp-direction convention: camera/image left-right is the gripper's
  lateral motion and grasp-alignment direction. The camera-to-gripper mounting
  orientation is fixed; generated grasp code must preserve this axis convention
  and must not infer or swap it dynamically.
- Meaning: camera pose in flange frame.
- File examples:
  - `LeftArm_drag_hand_eye_051.yml`
  - `RightArm_drag_hand_eye_051.yml`

Recommended runtime flow:
1. Connect to the selected XMC18 arm.
2. Load `H_Flange2Cam`.
3. Start RealSense RGB/depth stream at `640x480 @ 15fps`.
4. Detect grasp point and target point in image coordinates.
5. Use depth and intrinsics to lift pixel points into camera 3D coordinates.
6. Transform camera points into flange/base coordinates.
7. Validate target safety and reachability.
8. Execute robot motion with conservative speed.
9. Stop camera and return robot to a safe state on exit or error.

Coordinate conventions:
- OpenCV/RealSense camera frame uses meters after deprojection.
- Rokae Cartesian position uses meters and radians.
- `cartPosture(CoordinateType::flangeInBase, ec)` returns flange pose in base frame.
- For eye-in-hand tasks, combine:
  - base-to-flange from robot state.
  - flange-to-camera from hand-eye calibration.
  - camera-to-object from perception/depth.

Safety defaults:
- Start with low speed.
- Avoid autonomous motion until calibration has been verified visually.
- Do not start realtime following unless robot state, camera frames, and transforms are valid.
