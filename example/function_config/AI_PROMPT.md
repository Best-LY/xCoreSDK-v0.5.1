# AI Prompt - Functional Tools And Configuration

This folder contains runnable utility programs and shared configuration for the user's robot/camera system.

Use this folder for practical functional tools such as:
- Hand-eye calibration.
- RealSense RGB/depth display tests.
- Robot connection tests.
- Communication and data transfer tests.
- Shared robot IP, camera serial, checkerboard, and task parameters.

Current shared config:
- `robot_vision_config.hpp`
  - left arm: `192.168.2.161`
  - right arm: `192.168.2.160`
  - local PC: `192.168.2.6`
  - left RealSense serial: `236422072262`
  - right RealSense serial: `236422070786`
  - checkerboard: `8 x 6` inner corners
  - square size: `0.0286 m` (28.6 mm), confirmed by the user on 2026-09-29.
    Results calculated with the previous `0.020 m` setting for this board
    are invalid and must not be used for robot motion. Recalibrate both arms.
  - `visual_servo_config.hpp` records a provisional tool protrusion of
    `0.30 m` along camera optical Z, based on the user's rough left-tool
    measurement on 2026-09-30. It is not a calibrated camera-to-TCP transform.
    Changing this estimate alone does not invalidate hand-eye calibration if
    the camera-to-flange mounting is unchanged. Right-arm micro-approach
    retains its 1.5 m commissioning gate. The left-shoulder single 0.25..2 mm test
    uses the greater of 1.0 m camera depth or the tool projection plus 0.10 m
    tool uncertainty, 0.20 m non-contact clearance, 0.10 m perception reserve
    and the 0.002 m step. This is not authorization for continuous following
    or contact grasping. Never substitute the board follower's 0.07 m setting.

Rules for AI-generated code:
- Put runnable diagnostic/calibration/communication utilities here.
- Include `robot_vision_config.hpp` instead of hardcoding IPs, camera serials, or checkerboard parameters.
- Keep programs small and focused: one executable should validate one functional capability.
- Use `640x480 @ 15fps` for RealSense RGB/depth when stability matters.
- `drag_hand_eye_calibration.cpp` requires the physically measured square size
  in millimeters and checks it against aligned depth at capture time. Never
  silently reuse the legacy `0.020 m` value for a different board.
- New absolute grasp-point transforms require a verified hand-eye translation.
  The relative, fixed-orientation visual servo uses the hand-eye rotation;
  its translation cancels from the Cartesian correction.
- Use `LD_LIBRARY_PATH=/tmp/librealsense-install/lib:/tmp/libusb-1.0.27-install/lib:/tmp/opencv-4.11.0-install/lib` when running vision executables from the current local setup.

Build behavior:
- Complete `.cpp` files in this folder are automatically scanned by `../CMakeLists.txt`.
- If a file includes OpenCV or RealSense headers, CMake links OpenCV and librealsense automatically when their CMake packages are provided.
