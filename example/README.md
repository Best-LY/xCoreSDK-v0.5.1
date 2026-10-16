# Example Workspace Layout

This example workspace is organized into three user-facing code areas.

## original_cases

Official xCoreSDK v0.5.1 example source code.

Purpose:
- Preserve original SDK usage patterns.
- Provide reference implementations for robot connection, motion, realtime control, model usage, force control, and communication.

Build:
- `.cpp` files here are automatically scanned and compiled.

## function_config

Runnable functional tools and shared configuration.

Purpose:
- Hand-eye calibration.
- Connection tests.
- Camera tests.
- Communication and data transfer tools.
- Shared robot/camera/calibration parameters.

Build:
- `.cpp` files here are automatically scanned and compiled.

Important files:
- `robot_vision_config.hpp`
- `drag_hand_eye_calibration.cpp`

## task_library

Reference library for future AI-generated integrated task code.

Purpose:
- Store prompts, task contracts, architecture notes, pseudocode, and reference snippets.
- Help an AI model synthesize final robot task code for depth-camera grasping and target placement.

Build:
- Files here are not automatically compiled.
- Put runnable `.cpp` programs in `function_config` instead.

## Build Note

After adding a new `.cpp` to `original_cases` or `function_config`, rerun CMake or build from a CMake build directory configured with `CONFIGURE_DEPENDS`.

Example:

```bash
cmake -S /home/arm/xCoreSDK-v0.5.1 -B /tmp/xcore051-auto-build \
  -DCMAKE_BUILD_TYPE=Release \
  -DXCORE_USE_XMATE_MODEL=ON \
  -DOpenCV_DIR=/tmp/opencv-4.11.0-install/lib/cmake/opencv4 \
  -Drealsense2_DIR=/tmp/librealsense-install/lib/cmake/realsense2 \
  -DCMAKE_PREFIX_PATH='/tmp/librealsense-install;/tmp/libusb-1.0.27-install;/tmp/opencv-4.11.0-install'

cmake --build /tmp/xcore051-auto-build --parallel 4
```
