#pragma once

#include <array>
#include <cmath>

namespace board_pick_demo_config {

// Right-arm repeatable start posture. It moves J1/J4 away from the old
// +/-180 degree home while retaining the existing camera-facing arrangement.
inline const std::array<double, 6> kRightInitialJoints = {
    160.0 * M_PI / 180.0,
     90.0 * M_PI / 180.0,
     90.0 * M_PI / 180.0,
   -160.0 * M_PI / 180.0,
      0.0,
   -225.0 * M_PI / 180.0,
};

// First center the board while its complete pattern remains visible.
inline constexpr double kCoarseCameraToBoardM = 0.60;

// Current measured mounting estimate. Replace this with the calibrated
// camera-to-grasp-point optical-axis distance when the final tool is fixed.
inline constexpr double kCameraBehindGraspPointM = 0.28;
inline constexpr double kFinalGraspPointToBoardM = 0.20;
inline constexpr double kFinalCameraToBoardM =
    kCameraBehindGraspPointM + kFinalGraspPointToBoardM;

inline constexpr double kLiftInBaseZM = 0.20;
inline constexpr int kHoldAfterCloseSeconds = 3;
inline constexpr int kStableDetectionFrames = 10;
inline constexpr double kMaximumInterframeBoardMoveM = 0.03;
inline constexpr double kMaximumStableBoardSpreadM = 0.012;
inline constexpr double kMaximumCoarseBoardShiftM = 0.04;

inline constexpr double kJointMoveSpeedMmPerS = 60.0;
inline constexpr double kCoarseMoveSpeedMmPerS = 80.0;
inline constexpr double kApproachMoveSpeedMmPerS = 35.0;
inline constexpr double kLiftMoveSpeedMmPerS = 45.0;
inline constexpr double kPathSampleStepM = 0.005;
inline constexpr double kWorkspaceRadiusMaxM = 1.50;
inline constexpr double kWorkspaceZMinM = -0.20;
inline constexpr double kWorkspaceZMaxM = 1.80;
inline constexpr double kMinimumJacobianSigma = 0.008;
inline constexpr double kMaximumJacobianCondition = 500.0;
inline constexpr double kJointLimitMarginRad = 0.02;
inline constexpr double kMaximumSampleJointStepRad = 0.20;
inline constexpr double kReachedJointToleranceRad = 0.015;
inline constexpr double kReachedPositionToleranceM = 0.003;
inline constexpr double kReachedOrientationToleranceRad = 0.02;

}  // namespace board_pick_demo_config
