#pragma once

#include <cstddef>

namespace visual_servo_config {

// Camera and checkerboard settings must match drag_hand_eye_calibration.cpp.
inline constexpr int kColorWidth = 640;
inline constexpr int kColorHeight = 480;
inline constexpr int kCameraFps = 15;
inline constexpr double kDesiredBoardDistanceM = 0.60;

// Provisional optical-Z protrusion: user roughly measured the left tool at
// 0.30 m on 2026-09-30. This is NOT a calibrated TCP or a 3D clearance bound.
// Keep the board-following margin; mannequin approach has separate guards.
inline constexpr double kEstimatedToolProtrusionFromCameraM = 0.30;
inline constexpr double kMinimumToolToBoardClearanceM = 0.07;
inline constexpr double kMinimumAllowedBoardDistanceM =
    kEstimatedToolProtrusionFromCameraM + kMinimumToolToBoardClearanceM;

// Left-arm mannequin commissioning only, limited to a single step up to 2 mm.
// The rough tool measurement is not a TCP calibration. Reserve 0.10 m for
// its uncertainty and another 0.10 m for perception/model uncertainty.
inline constexpr double kLeftToolProtrusionUncertaintyM = 0.10;
inline constexpr double kMannequinNonContactClearanceM = 0.20;
inline constexpr double kMannequinPerceptionReserveM = 0.10;
inline constexpr double kLeftMicroApproachMinimumCameraDepthM = 1.00;

// User-reported measurement, 2026-10-07: 300 mm from flange mounting face
// along flange +Z to the effective grasp center. Not a six-DOF TCP calibration.
// Independent of the legacy camera protrusion estimate above.
inline constexpr double kLeftFlangeToGraspCenterM = 0.300;
// User-requested surface-to-grasp-center insertion along approach +Z.
// Staged left-elbow planning only; this does not release hardware execution.
inline constexpr double kLeftElbowSurfaceInsertionM = 0.080;
// Latest reach-first task; independent of the older fixed two-stage recipe.
inline constexpr double kLeftElbowReachInsertionM = 0.050;
inline constexpr const char* kLeftDemonstratedPoseFile =
    "/home/arm/xCoreSDK-v0.5.1/example/function_config/recorded_poses/"
    "20261006_manual_elbow_analysis/left.yml";
// Onsite swept-space clearance confirmed up to 60 degrees for this setup.
// Reconfirm clearance after any scene/base/other-arm change before execution.
inline constexpr double kLeftPregraspMaxRotationRad = 1.0471975511965976;

// Measurement conditioning.
inline constexpr std::size_t kMedianWindow = 5;
inline constexpr double kPositionEmaAlpha = 0.22;
inline constexpr double kMaxDepthPnPDifferenceM = 0.08;
inline constexpr std::size_t kMinDepthPatchPixels = 32;
inline constexpr double kMaxDepthPatchSpreadM = 0.03;
inline constexpr std::size_t kMinFallbackCornerDepths = 24;
inline constexpr double kMaxRawJumpM = 0.08;
inline constexpr double kMaxBoardSpeedMps = 0.60;
inline constexpr int kRelockFrames = 5;
inline constexpr double kRelockRadiusM = 0.015;
inline constexpr int kValidFramesToResume = 5;
inline constexpr int kLostFramesToResetFilter = 3;

// Command shaping. FollowPosition performs the 1 ms interpolation internally;
// vision only updates its target at the camera frame rate.
inline constexpr double kTranslationDeadbandM = 0.003;
// A newly detected or abruptly relocated target outside this envelope must not
// trigger a long Cartesian chase. Slow continuous motion remains trackable.
inline constexpr double kMaxCorrectionFromCurrentM = 0.18;
inline constexpr double kMaxCommandStepM = 0.008;
inline constexpr double kFollowSpeedScale = 0.20;
inline constexpr double kServoReadyPrincipalJointLimitRad =
    160.0 * 3.14159265358979323846 / 180.0;

// Predictive safety gates. IK also checks reachability and controller soft limits.
inline constexpr double kWorkspaceRadiusMaxM = 1.50;
inline constexpr double kWorkspaceZMinM = -0.20;
inline constexpr double kWorkspaceZMaxM = 1.80;
inline constexpr double kMaxPredictedJointStepRad = 0.12;
inline constexpr double kSafetyPathSampleStepM = 0.004;
inline constexpr double kMinJacobianSingularValue = 0.010;
inline constexpr double kMaxJacobianCondition = 300.0;

}  // namespace visual_servo_config
