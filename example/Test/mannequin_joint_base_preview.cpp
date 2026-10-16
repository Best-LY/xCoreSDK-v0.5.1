#include <array>
#include <chrono>
#include <cmath>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <string>
#include <thread>

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <opencv2/core.hpp>

#include "robot_vision_config.hpp"
#include "rokae/model.h"
#include "rokae/robot.h"
#include "rokae/utility.h"

namespace {

void require(bool condition, const std::string& message) {
  if (!condition) throw std::runtime_error(message);
}

void check(const std::error_code& ec, const std::string& operation) {
  if (ec) throw std::runtime_error(operation + ": " + ec.message() +
                                   " (" + std::to_string(ec.value()) + ")");
}

Eigen::Isometry3d fromFrame(const rokae::Frame& frame) {
  Eigen::Matrix3d rotation;
  rokae::Utils::eulerToMatrix(
      Eigen::Vector3d(frame.rpy[2], frame.rpy[1], frame.rpy[0]), rotation);
  Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
  pose.linear() = rotation;
  pose.translation() = Eigen::Vector3d(
      frame.trans[0], frame.trans[1], frame.trans[2]);
  return pose;
}

Eigen::Isometry3d fromArray(const std::array<double, 16>& data) {
  Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
  for (int row = 0; row < 4; ++row)
    for (int col = 0; col < 4; ++col)
      pose.matrix()(row, col) = data[4 * row + col];
  return pose;
}

}  // namespace

int main(int argc, char** argv) {
  Eigen::Vector3d kneeCamera = Eigen::Vector3d::Constant(
      std::numeric_limits<double>::quiet_NaN());
  double capturedMs = std::numeric_limits<double>::quiet_NaN();
  double betweenBurstDifferenceM = std::numeric_limits<double>::quiet_NaN();
  std::string serial;
  try {
    for (int i = 1; i < argc; ++i) {
      const std::string arg = argv[i];
      if (arg == "--x" && i + 1 < argc) kneeCamera.x() = std::stod(argv[++i]);
      else if (arg == "--y" && i + 1 < argc) kneeCamera.y() = std::stod(argv[++i]);
      else if (arg == "--z" && i + 1 < argc) kneeCamera.z() = std::stod(argv[++i]);
      else if (arg == "--capture-ms" && i + 1 < argc) capturedMs = std::stod(argv[++i]);
      else if (arg == "--between-burst-m" && i + 1 < argc) {
        betweenBurstDifferenceM = std::stod(argv[++i]);
      } else if (arg == "--camera-serial" && i + 1 < argc) serial = argv[++i];
      else if (arg == "--help") {
        std::cout << "Usage: " << argv[0]
                  << " --camera-serial SERIAL --x M --y M --z M"
                     " --capture-ms MS --between-burst-m M\n";
        return 0;
      } else {
        throw std::runtime_error("unknown/incomplete argument: " + arg);
      }
    }
    require(serial == user_config::SN_RIGHT_CAM, "right camera serial required");
    require(kneeCamera.allFinite() && kneeCamera.z() >= 1.5 &&
                kneeCamera.z() <= 3.0 &&
                std::abs(kneeCamera.x()) <= 1.0 &&
                std::abs(kneeCamera.y()) <= 1.0,
            "knee point is outside the preview-only input range");
    require(std::isfinite(betweenBurstDifferenceM) &&
                betweenBurstDifferenceM >= 0 &&
                betweenBurstDifferenceM <= 0.02,
            "two independent knee bursts are not consistent");
    const double nowMs = std::chrono::duration<double, std::milli>(
                             std::chrono::system_clock::now().time_since_epoch())
                             .count();
    require(std::isfinite(capturedMs) && nowMs - capturedMs >= -2000 &&
                nowMs - capturedMs <= 30000,
            "knee observation is older than 30 s");

    const std::string calibration =
        "/home/arm/xCoreSDK-v0.5.1/example/function_config/"
        "RightArm_interactive_calib_axis_verified.yml";
    cv::FileStorage fs(calibration, cv::FileStorage::READ);
    require(fs.isOpened(), "cannot open axis-verified right hand-eye file");
    cv::Mat matrix;
    std::string calibrationSerial;
    double squareDepthEstimate = 0;
    fs["H_Flange2Cam"] >> matrix;
    fs["camera_serial"] >> calibrationSerial;
    fs["square_size_depth_estimate_m"] >> squareDepthEstimate;
    require(matrix.rows == 4 && matrix.cols == 4 &&
                calibrationSerial == serial,
            "hand-eye matrix/camera serial mismatch");
    Eigen::Isometry3d flangeToCamera = Eigen::Isometry3d::Identity();
    for (int row = 0; row < 4; ++row)
      for (int col = 0; col < 4; ++col)
        flangeToCamera.matrix()(row, col) = matrix.at<double>(row, col);
    require((flangeToCamera.linear().transpose() * flangeToCamera.linear() -
             Eigen::Matrix3d::Identity()).norm() < 0.02,
            "hand-eye rotation is invalid");

    rokae::xMateRobot robot(user_config::IP_RIGHT, user_config::LOCAL_IP);
    std::error_code ec;
    const auto info = robot.robotInfo(ec);
    check(ec, "robotInfo");
    require(info.type.find("XMC18") != std::string::npos,
            "connected arm is not XMC18");
    const auto state = robot.operationState(ec);
    check(ec, "operationState");
    require(state == rokae::OperationState::idle,
            "right arm is not idle; no coordinate preview");
    const auto startJoints = robot.jointPos(ec);
    check(ec, "jointPos");
    std::this_thread::sleep_for(std::chrono::milliseconds(400));
    const auto endJoints = robot.jointPos(ec);
    check(ec, "jointPos stationary recheck");
    for (int joint = 0; joint < 6; ++joint)
      require(std::abs(endJoints[joint] - startJoints[joint]) < 0.001,
              "right arm moved while converting camera point");

    const auto frame = robot.cartPosture(rokae::CoordinateType::flangeInBase, ec);
    check(ec, "cartPosture flangeInBase");
    const auto finalJoints = robot.jointPos(ec);
    check(ec, "jointPos final recheck");
    for (int joint = 0; joint < 6; ++joint)
      require(std::abs(finalJoints[joint] - endJoints[joint]) < 0.001,
              "right arm moved during coordinate conversion");
    const auto finalState = robot.operationState(ec);
    check(ec, "operationState final recheck");
    require(finalState == rokae::OperationState::idle,
            "right arm is no longer idle");
    const Eigen::Isometry3d baseToFlange = fromFrame(frame);
    rokae::xMateModel<6> model = robot.model();
    const Eigen::Isometry3d modelFlange = fromArray(model.getCartPose(finalJoints));
    require((modelFlange.translation() - baseToFlange.translation()).norm() < 0.01 &&
                Eigen::AngleAxisd(baseToFlange.linear().transpose() *
                                  modelFlange.linear()).angle() < 0.03,
            "controller and model flange poses disagree");

    const Eigen::Isometry3d baseToCamera = baseToFlange * flangeToCamera;
    const Eigen::Vector3d kneeBase = baseToCamera * kneeCamera;
    std::cout << "[ESTIMATE] arm=right controller=" << info.version
              << " knee_C=[" << kneeCamera.transpose() << "] m\n";
    std::cout << "[ESTIMATE] knee_B=[" << kneeBase.transpose()
              << "] m, camera_B=[" << baseToCamera.translation().transpose()
              << "] m\n";
    std::cout << "[UNCERTAINTY] two-burst median difference="
              << betweenBurstDifferenceM * 1000.0 << " mm; checker square="
              << user_config::SQUARE_SIZE * 1000.0
              << " mm vs depth estimate=" << squareDepthEstimate * 1000.0
              << " mm; verify hand-eye translation\n";
    std::cout << "[SAFE] base point preview only; gripper-tip transform is"
                 " unmeasured; no robot mode/power/motion changed\n";
    return 0;
  } catch (const std::exception& error) {
    std::cerr << "[ERROR] " << error.what() << '\n';
    return 2;
  }
}
