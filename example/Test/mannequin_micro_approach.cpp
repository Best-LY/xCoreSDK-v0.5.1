#include <algorithm>
#include <array>
#include <chrono>
#include <csignal>
#include <cmath>
#include <iostream>
#include <limits>
#include <optional>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <librealsense2/rs.hpp>
#include <librealsense2/rsutil.h>
#include <opencv2/core.hpp>

#include "robot_vision_config.hpp"
#include "visual_servo_config.hpp"
#include "rokae/model.h"
#include "rokae/robot.h"
#include "rokae/utility.h"

namespace {

using Joints = std::array<double, 6>;
using Isometry = Eigen::Isometry3d;
using Model = rokae::xMateModel<6>;
volatile std::sig_atomic_t stopRequested = 0;
void onSignal(int) { stopRequested = 1; }

void require(bool valid, const std::string& message) {
  if (!valid) throw std::runtime_error(message);
}

void check(const std::error_code& ec, const std::string& action) {
  if (ec) throw std::runtime_error(action + ": " + ec.message() +
                                   " (" + std::to_string(ec.value()) + ")");
}

Isometry fromArray(const std::array<double, 16>& values) {
  Isometry pose = Isometry::Identity();
  for (int row = 0; row < 4; ++row)
    for (int col = 0; col < 4; ++col)
      pose.matrix()(row, col) = values[4 * row + col];
  return pose;
}

Isometry fromFrame(const rokae::Frame& frame) {
  Eigen::Matrix3d rotation;
  rokae::Utils::eulerToMatrix(
      Eigen::Vector3d(frame.rpy[2], frame.rpy[1], frame.rpy[0]), rotation);
  Isometry pose = Isometry::Identity();
  pose.linear() = rotation;
  pose.translation() = Eigen::Vector3d(frame.trans[0], frame.trans[1], frame.trans[2]);
  return pose;
}

Isometry cameraPose(Model& model, const Joints& q,
                    const Isometry& flangeToCamera) {
  return fromArray(model.getCartPose(q)) * flangeToCamera;
}

Eigen::Matrix<double, 6, 6> cameraJacobian(
    Model& model, const Joints& q, const Isometry& flangeToCamera) {
  Eigen::Matrix<double, 6, 6> jacobian;
  constexpr double epsilon = 1e-5;
  for (int i = 0; i < 6; ++i) {
    auto plus = q, minus = q;
    plus[i] += epsilon;
    minus[i] -= epsilon;
    const auto p = cameraPose(model, plus, flangeToCamera);
    const auto m = cameraPose(model, minus, flangeToCamera);
    jacobian.block<3, 1>(0, i) =
        (p.translation() - m.translation()) / (2.0 * epsilon);
    const Eigen::Matrix3d delta = p.linear() * m.linear().transpose();
    const Eigen::AngleAxisd angle(delta);
    jacobian.block<3, 1>(3, i) =
        angle.axis() * angle.angle() / (2.0 * epsilon);
  }
  return jacobian;
}

void verifyStationary(rokae::xMateRobot& robot, Joints* joints) {
  std::error_code ec;
  require(robot.operationState(ec) == rokae::OperationState::idle,
          "selected arm is not idle");
  check(ec, "operationState");
  *joints = robot.jointPos(ec);
  check(ec, "jointPos");
  for (int sample = 0; sample < 2; ++sample) {
    std::this_thread::sleep_for(std::chrono::milliseconds(200));
    const Joints next = robot.jointPos(ec);
    check(ec, "jointPos stationary check");
    for (int i = 0; i < 6; ++i)
      require(std::abs(next[i] - (*joints)[i]) < 0.001,
              "selected arm moved during stationary check at J" +
                  std::to_string(i + 1));
    *joints = next;
  }
}

struct Point {
  double x = std::numeric_limits<double>::quiet_NaN();
  double y = std::numeric_limits<double>::quiet_NaN();
  double depth = std::numeric_limits<double>::quiet_NaN();
  double person = std::numeric_limits<double>::quiet_NaN();
  double knee = std::numeric_limits<double>::quiet_NaN();
  double spread = std::numeric_limits<double>::quiet_NaN();
  double capturedMs = std::numeric_limits<double>::quiet_NaN();
  std::string serial;
  std::string joint;
};

double minimumCameraDepth(bool leftShoulder) {
  if (!leftShoulder) return 1.5;
  namespace cfg = visual_servo_config;
  return std::max(cfg::kLeftMicroApproachMinimumCameraDepthM,
      cfg::kEstimatedToolProtrusionFromCameraM + cfg::kLeftToolProtrusionUncertaintyM +
      cfg::kMannequinNonContactClearanceM + cfg::kMannequinPerceptionReserveM + 0.002);
}

void validatePoint(const Point& point, bool leftShoulder) {
  require(point.serial == (leftShoulder ? user_config::SN_LEFT_CAM : user_config::SN_RIGHT_CAM) &&
              point.joint == (leftShoulder ? "left_shoulder" : "right_knee"),
          "point does not match selected camera/joint");
  const double nowMs = std::chrono::duration<double, std::milli>(
                           std::chrono::system_clock::now().time_since_epoch())
                           .count();
  require(std::isfinite(point.capturedMs) &&
              nowMs - point.capturedMs >= -2000 &&
              nowMs - point.capturedMs <= 15000,
          "joint point is older than 15 s; capture again");
  require(point.person >= 0.5 && point.knee >= 0.9 &&
              point.spread >= 0 && point.spread < 0.04 &&
              std::isfinite(point.x) && std::isfinite(point.y) &&
              point.x >= 10 && point.x < 1270 && point.y >= 10 &&
              point.y < 710,
          "joint confidence, pixel or depth-spread check failed");
  const double minimumDepth = minimumCameraDepth(leftShoulder);
  require(std::isfinite(point.depth) && point.depth >= minimumDepth && point.depth <= 3.0,
          "joint camera depth=" + std::to_string(point.depth) +
              " m is outside commissioning range [" + std::to_string(minimumDepth) +
              ", 3.000000] m; no motion");
}

double liveDepth(rs2::pipeline& pipe, rs2::align& align, const Point& point) {
  std::vector<double> depths;
  for (int frame = 0; frame < 7; ++frame) {
    auto images = align.process(pipe.wait_for_frames(5000));
    const auto depth = images.get_depth_frame();
    require(depth && depth.get_width() == 1280 && depth.get_height() == 720,
            "selected camera color/depth are not aligned at 1280x720");
    if (frame < 2) continue;
    std::vector<double> patch;
    for (int dy = -5; dy <= 5; ++dy)
      for (int dx = -5; dx <= 5; ++dx) {
        const double z = depth.get_distance(static_cast<int>(std::round(point.x)) + dx,
                                            static_cast<int>(std::round(point.y)) + dy);
        if (z >= 0.35 && z <= 3.0) patch.push_back(z);
      }
    require(patch.size() >= 80, "not enough live depth pixels at knee");
    std::sort(patch.begin(), patch.end());
    require(patch[patch.size() * 9 / 10] - patch[patch.size() / 10] < 0.05,
            "live knee depth patch crosses a surface edge");
    depths.push_back(patch[patch.size() / 2]);
  }
  std::sort(depths.begin(), depths.end());
  return depths[depths.size() / 2];
}

void validatePath(Model& model, const Joints& start, const Joints& target,
                  const std::array<double[2], 6>& limits,
                  const Isometry& flangeToCamera, double stepM,
                  const Eigen::Vector3d& direction) {
  const auto cameraStart = cameraPose(model, start, flangeToCamera);
  const auto flangeStart = fromArray(model.getCartPose(start));
  std::array<Eigen::Vector3d, 6> linkStart{};
  for (int link = 0; link < 6; ++link)
    linkStart[link] = fromArray(model.getCartPose(
        start, static_cast<rokae::SegmentFrame>(link + 1))).translation();
  for (int sample = 0; sample <= 20; ++sample) {
    Joints q{};
    for (int i = 0; i < 6; ++i) {
      q[i] = start[i] + (target[i] - start[i]) * sample / 20.0;
      require(q[i] >= limits[i][0] + 0.05 &&
                  q[i] <= limits[i][1] - 0.05,
              "joint path approaches soft limit at J" + std::to_string(i + 1));
    }
    const auto flange = fromArray(model.getCartPose(q));
    const auto jacobian = cameraJacobian(model, q, flangeToCamera);
    const auto singular = jacobian.jacobiSvd().singularValues();
    require(singular.minCoeff() >= 0.01 &&
                singular.maxCoeff() / singular.minCoeff() <= 300,
            "camera Jacobian approaches singularity along path");
    const auto p = flange.translation();
    require(std::hypot(p.x(), p.y()) < visual_servo_config::kWorkspaceRadiusMaxM &&
                p.z() > visual_servo_config::kWorkspaceZMinM &&
                p.z() < visual_servo_config::kWorkspaceZMaxM,
            "joint path leaves configured workspace");
    require((p - flangeStart.translation()).norm() < 0.015,
            "flange moves more than 15 mm");
    for (int link = 0; link < 6; ++link) {
      const auto linkPos = fromArray(model.getCartPose(
          q, static_cast<rokae::SegmentFrame>(link + 1))).translation();
      require((linkPos - linkStart[link]).norm() < 0.03,
              "a robot link moves more than 30 mm");
    }
    const auto camera = flange * flangeToCamera;
    const Eigen::Vector3d displacement =
        cameraStart.linear().transpose() *
        (camera.translation() - cameraStart.translation());
    const double rotation = Eigen::AngleAxisd(
        cameraStart.linear().transpose() * camera.linear()).angle();
    const double forward = displacement.dot(direction);
    require((displacement - forward * direction).norm() < 0.0005 &&
                forward >= -0.0005 && forward < stepM + 0.0005 && rotation < 0.001,
            "camera path deviates from the short approach direction");
  }
  const auto end = cameraPose(model, target, flangeToCamera);
  const Eigen::Vector3d delta = cameraStart.linear().transpose() *
                                (end.translation() - cameraStart.translation());
  require(delta.dot(direction) >= stepM * 0.75, "predicted approach is too small");
  std::cout << "[PATH] predicted camera displacement [" << delta.transpose()
            << "] m; 21 samples passed\n";
}

void cleanup(rokae::xMateRobot& robot) {
  std::error_code ec;
  robot.setMotionControlMode(rokae::MotionControlMode::Idle, ec);
  if (ec) std::cerr << "[CLEANUP] set Idle: " << ec.message() << '\n';
  robot.setPowerState(false, ec);
  if (ec) std::cerr << "[CLEANUP] power off: " << ec.message() << '\n';
  robot.setOperateMode(rokae::OperateMode::manual, ec);
  if (ec) std::cerr << "[CLEANUP] set manual: " << ec.message() << '\n';
}

}  // namespace

int main(int argc, char** argv) {
  std::signal(SIGINT, onSignal);
  std::signal(SIGTERM, onSignal);
  try {
  Point point;
  double stepMm = 5.0;
  bool execute = false;
  bool leftShoulder = false;
  std::optional<Joints> expectedJoints;
  for (int i = 1; i < argc; ++i) {
    const std::string arg = argv[i];
    if (arg == "--pixel-x" && i + 1 < argc) point.x = std::stod(argv[++i]);
    else if (arg == "--pixel-y" && i + 1 < argc) point.y = std::stod(argv[++i]);
    else if (arg == "--depth-m" && i + 1 < argc) point.depth = std::stod(argv[++i]);
    else if (arg == "--person-conf" && i + 1 < argc) point.person = std::stod(argv[++i]);
    else if (arg == "--knee-conf" && i + 1 < argc) point.knee = std::stod(argv[++i]);
    else if (arg == "--spread-m" && i + 1 < argc) point.spread = std::stod(argv[++i]);
    else if (arg == "--capture-ms" && i + 1 < argc) point.capturedMs = std::stod(argv[++i]);
    else if (arg == "--camera-serial" && i + 1 < argc) point.serial = argv[++i];
    else if (arg == "--joint" && i + 1 < argc) point.joint = argv[++i];
    else if (arg == "--step-mm" && i + 1 < argc) stepMm = std::stod(argv[++i]);
    else if (arg == "--execute") execute = true;
    else if (arg == "--left-shoulder") leftShoulder = true;
    else if (arg == "--expected-joints" && i + 6 < argc) {
      Joints q{};
      for (auto& value : q) value = std::stod(argv[++i]);
      expectedJoints = q;
    }
    else if (arg == "--help") {
      std::cout << "Usage: " << argv[0]
                << " --pixel-x N --pixel-y N --depth-m N --person-conf N"
                   " --knee-conf N --spread-m N --capture-ms N"
                   " --camera-serial SERIAL --joint right_knee"
                   " [--step-mm 2..10] [--execute]"
                   " [--left-shoulder: left camera/left_shoulder, 0.25..2 mm]\n";
      return 0;
    } else {
      std::cerr << "[ERROR] unknown/incomplete argument: " << arg << '\n';
      return 2;
    }
  }
    require(std::isfinite(stepMm) &&
                (leftShoulder ? (stepMm >= 0.25 && stepMm <= 2.0)
                              : (stepMm >= 2.0 && stepMm <= 10.0)),
            "step must be 0.25..2 mm for left shoulder or 2..10 mm for right knee");
    const double stepM = stepMm / 1000.0;
    require(!leftShoulder || expectedJoints.has_value(),
            "left-shoulder step requires joints paired with the camera capture");
    validatePoint(point, leftShoulder);
    const std::string selectedSerial = leftShoulder ? user_config::SN_LEFT_CAM : user_config::SN_RIGHT_CAM;
    const std::string calibration =
        "/home/arm/xCoreSDK-v0.5.1/example/function_config/"
        + std::string(leftShoulder ? "LeftArm_interactive_calib.yml"
                                  : "RightArm_interactive_calib_axis_verified.yml");
    cv::FileStorage fs(calibration, cv::FileStorage::READ);
    require(fs.isOpened(), "selected calibration is missing");
    std::string serial;
    cv::Mat matrix;
    fs["camera_serial"] >> serial;
    fs["H_Flange2Cam"] >> matrix;
    require(serial == selectedSerial && matrix.rows == 4 && matrix.cols == 4 &&
                matrix.type() == CV_64F && cv::checkRange(matrix),
            "camera calibration does not match selected arm");
    if (leftShoulder) {
      cv::FileStorage validated(
          "/home/arm/xCoreSDK-v0.5.1/example/function_config/calibration_runs/"
          "left_validation_20260930_i2JHhp/calibration_used.yml", cv::FileStorage::READ);
      require(validated.isOpened(), "three-pose calibration reference is missing");
      cv::Mat reference;
      validated["H_Flange2Cam"] >> reference;
      require(reference.size() == matrix.size() && reference.type() == matrix.type() &&
                  cv::checkRange(reference) && cv::norm(matrix, reference) < 1e-10,
              "left calibration changed since the three-pose validation");
    }
    Isometry flangeToCamera = Isometry::Identity();
    for (int row = 0; row < 4; ++row)
      for (int col = 0; col < 4; ++col)
        flangeToCamera.matrix()(row, col) = matrix.at<double>(row, col);
    require((flangeToCamera.linear().transpose() * flangeToCamera.linear() -
             Eigen::Matrix3d::Identity()).norm() < 0.02 &&
                std::abs(flangeToCamera.linear().determinant() - 1.0) < 0.02 &&
                (flangeToCamera.matrix().row(3) - Eigen::RowVector4d(0,0,0,1)).norm() < 1e-9,
            "camera rotation matrix is invalid");

    rs2::pipeline pipe;
    rs2::config config;
    config.enable_device(selectedSerial);
    config.enable_stream(RS2_STREAM_COLOR, 1280, 720, RS2_FORMAT_BGR8, 15);
    config.enable_stream(RS2_STREAM_DEPTH, 640, 480, RS2_FORMAT_Z16, 15);
    const auto profile = pipe.start(config);
    rs2::align align(RS2_STREAM_COLOR);
    const double depthBefore = liveDepth(pipe, align, point);
    require(std::abs(depthBefore - point.depth) < 0.05 &&
                depthBefore >= minimumCameraDepth(leftShoulder),
            "live knee depth changed since snapshot");
    if (leftShoulder) {
      std::cout << "[CLEARANCE] camera_depth=" << depthBefore
                << " m minimum=" << minimumCameraDepth(true)
                << " m; provisional tool projection="
                << visual_servo_config::kEstimatedToolProtrusionFromCameraM
                << " m plus 0.10 m tool and 0.10 m perception reserves;"
                   " single non-contact test up to 2 mm only\n";
    }

    rokae::xMateRobot robot(leftShoulder ? user_config::IP_LEFT : user_config::IP_RIGHT,
                          user_config::LOCAL_IP);
    std::error_code ec;
    const auto info = robot.robotInfo(ec);
    check(ec, "robotInfo");
    require(info.type.find("XMC18") != std::string::npos,
            "connected robot is not XMC18");
    auto model = robot.model();
    Joints start{};
    verifyStationary(robot, &start);
    if (expectedJoints) {
      for (int i = 0; i < 6; ++i)
        require(std::isfinite((*expectedJoints)[i]) &&
                    std::abs(start[i] - (*expectedJoints)[i]) < 0.0005,
                "robot posture changed since shoulder observation");
    }
    std::array<double[2], 6> limits{};
    const bool limitsEnabled = robot.getSoftLimit(limits, ec);
    check(ec, "getSoftLimit");
    require(limitsEnabled, "controller soft limits are disabled");
    const auto measuredFlange = fromFrame(robot.cartPosture(
        rokae::CoordinateType::flangeInBase, ec));
    check(ec, "cartPosture");
    const auto modelFlange = fromArray(model.getCartPose(start));
    require((measuredFlange.translation() - modelFlange.translation()).norm() < 0.03 &&
                Eigen::AngleAxisd(measuredFlange.linear().transpose() *
                                  modelFlange.linear()).angle() < 0.1,
            "controller/model flange poses disagree");

    const auto camera = modelFlange * flangeToCamera;
    Eigen::Vector3d direction(0, 0, 1);
    if (leftShoulder) {
      const auto intrinsics = profile.get_stream(RS2_STREAM_COLOR)
          .as<rs2::video_stream_profile>().get_intrinsics();
      const float uv[2]{static_cast<float>(point.x), static_cast<float>(point.y)};
      float xyz[3]{};
      rs2_deproject_pixel_to_point(xyz, &intrinsics, uv, static_cast<float>(depthBefore));
      direction = Eigen::Vector3d(xyz[0], xyz[1], xyz[2]).normalized();
      require(direction.allFinite() && direction.z() > 0.95,
              "shoulder is too far from the camera forward direction");
    }
    const auto jacobian = cameraJacobian(model, start, flangeToCamera);
    const auto singular = jacobian.jacobiSvd().singularValues();
    const double sigma = singular.minCoeff();
    const double condition = singular.maxCoeff() / sigma;
    require(sigma >= 0.01 && condition <= 300,
            "camera Jacobian is near singular");
    Eigen::Matrix<double, 6, 1> desired = Eigen::Matrix<double, 6, 1>::Zero();
    desired.head<3>() = camera.linear() * direction * stepM;
    constexpr double damping = 0.002;
    std::optional<Joints> validTarget;
    for (const double rotationWeight : {1.0, 0.5, 0.2, 0.1, 0.05, 0.02,
                                        0.01, 0.0}) {
      auto weightedJacobian = jacobian;
      weightedJacobian.bottomRows<3>() *= rotationWeight;
      const Eigen::Matrix<double, 6, 1> dq =
          (weightedJacobian.transpose() * weightedJacobian +
           damping * damping * Eigen::Matrix<double, 6, 6>::Identity())
              .ldlt()
              .solve(weightedJacobian.transpose() * desired);
      const double maxStep = dq.cwiseAbs().maxCoeff();
      std::cout << "[MODEL] sigma=" << sigma << " cond=" << condition
                << " rotation_weight=" << rotationWeight
                << " max_joint_step=" << maxStep << " rad\n";
      if (!dq.allFinite() || maxStep > (leftShoulder ? 0.005 : 0.04)) continue;
      Joints candidate{};
      for (int i = 0; i < 6; ++i) candidate[i] = start[i] + dq[i];
      try {
        validatePath(model, start, candidate, limits, flangeToCamera, stepM, direction);
        validTarget = candidate;
        break;
      } catch (const std::exception& error) {
        std::cout << "[MODEL] candidate rejected: " << error.what() << '\n';
      }
    }
    require(validTarget.has_value(),
            "no requested small-step path meets joint/link/camera guards; no motion");
    const Joints target = *validTarget;
    std::cout << "[PLAN] arm=" << (leftShoulder ? "left" : "right")
              << " joint=" << point.joint << " XMC18 controller=" << info.version
              << " joint_depth=" << depthBefore << " m, sigma=" << sigma
              << " cond=" << condition << " step=" << stepMm << " mm\n";
    std::cout << "[PLAN] target joints rad=[";
    for (int i = 0; i < 6; ++i)
      std::cout << (i ? ", " : "") << target[i];
    std::cout << "]\n";
    if (!execute) {
      pipe.stop();
      std::cout << "[SAFE] dry run; no robot mode/power/motion changed\n";
      return 0;
    }

    validatePoint(point, leftShoulder);
    const double preMoveDepth = liveDepth(pipe, align, point);
    require(std::abs(preMoveDepth - depthBefore) < 0.03 &&
                preMoveDepth >= minimumCameraDepth(leftShoulder),
            "knee depth changed just before command");
    Joints now{};
    verifyStationary(robot, &now);
    for (int i = 0; i < 6; ++i)
      require(std::abs(now[i] - start[i]) < 0.001,
              "starting posture changed after path validation");
    require(!stopRequested, "interrupted before motion");

    bool configured = false;
    bool commandIssued = false;
    bool depthConsistent = true;
    try {
      configured = true;
      robot.setOperateMode(rokae::OperateMode::automatic, ec);
      check(ec, "setOperateMode automatic");
      robot.setPowerState(true, ec);
      check(ec, "setPowerState on");
      robot.setMotionControlMode(rokae::MotionControlMode::NrtCommand, ec);
      check(ec, "setMotionControlMode NrtCommand");
      robot.moveReset(ec);
      check(ec, "moveReset");
      robot.setDefaultSpeed(10.0, ec);
      check(ec, "setDefaultSpeed 10 mm/s");
      robot.setDefaultZone(0, ec);
      check(ec, "setDefaultZone 0");
      require(!stopRequested, "interrupted before command");
      rokae::MoveAbsJCommand command(
          std::vector<double>(target.begin(), target.end()), 10.0, 0.0);
      command.jointSpeed = 0.005;
      commandIssued = true;
      robot.executeCommand({command}, ec);
      check(ec, "execute MoveAbsJ micro approach");
      const auto deadline = std::chrono::steady_clock::now() +
                            std::chrono::seconds(20);
      bool arrived = false;
      while (std::chrono::steady_clock::now() < deadline) {
        require(!stopRequested, "interrupted during motion");
        std::this_thread::sleep_for(std::chrono::milliseconds(150));
        const auto state = robot.operationState(ec);
        check(ec, "operationState after command");
        const auto movingJoints = robot.jointPos(ec);
        check(ec, "jointPos while moving");
        for (int i = 0; i < 6; ++i) {
          require(std::isfinite(movingJoints[i]) &&
                      movingJoints[i] >= std::min(start[i], target[i]) - 0.002 &&
                      movingJoints[i] <= std::max(start[i], target[i]) + 0.002,
                  "joint left the bounded small-step interval");
        }
        if (state == rokae::OperationState::idle) {
          arrived = true;
          break;
        }
      }
      if (!arrived) {
        robot.stop(ec);
        check(ec, "stop after timeout");
        throw std::runtime_error("micro approach timed out");
      }
      const auto actual = robot.jointPos(ec);
      check(ec, "jointPos after command");
      for (int i = 0; i < 6; ++i)
        require(std::isfinite(actual[i]) && std::abs(actual[i] - target[i]) < 0.0005,
                "actual J" + std::to_string(i + 1) +
                    " did not reach the small target");
      const auto actualCamera = cameraPose(model, actual, flangeToCamera);
      const Eigen::Vector3d actualDelta = camera.linear().transpose() *
          (actualCamera.translation() - camera.translation());
      std::cout << "[RESULT] encoder/FK camera displacement_C=["
                << actualDelta.transpose() << "] m, approach="
                << actualDelta.dot(direction) * 1000.0 << " mm\n";
      require(actualDelta.dot(direction) >= 0.5 * stepM &&
                  (actualDelta - direction * stepM).norm() <= 0.001,
              "encoder/FK displacement differs from requested small step");
      const double depthAfter = liveDepth(pipe, align, point);
      const double measuredApproach = preMoveDepth - depthAfter;
      depthConsistent = std::abs(measuredApproach - stepM) <= 0.005;
      std::cout << "[RESULT] joint depth before=" << preMoveDepth
                << " m after=" << depthAfter
                << " m delta=" << measuredApproach * 1000.0
                << " mm (same image pixel; approximate)\n";
      if (!depthConsistent)
        std::cerr << "[WARN] measured depth change disagrees with model; "
                     "do not use this as a confirmed approach\n";
    } catch (...) {
      if (commandIssued) {
        std::error_code stopEc;
        robot.stop(stopEc);
        if (stopEc) std::cerr << "[CLEANUP] stop: " << stopEc.message() << '\n';
      }
      if (configured) cleanup(robot);
      pipe.stop();
      throw;
    }
    cleanup(robot);
    pipe.stop();
    std::cout << "[OK] one short robot step finished; returned to manual/power-off\n";
    return depthConsistent ? 0 : 3;
  } catch (const std::exception& error) {
    std::cerr << "[ERROR] " << error.what() << '\n';
    return 2;
  }
}
