#include <algorithm>
#include <any>
#include <array>
#include <chrono>
#include <cmath>
#include <csignal>
#include <filesystem>
#include <iomanip>
#include <iostream>
#include <limits>
#include <numeric>
#include <stdexcept>
#include <string>
#include <system_error>
#include <thread>
#include <vector>

#include <arpa/inet.h>
#include <ifaddrs.h>
#include <netinet/in.h>
#include <sys/socket.h>

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <librealsense2/rs.hpp>
#include <librealsense2/rsutil.h>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/opencv.hpp>

#include "board_pick_demo_config.hpp"
#include "robot_vision_config.hpp"
#include "visual_servo_config.hpp"
#include "rokae/model.h"
#include "rokae/robot.h"
#include "rokae/utility.h"

namespace {

namespace demo = board_pick_demo_config;
namespace vision = visual_servo_config;
using Isometry = Eigen::Isometry3d;

volatile std::sig_atomic_t gStop = 0;
void onSignal(int) { gStop = 1; }

struct Options {
  bool execute = false;
  bool gui = true;
  std::string robotIp = user_config::IP_RIGHT;
  std::string localIp = user_config::LOCAL_IP;
  std::string cameraSerial = user_config::SN_RIGHT_CAM;
  std::string calibrationFile;
  bool selfTest = false;
  bool help = false;
};

std::string defaultCalibrationFile() {
  const auto exampleDir = std::filesystem::path(__FILE__).parent_path().parent_path();
  const auto directory = exampleDir / "function_config";
  const auto verified = directory / "RightArm_interactive_calib_axis_verified.yml";
  return verified.string();
}

void usage(const char* program) {
  std::cout
      << "Usage: " << program << " [options]\n"
      << "  --execute              enable robot motion; default is dry-run\n"
      << "  --no-gui               use terminal Enter instead of camera window\n"
      << "  --ip <address>         right robot IP\n"
      << "  --local <address>      local realtime-network IP\n"
      << "  --camera <serial>      right RealSense serial\n"
      << "  --calib <file.yml>     H_Flange2Cam calibration file\n"
      << "  --self-test            verify geometry without hardware\n"
      << "  --help                 show this text\n\n"
      << "In the preview window press Enter or S for one cycle, Q to quit.\n";
}

bool parseArgs(int argc, char** argv, Options* options) {
  for (int i = 1; i < argc; ++i) {
    const std::string arg = argv[i];
    auto value = [&](const char* name) -> const char* {
      if (++i >= argc) {
        std::cerr << "[ERROR] missing value for " << name << '\n';
        return nullptr;
      }
      return argv[i];
    };
    if (arg == "--execute") options->execute = true;
    else if (arg == "--no-gui") options->gui = false;
    else if (arg == "--self-test") options->selfTest = true;
    else if (arg == "--ip") {
      const char* v = value("--ip"); if (!v) return false; options->robotIp = v;
    } else if (arg == "--local") {
      const char* v = value("--local"); if (!v) return false; options->localIp = v;
    } else if (arg == "--camera") {
      const char* v = value("--camera"); if (!v) return false; options->cameraSerial = v;
    } else if (arg == "--calib") {
      const char* v = value("--calib"); if (!v) return false; options->calibrationFile = v;
    } else if (arg == "--help" || arg == "-h") {
      options->help = true;
      return true;
    } else {
      std::cerr << "[ERROR] unknown option: " << arg << '\n';
      return false;
    }
  }
  return true;
}

bool checkEc(const std::string& action, const std::error_code& ec) {
  if (!ec) return true;
  std::cerr << "[ERROR] " << action << ": " << ec.message()
            << " (" << ec.value() << ")\n";
  return false;
}

bool localIpv4Exists(const std::string& expected) {
  ifaddrs* interfaces = nullptr;
  if (getifaddrs(&interfaces) != 0) return false;
  bool found = false;
  for (ifaddrs* item = interfaces; item; item = item->ifa_next) {
    if (!item->ifa_addr || item->ifa_addr->sa_family != AF_INET) continue;
    char address[INET_ADDRSTRLEN]{};
    const auto* ipv4 = reinterpret_cast<const sockaddr_in*>(item->ifa_addr);
    if (inet_ntop(AF_INET, &ipv4->sin_addr, address, sizeof(address)) &&
        expected == address) {
      found = true;
      break;
    }
  }
  freeifaddrs(interfaces);
  return found;
}

Isometry frameToIsometry(const rokae::Frame& frame) {
  Eigen::Matrix3d rotation;
  rokae::Utils::eulerToMatrix(
      Eigen::Vector3d(frame.rpy[2], frame.rpy[1], frame.rpy[0]), rotation);
  Isometry result = Isometry::Identity();
  result.linear() = rotation;
  result.translation() =
      Eigen::Vector3d(frame.trans[0], frame.trans[1], frame.trans[2]);
  return result;
}

std::array<double, 16> isometryArray(const Isometry& transform) {
  std::array<double, 16> values{};
  for (int row = 0; row < 4; ++row) {
    for (int col = 0; col < 4; ++col) {
      values[row * 4 + col] = transform.matrix()(row, col);
    }
  }
  return values;
}

rokae::CartesianPosition cartesianPosition(const Isometry& transform) {
  std::array<double, 6> posture{};
  rokae::Utils::transArrayToPosture(isometryArray(transform), posture);
  return rokae::CartesianPosition(
      {posture[0], posture[1], posture[2], posture[3], posture[4], posture[5]});
}

bool loadCalibration(const std::string& path,
                     const std::string& cameraSerial,
                     Isometry* flangeToCamera) {
  cv::FileStorage storage(path, cv::FileStorage::READ);
  cv::Mat matrix;
  if (!storage.isOpened()) {
    std::cerr << "[ERROR] cannot open calibration " << path << '\n';
    return false;
  }
  storage["H_Flange2Cam"] >> matrix;
  std::string storedSerial;
  int boardWidth = 0;
  int boardHeight = 0;
  if (!storage["camera_serial"].empty()) {
    storage["camera_serial"] >> storedSerial;
  }
  if (!storage["board_width"].empty()) storage["board_width"] >> boardWidth;
  if (!storage["board_height"].empty()) storage["board_height"] >> boardHeight;
  if (storedSerial != cameraSerial ||
      boardWidth != user_config::BOARD_SIZE.width ||
      boardHeight != user_config::BOARD_SIZE.height) {
    std::cerr << "[ERROR] calibration metadata does not match the selected "
                 "camera/checkerboard\n";
    return false;
  }
  if (matrix.rows != 4 || matrix.cols != 4) {
    std::cerr << "[ERROR] H_Flange2Cam is not 4x4\n";
    return false;
  }
  cv::Mat values;
  matrix.convertTo(values, CV_64F);
  *flangeToCamera = Isometry::Identity();
  for (int row = 0; row < 4; ++row) {
    for (int col = 0; col < 4; ++col) {
      flangeToCamera->matrix()(row, col) = values.at<double>(row, col);
    }
  }
  const double orthogonalError =
      (flangeToCamera->linear().transpose() * flangeToCamera->linear() -
       Eigen::Matrix3d::Identity()).norm();
  if (!flangeToCamera->matrix().allFinite() || orthogonalError > 0.02 ||
      std::abs(flangeToCamera->linear().determinant() - 1.0) > 0.02 ||
      (flangeToCamera->matrix().row(3) -
       Eigen::RowVector4d(0.0, 0.0, 0.0, 1.0)).norm() > 1e-6) {
    std::cerr << "[ERROR] invalid hand-eye rotation\n";
    return false;
  }
  std::cout << "[OK] calibration=" << path << '\n';
  return true;
}

double median(std::vector<double> values) {
  if (values.empty()) return std::numeric_limits<double>::quiet_NaN();
  const std::size_t middle = values.size() / 2;
  std::nth_element(values.begin(), values.begin() + middle, values.end());
  if (values.size() % 2) return values[middle];
  const double upper = values[middle];
  std::nth_element(values.begin(), values.begin() + middle - 1, values.end());
  return 0.5 * (upper + values[middle - 1]);
}

double localDepth(const rs2::depth_frame& depth, int px, int py) {
  std::vector<double> values;
  for (int y = py - 3; y <= py + 3; ++y) {
    for (int x = px - 3; x <= px + 3; ++x) {
      if (x < 0 || y < 0 || x >= depth.get_width() || y >= depth.get_height()) continue;
      const double value = depth.get_distance(x, y);
      if (value > 0.10 && value < 2.0 && std::isfinite(value)) values.push_back(value);
    }
  }
  return median(std::move(values));
}

struct BoardObservation {
  bool detected = false;
  Eigen::Vector3d center = Eigen::Vector3d::Zero();
  std::vector<cv::Point2f> corners;
};

BoardObservation observeBoard(const rs2::video_frame& color,
                              const rs2::depth_frame& depth,
                              const rs2_intrinsics& intrinsics,
                              cv::Mat* display) {
  cv::Mat image(cv::Size(vision::kColorWidth, vision::kColorHeight), CV_8UC3,
                const_cast<void*>(color.get_data()), cv::Mat::AUTO_STEP);
  *display = image.clone();
  cv::Mat gray;
  cv::cvtColor(image, gray, cv::COLOR_BGR2GRAY);
  BoardObservation observation;
  observation.detected = cv::findChessboardCorners(
      gray, user_config::BOARD_SIZE, observation.corners,
      cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_NORMALIZE_IMAGE);
  if (!observation.detected) return observation;
  cv::cornerSubPix(gray, observation.corners, cv::Size(11, 11), cv::Size(-1, -1),
                   cv::TermCriteria(cv::TermCriteria::EPS |
                                        cv::TermCriteria::COUNT,
                                    30, 0.01));
  cv::drawChessboardCorners(*display, user_config::BOARD_SIZE,
                            observation.corners, true);
  const cv::Point2f pixel =
      std::accumulate(observation.corners.begin(), observation.corners.end(),
                      cv::Point2f()) /
      static_cast<float>(observation.corners.size());
  const double depthM = localDepth(depth, static_cast<int>(std::lround(pixel.x)),
                                   static_cast<int>(std::lround(pixel.y)));
  if (!std::isfinite(depthM) ||
      depthM < vision::kMinimumAllowedBoardDistanceM) {
    observation.detected = false;
    return observation;
  }
  const float uv[2]{pixel.x, pixel.y};
  float xyz[3]{};
  rs2_deproject_pixel_to_point(xyz, &intrinsics, uv, static_cast<float>(depthM));
  observation.center = Eigen::Vector3d(xyz[0], xyz[1], xyz[2]);
  observation.detected = observation.center.allFinite();
  return observation;
}

void jacobianMetrics(rokae::xMateModel<6>& model,
                     const std::array<double, 6>& joints,
                     double* sigma, double* condition) {
  const auto values = model.jacobian(joints);
  Eigen::Matrix<double, 6, 6, Eigen::RowMajor> jacobian;
  std::copy(values.begin(), values.end(), jacobian.data());
  const auto singular = jacobian.jacobiSvd().singularValues();
  *sigma = singular.minCoeff();
  *condition = singular.maxCoeff() /
      std::max(*sigma, std::numeric_limits<double>::epsilon());
}

bool inWorkspace(const Eigen::Vector3d& point) {
  return std::hypot(point.x(), point.y()) <= demo::kWorkspaceRadiusMaxM &&
         point.z() >= demo::kWorkspaceZMinM &&
         point.z() <= demo::kWorkspaceZMaxM;
}

bool selectNearbyInitialJoints(const std::array<double, 6>& current,
                               const std::array<double[2], 6>& limits,
                               std::array<double, 6>* selected) {
  for (std::size_t joint = 0; joint < selected->size(); ++joint) {
    double nearest = 0.0;
    double shortest = std::numeric_limits<double>::infinity();
    for (int turns = -2; turns <= 2; ++turns) {
      const double candidate = demo::kRightInitialJoints[joint] +
                               turns * 2.0 * M_PI;
      if (candidate <= limits[joint][0] + demo::kJointLimitMarginRad ||
          candidate >= limits[joint][1] - demo::kJointLimitMarginRad) {
        continue;
      }
      const double travel = std::abs(candidate - current[joint]);
      if (travel < shortest) {
        shortest = travel;
        nearest = candidate;
      }
    }
    if (!std::isfinite(shortest)) {
      std::cerr << "[ERROR] no equivalent initial angle within soft limit "
                << "for J" << (joint + 1) << '\n';
      return false;
    }
    (*selected)[joint] = nearest;
  }
  return true;
}

bool validateJointPath(rokae::xMateModel<6>& model,
                       const std::array<double, 6>& current,
                       const std::array<double, 6>& target,
                       const std::array<double[2], 6>& limits,
                       std::string* reason) {
  constexpr int kSamples = 50;
  double previousSigma = 0.0;
  double previousCondition = std::numeric_limits<double>::infinity();
  bool escaping = false;
  for (int sample = 0; sample <= kSamples; ++sample) {
    const double ratio = static_cast<double>(sample) / kSamples;
    std::array<double, 6> joints{};
    for (std::size_t i = 0; i < joints.size(); ++i) {
      joints[i] = current[i] + ratio * (target[i] - current[i]);
      if (joints[i] <= limits[i][0] + demo::kJointLimitMarginRad ||
          joints[i] >= limits[i][1] - demo::kJointLimitMarginRad) {
        *reason = "initial path approaches controller limit on J" +
                  std::to_string(i + 1);
        return false;
      }
    }
    double sigma = 0.0;
    double condition = 0.0;
    jacobianMetrics(model, joints, &sigma, &condition);
    const bool singular = sigma < demo::kMinimumJacobianSigma ||
                          condition > demo::kMaximumJacobianCondition;
    if (sample == 0) {
      escaping = singular;
    } else if (singular) {
      if (!escaping || sigma + 1e-6 < previousSigma ||
          condition > previousCondition * 1.02 || sample > kSamples / 2) {
        *reason = "initial joint path approaches singularity at sample " +
                  std::to_string(sample);
        return false;
      }
    } else {
      escaping = false;
    }
    previousSigma = sigma;
    previousCondition = condition;
    const auto pose = model.getCartPose(joints);
    if (!inWorkspace(Eigen::Vector3d(pose[3], pose[7], pose[11]))) {
      *reason = "initial joint path leaves workspace";
      return false;
    }
  }
  return true;
}

bool validateLinearPath(rokae::xMateModel<6>& model,
                        const Isometry& start,
                        const Isometry& target,
                        const std::array<double, 6>& current,
                        const std::array<double[2], 6>& limits,
                        std::string* reason,
                        std::array<double, 6>* endpointJoints = nullptr) {
  const double distance = (target.translation() - start.translation()).norm();
  const int samples = std::max(2, static_cast<int>(
      std::ceil(distance / demo::kPathSampleStepM)));
  std::array<double, 6> seed{};
  for (std::size_t i = 0; i < seed.size(); ++i) {
    seed[i] = std::remainder(current[i], 2.0 * M_PI);
  }
  std::array<double, 6> previous = current;
  for (int sample = 1; sample <= samples; ++sample) {
    const double ratio = static_cast<double>(sample) / samples;
    Isometry pose = start;
    pose.translation() =
        start.translation() + ratio * (target.translation() - start.translation());
    if (!inWorkspace(pose.translation())) {
      *reason = "linear path leaves configured workspace";
      return false;
    }
    std::array<double, 6> predicted{};
    const int ik = model.getJointPos(isometryArray(pose), 0.0, seed, predicted);
    if (ik != 0) {
      *reason = "linear path IK rejected sample " + std::to_string(sample) +
                " code=" + std::to_string(ik);
      return false;
    }
    for (std::size_t i = 0; i < predicted.size(); ++i) {
      predicted[i] = previous[i] +
          std::remainder(predicted[i] - previous[i], 2.0 * M_PI);
      if (predicted[i] <= limits[i][0] + demo::kJointLimitMarginRad ||
          predicted[i] >= limits[i][1] - demo::kJointLimitMarginRad) {
        *reason = "linear path approaches controller limit on J" +
                  std::to_string(i + 1);
        return false;
      }
      if (std::abs(predicted[i] - previous[i]) >
          demo::kMaximumSampleJointStepRad) {
        *reason = "linear path changes J" + std::to_string(i + 1) +
                  " too quickly";
        return false;
      }
      seed[i] = std::remainder(predicted[i], 2.0 * M_PI);
    }
    double sigma = 0.0;
    double condition = 0.0;
    jacobianMetrics(model, predicted, &sigma, &condition);
    if (sigma < demo::kMinimumJacobianSigma ||
        condition > demo::kMaximumJacobianCondition) {
      *reason = "linear path approaches singularity at sample " +
                std::to_string(sample);
      return false;
    }
    previous = predicted;
  }
  if (endpointJoints) *endpointJoints = previous;
  return true;
}

bool waitForWaypoint(rokae::xMateRobot& robot, const std::string& pathId,
                     int timeoutSeconds) {
  using namespace rokae::EventInfoKey::MoveExecution;
  const auto deadline = std::chrono::steady_clock::now() +
                        std::chrono::seconds(timeoutSeconds);
  while (!gStop && std::chrono::steady_clock::now() < deadline) {
    std::error_code ec;
    const auto info = robot.queryEventInfo(rokae::Event::moveExecution, ec);
    if (!checkEc("query move execution", ec)) return false;
    if (info.count(ID) && info.count(WaypointIndex) &&
        std::any_cast<std::string>(info.at(ID)) == pathId &&
        std::any_cast<int>(info.at(WaypointIndex)) == 0) {
      if (info.count(Error)) {
        const auto moveError = std::any_cast<std::error_code>(info.at(Error));
        if (moveError) {
          std::cerr << "[ERROR] path " << pathId << ": "
                    << moveError.message() << " (" << moveError.value() << ")\n";
          return false;
        }
      }
      if (info.count(ReachTarget) &&
          std::any_cast<bool>(info.at(ReachTarget))) return true;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }
  std::cerr << "[ERROR] path " << pathId << " timed out or interrupted\n";
  return false;
}

bool moveAbsJ(rokae::xMateRobot& robot,
              const std::array<double, 6>& target,
              double speed) {
  std::error_code ec;
  std::string pathId;
  robot.moveAppend(rokae::MoveAbsJCommand(
                       std::vector<double>(target.begin(), target.end()),
                       speed, 0.0), pathId, ec);
  if (!checkEc("append MoveAbsJ", ec)) return false;
  robot.moveStart(ec);
  if (!checkEc("start MoveAbsJ", ec) || !waitForWaypoint(robot, pathId, 90)) {
    return false;
  }
  const auto actual = robot.jointPos(ec);
  if (!checkEc("read joints after MoveAbsJ", ec)) return false;
  for (std::size_t i = 0; i < target.size(); ++i) {
    if (std::abs(actual[i] - target[i]) > demo::kReachedJointToleranceRad) {
      std::cerr << "[ERROR] MoveAbsJ did not reach J" << (i + 1)
                << ": error=" << actual[i] - target[i] << " rad\n";
      return false;
    }
  }
  return true;
}

bool returnToInitial(rokae::xMateRobot& robot,
                     rokae::xMateModel<6>& model,
                     const std::array<double[2], 6>& limits,
                     const std::array<double, 6>& initialJoints) {
  std::error_code ec;
  const auto current = robot.jointPos(ec);
  if (!checkEc("read joints before return", ec)) return false;
  std::string reason;
  if (!validateJointPath(model, current, initialJoints,
                         limits, &reason)) {
    std::cerr << "[HOLD] return path rejected: " << reason << '\n';
    return false;
  }
  std::cout << "[SAFE] return path passed 50-sample check\n";
  return moveAbsJ(robot, initialJoints,
                  demo::kJointMoveSpeedMmPerS);
}

bool moveLinear(rokae::xMateRobot& robot, const Isometry& target, double speed,
                const std::string& label) {
  std::error_code ec;
  std::string pathId;
  robot.moveAppend(rokae::MoveLCommand(cartesianPosition(target), speed, 0.0),
                   pathId, ec);
  if (!checkEc("append " + label, ec)) return false;
  robot.moveStart(ec);
  if (!checkEc("start " + label, ec) || !waitForWaypoint(robot, pathId, 60)) {
    return false;
  }
  const auto frame = robot.cartPosture(rokae::CoordinateType::flangeInBase, ec);
  if (!checkEc("read flange after " + label, ec)) return false;
  const Isometry actual = frameToIsometry(frame);
  const double positionError = (actual.translation() - target.translation()).norm();
  const double rotationError = Eigen::AngleAxisd(
      actual.linear().transpose() * target.linear()).angle();
  if (positionError > demo::kReachedPositionToleranceM ||
      rotationError > demo::kReachedOrientationToleranceRad) {
    std::cerr << "[ERROR] " << label << " did not reach target: position="
              << positionError << " m orientation=" << rotationError << " rad\n";
    return false;
  }
  return true;
}

void requestGripperClose() {
  // Replace this body only when the final controller IO assignment is known.
  std::cout << "[GRIPPER] CLOSE requested (IO placeholder; no output written)\n";
}

Isometry flangeTargetForBoard(const Isometry& currentFlange,
                              const Isometry& flangeToCamera,
                              const Eigen::Vector3d& boardInCamera,
                              double cameraDistance) {
  const Isometry baseToCamera = currentFlange * flangeToCamera;
  Isometry desiredCamera = baseToCamera;
  desiredCamera.translation() += baseToCamera.linear() *
      (boardInCamera - Eigen::Vector3d(0.0, 0.0, cameraDistance));
  return desiredCamera * flangeToCamera.inverse();
}

int runSelfTest(const Options& options) {
  Isometry flangeToCamera;
  if (!loadCalibration(options.calibrationFile, options.cameraSerial,
                       &flangeToCamera)) return 2;
  const Isometry initial = Isometry::Identity();
  const Eigen::Vector3d boardInCamera(0.05, -0.03, 0.60);
  const Eigen::Vector3d boardInBase =
      (initial * flangeToCamera) * boardInCamera;
  auto checkTarget = [&](double distance, const std::string& label) {
    const Isometry target = flangeTargetForBoard(
        initial, flangeToCamera, boardInCamera, distance);
    const Eigen::Vector3d observed =
        (target * flangeToCamera).inverse() * boardInBase;
    const bool passed =
        (observed - Eigen::Vector3d(0.0, 0.0, distance)).norm() < 1e-9;
    std::cout << (passed ? "[PASS] " : "[FAIL] ") << label
              << " board_C=[" << observed.transpose() << "] m\n";
    return passed;
  };
  const bool coarse = checkTarget(demo::kCoarseCameraToBoardM, "coarse pose");
  const bool final = checkTarget(demo::kFinalCameraToBoardM, "final pose");
  const bool standOff =
      std::abs(demo::kFinalCameraToBoardM -
               demo::kCameraBehindGraspPointM -
               demo::kFinalGraspPointToBoardM) < 1e-12;
  std::cout << (standOff ? "[PASS] " : "[FAIL] ")
            << "grasp-point stand-off geometry\n";
  Isometry lifted = initial;
  lifted.translation().z() += demo::kLiftInBaseZM;
  const bool lift =
      std::abs(lifted.translation().z() - 0.20) < 1e-12 &&
      lifted.translation().x() == 0.0 &&
      lifted.translation().y() == 0.0;
  std::cout << (lift ? "[PASS] " : "[FAIL] ") << "base-frame lift\n";
  std::array<double[2], 6> limits{};
  for (auto& range : limits) {
    range[0] = -2.0 * M_PI;
    range[1] = 2.0 * M_PI;
  }
  auto equivalentCurrent = demo::kRightInitialJoints;
  equivalentCurrent[5] += 2.0 * M_PI;
  std::array<double, 6> selected{};
  const bool nearest = selectNearbyInitialJoints(
      equivalentCurrent, limits, &selected) &&
      std::abs(selected[5] - equivalentCurrent[5]) < 1e-12;
  std::cout << (nearest ? "[PASS] " : "[FAIL] ")
            << "initial pose avoids an unnecessary wrist revolution\n";
  return coarse && final && standOff && lift && nearest ? 0 : 2;
}

bool collectStableBoard(rs2::pipeline& pipe, rs2::align& alignToColor,
                        const rs2_intrinsics& intrinsics, bool gui,
                        Eigen::Vector3d* result) {
  std::vector<Eigen::Vector3d> samples;
  constexpr int kMaximumAttempts = 180;
  for (int attempt = 0; attempt < kMaximumAttempts && !gStop; ++attempt) {
    const auto frames = alignToColor.process(pipe.wait_for_frames(5000));
    const auto color = frames.get_color_frame();
    const auto depth = frames.get_depth_frame();
    if (!color || !depth) continue;
    cv::Mat display;
    const auto observation = observeBoard(color, depth, intrinsics, &display);
    if (observation.detected) {
      if (!samples.empty() &&
          (observation.center - samples.back()).norm() >
              demo::kMaximumInterframeBoardMoveM) {
        samples.clear();
      }
      samples.push_back(observation.center);
      cv::putText(display, "STABILIZING " + std::to_string(samples.size()) + "/" +
                               std::to_string(demo::kStableDetectionFrames),
                  cv::Point(15, 30), cv::FONT_HERSHEY_SIMPLEX, 0.7,
                  cv::Scalar(0, 255, 255), 2);
    } else {
      samples.clear();
      cv::putText(display, "BOARD NOT FOUND", cv::Point(15, 30),
                  cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 0, 255), 2);
    }
    if (gui) {
      cv::imshow("board_pick_demo", display);
      if ((cv::waitKey(1) & 0xff) == 'q') {
        gStop = 1;
        return false;
      }
    }
    if (samples.size() >= static_cast<std::size_t>(demo::kStableDetectionFrames)) {
      std::array<std::vector<double>, 3> components;
      for (const auto& sample : samples) {
        for (int axis = 0; axis < 3; ++axis) components[axis].push_back(sample[axis]);
      }
      const Eigen::Vector3d estimate(median(components[0]),
                                     median(components[1]),
                                     median(components[2]));
      const bool stable = std::all_of(samples.begin(), samples.end(),
          [&](const Eigen::Vector3d& sample) {
            return (sample - estimate).norm() <=
                   demo::kMaximumStableBoardSpreadM;
          });
      if (stable) {
        *result = estimate;
        return true;
      }
      samples.erase(samples.begin());
    }
  }
  std::cerr << "[HOLD] stable checkerboard was not acquired\n";
  return false;
}

bool waitForTrigger(rs2::pipeline& pipe, rs2::align& alignToColor,
                    const rs2_intrinsics& intrinsics, bool gui) {
  std::cout << "[WAIT] "
            << (gui ? "press Enter/S in the camera window" : "press Enter")
            << " for one cycle; Q quits\n";
  if (!gui) {
    std::string line;
    if (!std::getline(std::cin, line) || gStop) return false;
    return line != "q" && line != "Q";
  }
  while (!gStop) {
    const auto frames = alignToColor.process(pipe.wait_for_frames(5000));
    const auto color = frames.get_color_frame();
    const auto depth = frames.get_depth_frame();
    if (!color || !depth) continue;
    cv::Mat display;
    const auto observation = observeBoard(color, depth, intrinsics, &display);
    cv::putText(display, observation.detected ? "READY - ENTER/S TO START"
                                               : "PLACE COMPLETE BOARD IN VIEW",
                cv::Point(15, 30), cv::FONT_HERSHEY_SIMPLEX, 0.65,
                observation.detected ? cv::Scalar(0, 255, 0)
                                     : cv::Scalar(0, 0, 255), 2);
    cv::imshow("board_pick_demo", display);
    const int key = cv::waitKey(1) & 0xff;
    if (key == 'q' || key == 27) return false;
    if (key == 's' || key == 10 || key == 13) return true;
  }
  return false;
}

void cleanup(rokae::xMateRobot& robot, bool configured) {
  if (!configured) return;
  std::error_code ec;
  robot.stop(ec);
  checkEc("stop motion", ec);
  robot.setMotionControlMode(rokae::MotionControlMode::Idle, ec);
  robot.setPowerState(false, ec);
  robot.setOperateMode(rokae::OperateMode::manual, ec);
}

int run(const Options& options) {
  if (!localIpv4Exists(options.localIp)) {
    std::cerr << "[ERROR] local robot network address is not active: "
              << options.localIp << '\n';
    return 2;
  }
  Isometry flangeToCamera;
  if (!loadCalibration(options.calibrationFile, options.cameraSerial,
                       &flangeToCamera)) return 2;

  rokae::xMateRobot robot(options.robotIp, options.localIp);
  std::error_code ec;
  const auto info = robot.robotInfo(ec);
  if (!checkEc("robotInfo", ec)) return 2;
  std::cout << "[OK] robot=" << info.type << " controller=" << info.version << '\n';
  auto model = robot.model();
  std::array<double[2], 6> softLimits{};
  robot.getSoftLimit(softLimits, ec);
  if (!checkEc("getSoftLimit", ec)) return 2;

  rs2::pipeline pipe;
  rs2::config cameraConfig;
  cameraConfig.enable_device(options.cameraSerial);
  cameraConfig.enable_stream(RS2_STREAM_COLOR, vision::kColorWidth,
                             vision::kColorHeight, RS2_FORMAT_BGR8,
                             vision::kCameraFps);
  cameraConfig.enable_stream(RS2_STREAM_DEPTH, vision::kColorWidth,
                             vision::kColorHeight, RS2_FORMAT_Z16,
                             vision::kCameraFps);
  const auto profile = pipe.start(cameraConfig);
  const auto colorProfile = profile.get_stream(RS2_STREAM_COLOR)
                                .as<rs2::video_stream_profile>();
  const rs2_intrinsics intrinsics = colorProfile.get_intrinsics();
  rs2::align alignToColor(RS2_STREAM_COLOR);
  for (int i = 0; i < 15; ++i) pipe.wait_for_frames(5000);
  std::cout << "[OK] camera=" << options.cameraSerial << '\n';

  const auto currentJoints = robot.jointPos(ec);
  if (!checkEc("read current joints", ec)) return 2;
  std::array<double, 6> initialJoints{};
  if (!selectNearbyInitialJoints(currentJoints, softLimits, &initialJoints)) {
    return 2;
  }
  std::cout << "[INITIAL] selected joints=[";
  for (std::size_t i = 0; i < initialJoints.size(); ++i) {
    if (i) std::cout << ", ";
    std::cout << std::fixed << std::setprecision(4) << initialJoints[i];
  }
  std::cout << "] rad\n";
  std::string reason;
  if (!validateJointPath(model, currentJoints, initialJoints,
                         softLimits, &reason)) {
    std::cerr << "[HOLD] cannot reach configured initial posture: " << reason << '\n';
    return 2;
  }
  std::cout << "[SAFE] initial joint path passed 50-sample check\n";
  if (!options.execute) {
    std::cout << "[DRY-RUN] no robot motion; add --execute after clearing workspace\n";
    while (waitForTrigger(pipe, alignToColor, intrinsics, options.gui)) {
      Eigen::Vector3d board;
      if (collectStableBoard(pipe, alignToColor, intrinsics,
                             options.gui, &board)) {
        std::cout << "[DETECT] board_C=[" << board.transpose() << "] m\n";
      }
    }
    pipe.stop();
    return 0;
  }

  bool configured = false;
  try {
    robot.setOperateMode(rokae::OperateMode::automatic, ec);
    if (!checkEc("automatic mode", ec)) throw std::runtime_error("automatic mode failed");
    configured = true;
    robot.setPowerState(true, ec);
    if (!checkEc("power on", ec)) throw std::runtime_error("power on failed");
    robot.setMotionControlMode(rokae::MotionControlMode::NrtCommand, ec);
    if (!checkEc("NRT mode", ec)) throw std::runtime_error("NRT mode failed");
    robot.moveReset(ec);
    if (!checkEc("moveReset", ec)) throw std::runtime_error("move reset failed");
    robot.setDefaultZone(0.0, ec);
    if (!checkEc("set zone", ec)) throw std::runtime_error("zone failed");

    if (!moveAbsJ(robot, initialJoints,
                  demo::kJointMoveSpeedMmPerS)) {
      throw std::runtime_error("failed to reach initial posture");
    }
    std::cout << "[STATE] INITIAL - waiting for trigger\n";

    while (!gStop && waitForTrigger(pipe, alignToColor, intrinsics, options.gui)) {
      std::cout << "[STATE] DETECT_COARSE\n";
      Eigen::Vector3d board;
      if (!collectStableBoard(pipe, alignToColor, intrinsics, options.gui, &board)) continue;
      std::cout << "[DETECT] board_C=[" << board.transpose() << "] m\n";

      auto flangeFrame = robot.cartPosture(rokae::CoordinateType::flangeInBase, ec);
      if (!checkEc("read flange", ec)) throw std::runtime_error("flange read failed");
      auto joints = robot.jointPos(ec);
      if (!checkEc("read joints", ec)) throw std::runtime_error("joint read failed");
      Isometry flange = frameToIsometry(flangeFrame);
      const Eigen::Vector3d boardBaseBefore =
          (flange * flangeToCamera) * board;
      Isometry coarse = flangeTargetForBoard(
          flange, flangeToCamera, board, demo::kCoarseCameraToBoardM);
      if (!validateLinearPath(model, flange, coarse, joints, softLimits, &reason)) {
        std::cout << "[HOLD] coarse path rejected: " << reason << '\n';
        continue;
      }
      std::cout << "[STATE] MOVE_COARSE camera-board="
                << demo::kCoarseCameraToBoardM << " m\n";
      if (!moveLinear(robot, coarse, demo::kCoarseMoveSpeedMmPerS,
                      "coarse MoveL")) {
        throw std::runtime_error("coarse move failed");
      }

      std::cout << "[STATE] RELOCALIZE_BEFORE_APPROACH\n";
      Eigen::Vector3d refinedBoard;
      if (!collectStableBoard(pipe, alignToColor, intrinsics, options.gui,
                              &refinedBoard)) {
        if (gStop) break;
        std::cout << "[HOLD] board lost before final approach; returning initial\n";
        if (!returnToInitial(robot, model, softLimits, initialJoints)) {
          throw std::runtime_error("return after board loss failed");
        }
        continue;
      }
      flangeFrame = robot.cartPosture(rokae::CoordinateType::flangeInBase, ec);
      if (!checkEc("read flange", ec)) throw std::runtime_error("flange read failed");
      joints = robot.jointPos(ec);
      if (!checkEc("read joints", ec)) throw std::runtime_error("joint read failed");
      flange = frameToIsometry(flangeFrame);
      const Eigen::Vector3d boardBaseAfter =
          (flange * flangeToCamera) * refinedBoard;
      const double boardShift = (boardBaseAfter - boardBaseBefore).norm();
      if (boardShift > demo::kMaximumCoarseBoardShiftM) {
        std::cout << "[HOLD] board moved during coarse approach: "
                  << boardShift << " m\n";
        if (!returnToInitial(robot, model, softLimits, initialJoints)) {
          throw std::runtime_error("return after board relocation failed");
        }
        continue;
      }
      const Isometry approach = flangeTargetForBoard(
          flange, flangeToCamera, refinedBoard, demo::kFinalCameraToBoardM);
      Isometry plannedLift = approach;
      plannedLift.translation().z() += demo::kLiftInBaseZM;
      std::array<double, 6> approachJoints{};
      std::array<double, 6> liftedJoints{};
      if (!validateLinearPath(model, flange, approach, joints, softLimits,
                              &reason, &approachJoints) ||
          !validateLinearPath(model, approach, plannedLift, approachJoints,
                              softLimits, &reason, &liftedJoints) ||
          !validateJointPath(model, liftedJoints, initialJoints,
                             softLimits, &reason)) {
        std::cout << "[HOLD] approach/lift/return plan rejected: "
                  << reason << '\n';
        if (!returnToInitial(robot, model, softLimits, initialJoints)) {
          throw std::runtime_error("return after rejected approach failed");
        }
        continue;
      }
      std::cout << "[STATE] APPROACH grasp-point-board~"
                << demo::kFinalGraspPointToBoardM << " m, camera-board="
                << demo::kFinalCameraToBoardM << " m\n";
      if (!moveLinear(robot, approach, demo::kApproachMoveSpeedMmPerS,
                      "approach MoveL")) {
        throw std::runtime_error("approach failed");
      }

      flangeFrame = robot.cartPosture(rokae::CoordinateType::flangeInBase, ec);
      if (!checkEc("read flange", ec)) throw std::runtime_error("flange read failed");
      const Isometry actualApproach = frameToIsometry(flangeFrame);
      Isometry lift = actualApproach;
      lift.translation().z() += demo::kLiftInBaseZM;
      joints = robot.jointPos(ec);
      if (!checkEc("read joints", ec)) throw std::runtime_error("joint read failed");
      if (!validateLinearPath(model, actualApproach, lift, joints,
                              softLimits, &reason)) {
        std::cout << "[HOLD] lift rejected: " << reason << '\n';
        if (!returnToInitial(robot, model, softLimits, initialJoints)) {
          throw std::runtime_error("lift and return paths are unsafe");
        }
        continue;
      }
      requestGripperClose();
      std::cout << "[STATE] LIFT base +Z " << demo::kLiftInBaseZM << " m\n";
      if (!moveLinear(robot, lift, demo::kLiftMoveSpeedMmPerS, "lift MoveL")) {
        throw std::runtime_error("lift failed");
      }
      std::cout << "[STATE] HOLD " << demo::kHoldAfterCloseSeconds << " s\n";
      const auto holdUntil = std::chrono::steady_clock::now() +
          std::chrono::seconds(demo::kHoldAfterCloseSeconds);
      while (!gStop && std::chrono::steady_clock::now() < holdUntil) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
      }
      if (gStop) break;
      std::cout << "[STATE] RETURN_INITIAL\n";
      if (!returnToInitial(robot, model, softLimits, initialJoints)) {
        throw std::runtime_error("return initial failed");
      }
      std::cout << "[CYCLE] complete; ready for next trigger\n";
    }
  } catch (...) {
    cleanup(robot, configured);
    pipe.stop();
    throw;
  }
  cleanup(robot, configured);
  pipe.stop();
  if (options.gui) cv::destroyAllWindows();
  std::cout << "[OK] demo stopped safely\n";
  return 0;
}

}  // namespace

int main(int argc, char** argv) {
  std::signal(SIGINT, onSignal);
  std::signal(SIGTERM, onSignal);
  Options options;
  options.calibrationFile = defaultCalibrationFile();
  if (!parseArgs(argc, argv, &options)) return 2;
  if (options.help) {
    usage(argv[0]);
    return 0;
  }
  try {
    if (options.selfTest) return runSelfTest(options);
    return run(options);
  } catch (const rs2::error& error) {
    std::cerr << "[REALSENSE ERROR] " << error.what() << '\n';
  } catch (const std::exception& error) {
    std::cerr << "[EXCEPTION] " << error.what() << '\n';
  }
  return 2;
}
