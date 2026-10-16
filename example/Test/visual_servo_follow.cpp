#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdlib>
#include <deque>
#include <exception>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <memory>
#include <numeric>
#include <optional>
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
#include <opencv2/calib3d.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/opencv.hpp>

#include "robot_vision_config.hpp"
#include "visual_servo_config.hpp"
#include "rokae/model.h"
#include "rokae/planner.h"
#include "rokae/robot.h"
#include "rokae/utility.h"

namespace {

using Clock = std::chrono::steady_clock;
using Isometry = Eigen::Isometry3d;
using rokae::CoordinateType;
using rokae::MotionControlMode;
using rokae::OperateMode;
using rokae::RtSupportedFields::jointPos_m;
using user_config::BOARD_SIZE;
using user_config::IP_LEFT;
using user_config::IP_RIGHT;
using user_config::LOCAL_IP;
using user_config::SN_LEFT_CAM;
using user_config::SN_RIGHT_CAM;
namespace cfg = visual_servo_config;

volatile std::sig_atomic_t g_stop = 0;

void onSignal(int) { g_stop = 1; }

struct Options {
  std::string arm = "right";
  std::string robotIp = IP_RIGHT;
  std::string localIp = LOCAL_IP;
  std::string cameraSerial = SN_RIGHT_CAM;
  std::string calibrationFile;
  double distanceM = cfg::kDesiredBoardDistanceM;
  bool execute = false;
  bool jointFollow = false;
  bool gui = true;
  bool selfTest = false;
  bool listDevices = false;
  bool safetyScan = false;
  std::optional<std::array<double, 6>> candidateJoints;
  bool moveReady = false;
  bool identifyAxes = false;
  bool showToolset = false;
  double axisStepM = 0.012;
  std::string outputCalibrationFile;
  std::string logFile;
};

std::string legacyCalibrationFile(const std::string& arm) {
  const auto exampleDir = std::filesystem::path(__FILE__).parent_path().parent_path();
  const std::string name = arm == "left" ? "LeftArm_interactive_calib.yml"
                                          : "RightArm_interactive_calib.yml";
  return (exampleDir / "function_config" / name).string();
}

std::string defaultCalibrationFile(const std::string& arm) {
  const std::filesystem::path legacy = legacyCalibrationFile(arm);
  const auto verified = legacy.parent_path() /
                        (legacy.stem().string() + "_axis_verified.yml");
  return std::filesystem::exists(verified) ? verified.string() : legacy.string();
}

void printUsage(const char* program) {
  std::cout
      << "Usage: " << program << " [options]\n"
      << "  --left                   use left arm (192.168.2.161)\n"
      << "  --right                  use right arm (192.168.2.160), default\n"
      << "  --ip <address>           override robot IP\n"
      << "  --local <address>        local realtime-network IP\n"
      << "  --camera <serial>        RealSense serial; default follows the selected arm\n"
      << "  --calib <file.yml>       hand-eye result containing H_Flange2Cam\n"
      << "  --distance <meters>      board-to-camera distance, default "
      << cfg::kDesiredBoardDistanceM << "\n"
      << "  --execute                enable robot motion; requires axis-verified calibration\n"
      << "  --joint-follow           use guarded short joint targets instead of Cartesian IK\n"
      << "  --no-gui                 run without an OpenCV window\n"
      << "  --self-test              test filtering and command shaping without hardware\n"
      << "  --list-devices           list connected RealSense cameras and exit\n"
      << "  --safety-scan            model-only 6-direction reach/singularity scan\n"
      << "  --candidate-deg q1 ... q6  scan or dry-run a joint pose in degrees\n"
      << "  --move-ready             prepare servo-ready joints; requires --execute to move\n"
      << "  --identify-axes          identify camera rotation with +/- base-axis moves\n"
      << "  --axis-step <meters>     identification step, default 0.012\n"
      << "  --output-calib <file>    write axis-verified calibration to this file\n"
      << "  --show-toolset           print controller TCP/load configuration and exit\n"
      << "  --log <file.csv>         write per-frame commissioning data\n"
      << "  --help                   show this help\n\n"
      << "Observe-only still connects to the robot and validates every proposed target,\n"
      << "but does not switch mode, power on, or command motion.\n";
}

bool parseArgs(int argc, char** argv, Options* opt) {
  for (int i = 1; i < argc; ++i) {
    const std::string arg = argv[i];
    auto value = [&](const char* name) -> const char* {
      if (i + 1 >= argc) {
        std::cerr << "[ERROR] missing value for " << name << '\n';
        return nullptr;
      }
      return argv[++i];
    };

    if (arg == "--left") {
      opt->arm = "left";
      opt->robotIp = IP_LEFT;
      opt->cameraSerial = SN_LEFT_CAM;
    } else if (arg == "--right") {
      opt->arm = "right";
      opt->robotIp = IP_RIGHT;
      opt->cameraSerial = SN_RIGHT_CAM;
    } else if (arg == "--ip") {
      const char* v = value("--ip");
      if (!v) return false;
      opt->robotIp = v;
    } else if (arg == "--local") {
      const char* v = value("--local");
      if (!v) return false;
      opt->localIp = v;
    } else if (arg == "--camera") {
      const char* v = value("--camera");
      if (!v) return false;
      opt->cameraSerial = v;
    } else if (arg == "--calib") {
      const char* v = value("--calib");
      if (!v) return false;
      opt->calibrationFile = v;
    } else if (arg == "--distance") {
      const char* v = value("--distance");
      if (!v) return false;
      opt->distanceM = std::atof(v);
    } else if (arg == "--execute") {
      opt->execute = true;
    } else if (arg == "--joint-follow") {
      opt->jointFollow = true;
    } else if (arg == "--no-gui") {
      opt->gui = false;
    } else if (arg == "--self-test") {
      opt->selfTest = true;
    } else if (arg == "--list-devices") {
      opt->listDevices = true;
    } else if (arg == "--safety-scan") {
      opt->safetyScan = true;
    } else if (arg == "--candidate-deg") {
      std::array<double, 6> joints{};
      for (double& joint : joints) {
        if (i + 1 >= argc) {
          std::cerr << "[ERROR] --candidate-deg needs six angles\n";
          return false;
        }
        char* end = nullptr;
        const double degrees = std::strtod(argv[++i], &end);
        if (end == argv[i] || *end != '\0' || !std::isfinite(degrees)) {
          std::cerr << "[ERROR] invalid --candidate-deg angle: " << argv[i]
                    << '\n';
          return false;
        }
        joint = degrees * M_PI / 180.0;
      }
      opt->candidateJoints = joints;
    } else if (arg == "--move-ready") {
      opt->moveReady = true;
    } else if (arg == "--identify-axes") {
      opt->identifyAxes = true;
    } else if (arg == "--axis-step") {
      const char* v = value("--axis-step");
      if (!v) return false;
      opt->axisStepM = std::atof(v);
    } else if (arg == "--output-calib") {
      const char* v = value("--output-calib");
      if (!v) return false;
      opt->outputCalibrationFile = v;
    } else if (arg == "--show-toolset") {
      opt->showToolset = true;
    } else if (arg == "--log") {
      const char* v = value("--log");
      if (!v) return false;
      opt->logFile = v;
    } else if (arg == "--help" || arg == "-h") {
      printUsage(argv[0]);
      std::exit(0);
    } else {
      std::cerr << "[ERROR] unknown option: " << arg << '\n';
      return false;
    }
  }

  if (!std::isfinite(opt->distanceM) ||
      opt->distanceM < cfg::kMinimumAllowedBoardDistanceM ||
      opt->distanceM > 1.00) {
    std::cerr << "[ERROR] distance must be in ["
              << cfg::kMinimumAllowedBoardDistanceM
              << ", 1.00] m for the current camera/gripper geometry\n";
    return false;
  }
  if (!std::isfinite(opt->axisStepM) ||
      opt->axisStepM < 0.005 || opt->axisStepM > 0.025) {
    std::cerr << "[ERROR] axis step must be in [0.005, 0.025] m\n";
    return false;
  }
  if (opt->candidateJoints && !opt->safetyScan && !opt->moveReady) {
    std::cerr << "[ERROR] --candidate-deg requires --safety-scan or "
                 "--move-ready\n";
    return false;
  }
  return true;
}

bool checkEc(const std::string& action, const std::error_code& ec);
bool localIpv4Exists(const std::string& expected);

int showControllerToolset(const Options& opt) {
  if (!localIpv4Exists(opt.localIp)) {
    std::cerr << "[ERROR] local robot-network address " << opt.localIp
              << " is not active\n";
    return 2;
  }
  rokae::xMateRobot robot(opt.robotIp, opt.localIp);
  std::error_code ec;
  const auto info = robot.robotInfo(ec);
  if (!checkEc("robotInfo", ec)) return 2;
  const auto toolset = robot.toolset(ec);
  if (!checkEc("read toolset", ec)) return 2;
  std::cout << std::fixed << std::setprecision(6)
            << "[TOOLSET] arm=" << opt.arm << " robot=" << info.type << '\n'
            << "[TOOLSET] flange_T_tcp trans=["
            << toolset.end.trans[0] << ", " << toolset.end.trans[1] << ", "
            << toolset.end.trans[2] << "] m rpy=[" << toolset.end.rpy[0]
            << ", " << toolset.end.rpy[1] << ", " << toolset.end.rpy[2]
            << "] rad\n"
            << "[TOOLSET] load mass=" << toolset.load.mass << " kg cog=["
            << toolset.load.cog[0] << ", " << toolset.load.cog[1] << ", "
            << toolset.load.cog[2] << "] m inertia=["
            << toolset.load.inertia[0] << ", " << toolset.load.inertia[1]
            << ", " << toolset.load.inertia[2] << "]\n";
  return 0;
}

int listRealSenseDevices(const std::string& requiredSerial = {}) {
  rs2::context context;
  const auto devices = context.query_devices();
  std::cout << "[CAMERA] connected RealSense devices=" << devices.size() << '\n';
  bool found = requiredSerial.empty();
  for (const auto& device : devices) {
    const std::string name = device.supports(RS2_CAMERA_INFO_NAME)
                                 ? device.get_info(RS2_CAMERA_INFO_NAME) : "unknown";
    const std::string serial = device.supports(RS2_CAMERA_INFO_SERIAL_NUMBER)
                                   ? device.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER) : "unknown";
    std::cout << "[CAMERA] name=" << name << " serial=" << serial;
    if (device.supports(RS2_CAMERA_INFO_PHYSICAL_PORT)) {
      std::cout << " port=" << device.get_info(RS2_CAMERA_INFO_PHYSICAL_PORT);
    }
    if (serial == requiredSerial) {
      found = true;
      std::cout << " selected";
    }
    std::cout << '\n';
  }
  if (!found) {
    std::cerr << "[ERROR] configured camera serial " << requiredSerial
              << " is not connected\n";
    return 2;
  }
  return devices.size() == 0 ? 2 : 0;
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

Isometry cvMatrixToIsometry(const cv::Mat& matrix) {
  cv::Mat m;
  matrix.convertTo(m, CV_64F);
  Isometry result = Isometry::Identity();
  for (int r = 0; r < 4; ++r) {
    for (int c = 0; c < 4; ++c) result.matrix()(r, c) = m.at<double>(r, c);
  }
  return result;
}

Isometry frameToIsometry(const rokae::Frame& frame) {
  Eigen::Vector3d zyx(frame.rpy[2], frame.rpy[1], frame.rpy[0]);
  Eigen::Matrix3d rotation;
  rokae::Utils::eulerToMatrix(zyx, rotation);
  Isometry result = Isometry::Identity();
  result.linear() = rotation;
  result.translation() = Eigen::Vector3d(frame.trans[0], frame.trans[1], frame.trans[2]);
  return result;
}

std::array<double, 16> isometryToArray(const Isometry& transform) {
  std::array<double, 16> result{};
  for (int r = 0; r < 4; ++r) {
    for (int c = 0; c < 4; ++c) result[r * 4 + c] = transform.matrix()(r, c);
  }
  return result;
}

bool loadCalibration(const std::string& path, Isometry* flangeToCamera,
                     double* boardSquareSizeM = nullptr) {
  cv::FileStorage fs(path, cv::FileStorage::READ);
  if (!fs.isOpened()) {
    std::cerr << "[ERROR] cannot open calibration file: " << path << '\n';
    return false;
  }
  cv::Mat matrix;
  fs["H_Flange2Cam"] >> matrix;
  int boardWidth = 0;
  int boardHeight = 0;
  double squareSize = 0.0;
  double depthSquareEstimate = 0.0;
  if (!fs["board_width"].empty()) fs["board_width"] >> boardWidth;
  if (!fs["board_height"].empty()) fs["board_height"] >> boardHeight;
  if (!fs["square_size_m"].empty()) fs["square_size_m"] >> squareSize;
  if (!fs["square_size_depth_estimate_m"].empty()) {
    fs["square_size_depth_estimate_m"] >> depthSquareEstimate;
  }
  fs.release();

  if (matrix.rows != 4 || matrix.cols != 4) {
    std::cerr << "[ERROR] H_Flange2Cam must be a 4x4 matrix\n";
    return false;
  }
  *flangeToCamera = cvMatrixToIsometry(matrix);
  const double rotationError =
      (flangeToCamera->linear().transpose() * flangeToCamera->linear() -
       Eigen::Matrix3d::Identity()).norm();
  if (!std::isfinite(flangeToCamera->matrix().norm()) || rotationError > 0.02 ||
      std::abs(flangeToCamera->linear().determinant() - 1.0) > 0.02) {
    std::cerr << "[ERROR] calibration rotation is not a valid rotation matrix\n";
    return false;
  }
  if ((boardWidth && boardWidth != BOARD_SIZE.width) ||
      (boardHeight && boardHeight != BOARD_SIZE.height) ||
      (squareSize && (!std::isfinite(squareSize) || squareSize < 0.005 ||
                      squareSize > 0.10))) {
    std::cerr << "[ERROR] calibration checkerboard metadata is invalid\n";
    return false;
  }
  if (boardSquareSizeM) {
    *boardSquareSizeM = squareSize > 0.0 ? squareSize : user_config::SQUARE_SIZE;
  }
  const double configuredSquare = squareSize > 0.0
                                      ? squareSize : user_config::SQUARE_SIZE;
  if (std::isfinite(depthSquareEstimate) && depthSquareEstimate > 0.0 &&
      std::abs(depthSquareEstimate - configuredSquare) >
          0.10 * depthSquareEstimate) {
    std::cout << "[WARN] calibration checker square="
              << configuredSquare * 1000.0 << " mm, depth estimate="
              << depthSquareEstimate * 1000.0
              << " mm; verify hand-eye translation before absolute grasp mapping\n";
  }
  std::cout << "[OK] loaded calibration " << path << '\n';
  return true;
}

bool isAxisVerifiedCalibration(const std::string& path,
                               const std::string& cameraSerial) {
  cv::FileStorage storage(path, cv::FileStorage::READ);
  if (!storage.isOpened()) return false;
  const cv::FileNode serialNode = storage["camera_serial"];
  const cv::FileNode widthNode = storage["board_width"];
  const cv::FileNode heightNode = storage["board_height"];
  const cv::FileNode stepNode = storage["axis_identification_step_m"];
  const cv::FileNode correctionNode = storage["rotation_correction_angle_rad"];
  if (serialNode.empty() || widthNode.empty() || heightNode.empty() ||
      stepNode.empty() || correctionNode.empty()) return false;
  std::string serial;
  int width = 0;
  int height = 0;
  double step = 0.0;
  double correction = 0.0;
  serialNode >> serial;
  widthNode >> width;
  heightNode >> height;
  stepNode >> step;
  correctionNode >> correction;
  return serial == cameraSerial && width == BOARD_SIZE.width &&
         height == BOARD_SIZE.height && step >= 0.005 && step <= 0.025 &&
         std::isfinite(correction);
}

cv::Mat cameraMatrix(const rs2::video_stream_profile& profile, cv::Mat* distortion) {
  const rs2_intrinsics in = profile.get_intrinsics();
  cv::Mat k = cv::Mat::eye(3, 3, CV_64F);
  k.at<double>(0, 0) = in.fx;
  k.at<double>(1, 1) = in.fy;
  k.at<double>(0, 2) = in.ppx;
  k.at<double>(1, 2) = in.ppy;
  *distortion = cv::Mat::zeros(5, 1, CV_64F);
  for (int i = 0; i < 5; ++i) distortion->at<double>(i) = in.coeffs[i];
  return k;
}

double median(std::vector<double> values) {
  if (values.empty()) return std::numeric_limits<double>::quiet_NaN();
  const std::size_t mid = values.size() / 2;
  std::nth_element(values.begin(), values.begin() + mid, values.end());
  if (values.size() % 2) return values[mid];
  const double upper = values[mid];
  std::nth_element(values.begin(), values.begin() + mid - 1, values.end());
  return 0.5 * (upper + values[mid - 1]);
}

double qualityCheckedDepth(std::vector<double> samples) {
  if (samples.size() < cfg::kMinDepthPatchPixels) {
    return std::numeric_limits<double>::quiet_NaN();
  }
  std::sort(samples.begin(), samples.end());
  const double spread = samples[(samples.size() - 1) * 9 / 10] -
                        samples[(samples.size() - 1) / 10];
  if (spread > cfg::kMaxDepthPatchSpreadM) {
    return std::numeric_limits<double>::quiet_NaN();
  }
  return median(std::move(samples));
}

double neighborhoodDepth(const rs2::depth_frame& depth, int px, int py) {
  std::vector<double> samples;
  for (int y = py - 3; y <= py + 3; ++y) {
    for (int x = px - 3; x <= px + 3; ++x) {
      if (x < 0 || y < 0 || x >= depth.get_width() || y >= depth.get_height()) continue;
      const double d = depth.get_distance(x, y);
      if (d > 0.08 && d < 3.0 && std::isfinite(d)) samples.push_back(d);
    }
  }
  return qualityCheckedDepth(std::move(samples));
}

double robustBoardCenterDepth(const rs2::depth_frame& depth,
                              const cv::Point2f& pixelCenter,
                              const std::vector<cv::Point2f>& corners) {
  const double centerDepth = neighborhoodDepth(
      depth, static_cast<int>(std::lround(pixelCenter.x)),
      static_cast<int>(std::lround(pixelCenter.y)));
  if (std::isfinite(centerDepth)) return centerDepth;

  std::vector<double> cornerDepths;
  cornerDepths.reserve(corners.size());
  for (const auto& corner : corners) {
    const double value = neighborhoodDepth(
        depth, static_cast<int>(std::lround(corner.x)),
        static_cast<int>(std::lround(corner.y)));
    if (std::isfinite(value)) cornerDepths.push_back(value);
  }
  return cornerDepths.size() >= cfg::kMinFallbackCornerDepths
             ? median(std::move(cornerDepths))
             : std::numeric_limits<double>::quiet_NaN();
}

double estimateSquareSizeFromDepth(const std::vector<cv::Point2f>& corners,
                                   const rs2::depth_frame& depth,
                                   const rs2_intrinsics& intrinsics) {
  if (corners.size() != static_cast<std::size_t>(BOARD_SIZE.area())) {
    return std::numeric_limits<double>::quiet_NaN();
  }
  std::vector<Eigen::Vector3d> points(corners.size());
  std::vector<bool> valid(corners.size(), false);
  for (std::size_t i = 0; i < corners.size(); ++i) {
    const auto& pixel = corners[i];
    const double distance = neighborhoodDepth(
        depth, static_cast<int>(std::lround(pixel.x)),
        static_cast<int>(std::lround(pixel.y)));
    if (!std::isfinite(distance)) continue;
    const float pixelData[2]{pixel.x, pixel.y};
    float pointData[3]{};
    rs2_deproject_pixel_to_point(pointData, &intrinsics, pixelData,
                                 static_cast<float>(distance));
    points[i] = Eigen::Vector3d(pointData[0], pointData[1], pointData[2]);
    valid[i] = points[i].allFinite();
  }

  std::vector<double> spacings;
  for (int row = 0; row < BOARD_SIZE.height; ++row) {
    for (int col = 0; col < BOARD_SIZE.width; ++col) {
      const int index = row * BOARD_SIZE.width + col;
      if (!valid[index]) continue;
      if (col + 1 < BOARD_SIZE.width && valid[index + 1]) {
        spacings.push_back((points[index + 1] - points[index]).norm());
      }
      if (row + 1 < BOARD_SIZE.height && valid[index + BOARD_SIZE.width]) {
        spacings.push_back(
            (points[index + BOARD_SIZE.width] - points[index]).norm());
      }
    }
  }
  return median(std::move(spacings));
}

bool collectBoardCenter(rs2::pipeline& pipe,
                        rs2::align& alignToColor,
                        const rs2_intrinsics& intrinsics,
                        Eigen::Vector3d* center,
                        double* estimatedSquare) {
  std::vector<Eigen::Vector3d> samples;
  std::vector<double> squareSamples;
  constexpr int kRequiredSamples = 10;
  constexpr int kMaxAttempts = 240;
  for (int attempt = 0;
       attempt < kMaxAttempts && samples.size() < kRequiredSamples;
       ++attempt) {
    const auto frames = alignToColor.process(pipe.wait_for_frames(5000));
    const auto color = frames.get_color_frame();
    const auto depth = frames.get_depth_frame();
    if (!color || !depth) continue;
    cv::Mat image(cv::Size(cfg::kColorWidth, cfg::kColorHeight), CV_8UC3,
                  const_cast<void*>(color.get_data()), cv::Mat::AUTO_STEP);
    cv::Mat gray;
    cv::cvtColor(image, gray, cv::COLOR_BGR2GRAY);
    std::vector<cv::Point2f> corners;
    if (!cv::findChessboardCorners(
            gray, BOARD_SIZE, corners,
            cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_NORMALIZE_IMAGE)) {
      continue;
    }
    cv::cornerSubPix(gray, corners, cv::Size(11, 11), cv::Size(-1, -1),
                     cv::TermCriteria(cv::TermCriteria::EPS |
                                          cv::TermCriteria::COUNT,
                                      30, 0.01));
    const cv::Point2f pixelCenter =
        std::accumulate(corners.begin(), corners.end(), cv::Point2f()) *
        (1.0f / static_cast<float>(corners.size()));
    const double depthM = robustBoardCenterDepth(depth, pixelCenter, corners);
    if (!std::isfinite(depthM)) continue;
    const float pixelData[2]{pixelCenter.x, pixelCenter.y};
    float pointData[3]{};
    rs2_deproject_pixel_to_point(pointData, &intrinsics, pixelData,
                                 static_cast<float>(depthM));
    const Eigen::Vector3d point(pointData[0], pointData[1], pointData[2]);
    if (!point.allFinite()) continue;
    samples.push_back(point);
    const double square =
        estimateSquareSizeFromDepth(corners, depth, intrinsics);
    if (std::isfinite(square)) squareSamples.push_back(square);
  }
  if (samples.size() < kRequiredSamples) return false;
  for (int axis = 0; axis < 3; ++axis) {
    std::vector<double> values;
    values.reserve(samples.size());
    for (const auto& sample : samples) values.push_back(sample[axis]);
    (*center)[axis] = median(std::move(values));
  }
  *estimatedSquare = median(std::move(squareSamples));
  return center->allFinite();
}

class PositionFilter {
 public:
  bool update(const Eigen::Vector3d& raw, Clock::time_point now, std::string* reason) {
    if (!raw.allFinite() || raw.z() <= 0.0 || raw.z() > 3.0) {
      *reason = "invalid board position";
      return false;
    }
    if (raw.z() < cfg::kMinimumAllowedBoardDistanceM) {
      *reason = "board too close for estimated tool clearance";
      return false;
    }
    if (initialized_) {
      const double dt = std::chrono::duration<double>(now - lastTime_).count();
      const double jump = (raw - lastRaw_).norm();
      if (jump > cfg::kMaxRawJumpM ||
          (dt > 0.005 && dt < 0.5 && jump / dt > cfg::kMaxBoardSpeedMps)) {
        if ((raw - relockCandidate_).norm() <= cfg::kRelockRadiusM) {
          ++relockCount_;
          relockCandidate_ = 0.8 * relockCandidate_ + 0.2 * raw;
        } else {
          relockCandidate_ = raw;
          relockCount_ = 1;
        }
        if (relockCount_ < cfg::kRelockFrames) {
          *reason = "board measurement jump/speed rejected";
          return false;
        }

        history_.clear();
        filtered_ = relockCandidate_;
        lastRaw_ = raw;
        lastTime_ = now;
        relockCount_ = 0;
        *reason = "filter relocked at stable board position";
        return true;
      }
    }

    relockCount_ = 0;
    lastRaw_ = raw;
    lastTime_ = now;
    history_.push_back(raw);
    if (history_.size() > cfg::kMedianWindow) history_.pop_front();

    Eigen::Vector3d med;
    for (int axis = 0; axis < 3; ++axis) {
      std::vector<double> values;
      values.reserve(history_.size());
      for (const auto& p : history_) values.push_back(p[axis]);
      med[axis] = median(std::move(values));
    }
    if (!initialized_) {
      filtered_ = med;
      initialized_ = true;
    } else {
      filtered_ = (1.0 - cfg::kPositionEmaAlpha) * filtered_ +
                  cfg::kPositionEmaAlpha * med;
    }
    return true;
  }

  void reset() {
    initialized_ = false;
    history_.clear();
    relockCount_ = 0;
  }

  const Eigen::Vector3d& value() const { return filtered_; }

 private:
  bool initialized_ = false;
  Eigen::Vector3d filtered_ = Eigen::Vector3d::Zero();
  Eigen::Vector3d lastRaw_ = Eigen::Vector3d::Zero();
  Clock::time_point lastTime_{};
  std::deque<Eigen::Vector3d> history_;
  Eigen::Vector3d relockCandidate_ = Eigen::Vector3d::Zero();
  int relockCount_ = 0;
};

struct SafetyResult {
  bool safe = false;
  std::string reason;
  std::array<double, 6> predictedJoints{};
  double minSingular = 0.0;
  double condition = std::numeric_limits<double>::infinity();
};

void jacobianMetrics(rokae::xMateModel<6>& model,
                     const std::array<double, 6>& joints,
                     double* minSingular,
                     double* condition) {
  const auto data = model.jacobian(joints);
  Eigen::Matrix<double, 6, 6, Eigen::RowMajor> jacobian;
  std::copy(data.begin(), data.end(), jacobian.data());
  const auto singular = jacobian.jacobiSvd().singularValues();
  *minSingular = singular.minCoeff();
  *condition = singular.maxCoeff() /
               std::max(*minSingular, std::numeric_limits<double>::epsilon());
}

bool inConfiguredWorkspace(const Eigen::Vector3d& point) {
  return std::hypot(point.x(), point.y()) <= cfg::kWorkspaceRadiusMaxM &&
         point.z() >= cfg::kWorkspaceZMinM &&
         point.z() <= cfg::kWorkspaceZMaxM;
}

std::string servoStartPreflightFailure(
    rokae::xMateModel<6>& model,
    const std::array<double, 6>& joints,
    const std::array<double[2], 6>& limits,
    bool softLimitsEnabled) {
  const auto pose = model.getCartPose(joints);
  if (!inConfiguredWorkspace(Eigen::Vector3d(pose[3], pose[7], pose[11]))) {
    return "current flange is outside the configured workspace";
  }
  if (!softLimitsEnabled) return "controller soft limits are disabled";
  for (std::size_t i = 0; i < joints.size(); ++i) {
    if (joints[i] <= limits[i][0] + 0.02 ||
        joints[i] >= limits[i][1] - 0.02) {
      return "J" + std::to_string(i + 1) +
             " is too close to a controller soft limit";
    }
  }

  double sigma = 0.0;
  double condition = 0.0;
  jacobianMetrics(model, joints, &sigma, &condition);
  if (sigma < cfg::kMinJacobianSingularValue ||
      condition > cfg::kMaxJacobianCondition) {
    std::ostringstream reason;
    reason << "current pose is too close to singular (sigma=" << sigma
           << ", cond=" << condition << ')';
    return reason.str();
  }
  return {};
}

template <typename Check>
bool sampleTranslationPath(const Eigen::Vector3d& start,
                           const Eigen::Vector3d& end,
                           Check&& check) {
  const int samples = std::max(
      1, static_cast<int>(std::ceil(
             (end - start).norm() / cfg::kSafetyPathSampleStepM)));
  for (int sample = 1; sample <= samples; ++sample) {
    const double ratio = static_cast<double>(sample) / samples;
    if (!check(start + ratio * (end - start), sample, samples)) return false;
  }
  return true;
}

SafetyResult validateTarget(rokae::xMateModel<6>& model,
                            const Isometry& target,
                            const std::array<double, 6>& currentJoints) {
  SafetyResult result;
  const auto currentPose = model.getCartPose(currentJoints);
  const Eigen::Vector3d start(currentPose[3], currentPose[7], currentPose[11]);
  if (!inConfiguredWorkspace(start) ||
      !inConfiguredWorkspace(target.translation())) {
    result.reason = "outside configured workspace envelope";
    return result;
  }

  double currentMin = 0.0;
  double currentCondition = 0.0;
  jacobianMetrics(model, currentJoints, &currentMin, &currentCondition);
  if (currentMin < cfg::kMinJacobianSingularValue ||
      currentCondition > cfg::kMaxJacobianCondition) {
    result.reason = "Jacobian is too close to singular at current pose: sigma=" +
                    std::to_string(currentMin) + " cond=" +
                    std::to_string(currentCondition);
    return result;
  }
  result.minSingular = currentMin;
  result.condition = currentCondition;

  std::array<double, 6> modelSeed{};
  for (std::size_t i = 0; i < currentJoints.size(); ++i) {
    modelSeed[i] = std::remainder(currentJoints[i], 2.0 * M_PI);
  }
  std::array<double, 6> previousJoints = currentJoints;
  const bool pathSafe = sampleTranslationPath(
      start, target.translation(),
      [&](const Eigen::Vector3d& point, int sample, int samples) {
        if (!inConfiguredWorkspace(point)) {
          result.reason = "path leaves configured workspace envelope";
          return false;
        }
        Isometry waypoint = target;
        waypoint.translation() = point;
        std::array<double, 6> predicted{};
        const int ik = model.getJointPos(isometryToArray(waypoint), 0.0,
                                         modelSeed, predicted);
        if (ik != 0) {
          if (ik == -8) result.reason = "IK predicts singularity (-8)";
          else if (ik == -6 || ik == -7) {
            result.reason = "SDK model IK reports a joint-limit violation";
          } else if (ik >= -3) {
            result.reason = "target is outside robot workspace";
          } else {
            result.reason = "IK rejected target (code " + std::to_string(ik) + ")";
          }
          result.reason += " at path sample " + std::to_string(sample) +
                           "/" + std::to_string(samples);
          return false;
        }

        // xMateModel solves in principal angles; retain the controller's
        // nearest equivalent branch while checking the whole short path.
        for (std::size_t i = 0; i < predicted.size(); ++i) {
          predicted[i] = previousJoints[i] +
              std::remainder(predicted[i] - previousJoints[i], 2.0 * M_PI);
          if (std::abs(predicted[i] - currentJoints[i]) >
                  cfg::kMaxPredictedJointStepRad ||
              std::abs(predicted[i] - previousJoints[i]) >
                  cfg::kMaxPredictedJointStepRad) {
            result.reason = "predicted joint step too large on J" +
                            std::to_string(i + 1) + " at path sample " +
                            std::to_string(sample) + "/" +
                            std::to_string(samples);
            return false;
          }
          modelSeed[i] = std::remainder(predicted[i], 2.0 * M_PI);
        }

        double sigma = 0.0;
        double condition = 0.0;
        jacobianMetrics(model, predicted, &sigma, &condition);
        result.minSingular = std::min(result.minSingular, sigma);
        result.condition = std::max(result.condition, condition);
        if (sigma < cfg::kMinJacobianSingularValue ||
            condition > cfg::kMaxJacobianCondition) {
          result.reason = "Jacobian is too close to singular at path sample " +
                          std::to_string(sample) + "/" +
                          std::to_string(samples) + ": sigma=" +
                          std::to_string(sigma) + " cond=" +
                          std::to_string(condition);
          return false;
        }
        previousJoints = predicted;
        result.predictedJoints = predicted;
        return true;
      });
  if (!pathSafe) return result;
  result.safe = true;
  result.reason = "safe";
  return result;
}

Isometry arrayToIsometry(const std::array<double, 16>& values) {
  Isometry result = Isometry::Identity();
  for (int r = 0; r < 4; ++r) {
    for (int c = 0; c < 4; ++c) result.matrix()(r, c) = values[r * 4 + c];
  }
  return result;
}

std::array<double, 6> scanCameraDirections(
    rokae::xMateModel<6>& model,
    const Isometry& flangeToCamera,
    const Isometry& startFlange,
    const std::array<double, 6>& startJoints,
    const std::string& label) {
  struct Direction {
    const char* name;
    Eigen::Vector3d cameraAxis;
  };
  const std::array<Direction, 6> directions{{
      {"camera +X (board right)", Eigen::Vector3d::UnitX()},
      {"camera -X (board left)", -Eigen::Vector3d::UnitX()},
      {"camera +Y (board down)", Eigen::Vector3d::UnitY()},
      {"camera -Y (board up)", -Eigen::Vector3d::UnitY()},
      {"camera +Z (board farther)", Eigen::Vector3d::UnitZ()},
      {"camera -Z (board nearer)", -Eigen::Vector3d::UnitZ()},
  }};

  const Isometry baseToCamera = startFlange * flangeToCamera;
  std::array<double, 6> safeDistances{};
  constexpr double scanLimitM = cfg::kMaxCorrectionFromCurrentM;
  for (std::size_t i = 0; i < directions.size(); ++i) {
    const auto& direction = directions[i];
    std::array<double, 6> simulatedJoints = startJoints;
    std::string stopReason = "scan limit reached";
    for (double distance = cfg::kMaxCommandStepM;
         distance <= scanLimitM + 1e-9;
         distance += cfg::kMaxCommandStepM) {
      Isometry target = startFlange;
      target.translation() +=
          baseToCamera.linear() * direction.cameraAxis * distance;
      const SafetyResult safety = validateTarget(model, target, simulatedJoints);
      if (!safety.safe) {
        stopReason = safety.reason;
        break;
      }
      safeDistances[i] = distance;
      simulatedJoints = safety.predictedJoints;
    }
    std::cout << "[SCAN " << label << "] " << direction.name
              << " safe=" << safeDistances[i] << " m, stop=" << stopReason
              << '\n';
  }
  return safeDistances;
}

SafetyResult validateJointMicroStep(
    rokae::xMateModel<6>& model,
    const std::array<double, 6>& startJoints,
    const std::array<double[2], 6>& softLimits,
    const Isometry& flangeToCamera,
    const Eigen::Vector3d& desiredTranslation) {
  SafetyResult result;
  const auto cameraPose = [&](const std::array<double, 6>& joints) {
    return arrayToIsometry(model.getCartPose(joints)) * flangeToCamera;
  };
  const Isometry startCamera = cameraPose(startJoints);
  Eigen::Matrix<double, 6, 6> jacobian;
  constexpr double epsilon = 1e-5;
  for (int joint = 0; joint < 6; ++joint) {
    auto plus = startJoints;
    auto minus = startJoints;
    plus[joint] += epsilon;
    minus[joint] -= epsilon;
    const Isometry a = cameraPose(plus);
    const Isometry b = cameraPose(minus);
    jacobian.block<3, 1>(0, joint) =
        (a.translation() - b.translation()) / (2.0 * epsilon);
    const Eigen::AngleAxisd rotation(a.linear() * b.linear().transpose());
    jacobian.block<3, 1>(3, joint) =
        rotation.axis() * rotation.angle() / (2.0 * epsilon);
  }

  Eigen::Matrix<double, 6, 1> desired = Eigen::Matrix<double, 6, 1>::Zero();
  desired.head<3>() = desiredTranslation;
  for (const double rotationWeight : {1.0, 0.5, 0.2, 0.1, 0.05, 0.02}) {
    auto weighted = jacobian;
    weighted.bottomRows<3>() *= rotationWeight;
    constexpr double damping = 0.002;
    const Eigen::Matrix<double, 6, 1> deltaJoints =
        (weighted.transpose() * weighted +
         damping * damping * Eigen::Matrix<double, 6, 6>::Identity())
            .ldlt()
            .solve(weighted.transpose() * desired);
    if (!deltaJoints.allFinite() ||
        deltaJoints.cwiseAbs().maxCoeff() > 0.04) {
      result.reason = "joint increment exceeds 0.04 rad";
      continue;
    }

    std::array<double, 6> target{};
    for (int joint = 0; joint < 6; ++joint) {
      target[joint] = startJoints[joint] + deltaJoints[joint];
    }
    bool pathSafe = true;
    for (int sample = 0; sample <= 20; ++sample) {
      const double fraction = static_cast<double>(sample) / 20.0;
      std::array<double, 6> joints{};
      for (int joint = 0; joint < 6; ++joint) {
        joints[joint] = startJoints[joint] +
                        fraction * (target[joint] - startJoints[joint]);
        if (joints[joint] < softLimits[joint][0] + 0.05 ||
            joints[joint] > softLimits[joint][1] - 0.05) {
          result.reason = "joint path approaches controller soft limit";
          pathSafe = false;
        }
      }
      if (!pathSafe) break;
      const Isometry flange = arrayToIsometry(model.getCartPose(joints));
      const Isometry camera = flange * flangeToCamera;
      if (!inConfiguredWorkspace(flange.translation()) ||
          (camera.translation() - startCamera.translation() -
           fraction * desiredTranslation).norm() >
              std::max(0.001, 0.25 * desiredTranslation.norm()) ||
          Eigen::AngleAxisd(startCamera.linear().transpose() * camera.linear())
                  .angle() > 0.006) {
        result.reason = "joint path leaves short Cartesian envelope";
        pathSafe = false;
        break;
      }
      double sigma = 0.0;
      double condition = 0.0;
      jacobianMetrics(model, joints, &sigma, &condition);
      if (sigma < cfg::kMinJacobianSingularValue ||
          condition > cfg::kMaxJacobianCondition) {
        result.reason = "joint path approaches singularity";
        pathSafe = false;
        break;
      }
      if (sample == 0) {
        result.minSingular = sigma;
        result.condition = condition;
      } else {
        result.minSingular = std::min(result.minSingular, sigma);
        result.condition = std::max(result.condition, condition);
      }
    }
    if (!pathSafe) continue;
    const Eigen::Vector3d actual =
        cameraPose(target).translation() - startCamera.translation();
    if (actual.dot(desiredTranslation) <
        0.75 * desiredTranslation.squaredNorm()) {
      result.reason = "predicted translation is too small";
      continue;
    }
    result.safe = true;
    result.reason = "safe";
    result.predictedJoints = target;
    return result;
  }
  return result;
}

void scanJointMicroDirections(
    rokae::xMateModel<6>& model,
    const Isometry& flangeToCamera,
    const Isometry& startFlange,
    const std::array<double, 6>& startJoints,
    const std::array<double[2], 6>& softLimits) {
  const Isometry baseToCamera = startFlange * flangeToCamera;
  const std::array<Eigen::Vector3d, 6> axes{{
      Eigen::Vector3d::UnitX(), -Eigen::Vector3d::UnitX(),
      Eigen::Vector3d::UnitY(), -Eigen::Vector3d::UnitY(),
      Eigen::Vector3d::UnitZ(), -Eigen::Vector3d::UnitZ()}};
  const std::array<const char*, 6> names{{"+X", "-X", "+Y", "-Y", "+Z", "-Z"}};
  for (int i = 0; i < 6; ++i) {
    const SafetyResult result = validateJointMicroStep(
        model, startJoints, softLimits, flangeToCamera,
        baseToCamera.linear() * axes[i] * 0.002);
    std::cout << "[JOINT-PROBE] camera " << names[i] << " 2 mm "
              << (result.safe ? "safe" : "hold: " + result.reason)
              << " sigma_min=" << result.minSingular
              << " cond=" << result.condition << '\n';
  }
}

std::array<double, 6> makeServoReadyJoints(
    const std::array<double, 6>& currentJoints,
    bool* adjusted) {
  std::array<double, 6> readyJoints = currentJoints;
  constexpr double kTwoPi = 2.0 * M_PI;
  constexpr double kReadyLimit = cfg::kServoReadyPrincipalJointLimitRad;
  *adjusted = false;
  for (double& joint : readyJoints) {
    double wrapped = std::remainder(joint, kTwoPi);
    if (std::abs(std::abs(joint) - M_PI) < 0.02) {
      wrapped = std::copysign(M_PI, joint);
    }
    if (std::abs(wrapped) > kReadyLimit) {
      joint += std::copysign(kReadyLimit, wrapped) - wrapped;
      *adjusted = true;
    }
  }
  return readyJoints;
}

void cleanupRobot(rokae::xMateRobot& robot, bool robotConfigured,
                  bool stateReceiving);

int runSafetyScan(const Options& opt) {
  Isometry flangeToCamera;
  if (!loadCalibration(opt.calibrationFile, &flangeToCamera)) return 2;
  if (!isAxisVerifiedCalibration(opt.calibrationFile, opt.cameraSerial)) {
    std::cout << "[WARN] camera axes are unverified; directional scan labels "
                 "may be inaccurate\n";
  }
  if (!localIpv4Exists(opt.localIp)) {
    std::cerr << "[ERROR] local robot-network address " << opt.localIp
              << " is not active\n";
    return 2;
  }

  rokae::xMateRobot robot(opt.robotIp, opt.localIp);
  std::error_code ec;
  const auto info = robot.robotInfo(ec);
  if (!checkEc("robotInfo", ec)) return 2;
  auto model = robot.model();
  const auto flangeFrame = robot.cartPosture(CoordinateType::flangeInBase, ec);
  if (!checkEc("read flange pose", ec)) return 2;
  const auto currentJoints = robot.jointPos(ec);
  if (!checkEc("read joints", ec)) return 2;
  std::array<double[2], 6> softLimits{};
  const bool softLimitEnabled = robot.getSoftLimit(softLimits, ec);
  if (!checkEc("read soft limits", ec)) return 2;

  const Isometry currentFlange = frameToIsometry(flangeFrame);
  const Isometry modelFlange = arrayToIsometry(model.getCartPose(currentJoints));
  const Eigen::Matrix3d rotationDelta =
      currentFlange.linear().transpose() * modelFlange.linear();
  const double rotationDifference = Eigen::AngleAxisd(rotationDelta).angle();
  const double translationDifference =
      (currentFlange.translation() - modelFlange.translation()).norm();
  double currentMin = 0.0;
  double currentCondition = 0.0;
  jacobianMetrics(model, currentJoints, &currentMin, &currentCondition);
  std::cout << std::fixed << std::setprecision(4)
            << "[SAFETY] arm=" << opt.arm << " controller=" << info.version
            << " flange_B=[" << currentFlange.translation().transpose() << "] m"
            << " sigma_min=" << currentMin
            << " cond=" << currentCondition << '\n';
  std::cout << "[POSE-CHECK] controller_vs_model translation_delta="
            << translationDifference << " m rotation_delta="
            << rotationDifference << " rad\n";
  std::cout << "[SAFETY] controller_soft_limit="
            << (softLimitEnabled ? "enabled" : "disabled") << '\n';
  const std::string startupFailure = servoStartPreflightFailure(
      model, currentJoints, softLimits, softLimitEnabled);
  std::cout << "[STARTUP] realtime follow "
            << (startupFailure.empty() ? "permitted by model checks"
                                       : "rejected: " + startupFailure)
            << '\n';
  for (std::size_t i = 0; i < currentJoints.size(); ++i) {
    std::cout << "[JOINT] J" << (i + 1) << " current=" << currentJoints[i]
              << " range=[" << softLimits[i][0] << ", " << softLimits[i][1]
              << "] rad\n";
  }

  const auto currentModelPose = isometryToArray(modelFlange);
  std::array<double, 6> ikResult{};
  const int rawSeedIk = model.getJointPos(currentModelPose, 0.0,
                                          currentJoints, ikResult);
  std::array<double, 6> principalSeed{};
  for (std::size_t i = 0; i < principalSeed.size(); ++i) {
    principalSeed[i] = std::remainder(currentJoints[i], 2.0 * M_PI);
  }
  const int principalSeedIk = model.getJointPos(currentModelPose, 0.0,
                                                principalSeed, ikResult);
  std::cout << "[IK-SELF] current model pose raw_seed=" << rawSeedIk
            << " principal_seed=" << principalSeedIk << '\n';

  scanCameraDirections(model, flangeToCamera, currentFlange, currentJoints,
                       "CURRENT");
  if (softLimitEnabled) {
    scanJointMicroDirections(model, flangeToCamera, currentFlange,
                             currentJoints, softLimits);
  }

  bool adjusted = false;
  const auto readyJoints = makeServoReadyJoints(currentJoints, &adjusted);
  if (adjusted) {
    const Isometry readyFlange = arrayToIsometry(model.getCartPose(readyJoints));
    double readyMin = 0.0;
    double readyCondition = 0.0;
    jacobianMetrics(model, readyJoints, &readyMin, &readyCondition);
    std::cout << "[READY-CANDIDATE] joints=[";
    for (std::size_t i = 0; i < readyJoints.size(); ++i) {
      if (i) std::cout << ", ";
      std::cout << readyJoints[i];
    }
    std::cout << "] rad flange_B=[" << readyFlange.translation().transpose()
              << "] m sigma_min=" << readyMin
              << " cond=" << readyCondition << '\n';
    scanCameraDirections(model, flangeToCamera, readyFlange, readyJoints,
                         "READY");
  }
  if (opt.candidateJoints) {
    const auto& candidate = *opt.candidateJoints;
    for (std::size_t i = 0; i < candidate.size(); ++i) {
      if (candidate[i] <= softLimits[i][0] + 0.02 ||
          candidate[i] >= softLimits[i][1] - 0.02) {
        std::cerr << "[ERROR] candidate J" << (i + 1)
                  << " is outside the controller soft-limit margin\n";
        return 2;
      }
    }
    const Isometry candidateFlange =
        arrayToIsometry(model.getCartPose(candidate));
    if (!inConfiguredWorkspace(candidateFlange.translation())) {
      std::cerr << "[ERROR] candidate flange is outside the configured "
                   "workspace\n";
      return 2;
    }
    double candidateMin = 0.0;
    double candidateCondition = 0.0;
    jacobianMetrics(model, candidate, &candidateMin, &candidateCondition);
    std::cout << "[CANDIDATE] flange_B=["
              << candidateFlange.translation().transpose()
              << "] m sigma_min=" << candidateMin
              << " cond=" << candidateCondition << '\n';
    scanCameraDirections(model, flangeToCamera, candidateFlange, candidate,
                         "CANDIDATE");
    std::cout << "[CANDIDATE] pose scan does not validate the path from the "
                 "current joints or physical clearance\n";
  }
  std::cout << "[SAFE] model scan was read-only; no robot state was changed\n";
  return 0;
}

bool waitForIdle(rokae::xMateRobot& robot, int timeoutSeconds) {
  const auto deadline = Clock::now() + std::chrono::seconds(timeoutSeconds);
  while (Clock::now() < deadline) {
    std::this_thread::sleep_for(std::chrono::milliseconds(200));
    std::error_code ec;
    const auto state = robot.operationState(ec);
    if (!checkEc("operationState", ec)) return false;
    if (state == rokae::OperationState::idle) return true;
  }
  std::cerr << "[ERROR] timed out waiting for ready-position move\n";
  return false;
}

bool readStationaryJoints(rokae::xMateRobot& robot,
                          std::array<double, 6>* joints) {
  std::error_code ec;
  *joints = robot.jointPos(ec);
  if (!checkEc("read joints for stationary check", ec)) return false;
  for (int sample = 0; sample < 2; ++sample) {
    std::this_thread::sleep_for(std::chrono::milliseconds(200));
    const auto next = robot.jointPos(ec);
    if (!checkEc("recheck joints for stationary check", ec)) return false;
    for (std::size_t i = 0; i < joints->size(); ++i) {
      if (std::abs(next[i] - (*joints)[i]) > 0.002) {
        std::cerr << "[ERROR] J" << (i + 1)
                  << " changed during stationary check; stop other robot "
                     "motion before preview or execution\n";
        return false;
      }
    }
    *joints = next;
  }
  return true;
}

bool waitForFlangeTranslation(rokae::xMateRobot& robot,
                              const Eigen::Vector3d& target,
                              int timeoutSeconds) {
  const auto deadline = Clock::now() + std::chrono::seconds(timeoutSeconds);
  int settledSamples = 0;
  int sampleCount = 0;
  while (Clock::now() < deadline) {
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    std::error_code ec;
    const auto frame = robot.cartPosture(CoordinateType::flangeInBase, ec);
    if (!checkEc("read flange while waiting", ec)) return false;
    const Eigen::Vector3d actual(frame.trans[0], frame.trans[1], frame.trans[2]);
    const double error = (actual - target).norm();
    if (++sampleCount % 20 == 0) {
      std::cout << "[MOVE] target error=" << error << " m\n";
    }
    if (error <= 0.0010) {
      if (++settledSamples >= 5) return true;
    } else {
      settledSamples = 0;
    }
  }
  std::cerr << "[ERROR] timed out waiting for flange target ["
            << target.transpose() << "]\n";
  return false;
}

int runMoveReady(const Options& opt) {
  if (!localIpv4Exists(opt.localIp)) {
    std::cerr << "[ERROR] local robot-network address " << opt.localIp
              << " is not active\n";
    return 2;
  }
  rokae::xMateRobot robot(opt.robotIp, opt.localIp);
  std::error_code ec;
  const auto info = robot.robotInfo(ec);
  if (!checkEc("robotInfo", ec)) return 2;
  auto model = robot.model();
  std::array<double, 6> currentJoints{};
  if (!readStationaryJoints(robot, &currentJoints)) return 2;
  std::array<double[2], 6> softLimits{};
  robot.getSoftLimit(softLimits, ec);
  if (!checkEc("read soft limits", ec)) return 2;

  bool adjusted = false;
  std::array<double, 6> readyJoints{};
  if (opt.candidateJoints) {
    readyJoints = *opt.candidateJoints;
    for (std::size_t i = 0; i < readyJoints.size(); ++i) {
      adjusted = adjusted ||
                 std::abs(readyJoints[i] - currentJoints[i]) > 1e-4;
    }
  } else {
    readyJoints = makeServoReadyJoints(currentJoints, &adjusted);
  }
  std::cout << std::fixed << std::setprecision(4) << "[READY] target=[";
  for (std::size_t i = 0; i < readyJoints.size(); ++i) {
    if (i) std::cout << ", ";
    std::cout << readyJoints[i];
  }
  std::cout << "] rad\n";
  if (!adjusted) {
    std::cout << "[READY] target matches current posture\n";
    return 0;
  }

  constexpr int kPathSamples = 40;
  double previousMinSingular = 0.0;
  double previousCondition = std::numeric_limits<double>::infinity();
  bool escapingInitialSingularity = false;
  for (int sample = 0; sample <= kPathSamples; ++sample) {
    const double ratio = static_cast<double>(sample) / kPathSamples;
    std::array<double, 6> joints{};
    for (std::size_t i = 0; i < joints.size(); ++i) {
      joints[i] = currentJoints[i] + ratio * (readyJoints[i] - currentJoints[i]);
      if (joints[i] <= softLimits[i][0] + 0.02 ||
          joints[i] >= softLimits[i][1] - 0.02) {
        std::cerr << "[ERROR] ready path approaches controller limit on J"
                  << (i + 1) << '\n';
        return 2;
      }
    }
    double minSingular = 0.0;
    double condition = 0.0;
    jacobianMetrics(model, joints, &minSingular, &condition);
    const bool singular =
        minSingular < cfg::kMinJacobianSingularValue ||
        condition > cfg::kMaxJacobianCondition;
    if (sample == 0) {
      escapingInitialSingularity = singular;
      if (escapingInitialSingularity) {
        std::cout << "[READY] current pose is singular; validating monotonic "
                     "joint-space escape\n";
      }
    } else if (singular) {
      const bool improving =
          minSingular + 1e-6 >= previousMinSingular &&
          condition <= previousCondition * 1.02;
      if (!escapingInitialSingularity || !improving ||
          sample > kPathSamples / 2) {
        std::cerr << "[ERROR] ready path approaches singularity at sample "
                  << sample << " sigma_min=" << minSingular
                  << " cond=" << condition << '\n';
        return 2;
      }
    } else {
      escapingInitialSingularity = false;
    }
    previousMinSingular = minSingular;
    previousCondition = condition;
    const Isometry flange = arrayToIsometry(model.getCartPose(joints));
    const Eigen::Vector3d p = flange.translation();
    if (std::hypot(p.x(), p.y()) > cfg::kWorkspaceRadiusMaxM ||
        p.z() < cfg::kWorkspaceZMinM || p.z() > cfg::kWorkspaceZMaxM) {
      std::cerr << "[ERROR] ready path leaves configured workspace envelope\n";
      return 2;
    }
  }
  std::cout << "[READY] 40-sample joint path safety check passed for "
            << info.type << '\n';
  if (!opt.execute) {
    std::cout << "[SAFE] dry run only; add --execute after clearing the workspace\n";
    return 0;
  }

  const auto beforeMoveJoints = robot.jointPos(ec);
  if (!checkEc("recheck joints before ready move", ec)) return 2;
  for (std::size_t i = 0; i < currentJoints.size(); ++i) {
    if (std::abs(beforeMoveJoints[i] - currentJoints[i]) > 0.002) {
      std::cerr << "[ERROR] ready-move starting pose changed after path "
                   "validation; no motion was commanded\n";
      return 2;
    }
  }

  bool configured = false;
  try {
    robot.setOperateMode(OperateMode::automatic, ec);
    if (!checkEc("setOperateMode automatic", ec)) return 2;
    configured = true;
    robot.setPowerState(true, ec);
    if (!checkEc("setPowerState on", ec)) throw std::runtime_error("power on failed");
    robot.setMotionControlMode(MotionControlMode::NrtCommand, ec);
    if (!checkEc("setMotionControlMode NrtCommand", ec)) {
      throw std::runtime_error("NRT mode failed");
    }
    robot.moveReset(ec);
    if (!checkEc("moveReset", ec)) throw std::runtime_error("move reset failed");
    robot.setDefaultSpeed(20.0, ec);
    if (!checkEc("setDefaultSpeed 20", ec)) throw std::runtime_error("speed failed");
    robot.setDefaultZone(0, ec);
    if (!checkEc("setDefaultZone 0", ec)) throw std::runtime_error("zone failed");
    const std::vector<double> target(readyJoints.begin(), readyJoints.end());
    robot.executeCommand({rokae::MoveAbsJCommand(target, 20.0, 0.0)}, ec);
    if (!checkEc("execute servo-ready MoveAbsJ", ec)) {
      throw std::runtime_error("ready move command failed");
    }
    if (!waitForIdle(robot, 60)) throw std::runtime_error("ready move timeout");
  } catch (...) {
    cleanupRobot(robot, configured, false);
    throw;
  }
  cleanupRobot(robot, configured, false);
  std::cout << "[OK] servo-ready posture reached; robot returned to manual/power-off\n";
  return 0;
}

int runIdentifyAxes(const Options& opt) {
  Isometry flangeToCamera;
  double boardSquareSizeM = user_config::SQUARE_SIZE;
  if (!loadCalibration(opt.calibrationFile, &flangeToCamera,
                       &boardSquareSizeM)) return 2;
  if (!localIpv4Exists(opt.localIp)) {
    std::cerr << "[ERROR] local robot-network address " << opt.localIp
              << " is not active\n";
    return 2;
  }

  rokae::xMateRobot robot(opt.robotIp, opt.localIp);
  std::error_code ec;
  const auto info = robot.robotInfo(ec);
  if (!checkEc("robotInfo", ec)) return 2;
  auto model = robot.model();
  const auto originalFrame =
      robot.cartPosture(CoordinateType::flangeInBase, ec);
  if (!checkEc("read flange pose", ec)) return 2;
  const auto originalJoints = robot.jointPos(ec);
  if (!checkEc("read joints", ec)) return 2;
  const Isometry originalFlange = frameToIsometry(originalFrame);

  for (int axis = 0; axis < 3; ++axis) {
    for (double sign : {-1.0, 1.0}) {
      Isometry target = originalFlange;
      target.translation()[axis] += sign * opt.axisStepM;
      const auto safety = validateTarget(model, target, originalJoints);
      if (!safety.safe) {
        std::cerr << "[ERROR] axis identification target rejected: axis="
                  << axis << " sign=" << sign << " reason=" << safety.reason
                  << '\n';
        return 2;
      }
    }
  }
  std::cout << "[AXIS-ID] six +/-" << opt.axisStepM
            << " m targets passed IK and singularity checks\n";
  if (!opt.execute) {
    std::cout << "[SAFE] dry run only; keep the board fixed and add --execute\n";
    return 0;
  }

  rs2::pipeline pipe;
  rs2::config cameraConfig;
  cameraConfig.enable_device(opt.cameraSerial);
  cameraConfig.enable_stream(RS2_STREAM_COLOR, cfg::kColorWidth,
                             cfg::kColorHeight, RS2_FORMAT_BGR8,
                             cfg::kCameraFps);
  cameraConfig.enable_stream(RS2_STREAM_DEPTH, cfg::kColorWidth,
                             cfg::kColorHeight, RS2_FORMAT_Z16,
                             cfg::kCameraFps);
  const auto profile = pipe.start(cameraConfig);
  const auto colorProfile = profile.get_stream(RS2_STREAM_COLOR)
                                .as<rs2::video_stream_profile>();
  const rs2_intrinsics colorIntrinsics = colorProfile.get_intrinsics();
  rs2::align alignToColor(RS2_STREAM_COLOR);
  for (int i = 0; i < 15; ++i) pipe.wait_for_frames(5000);

  Eigen::Vector3d baseline;
  double estimatedSquare = 0.0;
  if (!collectBoardCenter(pipe, alignToColor, colorIntrinsics, &baseline,
                          &estimatedSquare)) {
    pipe.stop();
    std::cerr << "[ERROR] could not collect a stable checkerboard baseline\n";
    return 2;
  }
  std::cout << std::fixed << std::setprecision(5)
            << "[AXIS-ID] baseline_C=[" << baseline.transpose()
            << "] m square_est=" << estimatedSquare * 1000.0 << " mm\n";

  bool configured = false;
  bool stateReceiving = false;
  bool followerStarted = false;
  std::unique_ptr<rokae::FollowPosition<6>> axisFollower;
  auto cleanup = [&]() {
    if (followerStarted && axisFollower) {
      axisFollower->stop();
      followerStarted = false;
    }
    cleanupRobot(robot, configured, stateReceiving);
    pipe.stop();
  };
  try {
    robot.setRtNetworkTolerance(20, ec);
    if (!checkEc("setRtNetworkTolerance", ec)) {
      throw std::runtime_error("realtime network tolerance failed");
    }
    robot.setOperateMode(OperateMode::automatic, ec);
    if (!checkEc("setOperateMode automatic", ec)) {
      throw std::runtime_error("automatic mode failed");
    }
    configured = true;
    robot.setMotionControlMode(MotionControlMode::RtCommand, ec);
    if (!checkEc("setMotionControlMode RtCommand", ec)) {
      throw std::runtime_error("realtime mode failed");
    }
    robot.setPowerState(true, ec);
    if (!checkEc("setPowerState on", ec)) throw std::runtime_error("power on failed");
    robot.startReceiveRobotState(std::chrono::milliseconds(1), {jointPos_m});
    stateReceiving = true;
    std::this_thread::sleep_for(std::chrono::seconds(1));
    axisFollower = std::make_unique<rokae::FollowPosition<6>>(robot, model);
    axisFollower->setScale(0.30);

    auto moveTo = [&](const Eigen::Vector3d& translation) {
      Isometry target = originalFlange;
      target.translation() = translation;
      const auto joints = robot.jointPos(ec);
      if (!checkEc("read joints before axis move", ec)) {
        throw std::runtime_error("joint read failed");
      }
      const auto safety = validateTarget(model, target, joints);
      if (!safety.safe) {
        throw std::runtime_error("axis move rejected: " + safety.reason);
      }
      if (!followerStarted) {
        axisFollower->start(target);
        followerStarted = true;
      } else {
        axisFollower->update(target);
      }
      if (!waitForFlangeTranslation(robot, translation, 30)) {
        throw std::runtime_error("axis FollowPosition timeout");
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(500));
    };

    Eigen::Matrix3d cameraDeltaPerBase;
    for (int axis = 0; axis < 3; ++axis) {
      Eigen::Vector3d axisBaseline;
      double axisBaselineSquare = 0.0;
      if (!collectBoardCenter(pipe, alignToColor, colorIntrinsics,
                              &axisBaseline, &axisBaselineSquare)) {
        throw std::runtime_error("board lost at axis-local baseline");
      }
      std::cout << "[AXIS-ID] base " << "XYZ"[axis]
                << " local baseline_C=[" << axisBaseline.transpose() << "]\n";
      Eigen::Vector3d plusTranslation = originalFlange.translation();
      plusTranslation[axis] += opt.axisStepM;
      moveTo(plusTranslation);
      Eigen::Vector3d plusPoint;
      double plusSquare = 0.0;
      if (!collectBoardCenter(pipe, alignToColor, colorIntrinsics, &plusPoint,
                              &plusSquare)) {
        throw std::runtime_error("board lost at positive axis sample");
      }
      moveTo(originalFlange.translation());

      Eigen::Vector3d minusTranslation = originalFlange.translation();
      minusTranslation[axis] -= opt.axisStepM;
      moveTo(minusTranslation);
      Eigen::Vector3d minusPoint;
      double minusSquare = 0.0;
      if (!collectBoardCenter(pipe, alignToColor, colorIntrinsics, &minusPoint,
                              &minusSquare)) {
        throw std::runtime_error("board lost at negative axis sample");
      }
      cameraDeltaPerBase.col(axis) =
          -(plusPoint - minusPoint) / (2.0 * opt.axisStepM);
      std::cout << "[AXIS-ID] base " << "XYZ"[axis]
                << " plus_C=[" << plusPoint.transpose() << "] minus_C=["
                << minusPoint.transpose() << "]\n";
      moveTo(originalFlange.translation());
      Eigen::Vector3d returnedPoint;
      double returnedSquare = 0.0;
      if (!collectBoardCenter(pipe, alignToColor, colorIntrinsics,
                              &returnedPoint, &returnedSquare)) {
        throw std::runtime_error("board lost after returning to origin");
      }
      const double returnError = (returnedPoint - axisBaseline).norm();
      std::cout << "[AXIS-ID] return error=" << returnError << " m\n";
      if (returnError > 0.005) {
        throw std::runtime_error(
            "board moved or measurement drift exceeded 5 mm");
      }
    }

    Eigen::JacobiSVD<Eigen::Matrix3d> svd(
        cameraDeltaPerBase, Eigen::ComputeFullU | Eigen::ComputeFullV);
    Eigen::Matrix3d cameraFromBase =
        svd.matrixU() * svd.matrixV().transpose();
    if (cameraFromBase.determinant() < 0.0) {
      Eigen::Matrix3d u = svd.matrixU();
      u.col(2) *= -1.0;
      cameraFromBase = u * svd.matrixV().transpose();
    }
    const Eigen::Matrix3d baseFromCamera = cameraFromBase.transpose();
    const Eigen::Matrix3d correctedFlangeFromCamera =
        originalFlange.linear().transpose() * baseFromCamera;
    const double correctionAngle = Eigen::AngleAxisd(
        flangeToCamera.linear().transpose() * correctedFlangeFromCamera).angle();
    flangeToCamera.linear() = correctedFlangeFromCamera;

    std::filesystem::path outputPath = opt.outputCalibrationFile;
    if (outputPath.empty()) {
      const std::filesystem::path inputPath(opt.calibrationFile);
      outputPath = inputPath.parent_path() /
                   (inputPath.stem().string() + "_axis_verified.yml");
    }
    cv::Mat matrix(4, 4, CV_64F);
    for (int r = 0; r < 4; ++r) {
      for (int c = 0; c < 4; ++c) {
        matrix.at<double>(r, c) = flangeToCamera.matrix()(r, c);
      }
    }
    cv::FileStorage storage(outputPath.string(), cv::FileStorage::WRITE);
    storage << "H_Flange2Cam" << matrix;
    storage << "camera_serial" << opt.cameraSerial;
    storage << "board_width" << BOARD_SIZE.width;
    storage << "board_height" << BOARD_SIZE.height;
    storage << "square_size_m" << boardSquareSizeM;
    storage << "square_size_depth_estimate_m" << estimatedSquare;
    storage << "axis_identification_step_m" << opt.axisStepM;
    storage << "rotation_correction_angle_rad" << correctionAngle;
    storage.release();
    std::cout << "[AXIS-ID] raw camera_from_base=\n"
              << cameraDeltaPerBase << "\n[AXIS-ID] rotation correction="
              << correctionAngle << " rad\n[OK] saved " << outputPath.string()
              << '\n';
  } catch (...) {
    cleanup();
    throw;
  }
  cleanup();
  return 0;
}

Eigen::Vector3d trackingError(const Eigen::Vector3d& boardInCamera,
                              double desiredDistance) {
  Eigen::Vector3d error = boardInCamera - Eigen::Vector3d(0.0, 0.0, desiredDistance);
  for (int i = 0; i < 3; ++i) {
    if (std::abs(error[i]) < cfg::kTranslationDeadbandM) error[i] = 0.0;
  }
  return error;
}

bool shouldHoldFollowing(bool detected, bool measurementValid, bool targetSafe) {
  return !detected || !measurementValid || !targetSafe;
}

Isometry buildTarget(const Isometry& baseToFlange,
                     const Isometry& flangeToCamera,
                     const Eigen::Vector3d& boardInCamera,
                     double desiredDistance,
                     const Isometry* previousTarget) {
  const Isometry baseToCamera = baseToFlange * flangeToCamera;
  Eigen::Vector3d error = trackingError(boardInCamera, desiredDistance);
  if (error.norm() > cfg::kMaxCorrectionFromCurrentM) {
    error *= cfg::kMaxCorrectionFromCurrentM / error.norm();
  }

  Isometry desiredCamera = baseToCamera;
  desiredCamera.translation() += baseToCamera.linear() * error;
  Isometry target = desiredCamera * flangeToCamera.inverse();

  if (previousTarget) {
    Eigen::Vector3d step = target.translation() - previousTarget->translation();
    if (step.norm() > cfg::kMaxCommandStepM) {
      step *= cfg::kMaxCommandStepM / step.norm();
      target.translation() = previousTarget->translation() + step;
    }
  }
  return target;
}

int runSelfTest() {
  int failures = 0;
  auto expect = [&](bool condition, const std::string& label) {
    std::cout << (condition ? "[PASS] " : "[FAIL] ") << label << '\n';
    if (!condition) ++failures;
  };

  expect(std::abs(qualityCheckedDepth(std::vector<double>(49, 0.60)) - 0.60) <
             1e-12,
         "depth patch accepts a dense, consistent surface");
  expect(!std::isfinite(qualityCheckedDepth(std::vector<double>(10, 0.60))),
         "depth patch rejects sparse measurements");
  std::vector<double> edgeDepth(49, 0.60);
  std::fill(edgeDepth.begin() + 24, edgeDepth.end(), 0.66);
  expect(!std::isfinite(qualityCheckedDepth(edgeDepth)),
         "depth patch rejects mixed foreground/background surfaces");
  expect(shouldHoldFollowing(false, true, true) &&
             !shouldHoldFollowing(true, true, true),
         "first lost checkerboard frame pauses following");

  PositionFilter filter;
  std::string reason;
  const auto t0 = Clock::now();
  expect(filter.update(Eigen::Vector3d(0.0, 0.0, 0.60), t0, &reason),
         "filter accepts initial measurement");
  expect(filter.update(Eigen::Vector3d(0.002, -0.001, 0.601),
                       t0 + std::chrono::milliseconds(67), &reason),
         "filter accepts normal board motion");
  expect(!filter.update(Eigen::Vector3d(0.20, 0.0, 0.80),
                        t0 + std::chrono::milliseconds(134), &reason),
         "filter rejects abnormal jump");
  bool relocked = false;
  for (int i = 1; i < cfg::kRelockFrames; ++i) {
    relocked = filter.update(Eigen::Vector3d(0.201, 0.001, 0.801),
                               t0 + std::chrono::milliseconds(134 + 67 * i),
                               &reason);
  }
  expect(relocked, "filter relocks after a new position remains stable");
  expect(!filter.update(
             Eigen::Vector3d(0.0, 0.0, cfg::kMinimumAllowedBoardDistanceM - 0.01),
             t0 + std::chrono::milliseconds(469), &reason) &&
             reason == "board too close for estimated tool clearance",
         "filter holds when board breaches tool clearance");
  expect(filter.update(Eigen::Vector3d(0.201, 0.001, 0.801),
                       t0 + std::chrono::milliseconds(536), &reason),
         "filter resumes after board returns to safe depth");

  const Isometry identity = Isometry::Identity();
  const Isometry target = buildTarget(identity, identity,
                                      Eigen::Vector3d(0.02, -0.01, 0.61),
                                      0.55, &identity);
  expect(target.translation().norm() <= cfg::kMaxCommandStepM + 1e-12,
         "first Cartesian command is slew-limited");
  expect((target.linear() - Eigen::Matrix3d::Identity()).norm() < 1e-12,
         "command shaping preserves a valid rotation");
  expect(target.translation().z() > 0.0,
         "distance error moves camera toward the board");
  Isometry offsetCamera = Isometry::Identity();
  offsetCamera.translation() = Eigen::Vector3d(-0.10, -0.07, 0.23);
  offsetCamera.linear() = Eigen::AngleAxisd(
      0.4, Eigen::Vector3d::UnitY()).toRotationMatrix();
  const Eigen::Vector3d boardForRelativeMove(0.02, -0.01, 0.62);
  const Isometry relativeTarget = buildTarget(
      identity, offsetCamera, boardForRelativeMove, 0.60, nullptr);
  const Eigen::Vector3d expectedRelativeMove =
      offsetCamera.linear() * (boardForRelativeMove -
                               Eigen::Vector3d(0.0, 0.0, 0.60));
  expect((relativeTarget.translation() - expectedRelativeMove).norm() < 1e-12,
         "fixed-orientation relative move cancels hand-eye translation");
  expect(trackingError(Eigen::Vector3d(0.0, 0.0, 1.09), 0.60).norm() >
             cfg::kMaxCorrectionFromCurrentM,
         "large target relocation exceeds the tracking envelope");

  std::vector<Eigen::Vector3d> pathPoints;
  const bool completePath = sampleTranslationPath(
      Eigen::Vector3d::Zero(), Eigen::Vector3d(0.008, 0.0, 0.0),
      [&](const Eigen::Vector3d& point, int, int) {
        pathPoints.push_back(point);
        return true;
      });
  expect(completePath && pathPoints.size() == 2 &&
             (pathPoints[0] - Eigen::Vector3d(0.004, 0.0, 0.0)).norm() < 1e-12 &&
             (pathPoints[1] - Eigen::Vector3d(0.008, 0.0, 0.0)).norm() < 1e-12,
         "8 mm command checks midpoint and endpoint");
  int checkedPoints = 0;
  const bool rejectedPath = sampleTranslationPath(
      Eigen::Vector3d::Zero(), Eigen::Vector3d(0.008, 0.0, 0.0),
      [&](const Eigen::Vector3d&, int sample, int) {
        ++checkedPoints;
        return sample != 1;
      });
  expect(!rejectedPath && checkedPoints == 1,
         "unsafe midpoint prevents endpoint command");

  std::cout << (failures ? "[SELF-TEST] FAILED\n" : "[SELF-TEST] ALL PASSED\n");
  return failures ? 1 : 0;
}

void cleanupRobot(rokae::xMateRobot& robot, bool robotConfigured,
                  bool stateReceiving) {
  if (!robotConfigured) return;
  if (stateReceiving) robot.stopReceiveRobotState();
  std::error_code ec;
  robot.setMotionControlMode(MotionControlMode::Idle, ec);
  checkEc("setMotionControlMode Idle", ec);
  robot.setPowerState(false, ec);
  checkEc("setPowerState off", ec);
  robot.setOperateMode(OperateMode::manual, ec);
  checkEc("setOperateMode manual", ec);
}

std::string statusText(bool detected, bool measurementValid, bool targetSafe,
                       bool following, const std::string& reason) {
  if (!detected) return "SEARCHING BOARD";
  if (!measurementValid) return "HOLD: FILTER REJECTED";
  if (!targetSafe) return "HOLD: " + reason;
  return following ? "FOLLOWING" : "READY / OBSERVE ONLY";
}

int run(const Options& opt) {
  Isometry flangeToCamera;
  double boardSquareSizeM = user_config::SQUARE_SIZE;
  if (!loadCalibration(opt.calibrationFile, &flangeToCamera,
                       &boardSquareSizeM)) return 2;
  if (opt.execute &&
      !isAxisVerifiedCalibration(opt.calibrationFile, opt.cameraSerial)) {
    std::cerr << "[ERROR] realtime following requires an axis-verified "
                 "calibration for camera " << opt.cameraSerial << ". Run "
                 "--identify-axes --execute first, then use its output file.\n";
    return 2;
  }
  if (!opt.execute &&
      !isAxisVerifiedCalibration(opt.calibrationFile, opt.cameraSerial)) {
    std::cout << "[WARN] calibration axes are unverified; observe-only "
                 "target directions may be inaccurate\n";
  }

  std::cout << "[INFO] arm=" << opt.arm << " robot=" << opt.robotIp
            << " camera=" << opt.cameraSerial << " distance=" << opt.distanceM
            << " mode=" << (opt.execute ? "EXECUTE" : "OBSERVE")
            << " target=" << (opt.jointFollow ? "JOINT" : "CARTESIAN") << '\n';

  std::ofstream log;
  if (!opt.logFile.empty()) {
    log.open(opt.logFile, std::ios::out | std::ios::trunc);
    if (!log) {
      std::cerr << "[ERROR] cannot open log file: " << opt.logFile << '\n';
      return 2;
    }
    log << "time_s,detected,measurement_valid,target_safe,following,board_x_m,"
           "board_y_m,board_z_m,sigma_min,condition,reason\n";
    std::cout << "[INFO] logging to " << opt.logFile << '\n';
  }

  if (!localIpv4Exists(opt.localIp)) {
    std::cerr << "[ERROR] local robot-network address " << opt.localIp
              << " is not active. Bring up the wired robot interface before connecting; "
                 "refusing to use another route.\n";
    return 2;
  }

  if (listRealSenseDevices(opt.cameraSerial) != 0) return 2;

  rokae::xMateRobot robot(opt.robotIp, opt.localIp);
  std::error_code ec;
  const auto info = robot.robotInfo(ec);
  if (!checkEc("robotInfo", ec)) return 2;
  std::cout << "[OK] controller=" << info.version << " robot=" << info.type << '\n';
  if (info.joint_num != 6) {
    std::cerr << "[ERROR] this program requires a 6-axis XMC18 arm\n";
    return 2;
  }
  auto model = robot.model();
  std::array<double[2], 6> jointFollowLimits{};
  if (opt.jointFollow) {
    const bool enabled = robot.getSoftLimit(jointFollowLimits, ec);
    if (!checkEc("read soft limits for joint follow", ec)) return 2;
    if (!enabled) {
      std::cerr << "[ERROR] joint following requires enabled controller soft limits\n";
      return 2;
    }
  }

  if (opt.execute) {
    std::array<double, 6> joints{};
    if (!readStationaryJoints(robot, &joints)) return 2;
    std::array<double[2], 6> limits{};
    const bool softLimitsEnabled = robot.getSoftLimit(limits, ec);
    if (!checkEc("read soft limits for startup preflight", ec)) return 2;
    const std::string startupFailure = servoStartPreflightFailure(
        model, joints, limits, softLimitsEnabled);
    if (!startupFailure.empty()) {
      std::cerr << "[ERROR] startup preflight: " << startupFailure
                << "; robot mode was not changed\n";
      return 2;
    }
    std::cout << "[PREFLIGHT] startup pose passed workspace, soft-limit, and "
                 "Jacobian checks\n";
  }

  rs2::pipeline pipe;
  rs2::config cameraConfig;
  if (!opt.cameraSerial.empty()) cameraConfig.enable_device(opt.cameraSerial);
  cameraConfig.enable_stream(RS2_STREAM_COLOR, cfg::kColorWidth, cfg::kColorHeight,
                             RS2_FORMAT_BGR8, cfg::kCameraFps);
  cameraConfig.enable_stream(RS2_STREAM_DEPTH, cfg::kColorWidth, cfg::kColorHeight,
                             RS2_FORMAT_Z16, cfg::kCameraFps);
  const auto profile = pipe.start(cameraConfig);
  const auto colorProfile = profile.get_stream(RS2_STREAM_COLOR)
                                .as<rs2::video_stream_profile>();
  const rs2_intrinsics colorIntrinsics = colorProfile.get_intrinsics();
  cv::Mat distortion;
  const cv::Mat intrinsics = cameraMatrix(colorProfile, &distortion);
  rs2::align alignToColor(RS2_STREAM_COLOR);
  for (int i = 0; i < 15; ++i) pipe.wait_for_frames(5000);
  std::cout << "[OK] RealSense streams started\n";

  bool robotConfigured = false;
  bool stateReceiving = false;
  bool following = false;
  std::unique_ptr<rokae::FollowPosition<6>> follower;
  std::optional<Eigen::Matrix3d> jointReferenceRotation;
  if (opt.jointFollow) {
    const auto frame = robot.cartPosture(CoordinateType::flangeInBase, ec);
    if (!checkEc("read initial flange for joint follow", ec)) {
      pipe.stop();
      return 2;
    }
    jointReferenceRotation =
        (frameToIsometry(frame) * flangeToCamera).linear();
  }
  if (opt.execute) {
    try {
      std::array<double, 6> joints{};
      if (!readStationaryJoints(robot, &joints)) {
        throw std::runtime_error("startup joint recheck failed");
      }
      std::array<double[2], 6> limits{};
      const bool softLimitsEnabled = robot.getSoftLimit(limits, ec);
      if (!checkEc("recheck soft limits before arming", ec)) {
        throw std::runtime_error("startup soft-limit recheck failed");
      }
      const std::string startupFailure = servoStartPreflightFailure(
          model, joints, limits, softLimitsEnabled);
      if (!startupFailure.empty()) {
        throw std::runtime_error("startup pose changed during camera warmup: " +
                                 startupFailure);
      }
      robot.setRtNetworkTolerance(20, ec);
      if (!checkEc("setRtNetworkTolerance", ec)) {
        throw std::runtime_error("failed to configure realtime network tolerance");
      }
      robot.setOperateMode(OperateMode::automatic, ec);
      if (!checkEc("setOperateMode automatic", ec)) {
        throw std::runtime_error("failed to enter automatic mode");
      }
      robotConfigured = true;
      robot.setMotionControlMode(MotionControlMode::RtCommand, ec);
      if (!checkEc("setMotionControlMode RtCommand", ec)) {
        throw std::runtime_error("failed to enter realtime command mode");
      }
      robot.setPowerState(true, ec);
      if (!checkEc("setPowerState on", ec)) {
        throw std::runtime_error("failed to power on robot");
      }
      robot.startReceiveRobotState(std::chrono::milliseconds(1), {jointPos_m});
      stateReceiving = true;
      follower = std::make_unique<rokae::FollowPosition<6>>(robot, model);
      follower->setScale(cfg::kFollowSpeedScale);
      std::cout << "[ARMED] realtime following enabled; press Q or Ctrl+C to stop\n";
    } catch (...) {
      cleanupRobot(robot, robotConfigured, stateReceiving);
      pipe.stop();
      throw;
    }
  } else {
    std::cout << "[SAFE] observe-only: no robot mode or power state was changed\n";
  }

  std::vector<cv::Point3f> boardPoints;
  for (int row = 0; row < BOARD_SIZE.height; ++row) {
    for (int col = 0; col < BOARD_SIZE.width; ++col) {
      boardPoints.emplace_back(col * boardSquareSizeM,
                               row * boardSquareSizeM, 0.0f);
    }
  }
  const Eigen::Vector3d boardCenter(
      0.5 * (BOARD_SIZE.width - 1) * boardSquareSizeM,
      0.5 * (BOARD_SIZE.height - 1) * boardSquareSizeM, 0.0);

  PositionFilter filter;
  int validFrames = 0;
  int safeFrames = 0;
  int lostFrames = 0;
  int frameNumber = 0;
  bool boardScaleWarningPrinted = false;
  std::string lastPrintedReason;

  try {
    const auto runStart = Clock::now();
    while (!g_stop) {
      const auto frames = alignToColor.process(pipe.wait_for_frames(5000));
      const auto color = frames.get_color_frame();
      const auto depth = frames.get_depth_frame();
      if (!color || !depth) continue;

      cv::Mat image(cv::Size(cfg::kColorWidth, cfg::kColorHeight), CV_8UC3,
                    const_cast<void*>(color.get_data()), cv::Mat::AUTO_STEP);
      cv::Mat display = image.clone();
      cv::Mat gray;
      cv::cvtColor(image, gray, cv::COLOR_BGR2GRAY);
      std::vector<cv::Point2f> corners;
      const bool detected = cv::findChessboardCorners(
          gray, BOARD_SIZE, corners,
          cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_NORMALIZE_IMAGE);

      bool measurementValid = false;
      bool targetSafe = false;
      std::string reason = "board not detected";
      Eigen::Vector3d filteredBoard = Eigen::Vector3d::Zero();
      SafetyResult safety;

      if (detected) {
        lostFrames = 0;
        cv::cornerSubPix(gray, corners, cv::Size(11, 11), cv::Size(-1, -1),
                         cv::TermCriteria(cv::TermCriteria::EPS |
                                              cv::TermCriteria::COUNT,
                                          30, 0.01));
        cv::drawChessboardCorners(display, BOARD_SIZE, corners, true);
        const cv::Point2f pixelCenter =
            std::accumulate(corners.begin(), corners.end(), cv::Point2f()) *
            (1.0f / static_cast<float>(corners.size()));
        const double depthM = robustBoardCenterDepth(depth, pixelCenter, corners);
        if (std::isfinite(depthM)) {
          const float pixelData[2]{pixelCenter.x, pixelCenter.y};
          float pointData[3]{};
          rs2_deproject_pixel_to_point(pointData, &colorIntrinsics, pixelData,
                                       static_cast<float>(depthM));
          const Eigen::Vector3d rawBoard(pointData[0], pointData[1], pointData[2]);
          measurementValid = filter.update(rawBoard, Clock::now(), &reason);

          if (!boardScaleWarningPrinted) {
            cv::Mat rvec;
            cv::Mat tvec;
            if (cv::solvePnP(boardPoints, corners, intrinsics, distortion,
                             rvec, tvec, false, cv::SOLVEPNP_ITERATIVE)) {
              cv::Mat rotationCv;
              cv::Rodrigues(rvec, rotationCv);
              Eigen::Matrix3d rotation;
              Eigen::Vector3d translation;
              for (int r = 0; r < 3; ++r) {
                translation[r] = tvec.at<double>(r);
                for (int c = 0; c < 3; ++c) {
                  rotation(r, c) = rotationCv.at<double>(r, c);
                }
              }
              const Eigen::Vector3d pnpBoard = rotation * boardCenter + translation;
              if (std::abs(depthM - pnpBoard.z()) >
                  cfg::kMaxDepthPnPDifferenceM) {
                const double estimatedSquare = estimateSquareSizeFromDepth(
                    corners, depth, colorIntrinsics);
                std::cout << "[WARN] configured checker square="
                          << boardSquareSizeM * 1000.0
                          << " mm, depth estimate=" << estimatedSquare * 1000.0
                          << " mm; translation tracking uses depth deprojection\n";
              }
            }
            boardScaleWarningPrinted = true;
          }
        } else {
          reason = "invalid depth at board center";
        }

        if (measurementValid) {
          ++validFrames;
          filteredBoard = filter.value();
          const double trackingErrorM =
              trackingError(filteredBoard, opt.distanceM).norm();
          if (trackingErrorM > cfg::kMaxCorrectionFromCurrentM) {
            std::ostringstream message;
            message << std::fixed << std::setprecision(3)
                    << "board tracking error exceeds safety envelope: error="
                    << trackingErrorM << " m limit="
                    << cfg::kMaxCorrectionFromCurrentM << " m";
            reason = message.str();
            targetSafe = false;
            safeFrames = 0;
          } else {
            const auto flangeFrame =
                robot.cartPosture(CoordinateType::flangeInBase, ec);
            if (!checkEc("read flange pose", ec)) {
              reason = "robot flange pose unavailable";
              validFrames = 0;
            } else {
              const auto currentJoints = robot.jointPos(ec);
              if (!checkEc("read joints", ec)) {
                reason = "robot joints unavailable";
                validFrames = 0;
              } else {
                const Isometry currentFlange = frameToIsometry(flangeFrame);
                const Isometry target = buildTarget(
                    currentFlange, flangeToCamera, filteredBoard, opt.distanceM,
                    &currentFlange);
                if (opt.jointFollow) {
                  const Isometry currentCamera =
                      currentFlange * flangeToCamera;
                  const Isometry desiredCamera = target * flangeToCamera;
                  const Eigen::Vector3d fullStep =
                      desiredCamera.translation() -
                      currentCamera.translation();
                  for (const double fraction : {1.0, 0.5, 0.25}) {
                    safety = validateJointMicroStep(
                        model, currentJoints, jointFollowLimits,
                        flangeToCamera, fraction * fullStep);
                    if (safety.safe) break;
                  }
                  if (safety.safe) {
                    const Isometry predictedCamera =
                        arrayToIsometry(model.getCartPose(
                            safety.predictedJoints)) * flangeToCamera;
                    if (Eigen::AngleAxisd(
                            jointReferenceRotation->transpose() *
                            predictedCamera.linear()).angle() > 0.025) {
                      safety.safe = false;
                      safety.reason = "camera orientation drift exceeds 0.025 rad";
                    }
                  }
                } else {
                  safety = validateTarget(model, target, currentJoints);
                }
                targetSafe = safety.safe;
                reason = safety.reason;

                if (targetSafe) {
                  ++safeFrames;
                } else {
                  safeFrames = 0;
                }
                if (targetSafe && safeFrames >= cfg::kValidFramesToResume) {
                  if (opt.execute) {
                    if (!following) {
                      if (opt.jointFollow) {
                        follower->start(safety.predictedJoints);
                      } else {
                        follower->start(target);
                      }
                      following = true;
                      std::cout << "[FOLLOW] resumed after " << validFrames
                                << " valid frames\n";
                    } else {
                      if (opt.jointFollow) {
                        follower->update(safety.predictedJoints);
                      } else {
                        follower->update(target);
                      }
                    }
                  }
                }
              }
            }
          }
        } else {
          validFrames = 0;
          safeFrames = 0;
        }
      } else {
        validFrames = 0;
        safeFrames = 0;
        ++lostFrames;
        if (lostFrames >= cfg::kLostFramesToResetFilter) {
          filter.reset();
        }
      }

      const bool mustHold =
          shouldHoldFollowing(detected, measurementValid, targetSafe);
      if (mustHold && following) {
        follower->stop();
        following = false;
        std::cout << "[HOLD] " << reason << '\n';
      }

      const std::string reasonCategory = reason.substr(0, reason.find(':'));
      if (reasonCategory != lastPrintedReason &&
          (!targetSafe || !measurementValid)) {
        std::cout << "[STATUS] " << reason << '\n';
        lastPrintedReason = reasonCategory;
      }
      if (++frameNumber % 15 == 0 && measurementValid) {
        std::cout << std::fixed << std::setprecision(3)
                  << "[VISION] board_C=[" << filteredBoard.x() << ", "
                  << filteredBoard.y() << ", " << filteredBoard.z() << "] m"
                  << " safety=" << reason;
        if (targetSafe) {
          std::cout << " sigma_min=" << safety.minSingular
                    << " cond=" << safety.condition;
        }
        std::cout << '\n';
      }

      if (log) {
        std::string csvReason = reason;
        std::replace(csvReason.begin(), csvReason.end(), ',', ';');
        const double elapsed =
            std::chrono::duration<double>(Clock::now() - runStart).count();
        log << std::fixed << std::setprecision(6) << elapsed << ','
            << static_cast<int>(detected) << ','
            << static_cast<int>(measurementValid) << ','
            << static_cast<int>(targetSafe) << ','
            << static_cast<int>(following) << ',';
        if (measurementValid) {
          log << filteredBoard.x() << ',' << filteredBoard.y() << ','
              << filteredBoard.z() << ',';
        } else {
          log << "nan,nan,nan,";
        }
        if (targetSafe) {
          log << safety.minSingular << ',' << safety.condition << ',';
        } else {
          log << "nan,nan,";
        }
        log << '"' << csvReason << '"' << '\n';
      }

      if (opt.gui) {
        const std::string status = statusText(detected, measurementValid, targetSafe,
                                              following, reason);
        const cv::Scalar colorText = targetSafe ? cv::Scalar(0, 220, 0)
                                                : cv::Scalar(0, 0, 255);
        cv::putText(display, status, cv::Point(18, 35), cv::FONT_HERSHEY_SIMPLEX,
                    0.65, colorText, 2);
        if (measurementValid) {
          cv::putText(display,
                      "distance: " + std::to_string(filteredBoard.z()).substr(0, 5) +
                          " m / target: " + std::to_string(opt.distanceM).substr(0, 4),
                      cv::Point(18, 68), cv::FONT_HERSHEY_SIMPLEX, 0.55,
                      cv::Scalar(255, 255, 255), 1);
        }
        cv::putText(display, "[Q] stop", cv::Point(18, cfg::kColorHeight - 18),
                    cv::FONT_HERSHEY_SIMPLEX, 0.55, cv::Scalar(255, 255, 255), 1);
        cv::imshow("XMC18 Visual Servo", display);
        const int key = cv::waitKey(1);
        if (key == 'q' || key == 'Q' || key == 27) break;
      }
    }
  } catch (...) {
    if (following) follower->stop();
    cleanupRobot(robot, robotConfigured, stateReceiving);
    pipe.stop();
    cv::destroyAllWindows();
    throw;
  }

  if (following) follower->stop();
  cleanupRobot(robot, robotConfigured, stateReceiving);
  pipe.stop();
  cv::destroyAllWindows();
  std::cout << "[OK] visual servo stopped safely\n";
  return 0;
}

}  // namespace

int main(int argc, char** argv) {
  std::signal(SIGINT, onSignal);
  std::signal(SIGTERM, onSignal);
  Options options;
  if (!parseArgs(argc, argv, &options)) {
    printUsage(argv[0]);
    return 2;
  }
  if (options.calibrationFile.empty()) {
    options.calibrationFile = options.identifyAxes
                                  ? legacyCalibrationFile(options.arm)
                                  : defaultCalibrationFile(options.arm);
  }
  if (options.listDevices) return listRealSenseDevices();
  if (options.showToolset) return showControllerToolset(options);
  if (options.selfTest) return runSelfTest();
  if (options.safetyScan) {
    try {
      return runSafetyScan(options);
    } catch (const std::exception& e) {
      std::cerr << "[FATAL] " << e.what() << '\n';
      return 1;
    }
  }
  if (options.moveReady) {
    try {
      return runMoveReady(options);
    } catch (const std::exception& e) {
      std::cerr << "[FATAL] " << e.what() << '\n';
      return 1;
    }
  }
  if (options.identifyAxes) {
    try {
      return runIdentifyAxes(options);
    } catch (const std::exception& e) {
      std::cerr << "[FATAL] " << e.what() << '\n';
      return 1;
    }
  }
  try {
    return run(options);
  } catch (const std::exception& e) {
    std::cerr << "[FATAL] " << e.what() << '\n';
    return 1;
  }
}
