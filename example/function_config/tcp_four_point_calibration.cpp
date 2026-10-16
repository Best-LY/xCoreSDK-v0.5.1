#include <array>
#include <cmath>
#include <iostream>
#include <limits>
#include <string>
#include <system_error>
#include <vector>

#include <opencv2/core.hpp>

#include "robot_vision_config.hpp"
#include "rokae/robot.h"
#include "rokae/utility.h"

namespace {

bool check(const std::string& action, const std::error_code& ec) {
  if (!ec) return true;
  std::cerr << "[ERROR] " << action << ": " << ec.message()
            << " (" << ec.value() << ")\n";
  return false;
}

cv::Mat frameMatrix(const rokae::Frame& frame) {
  Eigen::Matrix3d rotation;
  rokae::Utils::eulerToMatrix(
      Eigen::Vector3d(frame.rpy[2], frame.rpy[1], frame.rpy[0]), rotation);
  cv::Mat matrix = cv::Mat::eye(4, 4, CV_64F);
  for (int r = 0; r < 3; ++r) {
    matrix.at<double>(r, 3) = frame.trans[r];
    for (int c = 0; c < 3; ++c) matrix.at<double>(r, c) = rotation(r, c);
  }
  return matrix;
}

}  // namespace

int main(int argc, char** argv) {
  bool apply = false;
  for (int i = 1; i < argc; ++i) {
    const std::string arg = argv[i];
    if (arg == "--apply") {
      apply = true;
    } else if (arg == "--help" || arg == "-h") {
      std::cout << "Usage: " << argv[0] << " [--apply]\n"
                << "Four-point held-tool TCP calibration. By default the result "
                   "is saved only; --apply also sets the controller toolset.\n";
      return 0;
    } else {
      std::cerr << "[ERROR] unknown option: " << arg << '\n';
      return 2;
    }
  }

  std::cout << "1. Left arm\n2. Right arm\nSelect: ";
  int selection = 0;
  if (!(std::cin >> selection) || (selection != 1 && selection != 2)) {
    std::cerr << "[ERROR] invalid arm selection\n";
    return 2;
  }
  std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');

  const bool left = selection == 1;
  const std::string armName = left ? "LeftArm" : "RightArm";
  const std::string robotIp = left ? user_config::IP_LEFT : user_config::IP_RIGHT;
  const std::string output = armName + "_tcp_four_point.yml";

  try {
    rokae::xMateRobot robot(robotIp, user_config::LOCAL_IP);
    std::error_code ec;
    const auto info = robot.robotInfo(ec);
    if (!check("robotInfo", ec)) return 2;
    const auto originalToolset = robot.toolset(ec);
    if (!check("read existing toolset", ec)) return 2;
    std::cout << "[INFO] connected " << info.type << " at " << robotIp << '\n'
              << "[IMPORTANT] Use low-speed manual jog. Keep the intended gripper "
                 "grasp center on one fixed physical reference point.\n"
              << "[IMPORTANT] Change wrist orientation substantially between the "
                 "four samples without moving that TCP point.\n";

    std::vector<std::array<double, 6>> points;
    points.reserve(4);
    for (int i = 0; i < 4; ++i) {
      std::cout << "Position sample " << (i + 1)
                << "/4, then press Enter..." << std::flush;
      std::string line;
      std::getline(std::cin, line);
      const auto joints = robot.jointPos(ec);
      if (!check("read joints", ec)) return 2;
      points.push_back(joints);
      std::cout << " captured\n";
    }

    const auto result = robot.calibrateFrame(
        rokae::FrameType::tool, points, true, ec);
    if (!check("four-point tool calibration", ec)) return 2;
    const cv::Mat transform = frameMatrix(result.frame);
    cv::FileStorage storage(output, cv::FileStorage::WRITE);
    storage << "H_Flange2Tcp" << transform;
    storage << "arm" << armName;
    storage << "robot_ip" << robotIp;
    storage << "method" << "four_point_held_tool";
    storage << "error_min_m" << result.errors[0];
    storage << "error_mean_m" << result.errors[1];
    storage << "error_max_m" << result.errors[2];
    storage << "controller_applied" << static_cast<int>(apply);
    storage.release();

    std::cout << "[RESULT] flange_T_tcp=\n" << transform << '\n'
              << "[RESULT] error min/mean/max=[" << result.errors[0] << ", "
              << result.errors[1] << ", " << result.errors[2] << "] m\n"
              << "[OK] saved " << output << '\n';

    if (apply) {
      rokae::Toolset calibrated = originalToolset;
      calibrated.end = result.frame;
      robot.setToolset(calibrated, ec);
      if (!check("apply calibrated TCP", ec)) return 2;
      std::cout << "[OK] calibrated TCP applied; existing load and reference "
                   "frame were preserved\n";
    } else {
      std::cout << "[SAFE] controller unchanged; review the result before --apply\n";
    }
    return 0;
  } catch (const std::exception& e) {
    std::cerr << "[FATAL] " << e.what() << '\n';
    return 1;
  }
}
