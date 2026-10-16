#include <chrono>
#include <array>
#include <cmath>
#include <cstdlib>
#include <cstdint>
#include <exception>
#include <iomanip>
#include <iostream>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>

#include <librealsense2/rs.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include "robot_vision_config.hpp"
#include "rokae/robot.h"
#include "rokae/utility.h"

namespace {

constexpr const char* kRightCameraSerial = "236422070786";

void checkRobot(const std::error_code& ec, const std::string& action) {
  if (ec) throw std::runtime_error(action + ": " + ec.message());
}

std::array<double, 6> readStationaryJoints(rokae::xMateRobot& robot) {
  std::error_code ec;
  const auto state = robot.operationState(ec);
  checkRobot(ec, "read operation state");
  if (state != rokae::OperationState::idle) {
    throw std::runtime_error("robot must be idle for calibration validation");
  }
  const auto q = robot.jointPos(ec);
  checkRobot(ec, "read joints");
  return q;
}

void requireUnchanged(const std::array<double, 6>& reference,
                      const std::array<double, 6>& q) {
  for (size_t i = 0; i < q.size(); ++i) {
    if (!std::isfinite(q[i]) || !std::isfinite(reference[i]) ||
        std::abs(q[i] - reference[i]) > 0.0005) {
      std::ostringstream message;
      message << std::setprecision(12)
              << "robot moved during capture; discard this burst: J" << i + 1
              << " reference=" << reference[i] << " current=" << q[i]
              << " delta_rad=" << q[i] - reference[i]
              << " allowed_abs_delta_rad=0.0005";
      throw std::runtime_error(message.str());
    }
  }
}

}  // namespace

int main(int argc, char** argv) {
  std::string serial = kRightCameraSerial;
  std::string prefix = "/tmp/xmc_right_joint_snapshot";
  bool vga = false;
  int burst = 1;
  std::string robotArm;
  bool explicitSerial = false;
  for (int i = 1; i < argc; ++i) {
    const std::string arg = argv[i];
    if (arg == "--camera" && i + 1 < argc) {
      serial = argv[++i];
      explicitSerial = true;
    } else if (arg == "--robot-arm" && i + 1 < argc) {
      robotArm = argv[++i];
    } else if (arg == "--output-prefix" && i + 1 < argc) {
      prefix = argv[++i];
    } else if (arg == "--vga") {
      vga = true;
    } else if (arg == "--burst" && i + 1 < argc) {
      burst = std::atoi(argv[++i]);
    } else if (arg == "--help") {
      std::cout << "Usage: " << argv[0]
                << " [--camera SERIAL] [--output-prefix PATH] [--vga]"
                   " [--burst 1..60] [--robot-arm left|right]\n"
                   "--robot-arm records stationary flange poses, read-only.\n";
      return 0;
    } else {
      std::cerr << "[ERROR] invalid or incomplete argument: " << arg << '\n';
      return 2;
    }
  }
  if (burst < 1 || burst > 60) {
    std::cerr << "[ERROR] --burst must be between 1 and 60\n";
    return 2;
  }
  if (!robotArm.empty()) {
    if (robotArm != "left" && robotArm != "right") {
      std::cerr << "[ERROR] --robot-arm must be left or right\n";
      return 2;
    }
    const auto expected = robotArm == "left" ? user_config::SN_LEFT_CAM
                                             : user_config::SN_RIGHT_CAM;
    if (explicitSerial && serial != expected) {
      std::cerr << "[ERROR] camera serial does not match selected robot arm\n";
      return 2;
    }
    serial = expected;
  }

  try {
    std::unique_ptr<rokae::xMateRobot> robot;
    std::array<double, 6> referenceJoints{};
    if (!robotArm.empty()) {
      robot = std::make_unique<rokae::xMateRobot>(
          robotArm == "left" ? user_config::IP_LEFT : user_config::IP_RIGHT,
          user_config::LOCAL_IP);
      referenceJoints = readStationaryJoints(*robot);
    }
    const int colorWidth = vga ? 640 : 1280;
    const int colorHeight = vga ? 480 : 720;
    rs2::pipeline pipe;
    rs2::config config;
    config.enable_device(serial);
    config.enable_stream(RS2_STREAM_COLOR, colorWidth, colorHeight,
                         RS2_FORMAT_BGR8, 15);
    config.enable_stream(RS2_STREAM_DEPTH, 640, 480, RS2_FORMAT_Z16, 15);
    const auto profile = pipe.start(config);
    rs2::align alignToColor(RS2_STREAM_COLOR);
    rs2::frameset frames;
    for (int i = 0; i < 30; ++i) {
      frames = alignToColor.process(pipe.wait_for_frames(5000));
    }

    for (int sample = 0; sample < burst; ++sample) {
      cv::Mat baseToFlange;
      std::array<double, 6> captureJoints{};
      if (robot) {
        requireUnchanged(referenceJoints, readStationaryJoints(*robot));
        // Discard a queued frame before bracketing a fresh image with joint reads.
        frames = alignToColor.process(pipe.wait_for_frames(5000));
        frames = alignToColor.process(pipe.wait_for_frames(5000));
        std::error_code ec;
        const auto flange = robot->cartPosture(rokae::CoordinateType::flangeInBase, ec);
        checkRobot(ec, "read flange in base");
        captureJoints = readStationaryJoints(*robot);
        requireUnchanged(referenceJoints, captureJoints);
        const std::array<double, 6> posture{{flange.trans[0], flange.trans[1],
            flange.trans[2], flange.rpy[0], flange.rpy[1], flange.rpy[2]}};
        std::array<double, 16> transform{};
        rokae::Utils::postureToTransArray(posture, transform);
        baseToFlange = cv::Mat(4, 4, CV_64F, transform.data()).clone();
        if (!cv::checkRange(baseToFlange)) throw std::runtime_error("invalid flange pose");
      } else if (sample) {
        frames = alignToColor.process(pipe.wait_for_frames(5000));
      }
      const auto capturedMs = std::chrono::duration_cast<std::chrono::milliseconds>(
          std::chrono::system_clock::now().time_since_epoch()).count();
      const auto color = frames.get_color_frame();
      const auto depth = frames.get_depth_frame();
      if (!color || !depth || color.get_width() != depth.get_width() ||
          color.get_height() != depth.get_height()) {
        throw std::runtime_error("missing or unaligned color/depth frame");
      }
      std::ostringstream name;
      name << prefix;
      if (burst > 1) name << '_' << std::setw(3) << std::setfill('0') << sample;
      const std::string samplePrefix = name.str();
      const std::string colorPath = samplePrefix + "_color.png";
      const std::string depthPath = samplePrefix + "_depth.png";
      const std::string metadataPath = samplePrefix + ".yml";
      const cv::Mat colorImage(color.get_height(), color.get_width(), CV_8UC3,
                               const_cast<void*>(color.get_data()), cv::Mat::AUTO_STEP);
      const cv::Mat depthImage(depth.get_height(), depth.get_width(), CV_16UC1,
                               const_cast<void*>(depth.get_data()), cv::Mat::AUTO_STEP);
      if (!cv::imwrite(colorPath, colorImage) ||
          !cv::imwrite(depthPath, depthImage)) {
        throw std::runtime_error("cannot write snapshot images");
      }

      const auto intrinsics = color.get_profile()
                                  .as<rs2::video_stream_profile>()
                                  .get_intrinsics();
      cv::FileStorage metadata(metadataPath, cv::FileStorage::WRITE);
      if (!metadata.isOpened()) {
        throw std::runtime_error("cannot write snapshot metadata");
      }
      metadata << "camera_serial" << serial;
      if (robot) {
        metadata << "robot_arm" << robotArm;
        metadata << "base_T_flange" << baseToFlange;
        metadata << "joint_positions_rad"
                 << cv::Mat(1, 6, CV_64F, captureJoints.data()).clone();
        metadata << "stationary_joint_tolerance_rad" << 0.0005;
        metadata << "robot_motion_permitted" << 0;
      }
      metadata << "color_file" << colorPath;
      metadata << "depth_file" << depthPath;
      metadata << "width" << color.get_width();
      metadata << "height" << color.get_height();
      metadata << "fx" << intrinsics.fx;
      metadata << "fy" << intrinsics.fy;
      metadata << "ppx" << intrinsics.ppx;
      metadata << "ppy" << intrinsics.ppy;
      metadata << "distortion_model" << static_cast<int>(intrinsics.model);
      cv::Mat distortion(1, 5, CV_64F);
      for (int i = 0; i < 5; ++i) {
        distortion.at<double>(0, i) = intrinsics.coeffs[i];
      }
      metadata << "distortion_coeffs" << distortion;
      metadata << "depth_scale_m" << depth.get_units();
      // Preserve milliseconds beyond FileStorage's 32-bit integer range.
      metadata << "capture_unix_ms" << std::to_string(capturedMs);
      metadata.release();
    }
    pipe.stop();
    std::cout << "[OK] camera=" << serial << " frames=" << burst
              << " prefix=" << prefix << " size=" << colorWidth << 'x'
              << colorHeight << '\n';
    std::cout << (robot ? "[SAFE] read-only robot poses; no mode/power/motion commands\n"
                        : "[SAFE] snapshot only; no robot connection or motion\n");
    return 0;
  } catch (const std::exception& error) {
    std::cerr << "[ERROR] snapshot failed: " << error.what() << '\n';
    return 2;
  }
}
