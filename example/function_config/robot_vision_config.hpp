#pragma once

#include <opencv2/core/types.hpp>

#include <string>

namespace user_config {

inline const std::string IP_LEFT = "192.168.2.161";
inline const std::string IP_RIGHT = "192.168.2.160";
inline const std::string LOCAL_IP = "192.168.2.6";

// Physical camera assignment used by the original 0.4.1b calibration files.
inline const std::string SN_LEFT_CAM = "236422072262";
inline const std::string SN_RIGHT_CAM = "236422070786";

inline const cv::Size BOARD_SIZE(8, 6);
inline constexpr float SQUARE_SIZE = 0.0286f; // User-confirmed 2026-09-29.
inline constexpr int MIN_CAPTURE_COUNT = 10;

}  // namespace user_config
