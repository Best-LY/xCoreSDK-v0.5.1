#include <iostream>
#include <vector>
#include <string>
#include <thread>
#include <chrono>
#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <csignal>

#include <opencv2/opencv.hpp>
#include <opencv2/calib3d.hpp>

#include <librealsense2/rs.hpp>
#include <librealsense2/rsutil.h>

#include <Eigen/Dense>

#include "rokae/robot.h"
#include "rokae/utility.h"
#include "rokae/data_types.h"
#include "robot_vision_config.hpp"

using namespace std;
using namespace cv;
using namespace rokae;
using namespace user_config;

static volatile sig_atomic_t stopRequested = 0;

static void onSignal(int) {
  stopRequested = 1;
}

static Mat rpyToRotationMatrix(const array<double, 3>& rpy) {
  Eigen::Matrix3d rot;
  Eigen::Vector3d euler(rpy[2], rpy[1], rpy[0]);
  Utils::eulerToMatrix(euler, rot);

  Mat R(3, 3, CV_64F);
  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 3; ++j) {
      R.at<double>(i, j) = rot(i, j);
    }
  }
  return R;
}

static Mat getIntrinsics(const rs2::video_stream_profile& profile, Mat& distCoeffs) {
  auto intrinsics = profile.get_intrinsics();
  Mat K = Mat::eye(3, 3, CV_64F);
  K.at<double>(0, 0) = intrinsics.fx;
  K.at<double>(1, 1) = intrinsics.fy;
  K.at<double>(0, 2) = intrinsics.ppx;
  K.at<double>(1, 2) = intrinsics.ppy;

  distCoeffs = Mat::zeros(5, 1, CV_64F);
  for (int i = 0; i < 5; ++i) {
    distCoeffs.at<double>(i) = intrinsics.coeffs[i];
  }
  return K;
}

static bool deviceExists(const string& serial) {
  if (serial.empty()) return false;

  rs2::context ctx;
  for (auto&& dev : ctx.query_devices()) {
    if (dev.supports(RS2_CAMERA_INFO_SERIAL_NUMBER) &&
        serial == dev.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER)) {
      return true;
    }
  }
  return false;
}

static void printRealSenseDevices() {
  rs2::context ctx;
  auto devices = ctx.query_devices();
  cout << "Detected RealSense devices: " << devices.size() << endl;
  for (auto&& dev : devices) {
    cout << "  - "
         << dev.get_info(RS2_CAMERA_INFO_NAME)
         << ", serial="
         << dev.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER)
         << endl;
  }
}

static bool startCamera(rs2::pipeline& pipe,
                        const string& preferredSerial,
                        Mat& K,
                        Mat& distCoeffs,
                        int& width,
                        int& height,
                        rs2_intrinsics& colorIntrinsics) {
  printRealSenseDevices();

  rs2::config cfg;
  if (!deviceExists(preferredSerial)) {
    cerr << "Configured camera serial not found: " << preferredSerial << endl;
    return false;
  }
  cfg.enable_device(preferredSerial);
  cout << "Using camera serial: " << preferredSerial << endl;

  cfg.enable_stream(RS2_STREAM_COLOR, 640, 480, RS2_FORMAT_BGR8, 15);
  cfg.enable_stream(RS2_STREAM_DEPTH, 640, 480, RS2_FORMAT_Z16, 15);

  try {
    auto profile = pipe.start(cfg);
    for (int i = 0; i < 10; ++i) {
      pipe.wait_for_frames(10000);
    }

    auto videoProfile = profile.get_stream(RS2_STREAM_COLOR).as<rs2::video_stream_profile>();
    colorIntrinsics = videoProfile.get_intrinsics();
    width = videoProfile.width();
    height = videoProfile.height();
    K = getIntrinsics(videoProfile, distCoeffs);

    cout << "Camera started: " << width << "x" << height << "@"
         << videoProfile.fps() << endl;
    return true;
  } catch (const rs2::error& e) {
    cerr << "Failed to start RealSense camera: " << e.what() << endl;
  } catch (const exception& e) {
    cerr << "Failed to start camera: " << e.what() << endl;
  }
  return false;
}

static double median(vector<double> values) {
  if (values.empty()) return numeric_limits<double>::quiet_NaN();
  const size_t middle = values.size() / 2;
  nth_element(values.begin(), values.begin() + middle, values.end());
  return values[middle];
}

static double depthAt(const rs2::depth_frame& depth, const Point2f& pixel) {
  vector<double> values;
  const int centerX = static_cast<int>(lround(pixel.x));
  const int centerY = static_cast<int>(lround(pixel.y));
  for (int y = centerY - 2; y <= centerY + 2; ++y) {
    for (int x = centerX - 2; x <= centerX + 2; ++x) {
      if (x < 0 || y < 0 || x >= depth.get_width() || y >= depth.get_height()) continue;
      const double value = depth.get_distance(x, y);
      if (value > 0.10 && value < 2.0 && isfinite(value)) values.push_back(value);
    }
  }
  return median(move(values));
}

static double estimateSquareSize(const vector<Point2f>& corners,
                                 const rs2::depth_frame& depth,
                                 const rs2_intrinsics& intrinsics) {
  if (corners.size() != static_cast<size_t>(BOARD_SIZE.area())) {
    return numeric_limits<double>::quiet_NaN();
  }
  const array<int, 4> indices{{
      2 * BOARD_SIZE.width + 3, 2 * BOARD_SIZE.width + 4,
      3 * BOARD_SIZE.width + 3, 3 * BOARD_SIZE.width + 4}};
  array<Eigen::Vector3d, 4> points{};
  for (size_t i = 0; i < indices.size(); ++i) {
    const Point2f pixel = corners[indices[i]];
    const double depthM = depthAt(depth, pixel);
    if (!isfinite(depthM)) return numeric_limits<double>::quiet_NaN();
    const float uv[2]{pixel.x, pixel.y};
    float xyz[3]{};
    rs2_deproject_pixel_to_point(xyz, &intrinsics, uv,
                                 static_cast<float>(depthM));
    points[i] = Eigen::Vector3d(xyz[0], xyz[1], xyz[2]);
  }
  return median({(points[1] - points[0]).norm(),
                 (points[3] - points[2]).norm(),
                 (points[2] - points[0]).norm(),
                 (points[3] - points[1]).norm()});
}

static bool setupDragMode(xMateRobot& robot) {
  error_code ec;

  robot.setOperateMode(OperateMode::manual, ec);
  if (ec) {
    cerr << "Failed to switch to manual mode: " << ec.message() << endl;
    return false;
  }

  robot.setPowerState(true, ec);
  if (ec) {
    cerr << "Failed to power on robot: " << ec.message() << endl;
    return false;
  }

  robot.enableDrag(DragParameter::Space::cartesianSpace,
                   DragParameter::Type::freely,
                   ec,
                   true);
  if (ec) {
    cerr << "Failed to enable drag mode: " << ec.message() << endl;
    robot.setPowerState(false, ec);
    return false;
  }

  cout << "Drag mode enabled." << endl;
  return true;
}

static void cleanupRobot(xMateRobot& robot) {
  error_code ec;
  robot.disableDrag(ec);
  if (ec) cerr << "Failed to disable drag: " << ec.message() << endl;
  robot.setPowerState(false, ec);
  if (ec) cerr << "Failed to power off: " << ec.message() << endl;
}

static void runInteractiveCalibration(xMateRobot& robot,
                                      const string& cameraSN,
                                      const string& armName,
                                      double squareSizeM) {
  rs2::pipeline pipe;
  Mat K;
  Mat distCoeffs;
  int camWidth = 0;
  int camHeight = 0;
  rs2_intrinsics colorIntrinsics{};

  cout << "[" << armName << "] starting camera..." << endl;
  if (!startCamera(pipe, cameraSN, K, distCoeffs, camWidth, camHeight,
                   colorIntrinsics)) {
    return;
  }

  vector<Point3f> objectPoints;
  for (int r = 0; r < BOARD_SIZE.height; ++r) {
    for (int c = 0; c < BOARD_SIZE.width; ++c) {
      objectPoints.emplace_back(c * squareSizeM, r * squareSizeM, 0.0f);
    }
  }

  cout << "[" << armName << "] enabling drag mode..." << endl;
  if (!setupDragMode(robot)) {
    pipe.stop();
    return;
  }

  cout << endl;
  cout << "Operation:" << endl;
  cout << "  1. Drag the arm until the camera sees the checkerboard." << endl;
  cout << "  2. Change both position and wrist orientation between captures." << endl;
  cout << "  3. Press Space to capture." << endl;
  cout << "  4. Capture at least " << MIN_CAPTURE_COUNT << " points, then press Q or ESC." << endl;

  vector<Mat> R_gripper2base;
  vector<Mat> t_gripper2base;
  vector<Mat> R_target2cam;
  vector<Mat> t_target2cam;
  int captureCount = 0;
  const string winName = "Drag Hand-Eye Calibration - " + armName;
  rs2::align alignToColor(RS2_STREAM_COLOR);
  int consecutiveFrameErrors = 0;

  try {
  while (!stopRequested) {
    rs2::frameset frames;
    try {
      frames = alignToColor.process(pipe.wait_for_frames(10000));
    } catch (const rs2::error& e) {
      cerr << "Frame timeout: " << e.what() << endl;
      if (stopRequested || ++consecutiveFrameErrors >= 3) {
        throw runtime_error("camera stream lost during drag calibration");
      }
      continue;
    }
    consecutiveFrameErrors = 0;

    auto colorFrame = frames.get_color_frame();
    auto depthFrame = frames.get_depth_frame();
    if (!colorFrame || !depthFrame) continue;

    Mat frame(Size(camWidth, camHeight), CV_8UC3,
              const_cast<void*>(colorFrame.get_data()), Mat::AUTO_STEP);
    Mat display = frame.clone();
    Mat gray;
    cvtColor(frame, gray, COLOR_BGR2GRAY);

    vector<Point2f> corners;
    bool found = findChessboardCorners(gray,
                                       BOARD_SIZE,
                                       corners,
                                       CALIB_CB_ADAPTIVE_THRESH | CALIB_CB_NORMALIZE_IMAGE);

    Scalar color = found ? Scalar(0, 255, 0) : Scalar(0, 0, 255);
    if (found) {
      drawChessboardCorners(display, BOARD_SIZE, corners, found);
      putText(display, "READY", Point(20, 40), FONT_HERSHEY_SIMPLEX, 0.9, color, 2);
    } else {
      putText(display, "NO BOARD", Point(20, 40), FONT_HERSHEY_SIMPLEX, 0.9, color, 2);
    }

    putText(display,
            "Points: " + to_string(captureCount),
            Point(20, 80),
            FONT_HERSHEY_SIMPLEX,
            0.8,
            color,
            2);
    putText(display,
            "[Space] Capture   [Q/Esc] Finish",
            Point(20, camHeight - 25),
            FONT_HERSHEY_SIMPLEX,
            0.65,
            Scalar(255, 255, 255),
            2);

    imshow(winName, display);
    int key = waitKey(30);

    if (key == 32) {
      if (!found) {
        cout << "Checkerboard not found, capture skipped." << endl;
        continue;
      }

      cornerSubPix(gray,
                   corners,
                   Size(11, 11),
                   Size(-1, -1),
                   TermCriteria(TermCriteria::EPS | TermCriteria::COUNT, 30, 0.01));

      const double depthSquareM =
          estimateSquareSize(corners, depthFrame, colorIntrinsics);
      if (!isfinite(depthSquareM) ||
          abs(depthSquareM - squareSizeM) > 0.10 * squareSizeM) {
        cout << "Capture skipped: measured square size from depth="
             << depthSquareM * 1000.0 << " mm, configured="
             << squareSizeM * 1000.0
             << " mm. Measure the physical board and restart if needed."
             << endl;
        continue;
      }

      Mat rvec;
      Mat tvec;
      bool pnpOk = solvePnP(objectPoints,
                            corners,
                            K,
                            distCoeffs,
                            rvec,
                            tvec,
                            false,
                            SOLVEPNP_ITERATIVE);
      if (!pnpOk) {
        cout << "solvePnP failed, capture skipped." << endl;
        continue;
      }

      Mat Rvis;
      Rodrigues(rvec, Rvis);

      error_code ec;
      auto pose = robot.cartPosture(CoordinateType::flangeInBase, ec);
      if (ec) {
        cerr << "Failed to read robot pose: " << ec.message() << endl;
        continue;
      }

      R_target2cam.push_back(Rvis);
      t_target2cam.push_back(tvec);
      R_gripper2base.push_back(rpyToRotationMatrix(pose.rpy));
      t_gripper2base.push_back((Mat_<double>(3, 1)
                                << pose.trans[0], pose.trans[1], pose.trans[2]));

      ++captureCount;
      cout << "Captured point " << captureCount
           << ", depth square=" << depthSquareM * 1000.0 << " mm"
           << ", robot trans=[" << pose.trans[0] << ", "
           << pose.trans[1] << ", " << pose.trans[2] << "]" << endl;

      Mat flash = Mat::ones(display.size(), display.type()) * 255;
      imshow(winName, flash);
      waitKey(60);
    } else if (key == 'q' || key == 'Q' || key == 27) {
      break;
    }
  }
  } catch (...) {
    cleanupRobot(robot);
    pipe.stop();
    destroyAllWindows();
    throw;
  }

  cleanupRobot(robot);
  pipe.stop();
  destroyAllWindows();

  if (captureCount < MIN_CAPTURE_COUNT) {
    cerr << "Only " << captureCount << " points captured. Need at least "
         << MIN_CAPTURE_COUNT << " for a stable calibration." << endl;
    return;
  }

  cout << "Calculating hand-eye calibration..." << endl;
  Mat R_cam2gripper;
  Mat t_cam2gripper;
  calibrateHandEye(R_gripper2base,
                   t_gripper2base,
                   R_target2cam,
                   t_target2cam,
                   R_cam2gripper,
                   t_cam2gripper,
                   CALIB_HAND_EYE_TSAI);

  Mat H = Mat::eye(4, 4, CV_64F);
  R_cam2gripper.copyTo(H(Range(0, 3), Range(0, 3)));
  t_cam2gripper.copyTo(H(Range(0, 3), Range(3, 4)));

  string filename = armName + "_drag_hand_eye_051.yml";
  FileStorage fs(filename, FileStorage::WRITE);
  fs << "H_Flange2Cam" << H;
  fs << "R" << R_cam2gripper;
  fs << "t" << t_cam2gripper;
  fs << "camera_serial" << cameraSN;
  fs << "board_width" << BOARD_SIZE.width;
  fs << "board_height" << BOARD_SIZE.height;
  fs << "square_size_m" << squareSizeM;
  fs.release();

  cout << "Calibration done." << endl;
  cout << "H_Flange2Cam:" << endl << H << endl;
  cout << "Saved to: " << filename << endl;
}

int main() {
  signal(SIGINT, onSignal);
  signal(SIGTERM, onSignal);
  try {
    cout << "========================================" << endl;
    cout << "  xCoreSDK 0.5.1 Drag Hand-Eye Calibration" << endl;
    cout << "========================================" << endl;
    cout << "1. Left arm  (" << IP_LEFT << ")" << endl;
    cout << "2. Right arm (" << IP_RIGHT << ")" << endl;
    cout << "Select: ";

    int choice = 0;
    if (!(cin >> choice)) return 1;

    cout << "Measured checkerboard square size in millimeters (current board: "
         << SQUARE_SIZE * 1000.0f << " mm): ";
    double squareSizeMm = 0.0;
    if (!(cin >> squareSizeMm) || !isfinite(squareSizeMm) ||
        squareSizeMm < 5.0 || squareSizeMm > 100.0) {
      cerr << "Invalid square size. Measure one physical square before calibration."
           << endl;
      return 1;
    }
    const double squareSizeM = squareSizeMm / 1000.0;

    if (choice == 1) {
      xMateRobot robot(IP_LEFT, LOCAL_IP);
      runInteractiveCalibration(robot, SN_LEFT_CAM, "LeftArm", squareSizeM);
    } else if (choice == 2) {
      xMateRobot robot(IP_RIGHT, LOCAL_IP);
      runInteractiveCalibration(robot, SN_RIGHT_CAM, "RightArm", squareSizeM);
    } else {
      cerr << "Invalid selection." << endl;
      return 1;
    }
  } catch (const exception& e) {
    cerr << "Fatal error: " << e.what() << endl;
    return 1;
  }

  return 0;
}
