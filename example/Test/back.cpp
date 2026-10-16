#include <chrono>
#include <cmath>
#include <iostream>
#include <string>
#include <thread>
#include <vector>

#include "rokae/robot.h"

using namespace rokae;

static const char* operationStateName(OperationState state) {
  switch (state) {
    case OperationState::idle: return "idle";
    case OperationState::jog: return "jog";
    case OperationState::rtControlling: return "rtControlling";
    case OperationState::drag: return "drag";
    case OperationState::rlProgram: return "rlProgram";
    case OperationState::moving: return "moving";
    case OperationState::jogging: return "jogging";
    case OperationState::unknown: return "unknown";
    default: return "other";
  }
}

static bool checkEc(const std::string& label, const std::error_code& ec) {
  if (ec) {
    std::cerr << "[ERROR] " << label << ": " << ec.message()
              << " (" << ec.value() << ")" << std::endl;
    return false;
  }
  std::cout << "[OK] " << label << std::endl;
  return true;
}

static bool waitRobotIdle(BaseRobot& robot, const std::string& name, int timeoutSeconds) {
  auto start = std::chrono::steady_clock::now();
  while (true) {
    std::this_thread::sleep_for(std::chrono::milliseconds(200));
    std::error_code ec;
    auto state = robot.operationState(ec);
    if (ec) {
      std::cerr << "[ERROR] " << name << " operationState: " << ec.message()
                << " (" << ec.value() << ")" << std::endl;
      return false;
    }

    std::cout << "[" << name << "] state=" << operationStateName(state) << std::endl;
    if (state == OperationState::idle || state == OperationState::unknown) {
      return true;
    }

    if (std::chrono::steady_clock::now() - start > std::chrono::seconds(timeoutSeconds)) {
      std::cerr << "[ERROR] " << name << " wait idle timeout after "
                << timeoutSeconds << "s" << std::endl;
      return false;
    }
  }
}

static bool prepareRobot(xMateRobot& robot, const std::string& name) {
  std::error_code ec;

  auto info = robot.robotInfo(ec);
  if (!checkEc(name + " robotInfo", ec)) return false;
  std::cout << "[" << name << "] version=" << info.version
            << ", type=" << info.type
            << ", joints=" << info.joint_num << std::endl;

  robot.setOperateMode(OperateMode::automatic, ec);
  if (!checkEc(name + " setOperateMode automatic", ec)) return false;

  robot.setPowerState(true, ec);
  if (!checkEc(name + " setPowerState on", ec)) return false;

  robot.setMotionControlMode(MotionControlMode::NrtCommand, ec);
  if (!checkEc(name + " setMotionControlMode NrtCommand", ec)) return false;

  robot.moveReset(ec);
  if (!checkEc(name + " moveReset", ec)) return false;

  robot.setDefaultSpeed(100.0, ec);
  if (!checkEc(name + " setDefaultSpeed 100", ec)) return false;

  robot.setDefaultZone(0, ec);
  if (!checkEc(name + " setDefaultZone 0", ec)) return false;

  return true;
}

static bool sendHome(xMateRobot& robot,
                     const std::string& name,
                     const std::vector<double>& target) {
  std::error_code ec;
  MoveAbsJCommand cmd(target, 100.0, 0.0);
  robot.executeCommand({cmd}, ec);
  if (!checkEc(name + " execute home MoveAbsJ", ec)) return false;
  return true;
}

static void cleanupRobot(xMateRobot& robot, const std::string& name) {
  std::error_code ec;
  robot.setPowerState(false, ec);
  checkEc(name + " setPowerState off", ec);

  robot.setOperateMode(OperateMode::manual, ec);
  checkEc(name + " setOperateMode manual", ec);

  robot.disconnectFromRobot(ec);
  checkEc(name + " disconnect", ec);
}

int main() {
  const std::string localIp = "192.168.2.6";
  // const std::vector<double> leftHome = {0, -M_PI / 2, -M_PI / 2, -M_PI, 0, M_PI * 0.75};
  // const std::vector<double> rightHome = {M_PI, M_PI / 2, M_PI / 2, -M_PI, 0, -M_PI * 1.25};

  const std::vector<double> leftHome = {0, -M_PI / 2, 0, -M_PI, 0, M_PI * 0.75};
  const std::vector<double> rightHome = {M_PI, M_PI / 2, 0, -M_PI, 0, -M_PI * 1.25};

  std::cout << "Back home with xCoreSDK " << BaseRobot::sdkVersion() << std::endl;

  try {
    xMateRobot left("192.168.2.161", localIp);
    xMateRobot right("192.168.2.160", localIp);

    if (!prepareRobot(left, "LeftArm")) {
      cleanupRobot(left, "LeftArm");
      return 1;
    }
    if (!prepareRobot(right, "RightArm")) {
      cleanupRobot(left, "LeftArm");
      cleanupRobot(right, "RightArm");
      return 1;
    }

    bool leftSent = sendHome(left, "LeftArm", leftHome);
    bool rightSent = sendHome(right, "RightArm", rightHome);
    if (!leftSent || !rightSent) {
      cleanupRobot(left, "LeftArm");
      cleanupRobot(right, "RightArm");
      return 1;
    }

    bool leftIdle = waitRobotIdle(left, "LeftArm", 90);
    bool rightIdle = waitRobotIdle(right, "RightArm", 90);

    cleanupRobot(left, "LeftArm");
    cleanupRobot(right, "RightArm");

    return (leftIdle && rightIdle) ? 0 : 1;
  } catch (const std::exception& e) {
    std::cerr << "[EXCEPTION] " << e.what() << std::endl;
    return 1;
  }
}
