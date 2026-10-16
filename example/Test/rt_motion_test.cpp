#include <array>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdlib>
#include <exception>
#include <iostream>
#include <string>
#include <system_error>
#include <thread>
#include <vector>

#include "rokae/robot.h"

using namespace rokae;

namespace {

volatile std::sig_atomic_t g_stop = 0;

void onSignal(int) {
  g_stop = 1;
}

bool checkEc(const std::string &label, const std::error_code &ec) {
  if (ec) {
    std::cerr << "[ERROR] " << label << ": " << ec.message()
              << " (" << ec.value() << ")" << std::endl;
    return false;
  }
  std::cout << "[OK] " << label << std::endl;
  return true;
}

JointPosition makeJointPosition(const std::array<double, 6> &joints) {
  return JointPosition(std::vector<double>(joints.begin(), joints.end()));
}

struct Options {
  std::string robotIp = "192.168.2.160";
  std::string localIp = "192.168.2.6";
  bool both = false;
  double durationSec = 5.0;
  double amplitudeRad = 0.01;
  double frequencyHz = 0.2;
  int jointIndex = 0;
  int priority = 80;
};

void printUsage(const char *program) {
  std::cout
      << "Usage: " << program << " [options]\n"
      << "Options:\n"
      << "  --left                 use left arm  192.168.2.161\n"
      << "  --right                use right arm 192.168.2.160, default\n"
      << "  --both                 test right arm first, then left arm\n"
      << "  --ip <robot_ip>        robot controller IP\n"
      << "  --local <local_ip>     local network IP, default 192.168.2.6\n"
      << "  --duration <seconds>   test duration, default 5\n"
      << "  --amp <radian>         sine amplitude, default 0.01\n"
      << "  --freq <hz>            sine frequency, default 0.2\n"
      << "  --joint <0-5>          joint index to test, default 0\n"
      << "  --priority <0-99>      loop priority hint, default 80; 0 disables\n"
      << "  --help                 show this help\n";
}

bool parseArgs(int argc, char **argv, Options *opt) {
  for (int i = 1; i < argc; ++i) {
    const std::string arg = argv[i];
    auto needValue = [&](const char *name) -> const char * {
      if (i + 1 >= argc) {
        std::cerr << "[ERROR] missing value for " << name << std::endl;
        return nullptr;
      }
      return argv[++i];
    };

    if (arg == "--left") {
      opt->robotIp = "192.168.2.161";
      opt->both = false;
    } else if (arg == "--right") {
      opt->robotIp = "192.168.2.160";
      opt->both = false;
    } else if (arg == "--both") {
      opt->both = true;
    } else if (arg == "--ip") {
      const char *value = needValue("--ip");
      if (!value) return false;
      opt->robotIp = value;
      opt->both = false;
    } else if (arg == "--local") {
      const char *value = needValue("--local");
      if (!value) return false;
      opt->localIp = value;
    } else if (arg == "--duration") {
      const char *value = needValue("--duration");
      if (!value) return false;
      opt->durationSec = std::atof(value);
    } else if (arg == "--amp") {
      const char *value = needValue("--amp");
      if (!value) return false;
      opt->amplitudeRad = std::atof(value);
    } else if (arg == "--freq") {
      const char *value = needValue("--freq");
      if (!value) return false;
      opt->frequencyHz = std::atof(value);
    } else if (arg == "--joint") {
      const char *value = needValue("--joint");
      if (!value) return false;
      opt->jointIndex = std::atoi(value);
    } else if (arg == "--priority") {
      const char *value = needValue("--priority");
      if (!value) return false;
      opt->priority = std::atoi(value);
    } else if (arg == "--help" || arg == "-h") {
      printUsage(argv[0]);
      std::exit(0);
    } else {
      std::cerr << "[ERROR] unknown option: " << arg << std::endl;
      return false;
    }
  }

  if (opt->durationSec <= 0 || opt->durationSec > 60) {
    std::cerr << "[ERROR] duration must be in (0, 60]" << std::endl;
    return false;
  }
  if (opt->amplitudeRad < 0 || opt->amplitudeRad > 0.05) {
    std::cerr << "[ERROR] amp must be in [0, 0.05] rad" << std::endl;
    return false;
  }
  if (opt->frequencyHz <= 0 || opt->frequencyHz > 1.0) {
    std::cerr << "[ERROR] freq must be in (0, 1.0] Hz" << std::endl;
    return false;
  }
  if (opt->jointIndex < 0 || opt->jointIndex >= 6) {
    std::cerr << "[ERROR] joint must be 0..5" << std::endl;
    return false;
  }
  if (opt->priority < 0 || opt->priority > 99) {
    std::cerr << "[ERROR] priority must be 0..99" << std::endl;
    return false;
  }

  return true;
}

void cleanup(xMateRobot &robot) {
  std::error_code ec;
  robot.setMotionControlMode(MotionControlMode::Idle, ec);
  checkEc("setMotionControlMode Idle", ec);
  robot.setPowerState(false, ec);
  checkEc("setPowerState off", ec);
  robot.setOperateMode(OperateMode::manual, ec);
  checkEc("setOperateMode manual", ec);
  robot.disconnectFromRobot(ec);
  checkEc("disconnect", ec);
}

int runSingleArm(const Options &opt, const std::string &name, const std::string &robotIp) {
  std::cout << "\n========== " << name << " realtime test ==========\n";
  std::cout << "robotIp=" << robotIp << ", localIp=" << opt.localIp
            << ", duration=" << opt.durationSec << "s"
            << ", amp=" << opt.amplitudeRad << "rad"
            << ", freq=" << opt.frequencyHz << "Hz"
            << ", joint=" << opt.jointIndex
            << ", priority=" << opt.priority << std::endl;

  xMateRobot robot;
  std::error_code ec;

  try {
    robot.connectToRobot(robotIp, opt.localIp);

    const auto info = robot.robotInfo(ec);
    if (!checkEc(name + " robotInfo", ec)) return 1;
    std::cout << "[" << name << "] controller=" << info.version
              << ", type=" << info.type
              << ", joints=" << info.joint_num << std::endl;
    if (info.joint_num != 6) {
      std::cerr << "[ERROR] " << name
                << " this test expects a 6-axis xMate robot" << std::endl;
      cleanup(robot);
      return 1;
    }

    robot.setOperateMode(OperateMode::automatic, ec);
    if (!checkEc(name + " setOperateMode automatic", ec)) return 1;
    robot.setMotionControlMode(MotionControlMode::RtCommand, ec);
    if (!checkEc(name + " setMotionControlMode RtCommand", ec)) return 1;
    robot.setPowerState(true, ec);
    if (!checkEc(name + " setPowerState on", ec)) return 1;

    auto rtCon = robot.getRtMotionController().lock();
    if (!rtCon) {
      std::cerr << "[ERROR] " << name
                << " getRtMotionController returned null" << std::endl;
      cleanup(robot);
      return 1;
    }

    robot.startReceiveRobotState(std::chrono::milliseconds(1),
                                 {RtSupportedFields::jointPos_m});
    std::cout << "[OK] " << name
              << " startReceiveRobotState 1ms jointPos_m" << std::endl;

    const std::array<double, 6> start = robot.jointPos(ec);
    if (!checkEc(name + " read start jointPos", ec)) {
      cleanup(robot);
      return 1;
    }

    std::cout << "[" << name << "] start joints:";
    for (double q : start) std::cout << ' ' << q;
    std::cout << std::endl;

    double t = 0.0;
    int cycleCount = 0;
    const double omega = 2.0 * M_PI * opt.frequencyHz;

    std::function<JointPosition()> callback = [&]() {
      t += 0.001;
      ++cycleCount;

      std::array<double, 6> target = start;
      target[opt.jointIndex] += opt.amplitudeRad * std::sin(omega * t);

      JointPosition cmd = makeJointPosition(target);
      if (g_stop || t >= opt.durationSec) {
        cmd = makeJointPosition(start);
        cmd.setFinished();
      }
      return cmd;
    };

    rtCon->setControlLoop(callback, opt.priority, false);
    rtCon->startMove(RtControllerMode::jointPosition);
    std::cout << "[OK] " << name << " startMove jointPosition" << std::endl;

    const auto wallStart = std::chrono::steady_clock::now();
    rtCon->startLoop(true);
    const auto wallEnd = std::chrono::steady_clock::now();
    const double wallSec =
        std::chrono::duration<double>(wallEnd - wallStart).count();

    std::cout << "[OK] " << name << " realtime loop finished" << std::endl;
    std::cout << "[" << name << "] cycles=" << cycleCount
              << ", planned_time=" << t
              << "s, wall_time=" << wallSec
              << "s, expected_cycles=" << static_cast<int>(opt.durationSec * 1000)
              << std::endl;

    robot.stopReceiveRobotState();
    cleanup(robot);
    return 0;
  } catch (const std::exception &e) {
    std::cerr << "[EXCEPTION] " << name << ": " << e.what() << std::endl;
    try {
      cleanup(robot);
    } catch (...) {
    }
    return 1;
  }
}

}  // namespace

int main(int argc, char **argv) {
  std::signal(SIGINT, onSignal);
  std::signal(SIGTERM, onSignal);

  Options opt;
  if (!parseArgs(argc, argv, &opt)) {
    printUsage(argv[0]);
    return 2;
  }

  std::cout << "xCoreSDK version: " << BaseRobot::sdkVersion() << std::endl;

  if (opt.both) {
    const int right = runSingleArm(opt, "RightArm", "192.168.2.160");
    if (g_stop || right != 0) return right;

    std::this_thread::sleep_for(std::chrono::seconds(1));

    const int left = runSingleArm(opt, "LeftArm", "192.168.2.161");
    return left;
  }

  const std::string name = (opt.robotIp == "192.168.2.161") ? "LeftArm" : "RightArm";
  return runSingleArm(opt, name, opt.robotIp);
}
