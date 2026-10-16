#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <csignal>
#include <iostream>
#include <memory>
#include <sstream>
#include <string>
#include <thread>
#include <vector>
#include <Eigen/Dense>
#include <opencv2/core.hpp>
#include "robot_vision_config.hpp"
#include "visual_servo_config.hpp"
#include "rokae/model.h"
#include "rokae/robot.h"
#include "rokae/utility.h"

namespace {
using Joints=std::array<double,6>;
volatile std::sig_atomic_t interrupted=0;
void onSignal(int) { interrupted=1; }
void require(bool ok,const std::string& why) { if(!ok) throw std::runtime_error(why); }
void check(const std::error_code& ec) {
  require(!ec,ec.message()+" (code="+std::to_string(ec.value())+")");
}
void stationary(rokae::xMateRobot& robot,const Joints& reference) {
  std::error_code ec;
  const auto state=robot.operationState(ec); check(ec);
  require(state==rokae::OperationState::idle,"robot is not idle");
  const auto q=robot.jointPos(ec); check(ec);
  for(int i=0;i<6;++i) require(std::isfinite(q[i]) && std::abs(q[i]-reference[i])<0.0005,
                              "posture changed during planning");
}
bool cleanup(rokae::xMateRobot& robot) {
  bool ok=true; std::error_code ec;
  const auto report=[&](const char* action) {
    if(ec) { ok=false; std::cerr<<"[CLEANUP] "<<action<<": "<<ec.message()<<'\n'; }
  };
  robot.stop(ec); report("stop");
  robot.setMotionControlMode(rokae::MotionControlMode::Idle,ec); report("Idle");
  robot.setPowerState(false,ec); report("power off");
  robot.setOperateMode(rokae::OperateMode::manual,ec); report("manual");
  return ok;
}
}

int main(int argc,char** argv) {
  std::unique_ptr<rokae::xMateRobot> robot;
  bool configured=false,motionIssued=false;
  try {
    require(argc>=2 && argc<=4,
            "Usage: return_observation SAVED_LEFT_POSE.yml [--nearest-pose] [--execute]");
    bool execute=false,nearest=false;
    for(int i=2;i<argc;++i) {
      const std::string arg=argv[i];
      if(arg=="--execute" && !execute) execute=true;
      else if(arg=="--nearest-pose" && !nearest) nearest=true;
      else throw std::runtime_error("invalid/duplicate argument: "+arg);
    }
    std::signal(SIGINT,onSignal); std::signal(SIGTERM,onSignal);
    cv::FileStorage fs(argv[1],cv::FileStorage::READ);
    require(fs.isOpened(),"saved observation pose missing");
    std::string arm,serial;
    fs["robot_arm"]>>arm; fs["camera_serial"]>>serial;
    require(arm=="left" && serial==user_config::SN_LEFT_CAM,"not a saved left-arm pose");
    cv::Mat savedQ,savedG;
    fs["joint_positions_rad"]>>savedQ; fs["base_T_flange"]>>savedG;
    require(savedQ.type()==CV_64F && savedQ.total()==6 && cv::checkRange(savedQ) &&
            savedG.type()==CV_64F && savedG.rows==4 && savedG.cols==4 && cv::checkRange(savedG),
            "invalid saved pose");
    Joints target{};
    std::copy(savedQ.ptr<double>(),savedQ.ptr<double>()+6,target.begin());
    robot=std::make_unique<rokae::xMateRobot>(user_config::IP_LEFT,user_config::LOCAL_IP);
    std::error_code ec;
    const auto info=robot->robotInfo(ec); check(ec);
    require(info.type.find("XMC18")!=std::string::npos,"unexpected robot type");
    const Joints start=robot->jointPos(ec); check(ec); stationary(*robot,start);
    rokae::xMateRobot right(user_config::IP_RIGHT,user_config::LOCAL_IP);
    const Joints rightStart=right.jointPos(ec); check(ec); stationary(right,rightStart);
    rokae::xMateModel<6> model=robot->model();
    const auto goalMatrix=model.getCartPose(target);
    double matrixError=0;
    for(int i=0;i<16;++i) matrixError=std::max(matrixError,std::abs(goalMatrix[i]-savedG.at<double>(i/4,i%4)));
    require(matrixError<0.005,"saved target differs from current robot model");
    std::array<double[2],6> limits{};
    const bool enabled=robot->getSoftLimit(limits,ec); check(ec);
    require(enabled,"controller soft limits are not enabled");
    if(nearest) {
      const auto tool=robot->toolset(ec); check(ec);
      for(const auto* frame:{&tool.end,&tool.ref}) {
        for(double v:frame->trans) require(std::isfinite(v) && std::abs(v)<1e-8,"nonzero active tool/reference");
        for(double v:frame->rpy) require(std::isfinite(v) && std::abs(v)<1e-8,"nonzero active tool/reference");
      }
      const auto base=robot->baseFrame(ec); check(ec);
      std::array<double,16> worldBase{};
      rokae::Utils::postureToTransArray(base,worldBase);
      using Matrix=Eigen::Matrix<double,4,4,Eigen::RowMajor>;
      const Matrix W=Eigen::Map<const Matrix>(worldBase.data());
      const auto liveBase=model.getCartPose(start);
      const auto liveRef=robot->posture(rokae::CoordinateType::endInRef,ec); check(ec);
      std::array<double,16> liveRefMatrix{};
      rokae::Utils::postureToTransArray(liveRef,liveRefMatrix);
      require((W*Eigen::Map<const Matrix>(liveBase.data())-
               Eigen::Map<const Matrix>(liveRefMatrix.data())).norm()<0.005,"active frame mismatch");
      const Matrix goalRef=W*Eigen::Map<const Matrix>(goalMatrix.data());
      std::array<double,16> goalArray{};
      std::copy(goalRef.data(),goalRef.data()+16,goalArray.begin());
      std::array<double,6> goalPosture{};
      rokae::Utils::transArrayToPosture(goalArray,goalPosture);
      target=model.calcIk(rokae::CartesianPosition(goalPosture),ec); check(ec);
      for(int i=0;i<6;++i) {
        double best=target[i],distance=1e9;
        for(int turns=-2;turns<=2;++turns) {
          const double candidate=target[i]+turns*2*M_PI;
          if(candidate>limits[i][0]+0.05 && candidate<limits[i][1]-0.05 &&
             std::abs(candidate-start[i])<distance) {
            best=candidate; distance=std::abs(candidate-start[i]);
          }
        }
        require(distance<1e9,"no equivalent joint angle within controller limits");
        target[i]=best;
      }
      const auto equivalent=model.getCartPose(target);
      for(int i=0;i<16;++i) require(std::isfinite(equivalent[i]) &&
          std::abs(equivalent[i]-goalMatrix[i])<0.001,"equivalent solution does not match saved flange pose");
      std::cout<<"[TARGET] same saved Cartesian observation pose; controller IK with nearest legal joint turns\n";
    }
    Joints delta{}; double maxChange=0,normSquared=0;
    for(int i=0;i<6;++i) {
      require(std::isfinite(start[i]),"invalid live joint");
      delta[i]=target[i]-start[i];
      maxChange=std::max(maxChange,std::abs(delta[i])); normSquared+=delta[i]*delta[i];
      std::cout<<"[JOINT] J"<<i+1<<" current="<<start[i]<<" target="<<target[i]
               <<" change_deg="<<delta[i]*180/M_PI<<'\n';
    }
    if(maxChange<0.0005) { std::cout<<"[OK] already at observation pose; no motion\n"; return 0; }
    const int samples=std::max(100,static_cast<int>(std::ceil(maxChange/0.005)));
    const auto checkPose=[&](const Joints& q) {
      for(int i=0;i<6;++i) require(std::isfinite(q[i]) && q[i]>limits[i][0]+0.05 &&
                                  q[i]<limits[i][1]-0.05,"joint-limit margin J"+std::to_string(i+1));
      const auto p=model.getCartPose(q);
      for(double v:p) require(std::isfinite(v),"invalid FK");
      if(!(std::hypot(p[3],p[7])<=visual_servo_config::kWorkspaceRadiusMaxM &&
           p[11]>=visual_servo_config::kWorkspaceZMinM &&
           p[11]<=visual_servo_config::kWorkspaceZMaxM)) {
        std::ostringstream detail;
        detail<<"flange outside configured workspace: base xyz=["<<p[3]<<", "<<p[7]
              <<", "<<p[11]<<"] m; radial limit="<<visual_servo_config::kWorkspaceRadiusMaxM
              <<" m, base Z range=["<<visual_servo_config::kWorkspaceZMinM<<", "
              <<visual_servo_config::kWorkspaceZMaxM<<"] m";
        throw std::runtime_error(detail.str());
      }
    };
    for(int k=0;k<=samples;++k) {
      require(!interrupted,"interrupted during planning");
      Joints q{};
      for(int i=0;i<6;++i) q[i]=start[i]+delta[i]*static_cast<double>(k)/samples;
      try { checkPose(q); }
      catch(const std::exception& e) {
        throw std::runtime_error("return path sample "+std::to_string(k)+"/"+
                                 std::to_string(samples)+": "+e.what());
      }
    }
    stationary(*robot,start); stationary(right,rightStart);
    std::cout<<"[PLAN] "<<samples+1<<" joint-space samples passed; max joint change="
             <<maxChange*180/M_PI<<" deg; no camera required; NOT a collision certificate"<<std::endl;
    if(!execute) { std::cout<<"[READ-ONLY] no power, mode, or motion commands\n"; return 0; }
    require(!interrupted,"interrupted before setup");
    configured=true;
    robot->setOperateMode(rokae::OperateMode::automatic,ec); check(ec);
    robot->setPowerState(true,ec); check(ec);
    robot->setMotionControlMode(rokae::MotionControlMode::NrtCommand,ec); check(ec);
    robot->moveReset(ec); check(ec);
    robot->setDefaultZone(0,ec); check(ec);
    stationary(*robot,start); stationary(right,rightStart);
    rokae::MoveAbsJCommand command(std::vector<double>(target.begin(),target.end()),10.0,0.0);
    command.jointSpeed=0.5;
    std::string pathId;
    robot->moveAppend(command,pathId,ec); check(ec);
    require(!interrupted,"interrupted before moveStart");
    motionIssued=true;
    robot->moveStart(ec); check(ec);
    std::cout<<"[EXECUTE] left only: MoveAbsJ, joint speed 1%, no gripper action"<<std::endl;
    const auto began=std::chrono::steady_clock::now();
    auto previousTime=began,nextReport=began,nextRight=began;
    Joints previous=start;
    bool arrived=false;
    while(std::chrono::steady_clock::now()-began<std::chrono::minutes(6)) {
      require(!interrupted,"interrupted during return");
      const auto current=robot->jointPos(ec); check(ec); checkPose(current);
      double t=0;
      for(int i=0;i<6;++i) t+=(current[i]-start[i])*delta[i]/normSquared;
      require(t>=-0.002 && t<=1.002,"motion outside planned joint interval");
      const auto now=std::chrono::steady_clock::now();
      const double dt=std::chrono::duration<double>(now-previousTime).count();
      for(int i=0;i<6;++i) {
        require(std::abs(current[i]-(start[i]+std::clamp(t,0.0,1.0)*delta[i]))<0.04,
                "actual joint path differs from sampled joint interpolation");
        require(dt<=0.02 || std::abs(current[i]-previous[i])/dt<0.10,
                "unexpected joint speed");
      }
      previous=current; previousTime=now;
      const auto power=robot->powerState(ec); check(ec);
      require(power==rokae::PowerState::on,"power/safety state changed");
      const auto state=robot->operationState(ec); check(ec);
      require(state==rokae::OperationState::idle || state==rokae::OperationState::moving,
              "unexpected operation state");
      const auto event=robot->queryEventInfo(rokae::Event::moveExecution,ec); check(ec);
      using namespace rokae::EventInfoKey::MoveExecution;
      if(event.count(ID) && std::any_cast<std::string>(event.at(ID))==pathId) {
        if(event.count(Error)) check(std::any_cast<std::error_code>(event.at(Error)));
        if(event.count(ReachTarget) && std::any_cast<bool>(event.at(ReachTarget))) {
          double error=0;
          for(int i=0;i<6;++i) error=std::max(error,std::abs(current[i]-target[i]));
          require(error<0.0005,"reported arrival does not match saved observation joints");
          std::cout<<"[RESULT] max joint error="<<error<<" rad"<<std::endl;
          arrived=true; break;
        }
      }
      if(now>=nextRight) { stationary(right,rightStart); nextRight=now+std::chrono::seconds(1); }
      if(now>=nextReport) {
        std::cout<<"[MOVING] joint path progress="<<t*100<<"%"<<std::endl;
        nextReport=now+std::chrono::seconds(5);
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    require(arrived,"observation return timed out");
    require(cleanup(*robot),"cleanup failed; verify controller state");
    configured=false;
    const auto finalPower=robot->powerState(ec); check(ec);
    const auto finalMode=robot->operateMode(ec); check(ec);
    stationary(*robot,target);
    require(finalPower==rokae::PowerState::off && finalMode==rokae::OperateMode::manual,
            "manual/power-off state not confirmed");
    std::cout<<"[OK] saved observation pose reached; idle, manual, power off\n";
    return 0;
  } catch(const std::exception& e) {
    if(configured && robot) cleanup(*robot);
    std::cerr<<"[ERROR] "<<e.what()<< (motionIssued ? "; motion was issued, stop requested\n" : "; no motion commanded\n");
    return 2;
  }
}
