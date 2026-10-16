#pragma once
#include <chrono>
#include <iostream>
#include <thread>
#include "staged_motion.hpp"
#include "rokae/robot.h"
#include "rokae/model.h"
#include "rokae/utility.h"

namespace staged_motion {
// The caller must retain the incident execution hold. This adapter has not
// been commissioned on hardware; replay success does not release that hold.
class SdkBackend : public Backend {
 public:
  SdkBackend(rokae::xMateRobot& left, rokae::xMateRobot& right,
             rokae::xMateModel<6>& model, const Pose& worldBase,
             std::function<bool()> canceled)
      : left_(left), right_(right), model_(model), worldBase_(worldBase), canceled_(std::move(canceled)) {}
  double unixMs() override {
    return std::chrono::duration<double,std::milli>(std::chrono::system_clock::now().time_since_epoch()).count();
  }
  Observation read() override {
    Observation o;
    std::error_code ec;
    o.joints=left_.jointPos(ec); check(ec);
    o.flange=matrix(model_.getCartPose(o.joints));
    const auto state=left_.operationState(ec); check(ec);
    o.state=state==rokae::OperationState::idle ? State::idle :
        (state==rokae::OperationState::moving ? State::moving : State::fault);
    const auto power=left_.powerState(ec); check(ec);
    o.powerOn=power==rokae::PowerState::on;
    require(power==rokae::PowerState::on || power==rokae::PowerState::off,"controller safety state changed");
    o.rightJoints=right_.jointPos(ec); check(ec);
    o.rightIdle=right_.operationState(ec)==rokae::OperationState::idle; check(ec);
    const auto tool=left_.toolset(ec); check(ec);
    validatePayloadReadback(tool.load.mass,tool.load.cog,tool.load.inertia);
    if(haveLoad_) {
      require(std::isfinite(tool.load.mass) && std::abs(tool.load.mass-load_.mass)<1e-9,
              "SDK payload mass changed");
      for(int i=0;i<3;++i) require(std::isfinite(tool.load.cog[i]) && std::isfinite(tool.load.inertia[i]) &&
          std::abs(tool.load.cog[i]-load_.cog[i])<1e-9 &&
          std::abs(tool.load.inertia[i]-load_.inertia[i])<1e-9,"SDK payload parameters changed");
    }
    o.framesMatch=true;
    for(const auto* f:{&tool.end,&tool.ref}) {
      for(double x:f->trans) o.framesMatch=o.framesMatch && std::isfinite(x) && std::abs(x)<1e-8;
      for(double x:f->rpy) o.framesMatch=o.framesMatch && std::isfinite(x) && std::abs(x)<1e-8;
    }
    const auto base=left_.baseFrame(ec); check(ec);
    std::array<double,16> m{};
    rokae::Utils::postureToTransArray(base,m);
    o.framesMatch=o.framesMatch && poseNear(matrix(m),worldBase_,1e-8,1e-8);
    const auto active=left_.posture(rokae::CoordinateType::endInRef,ec); check(ec);
    rokae::Utils::postureToTransArray(active,m);
    o.framesMatch=o.framesMatch && poseNear(matrix(m),worldBase_*o.flange,.005,.01);
    if(!currentId_.empty()) {
      const auto event=left_.queryEventInfo(rokae::Event::moveExecution,ec); check(ec);
      using namespace rokae::EventInfoKey::MoveExecution;
      if(event.count(ID)) o.commandId=std::any_cast<std::string>(event.at(ID));
      if(o.commandId==currentId_) {
        if(event.count(Error)) {
          const auto error=std::any_cast<std::error_code>(event.at(Error));
          if(error) o.controllerError=error.message()+" (code="+std::to_string(error.value())+")";
        }
        if(event.count(ReachTarget)) o.reached=std::any_cast<bool>(event.at(ReachTarget));
      }
    }
    o.monotonicSeconds=std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
    return o;
  }
  void prepare() override {
    std::error_code ec;
    active();
    const auto tool=left_.toolset(ec); check(ec);
    validatePayloadReadback(tool.load.mass,tool.load.cog,tool.load.inertia);
    load_=tool.load; haveLoad_=true;
    active();
    left_.setOperateMode(rokae::OperateMode::automatic,ec); check(ec);
    active();
    left_.setPowerState(true,ec); check(ec);
    active();
    left_.setMotionControlMode(rokae::MotionControlMode::NrtCommand,ec); check(ec);
    left_.moveReset(ec); check(ec);
    left_.setDefaultZone(0,ec); check(ec);
  }
  std::string start(const Stage& s) override {
    std::error_code ec;
    active();
    const auto state=left_.operationState(ec); check(ec);
    require(state==rokae::OperationState::idle,"controller not idle before command");
    const auto q=left_.jointPos(ec); check(ec);
    require(jointError(q,s.joints.front()) <= .0005,"start joints changed before dispatch");
    left_.moveReset(ec); check(ec);
    currentId_.clear();
    active();
    if(s.linear) {
      std::array<double,6> posture{};
      rokae::Utils::transArrayToPosture(array(worldBase_*s.end),posture);
      left_.moveAppend(rokae::MoveLCommand(rokae::CartesianPosition(posture),s.speed,0),currentId_,ec);
    } else {
      rokae::MoveAbsJCommand cmd(std::vector<double>(s.joints.back().begin(),s.joints.back().end()),10,0);
      cmd.jointSpeed=s.speed;
      left_.moveAppend(cmd,currentId_,ec);
    }
    check(ec);
    require(!currentId_.empty(),"controller returned empty command ID");
    active();
    left_.moveStart(ec); check(ec);
    std::cout<<"[DISPATCH] "<<(s.linear ? "MoveL mm/s=" : "MoveAbsJ fraction=")<<s.speed
             <<" ID="<<currentId_<<std::endl;
    return currentId_;
  }
  void wait() override { std::this_thread::sleep_for(std::chrono::milliseconds(100)); }
  void finish() override { require(abort(),"normal completion cleanup failed"); }
  bool abort() noexcept override {
    bool ok=true;
    // Do not reset errors, resume, retract, return home or actuate the gripper.
    const auto attempt=[&](const char* label,auto action) {
      try {
        std::error_code ec; action(ec);
        if(ec) { ok=false; std::cerr<<"[CLEANUP] "<<label<<": "<<ec.message()<<" ("<<ec.value()<<")\n"; }
      } catch(...) { ok=false; }
    };
    attempt("stop",[&](auto& ec){left_.stop(ec);});
    attempt("power off",[&](auto& ec){left_.setPowerState(false,ec);});
    attempt("clear queued motion",[&](auto& ec){left_.moveReset(ec);});
    attempt("idle",[&](auto& ec){left_.setMotionControlMode(rokae::MotionControlMode::Idle,ec);});
    attempt("manual",[&](auto& ec){left_.setOperateMode(rokae::OperateMode::manual,ec);});
    return ok;
  }
 private:
  void active() { require(!canceled_(),"operator interruption before SDK command"); }
  static void check(const std::error_code& ec) {
    require(!ec,ec.message()+" (code="+std::to_string(ec.value())+")");
  }
  static Pose matrix(const std::array<double,16>& values) {
    Pose p=Pose::Identity();
    for(int i=0;i<4;++i) for(int j=0;j<4;++j) p.matrix()(i,j)=values[4*i+j];
    rigid(p); return p;
  }
  static std::array<double,16> array(const Pose& p) {
    std::array<double,16> result{};
    for(int i=0;i<4;++i) for(int j=0;j<4;++j) result[4*i+j]=p.matrix()(i,j);
    return result;
  }
  rokae::xMateRobot& left_;
  rokae::xMateRobot& right_;
  rokae::xMateModel<6>& model_;
  Pose worldBase_;
  std::string currentId_;
  std::function<bool()> canceled_;
  rokae::Load load_;
  bool haveLoad_=false;
};
} // namespace staged_motion
