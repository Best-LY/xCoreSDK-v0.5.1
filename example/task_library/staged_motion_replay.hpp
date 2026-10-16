#pragma once
#include "staged_motion.hpp"

namespace staged_motion {
// Virtual clock and synthetic telemetry only. No SDK, sockets or device access.
// This exercises supervision, not controller dynamics or collision clearance.
class Replay : public Backend {
 public:
  Replay(const Plan& plan, std::function<Pose(const Joints&)> fk)
      : plan_(plan), fk_(std::move(fk)) {
    observation_.joints=plan.stages[0].joints.front();
    observation_.rightJoints=plan.rightReference;
    observation_.flange=fk_(observation_.joints);
    observation_.state=State::idle;
    observation_.rightIdle=true;
    observation_.framesMatch=true;
  }
  double unixMs() override { return plan_.captureUnixMs+1000+observation_.monotonicSeconds*1000; }
  Observation read() override { return observation_; }
  void prepare() override { prepared=true; observation_.powerOn=true; }
  std::string start(const Stage& s) override {
    require(prepared && observation_.state==State::idle,"replay not ready");
    ++commands;
    current_=&s;
    progress=0;
    const double distance=s.linear ? (s.end.translation()-s.start.translation()).norm() :
        jointError(s.joints.front(),s.joints.back());
    duration_=std::max(1.0,distance/(s.linear ? s.speed/1000.0 : .02));
    observation_.state=State::moving;
    observation_.reached=false;
    observation_.commandId="replay-stage-"+std::to_string(commands);
    return observation_.commandId;
  }
  void wait() override {
    observation_.monotonicSeconds+=.1;
    if(!current_ || observation_.reached) return;
    progress=std::min(1.0,progress+.1/duration_);
    const double index=progress*(current_->joints.size()-1);
    const auto lo=std::min(static_cast<std::size_t>(index),current_->joints.size()-2);
    const double blend=index-lo;
    for(int i=0;i<6;++i) observation_.joints[i]=(1-blend)*current_->joints[lo][i]+blend*current_->joints[lo+1][i];
    observation_.flange=fk_(observation_.joints);
    if(progress>=1) { observation_.state=State::idle; observation_.reached=true; }
  }
  void finish() override { finished=true; observation_.powerOn=false; }
  bool abort() noexcept override { aborted=true; observation_.powerOn=false; observation_.state=State::idle; return true; }
  int commands=0;
  bool prepared=false, finished=false, aborted=false;
  double progress=0;
 protected:
  const Plan& plan_;
  std::function<Pose(const Joints&)> fk_;
  Observation observation_;
  const Stage* current_=nullptr;
  double duration_=1;
};
} // namespace staged_motion
