#include <iostream>
#include <limits>
#include "staged_motion_replay.hpp"

namespace sm=staged_motion;
namespace {
sm::Pose fakeFk(const sm::Joints& q) {
  auto p=sm::Pose::Identity();
  p.translation()=Eigen::Vector3d(.3,0,.2+4*q[0]);
  return p;
}
sm::Plan fixture() {
  sm::Plan p;
  p.captureUnixMs=1700000000000;
  p.target={.3,0,1.3};
  for(auto& l:p.limits) l={-6.28,6.28};
  for(int s=0;s<2;++s) {
    auto& stage=p.stages[s];
    stage.linear=s==1; stage.speed=s ? 5 : .01;
    for(int i=0;i<=100;++i) {
      sm::Joints q{}; q.fill(s*.1+i*.001);
      stage.joints.push_back(q);
    }
    stage.start=fakeFk(stage.joints.front());
    stage.end=fakeFk(stage.joints.back());
  }
  return p;
}
enum class Fault {
  none, initial, stale, prepare, estop, right, frame, nan, jump, line, clock,
  eventError, falseArrival, staleEvent, movingAtArrival, boundary, secondSend,
  finalPower, cleanup, canceled, reversed, overspeed
};
class FaultRobot : public sm::Replay {
 public:
  FaultRobot(const sm::Plan& p,Fault f):Replay(p,fakeFk),fault(f) {}
  double unixMs() override { return Replay::unixMs()+(fault==Fault::stale ? 31000 : 0); }
  void prepare() override {
    Replay::prepare();
    if(fault==Fault::prepare) throw std::runtime_error("injected partial prepare failure");
  }
  std::string start(const sm::Stage& s) override {
    if(fault==Fault::secondSend && commands==1) throw std::runtime_error("injected second send failure");
    return Replay::start(s);
  }
  sm::Observation read() override {
    auto o=Replay::read();
    if(!prepared && fault==Fault::initial) o.joints[0]+=.01;
    if(commands && progress>.05) {
      if(fault==Fault::estop || fault==Fault::cleanup) o.powerOn=false;
      if(fault==Fault::right) o.rightJoints[0]+=.01;
      if(fault==Fault::frame) o.framesMatch=false;
      if(fault==Fault::nan) o.joints[0]=std::numeric_limits<double>::quiet_NaN();
      if(fault==Fault::jump) o.joints[0]+=.1;
      if(fault==Fault::clock) o.monotonicSeconds+=1;
      if(fault==Fault::eventError) o.controllerError="injected controller error (-32)";
      if(fault==Fault::falseArrival) { o.reached=true; o.state=sm::State::idle; }
      if(fault==Fault::staleEvent) o.commandId="previous-command";
      if(fault==Fault::movingAtArrival) o.state=sm::State::moving;
    }
    if(commands==2 && progress>.05 && fault==Fault::line) o.flange.translation().x()+=.005;
    if(commands==2 && progress>.05 && fault==Fault::overspeed) o.flange.translation().z()+=.002;
    if(fault==Fault::boundary && commands==1 && progress==1) {
      ++boundaryReads;
      if(boundaryReads>=6) o.joints[0]+=.01;
    }
    if(fault==Fault::finalPower && finished) o.powerOn=true;
    return o;
  }
  void wait() override {
    Replay::wait();
    if(fault==Fault::reversed && progress>.5 && !injected) {
      // Small enough to pass the speed check, large enough to violate progress.
      for(auto& q:observation_.joints) q-=.0008;
      observation_.flange=fakeFk(observation_.joints);
      injected=true;
    }
  }
  bool abort() noexcept override { Replay::abort(); return fault!=Fault::cleanup; }
  Fault fault;
  int boundaryReads=0;
  bool injected=false;
};
void check(bool ok,const std::string& message) { sm::require(ok,message); }
}

int main() {
  int passed=0;
  try {
    sm::validatePayloadReadback(0,{0,0,0},{0,0,0}); ++passed;
    sm::validatePayloadReadback(1,{0,0,-.1},{.1,.1,.1}); ++passed;
    for(int test=0;test<5;++test) {
      double mass=1;
      std::array<double,3> cog{},inertia{};
      if(test==0) mass=-1;
      if(test==1) mass=std::numeric_limits<double>::quiet_NaN();
      if(test==2) cog[0]=std::numeric_limits<double>::infinity();
      if(test==3) inertia[0]=-1;
      if(test==4) inertia[0]=std::numeric_limits<double>::quiet_NaN();
      bool rejected=false;
      try { sm::validatePayloadReadback(mass,cog,inertia); } catch(...) { rejected=true; }
      check(rejected,"invalid payload readback accepted"); ++passed;
    }
    {
      auto p=fixture(); FaultRobot robot(p,Fault::none);
      sm::run(p,robot,[]{return false;});
      check(robot.commands==2 && robot.finished && !robot.aborted,"nominal run failed");
      // Virtual time exceeds capture TTL at stage 2: no re-identification is required.
      check(robot.read().monotonicSeconds>80,"both full stages not traversed");
      ++passed;
    }
    {
      auto p=fixture(); p.insertion=.08;
      auto& s=p.stages[1];
      for(std::size_t j=0;j<s.joints.size();++j) s.joints[j].fill(.1+.12*double(j)/(s.joints.size()-1));
      s.end=fakeFk(s.joints.back());
      FaultRobot robot(p,Fault::none);
      sm::run(p,robot,[]{return false;});
      check(robot.commands==2 && robot.finished && !robot.aborted,"48 cm insertion replay failed");
      check(std::abs((s.end*p.tool-p.target).z()-.08)<1e-9,"surface point was used as final center");
      ++passed;
      p.insertion=.081;
      bool rejected=false;
      try { sm::validate(p); } catch(...) { rejected=true; }
      check(rejected,"insertion range not enforced"); ++passed;
    }
    const std::vector<std::pair<Fault,std::string>> faults={
      {Fault::initial,"stage boundary"},{Fault::stale,"stale initial"},
      {Fault::prepare,"prepare failure"},{Fault::estop,"power/safety"},
      {Fault::right,"right arm"},{Fault::frame,"reference changed"},
      {Fault::nan,"invalid telemetry"},{Fault::jump,"joint velocity"},
      {Fault::line,"linear velocity"},{Fault::clock,"telemetry gap"},
      {Fault::eventError,"(-32)"},{Fault::falseArrival,"stage boundary"},
      {Fault::staleEvent,"timeout"},{Fault::movingAtArrival,"timeout"},
      {Fault::boundary,"stage boundary"},{Fault::secondSend,"second send"},
      {Fault::finalPower,"power off"},{Fault::cleanup,"CLEANUP FAILED"},
      {Fault::canceled,"interruption"},{Fault::overspeed,"linear velocity"}
    };
    for(const auto& entry:faults) {
      auto p=fixture(); FaultRobot robot(p,entry.first); bool threw=false;
      try { sm::run(p,robot,[&]{return entry.first==Fault::canceled && robot.commands==1 && robot.progress>.1;}); }
      catch(const std::exception& error) {
        threw=true;
        check(std::string(error.what()).find(entry.second)!=std::string::npos,
              "wrong failure for "+entry.second+": "+error.what());
      }
      check(threw,"fault was not detected: "+entry.second);
      if(entry.first==Fault::initial || entry.first==Fault::stale)
        check(!robot.prepared && robot.commands==0,"invalid initial state touched robot");
      else check(robot.aborted,"fault did not attempt cleanup");
      const bool stageTwo=entry.first==Fault::line || entry.first==Fault::finalPower || entry.first==Fault::overspeed;
      check(robot.commands<=(stageTwo ? 2 : 1),"stage two issued after stage-one failure");
      ++passed;
    }
    for(int change=0;change<6;++change) {
      auto p=fixture();
      if(change==0) p.stages[1].joints.front()[0]+=.01;
      if(change==1) p.stages[0].speed=.5;
      if(change==2) p.stages[1].speed=50;
      if(change==3) p.target.x()+=.01;
      if(change==4) p.stages[1].joints[50][0]=std::numeric_limits<double>::quiet_NaN();
      if(change==5) p.stages[0].joints[50][0]+=.01;
      FaultRobot robot(p,Fault::none); bool threw=false;
      try { sm::run(p,robot,[]{return false;}); } catch(...) { threw=true; }
      check(threw && !robot.prepared,"invalid plan was admitted");
      ++passed;
    }
    std::cout<<"[PASS] "<<passed<<" staged supervisor tests; synthetic backend only; no SDK/device access\n";
    return 0;
  } catch(const std::exception& error) {
    std::cerr<<"[FAIL] after "<<passed<<" tests: "<<error.what()<<'\n'; return 1;
  }
}
