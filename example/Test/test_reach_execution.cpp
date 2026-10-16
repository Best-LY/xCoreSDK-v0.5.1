#include <iostream>
#include <opencv2/core.hpp>
#include "mannequin_reach_execution.hpp"
#include "staged_motion_replay.hpp"
using Model=rokae::xMateModel<6>;
namespace sm=staged_motion;
namespace mr=mannequin_reach;

namespace {
sm::Pose fk(const sm::Joints& q) {
  sm::Pose p=sm::Pose::Identity();
  p.translation()=Eigen::Vector3d(.3,0,.2+4*q[0]);
  p.linear()=Eigen::AngleAxisd(q[1],Eigen::Vector3d::UnitZ()).toRotationMatrix();
  return p;
}
mr::Leg leg(const std::string& command,double from,double to,bool rotate=false) {
  mr::Leg result; result.command=command;
  for(int i=0;i<=100;++i) {
    sm::Joints q{}; q[0]=from+(to-from)*i/100.;
    if(rotate) q[1]=q[0]*.1;
    result.samples.push_back({q,fk(q)});
  }
  return result;
}
mr::Candidate candidate(int count,bool linear,double offset=.4,bool rotate=false) {
  mr::Candidate c; c.passed=true;
  if(count==1) c.legs.push_back(leg(linear ? "MoveL" : "MoveAbsJ",0,.2125,rotate));
  else {
    const double via=(.8-offset)/4;
    c.legs.push_back(leg("MoveAbsJ",0,via));
    c.legs.push_back(leg("MoveL",via,.2125));
  }
  return c;
}
sm::Plan convert(const mr::Candidate& c,double insertion=.05) {
  sm::Limits limits{}; for(auto& l:limits) l={-6.28,6.28};
  return mr::executionPlan(c,limits,{},Eigen::Vector3d(.3,0,1.3),insertion,1700000000000.);
}
template<class F> void rejects(F f,const std::string& message) {
  bool rejected=false;
  try { f(); } catch(const std::exception&) { rejected=true; }
  sm::require(rejected,message);
}
}

int main() {
  try {
    int passed=0;
    for(const auto& c:{candidate(1,false),candidate(1,true),candidate(1,true,.4,true),
                       candidate(2,false,.05),candidate(2,false,.15),candidate(2,false,.4)}) {
      auto p=convert(c); sm::Replay replay(p,fk);
      sm::run(p,replay,[]{return false;});
      sm::require(replay.finished && !replay.aborted && replay.commands==int(c.legs.size()),
                  "candidate not completed through supervisor");
      sm::require((replay.read().flange*p.tool-Eigen::Vector3d(.3,0,1.35)).norm()<1e-8,
                  "wrong grasp-center goal");
      ++passed;
    }
    for(int fault=0;fault<6;++fault) {
      auto c=candidate(2,false,.15);
      if(fault==0) c.passed=false;
      if(fault==1) c.legs.clear();
      if(fault==2) c.legs[0].command="unknown";
      if(fault==3) c.legs[1].samples.front().q[0]+=.01;
      if(fault==4) c.legs[1].samples.back().flange.translation().z()+=.01;
      if(fault==5) c.legs[0].samples[40].q[0]=std::numeric_limits<double>::quiet_NaN();
      rejects([&]{convert(c);},"invalid candidate accepted"); ++passed;
    }
    rejects([&]{convert(candidate(1,false),.08);},"old 80 mm request accepted"); ++passed;
    {
      auto p=convert(candidate(1,false)); sm::Replay replay(p,fk);
      rejects([&]{sm::run(p,replay,[&]{return replay.progress>.1;});},"cancel ignored");
      sm::require(replay.aborted && !replay.finished && replay.commands==1,"single-stage abort failed");
      ++passed;
    }
    {
      auto p=convert(candidate(2,false,.15)); sm::Replay replay(p,fk);
      rejects([&]{sm::run(p,replay,[&]{return replay.commands==1 && replay.progress>=1;});},
              "handover cancellation ignored");
      sm::require(replay.aborted && replay.commands==1,"second leg dispatched after cancel");
      ++passed;
    }
    std::cout<<"[PASS] "<<passed<<" reach execution-adapter tests; no device connection or motion\n";
    return 0;
  } catch(const std::exception& e) { std::cerr<<"[FAIL] "<<e.what()<<'\n'; return 1; }
}
