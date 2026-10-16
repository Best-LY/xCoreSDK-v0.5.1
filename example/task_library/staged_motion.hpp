#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <functional>
#include <stdexcept>
#include <string>
#include <vector>
#include <Eigen/Geometry>

// SDK-independent supervisor. It is not a safety-rated collision controller.
namespace staged_motion {
using Joints = std::array<double, 6>;
using Pose = Eigen::Isometry3d;
using Limits = std::array<std::array<double, 2>, 6>;

inline void require(bool condition, const std::string& text) {
  if (!condition) throw std::runtime_error(text);
}
inline bool finite(const Joints& q) {
  return std::all_of(q.begin(), q.end(), [](double v) { return std::isfinite(v); });
}
inline void validatePayloadReadback(double mass, const std::array<double,3>& cog,
                                    const std::array<double,3>& inertia) {
  // Zero can be an unsupported/default SDK field, not evidence of no tool.
  // This validates numeric readback only, not physical payload or safety setup.
  require(std::isfinite(mass) && mass>=0,"invalid SDK payload mass readback");
  for(int i=0;i<3;++i) require(std::isfinite(cog[i]) && std::isfinite(inertia[i]) &&
      inertia[i]>=0,"invalid SDK payload parameter readback");
}
inline double jointError(const Joints& a, const Joints& b) {
  require(finite(a) && finite(b), "nonfinite joint observation");
  double result = 0;
  for (int i = 0; i < 6; ++i) result = std::max(result, std::abs(a[i] - b[i]));
  return result;
}
inline void rigid(const Pose& p) {
  require(p.matrix().allFinite() &&
      (p.linear().transpose()*p.linear()-Eigen::Matrix3d::Identity()).norm() < 1e-6 &&
      std::abs(p.linear().determinant()-1) < 1e-6 &&
      (p.matrix().row(3)-Eigen::RowVector4d(0,0,0,1)).norm() < 1e-9,
      "invalid rigid pose");
}
inline bool poseNear(const Pose& a, const Pose& b, double distance, double angle) {
  rigid(a); rigid(b);
  return (a.translation()-b.translation()).norm() <= distance &&
      Eigen::AngleAxisd(a.linear().transpose()*b.linear()).angle() <= angle;
}

struct Stage {
  bool linear = false;
  Pose start = Pose::Identity(), end = Pose::Identity();
  std::vector<Joints> joints;
  double speed = .01; // Joint fraction for stage 1; mm/s for stage 2.
};
struct Plan {
  enum class Layout { fixedPregrasp, reachCandidate };
  Layout layout = Layout::fixedPregrasp;
  std::vector<Stage> stages = std::vector<Stage>(2);
  Limits limits{};
  Joints rightReference{};
  Eigen::Vector3d target = Eigen::Vector3d::Zero();
  double insertion = 0; // Target is the observed surface, not the final grasp center.
  Eigen::Vector3d tool{0, 0, .3};
  double captureUnixMs = 0;
};
enum class State { idle, moving, fault };
struct Observation {
  Joints joints{}, rightJoints{};
  Pose flange = Pose::Identity();
  State state = State::fault;
  bool powerOn = false, rightIdle = false, framesMatch = false;
  std::string commandId, controllerError;
  bool reached = false;
  double monotonicSeconds = 0;
};
struct Backend {
  virtual ~Backend() = default;
  virtual double unixMs() = 0;
  virtual Observation read() = 0;
  virtual void prepare() = 0;
  virtual std::string start(const Stage&) = 0;
  virtual void wait() = 0;
  virtual void finish() = 0;
  // Must attempt every cleanup action even when an earlier one fails.
  virtual bool abort() noexcept = 0;
};

inline void validate(const Plan& p) {
  require(std::isfinite(p.captureUnixMs) && p.captureUnixMs > 0, "invalid capture time");
  require(p.target.allFinite() && (p.tool-Eigen::Vector3d(0,0,.3)).norm() < 1e-9,
          "invalid target/tool");
  require(std::isfinite(p.insertion) && p.insertion>=0 && p.insertion<=.080,"invalid surface insertion");
  require(finite(p.rightReference), "invalid right reference");
  require(!p.stages.empty() && p.stages.size()<=2,"invalid stage count");
  require(p.layout==Plan::Layout::fixedPregrasp || p.layout==Plan::Layout::reachCandidate,
          "unknown plan layout");
  if(p.layout==Plan::Layout::fixedPregrasp || p.stages.size()==2)
    require(p.stages.size()==2 && !p.stages[0].linear && p.stages[1].linear, "incorrect stage order");
  for (int i=0; i<6; ++i) require(std::isfinite(p.limits[i][0]) && std::isfinite(p.limits[i][1]) &&
      p.limits[i][1]-p.limits[i][0] > .1, "invalid joint limits");
  for (const auto& s : p.stages) {
    require(std::isfinite(s.speed) && s.speed>0 && s.speed<=(s.linear ? 5 : .01),
            "commissioning speed exceeded");
    rigid(s.start); rigid(s.end);
    if(s.linear) require((s.end.translation()-s.start.translation()).norm()>1e-8,
                          "empty Cartesian translation");
    require(s.joints.size() >= 2, "missing sampled route");
    for (std::size_t j=0; j<s.joints.size(); ++j) {
      require(finite(s.joints[j]), "nonfinite planned joint");
      for (int i=0; i<6; ++i) require(s.joints[j][i] >= p.limits[i][0]+.05 &&
          s.joints[j][i] <= p.limits[i][1]-.05, "planned joint outside margin");
      if (j) require(jointError(s.joints[j],s.joints[j-1]) <= .12, "sample branch discontinuity");
      if (!s.linear) {
        const double t=double(j)/(s.joints.size()-1);
        Joints expected{};
        for (int i=0;i<6;++i) expected[i]=s.joints.front()[i]+t*(s.joints.back()[i]-s.joints.front()[i]);
        require(jointError(expected,s.joints[j]) < 1e-8,"joint transit is not the sampled interpolation");
      }
    }
    require(jointError(s.joints.front(),s.joints.back()) > 1e-8,"empty stage motion");
  }
  for(std::size_t i=1;i<p.stages.size();++i)
    require(jointError(p.stages[i-1].joints.back(),p.stages[i].joints.front()) < 1e-9 &&
        poseNear(p.stages[i-1].end,p.stages[i].start,1e-9,1e-9), "discontinuous stage handover");
  const auto& b=p.stages.back();
  const double targetTolerance=p.layout==Plan::Layout::fixedPregrasp ? 1e-8 : .0025;
  require((b.end*p.tool-(p.target+p.insertion*b.end.linear().col(2))).norm() < targetTolerance,
          "route does not end at the locked target");
  if(p.layout==Plan::Layout::fixedPregrasp) {
    require((b.start.linear()-b.end.linear()).norm() < 1e-9, "approach orientation changes");
    require((b.start*p.tool-(p.target-.4*b.end.linear().col(2))).norm() < 1e-8,
            "route does not end at the locked target");
  }
}

inline void common(const Plan& p, const Observation& o) {
  require(finite(o.joints) && std::isfinite(o.monotonicSeconds), "invalid telemetry");
  rigid(o.flange);
  require(o.framesMatch,"tool/base/reference changed");
  require(o.rightIdle && jointError(o.rightJoints,p.rightReference) <= .001,"right arm moved/not idle");
  require(o.state != State::fault, "robot safety/operation state changed");
  require(o.controllerError.empty(), "controller: "+o.controllerError);
  for(int i=0;i<6;++i) require(o.joints[i] >= p.limits[i][0]+.04 &&
      o.joints[i] <= p.limits[i][1]-.04, "actual joint outside margin");
  const auto t=o.flange.translation();
  require(std::hypot(t.x(),t.y()) <= 1.5 && t.z() >= -.2 && t.z() <= 1.8,
      "actual flange outside workspace");
}

inline void atBoundary(const Plan& p, const Stage& s, const Observation& o, bool end) {
  common(p,o);
  require(o.state == State::idle,"stage boundary is not idle");
  require(jointError(o.joints,end ? s.joints.back() : s.joints.front()) <= .0005 &&
      poseNear(o.flange,end ? s.end : s.start,.003,.01), "stage boundary pose/joints mismatch");
}

inline double onRoute(const Plan& p, const Stage& s, const Observation& o) {
  double fraction=0;
  if(s.linear) {
    const Eigen::Vector3d line=s.end.translation()-s.start.translation();
    fraction=(o.flange.translation()-s.start.translation()).dot(line)/line.squaredNorm();
    Pose expected=s.start;
    expected.translation()+=fraction*line;
    if(p.layout==Plan::Layout::reachCandidate)
      expected.linear()=Eigen::Quaterniond(s.start.linear()).slerp(
          std::clamp(fraction,0.,1.),Eigen::Quaterniond(s.end.linear())).toRotationMatrix();
    require(poseNear(o.flange,expected,.003,.01),"actual flange departed from planned line/orientation");
  } else {
    double denominator=0;
    for(int i=0;i<6;++i) {
      const double d=s.joints.back()[i]-s.joints.front()[i];
      fraction+=(o.joints[i]-s.joints.front()[i])*d;
      denominator+=d*d;
    }
    fraction/=denominator;
  }
  require(std::isfinite(fraction) && fraction >= -.002 && fraction <= 1.002,
          "actual motion outside planned interval");
  const double index=std::clamp(fraction,0.,1.)*(s.joints.size()-1);
  const auto lo=std::min(static_cast<std::size_t>(index),s.joints.size()-2);
  const double blend=index-lo;
  Joints expected{};
  for(int i=0;i<6;++i) expected[i]=(1-blend)*s.joints[lo][i]+blend*s.joints[lo+1][i];
  require(jointError(o.joints,expected) <= .04,"actual joints departed from planned branch");
  const Eigen::Vector3d normal=-p.stages.back().end.linear().col(2);
  const double minimumGap=p.layout==Plan::Layout::fixedPregrasp && !s.linear ? .39 : -p.insertion-.002;
  require((o.flange*p.tool-p.target).dot(normal) >= minimumGap,
          "actual tool crossed target standoff plane");
  return fraction;
}

inline void run(const Plan& p, Backend& backend, const std::function<bool()>& canceled) {
  validate(p);
  const auto active=[&] { require(!canceled(),"operator interruption"); };
  const auto fresh=[&] {
    const double age=backend.unixMs()-p.captureUnixMs;
    require(std::isfinite(age) && age >= -2000 && age <= 30000,"stale initial observation");
  };
  active(); fresh();
  atBoundary(p,p.stages[0],backend.read(),false);
  // Arm cleanup before prepare: it may throw after partially changing state.
  try {
    backend.prepare();
    for(std::size_t stageIndex=0;stageIndex<p.stages.size();++stageIndex) {
      active();
      if(stageIndex==0) fresh(); // Stage 2 uses the same locked scene, not a new photo.
      const auto& stage=p.stages[stageIndex];
      Observation previous=backend.read();
      atBoundary(p,stage,previous,false);
      require(previous.powerOn,"power off before stage");
      active();
      const std::string id=backend.start(stage);
      require(!id.empty(),"empty controller command ID");
      const double began=previous.monotonicSeconds;
      const double timeout=stage.linear ?
          1000*(stage.end.translation()-stage.start.translation()).norm()/stage.speed+40.0 : 720.0;
      double stableSince=-1, maximumFraction=0;
      bool arrived=false;
      for(;;) {
        active(); backend.wait(); active();
        const Observation now=backend.read();
        common(p,now);
        require(now.powerOn,"power/safety state changed during motion");
        const double dt=now.monotonicSeconds-previous.monotonicSeconds;
        require(dt>0 && dt<=.5,"telemetry gap or nonmonotonic clock");
        require(now.monotonicSeconds-began<=timeout,"stage execution timeout");
        require(jointError(now.joints,previous.joints)/dt <= .10,"unexpected joint velocity");
        if(stage.linear) require((now.flange.translation()-previous.flange.translation()).norm()/dt <= .010,
            "unexpected linear velocity");
        const double fraction=onRoute(p,stage,now);
        require(fraction >= maximumFraction-.005,"unexpected reverse motion");
        maximumFraction=std::max(maximumFraction,fraction);
        if(now.commandId==id && now.reached && now.state==State::idle) {
          atBoundary(p,stage,now,true);
          if(stableSince<0) stableSince=now.monotonicSeconds;
          if(now.monotonicSeconds-stableSince>=.3) { arrived=true; break; }
        } else stableSince=-1;
        previous=now;
      }
      require(arrived,"stage did not finish");
    }
    backend.finish();
    const auto final=backend.read();
    atBoundary(p,p.stages.back(),final,true);
    require(!final.powerOn,"final power off not confirmed");
  } catch(const std::exception& error) {
    const std::string message=error.what();
    const bool stopped=backend.abort();
    throw std::runtime_error(message+(stopped ? "; cleanup reported success" :
        "; CLEANUP FAILED: use physical emergency stop and inspect controller"));
  } catch(...) {
    backend.abort();
    throw;
  }
}
} // namespace staged_motion
