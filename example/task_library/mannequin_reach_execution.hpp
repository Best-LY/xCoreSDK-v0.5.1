#pragma once
#include "mannequin_reach_planner.hpp"

namespace mannequin_reach {
// Convert the selected in-memory candidate, not a separately edited CSV.
// This is shared supervision data, not authorization to dispatch hardware.
inline staged_motion::Plan executionPlan(const Candidate& candidate,const Limits& limits,
                                         const Joints& right,const Eigen::Vector3d& surface,
                                         double insertion,double captureUnixMs) {
  require(candidate.passed,"cannot execute a rejected candidate");
  require(std::isfinite(insertion) && std::abs(insertion-.05)<1e-9,
          "reach execution target must use 50 mm insertion");
  staged_motion::Plan plan;
  plan.layout=staged_motion::Plan::Layout::reachCandidate;
  plan.stages.clear();
  plan.limits=limits; plan.rightReference=right;
  plan.target=surface; plan.insertion=insertion; plan.captureUnixMs=captureUnixMs;
  plan.tool=Eigen::Vector3d(0,0,cfg::kLeftFlangeToGraspCenterM);
  for(const auto& leg:candidate.legs) {
    require(leg.command=="MoveL" || leg.command=="MoveAbsJ","unknown candidate command");
    require(leg.samples.size()>=2,"candidate samples missing");
    staged_motion::Stage stage;
    stage.linear=leg.command=="MoveL"; stage.speed=stage.linear ? 5 : .01;
    stage.start=leg.samples.front().flange; stage.end=leg.samples.back().flange;
    for(const auto& sample:leg.samples) stage.joints.push_back(sample.q);
    plan.stages.push_back(std::move(stage));
  }
  staged_motion::validate(plan);
  return plan;
}
}
