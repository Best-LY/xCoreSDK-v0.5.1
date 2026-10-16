#pragma once
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <opencv2/core.hpp>
#include "staged_motion.hpp"
#include "visual_servo_config.hpp"
#include "rokae/model.h"
#include "rokae/utility.h"

namespace mannequin_reach {
using staged_motion::Pose;
using staged_motion::Joints;
using staged_motion::Limits;
using staged_motion::require;
namespace cfg=visual_servo_config;
inline std::array<double,16> array(const Pose& p) {
  std::array<double,16> a{};
  for(int i=0;i<4;++i) for(int j=0;j<4;++j) a[4*i+j]=p.matrix()(i,j);
  return a;
}
inline Pose pose(const std::array<double,16>& a) {
  Pose p=Pose::Identity();
  for(int i=0;i<4;++i) for(int j=0;j<4;++j) p.matrix()(i,j)=a[4*i+j];
  staged_motion::rigid(p); return p;
}
struct Sample { Joints q; Pose flange; };
struct Leg {
  std::string command;
  std::vector<Sample> samples;
};
struct Candidate {
  std::string name, error;
  double approachOffset=0, tcpLength=0, jointTravel=0, sigma=1e9, condition=0;
  std::vector<Leg> legs;
  bool passed=false;
};
inline Joints legalTurns(const Joints& q,const Joints& reference,const Limits& limits) {
  Joints result{};
  for(int i=0;i<6;++i) {
    require(std::isfinite(q[i]),"nonfinite IK");
    double distance=std::numeric_limits<double>::infinity();
    for(int turn=-3;turn<=3;++turn) {
      const double v=q[i]+turn*2*M_PI;
      if(v>=limits[i][0]+.05 && v<=limits[i][1]-.05 && std::abs(v-reference[i])<distance) {
        result[i]=v; distance=std::abs(v-reference[i]);
      }
    }
    require(std::isfinite(distance),"no legal equivalent angle J"+std::to_string(i+1));
  }
  return result;
}

class Planner {
 public:
  Planner(rokae::xMateModel<6>& model,const Limits& limits,const Pose& world,
          const Eigen::Vector3d& surface,const Pose& goal,double insertion,
          std::function<bool()> interrupted)
      : model_(model),limits_(limits),world_(world),surface_(surface),goal_(goal),
        insertion_(insertion),interrupted_(std::move(interrupted)) {}
  std::vector<Candidate> search(const Joints& start) {
    std::vector<Candidate> result;
    const auto trial=[&](const std::string& name,double offset,auto construct) {
      require(!interrupted_(),"planning interrupted");
      Candidate c; c.name=name; c.approachOffset=offset;
      try {
        construct(c);
        require(!c.legs.empty(),"empty route");
        require(staged_motion::poseNear(c.legs.back().samples.back().flange,goal_,.001,.005),
                "route misses final grasp pose");
        c.passed=true;
        std::cout<<"[CANDIDATE] "<<name<<" PASS tcp_length_m="<<c.tcpLength
                 <<" joint_travel_rad="<<c.jointTravel<<std::endl;
      } catch(const std::exception& e) {
        c.error=e.what(); c.legs.clear();
        std::cout<<"[CANDIDATE] "<<name<<" REJECT "<<c.error<<std::endl;
      }
      require(!interrupted_(),"planning interrupted");
      result.push_back(std::move(c));
    };
    trial("direct_linear",0,[&](Candidate& c){ linear(c,start,goal_); });
    trial("direct_joint",0,[&](Candidate& c){ joint(c,start,goal_); });
    // Distances are candidates, not a mandatory waypoint. Longer offsets are
    // retained as fallbacks so shortening does not take priority over reachability.
    for(double offset:{.05,.10,.15,.20,.30,.40}) {
      trial("joint_then_linear_"+std::to_string(int(std::lround(offset*1000)))+"mm",offset,[&](Candidate& c){
        Pose transit=goal_;
        transit.translation()-=(offset+insertion_)*goal_.linear().col(2);
        joint(c,start,transit);
        linear(c,c.legs.back().samples.back().q,goal_);
      });
    }
    return result;
  }
 private:
  Sample checked(Candidate& c,const Joints& q,const Joints& prior,const Pose& expected) {
    require(!interrupted_(),"planning interrupted");
    require(staged_motion::jointError(q,prior)<=.12,"IK branch jump");
    for(int i=0;i<6;++i) require(q[i]>=limits_[i][0]+.05 && q[i]<=limits_[i][1]-.05,
                                "joint limit J"+std::to_string(i+1));
    const Pose actual=pose(model_.getCartPose(q));
    require(staged_motion::poseNear(actual,expected,.001,.005),"IK/FK mismatch");
    const auto x=actual.translation();
    require(std::hypot(x.x(),x.y())<=cfg::kWorkspaceRadiusMaxM && x.z()>=cfg::kWorkspaceZMinM &&
            x.z()<=cfg::kWorkspaceZMaxM,"flange outside workspace");
    const Eigen::Vector3d grasp=actual*Eigen::Vector3d(0,0,cfg::kLeftFlangeToGraspCenterM);
    require((grasp-surface_).dot(goal_.linear().col(2))<=insertion_+.001,
            "path overshoots requested insertion plane");
    const auto values=model_.jacobian(q);
    Eigen::Matrix<double,6,6,Eigen::RowMajor> jac;
    std::copy(values.begin(),values.end(),jac.data());
    require(jac.allFinite(),"nonfinite model Jacobian");
    const auto sv=jac.jacobiSvd().singularValues();
    const double sigma=sv.minCoeff(),condition=sv.maxCoeff()/sigma;
    require(sigma>=cfg::kMinJacobianSingularValue && condition<=cfg::kMaxJacobianCondition,
            "Jacobian singularity margin");
    c.sigma=std::min(c.sigma,sigma); c.condition=std::max(c.condition,condition);
    return {q,actual};
  }
  void append(Candidate& c,Leg leg) {
    require(leg.samples.size()>=2,"empty leg");
    if(!c.legs.empty()) require(staged_motion::jointError(c.legs.back().samples.back().q,leg.samples.front().q)<1e-9,
                                "leg handover discontinuity");
    const Eigen::Vector3d tool(0,0,cfg::kLeftFlangeToGraspCenterM);
    for(std::size_t i=1;i<leg.samples.size();++i) {
      c.tcpLength+=(leg.samples[i].flange*tool-leg.samples[i-1].flange*tool).norm();
      for(int j=0;j<6;++j) c.jointTravel+=std::abs(leg.samples[i].q[j]-leg.samples[i-1].q[j]);
    }
    c.legs.push_back(std::move(leg));
  }
  void joint(Candidate& c,const Joints& start,const Pose& target) {
    std::array<double,6> posture{}; std::error_code ec;
    rokae::Utils::transArrayToPosture(array(world_*target),posture);
    auto goal=model_.calcIk(rokae::CartesianPosition(posture),ec);
    require(!ec,"controller endpoint IK: "+ec.message()+" ("+std::to_string(ec.value())+")");
    goal=legalTurns(goal,start,limits_);
    const int count=std::max(2,int(std::ceil(staged_motion::jointError(start,goal)/.005)));
    Leg leg; leg.command="MoveAbsJ"; Joints prior=start;
    for(int i=0;i<=count;++i) {
      Joints q{};
      for(int j=0;j<6;++j) q[j]=start[j]+double(i)/count*(goal[j]-start[j]);
      leg.samples.push_back(checked(c,q,prior,pose(model_.getCartPose(q)))); prior=q;
    }
    require(staged_motion::poseNear(leg.samples.back().flange,target,.001,.005),"joint goal mismatch");
    append(c,std::move(leg));
  }
  void linear(Candidate& c,const Joints& start,const Pose& target) {
    const Pose from=pose(model_.getCartPose(start));
    const double angle=Eigen::AngleAxisd(from.linear().transpose()*target.linear()).angle();
    const int count=std::max({2,int(std::ceil((target.translation()-from.translation()).norm()/.004)),
                               int(std::ceil(angle/.005))});
    Leg leg; leg.command="MoveL"; Joints prior=start;
    const Eigen::Quaterniond a(from.linear()),b(target.linear());
    for(int i=0;i<=count;++i) {
      const double t=double(i)/count;
      Pose expected=Pose::Identity();
      expected.translation()=from.translation()+t*(target.translation()-from.translation());
      expected.linear()=a.slerp(t,b).toRotationMatrix();
      Joints q=start;
      if(i) {
        Joints seed{};
        for(int j=0;j<6;++j) seed[j]=std::remainder(prior[j],2*M_PI);
        const int code=model_.getJointPos(array(expected),0,seed,q);
        require(code==0,"seeded linear IK sample "+std::to_string(i)+"/"+std::to_string(count)+
                         " code="+std::to_string(code));
        q=legalTurns(q,prior,limits_);
      }
      leg.samples.push_back(checked(c,q,prior,expected)); prior=q;
    }
    append(c,std::move(leg));
  }
  rokae::xMateModel<6>& model_;
  Limits limits_;
  Pose world_;
  Eigen::Vector3d surface_;
  Pose goal_;
  double insertion_;
  std::function<bool()> interrupted_;
};

inline std::size_t best(const std::vector<Candidate>& candidates) {
  std::size_t chosen=candidates.size();
  for(std::size_t i=0;i<candidates.size();++i) {
    const auto& c=candidates[i];
    if(!c.passed) continue;
    require(std::isfinite(c.tcpLength) && c.tcpLength>=0 && std::isfinite(c.jointTravel),"invalid route score");
    if(chosen==candidates.size() || c.tcpLength<candidates[chosen].tcpLength-1e-9 ||
        (std::abs(c.tcpLength-candidates[chosen].tcpLength)<=1e-9 && c.jointTravel<candidates[chosen].jointTravel)) chosen=i;
  }
  require(chosen<candidates.size(),"no feasible candidate; no motion permitted");
  return chosen;
}

inline void write(const std::string& prefix,const std::vector<Candidate>& candidates,std::size_t chosen,
                  const Eigen::Vector3d& surface,const Pose& goal,const Pose& world,const Pose& camera,
                  const Joints& right,const std::string& capture,const std::string& stamp,double insertion) {
  const auto& selected=candidates.at(chosen);
  const std::string path=prefix+".reach_plan.json",csvPath=prefix+".reach_path.csv";
  require(!std::filesystem::exists(path) && !std::filesystem::exists(csvPath),"reach report already exists");
  std::ofstream csv(csvPath); require(csv.good(),"cannot create reach CSV");
  csv<<"# KINEMATIC CANDIDATE ONLY; NOT COLLISION CLEARANCE\n"
       <<"leg,fraction,q1,q2,q3,q4,q5,q6,grasp_x,grasp_y,grasp_z";
  for(int i=0;i<16;++i) csv<<",flange_"<<i;
  csv<<'\n'<<std::setprecision(17);
  const Eigen::Vector3d tool(0,0,cfg::kLeftFlangeToGraspCenterM);
  for(std::size_t l=0;l<selected.legs.size();++l) {
    const auto& samples=selected.legs[l].samples;
    for(std::size_t i=0;i<samples.size();++i) {
      csv<<l<<','<<double(i)/(samples.size()-1);
      for(double q:samples[i].q) csv<<','<<q;
      const Eigen::Vector3d grasp=samples[i].flange*tool;
      csv<<','<<grasp.x()<<','<<grasp.y()<<','<<grasp.z();
      for(double v:array(samples[i].flange)) csv<<','<<v;
      csv<<'\n';
    }
  }
  csv<<"# COMPLETE: kinematics only; no execution authorization\n";
  csv.close(); require(!csv.fail(),"reach CSV write failed");
  cv::FileStorage out(path+".partial",cv::FileStorage::WRITE|cv::FileStorage::FORMAT_JSON);
  require(out.isOpened(),"cannot create reach manifest");
  const auto vec=[](const Eigen::Vector3d& x){return std::vector<double>{x.x(),x.y(),x.z()};};
  const auto mat=[](const Pose& p){const auto a=array(p); return std::vector<double>(a.begin(),a.end());};
  out<<"schema_version"<<1<<"plan_kind"<<"reach_surface_offset"<<"status"<<"kinematic_plan_complete";
  out<<"robot_motion_permitted"<<0<<"collision_validation"<<"not_performed";
  out<<"collision_resolution_source"<<"user_reported_technical_staff_2026-10-09";
  out<<"controller_trajectory_acceptance"<<"not_tested"<<"gripper_command"<<"none";
  out<<"capture_file"<<capture<<"capture_unix_ms"<<stamp<<"sample_csv"<<csvPath;
  out<<"observation_rounds"<<1<<"robot_arm"<<"left"<<"joint"<<"left_elbow";
  out<<"surface_point_base_m"<<vec(surface)<<"grasp_target_base_m"<<vec(goal*tool);
  out<<"tool_offset_flange_m"<<vec(tool)<<"surface_insertion_m"<<insertion;
  out<<"insertion_direction"<<"approach_positive_flange_z"<<"world_T_base"<<mat(world)<<"flange_T_camera"<<mat(camera);
  out<<"right_arm_reference_joints_rad"<<std::vector<double>(right.begin(),right.end());
  out<<"fixed_pregrasp_required"<<0<<"selected_candidate"<<selected.name;
  out<<"selection_policy"<<"feasible_then_minimum_sampled_grasp_path_length_then_joint_travel";
  out<<"sampled_grasp_path_length_m"<<selected.tcpLength<<"sampled_joint_travel_rad"<<selected.jointTravel;
  out<<"minimum_jacobian_sigma"<<selected.sigma<<"maximum_jacobian_condition"<<selected.condition;
  out<<"candidates"<<"[";
  for(const auto& c:candidates) {
    out<<"{"<<"name"<<c.name<<"passed"<<int(c.passed)<<"surface_pregrasp_m"<<c.approachOffset<<"error"<<c.error;
    if(c.passed) out<<"grasp_path_length_m"<<c.tcpLength<<"joint_travel_rad"<<c.jointTravel;
    out<<"}";
  }
  out<<"]"<<"stages"<<"[";
  for(const auto& leg:selected.legs) {
    const auto& a=leg.samples.front(); const auto& b=leg.samples.back();
    out<<"{"<<"command"<<leg.command<<"start_joints_rad"<<std::vector<double>(a.q.begin(),a.q.end());
    out<<"end_joints_rad"<<std::vector<double>(b.q.begin(),b.q.end());
    out<<"start_flange_base"<<mat(a.flange)<<"end_flange_base"<<mat(b.flange);
    out<<"sample_count"<<int(leg.samples.size())<<"stop_at_endpoint"<<1;
    out<<(leg.command=="MoveL" ? "linear_speed_mm_s" : "joint_speed_fraction")<<(leg.command=="MoveL" ? 5.0 : .01)<<"}";
  }
  out<<"]"; out.release(); std::filesystem::rename(path+".partial",path);
  std::cout<<"[SELECTED] "<<selected.name<<" grasp_path_m="<<selected.tcpLength
           <<" stages="<<selected.legs.size()<<"\n[REACH PLAN] "<<path
           <<"\n[READ-ONLY] no power/mode/motion commands; path shortening is not collision validation"<<std::endl;
}
} // namespace mannequin_reach
