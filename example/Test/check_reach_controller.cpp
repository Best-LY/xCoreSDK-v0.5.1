#include <array>
#include <cmath>
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>
#include <opencv2/core.hpp>
#include "mannequin_reach_planner.hpp"
#include "robot_vision_config.hpp"
#include "rokae/robot.h"

namespace {
using namespace mannequin_reach;
using Model = rokae::xMateModel<6>;

void check(const std::error_code& ec) {
  require(!ec, ec.message()+" (code="+std::to_string(ec.value())+")");
}

template<std::size_t N>
std::array<double,N> read(const cv::FileNode& node) {
  require(node.isSeq() && node.size()==N,"invalid array in reach manifest");
  std::array<double,N> a{};
  std::size_t i=0;
  for(const auto& item:node) {
    a[i++]=static_cast<double>(item);
    require(std::isfinite(a[i-1]),"nonfinite manifest value");
  }
  return a;
}

rokae::CartesianPosition cartesian(const Pose& p) {
  std::array<double,6> posture{};
  rokae::Utils::transArrayToPosture(array(p),posture);
  return rokae::CartesianPosition(posture);
}
}

int main(int argc,char** argv) {
  try {
    require(argc==2 && std::string(argv[1]).find("--")!=0,
            "Usage: check_reach_controller PLAN.reach_plan.json (read-only diagnostic)");
    cv::FileStorage input(argv[1],cv::FileStorage::READ);
    require(input.isOpened(),"cannot open reach manifest");
    require(static_cast<std::string>(input["plan_kind"])=="reach_surface_offset" &&
            static_cast<std::string>(input["robot_arm"])=="left" &&
            static_cast<int>(input["schema_version"])==1 &&
            static_cast<int>(input["robot_motion_permitted"])==0,
            "unsupported reach manifest");
    const auto stages=input["stages"];
    require(stages.isSeq() && stages.size()>=1 && stages.size()<=2,"invalid stages");
    const Pose world=pose(read<16>(input["world_T_base"]));
    for(const auto& stage:stages) {
      const auto command=static_cast<std::string>(stage["command"]);
      require(command=="MoveL" || command=="MoveAbsJ","unknown stage command");
      read<6>(stage["start_joints_rad"]); read<6>(stage["end_joints_rad"]);
      pose(read<16>(stage["start_flange_base"]));
      pose(read<16>(stage["end_flange_base"]));
    }
    std::cout<<"[READ-ONLY] Saved candidate diagnostic, not a fresh target or execution authorization.\n"
             <<"[READ-ONLY] No power/mode/move/gripper commands will be sent."<<std::endl;
    rokae::xMateRobot robot(user_config::IP_LEFT,user_config::LOCAL_IP);
    std::error_code ec;
    const auto state=robot.operationState(ec); check(ec);
    require(state==rokae::OperationState::idle,"left arm is not idle");
    const auto before=robot.jointPos(ec); check(ec);
    const auto tools=robot.toolset(ec); check(ec);
    for(const auto* frame:{&tools.end,&tools.ref}) {
      for(double v:frame->trans) require(std::isfinite(v) && std::abs(v)<1e-8,"nonzero active tool/ref");
      for(double v:frame->rpy) require(std::isfinite(v) && std::abs(v)<1e-8,"nonzero active tool/ref");
    }
    const auto base=robot.baseFrame(ec); check(ec);
    std::array<double,16> baseMatrix{};
    rokae::Utils::postureToTransArray(base,baseMatrix);
    require(staged_motion::poseNear(pose(baseMatrix),world,1e-8,1e-8),"base frame changed");
    Model model=robot.model();
    unsigned index=0,accepted=0;
    bool failed=false;
    for(const auto& stage:stages) {
      ++index;
      const auto q0=read<6>(stage["start_joints_rad"]);
      const auto q1=read<6>(stage["end_joints_rad"]);
      const auto from=pose(read<16>(stage["start_flange_base"]));
      const auto to=pose(read<16>(stage["end_flange_base"]));
      require(staged_motion::poseNear(pose(model.getCartPose(q0)),from,.001,.005) &&
              staged_motion::poseNear(pose(model.getCartPose(q1)),to,.001,.005),
              "saved joints/FK mismatch");
      if(static_cast<std::string>(stage["command"])!="MoveL") {
        std::cout<<"[STAGE "<<index<<"] MoveAbsJ: Cartesian checkPath not applicable; NOT controller-validated."<<std::endl;
        continue;
      }
      // Use the planned handover joints, never the currently observed arm joints.
      const auto result=robot.checkPath(cartesian(world*from),std::vector<double>(q0.begin(),q0.end()),
                                        cartesian(world*to),ec);
      std::cout<<"[STAGE "<<index<<"] checkPath code="<<ec.value()<<" message="<<ec.message()<<std::endl;
      if(ec) { failed=true; continue; }
      require(result.size()==6,"checkPath returned unexpected axis count");
      Joints end{};
      std::copy(result.begin(),result.end(),end.begin());
      for(double v:end) require(std::isfinite(v),"nonfinite controller endpoint");
      const double error=staged_motion::jointError(end,q1);
      std::cout<<"[STAGE "<<index<<"] endpoint_joint_delta_rad="<<error<<" q=[";
      for(double v:end) std::cout<<v<<' ';
      std::cout<<"]"<<std::endl;
      require(staged_motion::poseNear(pose(model.getCartPose(end)),to,.001,.005),"controller endpoint FK mismatch");
      if(error>.002) {
        failed=true;
        std::cout<<"[MISMATCH] Controller endpoint differs from sampled joint branch; candidate not certified."<<std::endl;
      } else ++accepted;
    }
    const auto after=robot.jointPos(ec); check(ec);
    require(staged_motion::jointError(before,after)<.0005,"arm changed position during diagnostic");
    const auto finalState=robot.operationState(ec); check(ec);
    require(finalState==rokae::OperationState::idle,"arm is no longer idle");
    std::cout<<"[RESULT] Controller-accepted matching linear legs="<<accepted
             <<". Whole-route collision validation NOT performed; no motion commanded."<<std::endl;
    return failed ? 2 : 0;
  } catch(const std::exception& e) {
    std::cerr<<"[ERROR] "<<e.what()<<"; no motion commanded"<<std::endl;
    return 2;
  }
}
