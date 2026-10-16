// Read-only IK comparisons. This program contains no motion or mode commands.
#include <array>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <opencv2/core.hpp>
#include "rokae/model.h"
#include "rokae/robot.h"
#include "rokae/utility.h"
#include "robot_vision_config.hpp"
#include "visual_servo_config.hpp"

using Pose=Eigen::Isometry3d;
void require(bool ok,const std::string& text) { if(!ok) throw std::runtime_error(text); }
void check(const std::error_code& ec) { require(!ec,ec.message()); }
Pose pose(const std::array<double,16>& a) {
  Pose p=Pose::Identity();
  for(int i=0;i<4;++i) for(int j=0;j<4;++j) p.matrix()(i,j)=a[i*4+j];
  require(p.matrix().allFinite() &&
          (p.linear().transpose()*p.linear()-Eigen::Matrix3d::Identity()).norm()<1e-6 &&
          std::abs(p.linear().determinant()-1)<1e-6,"invalid transform");
  return p;
}
Pose pose(const cv::Mat& m) {
  require(m.rows==4 && m.cols==4 && m.type()==CV_64F && cv::checkRange(m),"invalid matrix");
  std::array<double,16> a{};
  for(int i=0;i<4;++i) for(int j=0;j<4;++j) a[i*4+j]=m.at<double>(i,j);
  return pose(a);
}
std::array<double,6> posture(const Pose& p) {
  std::array<double,16> a{}; std::array<double,6> out{};
  for(int i=0;i<4;++i) for(int j=0;j<4;++j) a[i*4+j]=p.matrix()(i,j);
  rokae::Utils::transArrayToPosture(a,out);
  return out;
}

int main(int argc,char** argv) {
  try {
    require(argc==4,"Usage: mannequin_pose_diagnosis OLD_CAPTURE.yml OLD_GEOMETRY.yml REPORT.csv (read-only)");
    cv::FileStorage capture(argv[1],cv::FileStorage::READ);
    cv::FileStorage geometry(argv[2],cv::FileStorage::READ);
    cv::FileStorage calibration(
        "/home/arm/xCoreSDK-v0.5.1/example/function_config/LeftArm_interactive_calib.yml",cv::FileStorage::READ);
    require(capture.isOpened() && geometry.isOpened() && calibration.isOpened(),"missing input");
    cv::Mat gm,xm,pm,nm;
    capture["base_T_flange"]>>gm; calibration["H_Flange2Cam"]>>xm;
    geometry["point_C"]>>pm; geometry["outward_normal_C"]>>nm;
    require(pm.type()==CV_64F && nm.type()==CV_64F && pm.total()==3 && nm.total()==3 &&
            cv::checkRange(pm) && cv::checkRange(nm),"invalid observation");
    const Pose old=pose(gm),X=pose(xm);
    const Eigen::Vector3d P=old*X*Eigen::Vector3d(pm.at<double>(0),pm.at<double>(1),pm.at<double>(2));
    const Eigen::Vector3d N=old.linear()*X.linear()*
        Eigen::Vector3d(nm.at<double>(0),nm.at<double>(1),nm.at<double>(2));
    require(std::abs(N.norm()-1)<1e-6,"invalid normal");
    const Eigen::Matrix3d normalRotation=Eigen::Quaterniond::FromTwoVectors(old.linear().col(2),-N)
        .toRotationMatrix()*old.linear();
    rokae::xMateRobot robot(user_config::IP_LEFT,user_config::LOCAL_IP);
    std::error_code ec;
    const auto initialState=robot.operationState(ec); check(ec);
    require(initialState==rokae::OperationState::idle,"robot must be stationary for diagnosis");
    const auto q=robot.jointPos(ec); check(ec);
    rokae::xMateModel<6> model=robot.model();
    const Pose current=pose(model.getCartPose(q));
    const auto tool=robot.toolset(ec); check(ec);
    for(const auto* f:{&tool.end,&tool.ref}) {
      for(double v:f->trans) require(std::isfinite(v) && std::abs(v)<1e-8,"active nonzero tool/reference");
      for(double v:f->rpy) require(std::isfinite(v) && std::abs(v)<1e-8,"active nonzero tool/reference");
    }
    const auto base=robot.baseFrame(ec); check(ec);
    std::array<double,16> ba{}; rokae::Utils::postureToTransArray(base,ba);
    const Pose W=pose(ba);
    const auto currentRef=robot.posture(rokae::CoordinateType::endInRef,ec); check(ec);
    rokae::Utils::postureToTransArray(currentRef,ba);
    require((pose(ba).matrix()-(W*current).matrix()).norm()<0.01,"frame mismatch");
    std::ofstream file(argv[3]); require(file.good(),"cannot write report");
    const auto emit=[&](const std::string& line) { std::cout<<line<<std::endl; file<<line<<'\n'; };
    emit("# READ-ONLY: hypothetical endpoints, NOT path validation or permission to move");
    emit("case,gap_m,roll_deg,ik_code,fk_position_error_m,fk_rotation_error_rad,flange_x,flange_y,flange_z");
    const auto evaluate=[&](const std::string& name,double gap,double roll,const Pose& goal) {
      const auto now=robot.jointPos(ec); check(ec);
      for(int i=0;i<6;++i) require(std::isfinite(now[i]) && std::abs(now[i]-q[i])<0.0005,"robot moved during diagnosis");
      const auto solution=model.calcIk(rokae::CartesianPosition(posture(W*goal)),ec);
      const int code=ec.value();
      double posError=-1,rotError=-1;
      if(!ec) {
        const Pose fk=pose(model.getCartPose(solution));
        posError=(fk.translation()-goal.translation()).norm();
        rotError=Eigen::AngleAxisd(fk.linear().transpose()*goal.linear()).angle();
      }
      std::ostringstream line;
      line<<std::setprecision(10)<<name<<','<<gap<<','<<roll<<','<<code<<','<<posError<<','
          <<rotError<<','<<goal.translation().x()<<','<<goal.translation().y()<<','<<goal.translation().z();
      emit(line.str());
    };
    evaluate("current_manual_pose",-1,0,current);
    const double length=visual_servo_config::kLeftFlangeToGraspCenterM;
    for(double gap:{0.4,0.3,0.2,0.1,0.0}) {
      Pose normalGoal=Pose::Identity(); normalGoal.linear()=normalRotation;
      normalGoal.translation()=P+(gap+length)*N;
      evaluate("old_shoulder_surface_normal",gap,0,normalGoal);
      Pose manualGoal=current;
      manualGoal.translation()=P-(gap+length)*current.linear().col(2);
      evaluate("old_shoulder_manual_orientation",gap,0,manualGoal);
    }
    for(int degrees=45;degrees<360;degrees+=45) {
      Pose goal=Pose::Identity();
      goal.linear()=normalRotation*Eigen::AngleAxisd(degrees*M_PI/180,Eigen::Vector3d::UnitZ()).toRotationMatrix();
      goal.translation()=P+(0.4+length)*N;
      evaluate("old_shoulder_surface_normal_roll",0.4,degrees,goal);
    }
    cv::Mat oldJoints;
    capture["joint_positions_rad"]>>oldJoints;
    require(oldJoints.type()==CV_64F && oldJoints.total()==6 && cv::checkRange(oldJoints),
            "old joint reference missing");
    std::array<double[2],6> limits{};
    const bool limitsEnabled=robot.getSoftLimit(limits,ec); check(ec);
    require(limitsEnabled,"controller limits not enabled");
    // These are geometric probes with the CURRENT controller IK seed. They are
    // not a replay simulation from the historical joints or a collision check.
    const auto probePath=[&](const std::string& name,const Pose& goal) {
      constexpr int count=200;
      const Eigen::Quaterniond r0(old.linear()),r1(goal.linear());
      int solved=0,firstFailure=-1,firstCode=0,limitViolations=0;
      double startBranchError=-1,maxAdjacent=0,minSigma=1e9;
      std::array<double,6> previous{};
      for(int i=0;i<6;++i) previous[i]=oldJoints.ptr<double>()[i];
      bool previousValid=true;
      for(int sample=0;sample<=count;++sample) {
        const auto stationary=robot.jointPos(ec); check(ec);
        for(int i=0;i<6;++i) require(std::isfinite(stationary[i]) &&
            std::abs(stationary[i]-q[i])<0.0005,"robot moved during path diagnosis");
        const double t=static_cast<double>(sample)/count;
        Pose waypoint=Pose::Identity();
        waypoint.linear()=r0.slerp(t,r1).toRotationMatrix();
        waypoint.translation()=(1-t)*old.translation()+t*goal.translation();
        auto solution=model.calcIk(rokae::CartesianPosition(posture(W*waypoint)),ec);
        if(ec) {
          if(firstFailure<0) { firstFailure=sample; firstCode=ec.value(); }
          previousValid=false;
          continue;
        }
        ++solved;
        const Pose fk=pose(model.getCartPose(solution));
        require((fk.translation()-waypoint.translation()).norm()<0.001 &&
                Eigen::AngleAxisd(fk.linear().transpose()*waypoint.linear()).angle()<0.005,
                "path probe IK/FK mismatch");
        double adjacent=0;
        for(int i=0;i<6;++i) {
          solution[i]=previous[i]+std::remainder(solution[i]-previous[i],2*M_PI);
          adjacent=std::max(adjacent,std::abs(solution[i]-previous[i]));
          if(solution[i]<limits[i][0]+0.05 || solution[i]>limits[i][1]-0.05) ++limitViolations;
        }
        if(sample==0) startBranchError=adjacent;
        else if(previousValid) maxAdjacent=std::max(maxAdjacent,adjacent);
        const auto data=model.jacobian(solution);
        Eigen::Matrix<double,6,6,Eigen::RowMajor> jacobian;
        std::copy(data.begin(),data.end(),jacobian.data());
        minSigma=std::min(minSigma,jacobian.jacobiSvd().singularValues().minCoeff());
        previous=solution;
        previousValid=true;
      }
      std::ostringstream line;
      line<<"# POINTWISE_CURRENT_SEED "<<name<<" solved="<<solved<<"/"<<count+1
          <<" first_failed_sample="<<firstFailure<<" code="<<firstCode
          <<" start_branch_error_rad="<<startBranchError
          <<" max_adjacent_joint_change_rad="<<maxAdjacent
          <<" limit_margin_violations="<<limitViolations<<" sigma_min="<<minSigma
          <<"; NOT motion validation";
      emit(line.str());
    };
    probePath("original_start_to_manual_contact",current);
    Pose manualPregrasp=current;
    manualPregrasp.translation()=P-(0.4+length)*current.linear().col(2);
    probePath("original_start_to_shoulder_manual_direction_gap_0.4",manualPregrasp);
    require(file.good(),"report write failed");
    std::cout<<"[SAFE] no mode, power, tool setting, or motion commands sent\n";
    return 0;
  } catch(const std::exception& e) { std::cerr<<"[ERROR] "<<e.what()<<'\n'; return 2; }
}
