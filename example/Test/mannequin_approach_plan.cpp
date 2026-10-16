#include <algorithm>
#include <array>
#include <chrono>
#include <csignal>
#include <cmath>
#include <fstream>
#include <filesystem>
#include <iomanip>
#include <iostream>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <opencv2/core.hpp>
#include "robot_vision_config.hpp"
#include "visual_servo_config.hpp"
#include "rokae/model.h"
#include "rokae/robot.h"
#include "rokae/utility.h"
#include "staged_motion_replay.hpp"
#include "staged_motion_sdk.hpp"
#include "mannequin_reach_planner.hpp"
#include "mannequin_reach_execution.hpp"

namespace {
using Pose = Eigen::Isometry3d;
using Joints = std::array<double, 6>;
namespace cfg = visual_servo_config;
volatile std::sig_atomic_t stopRequested=0;
void onSignal(int) { stopRequested=1; }
void require(bool ok, const std::string& message) {
  if (!ok) throw std::runtime_error(message);
}
void check(const std::error_code& ec) {
  require(!ec, ec.message()+" (code="+std::to_string(ec.value())+")");
}
Pose fromMat(const cv::Mat& m) {
  require(m.rows == 4 && m.cols == 4 && m.type() == CV_64F && cv::checkRange(m),
          "invalid transform");
  Pose p = Pose::Identity();
  for (int i=0;i<4;++i) for (int j=0;j<4;++j) p.matrix()(i,j)=m.at<double>(i,j);
  require((p.linear().transpose()*p.linear()-Eigen::Matrix3d::Identity()).norm()<1e-6 &&
      std::abs(p.linear().determinant()-1)<1e-6 &&
      (p.matrix().row(3)-Eigen::RowVector4d(0,0,0,1)).norm()<1e-9,
      "transform is not rigid");
  return p;
}
Pose fromArray(const std::array<double,16>& a) {
  cv::Mat m(4,4,CV_64F);
  std::copy(a.begin(),a.end(),m.ptr<double>());
  return fromMat(m);
}
std::array<double,16> toArray(const Pose& p) {
  std::array<double,16> a{};
  for(int i=0;i<4;++i) for(int j=0;j<4;++j) a[i*4+j]=p.matrix()(i,j);
  return a;
}
void fresh(double stamp) {
  const double now=std::chrono::duration<double,std::milli>(
      std::chrono::system_clock::now().time_since_epoch()).count();
  require(std::isfinite(stamp) && now-stamp>=-2000 && now-stamp<=30000,
          "capture is not within 30 s");
}
Pose checkedWorldToBase(rokae::xMateRobot& robot,const Pose& flange) {
  std::error_code ec;
  const auto tool=robot.toolset(ec); check(ec);
  for(const auto* frame:{&tool.end,&tool.ref}) {
    for(double v:frame->trans) require(std::isfinite(v) && std::abs(v)<1e-8,
                                      "requires zero active tool/reference translation");
    for(double v:frame->rpy) require(std::isfinite(v) && std::abs(v)<1e-8,
                                    "requires zero active tool/reference rotation");
  }
  const auto endPosture=robot.posture(rokae::CoordinateType::endInRef,ec); check(ec);
  const auto basePosture=robot.baseFrame(ec); check(ec);
  std::array<double,16> matrix{};
  rokae::Utils::postureToTransArray(endPosture,matrix);
  const Pose endFrame=fromArray(matrix);
  rokae::Utils::postureToTransArray(basePosture,matrix);
  const Pose worldToBase=fromArray(matrix);
  const Pose expected=worldToBase*flange;
  std::cout<<"[FRAME] base_in_world=[";
  for(double v:basePosture) std::cout<<v<<' ';
  std::cout<<"] end_in_ref=[";
  for(double v:endPosture) std::cout<<v<<' ';
  std::cout<<"] expected_xyz=["<<expected.translation().transpose()<<"] m"<<std::endl;
  require((endFrame.translation()-expected.translation()).norm()<0.005 &&
          Eigen::AngleAxisd(endFrame.linear().transpose()*expected.linear()).angle()<0.01,
          "active Cartesian frame does not match world/base/tool transformation");
  return worldToBase;
}
}

int main(int argc,char** argv) {
  bool motionIssued=false;
  try {
    const std::string mode=argc>2 ? argv[2] : "";
    const bool diagnosticReplay=mode=="--diagnose-staged-ik";
    const bool reachPlan=mode=="--reach-plan";
    const bool stagedPlan=diagnosticReplay || mode=="--staged-plan";
    const bool graspPlan=mode=="--joint-grasp-plan" || mode=="--linear-grasp-plan";
    const bool graspAlign=mode=="--joint-grasp-align" || mode=="--linear-grasp-align";
    const bool zeroStandoff=graspPlan || graspAlign;
    const bool jointMotion=reachPlan || stagedPlan || mode=="--joint-pregrasp" || mode=="--joint-grasp-plan" || mode=="--joint-grasp-align";
    const bool pregrasp=zeroStandoff || jointMotion || mode=="--pregrasp";
    require(pregrasp ? (argc==4 || (argc==5 && std::string(argv[4])=="--execute")) :
            (argc==5 || (argc==6 && std::string(argv[5])=="--execute")),
            "Usage: mannequin_approach_plan CURRENT.yml X_C Y_C Z_C [--execute], or "
            "CURRENT.yml --return-to START.yml ARRIVAL.yml [--execute], or "
            "CURRENT.yml --pregrasp|--joint-pregrasp GEOMETRY.yml [--execute], or "
            "CURRENT.yml --joint-grasp-plan|--linear-grasp-plan GEOMETRY.yml (read-only), or "
            "CURRENT.yml --joint-grasp-align|--linear-grasp-align GEOMETRY.yml (execution suspended), or "
            "CURRENT.yml --staged-plan GEOMETRY.yml (read-only), or "
            "CURRENT.yml --reach-plan GEOMETRY.yml (read-only, surface +50 mm, candidate search), or "
            "SAVED.yml --diagnose-staged-ik SAVED_GEOMETRY.yml (historical diagnostic only)");
    const bool execute=std::string(argv[argc-1])=="--execute";
    require(!(reachPlan && execute),"reach candidate planning is read-only; no commissioned dispatch path");
    require(!(stagedPlan && execute),"staged planning is read-only; collision-aware path review required");
    // Containment after the 2026-10-07 unintended contact. Endpoint/FK checks
    // do not validate the swept volume of the fingers, links or environment.
    require(!(graspAlign && execute),
            "grasp alignment execution disabled after unintended contact; collision-aware path review required");
    require(!(graspPlan && execute),"zero-standoff grasp planning is read-only; no execution permitted");
    const bool returning=std::string(argv[2])=="--return-to";
    std::signal(SIGINT,onSignal);
    std::signal(SIGTERM,onSignal);
    Eigen::Vector3d point=Eigen::Vector3d::Zero();
    Eigen::Vector3d normalC=Eigen::Vector3d::Zero();
    bool demonstrated=false;
    std::string targetJoint="left_shoulder";
    if(pregrasp) {
      cv::FileStorage geometry(argv[3],cv::FileStorage::READ);
      require(geometry.isOpened(),"pregrasp geometry missing");
      std::string captureFile,serial,joint;
      geometry["capture_file"]>>captureFile;
      geometry["camera_serial"]>>serial; geometry["joint"]>>joint;
      require(captureFile==argv[1] && serial==user_config::SN_LEFT_CAM &&
              (joint=="left_shoulder" || joint=="left_elbow"),
              "pregrasp geometry does not match the capture/arm/target");
      int cocoIndex=-1; geometry["coco_index"]>>cocoIndex;
      require(cocoIndex==(joint=="left_shoulder" ? 5 : 7),"target joint/index mismatch");
      targetJoint=joint;
      require(!reachPlan || targetJoint=="left_elbow","reach mode currently targets left elbow only");
      std::string orientationSource;
      geometry["orientation_source"]>>orientationSource;
      require(orientationSource=="demonstrated" || orientationSource=="surface_normal",
              "unknown pregrasp orientation source");
      demonstrated=orientationSource=="demonstrated";
      require(!zeroStandoff || demonstrated,"grasp targeting requires the demonstrated tool orientation");
      require(!stagedPlan || demonstrated,"staged planning requires the demonstrated tool orientation");
      require(!reachPlan || demonstrated,"reach planning requires the demonstrated tool orientation");
      if(graspAlign) {
        double measuredLength=-1;
        geometry["measured_tool_length_m"]>>measuredLength;
        require(std::isfinite(measuredLength) && std::abs(measuredLength-0.300)<1e-9 &&
                std::abs(measuredLength-cfg::kLeftFlangeToGraspCenterM)<1e-9,
                "grasp alignment requires the explicit measured 300 mm tool length matching configuration");
      }
      cv::Mat pm;
      geometry["point_C"]>>pm;
      require(pm.rows==3 && pm.cols==1 && pm.type()==CV_64F && cv::checkRange(pm),"invalid point");
      for(int i=0;i<3;++i) point[i]=pm.at<double>(i);
      double p90=-1,person=-1,keypoint=-1,spread=-1;
      geometry["point_p90_m"]>>p90; geometry["person_confidence"]>>person;
      geometry["keypoint_confidence"]>>keypoint; geometry["depth_spread_m"]>>spread;
      require(std::isfinite(p90) && p90>=0 && p90<=0.02 &&
              std::isfinite(person) && person>=0.5 && person<=1 &&
              std::isfinite(keypoint) && keypoint>=0.9 && keypoint<=1 &&
              std::isfinite(spread) && spread>=0 && spread<=0.04,"unreliable visual point");
      double variation=0,rms=0; int frames=0;
      geometry["valid_frames"]>>frames;
      require(frames>=8 && frames<=10,"insufficient reliable frames");
      if(!demonstrated) {
        cv::Mat nm; geometry["outward_normal_C"]>>nm;
        require(nm.rows==3 && nm.cols==1 && nm.type()==CV_64F && cv::checkRange(nm),"invalid normal");
        for(int i=0;i<3;++i) normalC[i]=nm.at<double>(i);
        geometry["normal_p90_deg"]>>variation; geometry["fit_rms_max_m"]>>rms;
        require(std::isfinite(variation) && variation>=0 && variation<=8 &&
              std::isfinite(rms) && rms>=0 && rms<=0.005 && frames>=8 &&
              std::abs(normalC.norm()-1)<1e-6 && normalC.dot(point)<0,
              "unreliable pregrasp normal");
      }
    }
    if(!returning) {
      if(!pregrasp) point=Eigen::Vector3d(std::stod(argv[2]),std::stod(argv[3]),std::stod(argv[4]));
      // Close-up data is accepted only in the non-executable grasp planning mode.
      require(point.allFinite() && point.z()>=(graspPlan || reachPlan ? 0.35 : 1.0) && point.z()<=3.0 &&
              point.head<2>().norm()<0.5,"invalid close-to-axis observation");
    }
    cv::FileStorage capture(argv[1],cv::FileStorage::READ);
    require(capture.isOpened(),"cannot open paired capture");
    std::string serial,arm,stampText;
    capture["camera_serial"]>>serial; capture["robot_arm"]>>arm;
    require(capture["capture_unix_ms"].isString(),"capture timestamp text missing");
    capture["capture_unix_ms"]>>stampText;
    std::size_t consumed=0;
    const double stamp=std::stod(stampText,&consumed);
    require(consumed==stampText.size(),"invalid capture timestamp");
    if(!diagnosticReplay) fresh(stamp);
    else std::cout<<"[DIAGNOSTIC] historical observation replay; no new capture; NOT a current scene plan; "
                     "execution prohibited"<<std::endl;
    require(arm=="left" && serial==user_config::SN_LEFT_CAM,"left-arm capture required");
    cv::Mat gm,qm;
    capture["base_T_flange"]>>gm; capture["joint_positions_rad"]>>qm;
    const Pose capturedFlange=fromMat(gm);
    require(qm.type()==CV_64F && qm.total()==6 && cv::checkRange(qm),"invalid paired joints");
    Joints capturedJoints{};
    std::copy(qm.ptr<double>(),qm.ptr<double>()+6,capturedJoints.begin());
    cv::FileStorage calibration(
        "/home/arm/xCoreSDK-v0.5.1/example/function_config/LeftArm_interactive_calib.yml",
        cv::FileStorage::READ);
    cv::FileStorage validated(
        "/home/arm/xCoreSDK-v0.5.1/example/function_config/calibration_runs/"
        "left_validation_20260930_i2JHhp/calibration_used.yml",cv::FileStorage::READ);
    require(calibration.isOpened() && validated.isOpened(),"calibration/reference missing");
    cv::Mat xm,reference;
    calibration["H_Flange2Cam"]>>xm; validated["H_Flange2Cam"]>>reference;
    calibration["camera_serial"]>>serial;
    require(serial==user_config::SN_LEFT_CAM && xm.size()==reference.size() &&
            xm.type()==reference.type() && cv::norm(xm,reference)<1e-10,
            "calibration differs from three-pose validation");
    const Pose X=fromMat(xm);
    rokae::xMateRobot robot(user_config::IP_LEFT,user_config::LOCAL_IP);
    std::error_code ec;
    const auto info=robot.robotInfo(ec); check(ec);
    require(info.type.find("XMC18")!=std::string::npos,"unexpected robot");
    const auto state=robot.operationState(ec); check(ec);
    require(state==rokae::OperationState::idle,"robot is not idle");
    const auto start=robot.jointPos(ec); check(ec);
    for(int i=0;i<6;++i) require(std::abs(start[i]-capturedJoints[i])<0.0005,
                               "robot changed posture after capture");
    rokae::xMateModel<6> model=robot.model();
    const Pose G=fromArray(model.getCartPose(start));
    require((G.translation()-capturedFlange.translation()).norm()<0.005 &&
       Eigen::AngleAxisd(G.linear().transpose()*capturedFlange.linear()).angle()<0.01,
       "controller/model pose mismatch");
    const Eigen::Vector3d targetBase=G*X*point;
    const double cameraStop=cfg::kEstimatedToolProtrusionFromCameraM+
        cfg::kLeftToolProtrusionUncertaintyM+cfg::kMannequinNonContactClearanceM+
        cfg::kMannequinPerceptionReserveM;
    Pose target=G;
    const Eigen::Vector3d toolOffset(0,0,cfg::kLeftFlangeToGraspCenterM);
    Eigen::Vector3d normalBase=G.linear()*X.linear()*normalC;
    const double standOff=reachPlan ? -cfg::kLeftElbowReachInsertionM : (zeroStandoff ? 0.0 : (cfg::kMannequinNonContactClearanceM+
        cfg::kLeftToolProtrusionUncertaintyM+cfg::kMannequinPerceptionReserveM));
    if(graspPlan) std::cout<<"[GRASP-PLAN] grasp center = fresh visual point; zero standoff; "
                            "provisional TCP, NOT validated for contact or execution"<<std::endl;
    if(graspAlign) std::cout<<"[GRASP-ALIGN] measured flange-Z tool=0.300 m; zero target standoff; "
                             "NO gripper IO; requires onsite swept-space supervision"<<std::endl;
    Joints returnJoints{};
    if(returning) {
      // Historical poses are reference endpoints, not fresh scene observations.
      const auto readReference=[&](const char* file) {
        cv::FileStorage fs(file,cv::FileStorage::READ);
        require(fs.isOpened(),"cannot open return reference");
        std::string refArm,refSerial;
        fs["robot_arm"]>>refArm; fs["camera_serial"]>>refSerial;
        require(refArm=="left" && refSerial==user_config::SN_LEFT_CAM,
                "return reference must belong to left arm/camera");
        cv::Mat refQ,refG;
        fs["joint_positions_rad"]>>refQ; fs["base_T_flange"]>>refG;
        require(refQ.type()==CV_64F && refQ.total()==6 && cv::checkRange(refQ),
                "invalid reference joints");
        Joints q{};
        std::copy(refQ.ptr<double>(),refQ.ptr<double>()+6,q.begin());
        const auto pose=fromMat(refG);
        const auto fk=fromArray(model.getCartPose(q));
        require((fk.translation()-pose.translation()).norm()<0.005 &&
                Eigen::AngleAxisd(fk.linear().transpose()*pose.linear()).angle()<0.01,
                "reference model/controller mismatch");
        return std::make_pair(fk,q);
      };
      const auto original=readReference(argv[3]);
      const auto arrival=readReference(argv[4]);
      for(int i=0;i<6;++i) require(std::abs(start[i]-arrival.second[i])<0.002,
                                  "robot no longer at saved approach endpoint");
      require((G.translation()-arrival.first.translation()).norm()<0.003,
              "current pose differs from saved approach endpoint");
      require(Eigen::AngleAxisd(G.linear().transpose()*original.first.linear()).angle()<0.001,
              "return endpoints do not have the same orientation");
      target.translation()=original.first.translation();
      returnJoints=original.second;
    } else if(pregrasp) {
      if(demonstrated) {
        cv::FileStorage demo(cfg::kLeftDemonstratedPoseFile,cv::FileStorage::READ);
        require(demo.isOpened(),"saved manual demonstration missing");
        std::string demoArm,demoSerial;
        demo["robot_arm"]>>demoArm; demo["camera_serial"]>>demoSerial;
        require(demoArm=="left" && demoSerial==user_config::SN_LEFT_CAM,"wrong demonstration arm");
        cv::Mat demoG,demoQ;
        demo["base_T_flange"]>>demoG; demo["joint_positions_rad"]>>demoQ;
        const Pose reference=fromMat(demoG);
        require(demoQ.type()==CV_64F && demoQ.total()==6 && cv::checkRange(demoQ),"invalid demonstrated joints");
        Joints q{}; std::copy(demoQ.ptr<double>(),demoQ.ptr<double>()+6,q.begin());
        const Pose fk=fromArray(model.getCartPose(q));
        require((fk.translation()-reference.translation()).norm()<0.005 &&
                Eigen::AngleAxisd(fk.linear().transpose()*reference.linear()).angle()<0.01,
                "demonstration model/frame mismatch");
        target.linear()=reference.linear();
        normalBase=-target.linear().col(2);
      } else {
        const Eigen::Quaterniond alignment=Eigen::Quaterniond::FromTwoVectors(G.linear().col(2),-normalBase);
        target.linear()=alignment.toRotationMatrix()*G.linear();
      }
      const double rotation=Eigen::AngleAxisd(G.linear().transpose()*target.linear()).angle();
      require(rotation<=M_PI/2,
              "pregrasp orientation change exceeds 90-degree planning envelope");
      if(execute) require(rotation<=cfg::kLeftPregraspMaxRotationRad,
                          "pregrasp orientation exceeds configured onsite-clearance envelope");
      const Eigen::Vector3d graspTarget=targetBase+standOff*normalBase;
      target.translation()=graspTarget-target.linear()*toolOffset;
      require(((G*toolOffset)-targetBase).dot(normalBase)>standOff+0.05,
              "starting grasp center is already too close to target surface");
      std::cout<<(reachPlan ? "[GRASP GOAL] source=" : "[PREGRASP] source=")<<(demonstrated ? "manual demonstration" : "surface normal")
               <<", provisional TCP_F=["<<toolOffset.transpose()
               <<"] m, retreat_direction_B=["<<normalBase.transpose()
               <<"], grasp_center_B=["<<graspTarget.transpose()
               <<"] m, estimated standoff="<<standOff<<" m, rotation="
               <<rotation*180/M_PI<<" deg"<<std::endl;
    } else {
      target.translation()+=G.linear()*X.linear()*(point-Eigen::Vector3d(0,0,cameraStop));
    }
    const double length=(target.translation()-G.translation()).norm();
    require(std::isfinite(length) && length>0 && length<1.5,"invalid path length");
    if(!returning) std::cout<<"[TARGET] "<<targetJoint<<"_B=["<<targetBase.transpose()<<"] m\n";
    std::cout<<"[TARGET] "<<(returning ? "original start" : "approach")
             <<" flange_B=["<<target.translation().transpose()<<"] m\n"
             <<"[PLAN] flange distance="<<length<<" m, "
             <<(pregrasp ? (demonstrated ? "demonstrated orientation" : "surface-normal orientation")
                          : "fixed orientation")<<"\n";
    const Eigen::Quaterniond startRotation(G.linear()),targetRotation(target.linear());
    const auto plannedPose=[&](double fraction) {
      Pose p=G;
      p.translation()=G.translation()+fraction*(target.translation()-G.translation());
      p.linear()=startRotation.slerp(fraction,targetRotation).toRotationMatrix();
      return p;
    };
    std::array<double[2],6> limits{};
    const bool enabled=robot.getSoftLimit(limits,ec); check(ec);
    require(enabled,"controller soft limits disabled");
    for(int i=0;i<6;++i) std::cout<<"[LIMIT] J"<<i+1<<" ["<<limits[i][0]<<", "
        <<limits[i][1]<<"] rad; current="<<start[i]<<std::endl;
    const Pose worldToBase=checkedWorldToBase(robot,G);
    if(reachPlan) {
      require(std::abs(cfg::kLeftElbowReachInsertionM-.05)<1e-9,"reach target must be the requested 50 mm insertion");
      rokae::xMateRobot other(user_config::IP_RIGHT,user_config::LOCAL_IP);
      const auto otherStart=other.jointPos(ec); check(ec);
      const auto otherState=other.operationState(ec); check(ec);
      require(otherState==rokae::OperationState::idle,"other arm is not idle");
      mannequin_reach::Limits reachLimits{};
      for(int i=0;i<6;++i) reachLimits[i]={limits[i][0],limits[i][1]};
      mannequin_reach::Planner planner(model,reachLimits,worldToBase,targetBase,target,
                                       cfg::kLeftElbowReachInsertionM,[]{return stopRequested!=0;});
      const auto candidates=planner.search(start);
      const auto chosen=mannequin_reach::best(candidates);
      fresh(stamp);
      const auto unchanged=robot.jointPos(ec); check(ec);
      require(staged_motion::jointError(unchanged,start)<.0005,"left arm moved during search");
      require(robot.operationState(ec)==rokae::OperationState::idle,"left arm no longer idle"); check(ec);
      const auto otherEnd=other.jointPos(ec); check(ec);
      require(staged_motion::jointError(otherEnd,otherStart)<.0005,"right arm moved during search");
      require(other.operationState(ec)==rokae::OperationState::idle,"right arm no longer idle"); check(ec);
      require(!stopRequested,"interrupted before report");
      mannequin_reach::write(argv[3],candidates,chosen,targetBase,target,worldToBase,X,otherStart,
                             argv[1],stampText,cfg::kLeftElbowReachInsertionM);
      const auto executionPlan=mannequin_reach::executionPlan(candidates[chosen],reachLimits,
          otherStart,targetBase,cfg::kLeftElbowReachInsertionM,stamp);
      staged_motion::Replay replay(executionPlan,[&](const Joints& q){return fromArray(model.getCartPose(q));});
      staged_motion::run(executionPlan,replay,[]{return stopRequested!=0;});
      require(replay.finished && replay.commands==static_cast<int>(executionPlan.stages.size()),
              "reach supervisor replay did not complete every stage");
      std::cout<<"[REPLAY COMPLETE] "<<replay.commands
               <<" virtual commands, arrival and shutdown verified; no hardware dispatch"<<std::endl;
      return 0;
    }
    Joints jointGoal{},jointDelta{};
    double jointDistanceSquared=0,maxJointChange=0;
    if(pregrasp) {
      std::array<double,6> endpointPosture{};
      rokae::Utils::transArrayToPosture(toArray(worldToBase*target),endpointPosture);
      const auto endpoint=model.calcIk(rokae::CartesianPosition(endpointPosture),ec);
      require(!ec,"pregrasp ENDPOINT IK rejected: "+ec.message()+
              " (code="+std::to_string(ec.value())+"); requested endpoint is not solved by the controller");
      const auto endpointFk=fromArray(model.getCartPose(endpoint));
      require((endpointFk.translation()-target.translation()).norm()<0.001 &&
              Eigen::AngleAxisd(endpointFk.linear().transpose()*target.linear()).angle()<0.005,
              "pregrasp endpoint IK/FK mismatch");
      if(jointMotion) {
        // Select nearby legal turns without changing the controller's IK branch.
        for(int i=0;i<6;++i) {
          require(std::isfinite(endpoint[i]),"nonfinite endpoint joint");
          double bestDistance=1e9;
          for(int turns=-2;turns<=2;++turns) {
            const double candidate=endpoint[i]+turns*2*M_PI;
            if(candidate>=limits[i][0]+0.05 && candidate<=limits[i][1]-0.05 &&
               std::abs(candidate-start[i])<bestDistance) {
              jointGoal[i]=candidate; bestDistance=std::abs(candidate-start[i]);
            }
          }
          require(bestDistance<1e9,"no legal endpoint turn for J"+std::to_string(i+1));
          jointDelta[i]=jointGoal[i]-start[i];
          maxJointChange=std::max(maxJointChange,std::abs(jointDelta[i]));
          jointDistanceSquared+=jointDelta[i]*jointDelta[i];
          std::cout<<"[JOINT] J"<<i+1<<" target="<<jointGoal[i]
                   <<" change_deg="<<jointDelta[i]*180/M_PI<<'\n';
        }
        require(jointDistanceSquared>1e-10,"already at joint target; no motion needed");
        const auto goalFk=fromArray(model.getCartPose(jointGoal));
        require((goalFk.translation()-target.translation()).norm()<0.001 &&
                Eigen::AngleAxisd(goalFk.linear().transpose()*target.linear()).angle()<0.005,
                "joint endpoint turn selection changed target pose");
      }
      std::cout<<"[ENDPOINT] controller IK/FK passed"<<std::endl;
    }
    const auto jointAt=[&](double fraction) {
      Joints q{};
      for(int i=0;i<6;++i) q[i]=start[i]+fraction*jointDelta[i];
      return q;
    };
    const int samples=jointMotion ? std::max(100,static_cast<int>(std::ceil(maxJointChange/0.005))) :
                                   std::max(2,static_cast<int>(std::ceil(length/0.004)));
    std::cout<<"[PATH] "<<(jointMotion ? "joint interpolation, curved flange path" : "Cartesian straight line")
             <<"; sampled FK/workspace checks are NOT full-body collision detection"<<std::endl;
    Joints previous=start;
    std::vector<Joints> path;
    double minSigma=1e9,maxCondition=0;
    std::ofstream stagedCsv;
    const std::string reportPrefix=pregrasp ? std::string(argv[3])+
        (diagnosticReplay ? ".seeded_diagnostic" : "") : "";
    const std::string fullPlanPath=reportPrefix+".full_plan.json";
    if(stagedPlan) {
      require(!std::filesystem::exists(fullPlanPath),"full plan already exists; use a new capture/plan ID");
      stagedCsv.open(reportPrefix+".staged.csv");
      require(stagedCsv.good(),"cannot create staged plan report");
      stagedCsv<<"# READ-ONLY; NOT collision clearance; INCOMPLETE unless COMPLETE marker present\n"
               <<"stage,fraction,q1,q2,q3,q4,q5,q6,flange_x,flange_y,flange_z,"
                 "grasp_x,grasp_y,grasp_z,axis_x,axis_y,axis_z,signed_gap_m\n";
      stagedCsv<<std::setprecision(17);
    }
    const auto recordSample=[&](const char* stage,double t,const Joints& q,const Pose& pose) {
      if(!stagedPlan) return;
      const Eigen::Vector3d grasp=pose*toolOffset;
      stagedCsv<<stage<<','<<t;
      for(double angle:q) stagedCsv<<','<<angle;
      for(int i=0;i<3;++i) stagedCsv<<','<<pose.translation()[i];
      for(int i=0;i<3;++i) stagedCsv<<','<<grasp[i];
      for(int i=0;i<3;++i) stagedCsv<<','<<pose.linear()(i,2);
      stagedCsv<<','<<(grasp-targetBase).dot(normalBase)<<'\n';
      require(stagedCsv.good(),"staged plan report write failed");
    };
    const auto validateSample=[&](const Joints& q,const Joints& prior,const Pose& waypoint,
                                  double minimumGap,const std::string& label) {
      const auto p=waypoint.translation();
      if(pregrasp) require(((waypoint*toolOffset)-targetBase).dot(normalBase)>=minimumGap-1e-6,
                            "planned tool violates surface standoff at "+label);
      require(std::hypot(p.x(),p.y())<=cfg::kWorkspaceRadiusMaxM &&
          p.z()>=cfg::kWorkspaceZMinM && p.z()<=cfg::kWorkspaceZMaxM,
          "path outside workspace at "+label);
      for(int i=0;i<6;++i) {
        require(std::isfinite(q[i]) && q[i]>=limits[i][0]+0.05 && q[i]<=limits[i][1]-0.05,
                "controller joint-limit margin J"+std::to_string(i+1)+" at "+label);
        require(std::abs(q[i]-prior[i])<=cfg::kMaxPredictedJointStepRad,
                "IK branch jump at "+label);
      }
      const Pose actual=fromArray(model.getCartPose(q));
      require((actual.translation()-waypoint.translation()).norm()<0.001 &&
          Eigen::AngleAxisd(actual.linear().transpose()*waypoint.linear()).angle()<0.005,
          "IK/FK mismatch at "+label);
      const auto data=model.jacobian(q);
      Eigen::Matrix<double,6,6,Eigen::RowMajor> J;
      std::copy(data.begin(),data.end(),J.data());
      const auto sv=J.jacobiSvd().singularValues();
      const double sigma=sv.minCoeff(),condition=sv.maxCoeff()/sigma;
      minSigma=std::min(minSigma,sigma); maxCondition=std::max(maxCondition,condition);
      require(sigma>=cfg::kMinJacobianSingularValue && condition<=cfg::kMaxJacobianCondition,
          "path near singularity at "+label+" sigma="+std::to_string(sigma)+
          " condition="+std::to_string(condition));
      return actual;
    };
    for(int sample=0;sample<=samples;++sample) {
      require(!stopRequested,"interrupted during planning");
      const double fraction=static_cast<double>(sample)/samples;
      Joints q=jointMotion ? jointAt(fraction) : start;
      const Pose waypoint=jointMotion ? fromArray(model.getCartPose(q)) : plannedPose(fraction);
      if(sample && !jointMotion) {
        // Use controller IK in its active reference frame, then cross-check local FK.
        std::array<double,6> waypointPosture{};
        rokae::Utils::transArrayToPosture(toArray(worldToBase*waypoint),waypointPosture);
        q=model.calcIk(rokae::CartesianPosition(waypointPosture),ec);
        require(!ec,"controller IK at sample "+std::to_string(sample)+": "+
                ec.message()+" (code="+std::to_string(ec.value())+")");
        for(int i=0;i<6;++i) q[i]=previous[i]+std::remainder(q[i]-previous[i],2*M_PI);
      }
      const Pose actual=validateSample(q,previous,waypoint,standOff,
          "transit sample "+std::to_string(sample)+"/"+std::to_string(samples));
      recordSample("joint_transit",fraction,q,actual);
      previous=q;
      path.push_back(q);
    }
    if(!diagnosticReplay) fresh(stamp);
    if(returning) for(int i=0;i<6;++i) require(std::abs(previous[i]-returnJoints[i])<0.005,
                                             "return IK does not match original joint branch");
    const auto end=robot.jointPos(ec); check(ec);
    for(int i=0;i<6;++i) require(std::abs(end[i]-start[i])<0.0005,"robot moved during planning");
    std::cout<<"[KINEMATICS] "<<samples+1<<" samples passed sigma_min="<<minSigma
             <<" condition_max="<<maxCondition<<"\n[END-JOINTS]";
    for(double q:previous) std::cout<<' '<<q;
    std::cout<<std::endl;
    rokae::xMateRobot other(user_config::IP_RIGHT,user_config::LOCAL_IP);
    const auto otherStart=other.jointPos(ec); check(ec);
    const auto otherState=other.operationState(ec); check(ec);
    require(otherState==rokae::OperationState::idle,"other arm is not idle");
    const Pose commandTarget=worldToBase*target;
    if(stagedPlan) {
      // Solve stage two entirely against its preceding virtual joint state.
      // calcIk cannot accept that seed and must not be used as this path solver.
      const double insertion=targetJoint=="left_elbow" ? cfg::kLeftElbowSurfaceInsertionM : 0.0;
      require(std::isfinite(insertion) && insertion>=0 && insertion<=0.080,
              "surface insertion outside the reviewed planning range");
      const double approachDistance=standOff+insertion;
      const Eigen::Vector3d finalGraspTarget=targetBase-insertion*normalBase;
      const int approachSamples=std::max(2,static_cast<int>(std::ceil(approachDistance/0.004)));
      int localIkCount=0;
      std::vector<Joints> approachPath;
      const auto solveSeeded=[&](const Pose& waypoint,const Joints& prior,int sample) {
        Joints seed{},solution{};
        for(int i=0;i<6;++i) seed[i]=std::remainder(prior[i],2*M_PI);
        const int code=model.getJointPos(toArray(waypoint),0.0,seed,solution);
        require(code==0,"continuous local SDK IK at linear sample "+std::to_string(sample)+
                       " failed (code="+std::to_string(code)+
                       "); no fallback to live-pose controller selection");
        for(int i=0;i<6;++i) {
          require(std::isfinite(solution[i]),"nonfinite seeded IK solution");
          solution[i]=prior[i]+std::remainder(solution[i]-prior[i],2*M_PI);
        }
        return solution;
      };
      const Joints boundary=solveSeeded(target,jointGoal,0);
      validateSample(boundary,jointGoal,target,standOff,"stage boundary self-check");
      for(int i=0;i<6;++i) require(std::abs(boundary[i]-jointGoal[i])<0.0005,
                                  "seeded boundary IK does not preserve first-stage endpoint");
      std::cout<<"[STAGE 1] joint transit passed: "<<samples+1<<" samples; pregrasp gap="
               <<standOff<<" m\n[STAGE 2] fixed orientation, linear approach="<<approachDistance
               <<" m; surface insertion="<<insertion
               <<" m; seed=stage-one endpoint, then previous sample; boundary round-trip passed"<<std::endl;
      for(int sample=0;sample<=approachSamples;++sample) {
        require(!stopRequested,"interrupted during staged planning");
        const double t=static_cast<double>(sample)/approachSamples;
        Pose waypoint=target;
        waypoint.translation()-=approachDistance*t*normalBase;
        Joints q=previous;
        if(sample) {
          q=solveSeeded(waypoint,previous,sample);
          ++localIkCount;
        }
        const Pose actual=validateSample(q,previous,waypoint,-insertion,
            "linear approach sample "+std::to_string(sample)+"/"+std::to_string(approachSamples));
        recordSample("linear_approach",t,q,actual);
        approachPath.push_back(q);
        previous=q;
      }
      if(!diagnosticReplay) fresh(stamp);
      const auto unchanged=robot.jointPos(ec); check(ec);
      for(int i=0;i<6;++i) require(std::isfinite(unchanged[i]) && std::abs(unchanged[i]-start[i])<0.0005,
                                  "robot moved during staged planning");
      const auto leftFinalState=robot.operationState(ec); check(ec);
      require(leftFinalState==rokae::OperationState::idle,"left arm is no longer idle");
      const auto otherEnd=other.jointPos(ec); check(ec);
      for(int i=0;i<6;++i) require(std::isfinite(otherEnd[i]) && std::abs(otherEnd[i]-otherStart[i])<0.0005,
                                  "other arm moved during staged planning");
      const auto rightFinalState=other.operationState(ec); check(ec);
      require(rightFinalState==rokae::OperationState::idle,"right arm is no longer idle");
      stagedCsv<<"# COMPLETE: kinematic checks only; NOT approved for execution\n";
      stagedCsv.flush(); require(stagedCsv.good(),"staged report flush failed");
      stagedCsv.close(); require(!stagedCsv.fail(),"staged report close failed");

      // Commit a single plan only after BOTH stages and the stationary checks
      // pass. This bundle is an immutable observation/route, not motion authority.
      const auto matrixVector=[&](const Pose& pose) {
        const auto a=toArray(pose);
        return std::vector<double>(a.begin(),a.end());
      };
      const auto jointVector=[](const Joints& q) { return std::vector<double>(q.begin(),q.end()); };
      Pose finalTarget=target;
      finalTarget.translation()-=approachDistance*normalBase;
      cv::FileStorage plan(fullPlanPath+".partial",cv::FileStorage::WRITE | cv::FileStorage::FORMAT_JSON);
      require(plan.isOpened(),"cannot write full plan manifest");
      plan<<"schema_version"<<3<<"status"<<(diagnosticReplay ? "diagnostic_kinematic_complete" : "kinematic_plan_complete");
      plan<<"diagnostic_replay"<<(diagnosticReplay ? 1 : 0);
      plan<<"robot_motion_permitted"<<0<<"collision_validation"<<"not_performed";
      plan<<"controller_linear_ik_all_passed"<<0<<"controller_linear_ik_queries"<<0;
      plan<<"controller_linear_ik_status"<<"not_queried";
      plan<<"linear_ik_solver"<<"seeded_local_sdk";
      plan<<"linear_ik_seed_policy"<<"stage1_endpoint_then_previous_sample";
      plan<<"boundary_ik_round_trip_passed"<<1;
      plan<<"local_ik_sample_count"<<localIkCount;
      plan<<"observation_rounds"<<(diagnosticReplay ? 0 : 1)<<"reidentify_between_stages"<<0;
      plan<<"capture_file"<<argv[1]<<"geometry_file"<<argv[3]<<"capture_unix_ms"<<stampText;
      plan<<"sample_csv"<<(reportPrefix+".staged.csv");
      plan<<"robot_arm"<<"left"<<"camera_serial"<<user_config::SN_LEFT_CAM<<"joint"<<targetJoint;
      plan<<"target_point_base_m"<<std::vector<double>{targetBase.x(),targetBase.y(),targetBase.z()};
      plan<<"target_point_semantics"<<"observed_surface";
      plan<<"surface_insertion_m"<<insertion<<"insertion_direction"<<"approach_positive_flange_z";
      plan<<"grasp_target_base_m"<<std::vector<double>{finalGraspTarget.x(),finalGraspTarget.y(),finalGraspTarget.z()};
      plan<<"tool_offset_flange_m"<<std::vector<double>{0.0,0.0,cfg::kLeftFlangeToGraspCenterM};
      plan<<"flange_T_camera"<<matrixVector(X)<<"world_T_base"<<matrixVector(worldToBase);
      plan<<"right_arm_reference_joints_rad"<<jointVector(otherStart);
      plan<<"minimum_jacobian_sigma"<<minSigma<<"maximum_jacobian_condition"<<maxCondition;
      plan<<"stages"<<"[";
      plan<<"{"<<"name"<<"joint_transit"<<"command"<<"MoveAbsJ";
      plan<<"start_joints_rad"<<jointVector(start)<<"end_joints_rad"<<jointVector(jointGoal);
      plan<<"start_flange_base"<<matrixVector(G)<<"end_flange_base"<<matrixVector(target);
      plan<<"joint_speed_fraction"<<0.01<<"end_gap_m"<<standOff;
      plan<<"sample_count"<<(samples+1)<<"stop_at_endpoint"<<1<<"}";
      plan<<"{"<<"name"<<"linear_approach"<<"command"<<"MoveL";
      plan<<"start_joints_rad"<<jointVector(jointGoal)<<"end_joints_rad"<<jointVector(previous);
      plan<<"start_flange_base"<<matrixVector(target)<<"end_flange_base"<<matrixVector(finalTarget);
      plan<<"start_gap_m"<<standOff<<"end_gap_m"<<-insertion<<"fixed_orientation"<<1;
      plan<<"approach_distance_m"<<approachDistance;
      plan<<"linear_speed_mm_s"<<5.0<<"sample_count"<<(approachSamples+1)<<"stop_at_endpoint"<<1<<"}";
      plan<<"]"<<"gripper_command"<<"none";
      plan<<"invalidated_by"<<"["<<"target_motion"<<"base_motion"<<"camera_or_tool_mount_change"
          <<"scene_obstruction_change"<<"unexpected_robot_motion"<<"contact_or_estop"<<"]";
      plan.release();
      std::filesystem::rename(fullPlanPath+".partial",fullPlanPath);
      staged_motion::Plan executionPlan;
      executionPlan.captureUnixMs=stamp;
      executionPlan.rightReference=otherStart;
      executionPlan.target=targetBase;
      executionPlan.insertion=insertion;
      executionPlan.tool=toolOffset;
      for(int i=0;i<6;++i) executionPlan.limits[i]={limits[i][0],limits[i][1]};
      executionPlan.stages[0]={false,G,target,path,.01};
      executionPlan.stages[1]={true,target,finalTarget,approachPath,5};
      staged_motion::Replay replay(executionPlan,[&](const Joints& q) {
        return fromArray(model.getCartPose(q));
      });
      staged_motion::run(executionPlan,replay,[]{return stopRequested!=0;});
      require(replay.commands==2 && replay.finished && !replay.aborted,"staged supervisor replay incomplete");
      std::ofstream rehearsal(reportPrefix+".supervisor_replay.txt");
      require(rehearsal.good(),"cannot write supervisor replay report");
      rehearsal<<"PASS: two-stage supervisor replay against local FK; virtual telemetry/time only\n"
               <<"SDK commands dispatched: 0\n"
               <<"Controller MoveL acceptance: NOT TESTED\n"
               <<"Collision clearance: NOT VALIDATED\n"
               <<"Hardware execution remains disabled after incident\n";
      rehearsal.close(); require(!rehearsal.fail(),"supervisor replay report write failed");
      std::cout<<"[REPLAY] two-stage supervision passed with local FK; simulated telemetry, NO hardware dispatch\n";
      // This branch remains behind the early incident hold. Keep the SDK
      // adapter compiled, but do not turn replay success into motion authority.
      if(execute) {
        staged_motion::SdkBackend backend(robot,other,model,worldToBase,[]{return stopRequested!=0;});
        motionIssued=true;
        staged_motion::run(executionPlan,backend,[]{return stopRequested!=0;});
      }
      std::cout<<"[STAGED PLAN] both stages passed; sigma_min="<<minSigma
               <<" condition_max="<<maxCondition<<" local_ik_samples="<<localIkCount
               <<"\n[REPORT] "<<reportPrefix<<".staged.csv\n[FULL PLAN] "<<fullPlanPath<<'\n'
               <<"[READ-ONLY] no motion/mode/power commands; collision clearance NOT established"<<std::endl;
      return 0;
    }
    if (!execute) {
      std::cout<<"[READ-ONLY] no robot mode/power/motion changed; environment and self-collision not certified\n";
      return 0;
    }
    const auto shutdown=[&]() {
      bool ok=true;
      const auto report=[&](const char* action) {
        if(ec) { ok=false; std::cerr<<"[CLEANUP] "<<action<<": "<<ec.message()
                                  <<" ("<<ec.value()<<")\n"; }
      };
      robot.stop(ec); report("stop");
      robot.setMotionControlMode(rokae::MotionControlMode::Idle,ec); report("Idle");
      robot.setPowerState(false,ec); report("power off");
      robot.setOperateMode(rokae::OperateMode::manual,ec); report("manual");
      return ok;
    };
    try {
      fresh(stamp);
      require(!stopRequested,"interrupted before setup");
      robot.setOperateMode(rokae::OperateMode::automatic,ec); check(ec);
      robot.setPowerState(true,ec); check(ec);
      robot.setMotionControlMode(rokae::MotionControlMode::NrtCommand,ec); check(ec);
      robot.moveReset(ec); check(ec);
      robot.setDefaultSpeed(10.0,ec); check(ec);
      robot.setDefaultZone(0,ec); check(ec);
      const auto before=robot.jointPos(ec); check(ec);
      for(int i=0;i<6;++i) require(std::abs(before[i]-start[i])<0.0005,
                                  "robot changed posture before command");
      const auto otherBefore=other.jointPos(ec); check(ec);
      for(int i=0;i<6;++i) require(std::isfinite(otherBefore[i]) &&
          std::abs(otherBefore[i]-otherStart[i])<0.0005,"other arm moved before command");
      const auto otherBeforeState=other.operationState(ec); check(ec);
      require(otherBeforeState==rokae::OperationState::idle,"other arm is no longer idle");
      fresh(stamp);
      require(!stopRequested,"interrupted before command");
      std::array<double,6> posture{};
      rokae::Utils::transArrayToPosture(toArray(commandTarget),posture);
      rokae::CartesianPosition goal(posture);
      std::string pathId;
      if(jointMotion) {
        rokae::MoveAbsJCommand command(std::vector<double>(jointGoal.begin(),jointGoal.end()),10.0,0.0);
        command.jointSpeed=graspAlign ? 0.005 : 0.01;
        robot.moveAppend(command,pathId,ec); check(ec);
      } else {
        robot.moveAppend(rokae::MoveLCommand(goal,graspAlign ? 5.0 : 10.0,0.0),pathId,ec); check(ec);
      }
      require(!stopRequested,"interrupted before moveStart");
      std::cout<<"[EXECUTE] left "<<(returning ? "RETURN" : (pregrasp ? "PREGRASP" : "APPROACH"))
               <<(jointMotion ? (graspAlign ? " MoveAbsJ joint speed=0.5%" : " MoveAbsJ joint speed=1%") :
                                (graspAlign ? " MoveL speed=5 mm/s" : " MoveL speed=10 mm/s"))
               <<"; endpoint displacement="<<length<<" m; no gripper action"<<std::endl;
      motionIssued=true;
      robot.moveStart(ec); check(ec);
      const auto began=std::chrono::steady_clock::now();
      const auto deadline=began+std::chrono::seconds(jointMotion ? (graspAlign ? 720 : 360) :
          static_cast<int>(std::ceil(length/(graspAlign ? 0.005 : 0.01)))+30);
      auto nextReport=began;
      auto nextOtherCheck=began;
      auto previousTime=began;
      Joints previousActual=start;
      bool arrived=false;
      double maximumProgress=0;
      const Eigen::Vector3d direction=(target.translation()-G.translation())/length;
      while(std::chrono::steady_clock::now()<deadline) {
        require(!stopRequested,"interrupted during motion");
        const auto q=robot.jointPos(ec); check(ec);
        for(int i=0;i<6;++i) require(std::isfinite(q[i]) &&
            q[i]>=limits[i][0]+0.04 && q[i]<=limits[i][1]-0.04,
            "actual joint-limit margin J"+std::to_string(i+1));
        const auto actual=fromArray(model.getCartPose(q));
        const auto now=std::chrono::steady_clock::now();
        const double dt=std::chrono::duration<double>(now-previousTime).count();
        if(jointMotion) {
          for(int i=0;i<6;++i) require(dt<=0.02 || std::abs(q[i]-previousActual[i])/dt<(graspAlign ? 0.05 : 0.10),
                                      "unexpected joint speed");
        }
        previousActual=q; previousTime=now;
        const auto actualPosition=actual.translation();
        require(std::hypot(actualPosition.x(),actualPosition.y())<=cfg::kWorkspaceRadiusMaxM &&
                actualPosition.z()>=cfg::kWorkspaceZMinM && actualPosition.z()<=cfg::kWorkspaceZMaxM,
                "actual flange outside workspace");
        const Eigen::Vector3d delta=actual.translation()-G.translation();
        double progress=delta.dot(direction);
        if(jointMotion) {
          double t=0;
          for(int i=0;i<6;++i) t+=(q[i]-start[i])*jointDelta[i]/jointDistanceSquared;
          require(t>=-0.002 && t<=1.002,"actual joints outside planned interval");
          progress=t*length;
        }
        const double fraction=std::clamp(progress/length,0.0,1.0);
        if(!jointMotion) {
          const Pose expectedPose=plannedPose(fraction);
          require(progress>=-0.003 && progress<=length+0.003 &&
                (delta-direction*progress).norm()<0.005 &&
                Eigen::AngleAxisd(actual.linear().transpose()*expectedPose.linear()).angle()<0.01,
                "actual flange departed from the planned straight path");
        }
        if(pregrasp) require(((actual*toolOffset)-targetBase).dot(normalBase)>=standOff-(graspAlign ? 0.002 : 0.01),
                              "actual tool violates surface standoff");
        require(progress>=maximumProgress-0.005,"unexpected reverse motion");
        maximumProgress=std::max(maximumProgress,progress);
        const double index=std::clamp(progress/length,0.0,1.0)*samples;
        const int lower=std::min(static_cast<int>(index),samples-1);
        const double blend=index-lower;
        for(int i=0;i<6;++i) {
          const double predicted=path[lower][i]*(1-blend)+path[lower+1][i]*blend;
          require(std::abs(q[i]-predicted)<0.04,"actual joints differ from planned path at J"+
                  std::to_string(i+1));
        }
        const auto power=robot.powerState(ec); check(ec);
        require(power==rokae::PowerState::on,"power/safety state changed during motion");
        const auto stateNow=robot.operationState(ec); check(ec);
        require(stateNow==rokae::OperationState::idle ||
                stateNow==rokae::OperationState::moving,"unexpected operation state");
        const auto infoNow=robot.queryEventInfo(rokae::Event::moveExecution,ec); check(ec);
        using namespace rokae::EventInfoKey::MoveExecution;
        if(infoNow.count(ID) && std::any_cast<std::string>(infoNow.at(ID))==pathId) {
          if(infoNow.count(Error)) check(std::any_cast<std::error_code>(infoNow.at(Error)));
          if(infoNow.count(ReachTarget) && std::any_cast<bool>(infoNow.at(ReachTarget))) {
            require((actual.translation()-target.translation()).norm()<0.003 &&
                    Eigen::AngleAxisd(actual.linear().transpose()*target.linear()).angle()<0.01,
                    "controller reports arrival but target pose differs");
            if(returning) for(int i=0;i<6;++i) require(std::abs(q[i]-returnJoints[i])<0.005,
                                                      "return joint endpoint mismatch");
            if(jointMotion) for(int i=0;i<6;++i) require(std::abs(q[i]-jointGoal[i])<0.0005,
                                                       "joint arrival differs from planned endpoint");
            if(pregrasp) {
              const Eigen::Vector3d grasp=actual*toolOffset;
              std::cout<<"[RESULT] estimated grasp_center_B=["<<grasp.transpose()
                       <<"] m, normal gap="<<(grasp-targetBase).dot(normalBase)
                       <<" m, tool_axis_alignment="<<actual.linear().col(2).dot(-normalBase)
                       <<(graspAlign ? " (measured tool; alignment only, no gripper command)" :
                                       " (provisional tool, no contact)")<<std::endl;
            }
            std::cout<<"[RESULT] flange_B=["<<actual.translation().transpose()
                     <<"] m, position_error="<<(actual.translation()-target.translation()).norm()
                     <<" m"<<std::endl;
            arrived=true;
            break;
          }
        }
        if(now>=nextOtherCheck) {
          const auto otherQ=other.jointPos(ec); check(ec);
          for(int i=0;i<6;++i) require(std::isfinite(otherQ[i]) &&
              std::abs(otherQ[i]-otherStart[i])<0.001,"other arm moved during approach");
          const auto otherStateNow=other.operationState(ec); check(ec);
          require(otherStateNow==rokae::OperationState::idle,"other arm is no longer idle");
          nextOtherCheck=now+std::chrono::seconds(1);
        }
        if(now>=nextReport) {
          if(jointMotion) std::cout<<"[MOVING] joint path progress="<<fraction*100<<"%"<<std::endl;
          else std::cout<<"[MOVING] "<<progress*1000<<" / "<<length*1000<<" mm"<<std::endl;
          nextReport=now+std::chrono::seconds(5);
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
      }
      require(arrived,"motion timed out before target arrival");
    } catch(...) {
      shutdown();
      throw;
    }
    require(shutdown(),"motion finished but cleanup reported an error; verify controller state");
    const auto finalPower=robot.powerState(ec); check(ec);
    const auto finalState=robot.operationState(ec); check(ec);
    require(finalPower==rokae::PowerState::off && finalState==rokae::OperationState::idle,
            "final idle/power-off not confirmed");
    std::cout<<"[OK] "<<(returning ? "return to original start" :
                         (graspAlign ? "grasp-center alignment" :
                          (pregrasp ? "oriented non-contact pregrasp" : "complete non-contact approach")))
             <<" finished; idle, power off, manual\n";
    return 0;
  } catch(const std::exception& e) {
    std::cerr<<"[ERROR] "<<e.what()
             <<(motionIssued ? "; motion was commanded, see stop/cleanup result\n"
                              : "; no motion commanded\n");
    return 2;
  }
}
