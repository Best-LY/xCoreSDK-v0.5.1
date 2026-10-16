#include <chrono>
#include <iostream>
#include <stdexcept>
#include <string>
#include <opencv2/core.hpp>
#include "rokae/robot.h"
#include "robot_vision_config.hpp"

namespace {
void check(const std::error_code& ec) {
  if(ec) throw std::runtime_error(ec.message()+" (code="+std::to_string(ec.value())+")");
}
void load(cv::FileStorage& out,const rokae::Load& value) {
  out<<"mass_kg"<<value.mass;
  out<<"cog_m"<<std::vector<double>(value.cog.begin(),value.cog.end());
  out<<"inertia_kg_m2"<<std::vector<double>(value.inertia.begin(),value.inertia.end());
}
void frame(cv::FileStorage& out,const rokae::Frame& value) {
  out<<"translation_m"<<std::vector<double>(value.trans.begin(),value.trans.end());
  out<<"rpy_rad"<<std::vector<double>(value.rpy.begin(),value.rpy.end());
}
}

int main(int argc,char** argv) {
  try {
    if(argc!=2 || (std::string(argv[1])!="--left" && std::string(argv[1])!="--right")) {
      std::cerr<<"Usage: tool_load_readback --left|--right (query only, JSON on stdout)\n";
      return 2;
    }
    const bool left=std::string(argv[1])=="--left";
    rokae::xMateRobot robot(left ? user_config::IP_LEFT : user_config::IP_RIGHT,user_config::LOCAL_IP);
    std::error_code ec;
    const auto state=robot.operationState(ec); check(ec);
    if(state!=rokae::OperationState::idle) throw std::runtime_error("robot not idle; no further queries");
    const auto tool=robot.toolset(ec); check(ec);
    const auto tools=robot.toolsInfo(ec); check(ec);
    const auto stamp=std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::system_clock::now().time_since_epoch()).count();
    cv::FileStorage out("readback.json",cv::FileStorage::WRITE|cv::FileStorage::MEMORY|cv::FileStorage::FORMAT_JSON);
    out<<"schema_version"<<1<<"arm"<<(left ? "left" : "right");
    out<<"captured_unix_ms"<<std::to_string(stamp)<<"motion_commands_sent"<<0;
    out<<"scope"<<"SDK active toolset and tools in currently loaded RL project only";
    out<<"active_sdk_toolset"<<"{";
    out<<"load"<<"{"; load(out,tool.load); out<<"}";
    out<<"end"<<"{"; frame(out,tool.end); out<<"}";
    out<<"ref"<<"{"; frame(out,tool.ref); out<<"}";
    out<<"}"<<"loaded_project_tools"<<"[";
    for(const auto& item:tools) {
      out<<"{"<<"name"<<item.name<<"robot_held"<<int(item.robotHeld)<<"load"<<"{";
      load(out,item.load); out<<"}"<<"frame"<<"{"; frame(out,item.pos); out<<"}"<<"}";
    }
    out<<"]"<<"collision_detection_enabled_readback"<<"not_available_in_this_query";
    std::cout<<out.releaseAndGetString()<<std::endl;
    return 0;
  } catch(const std::exception& error) {
    std::cerr<<"[ERROR] "<<error.what()<<"; no explicit motion/mode/power commands sent\n";
    return 2;
  }
}
