// Copyright 2014 WUT
/*
 * robot_mass_matrix.cpp
 *
 *  Created on: 12 mar 2014
 *      Author: Konrad Banachowicz
 */



#include "velma_fk.h"

#include <string>

#include "rtt/Component.hpp"

VelmaFK::VelmaFK(const std::string& name) : RTT::TaskContext(name, PreOperational) {
  this->ports()->addPort("JointPositionCommand", port_joint_position_command_);
  this->ports()->addPort("LeftToolCommand", port_left_tool_position_command_);
  this->ports()->addPort("RightToolCommand", port_right_tool_position_command_);
  this->ports()->addPort("LeftPositionCommand", port_left_position_command_);
  this->ports()->addPort("RightPositionCommand", port_right_position_command_);
}

VelmaFK::~VelmaFK() {
}

bool VelmaFK::configureHook() {
  robot_ = this->getProvider<Robot>("robot");
  if (!robot_) {
    RTT::log(RTT::Error) << "Unable to load RobotService"
                         << RTT::endlog();
    return false;
  }

  tools_.resize(2);
  joint_position_command_.resize(16);
  return true;
}

void VelmaFK::updateHook() {
  geometry_msgs::Pose tool_left, tool_right, cmd_left, cmd_right;
  Eigen::Affine3d r[2];
  
  port_joint_position_command_.read(joint_position_command_);
  
  if (port_left_tool_position_command_.read(tool_left) == RTT::NewData) {
    tools_[0](0) = tool_left.position.x;
    tools_[0](1) = tool_left.position.y;
    tools_[0](2) = tool_left.position.z;

    tools_[0](3) = tool_left.orientation.w;
    tools_[0](4) = tool_left.orientation.x;
    tools_[0](5) = tool_left.orientation.y;
    tools_[0](6) = tool_left.orientation.z;
  }
  
  if (port_right_tool_position_command_.read(tool_right) == RTT::NewData) {
    tools_[1](0) = tool_right.position.x;
    tools_[1](1) = tool_right.position.y;
    tools_[1](2) = tool_right.position.z;

    tools_[1](3) = tool_right.orientation.w;
    tools_[1](4) = tool_right.orientation.x;
    tools_[1](5) = tool_right.orientation.y;
    tools_[1](6) = tool_right.orientation.z;
  }
  
  robot_->fkin(r, joint_position_command_, &tools_[0]);
  
  tf::poseEigenToMsg(r[0], cmd_left);
  tf::poseEigenToMsg(r[1], cmd_right);
  
  port_left_position_command_.write(cmd_left);
  port_right_position_command_.write(cmd_right);
}

ORO_CREATE_COMPONENT(VelmaFK)

