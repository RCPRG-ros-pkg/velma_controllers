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
  this->ports()->addPort("JointPosition_INPORT", port_joint_position_in_);
  this->ports()->addPort("LeftTool_INPORT", port_left_tool_position_in_);
  this->ports()->addPort("RightTool_INPORT", port_right_tool_position_in_);
  this->ports()->addPort("LeftPosition_OUTPORT", port_left_position_out_);
  this->ports()->addPort("RightPosition_OUTPORT", port_right_position_out_);
  this->ports()->addPort("LeftWrist_OUTPORT", port_left_wrist_out_);
  this->ports()->addPort("RightWrist_OUTPORT", port_right_wrist_out_);
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

  empty_tools_.resize(2);
  empty_tools_[0].setZero();
  empty_tools_[0](3) = 1.0;     // w component of orientation quaternion
  empty_tools_[1].setZero();
  empty_tools_[1](3) = 1.0;     // w component of orientation quaternion

  tools_.resize(2);
  joint_position_in_.resize(16);
  return true;
}

void VelmaFK::updateHook() {
  geometry_msgs::Pose left_tool_position_in, right_tool_position_in, left_position_out, right_position_out, left_wrist_out, right_wrist_out;
  Eigen::Affine3d r[2];

  port_joint_position_in_.read(joint_position_in_);

  if (port_left_tool_position_in_.read(left_tool_position_in) == RTT::NewData) {
    tools_[1](0) = left_tool_position_in.position.x;
    tools_[1](1) = left_tool_position_in.position.y;
    tools_[1](2) = left_tool_position_in.position.z;

    tools_[1](3) = left_tool_position_in.orientation.w;
    tools_[1](4) = left_tool_position_in.orientation.x;
    tools_[1](5) = left_tool_position_in.orientation.y;
    tools_[1](6) = left_tool_position_in.orientation.z;
  }

  if (port_right_tool_position_in_.read(right_tool_position_in) == RTT::NewData) {
    tools_[0](0) = right_tool_position_in.position.x;
    tools_[0](1) = right_tool_position_in.position.y;
    tools_[0](2) = right_tool_position_in.position.z;

    tools_[0](3) = right_tool_position_in.orientation.w;
    tools_[0](4) = right_tool_position_in.orientation.x;
    tools_[0](5) = right_tool_position_in.orientation.y;
    tools_[0](6) = right_tool_position_in.orientation.z;
  }

  robot_->fkin(r, joint_position_in_, &tools_[0]);

  tf::poseEigenToMsg(r[1], left_position_out);
  tf::poseEigenToMsg(r[0], right_position_out);

  port_left_position_out_.write(left_position_out);
  port_right_position_out_.write(right_position_out);

  // the poses of the wrists
  robot_->fkin(r, joint_position_in_, &empty_tools_[0]);

  tf::poseEigenToMsg(r[1], left_wrist_out);
  tf::poseEigenToMsg(r[0], right_wrist_out);

  port_left_wrist_out_.write(left_wrist_out);
  port_right_wrist_out_.write(right_wrist_out);
}

ORO_CREATE_COMPONENT(VelmaFK)

