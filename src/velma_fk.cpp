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

using namespace RTT;

VelmaFK::VelmaFK(const std::string& name) :
    RTT::TaskContext(name, PreOperational),
    port_joint_position_in_("JointPosition_INPORT"),
    port_left_tool_position_in_("LeftTool_INPORT"),
    port_right_tool_position_in_("RightTool_INPORT"),
    port_left_position_out_("LeftPosition_OUTPORT", true),
    port_right_position_out_("RightPosition_OUTPORT", true),
    port_left_wrist_out_("LeftWrist_OUTPORT", true),
    port_right_wrist_out_("RightWrist_OUTPORT", true) {

  this->ports()->addPort(port_joint_position_in_);
  this->ports()->addPort(port_left_tool_position_in_);
  this->ports()->addPort(port_right_tool_position_in_);
  this->ports()->addPort(port_left_position_out_);
  this->ports()->addPort(port_right_position_out_);
  this->ports()->addPort(port_left_wrist_out_);
  this->ports()->addPort(port_right_wrist_out_);

  geometry_msgs::Pose left_position_out, right_position_out, left_wrist_out, right_wrist_out;

  port_left_position_out_.setDataSample(left_position_out);
  port_right_position_out_.setDataSample(right_position_out);
  port_left_wrist_out_.setDataSample(left_wrist_out);
  port_right_wrist_out_.setDataSample(right_wrist_out);
}

VelmaFK::~VelmaFK() {
}

bool VelmaFK::configureHook() {
  RTT::Logger::In in("VelmaFK::configureHook");

  robot_ = this->getProvider<Robot>("robot");
  if (!robot_) {
    Logger::log() << Logger::Error << "Unable to load RobotService" << Logger::endl;
    return false;
  }

  empty_tools_[0].setZero();
  empty_tools_[0](3) = 1.0;     // w component of orientation quaternion
  empty_tools_[1].setZero();
  empty_tools_[1](3) = 1.0;     // w component of orientation quaternion

  tools_[0].setZero();
  tools_[0](3) = 1.0;           // w component of orientation quaternion
  tools_[1].setZero();
  tools_[1](3) = 1.0;           // w component of orientation quaternion

  return true;
}

void VelmaFK::updateHook() {
  geometry_msgs::Pose left_tool_position_in, right_tool_position_in, left_position_out, right_position_out, left_wrist_out, right_wrist_out;
  Eigen::Affine3d r_tool[2];
  Eigen::Affine3d r_wrist[2];

  if (port_joint_position_in_.read(joint_position_in_) != RTT::NewData) {
    RTT::Logger::In in("VelmaFK::updateHook");
    Logger::log() << Logger::Error << "Unable to read port "
                         << port_joint_position_in_.getName() << Logger::endl;
    return;
  }

  bool left_tool_available = false;
  if (port_left_tool_position_in_.read(left_tool_position_in) == RTT::NewData) {
    tools_[1](0) = left_tool_position_in.position.x;
    tools_[1](1) = left_tool_position_in.position.y;
    tools_[1](2) = left_tool_position_in.position.z;

    tools_[1](3) = left_tool_position_in.orientation.w;
    tools_[1](4) = left_tool_position_in.orientation.x;
    tools_[1](5) = left_tool_position_in.orientation.y;
    tools_[1](6) = left_tool_position_in.orientation.z;
    left_tool_available = true;
  }

  bool right_tool_available = false;
  if (port_right_tool_position_in_.read(right_tool_position_in) == RTT::NewData) {
    tools_[0](0) = right_tool_position_in.position.x;
    tools_[0](1) = right_tool_position_in.position.y;
    tools_[0](2) = right_tool_position_in.position.z;

    tools_[0](3) = right_tool_position_in.orientation.w;
    tools_[0](4) = right_tool_position_in.orientation.x;
    tools_[0](5) = right_tool_position_in.orientation.y;
    tools_[0](6) = right_tool_position_in.orientation.z;
    right_tool_available = true;
  }

  robot_->fkin(r_tool, joint_position_in_, &tools_[0]);

  if (left_tool_available) {
    tf::poseEigenToMsg(r_tool[1], left_position_out);
    port_left_position_out_.write(left_position_out);
  }

  if (right_tool_available) {
    tf::poseEigenToMsg(r_tool[0], right_position_out);
    port_right_position_out_.write(right_position_out);
  }

  // the poses of the wrists
  robot_->fkin(r_wrist, joint_position_in_, &empty_tools_[0]);

  tf::poseEigenToMsg(r_wrist[1], left_wrist_out);
  tf::poseEigenToMsg(r_wrist[0], right_wrist_out);

  port_left_wrist_out_.write(left_wrist_out);
  port_right_wrist_out_.write(right_wrist_out);
}

ORO_CREATE_COMPONENT(VelmaFK)

