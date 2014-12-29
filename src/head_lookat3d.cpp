// Copyright 2014 WUT
/*
 * head_lookat3d.cpp.cpp
 *
 *  Created on: 28 mar 2014
 *      Author: mwalecki
 */

#include <string>
#include <vector>

#include "rtt/TaskContext.hpp"
#include "rtt/Port.hpp"
#include "rtt/Component.hpp"

#include "head_kinematics.h"

#include "geometry_msgs/Pose.h"

#include "Eigen/Dense"

#define HEAD_CTRL_MNJ 2
#define FULL_TORSO_MNJ 4
#define DEFAULT_TARGET_X  1.00d
#define DEFAULT_TARGET_Y  0.00d
#define DEFAULT_TARGET_Z  1.54d

class HeadLookAt3D : public RTT::TaskContext {
 public:
  explicit HeadLookAt3D(const std::string & name) : TaskContext(name, PreOperational) {
    this->ports()->addPort("HeadJointPositionCommand", port_HeadJointPositionCommand).doc("");
    this->ports()->addPort("TargetPoint", port_TargetPoint).doc("");
    this->ports()->addPort("JointPosition", port_JointPosition).doc("");
  }

  ~HeadLookAt3D() {
  }

  bool configureHook() {
    head_jnt_pos_cmd_.resize(HEAD_CTRL_MNJ);
    port_HeadJointPositionCommand.setDataSample(head_jnt_pos_cmd_);
    jnt_pos_.resize(FULL_TORSO_MNJ);

    h = new HeadKinematics(H_ROT, H_LEAN, H_HEAD, H_CAM);

    current_point.position.x = DEFAULT_TARGET_X;
    current_point.position.y = DEFAULT_TARGET_Y;
    current_point.position.z = DEFAULT_TARGET_Z;

    return true;
  }

  bool startHook() {
    return true;
  }

  void stopHook() {
  }

  void updateHook() {
    double joint_pan, joint_tilt;
    RTT::FlowStatus target_point_fs, jnt_pos_fs;

    target_point_fs = port_TargetPoint.read(target_point);
    if (target_point_fs == RTT::NewData) {
      current_point.position = target_point.position;
    }

    jnt_pos_fs = port_JointPosition.read(jnt_pos_);
    if (jnt_pos_fs == RTT::NewData) {
      h->UpdateTorsoPose(jnt_pos_(0), jnt_pos_(1));
      h->UpdateTargetPosition(current_point.position.x, current_point.position.y, current_point.position.z);
      h->TransformTargetToHeadFrame();
      if (h->CalculateHeadPose(&joint_pan, &joint_tilt) == 0) {
        head_jnt_pos_cmd_(0) = joint_pan;
        head_jnt_pos_cmd_(1) = joint_tilt;
      }
      port_HeadJointPositionCommand.write(head_jnt_pos_cmd_);
    }
  }

 private:
  HeadKinematics *h;

  RTT::OutputPort<Eigen::VectorXd > port_HeadJointPositionCommand;
  RTT::InputPort<geometry_msgs::Pose > port_TargetPoint;
  RTT::InputPort<Eigen::VectorXd > port_JointPosition;

  geometry_msgs::Pose target_point;
  geometry_msgs::Pose current_point;
  Eigen::VectorXd head_jnt_pos_cmd_;
  Eigen::VectorXd jnt_pos_;
};

ORO_CREATE_COMPONENT(HeadLookAt3D)

