// Copyright 2014 WUT
/*
 * torso_teleop_joy.cpp
 *
 *  Created on: 28 mar 2014
 *      Author: Michał Walencki
 */

#include <vector>
#include <string>

#include "rtt/TaskContext.hpp"
#include "rtt/Port.hpp"
#include "rtt/Component.hpp"

#include "Eigen/Dense"

#include "geometry_msgs/Pose.h"
#include "sensor_msgs/Joy.h"

#define FULL_TORSO_MNJ 4
#define HEAD_CTRL_MNJ 2
#define TORSO_CTRL_MNJ 1
#define TEST_LOOP_T 5000

#define GEAR0 158.0
#define GEAR1 1.0
#define GEAR2 113.0
#define GEAR3 50.0

#define ENC0 5000.0
#define ENC1 5000.0
#define ENC2 500.0
#define ENC3 500.0

#define MAX_VEL_X  (50000/(ENC2 * 4.0 * GEAR2) * M_PI * 2)
#define MAX_VEL_Y  (20000/(ENC3 * 4.0 * GEAR3) * M_PI * 2)
#define MAX_VEL_T  (200/(ENC1 * 4.0 * GEAR1) * M_PI * 2)
#define MAX_TRQ_T  35

#define HTP_MIN_X  0.0
#define HTP_MAX_X  3.0
#define HTP_MIN_Y -2.0
#define HTP_MAX_Y  2.0
#define HTP_MIN_Z  0.0
#define HTP_MAX_Z  2.5
#define HTP_DEF_X  1.00d
#define HTP_DEF_Y  0.00d
#define HTP_DEF_Z  1.54d

#define JOY_CMD_TIMEOUT 5000  // [UpdateHookCycles]

class TorsoTeleopJoy : public RTT::TaskContext {
 public:
  explicit TorsoTeleopJoy(const std::string & name) : TaskContext(name, PreOperational) {
    this->ports()->addPort("HeadJointPositionCommand", port_HeadJointPositionCommand).doc("");
    this->ports()->addPort("TorsoJointPositionCommand", port_TorsoJointPositionCommand).doc("");
    this->ports()->addPort("TorsoJointTorqueCommand", port_TorsoJointTorqueCommand).doc("");
    this->ports()->addPort("NullSpaceTorqueCommand", port_NullSpaceTorqueCommand).doc("");
    this->ports()->addPort("HeadTargetPoint", port_HeadTargetPointCommand).doc("");
    this->ports()->addPort("PrimaryFrameSelector", port_primary_frame_selector_).doc("");

    this->ports()->addPort("JointVelocity", port_JointVelocity).doc("");
    this->ports()->addPort("JointPosition", port_JointPosition).doc("");

    this->ports()->addPort("Joy", port_Joy).doc("");
  }

  ~TorsoTeleopJoy() {
  }

  bool configureHook() {
    jnt_pos_.resize(FULL_TORSO_MNJ);
    jnt_vel_.resize(FULL_TORSO_MNJ);
    head_jnt_pos_cmd_.resize(HEAD_CTRL_MNJ);
    torso_jnt_pos_cmd_.resize(TORSO_CTRL_MNJ);
    torso_jnt_trq_cmd_.resize(TORSO_CTRL_MNJ);
    nullspace_torque_command_.resize(16);
    for (int i = 0; i < 16; i++)
      nullspace_torque_command_(i) = 0;

    port_HeadJointPositionCommand.setDataSample(head_jnt_pos_cmd_);
    port_TorsoJointPositionCommand.setDataSample(torso_jnt_pos_cmd_);
    port_TorsoJointTorqueCommand.setDataSample(torso_jnt_trq_cmd_);
    port_NullSpaceTorqueCommand.setDataSample(nullspace_torque_command_);
    port_HeadTargetPointCommand.setDataSample(head_target_point_cmd_);

    head_target_point_cmd_.position.x = HTP_DEF_X;
    head_target_point_cmd_.position.y = HTP_DEF_Y;
    head_target_point_cmd_.position.z = HTP_DEF_Z;
    head_target_point_cmd_.orientation.x = 0;
    head_target_point_cmd_.orientation.y = 0;
    head_target_point_cmd_.orientation.z = 0;
    head_target_point_cmd_.orientation.w = 1;
    primary_frame_selector_ = 0;

    joyCmdWD = 0;  // Component sends position commands after start.

    return true;
  }

  bool startHook() {
    Eigen::VectorXd jnt_pos;
    // Really hope some VT component is up
    //while (port_JointPosition.read(jnt_pos) != RTT::NewData)
    //  usleep(100);
    port_JointPosition.read(jnt_pos);

    //torso_jnt_pos_cmd_(0) = jnt_pos(1);

    //head_jnt_pos_cmd_(0) = jnt_pos(2);
    //head_jnt_pos_cmd_(1) = jnt_pos(3);

    setVelX = 0.0;
    setVelY = 0.0;
    setVelT = 0.0;

    setTrqT = 0.0;

    targetVelX = 0.0;
    targetVelY = 0.0;
    targetVelZ = 0.0;

    return true;
  }

  void stopHook() {
  }

  void updateHook() {
    RTT::FlowStatus joy_fs;

    joy_fs = port_Joy.read(joy_);
    if (joy_fs == RTT::NewData) {
      setVelX =  joy_.axes[0] * MAX_VEL_X * ((joy_.buttons[6] || joy_.buttons[4]) ? 1.0 : 0.1);
      setVelY = - joy_.axes[1] * MAX_VEL_Y * ((joy_.buttons[6] || joy_.buttons[4]) ? 1.0 : 0.1);
      setVelT =  joy_.axes[4] * MAX_VEL_T * ((joy_.buttons[6] || joy_.buttons[4]) ? 1.0 : 0.6);
      setTrqT =  joy_.axes[3] * MAX_TRQ_T * ((joy_.buttons[6] || joy_.buttons[4]) ? 1.0 : 0.6);
      targetVelX = 0;
      targetVelY =  joy_.axes[0] * 0.001 * ((joy_.buttons[6] || joy_.buttons[4]) ? 1.0 : 0.1);
      targetVelZ =  joy_.axes[1] * 0.001 * ((joy_.buttons[6] || joy_.buttons[4]) ? 1.0 : 0.1);

      //  [1] [2] [3] [4] Buttons - Primary frame selection
      if (joy_.buttons[0]) {
        primary_frame_selector_ = 0;
        head_target_point_cmd_.position.x = HTP_DEF_X;
        head_target_point_cmd_.position.y = HTP_DEF_Y;
        head_target_point_cmd_.position.z = HTP_DEF_Z;
      } else if (joy_.buttons[1]) {
        primary_frame_selector_ = 1;
        head_target_point_cmd_.position.x = 0;
        head_target_point_cmd_.position.y = 0;
        head_target_point_cmd_.position.z = 0;
      } else if (joy_.buttons[2]) {
        primary_frame_selector_ = 2;
        head_target_point_cmd_.position.x = 0;
        head_target_point_cmd_.position.y = 0;
        head_target_point_cmd_.position.z = 0;
      } else if (joy_.buttons[3]) {
        primary_frame_selector_ = 3;
        head_target_point_cmd_.position.x = 0;
        head_target_point_cmd_.position.y = 0;
        head_target_point_cmd_.position.z = 0;
      }


      joyCmdWD = 0;
    }
    if (joyCmdWD < JOY_CMD_TIMEOUT) {
      head_jnt_pos_cmd_(0) += setVelX / 1000.0;
      head_jnt_pos_cmd_(1) += setVelY / 1000.0;
      port_HeadJointPositionCommand.write(head_jnt_pos_cmd_);
      torso_jnt_pos_cmd_(0) += setVelT / 1000.0;
      port_TorsoJointPositionCommand.write(torso_jnt_pos_cmd_);
      torso_jnt_trq_cmd_(0) = setTrqT;
      port_TorsoJointTorqueCommand.write(torso_jnt_trq_cmd_);
      nullspace_torque_command_(0) = setTrqT;
      port_NullSpaceTorqueCommand.write(nullspace_torque_command_);

      head_target_point_cmd_.position.x += targetVelX;
      if (head_target_point_cmd_.position.x < HTP_MIN_X)
        head_target_point_cmd_.position.x = HTP_MIN_X;
      else if (head_target_point_cmd_.position.x > HTP_MAX_X)
        head_target_point_cmd_.position.x = HTP_MAX_X;
      head_target_point_cmd_.position.y += targetVelY;
      if (head_target_point_cmd_.position.y < HTP_MIN_Y)
        head_target_point_cmd_.position.y = HTP_MIN_Y;
      else if (head_target_point_cmd_.position.y > HTP_MAX_Y)
        head_target_point_cmd_.position.y = HTP_MAX_Y;
      head_target_point_cmd_.position.z += targetVelZ;
      if (head_target_point_cmd_.position.z < HTP_MIN_Z)
        head_target_point_cmd_.position.z = HTP_MIN_Z;
      else if (head_target_point_cmd_.position.z > HTP_MAX_Z)
        head_target_point_cmd_.position.z = HTP_MAX_Z;

      port_HeadTargetPointCommand.write(head_target_point_cmd_);

      port_primary_frame_selector_.write(primary_frame_selector_);

      joyCmdWD++;
    }
  }

 private:
  RTT::OutputPort<Eigen::VectorXd > port_HeadJointPositionCommand;
  RTT::OutputPort<Eigen::VectorXd > port_TorsoJointPositionCommand;
  RTT::OutputPort<Eigen::VectorXd > port_TorsoJointTorqueCommand;
  RTT::OutputPort<Eigen::VectorXd > port_NullSpaceTorqueCommand;
  RTT::OutputPort<geometry_msgs::Pose > port_HeadTargetPointCommand;
  RTT::OutputPort<int> port_primary_frame_selector_;

  RTT::InputPort<Eigen::VectorXd > port_JointVelocity;
  RTT::InputPort<Eigen::VectorXd > port_JointPosition;

  RTT::InputPort<sensor_msgs::Joy > port_Joy;

  Eigen::VectorXd jnt_pos_;
  Eigen::VectorXd jnt_vel_;

  sensor_msgs::Joy joy_;

  Eigen::VectorXd head_jnt_pos_cmd_;
  Eigen::VectorXd torso_jnt_pos_cmd_;
  Eigen::VectorXd torso_jnt_trq_cmd_;
  Eigen::VectorXd nullspace_torque_command_;
  geometry_msgs::Pose head_target_point_cmd_;
  int primary_frame_selector_;

  bool synchro_;
  int joyCmdWD;
  int testMoveDir;

  double setVelX, setVelY, setVelT;
  double targetVelX, targetVelY, targetVelZ;
  int setTrqT;
};

ORO_CREATE_COMPONENT(TorsoTeleopJoy)


