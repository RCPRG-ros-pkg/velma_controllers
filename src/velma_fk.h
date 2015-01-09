// Copyright 2014 WUT
/*
 * velma_fk.h
 *
 *  Created on: 12 mar 2014
 *      Author: Konrad Banachowicz
 */

#ifndef VELMA_FK_H_
#define VELMA_FK_H_

#include <string>
#include <vector>

#include "rtt/TaskContext.hpp"
#include "rtt/Port.hpp"
#include "Eigen/Dense"

#include "geometry_msgs/Pose.h"
#include "eigen_conversions/eigen_msg.h"

#include "controller_common/robot.h"

class VelmaFK: public RTT::TaskContext {
 public:
  explicit VelmaFK(const std::string& name);
  virtual ~VelmaFK();

  bool configureHook();
  void updateHook();
 private:
  typedef Eigen::Matrix<double, 7, 1> Tool;
  typedef controller_common::Robot Robot;

  RTT::OutputPort<geometry_msgs::Pose> port_left_position_command_;
  RTT::OutputPort<geometry_msgs::Pose> port_right_position_command_;

  RTT::InputPort<Eigen::VectorXd > port_joint_position_command_;
  RTT::InputPort<geometry_msgs::Pose> port_left_tool_position_command_;
  RTT::InputPort<geometry_msgs::Pose> port_right_tool_position_command_;


  Eigen::VectorXd joint_position_command_;
  boost::shared_ptr<Robot> robot_;
  std::vector<Tool> tools_;
};

#endif  // VELMA_FK_H_
