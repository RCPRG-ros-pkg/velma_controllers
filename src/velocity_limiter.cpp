// Copyright 2014 WUT
/*
 * velocity_limiter.cpp
 *
 *  Created on: Feb 26, 2014
 *      Author: Konrad Banachowicz
 */

#include <string>

#include "Eigen/Dense"

#include "rtt/TaskContext.hpp"
#include "rtt/Port.hpp"
#include "rtt/Component.hpp"

class VelocityLimiter: public RTT::TaskContext {
  typedef Eigen::Matrix<double, 1, 1 > Joints;

 public:
  explicit VelocityLimiter(const std::string& name):
    RTT::TaskContext(name),
    position_out_(0),
    max_vel_(0),
    port_position_msr_in_("PositionMsr_INPORT"),
    port_position_in_("Position_INPORT"),
    port_position_out_("Position_OUTPORT", false) {

    this->ports()->addPort(port_position_msr_in_);
    this->ports()->addPort(port_position_in_);
    this->ports()->addPort(port_position_out_);
    this->addProperty("max_vel", max_vel_);
  }

  bool configureHook() {
    return true;
  }

  bool startHook() {
    first_step_ = true;
    return true;
  }

  void updateHook() {
    double position_cmd;

    Joints position;
    if (first_step_) {
      if (port_position_msr_in_.read(position) != RTT::NewData) {
        RTT::Logger::In in("VelocityLimiter::updateHook");
        RTT::Logger::log() << RTT::Logger::Error << "could not read data on port " << port_position_msr_in_.getName() << RTT::Logger::endl;
        error();
        return;
      }
      position_out_ = position(0);
      first_step_ = false;
    }

    if (port_position_in_.read(position_cmd) == RTT::NewData) {
      if (fabs(position_out_ - position_cmd) < max_vel_) {
        position_out_ = position_cmd;
      } else if ((position_out_ - position_cmd) < 0.0) {
        position_out_ += max_vel_;
      } else {
        position_out_ -= max_vel_;
      }
      port_position_out_.write(position_out_);
    }
  }

 private:

  RTT::InputPort<Joints> port_position_msr_in_;
  RTT::InputPort<double> port_position_in_;
  RTT::OutputPort<double> port_position_out_;

  double position_out_;
  double max_vel_;

  bool first_step_;
};

ORO_CREATE_COMPONENT(VelocityLimiter)

