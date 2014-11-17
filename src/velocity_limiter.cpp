// Copyright 2014 WUT
/*
 * velma_sim.cpp
 *
 *  Created on: Feb 26, 2014
 *      Author: Konrad Banachowicz
 */

#include <string>

#include "rtt/TaskContext.hpp"
#include "rtt/Port.hpp"
#include "rtt/Component.hpp"

class VelocityLimiter: public RTT::TaskContext {
 public:
  explicit VelocityLimiter(const std::string& name):
    RTT::TaskContext(name),position_out_(0), max_vel_(0) {
    this->ports()->addPort("PositionMsr", port_position_msr_);
    this->ports()->addPort("PositionIn", port_position_in_);
    this->ports()->addPort("PositionOut", port_position_out_);
    this->addProperty("max_vel", max_vel_);
  }

  bool configureHook() {
    return true;
  }

  bool startHook() {
    if (port_position_msr_.read(position_out_) == RTT::NoData) {
      return false;
    }
    return true;
  }

  void updateHook() {
    double position_cmd;
    
    if(port_position_in_.read(position_cmd) == RTT::NewData) {
      if (abs(position_out_ - position_cmd) < max_vel_) {
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

  RTT::InputPort<double> port_position_msr_;
  RTT::InputPort<double> port_position_in_;
  RTT::OutputPort<double> port_position_out_;

  double position_out_;
  double max_vel_;
};

ORO_CREATE_COMPONENT(VelocityLimiter)

