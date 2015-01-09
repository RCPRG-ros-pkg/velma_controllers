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

#include "rtt_rosclock/rtt_rosclock.h"

#include "std_msgs/Header.h"

class TriggerGenerator: public RTT::TaskContext {
 public:
  explicit TriggerGenerator(const std::string& name):
    RTT::TaskContext(name, PreOperational), trigger_(false) {
    this->ports()->addPort("Trigger", port_trigger_);
    this->ports()->addPort("TriggerStamp", port_trigger_stamp_);
  }

  bool configureHook() {
    return true;
  }

  bool startHook() {
    return true;
  }

  void updateHook() {
    if (trigger_ == false) {
      std_msgs::Header hdr;
      hdr.stamp = rtt_rosclock::host_now();
      port_trigger_stamp_.write(hdr);
    }
    port_trigger_.write(trigger_);
    trigger_ = !trigger_;
  }

 private:
  RTT::OutputPort<bool> port_trigger_;
  RTT::OutputPort<std_msgs::Header> port_trigger_stamp_;

  bool trigger_;
};

ORO_CREATE_COMPONENT(TriggerGenerator)

