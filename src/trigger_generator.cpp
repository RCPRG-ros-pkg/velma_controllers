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
    RTT::TaskContext(name, PreOperational),
    trigger_(false),
    port_trigger_out_("Trigger_OUTPORT", true),
    port_trigger_stamp_out_("TriggerStamp_OUTPORT", true),
    skip_cycles_(0),
    cycle_counter_(0) {

    this->ports()->addPort(port_trigger_out_);
    this->ports()->addPort(port_trigger_stamp_out_);

    port_trigger_out_.setDataSample(trigger_);
    port_trigger_stamp_out_.setDataSample(hdr_);

    this->properties()->addProperty("skip_cycles", skip_cycles_);
  }

  bool configureHook() {
    if (skip_cycles_ == 0) {
      return false;
    }
    return true;
  }

  bool startHook() {
    cycle_counter_ = 0;
    return true;
  }

  void updateHook() {
    if (cycle_counter_ < skip_cycles_) {
      cycle_counter_++;
      return;
    }
    cycle_counter_ = 0;
    if (trigger_ == false) {
      hdr_.stamp = rtt_rosclock::host_now();
      port_trigger_stamp_out_.write(hdr_);
    }
    port_trigger_out_.write(uint32_t(trigger_));
    trigger_ = !trigger_;
  }

 private:
  RTT::OutputPort<uint32_t> port_trigger_out_;
  RTT::OutputPort<std_msgs::Header> port_trigger_stamp_out_;

  bool trigger_;
  std_msgs::Header hdr_;

  uint32_t skip_cycles_;
  uint32_t cycle_counter_;
};

ORO_CREATE_COMPONENT(TriggerGenerator)

