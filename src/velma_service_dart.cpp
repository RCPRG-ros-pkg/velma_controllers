// Copyright 2014 WUT
/*
 * velma_service.cpp
 *
 *  Created on: 17 lut 2014
 *      Author: Konrad Banachowicz
 */

#include <rtt/plugin/ServicePlugin.hpp>

#include <controller_common/robot_service.h>

#include "params.inc"

class VelmaServiceDart : public controller_common::RobotService<15,2> {
 public:
  explicit VelmaServiceDart(RTT::TaskContext* owner) : controller_common::RobotService<15,2>(owner, "robot_dart") {
    std::cout << "Loaded VelmaServiceDart to task context " << owner->getName() << std::endl;
  }

  virtual ~VelmaServiceDart() {
  }

  virtual void jacobian(Jacobian &x, const Joints &q, const Tool tool[2]) {
    x.setZero();
    // TODO: calculate for proper arms mount
    throw std::runtime_error("not implemented");
//#include "jacobian.inc"
  }

  virtual void fkin(Eigen::Affine3d *x, const Joints &q, const Tool tool[2]) {
    // TODO: calculate for proper arms mount
    throw std::runtime_error("not implemented");
//#include "fkin.inc"
  }

  virtual int getNumberOfDofs() {
    return 15;
  }

  virtual int getNumberOfEffectors() {
    return 2;
  }
};

ORO_SERVICE_NAMED_PLUGIN(VelmaServiceDart, "robot_dart");

