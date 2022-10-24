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

class VelmaService : public controller_common::RobotService<15,2> {
 public:
  explicit VelmaService(RTT::TaskContext* owner) : controller_common::RobotService<15,2>(owner, "robot") {
    std::cout << "Loaded VelmaService to task context " << owner->getName() << std::endl;
  }

  virtual ~VelmaService() {
  }

  virtual void jacobian(Jacobian &x, const Joints &q, const Tool tool[2]) {
    x.setZero();
#include "jacobian.inc"
  }

  virtual void fkin(Eigen::Affine3d *x, const Joints &q, const Tool tool[2]) {
#include "fkin.inc"
  }

  virtual int getNumberOfDofs() {
    return 15;
  }

  virtual int getNumberOfEffectors() {
    return 2;
  }
};

ORO_SERVICE_NAMED_PLUGIN(VelmaService, "robot");

