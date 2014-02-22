/*
 * velma_service.cpp
 *
 *  Created on: 17 lut 2014
 *      Author: konradb3
 */

#include <rtt/plugin/ServicePlugin.hpp>

#include <controller_common/robot_service.h>

#include "params.inc"

class VelmaService : public controller_common::RobotService<16, 2> {
public:
	VelmaService(RTT::TaskContext* owner) : controller_common::RobotService<16, 2>(owner) {

	}

	virtual ~VelmaService() {

	}

	virtual void jacobian(Jacobian &x, const Joints &q, const Tool tool[2]){
#include "jacobian.inc"
	}

	virtual void inertia(Inertia &x, const Joints &q, const ToolMass toolM[2]){
#include "inertia.inc"
	}

	virtual void fkin(Eigen::Affine3d *x, const Joints &q, const Tool tool[2]){
#include "fkin.inc"
	}
};

ORO_SERVICE_NAMED_PLUGIN(VelmaService, "robot");
