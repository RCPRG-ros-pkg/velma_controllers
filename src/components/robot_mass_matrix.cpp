// Copyright 2014 WUT
/*
 * controller_common.cpp
 *
 *  Created on: 25 sty 2014
 *      Author: konrad
 */
#include <rtt/Component.hpp>

#include "controller_common/robot_mass_matrix.h"

typedef RobotMassMatrix<15, 2> RobotMassMatrix15_2;

ORO_LIST_COMPONENT_TYPE(RobotMassMatrix15_2)

