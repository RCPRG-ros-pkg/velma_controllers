/*
 * velma_controller.cpp
 *
 *  Created on: 17 lut 2014
 *      Author: konradb3
 */
#include <rtt/Component.hpp>

#include "controller_common/vector_concate.h"
#include "controller_common/vector_split.h"
#include "controller_common/cartesian_impedance.h"

typedef CartesianImpedance<16, 2> VelmaCartesianImpedance;

ORO_LIST_COMPONENT_TYPE(VelmaCartesianImpedance)

ORO_CREATE_COMPONENT_LIBRARY()
