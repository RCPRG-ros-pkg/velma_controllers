// Copyright 2014 WUT
/*
 * controller_common.cpp
 *
 *  Created on: 25 sty 2014
 *      Author: konrad
 */
#include <rtt/Component.hpp>

#include "controller_common/cartesian_impedance.h"

typedef CartesianImpedance<15, 2> CartesianImpedance15_2;

ORO_LIST_COMPONENT_TYPE(CartesianImpedance15_2)

