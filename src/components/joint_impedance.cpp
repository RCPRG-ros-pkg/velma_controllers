// Copyright 2014 WUT
/*
 * controller_common.cpp
 *
 *  Created on: 25 sty 2014
 *      Author: konrad
 */
#include <rtt/Component.hpp>

#include "controller_common/joint_impedance.h"

typedef JointImpedance<15> JointImpedance15;

ORO_LIST_COMPONENT_TYPE(JointImpedance15)

