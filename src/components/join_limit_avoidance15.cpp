// Copyright 2014 WUT
/*
 * controller_common.cpp
 *
 *  Created on: 25 sty 2014
 *      Author: konrad
 */
#include <rtt/Component.hpp>

#include "controller_common/joint_limit_avoidance.h"

typedef JointLimitAvoidance<15> JointLimitAvoidance15;

ORO_LIST_COMPONENT_TYPE(JointLimitAvoidance15)

