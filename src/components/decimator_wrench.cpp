// Copyright 2014 WUT
/*
 * controller_common.cpp
 *
 *  Created on: 25 sty 2014
 *      Author: konrad
 */
#include <rtt/Component.hpp>

#include "controller_common/decimator.h"

#include "geometry_msgs/Wrench.h"

typedef Decimator<geometry_msgs::Wrench> DecimatorWrench;

ORO_LIST_COMPONENT_TYPE(DecimatorWrench)

