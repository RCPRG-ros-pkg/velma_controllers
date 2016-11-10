// Copyright 2014 WUT
/*
 * controller_common.cpp
 *
 *  Created on: 25 sty 2014
 *      Author: konrad
 */
#include <rtt/Component.hpp>

#include "controller_common/torque_publisher.h"

typedef TorquePublisher<7> TorquePublisher7;

ORO_LIST_COMPONENT_TYPE(TorquePublisher7)

