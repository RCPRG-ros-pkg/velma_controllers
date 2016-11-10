// Copyright 2014 WUT
/*
 * controller_common.cpp
 *
 *  Created on: 25 sty 2014
 *      Author: konrad
 */
#include <rtt/Component.hpp>

#include "controller_common/JointStatePublisher.h"

typedef JointStatePublisher<33> JointStatePublisher33;

ORO_LIST_COMPONENT_TYPE(JointStatePublisher33)

