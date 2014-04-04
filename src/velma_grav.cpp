/*
 * robot_mass_matrix.cpp
 *
 *  Created on: 12 mar 2014
 *      Author: konradb3
 */

#include <rtt/Component.hpp>

#include "velma_grav.h"

VelmaGrav::VelmaGrav(const std::string& name) : RTT::TaskContext(name, PreOperational) {

	this->ports()->addPort("GravTrq", port_grav_trq_);
	this->ports()->addPort("GravTrqLeft", port_grav_trq_left_);
	this->ports()->addPort("GravTrqRight", port_grav_trq_right_);

}

VelmaGrav::~VelmaGrav() {

}

bool VelmaGrav::configureHook() {
	grav_trq_.resize(16);
	grav_trq_left_.resize(7);
	grav_trq_right_.resize(7);

	grav_trq_(0) = 0.0;
	grav_trq_(1) = 0.0;

	return true;
}

void VelmaGrav::updateHook() {

	port_grav_trq_left_.read(grav_trq_left_);
	port_grav_trq_right_.read(grav_trq_right_);
	
	grav_trq_.segment<7>(2) = -0.5 * grav_trq_right_;
	grav_trq_.segment<7>(9) = -0.5 * grav_trq_left_;
	
	port_grav_trq_.write(grav_trq_);
}

ORO_CREATE_COMPONENT(VelmaGrav)

