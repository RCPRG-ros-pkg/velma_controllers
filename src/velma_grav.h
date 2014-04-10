/*
 * robot_mass_matrix.h
 *
 *  Created on: 12 mar 2014
 *      Author: konradb3
 */

#ifndef VELMA_GRAV_H_
#define VELMA_GRAV_H_

#include <rtt/TaskContext.hpp>
#include <rtt/Port.hpp>
#include <Eigen/Dense>

class VelmaGrav: public RTT::TaskContext {
public:
	VelmaGrav(const std::string& name);
	virtual ~VelmaGrav();

	bool configureHook();
	void updateHook();
private:
	RTT::OutputPort<Eigen::VectorXd> port_grav_trq_;
	RTT::InputPort<Eigen::VectorXd > port_grav_trq_left_;
	RTT::InputPort<Eigen::VectorXd > port_grav_trq_right_;
	RTT::InputPort<Eigen::VectorXd > port_joint_velocity_;

	int number_of_joints_;
	int number_of_effectors_;

	Eigen::VectorXd grav_trq_, grav_trq_left_, grav_trq_right_, joint_velocity_;
};

#endif /* VELMA_GRAV_H_ */
