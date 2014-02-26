/*
 * velma_sim.cpp
 *
 *  Created on: Feb 26, 2014
 *      Author: konrad
 */

#include <rtt/TaskContext.hpp>
#include <rtt/Port.hpp>
#include <rtt/Component.hpp>

#include <Eigen/Dense>

#include <controller_common/robot.h>

#define DOFS 16

#define POS_DOFS 1

typedef controller_common::Robot<DOFS, 2> Robot;
typedef Eigen::Matrix<double, DOFS * 2, 1> State;
typedef Eigen::Matrix<double, DOFS, DOFS> Inertia;
typedef Eigen::Matrix<double, DOFS, 1> Joints;
typedef Eigen::Matrix<double, 4, 1> ToolMass;

class VelmaSim : public RTT::TaskContext
{
public:
  VelmaSim(const std::string& name) :
      RTT::TaskContext(name, PreOperational), joint_position_(DOFS), joint_velocity_(DOFS), joint_torque_command_(DOFS), joint_position_command_(POS_DOFS)
  {
    joint_position_ = Eigen::VectorXd::Zero(DOFS, 1);
    joint_velocity_ = Eigen::VectorXd::Zero(DOFS, 1);
    joint_torque_command_ = Eigen::VectorXd::Zero(DOFS, 1);
    joint_position_command_ = Eigen::VectorXd::Zero(POS_DOFS, 1);

    this->ports()->addPort("JointPosition", port_joint_position_);
    this->ports()->addPort("JointVelocity", port_joint_velocity_);

    this->ports()->addPort("JointTorqueCommand",
                        port_joint_torque_command_);

    port_joint_position_.setDataSample(joint_position_);
    port_joint_velocity_.setDataSample(joint_velocity_);

    this->addProperty("InitialState", init_state_);

  }

  bool configureHook()
  {

    if (init_state_.size() != DOFS)
    {
      return false;
    }

    state_ = State::Zero();

    for (size_t i = 0; i < DOFS; i++)
    {
      state_(i) = init_state_[i];
    }

    robot_ = this->getProvider<Robot>("robot");
    if (!robot_)
    {
      RTT::log(RTT::Error) << "Unable to load RobotService" << RTT::endlog();
      return false;
    }

    return true;
  }

  bool startHook()
  {
    return true;
  }

  void updateHook()
  {
    port_joint_torque_command_.read(joint_torque_command_);
    port_joint_position_command_.read(joint_position_command_);

    State deriv = stateDeriv(state_);

    state_.noalias() += deriv * this->getPeriod();

    //std::cout << "dupa" << std::endl;

    joint_position_ = state_.block(0, 0, DOFS, 1);
    joint_velocity_ = state_.block(DOFS, 0, DOFS, 1);

    port_joint_position_.write(joint_position_);
    port_joint_velocity_.write(joint_velocity_);
  }

private:

  State stateDeriv(const State& s) {
    Joints q, q_dotdot, tau;
    Inertia M, Mi;
    ToolMass tm[2];
    State ret;
    q = s.segment<DOFS>(0);
    robot_->inertia(M, q, tm);
    Mi = M.inverse();

    //std::cout << "Mi :" << Mi << std::endl;
    //std::cout << "tau : " << joint_torque_command_ << std::endl;
    q_dotdot.noalias() = Mi * joint_torque_command_;
    ret.segment<DOFS>(0) = s.segment<DOFS>(DOFS);
    ret.segment<DOFS>(DOFS) = q_dotdot;

    //std::cout << "ret :" << ret << std::endl;

    return ret;
  }

  RTT::OutputPort<Eigen::VectorXd> port_joint_position_;
  RTT::OutputPort<Eigen::VectorXd> port_joint_velocity_;

  RTT::InputPort<Eigen::VectorXd> port_joint_torque_command_;
  RTT::InputPort<Eigen::VectorXd> port_joint_position_command_;

  std::vector<double> init_state_;

  State state_;

  Eigen::VectorXd joint_position_;
  Eigen::VectorXd joint_velocity_;
  Eigen::VectorXd joint_torque_command_;
  Eigen::VectorXd joint_position_command_;

  boost::shared_ptr<Robot> robot_;

};

ORO_CREATE_COMPONENT(VelmaSim)
