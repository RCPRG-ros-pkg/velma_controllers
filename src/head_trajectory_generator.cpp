// Copyright 2014 WUT

#include <vector>
#include <string>

#include "rtt/TaskContext.hpp"
#include "rtt/Port.hpp"
#include "rtt/Component.hpp"

#include "Eigen/Dense"

#define TORSO_MNJ 4
#define HEAD_CTRL_MNJ 2
#define TEST_LOOP_T 5000

class HeadTrajectory : public RTT::TaskContext {
 public:
  explicit HeadTrajectory(const std::string & name) : TaskContext(name) {
    this->ports()->addPort("JointPositionCommand", port_JointPositionCommand).doc("");
    this->ports()->addPort("FakeJointPositionCommand", port_FakeJointPositionCommand).doc("");

    this->ports()->addPort("JointVelocity", port_JointVelocity).doc("");
    this->ports()->addPort("JointPosition", port_JointPosition).doc("");
  }

  ~HeadTrajectory() {
  }

  bool configureHook() {
    jnt_pos_.resize(TORSO_MNJ);
    jnt_vel_.resize(TORSO_MNJ);
    jnt_pos_cmd_.resize(HEAD_CTRL_MNJ);
    fake_jnt_pos_cmd_.resize(1);

    port_JointPositionCommand.setDataSample(jnt_pos_cmd_);
    port_FakeJointPositionCommand.setDataSample(fake_jnt_pos_cmd_);

    return true;
  }

  bool startHook() {
    loopCnt = TEST_LOOP_T;
    return true;
  }

  void stopHook() {
  }

  void updateHook() {
    int32_t pos0, pos1, pos2, pos3;
    // Test of 2,3 drives control
    if (++loopCnt > TEST_LOOP_T) {
      loopCnt = 0;
      testMoveDir = (testMoveDir == 5000) ? (-5000) : 5000;
      jnt_pos_cmd_(0) = 2 * testMoveDir;
      jnt_pos_cmd_(1) = testMoveDir;
      fake_jnt_pos_cmd_(0) = 0;
      port_JointPositionCommand.write(jnt_pos_cmd_);
      port_FakeJointPositionCommand.write(fake_jnt_pos_cmd_);
      std::cout << "Position Command Sent!" << std::endl;
    }
  }

 private:
  RTT::OutputPort<Eigen::VectorXd > port_JointPositionCommand;
  RTT::OutputPort<Eigen::VectorXd > port_FakeJointPositionCommand;

  RTT::InputPort<Eigen::VectorXd > port_JointVelocity;
  RTT::InputPort<Eigen::VectorXd > port_JointPosition;


  Eigen::VectorXd jnt_pos_;
  Eigen::VectorXd jnt_vel_;

  Eigen::VectorXd jnt_pos_cmd_;
  Eigen::VectorXd fake_jnt_pos_cmd_;

  bool synchro_;
  int loopCnt;
  int testMoveDir;
};

ORO_CREATE_COMPONENT(HeadTrajectory)


