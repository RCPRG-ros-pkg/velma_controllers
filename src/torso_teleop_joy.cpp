#include <rtt/TaskContext.hpp>
#include <rtt/Port.hpp>
#include <rtt/Component.hpp>

#include <vector>
#include <Eigen/Dense>

#include "sensor_msgs/Joy.h"

#define FULL_TORSO_MNJ 4
#define HEAD_CTRL_MNJ 2
#define TORSO_CTRL_MNJ 1
#define TEST_LOOP_T 5000

#define MAX_VEL_X	50000
#define MAX_VEL_Y	20000
#define MAX_VEL_T	200
#define MAX_TRQ_T	35

class TorsoTeleopJoy : public RTT::TaskContext {
public:
	TorsoTeleopJoy(const std::string & name) : TaskContext(name) {

		this->ports()->addPort("HeadJointPositionCommand", port_HeadJointPositionCommand).doc("");
		this->ports()->addPort("TorsoJointPositionCommand", port_TorsoJointPositionCommand).doc("");
		this->ports()->addPort("TorsoJointTorqueCommand", port_TorsoJointTorqueCommand).doc("");
		this->ports()->addPort("NullSpaceTorqueCommand", port_NullSpaceTorqueCommand).doc("");

		this->ports()->addPort("JointVelocity", port_JointVelocity).doc("");
		this->ports()->addPort("JointPosition", port_JointPosition).doc("");
		
		this->ports()->addPort("Joy", port_Joy).doc("");
	}

	~TorsoTeleopJoy() {
	}

	bool configureHook() {
		jnt_pos_.resize(FULL_TORSO_MNJ);
		jnt_vel_.resize(FULL_TORSO_MNJ);
		head_jnt_pos_cmd_.resize(HEAD_CTRL_MNJ);
		torso_jnt_pos_cmd_.resize(TORSO_CTRL_MNJ);
		torso_jnt_trq_cmd_.resize(TORSO_CTRL_MNJ);
		nullspace_torque_command_.resize(16);
		for(int i=0; i<16; i++)
			nullspace_torque_command_(i) = 0;
		

		port_HeadJointPositionCommand.setDataSample(head_jnt_pos_cmd_);
		port_TorsoJointPositionCommand.setDataSample(torso_jnt_pos_cmd_);
		port_TorsoJointTorqueCommand.setDataSample(torso_jnt_trq_cmd_);
		port_NullSpaceTorqueCommand.setDataSample(nullspace_torque_command_);
		
		set1000PosX = 0;
		set1000PosY = 0;
		setPosX = 0;
		setPosY = 0;
		setVelX = 0;
		setVelY = 0;
		set1000PosT = 0;
		setPosT = 0;
		setVelT = 0;
		setTrqT = 0;

		return true;
	}

	bool startHook() {
		return true;
	}

	void stopHook() {
	}

	void updateHook() {
		int32_t pos0, pos1, pos2, pos3;
		RTT::FlowStatus joy_fs;
		// Test of 2,3 drives control
		/*if(++loopCnt > TEST_LOOP_T){
			loopCnt = 0;
			testMoveDir = (testMoveDir == 5000) ? (-5000) : 5000;
			jnt_pos_cmd_(0) = 2*testMoveDir;
			jnt_pos_cmd_(1) = testMoveDir;
			port_JointPositionCommand.write(jnt_pos_cmd_);
			std::cout << "Sent!" << std::endl;
		}*/
		
		joy_fs = port_Joy.read(joy_);
		if(joy_fs == RTT::NewData){
			setVelX = - joy_.axes[0] * MAX_VEL_X * ((joy_.buttons[6] || joy_.buttons[4]) ? 1.0 : 0.1);
			setVelY = - joy_.axes[1] * MAX_VEL_Y * ((joy_.buttons[6] || joy_.buttons[4]) ? 1.0 : 0.1);
			setVelT = - joy_.axes[4] * MAX_VEL_T * ((joy_.buttons[6] || joy_.buttons[4]) ? 1.0 : 0.6);
			setTrqT = - joy_.axes[3] * MAX_TRQ_T * ((joy_.buttons[6] || joy_.buttons[4]) ? 1.0 : 0.6);
			//std::cout<<"setVelX "<<setVelX<<" setVelY "<<setVelY<< std::endl;
		}
		set1000PosX += setVelX;
		set1000PosY += setVelY;
		set1000PosT += setVelT;
		setPosX = set1000PosX / 1000;
		setPosY = set1000PosY / 1000;
		setPosT = set1000PosT / 1000;
		head_jnt_pos_cmd_(0) = setPosX;
		head_jnt_pos_cmd_(1) = setPosY;
		port_HeadJointPositionCommand.write(head_jnt_pos_cmd_);
		torso_jnt_pos_cmd_(0) = setPosT;
		port_TorsoJointPositionCommand.write(torso_jnt_pos_cmd_);
		torso_jnt_trq_cmd_(0) = setTrqT;
		port_TorsoJointTorqueCommand.write(torso_jnt_trq_cmd_);
		nullspace_torque_command_(0) = setTrqT;
		port_NullSpaceTorqueCommand.write(nullspace_torque_command_);
	}

private:
	RTT::OutputPort<Eigen::VectorXd > port_HeadJointPositionCommand;
	RTT::OutputPort<Eigen::VectorXd > port_TorsoJointPositionCommand;
	RTT::OutputPort<Eigen::VectorXd > port_TorsoJointTorqueCommand;
	RTT::OutputPort<Eigen::VectorXd > port_NullSpaceTorqueCommand;

	RTT::InputPort<Eigen::VectorXd > port_JointVelocity;
	RTT::InputPort<Eigen::VectorXd > port_JointPosition;

	RTT::InputPort<sensor_msgs::Joy > port_Joy;

	Eigen::VectorXd jnt_pos_;
	Eigen::VectorXd jnt_vel_;
	
	sensor_msgs::Joy joy_;

	Eigen::VectorXd head_jnt_pos_cmd_;
	Eigen::VectorXd torso_jnt_pos_cmd_;
	Eigen::VectorXd torso_jnt_trq_cmd_;
	Eigen::VectorXd nullspace_torque_command_;

	bool synchro_;
	int loopCnt;
	int testMoveDir;
	
	long int set1000PosX, set1000PosY, set1000PosT;
	int setPosX, setPosY, setPosT;
	float setVelX, setVelY, setVelT;
	int setTrqT;
};

ORO_CREATE_COMPONENT(TorsoTeleopJoy)

