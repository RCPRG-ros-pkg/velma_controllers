#include <rtt/TaskContext.hpp>
#include <rtt/Port.hpp>
#include <rtt/Component.hpp>

#include <vector>
#include <Eigen/Dense>

#include <geometry_msgs/Point.h>

#include "head_kinematics.h"

#define H_ROT	0.687d
#define H_LEAN	0.375d
#define H_HEAD	0.385d
#define H_CAM	0.200d

#define HEAD_CTRL_MNJ 2
#define FULL_TORSO_MNJ 4
#define DEFAULT_TARGET_X	1.00d
#define DEFAULT_TARGET_Y	0.00d
#define DEFAULT_TARGET_Z	1.54d

class HeadLookAt3D : public RTT::TaskContext {
public:
	HeadLookAt3D(const std::string & name) : TaskContext(name, PreOperational) {

		this->ports()->addPort("HeadJointPositionCommand", port_HeadJointPositionCommand).doc("");
		this->ports()->addPort("TargetPoint", port_TargetPoint).doc("");
		this->ports()->addPort("JointPosition", port_JointPosition).doc("");
	}

	~HeadLookAt3D() {
	}

	bool configureHook() {
		head_jnt_pos_cmd_.resize(HEAD_CTRL_MNJ);
		port_HeadJointPositionCommand.setDataSample(head_jnt_pos_cmd_);
		jnt_pos_.resize(FULL_TORSO_MNJ);
		
		h = new HeadKinematics(H_ROT, H_LEAN, H_HEAD, H_CAM);
		
		current_point.x = DEFAULT_TARGET_X;
		current_point.y = DEFAULT_TARGET_Y;
		current_point.z = DEFAULT_TARGET_Z;

		return true;
	}

	bool startHook() {
		
		return true;
	}

	void stopHook() {
	}

	void updateHook() {
		double joint_pan, joint_tilt;
		RTT::FlowStatus target_point_fs, jnt_pos_fs;
		
		target_point_fs = port_TargetPoint.read(target_point_);
		if(target_point_fs == RTT::NewData){
			current_point = target_point_;
		}
		
		jnt_pos_fs = port_JointPosition.read(jnt_pos_);
		if(jnt_pos_fs == RTT::NewData){
			h->UpdateTorsoPose(jnt_pos_(0), jnt_pos_(1));
			h->UpdateTargetPosition(current_point.x, current_point.y, current_point.z);
			h->TransformTargetToHeadFrame();
			if(h->CalculateHeadPose(joint_pan, joint_tilt) == 0){
				head_jnt_pos_cmd_(0) = joint_pan;
				head_jnt_pos_cmd_(1) = joint_tilt;
			}
			port_HeadJointPositionCommand.write(head_jnt_pos_cmd_);
		}
	}

private:
	HeadKinematics *h;

	RTT::OutputPort<Eigen::VectorXd > port_HeadJointPositionCommand;
	RTT::InputPort<geometry_msgs::Point > port_TargetPoint;
	RTT::InputPort<Eigen::VectorXd > port_JointPosition;
	
	geometry_msgs::Point target_point_;
	geometry_msgs::Point current_point;
	Eigen::VectorXd head_jnt_pos_cmd_;
	Eigen::VectorXd jnt_pos_;

};

ORO_CREATE_COMPONENT(HeadLookAt3D)

