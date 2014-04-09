#ifndef _HEAD_LOOK_AT_H_
#define _HEAD_LOOK_AT_H_

class HeadKinematics {
public:
	HeadKinematics(double h_rot_, double h_lean_, double h_head_, double h_cam_);
	~HeadKinematics();
	int UpdateTorsoPose(double joint1, double joint2);
	int UpdateTargetPosition(double xx, double yy, double zz);
	int UpdateTargetPositionHeadFrame(double xx, double yy, double zz);
	int TransformTargetToHeadFrame();
	int CalculateHeadPose(double &joint_pan, double &joint_tilt);
	bool vb;	// verbose
protected:
private:
	// Kinematic parameters
	const double h_rot;	// height of torso rotating link
	const double h_lean; // length of torso tilting link
	const double h_head; // height of head pan-tilt joints frames w. respect to torso tilting link end
	const double h_cam;	// height of camera attachment point w. respect to head tilt frame
	// Current task parameters and results
	double xb, yb, zb;	//target coord. with respect to base frame
	double xh, yh, zh;	//target coord. with respect to head frame
	double torso_joint[2];
};

#endif

