#ifndef _HEAD_LOOK_AT_H_
#define _HEAD_LOOK_AT_H_

class HeadLookAt {
public:
	HeadLookAt(double cam_v_offset);
	~HeadLookAt();
	int UpdateTorsoPose(double joint_position[2]);
	int UpdateTargetPosition(double xx, double yy, double zz);
	int CalculateHeadPose();
protected:
private:
	// Kinematic parameters
	const double h;
	// Current task parameters and results
	double x, y, z;
	double torso_joint[2];
	double head_joint[2];
};

#endif

