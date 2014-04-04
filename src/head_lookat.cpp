
#include "head_lookat.h"

#include <iostream>
#include <cmath>
#include <cstdlib>

#define CAM_V_OFFSET	0.2d

using namespace std;

int main(int argc, char* argv[]){
	if(argc!=4)
		return -1;
	HeadLookAt h(CAM_V_OFFSET);
	h.UpdateTargetPosition(strtod(argv[1], NULL), strtod(argv[2], NULL), strtod(argv[3], NULL));
	h.CalculateHeadPose();
}

HeadLookAt::HeadLookAt(double cam_v_offset):
		h(cam_v_offset){
	
}

HeadLookAt::~HeadLookAt() {

}

int HeadLookAt::UpdateTorsoPose(double joint_position[2]){
	torso_joint[0] = joint_position[0];
	torso_joint[1] = joint_position[1];
}

int HeadLookAt::UpdateTargetPosition(double xx, double yy, double zz){
	x = xx;
	y = yy;
	z = zz;
}

int HeadLookAt::CalculateHeadPose(){
	double theta1, theta2;
	double alpha;
	double beta;
	double x1, y1, z1;

	// Theta1
	if(x<=0){
		return -1;
	} else{
		theta1 = atan2(-y, x);
		cout << "Theta1: " << 180*theta1/M_PI << endl;
		x1 = x/cos(theta1);
		z1 = z;
	}
	
	// Theta2
	if(z>=h){
		return -1;
	} else{
		alpha = atan2(x1, -z1);
		cout << "Alpha: " << 180*alpha/M_PI << endl;
	}
	
	beta = acos(h/sqrt(x*x + y*y + z*z));
	cout << "Beta: " << 180*beta/M_PI << endl;
	
	theta2 = M_PI - (alpha + beta);
	cout << "Theta2: " << 180*theta2/M_PI << endl;
}


