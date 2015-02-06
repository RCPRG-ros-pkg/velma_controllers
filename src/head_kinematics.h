// Copyright 2014 WUT
#ifndef HEAD_KINEMATICS_H_
#define HEAD_KINEMATICS_H_

#define V_ROT   0.8d
#define V_LEAN  0.375d
#define V_HEAD  0.39224d
#define H_CAM   0.0     // horizontal displacement of camera w. respect to head tilt frame
#define V_CAM   0.225d  // vertical displacement of camera w. respect to head tilt frame

class HeadKinematics {
 public:
  HeadKinematics(double h_rot_, double h_lean_, double h_head_, double y_cam_, double z_cam_);
  ~HeadKinematics();
  int UpdateTorsoPose(double joint1, double joint2);
  int UpdateTargetPosition(double xx, double yy, double zz);
  int UpdateTargetPositionHeadFrame(double xx, double yy, double zz);
  int TransformTargetToHeadFrame();
  int CalculateHeadPose(double *joint_pan, double *joint_tilt);
  bool vb;  // verbose
 protected:
 private:
  // Kinematic parameters
  const double h_rot;  // height of torso rotating link
  const double h_lean;  // length of torso tilting link
  const double h_head;  // height of head pan-tilt joints frames w. respect to torso tilting link end
  const double h_cam;  // horizontal displacement of camera w. respect to head tilt frame
  const double v_cam;  // vertical displacement of camera w. respect to head tilt frame
  // Current task parameters and results
  double xb, yb, zb;  // target coord. with respect to base frame
  double xh, yh, zh;  // target coord. with respect to head frame
  double torso_joint[2];
};

#endif  // HEAD_KINEMATICS_H_

