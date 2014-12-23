// Copyright 2014 WUT

#include "head_kinematics.h"

#include <iostream>
#include <cmath>
#include <cstdlib>
#include <string>

using namespace std;

main(int argc, char* argv[]) {
  double head_joint[2];

  if ((argc == 5) && (argv[1] == std::string("--head"))) {
    HeadKinematics h(H_ROT, H_LEAN, H_HEAD, H_CAM);
    h.vb = true;
    h.UpdateTargetPositionHeadFrame(strtod(argv[2], NULL), strtod(argv[3], NULL), strtod(argv[4], NULL));
    h.CalculateHeadPose(head_joint[0], head_joint[1]);
  } else if ((argc == 7) && (argv[1] == std::string("--base"))) {
    HeadKinematics h(H_ROT, H_LEAN, H_HEAD, H_CAM);
    h.vb = true;
    h.UpdateTorsoPose(M_PI * strtod(argv[5], NULL) / 180, M_PI * strtod(argv[6], NULL) / 180);
    h.UpdateTargetPosition(strtod(argv[2], NULL), strtod(argv[3], NULL), strtod(argv[4], NULL));
    h.TransformTargetToHeadFrame();
    h.CalculateHeadPose(head_joint[0], head_joint[1]);
  } else {
    cout << "usage: " << argv[0] << " --head target_x_head target_y_head terget_z_head" << endl;
    cout << "usage: " << argv[0]
         << " --base target_x_base target_y_base terget_z_base torso_0_joint torso_1_joint" << endl;
    return -1;
  }
}

HeadKinematics::HeadKinematics(double h_rot_, double h_lean_, double h_head_, double h_cam_):
  h_rot(h_rot_), h_lean(h_lean_), h_head(h_head_), h_cam(h_cam_) {
  vb = false;
}

HeadKinematics::~HeadKinematics() {
}

int HeadKinematics::UpdateTorsoPose(double joint1, double joint2) {
  torso_joint[0] = joint1;
  torso_joint[1] = joint2 + M_PI / 2;
}

int HeadKinematics::UpdateTargetPosition(double xx, double yy, double zz) {
  xb = xx;
  yb = yy;
  zb = zz;
}

int HeadKinematics::UpdateTargetPositionHeadFrame(double xx, double yy, double zz) {
  xh = xx;
  yh = yy;
  zh = zz;
}

int HeadKinematics::TransformTargetToHeadFrame() {
  double x1, y1, z1;  // Target coordinates with respect to torso rotating link
  // double xh, yh, zh; // Target coordinates with respect to head frame (after torso lean link)

  if (vb) cout << "Target in base coordinates: " << xb << " " << yb << " " << zb << endl;

  x1 = xb * cos(torso_joint[0]) + yb * sin(torso_joint[0]);
  y1 = -xb * sin(torso_joint[0]) + yb * cos(torso_joint[0]);
  z1 = zb - h_rot;
  if (vb) cout << "Target in torso rot coordinates: " << x1 << " " << y1 << " " << z1 << endl;

  xh = x1 - h_lean * sin(torso_joint[1]);
  yh = y1;
  zh = z1 - h_lean * cos(torso_joint[1]) - h_head;
  if (vb) cout << "Target in head coordinates: " << xh << " " << yh << " " << zh << endl;
}

int HeadKinematics::CalculateHeadPose(double &joint_pan, double &joint_tilt) {
  double theta1, theta2;
  double alpha;
  double beta;
  double x1, y1, z1;

  // Theta1
  if ((xh == 0) && (yh == 0)) {
    return -1;
    if (vb) cout << "Theta1: NaN" << endl;
  }
  theta1 = atan2(yh, xh);
  x1 = xh / cos(theta1);
  z1 = zh;
  if (vb) cout << "Theta1: " << 180 * theta1 / M_PI << endl;

  // Theta2
  if ((x1 == 0) && (z1 == 0)) {
    return -1;
    if (vb) cout << "Theta2: NaN" << endl;
  }
  alpha = atan2(x1, -z1);
  if (vb) cout << "Alpha: " << 180 * alpha / M_PI << endl;

  beta = acos(h_cam / sqrt(xh * xh + yh * yh + zh * zh));
  if (vb) cout << "Beta: " << 180 * beta / M_PI << endl;

  theta2 = M_PI - (alpha + beta);
  if (vb) cout << "Theta2: " << 180 * theta2 / M_PI << endl;

  joint_pan = theta1;
  joint_tilt = theta2;
  return 0;
}




