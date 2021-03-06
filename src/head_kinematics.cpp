// Copyright 2014 WUT

#include "head_kinematics.h"

#include <iostream>
#include <cmath>
#include <cstdlib>
#include <string>

main(int argc, char* argv[]) {
  double head_joint[2];

  if ((argc == 5) && (argv[1] == std::string("--head"))) {
    HeadKinematics h(V_ROT, V_LEAN, V_HEAD, H_CAM, V_CAM);
    h.vb = true;
    h.UpdateTargetPositionHeadFrame(strtod(argv[2], NULL), strtod(argv[3], NULL), strtod(argv[4], NULL));
    h.CalculateHeadPose(&head_joint[0], &head_joint[1]);
  } else if ((argc == 7) && (argv[1] == std::string("--base"))) {
    HeadKinematics h(V_ROT, V_LEAN, V_HEAD, H_CAM, V_CAM);
    h.vb = true;
    h.UpdateTorsoPose(M_PI * strtod(argv[5], NULL) / 180, M_PI * strtod(argv[6], NULL) / 180);
    h.UpdateTargetPosition(strtod(argv[2], NULL), strtod(argv[3], NULL), strtod(argv[4], NULL));
    h.TransformTargetToHeadFrame();
    h.CalculateHeadPose(&head_joint[0], &head_joint[1]);
  } else if ((argc == 8) && (argv[1] == std::string("--cam")) && (argv[4] == std::string("--head"))) {
    HeadKinematics h(V_ROT, V_LEAN, V_HEAD, strtod(argv[2], NULL), strtod(argv[3], NULL));
    h.vb = true;
    h.UpdateTargetPositionHeadFrame(strtod(argv[5], NULL), strtod(argv[6], NULL), strtod(argv[7], NULL));
    h.CalculateHeadPose(&head_joint[0], &head_joint[1]);
  } else if ((argc == 10) && (argv[1] == std::string("--cam")) && (argv[4] == std::string("--base"))) {
    HeadKinematics h(V_ROT, V_LEAN, V_HEAD, strtod(argv[2], NULL), strtod(argv[3], NULL));
    h.vb = true;
    h.UpdateTorsoPose(M_PI * strtod(argv[8], NULL) / 180, M_PI * strtod(argv[9], NULL) / 180);
    h.UpdateTargetPosition(strtod(argv[5], NULL), strtod(argv[6], NULL), strtod(argv[7], NULL));
    h.TransformTargetToHeadFrame();
    h.CalculateHeadPose(&head_joint[0], &head_joint[1]);
  } else {
    std::cout << "usage: " << argv[0] << " --head target_x_head target_y_head target_z_head" << std::endl;
    std::cout << "usage: " << argv[0]
         << " --base target_x_base target_y_base terget_z_base torso_0_joint torso_1_joint" << std::endl;
    std::cout << "usage: " << argv[0] << " --cam h_offs v_offs --head target_x_head target_y_head target_z_head" << std::endl;
    std::cout << "usage: " << argv[0]
         << " --cam h_offs v_offs --base target_x_base target_y_base terget_z_base torso_0_joint torso_1_joint" << std::endl;
    return -1;
  }
}

HeadKinematics::HeadKinematics(double h_rot_, double h_lean_, double h_head_, double h_cam_, double v_cam_):
  h_rot(h_rot_), h_lean(h_lean_), h_head(h_head_), h_cam(h_cam_), v_cam(v_cam_) {
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

  if (vb) std::cout << "Target in base coordinates: " << xb << " " << yb << " " << zb << std::endl;

  x1 = xb * cos(torso_joint[0]) + yb * sin(torso_joint[0]);
  y1 = -xb * sin(torso_joint[0]) + yb * cos(torso_joint[0]);
  z1 = zb - h_rot;
  if (vb) std::cout << "Target in torso rot coordinates: " << x1 << " " << y1 << " " << z1 << std::endl;

  xh = x1 - h_lean * sin(torso_joint[1]);
  yh = y1;
  zh = z1 - h_lean * cos(torso_joint[1]) - h_head;
  if (vb) std::cout << "Target in head coordinates: " << xh << " " << yh << " " << zh << std::endl;
}

int HeadKinematics::CalculateHeadPose(double *joint_pan, double *joint_tilt) {
  double theta1, theta2;
  double alpha;
  double beta;
  double x1, y1, z1;

  // Theta1
  if ((xh == 0) && (yh == 0)) {
    return -1;
    if (vb) std::cout << "Theta1: NaN" << std::endl;
  }
  if ((xh*xh + yh*yh < v_cam*v_cam)) {
    return -1;
    if (vb) std::cout << "Theta1: NaN" << std::endl;
  }
  theta1 = asin(h_cam / sqrt(xh*xh+yh*yh)) + atan2(yh, xh);
  
  
  x1 = xh / cos(theta1);
  z1 = zh;
  if (vb) std::cout << "Theta1: " << 180 * theta1 / M_PI << std::endl;

  // Theta2
  if ((x1 == 0) && (z1 == 0)) {
    return -1;
    if (vb) std::cout << "Theta2: NaN" << std::endl;
  }
  alpha = atan2(x1, -z1);
  if (vb) std::cout << "Alpha: " << 180 * alpha / M_PI << std::endl;

  beta = acos(v_cam / sqrt(xh * xh + yh * yh + zh * zh));
  if (vb) std::cout << "Beta: " << 180 * beta / M_PI << std::endl;

  theta2 = M_PI - (alpha + beta);
  if (vb) std::cout << "Theta2: " << 180 * theta2 / M_PI << std::endl;

  *joint_pan = theta1;
  *joint_tilt = theta2;
  return 0;
}




