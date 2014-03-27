#!/usr/bin/env python

# Software License Agreement (BSD License)
#
# Copyright (c) 2011, Robot Control and Pattern Recognition Group, Warsaw University of Technology
#
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
# * Redistributions of source code must retain the above copyright
# notice, this list of conditions and the following disclaimer.
# * Redistributions in binary form must reproduce the above copyright
# notice, this list of conditions and the following disclaimer in the
# documentation and/or other materials provided with the distribution.
# * Neither the name of the <organization> nor the
# names of its contributors may be used to endorse or promote products
# derived from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
# ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
# WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL <COPYRIGHT HOLDER> BE LIABLE FOR ANY
# DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
# (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
# ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
# (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
# SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

import rospy
import tf

from cartesian_trajectory_msgs.msg import *
from geometry_msgs.msg import *

from tf.transformations import * 

from PyKDL import Rotation

class Test:
    def __init__(self):
        rospy.init_node('test_trj')
    
        self.frame_id = 'torso_base'
        self.prev_time = rospy.Time.now()

        # publisher with imu data
        self.right_pub = rospy.Publisher("/right_arm/trajectory", CartesianTrajectory)
        self.left_pub = rospy.Publisher("/left_arm/trajectory", CartesianTrajectory)
        self.listener = tf.TransformListener();
        rospy.sleep(1.0)
    def spin(self):
        self.prev_time = rospy.Time.now()

        trjr = CartesianTrajectory()
        
        trjr.header.stamp = rospy.Time.now() + rospy.Duration(0.1)
        
        trjr.points.append(CartesianTrajectoryPoint(
        rospy.Duration(10.0),
        Pose(Point(0.3, -0.50, 1.18), Quaternion(0.0, 0.0, 0.0, 1.0)),
        Twist()))
        
        trjl = CartesianTrajectory()
        
        trjl.header.stamp = trjr.header.stamp
        
        trjl.points.append(CartesianTrajectoryPoint(
        rospy.Duration(10.0),
        Pose(Point(0.3, 0.50, 1.18), Quaternion(0.0, 0.0, 1.0, 0.0)),
        Twist()))
        
        self.right_pub.publish(trjr)
        self.left_pub.publish(trjl)
        
        while not rospy.is_shutdown():
          rospy.sleep(1.0)

if __name__ == '__main__':
    test = Test()

    try:
        test.spin()
    except rospy.ROSInterruptException: pass
    except IOError: pass
    except KeyError: pass

