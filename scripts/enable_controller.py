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

from controller_manager_msgs.srv import *
from std_msgs.msg import *
from diagnostic_msgs.msg import *
from geometry_msgs.msg import *

from tf.transformations import * 

from PyKDL import Rotation

class Test:
    def diagnostic_callback(self, data):
        for stat in data.status:
            if stat.name=='left_arm_ robot state':
                self.left_arm_power = (stat.values[0].value == '1111111')
            elif stat.name=='left_arm_ FRI state':
                self.left_arm_command = stat.values[1].value == 'command'
                self.left_fri_state = (stat.values[2].value == 'OK') or (stat.values[2].value == 'PERFECT')
            elif stat.name=='right_arm_ robot state':
                self.right_arm_power = (stat.values[0].value == '1111111')
            elif stat.name=='right_arm_ FRI state':
                self.right_arm_command = stat.values[1].value == 'command'
                self.right_fri_state = (stat.values[2].value == 'OK') or (stat.values[2].value == 'PERFECT')
        
#        print self.left_arm_power
#        print self.left_fri_state
        
        if self.state==0:
            if (self.left_arm_power and self.right_arm_power and self.left_fri_state and self.right_fri_state):
                if self.conmanSwitch(['CImp'], [], True):
                    self.state = 1;
        elif self.state==1:
            if (self.left_arm_power and self.right_arm_power and self.left_fri_state and self.right_fri_state):
                if self.conmanSwitch(['JntLimit'], [], True):
                    self.state = 2;
        elif self.state==2:
            if (self.left_arm_power and self.right_arm_power and self.left_fri_state and self.right_fri_state):
                if self.conmanSwitch(['PoseInt'], [], True):
                    self.state = 3;
        elif self.state==3:
            print 'dupa';
            if (self.left_arm_power and self.right_arm_power and self.left_fri_state and self.right_fri_state):
                xxx = Int32();
                xxx.data = 1;
                self.leftKRL_CMD.publish(xxx);
                self.rightKRL_CMD.publish(xxx);
                self.state = 4;
        
    def __init__(self):
        rospy.init_node('test_trj')
        
        rospy.wait_for_service('/controller_manager/switch_controller')
        self.conmanSwitch = rospy.ServiceProxy('/controller_manager/switch_controller', SwitchController)
        
        # publisher with imu data
        self.leftKRL_CMD = rospy.Publisher("/left_arm/KRL_CMD", Int32, queue_size=1, latch=True)
        self.rightKRL_CMD = rospy.Publisher("/right_arm/KRL_CMD", Int32, queue_size=1, latch=True)
        
        rospy.Subscriber("/diagnostic", DiagnosticArray, self.diagnostic_callback)
        
        self.left_arm_power = False;
        self.right_arm_power = False;
        
        self.left_arm_command = False;
        self.right_arm_command = False;
        
        self.left_fri_state = False;
        self.right_fri_state = False;
        
        self.state = 0;
        
    def spin(self):
        rospy.spin()

if __name__ == '__main__':
    test = Test()

    try:
        test.spin()
    except rospy.ROSInterruptException: pass
    except IOError: pass
    except KeyError: pass

