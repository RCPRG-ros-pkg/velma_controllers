#!/usr/bin/env python
import roslib; roslib.load_manifest('velma_controller')

import sys
import rospy
import math
import copy
import tf

from std_msgs.msg import ColorRGBA
from interactive_markers.interactive_marker_server import *
from interactive_markers.menu_handler import *
from visualization_msgs.msg import *
from geometry_msgs.msg import *
from tf.transformations import * 
import tf_conversions.posemath as pm
import PyKDL
from cartesian_trajectory_msgs.msg import *
import actionlib

class IntMarkersCimp:
    def __init__(self, prefix):
        self.prefix = prefix

        self.action_trajectory_client = None

        self.T_B_W = None
        self.T_W_T = None
        self.T_T_W = None

        self.listener = tf.TransformListener();

        # create an interactive marker server on the topic namespace simple_marker
        self.server = InteractiveMarkerServer('int_markers_cimp_' + self.prefix)

        while not rospy.is_shutdown():
            rospy.sleep(1)
            time_now = rospy.Time.now() - rospy.Duration(0.5)
            try:
                self.listener.waitForTransform('torso_base', self.prefix+'_arm_7_link', time_now, rospy.Duration(1.0))
                self.listener.waitForTransform(self.prefix+'_arm_7_link', self.prefix+'_arm_tool', time_now, rospy.Duration(1.0))
                break
            except:
                print "int_markers_cimp_[%s] script: waitForTransform error, retrying in 1s..."%(self.prefix)

        if rospy.is_shutdown():
            exit(0)

        self.getTransformations()

        self.insert6DofGlobalMarker()

        self.marker_state = "global"
        self.int_hide_marker = InteractiveMarker()
        self.int_hide_marker.header.frame_id = 'torso_base'
        self.int_hide_marker.name = self.prefix+'_arm_hide_marker';
        self.int_hide_marker.scale = 0.2
        self.int_hide_marker.pose.orientation = Quaternion(0,0,0,1)
        if self.prefix == "right":
            self.int_hide_marker.pose.position = Point(0, -0.5, 0)
            self.hide = self.createButtoMarkerControl(Point(0.15,0.15,0.15), Point(0.0, 0.0, 0.0), ColorRGBA(1,0,0,1) )
        else:
            self.int_hide_marker.pose.position = Point(0, 0.5, 0)
            self.hide = self.createButtoMarkerControl(Point(0.15,0.15,0.15), Point(0.0, 0.0, 0.0), ColorRGBA(0,1,0,1) )
        self.hide.interaction_mode = InteractiveMarkerControl.BUTTON
        self.hide.name = 'button_hide'
        self.int_hide_marker.controls.append( self.hide )
        self.server.insert(self.int_hide_marker, self.processFeedbackHide);
        
        self.server.applyChanges();

        self.action_trajectory_client = actionlib.SimpleActionClient("/" + self.prefix + "_arm/cartesian_trajectory", CartesianTrajectoryAction)
        print "int_markers_cimp_[%s] script: waiting for /[%s]_arm/cartesian_trajectory action..."%(self.prefix, self.prefix)
        self.action_trajectory_client.wait_for_server()
        print "int_markers_cimp_[%s] script: connected to action /[%s]_arm/cartesian_trajectory"%(self.prefix, self.prefix)

    def getTransformations(self):
        pose = self.listener.lookupTransform('torso_base', self.prefix+'_arm_7_link', rospy.Time(0))
        self.T_B_W = pm.fromTf(pose)

        pose_tool = self.listener.lookupTransform(self.prefix+'_arm_7_link', self.prefix+'_arm_tool', rospy.Time(0))
        self.T_W_T = pm.fromTf(pose_tool)
        self.T_T_W = self.T_W_T.Inverse()

    def erase6DofMarker(self):
        self.server.erase(self.prefix+'_arm_position_marker')
        self.server.applyChanges();

    def insert6DofGlobalMarker(self):
        T_B_T = self.T_B_W * self.T_W_T
        int_position_marker = InteractiveMarker()
        int_position_marker.header.frame_id = 'torso_base'
        int_position_marker.name = self.prefix+'_arm_position_marker'
        int_position_marker.scale = 0.2
        int_position_marker.pose = pm.toMsg(T_B_T)

        int_position_marker.controls.append(self.createInteractiveMarkerControl6DOF(InteractiveMarkerControl.ROTATE_AXIS,'x'));
        int_position_marker.controls.append(self.createInteractiveMarkerControl6DOF(InteractiveMarkerControl.ROTATE_AXIS,'y'));
        int_position_marker.controls.append(self.createInteractiveMarkerControl6DOF(InteractiveMarkerControl.ROTATE_AXIS,'z'));
        int_position_marker.controls.append(self.createInteractiveMarkerControl6DOF(InteractiveMarkerControl.MOVE_AXIS,'x'));
        int_position_marker.controls.append(self.createInteractiveMarkerControl6DOF(InteractiveMarkerControl.MOVE_AXIS,'y'));
        int_position_marker.controls.append(self.createInteractiveMarkerControl6DOF(InteractiveMarkerControl.MOVE_AXIS,'z'));

        box = self.createAxisMarkerControl(Point(0.15,0.015,0.015), Point(0.0, 0.0, 0.0) )
        box.interaction_mode = InteractiveMarkerControl.BUTTON
        box.name = 'button'
        int_position_marker.controls.append( box )
        self.server.insert(int_position_marker, self.processFeedback);
        self.server.applyChanges();

    def globalState(self):
        if self.marker_state == "global" or self.marker_state=="local":
           self.erase6DofMarker()

        self.marker_state = "global"
        self.insert6DofGlobalMarker()

    def hiddenState(self):
        if self.marker_state == "global" or self.marker_state=="local":
           self.erase6DofMarker()

        self.marker_state="hidden"

    def processFeedbackHide(self, feedback):
        if ( feedback.event_type == InteractiveMarkerFeedback.BUTTON_CLICK and feedback.control_name == "button_hide" ):
           if self.marker_state == "global":
               self.hiddenState()
           else:
               self.globalState();

    def processFeedback(self, feedback):
        if self.action_trajectory_client == None:
            return

        print "feedback", feedback.marker_name, feedback.control_name, feedback.event_type

        if ( feedback.marker_name == self.prefix+'_arm_position_marker' ) and ( feedback.event_type == InteractiveMarkerFeedback.BUTTON_CLICK ) and ( feedback.control_name == "button" ):
            T_B_Td = pm.fromMsg(feedback.pose)
            self.p = pm.toMsg(T_B_Td)
            duration = 5.0

            action_trajectory_goal = CartesianTrajectoryGoal()
            
            action_trajectory_goal.trajectory.header.stamp = rospy.Time.now() + rospy.Duration(0.1)
            
            action_trajectory_goal.trajectory.points.append(CartesianTrajectoryPoint(
            rospy.Duration(duration),
            self.p,
            Twist()))

            self.action_trajectory_client.send_goal(action_trajectory_goal)

            print "duration: %s"%(duration)
            print "pose: %s"%(self.p)

    def createSphereMarkerControl(self, scale, position, color):
        marker = Marker()
        marker.type = Marker.SPHERE
        marker.scale = scale
        marker.pose.position = position
        marker.color = color
        control = InteractiveMarkerControl()
        control.always_visible = True;
        control.markers.append( marker );
        return control

    def createBoxMarkerControl(self, scale, position):
        marker = Marker()
        marker.type = Marker.CUBE
        marker.scale = scale
        marker.pose.position = position
        marker.color = ColorRGBA(0.5,0.5,0.5,1)
        control = InteractiveMarkerControl()
        control.always_visible = True;
        control.markers.append( marker );
        return control

    def createAxisMarkerControl(self, scale, position):
        markerX = Marker()
        markerX.type = Marker.ARROW
        markerX.scale = scale
        markerX.pose.position = position
        ori = quaternion_about_axis(0, [0, 1 ,0])
        markerX.pose.orientation = Quaternion(ori[0], ori[1], ori[2], ori[3])
        markerX.color = ColorRGBA(1,0,0,1)
        markerY = Marker()
        markerY.type = Marker.ARROW
        markerY.scale = scale
        markerY.pose.position = position
        ori = quaternion_about_axis(math.pi/2.0, [0, 0 ,1])
        markerY.pose.orientation = Quaternion(ori[0], ori[1], ori[2], ori[3])
        markerY.color = ColorRGBA(0,1,0,1)
        markerZ = Marker()
        markerZ.type = Marker.ARROW
        markerZ.scale = scale
        markerZ.pose.position = position
        ori = quaternion_about_axis(-math.pi/2.0, [0, 1 ,0])
        markerZ.pose.orientation = Quaternion(ori[0], ori[1], ori[2], ori[3])
        markerZ.color = ColorRGBA(0,0,1,1)
        control = InteractiveMarkerControl()
        control.always_visible = True;
        control.markers.append( markerX );
        control.markers.append( markerY );
        control.markers.append( markerZ );
        return control

    def createButtoMarkerControl(self, scale, position, color):
        marker = Marker()
        marker.type = Marker.SPHERE
        marker.scale = scale
        marker.pose.position = position
        marker.pose.orientation = Quaternion(0,0,0,1)
        marker.color = color
        control = InteractiveMarkerControl()
        control.always_visible = True;
        control.markers.append( marker );
        return control

    def createInteractiveMarkerControl6DOF(self, mode, axis):
        control = InteractiveMarkerControl()
        control.orientation_mode = InteractiveMarkerControl.FIXED
        if mode == InteractiveMarkerControl.ROTATE_AXIS:
            control.name = 'rotate_';
        if mode == InteractiveMarkerControl.MOVE_AXIS:
            control.name = 'move_';
        if axis == 'x':
            control.orientation = Quaternion(1,0,0,1)
            control.name = control.name+'x';
        if axis == 'y':
            control.orientation = Quaternion(0,1,0,1)
            control.name = control.name+'x';
        if axis == 'z':
            control.orientation = Quaternion(0,0,1,1)
            control.name = control.name+'x';
        control.interaction_mode = mode
        return control

if __name__ == "__main__":

    rospy.init_node('int_markers_cimp', anonymous=False)

    int_right = IntMarkersCimp("right")
    int_left = IntMarkersCimp("left")

    rospy.spin()

