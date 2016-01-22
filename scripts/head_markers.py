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
import actionlib
import control_msgs.msg
import trajectory_msgs.msg

import random

def insert6DofGlobalMarker():
    global server

    int_position_marker = InteractiveMarker()
    int_position_marker.header.frame_id = 'head_pan_link'
    int_position_marker.name = 'head_position_marker'
    int_position_marker.scale = 0.2
#    int_position_marker.pose = pm.toMsg(T_B_T)

#    int_position_marker.controls.append(createInteractiveMarkerControl6DOF(InteractiveMarkerControl.ROTATE_AXIS,'x'));
    int_position_marker.controls.append(createInteractiveMarkerControl6DOF(InteractiveMarkerControl.ROTATE_AXIS,'y'));
    int_position_marker.controls.append(createInteractiveMarkerControl6DOF(InteractiveMarkerControl.ROTATE_AXIS,'z'));
#    int_position_marker.controls.append(createInteractiveMarkerControl6DOF(InteractiveMarkerControl.MOVE_AXIS,'x'));
#    int_position_marker.controls.append(createInteractiveMarkerControl6DOF(InteractiveMarkerControl.MOVE_AXIS,'y'));
#    int_position_marker.controls.append(createInteractiveMarkerControl6DOF(InteractiveMarkerControl.MOVE_AXIS,'z'));

    box = createAxisMarkerControl(Point(0.15,0.015,0.015), Point(0.0, 0.0, 0.0) )
    box.interaction_mode = InteractiveMarkerControl.BUTTON
    box.name = 'button'
    int_position_marker.controls.append( box )
    server.insert(int_position_marker, processFeedback);
    server.applyChanges();

def processFeedback(feedback):

    global p
    global action_trajectory_client

    if action_trajectory_client == None:
        return

    print "feedback", feedback.marker_name, feedback.control_name, feedback.event_type

    if ( feedback.marker_name == 'head_position_marker' ) and ( feedback.event_type == InteractiveMarkerFeedback.BUTTON_CLICK ) and ( feedback.control_name == "button" ):
        T_B_Td = pm.fromMsg(feedback.pose)
        p = pm.toMsg(T_B_Td)
        duration = 5.0


        action_trajectory_goal = control_msgs.msg.FollowJointTrajectoryGoal()
        
        action_trajectory_goal.trajectory.header.stamp = rospy.Time.now() + rospy.Duration(0.1)
        action_trajectory_goal.trajectory.joint_names = ['head_pan_joint', 'head_tilt_joint']
        pt = trajectory_msgs.msg.JointTrajectoryPoint()
        pt.positions = [random.uniform(-1,1), random.uniform(-0.5,0.5)]
        pt.time_from_start = rospy.Duration(duration)
        action_trajectory_goal.trajectory.points = [pt]
        
#        action_trajectory_goal.trajectory.points.append(CartesianTrajectoryPoint(
#        rospy.Duration(duration),
#        p,
#        Twist()))

        action_trajectory_client.send_goal(action_trajectory_goal)

        print "duration: %s"%(duration)
        print "pose: %s"%(p)

def createAxisMarkerControl(scale, position):
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

def createInteractiveMarkerControl6DOF(mode, axis):
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

    rospy.init_node('head_markers', anonymous=True)

    action_trajectory_client = None

    # create an interactive marker server on the topic namespace simple_marker
    server = InteractiveMarkerServer('head_markers')

    insert6DofGlobalMarker()

    server.applyChanges();

    action_trajectory_client = actionlib.SimpleActionClient("/head_spline_trajectory_action_joint", control_msgs.msg.FollowJointTrajectoryAction)
    action_trajectory_client.wait_for_server()

    rospy.spin()

