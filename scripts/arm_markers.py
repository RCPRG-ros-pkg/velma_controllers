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

def getTransformations():
    global listener
    global prefix
    global T_B_W
    global T_W_T
    global T_T_W

    pose = listener.lookupTransform('torso_base', prefix+'_arm_7_link', rospy.Time(0))
    T_B_W = pm.fromTf(pose)

    pose_tool = listener.lookupTransform(prefix+'_arm_7_link', prefix+'_arm_tool', rospy.Time(0))
    T_W_T = pm.fromTf(pose_tool)
    T_T_W = T_W_T.Inverse()

def erase6DofMarker():
    global server
    server.erase(prefix+'_arm_position_marker')
    server.applyChanges();

def insert6DofGlobalMarker():
    global server
    global T_B_W
    global T_W_T

    T_B_T = T_B_W * T_W_T
    int_position_marker = InteractiveMarker()
    int_position_marker.header.frame_id = 'torso_base'
    int_position_marker.name = prefix+'_arm_position_marker'
    int_position_marker.scale = 0.2
    int_position_marker.pose = pm.toMsg(T_B_T)

    int_position_marker.controls.append(createInteractiveMarkerControl6DOF(InteractiveMarkerControl.ROTATE_AXIS,'x'));
    int_position_marker.controls.append(createInteractiveMarkerControl6DOF(InteractiveMarkerControl.ROTATE_AXIS,'y'));
    int_position_marker.controls.append(createInteractiveMarkerControl6DOF(InteractiveMarkerControl.ROTATE_AXIS,'z'));
    int_position_marker.controls.append(createInteractiveMarkerControl6DOF(InteractiveMarkerControl.MOVE_AXIS,'x'));
    int_position_marker.controls.append(createInteractiveMarkerControl6DOF(InteractiveMarkerControl.MOVE_AXIS,'y'));
    int_position_marker.controls.append(createInteractiveMarkerControl6DOF(InteractiveMarkerControl.MOVE_AXIS,'z'));

    box = createAxisMarkerControl(Point(0.15,0.015,0.015), Point(0.0, 0.0, 0.0) )
    box.interaction_mode = InteractiveMarkerControl.BUTTON
    box.name = 'button'
    int_position_marker.controls.append( box )
    server.insert(int_position_marker, processFeedback);
    server.applyChanges();

def globalState():
    global marker_state
    global listener
    global prefix

    if marker_state == "global" or marker_state=="local":
       erase6DofMarker()

    marker_state="global"
    insert6DofGlobalMarker()

def hiddenState():
    global marker_state

    if marker_state == "global" or marker_state=="local":
       erase6DofMarker()

    marker_state="hidden"

def processFeedbackHide(feedback):
    global marker_state

    if ( feedback.event_type == InteractiveMarkerFeedback.BUTTON_CLICK and feedback.control_name == "button_hide" ):
       if marker_state == "global":
           hiddenState()
       else:
           globalState();

def processFeedback(feedback):

    global prefix
    global listener
    global hand_arm_transform
    global tool
    global p
    global action_trajectory_client

    if feedback.marker_name == prefix+'_arm_position_marker':
         T_B_Td = pm.fromMsg(feedback.pose)

    if ( feedback.event_type == InteractiveMarkerFeedback.BUTTON_CLICK and feedback.control_name == "button" ):
        p = pm.toMsg(T_B_Td)
        duration = 5.0

        action_trajectory_goal = CartesianTrajectoryGoal()
        
        action_trajectory_goal.trajectory.header.stamp = rospy.Time.now() + rospy.Duration(0.1)
        
        action_trajectory_goal.trajectory.points.append(CartesianTrajectoryPoint(
        rospy.Duration(duration),
        p,
        Twist()))

        action_trajectory_client.send_goal(action_trajectory_goal)

        print "duration: %s"%(duration)
        print "pose: %s"%(p)

def loop():
    # start the ROS main loop
    global prefix
    global gripper
    global listener
    global T_B_W
    global T_W_T

    rospy.sleep(1)
    rate = rospy.Rate(10.0)
    while not rospy.is_shutdown():
        getTransformations()
        rate.sleep()

        m = Marker()
        m.header.frame_id = prefix+"_arm_tool"
        m.header.stamp = rospy.Time.now()
        m.ns = "tool_pose"
        m.type = Marker.ARROW
        m.scale = Point(0.15,0.005,0.005)

        m.id = 0
        ori = quaternion_about_axis(0, [0, 1 ,0])
        m.pose.orientation = Quaternion(ori[0], ori[1], ori[2], ori[3])
        m.color = ColorRGBA(0.5,0,0,1)
        pub.publish(m)

        m.id = 1
        ori = quaternion_about_axis(math.pi/2.0, [0, 0 ,1])
        m.pose.orientation = Quaternion(ori[0], ori[1], ori[2], ori[3])
        m.color = ColorRGBA(0,0.5,0,1)
        pub.publish(m)

        m.id = 2
        ori = quaternion_about_axis(-math.pi/2.0, [0, 1 ,0])
        m.pose.orientation = Quaternion(ori[0], ori[1], ori[2], ori[3])
        m.color = ColorRGBA(0,0,0.5,1)
        pub.publish(m)



def createSphereMarkerControl(scale, position, color):
    marker = Marker()
    marker.type = Marker.SPHERE
    marker.scale = scale
    marker.pose.position = position
    marker.color = color
    control = InteractiveMarkerControl()
    control.always_visible = True;
    control.markers.append( marker );
    return control

def createBoxMarkerControl(scale, position):
    marker = Marker()
    marker.type = Marker.CUBE
    marker.scale = scale
    marker.pose.position = position
    marker.color = ColorRGBA(0.5,0.5,0.5,1)
    control = InteractiveMarkerControl()
    control.always_visible = True;
    control.markers.append( marker );
    return control

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

def createButtoMarkerControl(scale, position, color):
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
    a = []
    for arg in sys.argv:
        a.append(arg)

    if len(a) < 2:
        print "Usage: %s prefix"%a[0]
        print "you provided:"
        for x in a:
            print x
        exit(0)

    prefix = a[1]
    if prefix != "left" and prefix != "right":
        print "Usage: %s prefix"%a[0]
        print "prefix should be left or right"
        print "you provided:"
        for x in a:
            print x
        exit(0)

    rospy.init_node(prefix+'_arm_markers', anonymous=True)

    T_B_W = None
    T_W_T = None
    T_T_W = None

    listener = tf.TransformListener();

    # create an interactive marker server on the topic namespace simple_marker
    server = InteractiveMarkerServer(prefix+'_arm_markers')

    rospy.sleep(2)

    listener.waitForTransform('torso_base', prefix+'_arm_7_link', rospy.Time(0), rospy.Duration(60.0))
    listener.waitForTransform(prefix+'_arm_7_link', prefix+'_arm_tool', rospy.Time(0), rospy.Duration(60.0))

    getTransformations()

    insert6DofGlobalMarker()

    marker_state="global"
    int_hide_marker = InteractiveMarker()
    int_hide_marker.header.frame_id = 'torso_base'
    int_hide_marker.name = prefix+'_arm_hide_marker';
    int_hide_marker.scale = 0.2
    int_hide_marker.pose.orientation = Quaternion(0,0,0,1)
    if prefix == "right":
        int_hide_marker.pose.position = Point(0, -0.5, 0)
        hide = createButtoMarkerControl(Point(0.15,0.15,0.15), Point(0.0, 0.0, 0.0), ColorRGBA(1,0,0,1) )
    else:
        int_hide_marker.pose.position = Point(0, 0.5, 0)
        hide = createButtoMarkerControl(Point(0.15,0.15,0.15), Point(0.0, 0.0, 0.0), ColorRGBA(0,1,0,1) )
    hide.interaction_mode = InteractiveMarkerControl.BUTTON
    hide.name = 'button_hide'
    int_hide_marker.controls.append( hide )
    server.insert(int_hide_marker, processFeedbackHide);
    
    server.applyChanges();

    pub = rospy.Publisher('/' + prefix + '_arm/tool_marker', Marker, queue_size=100)

    action_trajectory_client = actionlib.SimpleActionClient("/" + prefix + "_arm/cartesian_trajectory", CartesianTrajectoryAction)
    action_trajectory_client.wait_for_server()

    loop()


