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
from cartesian_trajectory_msgs.msg import *
import actionlib

def processFeedback(feedback):

    global prefix
    global listener
    global hand_arm_transform
    global tool
    global p
    global action_trajectory_client

    if feedback.marker_name == 'arm_position_marker':
         gripper = feedback.pose

    if ( feedback.event_type == InteractiveMarkerFeedback.BUTTON_CLICK and feedback.control_name == "button" ):
        real_gripper = listener.lookupTransform('torso_base', prefix+'_HandPalmLink', rospy.Time(0))
        real_tool = listener.lookupTransform('torso_base', prefix+'_arm_7_link', rospy.Time(0))
        p = pm.toMsg(pm.fromMsg(gripper) * tool)
        dx = p.position.x-real_tool[0][0]
        dy = p.position.y-real_tool[0][1]
        dz = p.position.z-real_tool[0][2]
        length = math.sqrt(dx*dx + dy*dy + dz*dz)
        qw = quaternion_multiply(real_tool[1], quaternion_inverse([p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w]))[3]
        if qw>0.99999:
            qw=0.99999
        if qw<-0.99999:
            qw=-0.99999
        angle = abs(2.0 * math.acos(qw))

        duration = length*20
        if angle*2>duration:
            duration = angle*2

        action_trajectory_goal = CartesianTrajectoryGoal()
        
        action_trajectory_goal.trajectory.header.stamp = rospy.Time.now() + rospy.Duration(0.1)
        
        action_trajectory_goal.trajectory.points.append(CartesianTrajectoryPoint(
        rospy.Duration(duration),
        p,
        Twist()))

        action_trajectory_goal.path_tolerance.position = Vector3(0.13,0.13,0.13)
        action_trajectory_goal.path_tolerance.rotation = Vector3(3.0/180.0*math.pi,3.0/180.0*math.pi,3.0/180.0*math.pi)
        action_trajectory_goal.goal_tolerance.position = Vector3(0.005,0.005,0.005)
        action_trajectory_goal.goal_tolerance.rotation = Vector3(1.0/180.0*math.pi,1.0/180.0*math.pi,1.0/180.0*math.pi)
        action_trajectory_client.send_goal(action_trajectory_goal)

        print "duration: %s"%(duration)
        print "pose: %s"%(p)

def loop():
    # start the ROS main loop
    global prefix
    global gripper
    global real_gripper
    global listener
    global p

    rospy.sleep(1)
    rate = rospy.Rate(10.0)
    while not rospy.is_shutdown():
        rate.sleep()
        real_gripper = listener.lookupTransform('torso_base', prefix+'_HandPalmLink', rospy.Time(0))

        marker = Marker()
        marker.header.frame_id = 'torso_base'
        marker.header.stamp = rospy.Time.now()
        marker.ns = 'torso_base'
        marker.id = 0
        marker.type = 0
        marker.action = 0
        marker.points.append(gripper.position)
        marker.points.append(Point(real_gripper[0][0], real_gripper[0][1], real_gripper[0][2]))
        marker.pose.position.x = 0
        marker.pose.position.y = 0
        marker.pose.position.z = 0
        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = 0.0;
        marker.pose.orientation.w = 1.0;
        marker.scale.x = 0.001;
        marker.scale.y = 0.002;
        marker.scale.z = 0.0;
        marker.color.a = 1.0;
        marker.color.r = 0.0;
        marker.color.g = 0.0;
        marker.color.b = 1.0;
        pub.publish(marker)

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
    marker.color.r = 0.5
    marker.color.g = 0.5
    marker.color.b = 0.5
    marker.color.a = 1.0
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
    markerX.color.r = 1
    markerX.color.g = 0
    markerX.color.b = 0
    markerX.color.a = 1.0
    markerY = Marker()
    markerY.type = Marker.ARROW
    markerY.scale = scale
    markerY.pose.position = position
    ori = quaternion_about_axis(math.pi/2.0, [0, 0 ,1])
    markerY.pose.orientation = Quaternion(ori[0], ori[1], ori[2], ori[3])
    markerY.color.r = 0
    markerY.color.g = 1
    markerY.color.b = 0
    markerY.color.a = 1.0
    markerZ = Marker()
    markerZ.type = Marker.ARROW
    markerZ.scale = scale
    markerZ.pose.position = position
    ori = quaternion_about_axis(-math.pi/2.0, [0, 1 ,0])
    markerZ.pose.orientation = Quaternion(ori[0], ori[1], ori[2], ori[3])
    markerZ.color.r = 0
    markerZ.color.g = 0
    markerZ.color.b = 1
    markerZ.color.a = 1.0
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
    a = []
    for arg in sys.argv:
        a.append(arg)

    if 2 != len(a):
        print "Usage: %s prefix"%a[0]
        exit(0)

    prefix = a[1]

    rospy.init_node(prefix+'_arm_markers', anonymous=True)

    listener = tf.TransformListener();

    # create an interactive marker server on the topic namespace simple_marker
    server = InteractiveMarkerServer(prefix+'_arm_markers')

    rospy.sleep(1)
    listener.waitForTransform(prefix+'_arm_7_link', prefix+'_HandPalmLink', rospy.Time.now(), rospy.Duration(4.0))
    listener.waitForTransform('torso_base', prefix+'_arm_7_link', rospy.Time.now(), rospy.Duration(4.0))

    tool_msg = listener.lookupTransform(prefix+'_HandPalmLink', prefix+'_arm_7_link', rospy.Time(0))
    tool = pm.fromTf(tool_msg)
    real_gripper = listener.lookupTransform('torso_base', prefix+'_HandPalmLink', rospy.Time(0))

    int_position_marker = InteractiveMarker()
    int_position_marker.header.frame_id = 'torso_base'
    int_position_marker.name = 'arm_position_marker';
    int_position_marker.scale = 0.2
    int_position_marker.pose.position = Point(real_gripper[0][0], real_gripper[0][1], real_gripper[0][2])
    int_position_marker.pose.orientation = Quaternion(real_gripper[1][0], real_gripper[1][1], real_gripper[1][2], real_gripper[1][3])
    gripper = int_position_marker.pose
    p = pm.toMsg(pm.fromMsg(gripper) * tool)

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

    pub = rospy.Publisher('/' + prefix + '_hand/target', Marker)

    action_trajectory_client = actionlib.SimpleActionClient("/" + prefix + "_arm/cartesian_trajectory", CartesianTrajectoryAction)
    action_trajectory_client.wait_for_server()

    loop()


