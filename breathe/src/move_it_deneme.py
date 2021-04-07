#!/usr/bin/env python

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import numpy as np
from math import *
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import tf



moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node('move_group_python_interface_tutorial',
                anonymous=True)

robot = moveit_commander.RobotCommander()
scene = moveit_commander.PlanningSceneInterface()

group_names = robot.get_group_names()
print "============ Robot Groups:", robot.get_group_names()

group_name = "manipulator"
group = moveit_commander.MoveGroupCommander(group_name)

# # We can get the name of the reference frame for this robot:
# planning_frame = group.get_planning_frame()
# print "============ Reference frame: %s" % planning_frame

# # We can also print the name of the end-effector link for this group:
# eef_link = group.get_end_effector_link()
# print "============ End effector: %s" % eef_link

# # We can get a list of all the groups in the robot:
# group_names = robot.get_group_names()
# print "============ Robot Groups:", robot.get_group_names()

# # Sometimes for debugging it is useful to print the entire state of the
# # robot:
# print "============ Printing robot state"
# print robot.get_current_state()
# print ""

# # We can get the joint values from the group and adjust some of the values:
joint_goal = group.get_current_joint_values()
joint_goal[0] = pi/2
joint_goal[1] = -1.8652
joint_goal[2] = 2.1735
joint_goal[3] = -2.8441474
joint_goal[4] = 1.0508
joint_goal[5] = -2.2357081
print "joint goal"
print joint_goal


# # The go command can be called with joint values, poses, or without any
# # parameters if you have already set the pose or joint target for the group
group.go(joint_goal, wait=True)

# # Calling ``stop()`` ensures that there is no residual movement
group.stop()

waypoints = []
scale = 0.1
wpose = group.get_current_pose().pose
print wpose

# print (np.array([wpose.orientation.x,wpose.orientation.y,wpose.orientation.z,wpose.orientation.w]))
# print quaternion_from_euler(1.5708, 0.0, 1.5708)
euler2 = euler_from_quaternion(np.array([wpose.orientation.x,wpose.orientation.y,wpose.orientation.z,wpose.orientation.w]))
euler = list(euler2)
matrixx = tf.transformations.quaternion_matrix([wpose.orientation.x,wpose.orientation.y,wpose.orientation.z,wpose.orientation.w])

inv_matrixx = tf.transformations.inverse_matrix(matrixx)
print inv_matrixx
vectorx = np.dot(([1,0,0,1]),inv_matrixx)
print "euler"
print euler2
print "--"
roll = euler[0]
pitch = euler[1]
yaw = euler[2]
x = vectorx[0]
y = vectorx[1]
z = vectorx[2]
# wpose.orientation.w += +scale*0.0415
# wpose.orientation.x += +scale*0.0415
# wpose.orientation.y += +scale*0.0415
# wpose.orientation.z += +scale*0.0415
wpose.position.z += scale * z  # First move up (z)
wpose.position.y += scale * y  # and sideways (y)
wpose.position.x += scale * x  # Second move forward/backwards in (x)
waypoints.append(copy.deepcopy(wpose))

scale = -0.1

wpose.position.z += scale * z  # First move up (z)
wpose.position.y += scale * y  # and sideways (y)
wpose.position.x += scale * x  # Second move forward/backwards in (x)
waypoints.append(copy.deepcopy(wpose))

# wpose.position.y -= scale * 0.2  # Third move sideways (y)
# waypoints.append(copy.deepcopy(wpose))

# We want the Cartesian path to be interpolated at a resolution of 1 cm
# which is why we will specify 0.01 as the eef_step in Cartesian
# translation.  We will disable the jump threshold by setting it to 0.0 disabling:
(plan, fraction) = group.compute_cartesian_path(
                                   waypoints,   # waypoints to follow
                                   0.5,        # eef_step
                                   0.0)         # jump_threshold

# Note: We are just planning, not asking move_group to actually move the robot yet:
print x,y,z
group.execute(plan, wait=True)
#print plan.joint_trajectory.points[1].positions
# Calling `stop()` ensures that there is no residual movement
group.stop()
# It is always good to clear your targets after planning with poses.
# Note: there is no equivalent function for clear_joint_value_targets()
group.clear_pose_targets()
