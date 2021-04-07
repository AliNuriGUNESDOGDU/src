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







import argparse
import actionlib
import control_msgs.msg
from trajectory_msgs.msg import *
import matplotlib.pyplot as plt
import os
import subprocess



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

# # # We can get the joint values from the group and adjust some of the values:
# joint_goal = group.get_current_joint_values()
# joint_goal[0] = pi/2
# joint_goal[1] = -1.8652
# joint_goal[2] = 2.1735
# joint_goal[3] = -2.8441474
# joint_goal[4] = -1.0508
# joint_goal[5] = -2.2357081


# # # The go command can be called with joint values, poses, or without any
# # # parameters if you have already set the pose or joint target for the group
# group.go(joint_goal, wait=True)

# # # Calling ``stop()`` ensures that there is no residual movement
# group.stop()

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
wpose_next = copy.deepcopy(wpose)


# Breathe Type-4
case = 0.02
typestr = "quite_abdomen"
angle = 0.2

scale = 1.0
#'''


volume_data = [0.05, 0.4910,1.7458,3.296,4.9083,6.5336,8.1843,9.8487,11.4723,13.0,14.4342,15.7978,
        17.0696,18.1916,19.1012,19.7648,20.1988,20.4667,20.6032,20.5069,19.9832,18.8786,17.2090,
        15.1815,12.9707,10.6080,8.1561,5.7395,3.3896,1.0770,0.52,0.25,0.0]
kp = (1/20.6032)*case*scale
#print kp
volume_data = [(element) * kp for element in volume_data]
ind = 0;
d = 0

while d < 33:
    scale = volume_data[ind%33]
    ind += 1
    d += 1
    wpose_next.position.z = wpose.position.z + scale * z  # First move up (z)
    wpose_next.position.y = wpose.position.y + scale * y  # and sideways (y)
    wpose_next.position.x = wpose.position.x + scale * x  # Second move forward/backwards in (x)
    waypoints.append(copy.deepcopy(wpose_next))



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
time = 0.21
length_path = len(plan.joint_trajectory.points)

a0= plan.joint_trajectory.points[0].time_from_start
a1= rospy.Duration(0.1)
d= 0
for point in plan.joint_trajectory.points:
    d+=1
    d = d%length_path
    point.time_from_start = a1
    print type(point.positions) ,point.positions
    print point.positions[0]-plan.joint_trajectory.points[d].positions[0]
    w1 = -(point.positions[0]-plan.joint_trajectory.points[d].positions[0])/time
    w2 = -(point.positions[1]-plan.joint_trajectory.points[d].positions[1])/time
    w3 = -(point.positions[2]-plan.joint_trajectory.points[d].positions[2])/time
    w4 = -(point.positions[3]-plan.joint_trajectory.points[d].positions[3])/time
    w5 = -(point.positions[4]-plan.joint_trajectory.points[d].positions[4])/time
    w6 = -(point.positions[5]-plan.joint_trajectory.points[d].positions[5])/time
    point.velocities = (w1,w2,w3,w4,w5,w6)
    a1 += rospy.Duration(time)
    print point.velocities
    a0 = copy.deepcopy(point.time_from_start)
# print plan.joint_trajectory.points
# Calling `stop()` ensures that there is no residual movement
# group.execute(plan, wait=True)
group.stop()
# It is always good to clear your targets after planning with poses.
# Note: there is no equivalent function for clear_joint_value_targets()
group.clear_pose_targets()


if rospy.has_param('arm_controller'):
    namespace_ = '/arm_controller/follow_joint_trajectory'
else:
    namespace_ = '/follow_joint_trajectory'
# Create an action client
client = actionlib.SimpleActionClient(
    namespace_,  # namespace of the action topics
    control_msgs.msg.FollowJointTrajectoryAction # action type
)
print "Waiting for server..."        
client.wait_for_server()
print "Connected to server"
g = control_msgs.msg.FollowJointTrajectoryGoal()
g.trajectory = JointTrajectory()

JOINT_NAMES = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
            'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']
g.trajectory.joint_names = JOINT_NAMES
d = 2.0
ind = 0
while d < 50.0:
    q1 = plan.joint_trajectory.points[ind%length_path].positions[0]
    q2 = plan.joint_trajectory.points[ind%length_path].positions[1]
    q3 = plan.joint_trajectory.points[ind%length_path].positions[2]
    q4 = plan.joint_trajectory.points[ind%length_path].positions[3]
    q5 = plan.joint_trajectory.points[ind%length_path].positions[4]
    q6 = plan.joint_trajectory.points[ind%length_path].positions[5]
    #print [q1,q2,q3,q4,q5,q6]

    d += 0.1
    ind += 1

    g.trajectory.points.append(
        JointTrajectoryPoint(positions=[q1,q2,q3,q4,q5,q6], 
            velocities=[0,0,0,0,0,0], time_from_start=rospy.Duration(d)))

client.send_goal(g)