#!/usr/bin/env python
# 34567891123456789212345678931234567894123456789512345678961234567897123456789
"""Module to give UR5 ability to breathe
This module uses PEP-8 as coding standard.
"""

import argparse
import copy
import math
import numpy as np
import os
import sys

import actionlib
import control_msgs.msg
import moveit_commander
import rospy
import tf
import trajectory_msgs.msg

from std_msgs.msg import String


class PickPlace(object):
    """This class implements breathing main class
    implements an action client
    send_default():
    method3():
    """

    def __init__(self):
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node('pick_place', anonymous=True)

        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()

        group_name = "manipulator"
        self.group = moveit_commander.MoveGroupCommander(group_name)
        self.group.allow_replanning(True)
        self.group.set_goal_position_tolerance(0.01)
        self.group.set_goal_orientation_tolerance(0.3)
        self.group.set_planning_time(1)

        self.gripper_pub = None
        self.client = None
        self.goal_j = None

        self.waypoints = []
        self.plan = None
        
        self.gaze_point = np.array([-0.35, 1.40, 0.05])
        self.move_vector = (np.array([0, 1, 1])
                            / np.linalg.norm(np.array([0, 1, 1])))

    def start_publisher(self):
        self.gripper_pub = rospy.Publisher('gripper_control', String, queue_size=1)

    def publish_gripper(self,gripper_val):
        self.gripper_pub.publish(gripper_val)        

    def start_client(self):
        """Method to start action client
        """
        JOINT_NAMES = [
            'shoulder_pan_joint',
            'shoulder_lift_joint',
            'elbow_joint',
            'wrist_1_joint',
            'wrist_2_joint',
            'wrist_3_joint']
        # Check whether simulation or not
        if rospy.has_param('arm_controller'):
            namespace_ = '/arm_controller/follow_joint_trajectory'
        else:
            namespace_ = '/follow_joint_trajectory'
        # Create an action client
        self.client = actionlib.SimpleActionClient(
            namespace_,  # namespace of the action topics
            control_msgs.msg.FollowJointTrajectoryAction  # action type
        )
        print "Waiting for server..."
        self.client.wait_for_server()
        print "Connected to server"
        self.goal_j = control_msgs.msg.FollowJointTrajectoryGoal()
        self.goal_j.trajectory = trajectory_msgs.msg.JointTrajectory()
        self.goal_j.trajectory.joint_names = JOINT_NAMES

    def plan_path(self):
        self.plan, fraction = self.group.compute_cartesian_path(
            self.waypoints,  # waypoints to follow
            0.5,            # eef_step
            0.0)            # jump_threshold

    def execute_path(self):
        d = 0.0
        inc = 60/self.bpm/len(self.plan.joint_trajectory.points)
        for i in range(self.num_cycles):
            for point in self.plan.joint_trajectory.points:
                b = list(point.positions)
                b[-1] = -1.4
                point.positions = tuple(b)
                d += inc
                self.goal_j.trajectory.points.append(
                    trajectory_msgs.msg.JointTrajectoryPoint(
                        positions=point.positions,
                        velocities=[0, 0, 0, 0, 0, 0],
                        time_from_start=rospy.Duration(d)))
        self.client.send_goal(self.goal_j)
    
    def grip_now(self):
        self.start_publisher()
        print "started publisher"
        rospy.sleep(1.0)
        self.gripper_pub.publish("0")
        print "published"
        rospy.sleep(1.0)
        self.gripper_pub.publish("0.2")
        print "published"
        rospy.sleep(1.0)
        self.gripper_pub.publish("0.4")
        #self.gripper_pub.publish("120")
        print "published"
        rospy.sleep(1.0)
        self.gripper_pub.publish("0.6")
        print "published"
        rospy.sleep(1.0)
        self.gripper_pub.publish("0.0")
        print "published"

def send2take():
    group_name = "manipulator"
    group = moveit_commander.MoveGroupCommander(group_name)
    group.allow_replanning(True)
    group.set_goal_position_tolerance(0.01)
    group.set_goal_orientation_tolerance(0.01)
    group.set_planning_time(2)
    wpose = group.get_current_pose().pose
    print wpose
    wpose.position.x = -0.262
    wpose.position.y = -0.165
    wpose.position.z = 0.532
    wpose.orientation.x = -0.5
    wpose.orientation.y = -0.5
    wpose.orientation.z = 0.5
    wpose.orientation.w = 0.5
    group.set_pose_target(wpose)
    plan = group.go(wait=True)
    group.stop()
    group.clear_pose_targets()
    
    
    # wpose = group.get_current_pose().pose
    # wpose.position.x = -0.33
    # wpose.position.y += -0.0
    # wpose.position.z = 0.11
    # wpose.orientation.x = 0.44
    # wpose.orientation.y = 0.84
    # wpose.orientation.z = 0.12
    # wpose.orientation.w = -0.267
    # group.set_pose_target(wpose)
    
    # plan = group.go(wait=True)
    # group.stop()
    # group.clear_pose_targets()

if __name__ == '__main__':
    try:
        pp = PickPlace()
        #pp.grip_now()
        send2take()
        
        rospy.spin()
    except rospy.ROSInterruptException:
        print ("Program interrupted before completion")
