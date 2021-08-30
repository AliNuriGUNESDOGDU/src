#!/usr/bin/env python
# 34567891123456789212345678931234567894123456789512345678961234567897123456789
"""Module to give UR5 ability to breathe
This module uses PEP-8 as coding standard.
"""

import copy
import math
import numpy as np
import os
import sys

import actionlib
import argparse
import control_msgs.msg
import moveit_commander
import rospy
import trajectory_msgs.msg

class BreathingSource(object):
    """This class implement different source of breathing as class methods
    method1():
    method2():
    method3():
    """
    def __init__(self):
        self.property1 = 1

class Breathe(object):
    """This class implements breathing main class
    implements an action client
    method2():
    method3():
    """
    def __init__(self,args):
        moveit_commander.roscpp_initialize(sys.argv) # ?
        rospy.init_node('breathing')

        self.amplitude = args.amplitude
        self.bpm = args.bpm
        self.gaze = args.gaze
        self.source = args.source       

        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()
        self.group_name = "manipulator"
        self.group = moveit_commander.MoveGroupCommander(self.group_name)

        self.waypoints = []
        self.start_pose = self.group.get_current_pose().pose
        self.pose_next = copy.deepcopy(self.start_pose)
        self.move_vector = (np.array([1, 1, 1])
            /np.linalg.norm(np.array([1, 1, 1])))
        self.gaze_point = np.array([0.2, 0.0, -0.2])
        self.plan = None

        
        


        ### Implement a source checker here
        volume_data = [0.05, 0.4910,1.7458,3.296,4.9083,6.5336,8.1843,9.8487,11.4723,13.0,14.4342,15.7978,
        17.0696,18.1916,19.1012,19.7648,20.1988,20.4667,20.6032,20.5069,19.9832,18.8786,17.2090,
        15.1815,12.9707,10.6080,8.1561,5.7395,3.3896,1.0770,0.52,0.25,0.0]

        ind = 0
        d = 0

        while d < 33:
            scale = volume_data[ind%33]
            ind += 1
            d += 1
            self.pose_next.position.z = self.start_pose.position.z + scale * self.move_vector[2]  # First move up (z)
            self.pose_next.position.y = self.start_pose.position.y + scale * self.move_vector[1]  # and sideways (y)
            self.pose_next.position.x = self.start_pose.position.x + scale * self.move_vector[0]  # Second move forward/backwards in (x)
            self.waypoints.append(copy.deepcopy(self.pose_next))

        self.maintain_gaze()
        self.plan_path()
        self.execute_path()



    def maintain_gaze(self):
        for wpose in self.waypoints:
            gaze_vector_W = self.gaze_point - np.array(
                [wpose.position.x, wpose.position.y, wpose.position.z])
            gaze_vector_W = gaze_vector_W/np.linalg.norm(gaze_vector_W) 
            print gaze_vector_W
            angle = np.dot(gaze_vector_W,[0,0,1])
            print angle
            axis = np.cross(gaze_vector_W,[0,0,1])
            axis = axis/np.linalg.norm(axis)
            print axis
            sin_ = math.sin(angle/2)
            wpose.orientation.x = axis[0]*sin_
            wpose.orientation.y = axis[1]*sin_
            wpose.orientation.z = axis[2]*sin_
            wpose.orientation.w = math.cos(angle/2)

    def plan_path(self):
        self.plan, fraction = self.group.compute_cartesian_path(
            self.waypoints,   # waypoints to follow
            0.5,        # eef_step
            0.0)         # jump_threshold

    def execute_path(self):
        JOINT_NAMES = ['shoulder_pan_joint', 'shoulder_lift_joint', 
            'elbow_joint','wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']
        # Check whether simulation or not
        if rospy.has_param('arm_controller'):
            namespace_ = '/arm_controller/follow_joint_trajectory'
        else:
            namespace_ = '/follow_joint_trajectory'
        # Create an action client
        client = actionlib.SimpleActionClient(
            namespace_, # namespace of the action topics
            control_msgs.msg.FollowJointTrajectoryAction # action type
        )
        print "Waiting for server..."
        client.wait_for_server()
        print "Connected to server"

        g = control_msgs.msg.FollowJointTrajectoryGoal()
        g.trajectory = trajectory_msgs.msg.JointTrajectory()
        g.trajectory.joint_names = JOINT_NAMES
        d= 0.0
        for point in self.plan.joint_trajectory.points:
            d += 0.1 # change later
            g.trajectory.points.append(
                trajectory_msgs.msg.JointTrajectoryPoint(positions=point.positions,
                velocities=[0,0,0,0,0,0], time_from_start=rospy.Duration(d)))
        client.send_goal(g)

        
        
            

if __name__ == '__main__':
    try:
        parser = argparse.ArgumentParser()
        parser.add_argument("-a","--amplitude", type=float, default="0.0",
            help="Value between 0.0 and 0.2")
        parser.add_argument("-b","--bpm", type=float, default="10.0",
            help="\nValue between 0.0 and 0.2")
        parser.add_argument("-g","--gaze", type=int, default="1",
            help="Value between 0.0 and 0.2")
        parser.add_argument("-s","--source", type=int, default="1",
            help="Value between 0.0 and 0.2")
        args = parser.parse_args()
        print args.amplitude, args.bpm, args.gaze, args.source
        
        Breathe(args)
        rospy.spin()
    except rospy.ROSInterruptException:
        print ("Program interrupted before completion")

        