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
    send_default():
    method3():
    """

    def __init__(self, args):
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node('breathing', anonymous=True)

        self.amplitude = args.amplitude
        self.bpm = args.bpm
        self.gaze = args.gaze
        self.num_cycles = args.number
        self.source = args.source

        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()

        group_name = "manipulator"
        self.group = moveit_commander.MoveGroupCommander(group_name)
        self.group.allow_replanning(True)
        self.group.set_goal_position_tolerance(0.01)
        self.group.set_goal_orientation_tolerance(0.1)
        self.group.set_planning_time(1)

        self.client = None
        self.goal_j = None

        self.waypoints = []
        self.plan = None
        
        self.gaze_point = np.array([-3.6, 1.8, 0.3])
        self.move_vector = (np.array([1, 0, 0])
                            / np.linalg.norm(np.array([1, 1, 1])))

    def choose_source(self):
        self.waypoints = []
        start_pose = self.group.get_current_pose().pose
        pose_next = copy.deepcopy(start_pose)
        if self.source == 1:
            shape_data = [0.05, 0.4910, 1.7458, 3.296, 4.9083, 6.5336, 
                           8.1843, 9.8487, 11.4723, 13.0, 14.4342, 15.7978, 
                           17.0696, 18.1916, 19.1012, 19.7648, 20.1988, 
                           20.4667, 20.6032, 20.5069, 19.9832, 18.8786, 
                           17.2090, 15.1815, 12.9707, 10.6080, 8.1561, 
                           5.7395, 3.3896, 1.0770, 0.52, 0.25, 0.0]
            max_data = math.max(shape_data)
            for data in shape_data:
                pose_next.position.x = start_pose.position.x \
                    + data*2*self.amplitude*self.move_vector[0]/max_data
                pose_next.position.y = start_pose.position.y \
                    + data*2*self.amplitude*self.move_vector[1]/max_data
                pose_next.position.z = start_pose.position.z \
                    + data*2*self.amplitude*self.move_vector[2]/max_data
                self.waypoints.append(copy.deepcopy(pose_next))
        elif self.source == 2:
            shape_data = [math.sin(element)
                           for element in 
                           (np.linspace(0, 2*math.pi, 60)).tolist()]
            for data in shape_data:
                pose_next.position.x = start_pose.position.x \
                    + data*self.amplitude*self.move_vector[0]
                pose_next.position.y = start_pose.position.y \
                    + data*self.amplitude*self.move_vector[1]
                pose_next.position.z = start_pose.position.z \
                    + data*self.amplitude*self.move_vector[2]
                self.waypoints.append(copy.deepcopy(pose_next))
        else:
            pass
        

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

    def send_default(self):
        """ Method to initialize a looking direction for maintaining gaze

        """
        wpose = self.group.get_current_pose().pose
        wpose.position.x = -0.06
        wpose.position.y = -0.26
        wpose.position.z = 0.4
        gaze_vector_W = self.gaze_point - np.array(
            [wpose.position.x, wpose.position.y, wpose.position.z])
        gaze_vector_W = gaze_vector_W/np.linalg.norm(gaze_vector_W)
        axis = np.cross([1, 0, 0], gaze_vector_W)
        angle = math.acos(np.dot([1, 0, 0], gaze_vector_W))
        axis = axis/np.linalg.norm(axis)
        sin_ = math.sin(angle/2)
        wpose.orientation.x = axis[0]*sin_
        wpose.orientation.y = axis[1]*sin_
        wpose.orientation.z = axis[2]*sin_
        wpose.orientation.w = math.cos(angle/2)
        self.group.set_pose_target(wpose)
        plan = self.group.go(wait=True)
        #self.group.execute(plan, wait=True)
        self.group.stop()
        self.group.clear_pose_targets()

    def no_gaze(self):
        self.choose_source()

    def slide_on_gaze(self):
        wpose = self.group.get_current_pose().pose
        matrixx = tf.transformations.quaternion_matrix([
            wpose.orientation.x,
            wpose.orientation.y,
            wpose.orientation.z,
            wpose.orientation.w])
        inv_matrixx = tf.transformations.inverse_matrix(matrixx)
        self.move_vector = np.dot(([1, 0, 0, 1]), inv_matrixx)
        self.move_vector /= np.linalg.norm(self.move_vector)
        self.choose_source()

    def maintain_gaze(self):
        self.choose_source()
        for wpose in self.waypoints:
            gaze_vector_W = self.gaze_point - np.array(
                [wpose.position.x, wpose.position.y, wpose.position.z])
            gaze_vector_W = gaze_vector_W/np.linalg.norm(gaze_vector_W)
            angle = math.acos(np.dot([1, 0, 0], gaze_vector_W))
            axis = np.cross([1, 0, 0], gaze_vector_W)
            axis = axis/np.linalg.norm(axis)
            sin_ = math.sin(angle/2)
            wpose.orientation.x = axis[0]*sin_
            wpose.orientation.y = axis[1]*sin_
            wpose.orientation.z = axis[2]*sin_
            wpose.orientation.w = math.cos(angle/2)

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
                d += inc
                self.goal_j.trajectory.points.append(
                    trajectory_msgs.msg.JointTrajectoryPoint(
                        positions=point.positions,
                        velocities=[0, 0, 0, 0, 0, 0],
                        time_from_start=rospy.Duration(d)))
        self.client.send_goal(self.goal_j)

    def breathe_now(self):
        self.start_client()
        if self.gaze == 1:
            self.no_gaze()
        elif self.gaze == 2:
            self.slide_on_gaze()
        else:
            self.send_default()
            self.maintain_gaze()
        self.plan_path()
        self.execute_path()

def send2take():
    group_name = "manipulator"
    group = moveit_commander.MoveGroupCommander(group_name)
    group.allow_replanning(True)
    group.set_goal_position_tolerance(0.01)
    group.set_goal_orientation_tolerance(0.1)
    group.set_planning_time(1)
    wpose = group.get_current_pose().pose
    wpose.position.x = 0.24
    wpose.position.y = -0.36
    wpose.position.z = 0.44
    wpose.orientation.x = 0.62
    wpose.orientation.y = 0.39
    wpose.orientation.z = -0.52
    wpose.orientation.w = 0.42
    group.set_pose_target(wpose)
    plan = group.go(wait=True)
    group.stop()
    group.clear_pose_targets()
    
    wpose = group.get_current_pose().pose
    wpose.position.x = -0.05
    wpose.position.y = -0.26
    wpose.position.z = 0.41
    wpose.orientation.x = 0.21
    wpose.orientation.y = 0.08
    wpose.orientation.z = -0.79
    wpose.orientation.w = 0.58
    group.set_pose_target(wpose)
    
    plan = group.go(wait=True)
    group.stop()
    group.clear_pose_targets()

if __name__ == '__main__':
    try:
        parser = argparse.ArgumentParser()
        parser.add_argument("-a", "--amplitude", type=float, default="0.0",
                            help="Value between 0.0 and 0.2")
        parser.add_argument("-b", "--bpm", type=float, default="10.0",
                            help="\nValue between 0.0 and 0.2")
        parser.add_argument("-g", "--gaze", type=int, default="1",
                            help="Value between 0.0 and 0.2")
        parser.add_argument("-n", "--number", type=int, default="6",
                            help="Value between 0.0 and 0.2")
        parser.add_argument("-s", "--source", type=int, default="1",
                            help="Value between 0.0 and 0.2")
        args = parser.parse_args()
        print args.amplitude, args.bpm, args.gaze, args.source

        br = Breathe(args)
        send2take()
        br.breathe_now()
        
        rospy.spin()
    except rospy.ROSInterruptException:
        print ("Program interrupted before completion")
