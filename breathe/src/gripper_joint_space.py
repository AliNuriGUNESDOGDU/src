#!/usr/bin/env python
# 34567891123456789212345678931234567894123456789512345678961234567897123456789
"""Module to give UR5 ability to pick and place
This module uses PEP-8 as coding standard.
"""

import argparse
import copy
import math
import numpy as np
import os
import sys

import actionlib
import actionlib_msgs.msg
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
        
        # Gripper publisher and joint controller client
        self.gripper_pub = None
        self.client = None
        self.goal_j = None

        #rospy.init_node('pick_place', anonymous=True)
        # Start outputs
        self.start_client()
        self.start_publisher()

    def start_publisher(self):
        self.gripper_pub = rospy.Publisher(
            'gripper_control', String, queue_size=1)

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
        print("Waiting for server...")
        self.client.wait_for_server()
        print("Connected to server")
        self.goal_j = control_msgs.msg.FollowJointTrajectoryGoal()
        self.goal_j.trajectory = trajectory_msgs.msg.JointTrajectory()
        self.goal_j.trajectory.joint_names = JOINT_NAMES

    def state_machine(self):
        """State machine of pick place and wait function
        GO_WAIT->WAIT->GO_PICK->PICK->GO_SHOW->SHOW->GO_RELEASE->RELEASE->END
        """
        # Parameters of state machine
        # Joint states of the pick and place action
        q_wait = [-1.36,-3.91,1.91,-2.28,-4.15,3.9]
        q_go_pick = [-0.93,-3.59,0.98,-2.13,-4.7,5.29]
        q_pick = [-0.93,-3.59,0.98,-2.13,-4.7,5.29]
        q_show = [-1.38,-3.78,1.42,-0.83,-4.82,4.61]
        q_release = [-2.02,-3.98,1.42,-1.49,-4.88,4.62]
        # Times
        t_go_wait = 0.4
        t_wait = 5.6
        t_go_pick = 2.4
        t_gripper_open = 0.3
        t_pick = 2.4
        t_gripper_close = 0.2
        t_go_show = 2.4
        t_show = 5.2
        t_go_release = 2.4
        t_release = 14.4
        # Other Params
        rate = 20 # Hz
        t_passed = 0.0        
        # States
        GO_WAIT         = False
        WAIT            = False
        GO_PICK         = False
        PICK            = False
        GO_SHOW         = False
        SHOW            = False
        GO_RELEASE      = False
        RELEASE         = False
        END             = False
        IN_GO_WAIT      = True
        IN_WAIT         = False
        IN_GO_PICK      = False
        IN_PICK         = False
        IN_GO_SHOW      = False
        IN_SHOW         = False
        IN_GO_RELEASE   = False
        IN_RELEASE      = False
        IN_END          = False

        # State Machine
        while not rospy.is_shutdown():
            
            rospy.Rate(rate)
            if IN_GO_WAIT:
                self.go(q_wait,t_go_wait)
                IN_GO_WAIT = False
                GO_WAIT = True
                print("IN_GO_WAIT")
                pass
            elif GO_WAIT:
                if (self.client.get_state() 
                == actionlib_msgs.msg.GoalStatus.SUCCEEDED):
                    GO_WAIT = False
                    IN_WAIT = True
                    pass
                pass
            elif IN_WAIT:
                t_passed = 0.0
                IN_WAIT = False
                WAIT = True
                print("IN_WAIT")
                pass
            if WAIT:
                if t_passed >= t_wait:
                    WAIT = False
                    IN_GO_PICK = True
                    pass
                else:
                    t_passed += 1.0/rate
                    pass
                pass
            elif IN_GO_PICK:
                self.go(q_go_pick,t_go_pick)
                self.gripper_pub.publish("0.0")
                IN_GO_PICK = False
                GO_PICK = True
                print("IN_GO_PICK")
                pass
            elif GO_PICK:
                if (self.client.get_state() 
                == actionlib_msgs.msg.GoalStatus.SUCCEEDED):
                    GO_PICK = False
                    IN_PICK = True
                    pass
                pass
            elif IN_PICK:
                t_passed = 0.0
                self.gripper_pub.publish("0.25")
                IN_PICK = False
                PICK = True
                print("IN_PICK")
                pass
            elif PICK   :
                if t_passed >= t_pick:
                    PICK = False
                    IN_GO_SHOW = True
                    pass
                else:
                    t_passed += 1.0/rate
                    pass
                pass
            elif IN_GO_SHOW:
                self.go(q_show,t_go_show)
                IN_GO_SHOW = False
                GO_SHOW = True
                print("IN_GO_SHOW")
                pass
            elif GO_SHOW:
                if (self.client.get_state() 
                == actionlib_msgs.msg.GoalStatus.SUCCEEDED):
                    GO_SHOW = False
                    IN_SHOW = True
                    pass
                pass
            elif IN_SHOW:
                t_passed = 0.0
                IN_SHOW = False
                SHOW = True
                print("IN_SHOW")
                pass
            elif SHOW:
                if t_passed >= t_show:
                    SHOW = False
                    IN_GO_RELEASE = True
                    pass
                else:
                    t_passed += 1.0/rate
                    pass
                pass
            elif IN_GO_RELEASE:
                self.go(q_release,t_go_release)
                IN_GO_RELEASE = False
                GO_RELEASE = True
                print("IN_GO_RELEASE")
                pass
            elif GO_RELEASE:
                if (self.client.get_state() 
                == actionlib_msgs.msg.GoalStatus.SUCCEEDED):
                    GO_RELEASE = False
                    IN_RELEASE = True
                    pass
                pass
            elif IN_RELEASE:
                t_passed = 0.0
                self.gripper_pub.publish("0")
                IN_RELEASE = False
                RELEASE = True
                print("IN_RELEASE")
                pass
            elif RELEASE:
                if t_passed >= t_release:
                    RELEASE = False
                    IN_END = True
                    pass
                else:
                    t_passed += 1.0/rate
                    pass
                pass
            elif IN_END:
                #self.gripper_pub.publish("0.7")
                self.go(q_wait,t_go_pick)
                IN_END = False
                END = True
                print("IN_END")
                pass
            elif END    :
                if (self.client.get_state() 
                == actionlib_msgs.msg.GoalStatus.SUCCEEDED):
                    END = False
                    self.gripper_pub.publish("0.7")
                    print("Finished machine")
                    break
                    pass
                pass
            else        :   
                #rospy.spin()             
                pass

    def go(self,q_to_go,time_to_go):
        """Send robot to calculated joint positions
        """
        self.goal_j.trajectory.points = [
            trajectory_msgs.msg.JointTrajectoryPoint(
                positions = q_to_go, 
                velocities = [0,0,0,0,0,0],
                time_from_start=rospy.Duration(time_to_go))]
        self.client.send_goal(self.goal_j)
    
    def grip_now(self):
        self.start_publisher()
        print "started publisher"
        # rospy.sleep(1.0)
        # self.gripper_pub.publish("0")
        # print "published"
        # rospy.sleep(1.0)
        # self.gripper_pub.publish("0.2")
        # print "published"
        # rospy.sleep(1.0)
        # self.gripper_pub.publish("0.4")
        # #self.gripper_pub.publish("120")
        # print "published"
        # rospy.sleep(1.0)
        # self.gripper_pub.publish("0.6")
        # print "published"
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
        q1 = [-1.36,-3.91,1.91,-2.28,-4.15,3.9]
        #pp.go(q1,1.2)
        q1 = [-0.93,-3.59,0.98,-2.13,-4.7,5.29]
        # pp.grip_now()
        # pp.go(q1,1.2)
        pp.state_machine()
        #pp.grip_now()
        #send2take()
        
        rospy.spin()
    except rospy.ROSInterruptException:
        print ("Program interrupted before completion")
