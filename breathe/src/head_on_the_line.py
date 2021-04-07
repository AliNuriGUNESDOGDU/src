#!/usr/bin/env python


import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import math
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list
import tf
import argparse
import actionlib
import control_msgs.msg
from trajectory_msgs.msg import *
import matplotlib.pyplot as plt
import numpy as np
import os
import subprocess

def forwardkinemodel(q1,q2):
    a1 = 0.42500
    a2 = 0.39225
    z = -a1*math.sin(q1)-a2*math.sin(q1+q2)
    x = a1*math.cos(q1)+a2*math.cos(q1+q2)
    return x,z
def inversekinematicmodel(x,z):
    a1 = 0.42500
    a2 = 0.39225
    q2 = -math.acos((x*x+z*z-a1*a1-a2*a2)/2/a1/a2)
    q1 = math.atan2(x,z)-math.atan2((a1+a2*math.cos(-q2)),(a2*math.sin(-q2)))
    return q1,q2

def gripper_client(value,link):
    JOINT_NAMES = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
               'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']
    Q1 = [2.2,0,-1.57,0,0,0]
    Q2 = [1.5,0,-1.57,0,0,0]
    Q3 = [1.5,-0.2,-1.57,0,0,0]
    Q4 = [0.15,0,0,0,0,0]
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
    g.trajectory.joint_names = JOINT_NAMES
    d = 2.0

    video_record = True

    #'''
    # Breathe Type-10
    q1_init = 1.5708
    q2_init = 0.8652
    q3_init = -2.1735
    q4_init = -0.8441474
    q5_init = -1.0508
    q6_init = -2.2357081

    g.trajectory.points = [
        JointTrajectoryPoint(positions=[q1_init,q2_init,q3_init,q4_init,q5_init,q6_init], 
            velocities=[0.01,0,0,0,0,0], 
            time_from_start=rospy.Duration(1.0))]

    x_init,z_init = forwardkinemodel(q2_init,q3_init)
    amplitude = 0.02
    phase_angle_x = 0.0
    phase_angle_y = 0.0
    bpm = 30   
    z_kp = math.sin(q2_init+q3_init+q4_init+math.pi)
    xy_kp = math.cos(q2_init+q3_init+q4_init+math.pi)
    x_kp = xy_kp*math.cos(math.pi/2+q5_init)
    y_kp = xy_kp*math.sin(math.pi/2+q5_init)
    print x_kp,y_kp,z_kp
    #'''

    numberofsamples = round(600/bpm)
    angle = 0.0
    print forwardkinemodel(0.3,-2.5)
    save_string = "bpm_"+ str(bpm) + "_ampl_" +str(amplitude) + "_px_" \
        + str(phase_angle_x) + "_py_" + str(phase_angle_y) +"_ykp_" +str(y_kp)


    while d < 50.0:
        x_change = x_kp*amplitude*math.sin(angle)
        y_change = y_kp*amplitude*math.sin(angle)
        z_change = z_kp*amplitude*math.sin(angle)
        x_abs = x_init + x_change
        y_abs = 0 + y_change
        z_abs = z_init - z_change
        q_change = math.atan2(y_abs,x_abs)
        q1 = q1_init+q_change
        q2,q3 = inversekinematicmodel(x_abs,z_abs)
        q4 = q4_init+q3_init+q2_init-q2-q3
        q5 = q5_init - q_change
        q6 = q6_init
        print [q1,q2,q3,q4,q5,q6]

        angle += math.pi/numberofsamples
        d += 0.1
    
        g.trajectory.points.append(
            JointTrajectoryPoint(positions=[q1,q2,q3,q4,q5,q6], 
                velocities=[0,0,0,0,0,0], time_from_start=rospy.Duration(d)))

    client.send_goal(g)



if __name__ == '__main__':
    try:
        # Get the angle from the command line
        parser = argparse.ArgumentParser()
        parser.add_argument("--value", type=float, default="0.7",
                            help="Value between 0.0 (open) and 0.7 (closed)")
        parser.add_argument("--link", type=float, default="0.7",
                            help="Value between 0.0 (open) and 0.7 (closed)")
        args = parser.parse_args()
        gripper_value = args.value
        link_value = args.link
        # Start the ROS node
        rospy.init_node('move_group_python_interface_tutorial',anonymous=True)
        # Set the value to the gripper
        # positions,velocities= twospringmodel(1.0)
        # arr_pos = np.array(positions)
        # arr_vel = np.array(velocities)
        # plt.plot(arr_pos)
        # plt.show()
        # plt.plot(arr_vel)
        # plt.show()
        result = gripper_client(gripper_value,link_value)
         
        # print forwardkinemodel (0.115,0.749)
        # print inversekinematicmodel(0.1,0.4)
        # x1 = -0.2
        # y1 = 0.347
        # for i in range(20):
        #     x1 = 0.005 +x1
        #     y1 = 0.005 +y1
        #     print "real"
        #     print (x1,y1)
        #     print "from inverse"
        #     print inversekinematicmodel(x1,y1)
        
    except rospy.ROSInterruptException:
        print ("Program interrupted before completion")




