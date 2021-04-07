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

def cosinetheorem(a1,a2,a3):
    asdf = -(a1*a1-a2*a2-a3*a3)/2/a2/a3
    return math.acos(max(-1.0, min(1.0, asdf)))
    

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
    amplitude = 0.15
    phase_angle_x = 0.0
    phase_angle_y = 0.0
    bpm = 18   
    z_kp = math.sin(q2_init+q3_init+q4_init+math.pi)
    xy_kp = math.cos(q2_init+q3_init+q4_init+math.pi)
    x_kp = xy_kp*math.cos(math.pi/2+q5_init)
    y_kp = xy_kp*math.sin(math.pi/2+q5_init)
    print x_kp,y_kp,z_kp
    '''
    Do the transformation in Z wrt q1 later
    '''
    # Table is a surface at z = -0.4
    z_to_look = -0.3
    z_distance_init = z_init-z_to_look
    xy_distance_init = z_distance_init/math.tan(q2_init+q3_init+q4_init+math.pi)
    y_distance_init = y_kp*xy_distance_init
    x_distance_init = x_kp*xy_distance_init
    y_to_look = 0 + y_distance_init
    x_to_look = x_init + x_distance_init
    xz_distance_init = math.sqrt(
        x_distance_init*x_distance_init+z_distance_init*z_distance_init)
    xyz_distance_init = math.sqrt(
        x_distance_init*x_distance_init+
        y_distance_init*y_distance_init+
        z_distance_init*z_distance_init)

    print x_to_look,y_to_look,z_to_look




    numberofsamples = round(600/bpm)
    angle = 0.0
    save_string = "bpm_"+ str(bpm) + "_ampl_" +str(amplitude) + "_px_" \
        + str(phase_angle_x) + "_py_" + str(phase_angle_y) +"_ykp_" +str(y_kp)

    quite_cage = 0.00
    quite_abdomen = 0.009 
    deep = 0.033
        #'''
    # Breathe Type-4
    case = quite_abdomen
    typestr = "quite_abdomen"
    angle = 0.2

    scale = 4.0
    #'''

    
    volume_data = [0, 0.4910,1.7458,3.296,4.9083,6.5336,8.1843,9.8487,11.4723,13.0,14.4342,15.7978,
        17.0696,18.1916,19.1012,19.7648,20.1988,20.4667,20.6032,20.5069,19.9832,18.8786,17.2090,
        15.1815,12.9707,10.6080,8.1561,5.7395,3.3896,1.0770,0.52,0.25,0.0]
    kp = (1/20.6032)*case*scale
    #print kp
    volume_data = [(element) * kp for element in volume_data]
    ind = 0;


    while d < 50.0:
        amplitude = volume_data[ind%33]
        ind += 1
        x_change_mag = 0.3*amplitude
        #y_change = y_kp*amplitude*
        z_change_mag = 0.9*amplitude
        x_change = x_change_mag
        z_change = z_change_mag
        x_abs = x_init + x_change
        y_abs = 0
        z_abs = z_init - z_change

        xz_distance = math.sqrt(
            (x_abs-x_to_look)*(x_abs-x_to_look)+(z_abs-z_to_look)*(z_abs-z_to_look))
        xz_change = math.sqrt(x_change*x_change+z_change*z_change)

        xyz_distance = math.sqrt(
            (x_abs-x_to_look)*(x_abs-x_to_look)+
            y_distance_init*y_distance_init+
            (z_abs-z_to_look)*(z_abs-z_to_look))

        q_comp_xz = cosinetheorem(xz_change,xz_distance_init,xz_distance)
        q_comp_y = cosinetheorem(xz_change,xyz_distance_init,xyz_distance)
        print xz_change,xyz_distance_init,xyz_distance


        q_change = math.atan2(y_abs,x_abs)
        q1 = q1_init+0*math.copysign(1,math.sin(angle))*q_comp_y
        q2,q3 = inversekinematicmodel(x_abs,z_abs)
        q4 = q4_init+q3_init+q2_init-q2-q3+1*q_comp_xz
        q5 = q5_init+1*q_comp_y
        q6 = q6_init
        #print [q1,q2,q3,q4,q5,q6]

        angle += math.pi/numberofsamples
        d += (60.0/33/bpm)
    
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




