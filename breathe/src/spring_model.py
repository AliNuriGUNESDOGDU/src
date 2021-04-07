#!/usr/bin/env python


import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list
import tf
import argparse
import actionlib
import control_msgs.msg
from trajectory_msgs.msg import *
import matplotlib.pyplot as plt
import numpy as np

def twospringmodel(theta_0):
    alpha = 1
    k_ac = 15
    r = 0.7
    k_pe = 40
    b = 15
    frequency = 30
    inertia = 2

    position = theta_0
    velocity = 0
    positions = [position]
    velocities = [velocity]
    dt = 0.01
    time = 0
    frequency = 12 #beat per minute
    num_breathe = 0
    print "before while"

    while time<60 :
        time = time + dt
        Torque = (-alpha*k_ac*(position-r*theta_0)-
            k_pe*(position-theta_0)-b*velocity)        
        velocity = velocity + dt*Torque/inertia
        position = position + velocity*dt
        positions.append(position)
        velocities.append(velocity)
        if -0.01 < velocity:
            alpha = 0
            pass
        if time % 2 <= 0.1:
            alpha = 1
        pass
    pass
    print "after while"
    return positions,velocities

def gripper_client(value,link):
    JOINT_NAMES = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
               'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']
    Q1 = [2.2,0,-1.57,0,0,0]
    Q2 = [1.5,0,-1.57,0,0,0]
    Q3 = [1.5,-0.2,-1.57,0,0,0]
    Q4 = [0.15,0,0,0,0,0]
    # Create an action client
    client = actionlib.SimpleActionClient(
        '/follow_joint_trajectory',  # namespace of the action topics
        control_msgs.msg.FollowJointTrajectoryAction # action type
    )
    print "Waiting for server..."        
    client.wait_for_server()
    print "Connected to server"
    g = control_msgs.msg.FollowJointTrajectoryGoal()
    g.trajectory = JointTrajectory()
    g.trajectory.joint_names = JOINT_NAMES
    positions1,velocities1= twospringmodel(1.0)
    arr_pos = np.array(positions1)
    arr_vel = np.array(velocities1)
    d = 2.0
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
    for pos, vel in zip(positions1, velocities1):
        d += 0.01
        g.trajectory.points.append(
            JointTrajectoryPoint(positions=[q1_init,q2_init-pos+1,q3_init-1+pos,q4_init,q5_init,q6_init], 
                velocities=[0.01,-vel,vel,0,0,0], time_from_start=rospy.Duration(d)))


    # for i in range(len(positions)-2):
    #     d += 0.1
    #     g.trajectory.points.append(
    #         JointTrajectoryPoint(positions=[0,-2+positions1(i)-1,1+1-positions1(i),-1.5,-1.5,-1.5], 
    #             velocities=[0.01,velocities1(i),-velocities1(i),0,0,0], time_from_start=rospy.Duration(d)))

    client.send_goal(g)


    # Prepare the data
    # x = np.linspace(0, 10, 100)
    
    # Plot the data
    # plt.plot(x, x, label='linear')
    # plt.show()

    

    
    #arr_vel = numpy.array(velocities)
    



    

    # g.trajectory.points = [
    #     JointTrajectoryPoint(positions=Q1, velocities=[0]*6, time_from_start=rospy.Duration(2.0)),
    #     JointTrajectoryPoint(positions=Q2, velocities=[0]*6, time_from_start=rospy.Duration(3.0)),
    #     JointTrajectoryPoint(positions=Q3, velocities=[0]*6, time_from_start=rospy.Duration(4.0))]
    # client.send_goal(g)
    # moveit_commander.roscpp_initialize(sys.argv)
    
    # robot = moveit_commander.RobotCommander()
    # scene = moveit_commander.PlanningSceneInterface()
    # #display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',moveit_msgs.msg.DisplayTrajectory)
    # group_name = "manipulator"
    # group = moveit_commander.MoveGroupCommander(group_name)


    # joint_goal = group.get_current_joint_values() 

    # print (len(joint_goal))
    # moveit_commander.roscpp_initialize(sys.argv)

    # # Create an action client
    # client = actionlib.SimpleActionClient(
    #   'simple_body_mover',  # namespace of the action topics
    #   control_msgs.msg.GripperCommandAction # action type
    # )
    
    # # Wait until the action server has been started and is listening for goals
    # client.wait_for_server()


    # # Create a goal to send (to the action server)
    # goal = control_msgs.msg.GripperCommandGoal()
    # goal.command.position = value   # From 0.0 to 0.7
    # goal.command.max_effort = -1  # Do not limit the effort
    # Q1 = [2.2,0,-1.57,0,0,0]
    # # client.send_goal(goal)

    # print(value)
    # print(link)
    # moveit_commander.roscpp_initialize(sys.argv)
    # robot = moveit_commander.RobotCommander()
    # group_name = "manipulator"
    # group = moveit_commander.MoveGroupCommander(group_name)
    # joint_goal = group.get_current_joint_values()
    # print 
    # (len(joint_goal))
    # joint_goal[link] = value
    # group.go(joint_goal, wait=True)

    # JOINT_NAMES = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
 #               'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']

    # g = FollowJointTrajectoryGoal()
    # g.trajectory = JointTrajectory()
    # g.trajectory.points
    # g.trajectory.joint_names = JOINT_NAMES
    # g.trajectory.points = [
 #        JointTrajectoryPoint(positions=Q1, velocities=[0]*6, time_from_start=rospy.Duration(4.0))]
    # group.stop()
    #scene = moveit_commander.PlanningSceneInterface()

    #client.wait_for_result()
    #return client.get_result()
    # pose_target = geometry_msgs.msg.Pose()
    # pose_target.orientation.w = 1.0
    # pose_target.position.x = 0.3
    # pose_target.position.y = 0
    # pose_target.position.z = 1.1

    # #group.set_pose_target(pose_target)   
    # joint_goal[5] = 1


    # #group.plan()
    # group.go(joint_goal, wait=True)
    # group.stop()


if __name__ == '__main__':
    try:
        # Get the angle from the command line
        parser = argparse.ArgumentParser()
        parser.add_argument("--value", type=float, default="0.7",
                            help="Value between 0.0 (open) and 0.7 (closed)")
        parser.add_argument("--link", type=int, default="1",
                            help="Value between 0.0 (open) and 0.7 (closed)")
        args = parser.parse_args()
        gripper_value = args.value
        link_value = args.link
        # Start the ROS node
        rospy.init_node('move_group_python_interface_tutorial',anonymous=True)
        # Set the value to the gripper
        positions,velocities= twospringmodel(1.0)
        arr_pos = np.array(positions)
        arr_vel = np.array(velocities)
        plt.plot(arr_pos)
        plt.show()
        plt.plot(arr_vel)
        plt.show()
        result = gripper_client(gripper_value,link_value)
        
    except rospy.ROSInterruptException:
        print ("Program interrupted before completion")




