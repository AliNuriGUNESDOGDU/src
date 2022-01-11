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
import actionlib_msgs.msg
import control_msgs.msg
import moveit_commander
import moveit_msgs.srv
import rospy
import sensor_msgs.msg
import tf
import trajectory_msgs.msg

import gripper_joint_space_fst as pp1
import gripper_joint_space_scn as pp2


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

        self.amplitude = args.amplitude
        self.bpm = args.bpm
        self.gaze = args.gaze
        self.num_cycles = args.number
        self.source = args.source
        self.random_available = True

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
        
        self.gaze_point = np.array([-0.58, 0.57, -0.32])
        self.move_vector = (np.array([-3, 0, 1])
                            / np.linalg.norm(np.array([-3, 0, 1])))
        self.start_client()

    def choose_source(self):
        self.waypoints = []
        start_pose = self.group.get_current_pose().pose
        pose_next = copy.deepcopy(start_pose)
        old_rand_amp = 0
        old_rand_mo_ve = np.array([0.0, 0.0, 0.0])
        amplitude_orig = self.amplitude
        for i in range(self.num_cycles):
            #self.amplitude = amplitude_orig*2*self.num_cycles/(i+self.num_cycles)
            self.amplitude = amplitude_orig/self.num_cycles*(i+self.num_cycles)
            if self.random_available:
                self.amplitude -= old_rand_amp
                rand_amp = float(np.random.normal(loc=0,scale=0.003,size=1))
                self.amplitude += rand_amp
                old_rand_amp = rand_amp
                print (self.amplitude)

                self.move_vector -= old_rand_mo_ve
                rand_mo_ve = np.random.normal(loc=0,scale=0.07,size=3)
                self.move_vector += rand_mo_ve
                self.move_vector /= np.linalg.norm(self.move_vector)
                old_rand_mo_ve = rand_mo_ve
                print (self.move_vector)

            
            if self.source == 1:
                shape_data = [0.05, 0.4910, 1.7458, 3.296, 4.9083, 6.5336, 
                            8.1843, 9.8487, 11.4723, 13.0, 14.4342, 15.7978, 
                            17.0696, 18.1916, 19.1012, 19.7648, 20.1988, 
                            20.4667, 20.6032, 20.5069, 19.9832, 18.8786, 
                            17.2090, 15.1815, 12.9707, 10.6080, 8.1561, 
                            5.7395, 3.3896, 1.0770, 0.52, 0.25]
                max_data = max(shape_data)
                for data in shape_data:
                    pose_next.position.x = start_pose.position.x \
                        + data*self.amplitude*self.move_vector[0]/max_data
                    pose_next.position.y = start_pose.position.y \
                        + data*self.amplitude*self.move_vector[1]/max_data
                    pose_next.position.z = start_pose.position.z \
                        + data*self.amplitude*self.move_vector[2]/max_data
                    self.waypoints.append(copy.deepcopy(pose_next))
            elif self.source == 2:
                shape_data = [math.sin(element)
                            for element in 
                            (np.linspace(0, 119/60*math.pi, 60)).tolist()]
                for data in shape_data:
                    pose_next.position.x = start_pose.position.x \
                        + data*self.amplitude*self.move_vector[0]
                    pose_next.position.y = start_pose.position.y \
                        + data*self.amplitude*self.move_vector[1]
                    pose_next.position.z = start_pose.position.z \
                        + data*self.amplitude*self.move_vector[2]
                    self.waypoints.append(copy.deepcopy(pose_next))
            else:
                shape_data = [0.0, 0.005624999999999991, 0.014378906250000045, 
                0.024486547851562568, 0.03474537368774422, 0.0443933953943253, 
                0.0529963630275786, 0.060354717840215955, 0.06642930274659942, 
                0.07128390374926041, 0.07504226099316191, 0.0778570788273204, 
                0.07988866580429121, 0.08129106189085289, 0.08220379893995788, 
                0.08274774841852639, 0.08302380913885798, 0.0790451566321223, 
                0.07260624098380641, 0.06495160665441868, 0.05691987986583036, 
                0.04905405662967122, 0.041685214275588134, 0.03499542709050685, 
                0.02906453874530568, 0.02390450659228971, 0.019484261167040162, 
                0.015747394717907204, 0.012624483451303736, 0.010041439737111357, 
                0.007924965428885544, 0.006205920714800195, 0.004821221732756786, 
                0.0037147237823418333, 0.002837426374215357, 0.0021472441754255556, 
                0.0016085180924106934, 0.0011913883859058227, 0.0008711129000450457, 
                0.0006273850763798272, 0.00044368593276999935]
                max_data = max(shape_data)
                for data in shape_data:
                    pose_next.position.x = start_pose.position.x \
                        + data*self.amplitude*self.move_vector[0]/max_data
                    pose_next.position.y = start_pose.position.y \
                        + data*self.amplitude*self.move_vector[1]/max_data
                    pose_next.position.z = start_pose.position.z \
                        + data*self.amplitude*self.move_vector[2]/max_data
                    self.waypoints.append(copy.deepcopy(pose_next))
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
        wpose.position.x = 0.1
        wpose.position.y = 0.51
        wpose.position.z = 0.1
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
        self.move_vector = self.move_vector[0:3]
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
        JOINT_NAMES = [
            'shoulder_pan_joint',
            'shoulder_lift_joint',
            'elbow_joint',
            'wrist_1_joint',
            'wrist_2_joint',
            'wrist_3_joint']
        fk_srv = rospy.ServiceProxy('/compute_fk',
                                         moveit_msgs.srv.GetPositionFK)
        rospy.loginfo("Waiting for /compute_fk service...")
        fk_srv.wait_for_service()
        rospy.loginfo("Connected!")
        req = moveit_msgs.srv.GetPositionFKRequest()
        req.header.frame_id = 'base_link'
        req.fk_link_names = ['tool0']
        js = sensor_msgs.msg.JointState()
        js.name = JOINT_NAMES
        
        
        d = 0.0
        old_b = list(self.plan.joint_trajectory.points[-1].positions)
        old_b[-1] = -1.57
        old_old_b = list(self.plan.joint_trajectory.points[-2].positions)
        old_old_b[-1] = -1.57
        old_old_old_b = list(self.plan.joint_trajectory.points[-3].positions)
        old_old_old_b[-1] = -1.57
        inc = 60/self.bpm/len(self.plan.joint_trajectory.points)*self.num_cycles
        old_rand_inc = 0.0
        for i in range(1):
            for point in self.plan.joint_trajectory.points:
                if self.random_available:
                    inc -= old_rand_inc
                    rand_inc = float(np.random.normal(loc=0,scale=0.003,size=1))
                    inc += rand_inc
                    old_rand_inc = rand_inc
                    print(inc)
                b = list(point.positions)
                b[-1] = -1.57
                list_b = (np.array(b)
                +np.array(old_b)
                +np.array(old_old_b)
                +np.array(old_old_old_b))/4
                old_old_old_b = old_old_b
                old_old_b = old_b
                old_b = b
                list_b[-1] = 0.0
                js.position = list_b
                req.robot_state.joint_state = js
                resp = fk_srv.call(req)
                wpose = resp.pose_stamped[0].pose
                matrixx = tf.transformations.quaternion_matrix([
                    wpose.orientation.x,
                    wpose.orientation.y,
                    wpose.orientation.z,
                    wpose.orientation.w])
                z_base_in_wrist_2 = np.dot(([0, 0, 1, 1]), matrixx)
                theta = math.atan(z_base_in_wrist_2[1]/z_base_in_wrist_2[0])
                if 0 > z_base_in_wrist_2[1]*math.sin(theta)+z_base_in_wrist_2[0]*math.cos(theta):
                    theta -= math.pi                        
                if theta < -math.pi:
                    theta += math.pi*2
                #rospy.loginfo("theta")
                #rospy.loginfo(theta)
                list_b[-1] = theta
                #list_b[-1] = -1.57
                point.positions = tuple(list_b)
                d += inc
                self.goal_j.trajectory.points.append(
                    trajectory_msgs.msg.JointTrajectoryPoint(
                        positions=point.positions,
                        velocities=[0, 0, 0, 0, 0, 0],
                        time_from_start=rospy.Duration(d)))
        self.client.send_goal(self.goal_j)

    def breathe_now(self):
        
        if self.gaze == 1:
            self.no_gaze()
        elif self.gaze == 2:
            self.slide_on_gaze()
        else:
            #self.send_default()
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
        parser.add_argument("-n", "--number", type=int, default="1",
                            help="Value between 0.0 and 0.2")
        parser.add_argument("-s", "--source", type=int, default="1",
                            help="Value between 0.0 and 0.2")
        args = parser.parse_args()
        print args.amplitude, args.bpm, args.gaze, args.source


        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node('breathing', anonymous=True)

        br = Breathe(args)


        rate = 10
        sample = rospy.Rate(rate)
        #send2take()
        rospy.sleep(5)
        print("--------------------------")
        print("--------------------------")
        print("STARTING NOW")
        print("--------------------------")
        print("--------------------------")
        q1 = [2.15,0.99,-2.1,-0.98,-1.57,-1.57]
        q2 = [2.37,0.98,-2.04,-1.15,-1.57,-1.57]
        a1 = pp1.PickPlace()
        a2 = pp2.PickPlace()
        a1.go(q1,1.0)

        while ((a1.client.get_state()!=actionlib_msgs.msg.GoalStatus.SUCCEEDED)
            and not rospy.is_shutdown()):
            #print "not yet"
            sample.sleep()

        br.gaze_point = np.array([-0.51, 0.57, -0.32])
        br.breathe_now()
        while ((br.client.get_state()!=actionlib_msgs.msg.GoalStatus.SUCCEEDED)
            and not rospy.is_shutdown()):
            #print "not yet"
            sample.sleep()
        print "now or never"
        #rospy.sleep(12)
        rospy.loginfo("wait")
        rospy.sleep(1.0)
        rospy.loginfo("continue")
        a1.state_machine()
        while ((a1.client.get_state()!=actionlib_msgs.msg.GoalStatus.SUCCEEDED)
            and not rospy.is_shutdown()):
            #print "not yet"
            sample.sleep()
        rospy.sleep(0.30)
        a2.go(q2,1.0)
        while ((a2.client.get_state()!=actionlib_msgs.msg.GoalStatus.SUCCEEDED)
            and not rospy.is_shutdown()):
            #print "not yet"
            sample.sleep()
        br2 = Breathe(args)
        br2.gaze_point = np.array([-0.69, 0.53, -0.32])
        br2.breathe_now()

        while ((br2.client.get_state()!=actionlib_msgs.msg.GoalStatus.SUCCEEDED)
            and not rospy.is_shutdown()):
            #print "not yet"
            sample.sleep()
        rospy.loginfo("wait")
        rospy.sleep(1.0)
        rospy.loginfo("continue")
        a2.state_machine()


        # br.amplitude = 0.2
        # br.breathe_now()
        # rospy.sleep(8)
        # a.state_machine()
        # br.gaze = 1
        # br.breathe_now()
        # rospy.sleep(10)
        # a.state_machine()
        print "finished"
        #br.send_default()
        
        rospy.spin()
    except rospy.ROSInterruptException:
        print ("Program interrupted before completion")
