#!/usr/bin/env python

import socket
import time
import sys
import rospy
from std_msgs.msg import String

HOST = "10.0.0.2" # The robot's computer address
PORT = 63352 # The gripper's port number

gripper_socket = None
def control_callback(data):
    global gripper_socket
    control_info = data.data
    if "." in control_info: #which means joint state information is sent [0 - 0.7]
        try:
            control_info = float(control_info) # check if the msg sent is float
            control_info = round(255 * (control_info/0.7)) # round to integer value

        except Exception as e:
            print("Not supported!")
            print("Use joint state [0 - 0.7] or gripper position [0 - 255]")
            return
        
    else:
        try:
            control_info = int(control_info) # check if the msg sent is integer
        except Exception as e:
            print("Not supported!")
            print("Use joint state [0 - 0.7] or gripper position [0 - 255]")
            return

    if control_info > 255: # check if value is exceeding boundaries
        control_info = 255
    elif control_info < 0:
        control_info = 0
    try:
        gripper_socket.send(b"SET POS {}".format(control_info)) # check if socket is connected
    except Exception as e:
        print("Error in connection to gripper!")
        return
def gripper_control(): 
    global gripper_socket
    pub = rospy.Publisher('gripper_control', String, queue_size=1) # create the publisher topic
    rospy.Subscriber("gripper_control", String, control_callback) # listen for control messages
    rospy.init_node("gripper_state_control",anonymous=True)

    gripper_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM) # tcp connection to the robot's computer and gripper's port
    while not rospy.is_shutdown(): # try until connecting to server
        try:
            gripper_socket.settimeout(10)
            gripper_socket.connect((HOST, PORT)) # connect to gripper socket
            print("Connected to " + HOST)
            while rospy.is_shutdown(): # wait for control messages
                pass
 
        except Exception as e:
            pass
        
    gripper_socket.close()

if __name__ == "__main__":
    gripper_control()