#!/usr/bin/env python

"""@package docstring
Command-line interface for sending simple commands to a ROS node controlling a S-Model gripper.

This serves as an example for publishing messages on the 'SModelRobotOutput' topic using the 'SModel_robot_output' msg type for sending commands to a S-Model gripper. In this example, only the simple control mode is implemented. For using the advanced control mode, please refer to the Robotiq support website (support.robotiq.com).
"""

import roslib; 
# roslib.load_manifest('robotiq_s_model_control')
import rospy
from robotiq_s_model_articulated_msgs.msg import SModelRobotOutput 
from time import sleep


def genCommand(char, command):
    """Update the command according to the character entered by the user."""    
        
    if char == 'a':
        command = SModelRobotOutput()
        command.rACT = 1
        command.rGTO = 1
        command.rSPA = 255
        command.rFRA = 150

    if char == 'r':
        command = SModelRobotOutput()
        command.rACT = 0

    if char == 'c':
        command.rPRA = 255

    if char == 'o':
        command.rPRA = 0

    if char == 'b':
        command.rMOD = 0
        
    if char == 'p':
        command.rMOD = 1
        
    if char == 'w':
        command.rMOD = 2
        
    if char == 's':
        command.rMOD = 3

    #If the command entered is a int, assign this value to rPRA
    try: 
        command.rPRA = int(char)
        if command.rPRA > 255:
            command.rPRA = 255
        if command.rPRA < 0:
            command.rPRA = 0
    except ValueError:
        pass                    
        
    if char == 'f':
        command.rSPA += 25
        if command.rSPA > 255:
            command.rSPA = 255
            
    if char == 'l':
        command.rSPA -= 25
        if command.rSPA < 0:
            command.rSPA = 0

            
    if char == 'i':
        command.rFRA += 25
        if command.rFRA > 255:
            command.rFRA = 255
            
    if char == 'd':
        command.rFRA -= 25
        if command.rFRA < 0:
            command.rFRA = 0

    print("gen cmd: ", command)
    return command


def gripper_control(action):
    """Main loop which requests new commands and publish them on the SModelRobotOutput topic."""

    # rospy.init_node('SModelSimpleController')

    topic_name = 'left_hand/command'
    # topic_name = 'right_hand/command'
    pub = rospy.Publisher(topic_name, SModelRobotOutput)

    command = SModelRobotOutput()

    gripper_cmd = action[-1]
    if action[-1] == 1: # open
        gripper_cmd = 'o'
    elif action[1] == -1: # close
        gripper_cmd = 'c' 

    if not rospy.is_shutdown():

        command = genCommand(gripper_cmd, command)      
        # print(command)      
        
        pub.publish(command)

        rospy.sleep(0.1)
                        

if __name__ == '__main__':
    action = [0,1]
    rospy.init_node("test_gripper")
    gripper_control(action)
