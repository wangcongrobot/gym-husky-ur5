# This class is used to communicate with husky ur5 robot.
# First initial version only consider one ur 5 arm


from __future__ import print_function

import copy
import os
import sys
import threading
import time
from time import sleep

import numpy as np

# ROS 
import rospy
from std_msgs.msg import Float64
from geometry_msgs.msg import Twist, Pose
from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry
from husky_train.srv import EePose, EePoseRequest, EeRpy, EeRpyRequest, EeTraj, EeTrajRequest, JointTraj, JointTrajRequest

from robotiq_msg.msg import SModelRobotOutput


class HuskyUR5ROS(object):

    def __init__(self,
                 robot_name='husky_ur5',
                 use_arm=True,
                 use_base=True,
                 use_camera=True,
                 use_gripper=True,
                 ):

        try:
            rospy.init_node('husky_ur5_ros_interface', anonymous=True)
            rospy.loginfo("ROS node husky_ur5_ros_interface has already been initialized")
        except rospy.exceptions.ROSException:
            rospy.logwarn('ROS node husky_ur5_ros_interface has not been initialized')

        ####### Base Husky
        # Topics
        self.rostopic_base_cmd = '/husky_velocity_controller/cmd_vel' # 
        self.rostopic_base_odometry = '/odometry/filtered' # nav_msgs/Odometry
        self.rostopic_base_estop = '/estop' # std_msgs/Bool

        # Subscibers

        # Publishers
        self.base_ctrl_pub = rospy.Publisher(self.rostopic_base_cmd, Twist, queue_size=1)

        ####### Arm UR5

        # Params
        self.arm_joint_names_r = ["r_ur5_arm_shoulder_pan_joint", 
                                "r_ur5_arm_shoulder_lift_joint", 
                                "r_ur5_arm_elbow_joint", 
                                "r_ur5_arm_wrist_1_joint", 
                                "r_ur5_arm_wrist_2_joint", 
                                "r_ur5_arm_wrist_3_joint"]
        self.arm_joint_names = ["l_ur5_arm_shoulder_pan_joint", 
                                "l_ur5_arm_shoulder_lift_joint", 
                                "l_ur5_arm_elbow_joint", 
                                "l_ur5_arm_wrist_1_joint", 
                                "l_ur5_arm_wrist_2_joint", 
                                "l_ur5_arm_wrist_3_joint"]
        self.arm_dof = len(self.arm_joint_names)
        self.arm_base_frame = ''
        self.arm_ee_frame = ''

        self.arm_joint_angles = dict()
        self.arm_joint_velocities = dict()
        self.arm_joint_efforts = dict()

        self.arm_joint_state_lock = threading.RLock()

        # Topics
        self.rostopic_arm_joint_states = '/joint_states'
        self.rostopic_arm_set_joint = ''

        # Subscribers
        rospy.Subscriber(self.rostopic_arm_joint_states, JointState, self.arm_callback_joint_states)

        # Publishers
        # self.arm_joint_pub = rospy.Publisher(self.rostopic_arm_set_joint, JointState, queue_size=1)

        # Service
        self.arm_ee_traj_client = rospy.ServiceProxy('/ee_traj_srv', EeTraj)
        self.arm_joint_traj_client = rospy.ServiceProxy('/joint_traj_srv', JointTraj)
        self.arm_ee_pose_client = rospy.ServiceProxy('ee_pose_srv', EePose)
        self.arm_ee_rpy_client = rospy.ServiceProxy('/ee_rpy_srv', EeRpy)

        ####### Gripper Robotiq 3-finger gripper
        # Topics
        self.rostopic_gripper_left_cmd = '/left_hand/command'
        self.rostopic_gripper_right_cmd = '/right_hand/command'
        # Publisher
        self.gripper_left_pub_cmd = rospy.Publisher(self.rostopic_gripper_left_cmd, SModelRobotOutput, queue_size=1)
        self.gripper_right_pub_cmd = rospy.Publisher(self.rostopic_gripper_right_cmd, SModelRobotOutput, queue_size=1)

        ####### Camera BB8 Stereo camera

        ####### Camera RGBD

        ####### Target
        self.target_pose = rospy.Subscriber('/object_target', Pose)


        ####### Initialization
        self.check_all_systems_ready()

        rospy.sleep(2)
        print("Husky UR5 ROS Interface Initialize Successfully!")

    # Initialization
    def check_all_systems_ready(self):
        joints = None
        while joints is None and not rospy.is_shutdown():
            try:
                joints = rospy.wait_for_message(self.rostopic_arm_joint_states, JointState, timeout=1.0)
                rospy.logdebug("Current " + str(self.rostopic_arm_joint_states) + "READY=>" + str(joints))
            except:
                rospy.logerr("Current " + str(self.rostopic_arm_joint_states) + " not ready yet, retrying...")

        self.gripper_activate('left')
        self.gripper_activate('right')

    ### Base Husky
        
    def base_stop(self):
        """
        Stop the base
        """
        msg = Twist()
        msg.linear.x = 0
        msg.angular.z = 0
        if not rospy.is_shutdown():
            self.base_ctrl_pub.publish(msg)

    def base_go_to_relative(self, xyt_position):
        ''' Twist
        linear:
        x: 0.0
        y: 0.0
        z: 0.0
        angular:
        x: 0.0
        y: 0.0
        z: 0.0
        '''
        cmd = Twist()
        cmd.linear.x = xyt_position.x
        cmd.angular.z = xyt_position.z
        if not rospy.is_shutdown():
            self.base_ctrl_pub.publish(cmd)

    def base_go_to_absolute(self, xyt_position):

        return NotImplementedError

    ### Arm UR5
    def arm_get_joint_name(self, arm):
        return self.arm_joint_names

    def arm_get_ee_pose(self, arm):
        ee_pose_req = EePoseRequest()
        ee_pose = self.arm_ee_pose_client(ee_pose_req)

        return ee_pose

    def arm_get_ee_rpy(self, arm):
        ee_rpy_req = EeRpyRequest()
        ee_rpy = self.arm_ee_rpy_client(ee_rpy_req)

        return ee_rpy

    def arm_get_joint_angles(self, arm):

        return NotImplementedError

    def arm_get_joint_velocity(self, arm):

        return NotImplementedError

    def arm_get_joint_torque(self, arm):

        return NotImplementedError

    def arm_set_joint_positions(self, positions):
    
        joint_state = JointState()
        joint_state.position = tuple(positions)
        self.joint_pub.publish(joint_state)
        return NotImplementedError

    def arm_set_joint_velocities(self, velocityies):
    
        return NotImplementedError

    def arm_set_joint_torques(self, torques):

        return NotImplementedError

    def arm_set_ee_pose(self, pose):
        ee_target = EeTrajRequest()
        ee_target.pose.orientation.w = pose.orientation.w
        ee_target.pose.orientation.x = pose.orientation.x
        ee_target.pose.orientation.y = pose.orientation.y
        ee_target.pose.orientation.z = pose.orientation.z
        ee_target.pose.position.x = pose.position.x
        ee_target.pose.position.y = pose.position.y
        ee_target.pose.position.z = pose.position.z

        result = self.arm_ee_traj_client(ee_target)

        return True

    def arm_move_ee_xyz(self, displacement, eef_step=0.005):
        """
        Keep the current orientation fixed, move the end
        effector in {xyz} directions
        """

    def arm_go_home(self, arm):
        """
        Arm to the home position
        """

        return NotImplementedError
    
    def arm_move_to_neutral(self, arm):

        return NotImplementedError

    def arm_callback_joint_states(self, msg):
        """
        ROS subscriber callback for arm joint state (position, velocity)

        :param msg: Contains message published in topic
        :type msg: sensor_msgs/JointState
        """
        self.arm_joint_state_lock.acquire()
        for idx, name in enumerate(msg.name):
            if name in self.arm_joint_names:
                if idx < len(msg.position):
                    self.arm_joint_angles[name] = msg.position[idx]
                if idx < len(msg.velocity):
                    self.arm_joint_velocities[name] = msg.velocity[idx]
                if idx < len(msg.effort):
                    self.arm_joint_efforts[name] = msg.effort[idx]
        self.arm_joint_state_lock.release()


    ### Camera BB8 Stereo
    def camera_get_rgb(self):

        return NotImplementedError



    ### Gripper Robotiq 3finger
    def gripper_gen_cmd(self, char, command):
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

        return command

    def gripper_activate(self, gripper):
        if gripper == 'left':
            command = self.gripper_gen_cmd('a', 'left')
            if not rospy.is_shutdown():
                self.gripper_left_pub_cmd.publish(command)
                rospy.sleep(2.0)
                rospy.loginfo("Left Gripper Activated")

        if gripper == 'right':
            command = self.gripper_gen_cmd('a', 'right')
            if not rospy.is_shutdown():
                self.gripper_right_pub_cmd.publish(command)
                rospy.sleep(2.0)      
                rospy.loginfo("Right Gripper Activated")  
    
    def gripper_open(self, gripper):
        if gripper == 'left':
            command = self.gripper_gen_cmd('o', 'left')
            if not rospy.is_shutdown():
                self.gripper_left_pub_cmd.publish(command)
                rospy.sleep(2.0)
                rospy.loginfo("Left Gripper Opened")

        if gripper == 'right':
            command = self.gripper_gen_cmd('o', 'left')
            if not rosy.shutdown():
                self.gripper_right_pub_cmd.publish(command)
                rospy.sleep(2.0)  
                rospy.loginfo("Right Gripper Opened")

    def gripper_close(self, gripper):
        if gripper == 'left':
            command = self.gripper_gen_cmd('c', 'left')
            if not rospy.is_shutdown():
                self.gripper_left_pub_cmd.publish(command)
                rospy.sleep(2.0)
                rospy.loginfo("Left Gripper Closed")

        if gripper == 'right':
            command = self.gripper_gen_cmd('c', 'left')
            if not rosy.shutdown():
                self.gripper_right_pub_cmd.publish(command)
                rospy.sleep(2.0) 
                rospy.loginfo("Right Gripper Closed")

