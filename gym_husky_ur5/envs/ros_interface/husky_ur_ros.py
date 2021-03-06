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
import roslaunch
import rosparam
import rostopic

import rosmsg, rosservice
from std_msgs.msg import Float64
from geometry_msgs.msg import Twist, Pose
from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry

from husky_train.srv import EePose, EePoseRequest, EeRpy, EeRpyRequest, EeTraj, EeTrajRequest, JointTraj, JointTrajRequest, EeDelta, EeDeltaRequest

from robotiq_s_model_articulated_msgs.msg import SModelRobotOutput

class HuskyUR5ROS(object):

    def __init__(self,
                 robot_name='husky_ur5',
                 use_arm="left",
                 use_base=True,
                 use_camera=True,
                 use_gripper=True,
                 debug_print=True,
                 ):

        # Environment variable
        self.env = os.environ.copy()

        self.debug_print = debug_print

        self.use_arm = use_arm

        # run ROS core if not already running
        self.core = None # roscore
        self.gui = None # RQT
        master_uri = 11311
        self.init_core(port=master_uri)
        try:
            rospy.init_node('husky_ur5_ros_interface', anonymous=True)
            rospy.logwarn("ROS node husky_ur5_ros_interface has already been initialized")
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
        self.arm_joint_names_l = ["l_ur5_arm_shoulder_pan_joint", 
                                  "l_ur5_arm_shoulder_lift_joint", 
                                  "l_ur5_arm_elbow_joint", 
                                  "l_ur5_arm_wrist_1_joint", 
                                  "l_ur5_arm_wrist_2_joint", 
                                  "l_ur5_arm_wrist_3_joint"]
        if self.use_arm == "right":
            self.arm_joint_names = self.arm_joint_names_r
        if self.use_arm == "left":
            self.arm_joint_names = self.arm_joint_names_l

        self.arm_joint_home_r = [1.56996011734, -2.84751588503, 2.79816007614, -1.57001763979, 0.524644315243, -3.5587941305e-05]
        self.arm_joint_home_l = [-1.57001048723, -0.294129673635, -2.79563862482, -1.57005387941, -0.524643723165, 2.39684504777e-05]
        self.arm_joint_prepare_r = [1.93697440624, -1.95270091692, 1.76591014862, 0.136884212494, 1.84678673744, 0.0161745846272]
        self.arm_joint_prepare_l = [-1.87909251848, -1.03788692156, -1.53594905535, -3.57448703447, -1.69703323046, 1.44130086899]
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
        self.arm_ee_delta_pose_client = rospy.ServiceProxy('/ee_delta_srv', EeDelta)

        ####### Gripper Robotiq 3-finger gripper
        # Topics
        self.rostopic_gripper_left_cmd = 'l_gripper/SModelRobotOutput'
        self.rostopic_gripper_right_cmd = 'r_gripper/SModelRobotOutput'
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
                rospy.logwarn("Current " + str(self.rostopic_arm_joint_states) + "READY=>" + str(joints))
            except:
                rospy.logerr("Current " + str(self.rostopic_arm_joint_states) + " not ready yet, retrying...")
        
        if self.use_arm == 'left':
            self.gripper_reset('left')
            self.gripper_activate('left')
        if self.use_arm == 'right':
            self.gripper_reset('right')
            self.gripper_activate('right')

    @staticmethod
    def is_core_running():
        """
        Return True is the ROS core is running.
        """
        try:
            rostopic.get_topic_class('/roscore')
        except rostopic.ROSTopicIOException as e:
            return False
        return True

    def init_core(self, uri='localhost', port=11311):
        """Initialize the core if it is not running.
        Form [1], "the ROS core will start up:
        - a ROS master
        - a ROS parameter server
        - a rosout logging node"
        
        Args:
            uri (str): ROS master URI. The ROS_MASTER_URI will be set to `http://<uri>:<port>/`.
            port (int): Port to run the master on.

        References:
            - [1] roscore: http://wiki.ros.org/roscore 
        """
        # if the core is not already running, run it
        if not self.is_core_running():
            self.env["ROS_MASTER_URI"] = "http://" + uri + ":" + str(port)

            # this is for the rospy methods such as: wait_for_service(), init_node(), ...
            os.environ['ROS_MASTER_URI'] = self.env['ROS_MASTER_URI']

            # run ROS core if not already running
            # if "roscore" not in [p.name() for p in ptutil.process_iter()]:
            # subprocess.Popen("roscore", env=self.env)
            self.core = subprocess.Popen(["roscore", "-p", str(port)], env=self.env, preexec_fn=os.setid) # , shell=True)
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
        cmd.linear.x = xyt_position[0]
        cmd.angular.z = xyt_position[1]
        # if not rospy.is_shutdown():
        self.base_ctrl_pub.publish(cmd)

    def base_go_to_absolute(self, xyt_position):

        return NotImplementedError

    def base_velocity_cmd(self, cmd):
        command = Twist()
        command.linear.x = cmd[0]
        command.angular.z = cmd[1]

        self.base_ctrl_pub.publish(command)
        rospy.sleep(1.0)

        return True

    ### Arm UR5
    def arm_get_joint_name(self, arm):
        if arm == "left":
            return self.arm_joint_names
        if arm == "right":
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

        return self.arm_joint_angles

    def arm_get_joint_velocity(self, arm):

        return self.arm_joint_velocities

    def arm_get_joint_torque(self, arm):

        return NotImplementedError

    def arm_set_joint_positions(self, positions):
    
        joint_positions = JointTrajRequest()
        for i in range(len(positions)):
            joint_positions.point.positions.append(positions[i])

        result = self.arm_joint_traj_client(joint_positions)

        return result.success

    def arm_set_joint_velocities(self, velocityies):
    
        return NotImplementedError

    def arm_set_joint_torques(self, torques):

        return NotImplementedError

    def arm_set_ee_pose(self, action):
        ee_target = EeTrajRequest()
   
        ee_target.pose.position.x = action[0]
        ee_target.pose.position.y = action[1]
        ee_target.pose.position.z = action[2]
        ee_target.pose.orientation.w = action[3]
        ee_target.pose.orientation.x = action[4]
        ee_target.pose.orientation.y = action[5]
        ee_target.pose.orientation.z = action[6]

        result = self.arm_ee_traj_client(ee_target)

        return True

    def arm_set_ee_pose_relative(self, action):
        ee_delta_target = EeDeltaRequest()
        ee_delta_target.pose.position.x = action[0]
        ee_delta_target.pose.position.y = action[1]
        ee_delta_target.pose.position.z = action[2]
        return self.arm_ee_delta_pose_client(ee_delta_target)

    def arm_move_ee_xyz(self, displacement, eef_step=0.005):
        """
        Keep the current orientation fixed, move the end
        effector in {xyz} directions
        """

    def arm_go_home(self, arm):
        """
        Arm to the home position
        """
        if arm == "left":
            self.arm_set_joint_positions(self.arm_joint_home_l)
        if arm == "right":
            self.arm_set_joint_positions(self.arm_joint_home_r)
    
    def arm_move_to_neutral(self, arm):

        return NotImplementedError

    def arm_go_prepare(self, arm):
        if arm == "left":
            self.arm_set_joint_positions(self.arm_joint_prepare_l)
        if arm == "right":
            self.arm_set_joint_positions(self.arm_joint_prepare_r)

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
        # print("callback")

    ### Camera BB8 Stereo
    def camera_get_rgb(self):

        return NotImplementedError

    ### Gripper Robotiq 3finger
    def gripper_gen_cmd(self, char, command):
        """Update the command according to the character entered by the user."""    
        # command = SModelRobotOutput()
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

        # print("generated command: ", command)
        return command

    def gripper_activate(self, gripper):
        command = SModelRobotOutput()
        if gripper == 'left':
            command = self.gripper_gen_cmd('a', command)
            # if not rospy.is_shutdown():
            self.gripper_left_pub_cmd.publish(command)
            # print(command)
            rospy.sleep(1.0)
            rospy.logwarn("Left Gripper Activated")

        if gripper == 'right':
            command = self.gripper_gen_cmd('a', command)
            # if not rospy.is_shutdown():
            self.gripper_right_pub_cmd.publish(command)
            rospy.sleep(1.0)      
            rospy.logwarn("Right Gripper Activated")  

    def gripper_reset(self, gripper):
        command = SModelRobotOutput()
        if gripper == 'left':
            command = self.gripper_gen_cmd('r', command)
            # if not rospy.is_shutdown():
            self.gripper_left_pub_cmd.publish(command)
            # print(command)
            rospy.sleep(1.0)
            rospy.logwarn("Left Gripper Reset")

        if gripper == 'right':
            command = self.gripper_gen_cmd('r', command)
            # if not rospy.is_shutdown():
            self.gripper_right_pub_cmd.publish(command)
            rospy.sleep(1.0)      
            rospy.logwarn("Right Gripper Reset")  

    def gripper_open(self, gripper):
        command = SModelRobotOutput()
        command = self.gripper_gen_cmd('a', command)
        if gripper == 'left':
            command = self.gripper_gen_cmd('o', command)
            # if not rospy.is_shutdown():
            self.gripper_left_pub_cmd.publish(command)
            rospy.sleep(2.0)
            rospy.logwarn("Left Gripper Opened")

        if gripper == 'right':
            command = self.gripper_gen_cmd('o', command)
            # if not rospy.shutdown():
            self.gripper_right_pub_cmd.publish(command)
            rospy.sleep(2.0)  
            rospy.logwarn("Right Gripper Opened")

    def gripper_close(self, gripper):
        command = SModelRobotOutput()
        command = self.gripper_gen_cmd('a', command)
        if gripper == 'left':
            command = self.gripper_gen_cmd('c', command)
            # if not rospy.is_shutdown():
            self.gripper_left_pub_cmd.publish(command)
            rospy.sleep(2.0)
            rospy.logwarn("Left Gripper Closed")

        if gripper == 'right':
            command = self.gripper_gen_cmd('c', command)
            # if not rospy.shutdown():
            self.gripper_right_pub_cmd.publish(command)
            rospy.sleep(2.0) 
            rospy.logwarn("Right Gripper Closed")

