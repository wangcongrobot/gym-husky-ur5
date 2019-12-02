import numpy as np

# ROS
import rospy
from std_msgs.msg import Float64
from sensor_msgs.msg import JointState
from husky_train.srv import EePose, EePoseRequest, EeRpy, EeRpyRequest, EeTraj, EeTrajRequest, JointTraj, JointTrajRequest


class ROSTester(object):
    def __init__(self):
        super().__init__()

        print("Initial ROSTester")

        # ROS
        self.JOINT_STATES_SUBSCRIBER = '/joint_states'
        self.joint_names = ["r_ur5_arm_shoulder_pan_joint", 
                          "r_ur5_arm_shoulder_lift_joint", 
                          "r_ur5_arm_elbow_joint", 
                          "r_ur5_arm_wrist_1_joint", 
                          "r_ur5_arm_wrist_2_joint", 
                          "r_ur5_arm_wrist_3_joint"]

        self.joint_states_sub = rospy.Subscriber(self.JOINT_STATES_SUBSCRIBER, JointState, self.joints_callback)
        self.joints = JointState()

        self.ee_traj_client = rospy.ServiceProxy('/ee_traj_srv', EeTraj)
        self.joint_traj_client = rospy.ServiceProxy('/joint_traj_srv', JointTraj)
        self.ee_pose_client = rospy.ServiceProxy('/ee_pose_srv', EePose)
        self.ee_rpy_client = rospy.ServiceProxy('/ee_rpy_srv', EeRpy)

        self._check_all_systems_ready()

    # ROS methods

    def _check_all_systems_ready(self):
        """
        Checks that all the sensors, publishers, and other simulation systems are operational.
        """
        self._check_all_sensors_ready()
        print("*********************")
        print("Check all system ready")
        return True

    def _check_all_sensors_ready(self):
        self._check_joint_states_ready()

        rospy.logdebug("ALL SENSORS READY")

    def _check_joint_states_ready(self):
        print("check joint states ready")
        self.joints = None
        while self.joints is None and not rospy.is_shutdown():
            try:
                self.joints = rospy.wait_for_message(self.JOINT_STATES_SUBSCRIBER, JointState, timeout=1.0)
                rospy.logdebug("Current " + str(self.JOINT_STATES_SUBSCRIBER) + "READY=>" + str(self.joints))

            except:
                rospy.logerr("Current " + str(self.JOINT_STATES_SUBSCRIBER) + " not ready yet, retrying...")

        print(self.joints)
        return self.joints

    def joints_callback(self, data):
        self.joints = data

    def get_joints(self):
        return self.joints
    
    def get_joint_names(self):
        return self.joints.name

    def set_trajectory_ee(self, action):
        ee_target = EeTrajRequest()
        
        ee_target.pose.position.x = action[0]
        ee_target.pose.position.y = action[1]
        ee_target.pose.position.z = action[2]
        ee_target.pose.orientation.w = action[3]
        ee_target.pose.orientation.x = action[4]
        ee_target.pose.orientation.y = action[5]
        ee_target.pose.orientation.z = action[6]
        result = self.ee_traj_client(ee_target)

        return True

    def set_trajectory_joints(self, initial_qpos):
        joint_point = JointTrajRequest()
        joint_point.point.positions = [None] * 6
        joint_point.point.positions[0] = initial_qpos["r_ur5_arm_shoulder_pan_joint"]
        joint_point.point.positions[1] = initial_qpos["r_ur5_arm_shoulder_lift_joint"]
        joint_point.point.positions[2] = initial_qpos["r_ur5_arm_elbow_joint"]
        joint_point.point.positions[3] = initial_qpos["r_ur5_arm_wrist_1_joint"]
        joint_point.point.positions[4] = initial_qpos["r_ur5_arm_wrist_2_joint"]
        joint_point.point.positions[5] = initial_qpos["r_ur5_arm_wrist_3_joint"]

        result = self.joint_traj_client(joint_point)

        return True

    def get_ee_pose(self):
        gripper_pose_req = EePoseRequest()
        gripper_pose = self.ee_pose_client(gripper_pose_req)

        return gripper_pose
    
    def get_ee_rpy(self):
        gripper_rpy_req = EeRpyRequest()
        gripper_rpy = self.ee_rpy_client(gripper_rpy_req)

        return gripper_rpy

if __name__ == "__main__":
    rospy.init_node("husky_ros_gym")
    tester = ROSTester()

    ee_pose = tester.get_ee_pose()
    print("**************")
    print("ee_pose: ")
    print(ee_pose)

    ee_rpy = tester.get_ee_rpy()
    print("***************")
    print("ee_rpy: ")
    print(ee_rpy)

    joints = tester.get_joints()
    print("****************")
    print("joints: ")
    print(joints)

    joint_names = tester.get_joint_names()
    print("******************")
    print("joint_names: ")
    print(joint_names)

    pos = [ee_pose.pose.position.x, ee_pose.pose.position.y, ee_pose.pose.position.z+0.5]
    rot = [ee_pose.pose.orientation.w, ee_pose.pose.orientation.x, ee_pose.pose.orientation.y, ee_pose.pose.orientation.z]
    action = np.concatenate([pos, rot])
    tester.set_trajectory_ee(action)
    print("********************")
    print("set trajectory ee")

    # qpos = joints
    # tester.set_trajectory_joints(qpos)
    # print("*********************")
    # print('set trajcetory joints')

    # rospy.shutdown()