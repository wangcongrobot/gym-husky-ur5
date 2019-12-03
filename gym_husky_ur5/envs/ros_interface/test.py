import rospy

from husky_ur_ros import HuskyUR5ROS


husky_ur_ros = HuskyUR5ROS()

joint_names = husky_ur_ros.arm_get_joint_name('right')

print(joint_names)