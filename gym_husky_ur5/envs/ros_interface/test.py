import rospy

from husky_ur_ros import HuskyUR5ROS

husky_ur_ros = HuskyUR5ROS()

def test_arm():
    
    joint_names = husky_ur_ros.arm_get_joint_name('left')

    print(joint_names)

    joint_values = husky_ur_ros.arm_get_joint_angles('left')
    print(joint_values)

    joint_target = [0.5, 
                    -1.4, 
                    1.49, 
                    -0.086, 
                    0.5, 
                    -2.0]
    husky_ur_ros.arm_set_joint_positions(joint_target)

    ee_pose = husky_ur_ros.arm_get_ee_pose('left')
    print(ee_pose)
    '''
    pose: 
  position: 
    x: 0.1858453113077738
    y: 0.19050837742130322
    z: 0.6712833076182397
  orientation: 
    x: -0.6269294640671255
    y: 0.21130153236837768
    z: -0.6742962500890209
    w: 0.32807876587668194
    '''
    # ee_pose.pose.position.z += 0.1

    action = [ee_pose.pose.position.x, 
              ee_pose.pose.position.y, 
              ee_pose.pose.position.z + 0.05, 
              ee_pose.pose.orientation.w,
              ee_pose.pose.orientation.x,
              ee_pose.pose.orientation.y,
              ee_pose.pose.orientation.z]
    action = [0.3169, -0.41, 0.93, 0.924, 0, 0, 0.383]
    husky_ur_ros.arm_set_ee_pose(action)

def test_gripper():
    husky_ur_ros.gripper_open('right')
    husky_ur_ros.gripper_open('left')
    rospy.sleep(1.0)
    husky_ur_ros.gripper_open('right')
    husky_ur_ros.gripper_close('left')


def test_base():
    base_cmd = [0.5, 0.5]
    husky_ur_ros.base_velocity_cmd(base_cmd)


def test_camera():
    pass



if __name__ == "__main__":
    # test_arm()
    # test_gripper()
    test_base()