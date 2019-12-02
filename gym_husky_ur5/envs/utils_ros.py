import numpy as np

from gym import error

import rospy
from std_msgs.msg import Float64
from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry
from husky_train.srv import EePose, EePoseRequest, EeRpy, EeRpyRequest, EeTraj, EeTrajRequest, JointTraj, JointTrajRequest

from robotiq_msg.msg import SModelRobotOutput
from geometry_msgs.msg import Pose

import roslib; 
# roslib.load_manifest('robotiq_s_model_control')
import rospy
from robotiq_msg.msg import SModelRobotOutput 
from time import sleep

from geometry_msgs.msg import Twist

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

    return command

def gripper_control(action):
    """Main loop which requests new commands and publish them on the SModelRobotOutput topic."""

    # rospy.init_node('SModelSimpleController')

    topic_name = 'left_hand/command'
    topic_name = 'right_hand/command'
    pub = rospy.Publisher(topic_name, SModelRobotOutput)

    command = SModelRobotOutput()

    gripper_cmd = action[-1]
    if action[-1] == 1: # open
        gripper_cmd = 'o'
    elif action[-1] == -1: # close
        gripper_cmd = 'c' 

    if not rospy.is_shutdown():

        command = genCommand(gripper_cmd, command)            
        
        pub.publish(command)

        rospy.sleep(0.1)
                        
def husky_control(action):

    command = Twist()
    command.linear.x = action[3]
    command.angular.z = action[4]
    '''
    linear:
    x: 0.0
    y: 0.0
    z: 0.0
    angular:
    x: 0.0
    y: 0.0
    z: 0.0
    '''

    topic_name = '/husky_velocity_controller/cmd_vel'
    pub = rospy.Publisher(topic_name, Twist)
    if not rospy.is_shutdown():
        pub.publish(command)

def robot_get_obs(data):
    if data.position is not None and data.name:
        names = [n for n in data.name]
        i = 0
        r = 0
        for name in names:
            r += 1

        return (
            np.array([data.position[i] for i in range(r)]),
            np.array([data.velocity[i] for i in range(r)]),
        )
    return np.zeros(0), np.zeros(0)

def set_trajectory_ee(action):

    ee_traj_client = rospy.ServiceProxy('/ee_traj_srv', EeTraj)

    ee_target = EeTrajRequest()
    ee_target.pose.orientation.w = 1.0
    ee_target.pose.position.x = action[0]
    ee_target.pose.position.y = action[1]
    ee_target.pose.position.z = action[2]
    result = ee_traj_client(ee_target)

    return True
def ctrl_set_action(sim, action):
    """For torque actuators it copies the action into mujoco ctrl field.
    For position actuators it sets the target relative to the current qpos.
    """
    if sim.model.nmocap > 0:
        _, action = np.split(action, (sim.model.nmocap * 7, ))
        # print("ctrl_set_action: ", action)
        # action=np.array([0., 0., 0., 0.])
        # action = -0.5 * np.array([0.5, 0., 0, 1, 1, 1, 0, 1, 1, 1, 1, 1, 1])
        # action = -1.0 * np.array([0, 1, 1, 1, 0, 1, 1, 1, 1, 1, 1])
        # action = np.array([1.0, 1.0, 1.0, -1.0, -1.0, -1.0])
        print("ctrl_set_action", action)
        print("ctrl_set_action.shape: ", action.shape)

    if sim.data.ctrl is not None:
        for i in range(action.shape[0]):
            if sim.model.actuator_biastype[i] == 0:
                sim.data.ctrl[i] = action[i]
            else:
                idx = sim.model.jnt_qposadr[sim.model.actuator_trnid[i, 0]]
                sim.data.ctrl[i] = sim.data.qpos[idx] + action[i]


def mocap_set_action(sim, action):
    """The action controls the robot using mocaps. Specifically, bodies
    on the robot (for example the gripper wrist) is controlled with
    mocap bodies. In this case the action is the desired difference
    in position and orientation (quaternion), in world coordinates,
    of the of the target body. The mocap is positioned relative to
    the target body according to the delta, and the MuJoCo equality
    constraint optimizer tries to center the welded body on the mocap.
    """
    if sim.model.nmocap > 0:
        action, _ = np.split(action, (sim.model.nmocap * 7, ))
        action = action.reshape(sim.model.nmocap, 7)
        print("mocap_set_action: ", action)

        pos_delta = action[:, :3]
        quat_delta = action[:, 3:]
        # TEST POSITION
        # pos_delta = [0, 0, 0]
        # quat_delta = [0, 0., 0., 0]

        reset_mocap2body_xpos(sim)
        sim.data.mocap_pos[:] = sim.data.mocap_pos + pos_delta
        sim.data.mocap_quat[:] = sim.data.mocap_quat + quat_delta
        print("real mocap position sim.data.mocap_pos: ", sim.data.mocap_pos)


def reset_mocap_welds(sim):
    """Resets the mocap welds that we use for actuation.
    """
    if sim.model.nmocap > 0 and sim.model.eq_data is not None:
        for i in range(sim.model.eq_data.shape[0]):
            if sim.model.eq_type[i] == mujoco_py.const.EQ_WELD:
                sim.model.eq_data[i, :] = np.array(
                    [0., 0., 0., 1., 0., 0., 0.])
    sim.forward()


def reset_mocap2body_xpos(sim):
    """Resets the position and orientation of the mocap bodies to the same
    values as the bodies they're welded to.
    """

    if (sim.model.eq_type is None or
        sim.model.eq_obj1id is None or
        sim.model.eq_obj2id is None):
        return
    for eq_type, obj1_id, obj2_id in zip(sim.model.eq_type,
                                         sim.model.eq_obj1id,
                                         sim.model.eq_obj2id):
        if eq_type != mujoco_py.const.EQ_WELD:
            continue

        mocap_id = sim.model.body_mocapid[obj1_id]
        if mocap_id != -1:
            # obj1 is the mocap, obj2 is the welded body
            body_idx = obj2_id
        else:
            # obj2 is the mocap, obj1 is the welded body
            mocap_id = sim.model.body_mocapid[obj2_id]
            body_idx = obj1_id

        assert (mocap_id != -1)
        sim.data.mocap_pos[mocap_id][:] = sim.data.body_xpos[body_idx]
        sim.data.mocap_quat[mocap_id][:] = sim.data.body_xquat[body_idx]
