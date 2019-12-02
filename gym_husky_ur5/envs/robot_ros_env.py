import os
import copy
import numpy as np

import gym
from gym import error, spaces
from gym.utils import seeding

try:
    import mujoco_py
except ImportError as e:
    raise error.DependencyNotInstalled("{}. (HINT: you need to install mujoco_py, and also perform the setup instructions here: https://github.com/openai/mujoco-py/.)".format(e))

# ROS
import rospy
from std_msgs.msg import Float64
from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry
from husky_train.srv import EePose, EePoseRequest, EeRpy, EeRpyRequest, EeTraj, EeTrajRequest, JointTraj, JointTrajRequest

from robotiq_msg.msg import SModelRobotOutput
from geometry_msgs.msg import Pose

from collections import OrderedDict

def convert_observation_to_space(observation):
    if isinstance(observation, dict):
        space = spaces.Dict(OrderedDict([
            (key, convert_observation_to_space(value))
            for key, value in observation.items()
        ]))
    elif isinstance(observation, np.ndarray):
        low = np.full(observation.shape, -float('inf'))
        high = np.full(observation.shape, float('inf'))
        space = spaces.Box(low, high, dtype=observation.dtype)
    else:
        raise NotImplementedError(type(observation), observation)

    return space

DEFAULT_SIZE = 500

class RobotROSEnv(gym.Env):
    def __init__(self, model_path, initial_qpos, n_actions, n_substeps):
        if model_path.startswith('/'):
            fullpath = model_path
        else:
            fullpath = os.path.join(os.path.dirname(__file__), 'assets', model_path)
        if not os.path.exists(fullpath):
            raise IOError('File {} does not exist'.format(fullpath))

        model = mujoco_py.load_model_from_path(fullpath)
        self.sim = mujoco_py.MjSim(model, nsubsteps=n_substeps)
        self.viewer = None
        self._viewers = {}

        self.metadata = {
            'render.modes': ['human', 'rgb_array'],
            'video.frames_per_second': int(np.round(1.0 / self.dt))
        }

        # ROS
        self.JOINT_STATES_SUBSCRIBER = '/joint_states'
        self.joint_names = ["r_ur5_arm_shoulder_pan_joint", 
                          "r_ur5_arm_shoulder_lift_joint", 
                          "r_ur5_arm_elbow_joint", 
                          "r_ur5_arm_wrist_1_joint", 
                          "r_ur5_arm_wrist_2_joint", 
                          "r_ur5_arm_wrist_3_joint"]
        # self.joint_values = 

        self._check_all_systems_ready()
        
        self.joint_states_sub = rospy.Subscriber(self.JOINT_STATES_SUBSCRIBER, JointState, self.joints_callback)
        self.joints = JointState()

        # Service
        self.ee_traj_client = rospy.ServiceProxy('/ee_traj_srv', EeTraj)
        self.joint_traj_client = rospy.ServiceProxy('/joint_traj_srv', JointTraj)
        self.ee_pose_client = rospy.ServiceProxy('/ee_pose_srv', EePose)
        self.ee_rpy_client = rospy.ServiceProxy('/ee_rpy_srv', EeRpy)

        # Publisher
        GRIPPER_CMD = 'right_hand/command'
        self.gripper_cmd = rospy.Publisher(GRIPPER_CMD,SModelRobotOutput)


        # Subscribe
        OBJECT_TARGET = '/object_target'
        self.object_target = rospy.Subscriber(OBJECT_TARGET, Pose)

        # joint names
        JOINT1 = 'r_ur5_arm_shoulder_pan_joint'
        JOINT2 = 'r_ur5_arm_shoulder_lift_joint'
        JOINT3 = 'r_ur5_arm_elbow_joint'
        JOINT4 = 'r_ur5_arm_wrist_1_joint'
        JOINT5 = 'r_ur5_arm_wrist_2_joint'
        JOINT6 = 'r_ur5_arm_wrist_3_joint'



        self.controllers_list = []
        self.robot_name_space = ""

        self.seed()
        self._env_setup(initial_qpos=initial_qpos)
        self.initial_state = copy.deepcopy(self.sim.get_state())

        self.goal = self._sample_goal()
        obs = self._get_obs()
        self.action_space = spaces.Box(-1., 1., shape=(n_actions,), dtype='float32')
        self.observation_space = convert_observation_to_space(obs)

    @property
    def dt(self):
        return self.sim.model.opt.timestep * self.sim.nsubsteps

    # ROS methods

    def _check_all_systems_ready(self):
        """
        Checks that all the sensors, publishers, and other simulation systems are operational.
        """
        self._check_all_sensors_ready()
        return True

    def _check_all_sensors_ready(self):
        self._check_joint_states_ready()

        rospy.logdebug("ALL SENSORS READY")

    def _check_joint_states_ready(self):
        self.joints = None
        while self.joints is None and not rospy.is_shutdown():
            try:
                self.joints = rospy.wait_for_message(self.JOINT_STATES_SUBSCRIBER, JointState, timeout=1.0)
                rospy.logdebug("Current " + str(self.JOINT_STATES_SUBSCRIBER) + "READY=>" + str(self.joints))

            except:
                rospy.logerr("Current " + str(self.JOINT_STATES_SUBSCRIBER) + " not ready yet, retrying...")
        return self.joints

    def joints_callback(self, data):
        self.joints = data

    def get_joints(self):
        return self.joints
    
    def get_joint_names(self):
        return self.joints.name

    def set_trajectory_ee(self, action):
        ee_target = EeTrajRequest()
        ee_target.pose.orientation.w = 1.0
        ee_target.pose.position.x = action[0]
        ee_target.pose.position.y = action[1]
        ee_target.pose.position.z = action[2]
        result = self.ee_traj_client(ee_target)

        return True

    def set_trajectory_joints(self, initial_qpos):
        # joint_point = JointTrajRequest()
        # joint_point.point.positions = [None] * 6
        # joint_point.point.positions[0] = initial_qpos["r_ur5_arm_shoulder_pan_joint"]
        # joint_point.point.positions[1] = initial_qpos["r_ur5_arm_shoulder_lift_joint"]
        # joint_point.point.positions[2] = initial_qpos["r_ur5_arm_elbow_joint"]
        # joint_point.point.positions[3] = initial_qpos["r_ur5_arm_wrist_1_joint"]
        # joint_point.point.positions[4] = initial_qpos["r_ur5_arm_wrist_2_joint"]
        # joint_point.point.positions[5] = initial_qpos["r_ur5_arm_wrist_3_joint"]

        # result = self.joint_traj_client(joint_point)

        return True

    def get_ee_pose(self):
        gripper_pose_req = EePoseRequest()
        gripper_pose = self.ee_pose_client(gripper_pose_req)

        return gripper_pose
    
    def get_ee_rpy(self):
        gripper_rpy_req = EeRpyRequest()
        gripper_rpy = self.ee_rpy_client(gripper_rpy_req)

        return gripper_rpy


    # Env methods
    # ----------------------------

    def seed(self, seed=None):
        self.np_random, seed = seeding.np_random(seed)
        return [seed]

    def step(self, action):
        action = np.clip(action, self.action_space.low, self.action_space.high)
        self._set_action(action)
        self.sim.step()
        self._step_callback()
        obs = self._get_obs()

        done = False
        # info = {
        #     'is_success': self._is_success(obs['achieved_goal'], self.goal),
        # }
        reward, done, info = self.compute_reward(action, self.goal)
        # print("reward: ", reward)
        # reward = self.compute_reward(obs['achieved_goal'], self.goal, info)
        return obs, reward, done, info

    def reset(self):
        # Attempt to reset the simulator. Since we randomize initial conditions, it
        # is possible to get into a state with numerical issues (e.g. due to penetration or
        # Gimbel lock) or we may not achieve an initial condition (e.g. an object is within the hand).
        # In this case, we just keep randomizing until we eventually achieve a valid initial
        # configuration.
        # super(RobotEnv, self).reset()
        did_reset_sim = False
        while not did_reset_sim:
            did_reset_sim = self._reset_sim()
        self.goal = self._sample_goal().copy()
        obs = self._get_obs()
        return obs

    def close(self):
        if self.viewer is not None:
            # self.viewer.finish()
            self.viewer = None
            self._viewers = {}

    def render(self, mode='human', width=DEFAULT_SIZE, height=DEFAULT_SIZE):
        self._render_callback()
        if mode == 'rgb_array':
            self._get_viewer(mode).render(width, height)
            # window size used for old mujoco-py:
            data = self._get_viewer(mode).read_pixels(width, height, depth=False)
            # original image is upside-down, so flip it
            return data[::-1, :, :]
        elif mode == 'human':
            self._get_viewer(mode).render()

    def _get_viewer(self, mode):
        self.viewer = self._viewers.get(mode)
        if self.viewer is None:
            if mode == 'human':
                self.viewer = mujoco_py.MjViewer(self.sim)
            elif mode == 'rgb_array':
                self.viewer = mujoco_py.MjRenderContextOffscreen(self.sim, device_id=-1)
            self._viewer_setup()
            self._viewers[mode] = self.viewer
        return self.viewer

    # Extension methods
    # ----------------------------

    def _reset_sim(self):
        """Resets a simulation and indicates whether or not it was successful.
        If a reset was unsuccessful (e.g. if a randomized state caused an error in the
        simulation), this method should indicate such a failure by returning False.
        In such a case, this method will be called again to attempt a the reset again.
        """
        self.sim.set_state(self.initial_state)
        self.sim.forward()
        return True

    def _get_obs(self):
        """Returns the observation.
        """
        raise NotImplementedError()

    def _set_action(self, action):
        """Applies the given action to the simulation.
        """
        raise NotImplementedError()

    def _is_success(self, achieved_goal, desired_goal):
        """Indicates whether or not the achieved goal successfully achieved the desired goal.
        """
        raise NotImplementedError()

    def _sample_goal(self):
        """Samples a new goal and returns it.
        """
        raise NotImplementedError()

    def _env_setup(self, initial_qpos):
        """Initial configuration of the environment. Can be used to configure initial state
        and extract information from the simulation.
        """
        pass

    def _viewer_setup(self):
        """Initial configuration of the viewer. Can be used to set the camera position,
        for example.
        """
        pass

    def _render_callback(self):
        """A custom callback that is called before rendering. Can be used
        to implement custom visualizations.
        """
        pass

    def _step_callback(self):
        """A custom callback that is called after stepping the simulation. Can be used
        to enforce additional constraints on the simulation state.
        """
        pass
