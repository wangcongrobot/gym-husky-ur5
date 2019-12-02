import numpy as np

from gym_husky_ur5.envs import robot_ros_env, utils_gripper
from gym.envs.robotics import rotations, utils

class MobileDualUR5HuskyROSEnv(robot_ros_env.RobotROSEnv):
    """Superclass for all Dual_UR5_Husky environments.
    """

    def __init__(
        self, model_path, n_substeps, gripper_extra_height, block_gripper,
        has_object, target_in_the_air, target_offset, obj_range, target_range,
        distance_threshold, initial_qpos, reward_type, n_actions,
    ):
        """Initializes a new Dual_UR5_Husky environment.

        Args:
            model_path (string): path to the environments XML file
            n_substeps (int): number of substeps the simulation runs on every call to step
            gripper_extra_height (float): additional height above the table when positioning the gripper
            block_gripper (boolean): whether or not the gripper is blocked (i.e. not movable) or not
            has_object (boolean): whether or not the environment has an object
            target_in_the_air (boolean): whether or not the target should be in the air above the table or on the table surface
            target_offset (float or array with 3 elements): offset of the target
            obj_range (float): range of a uniform distribution for sampling initial object positions
            target_range (float): range of a uniform distribution for sampling a target
            distance_threshold (float): the threshold after which a goal is considered achieved
            initial_qpos (dict): a dictionary of joint names and values that define the initial configuration
            reward_type ('sparse' or 'dense'): the reward type, i.e. sparse or dense
            n_actions : the number of actuator
        """
        self.gripper_extra_height = gripper_extra_height
        self.block_gripper = block_gripper
        self.has_object = has_object
        self.target_in_the_air = target_in_the_air
        self.target_offset = target_offset
        self.obj_range = obj_range
        self.target_range = target_range
        self.distance_threshold = distance_threshold
        self.reward_type = reward_type
        self.n_actions = n_actions

        self.arm_dof = 3
        self.gripper_dof = 1
        # self.n_actions = self.arm_dof + self.gripper_dof
        
        self.gripper_actual_dof = 4
        self.gripper_close = False

        self.husky_init_pos = [0,0]

        self.init_pos = {
            'r_ur5_arm_shoulder_pan_joint': 0.0,
            'r_ur5_arm_shoulder_lift_joint': 0.0,
            'r_ur5_arm_elbow_joint': 0.0,
            'r_ur5_arm_wrist_1_joint': 0.0,
            'r_ur5_arm_wrist_2_joint': 0.0,
            'r_ur5_arm_wrist_3_joint': 0.0,
        }

        super(MobileDualUR5HuskyROSEnv, self).__init__(
            model_path=model_path, n_substeps=n_substeps, n_actions=self.n_actions,
            initial_qpos=initial_qpos)

    # GoalEnv methods
    # ----------------------------

    def compute_reward(self, action, goal):
        # return 0, 0
        return self.reward_pick(action, goal)
        # return self.reward_place(action, goal)
        # return self.staged_reward_simple(action, goal)
        # return self.staged_reward_standford(action, goal)

    def reward_pick(self, action, goal):
        """
        Simple reward function: reach and pick
        """
        # object_pos_1 = self.sim.data.get_site_xpos('object1')
        object_pos = self.sim.data.get_site_xpos('object0')
        print("self.sim.data.get_site_xpos('object0'): ", object_pos)
        grip_pos = self.sim.data.get_site_xpos('r_grip_site')
        print("self.sim.data.get_site_xpos('r_grip_pos'): ", grip_pos)

        grip_obj_pos = object_pos - grip_pos
        obj_target_pos = goal - object_pos

        reward_ctrl = 0
        reward_dist_object = 0
        reward_grasping = 0
        reward_dist_target = 0
        reward_target = 0
        reward = 0
        _is_success = False

        reward_ctrl = -np.square(action).sum()

        reward_dist_object = -np.linalg.norm(grip_obj_pos)
        print("distance between gripper and object: ", reward_dist_object)
        reward_dist_target = -np.linalg.norm(obj_target_pos)

        # reward_grasping = 0
        # if np.linalg.norm(grip_obj_pos) < 0.1:
        #     print("!!!!!!!!!!!!!!!!!!!!!!!!")
        #     reward_grasping += 1.0
        #     if np.linalg.norm(grip_obj_pos) < 0.05:
        #         reward_grasping += 10.0
        self.gripper_close = False
        if np.linalg.norm(grip_obj_pos) < 0.1:
            # reward_grasping += 0.5
            if np.linalg.norm(grip_obj_pos) < 0.05:
                # reward_grasping += 1.0
                self.gripper_close = True
                # stage 1: approaching and grasping/lifting
                if object_pos[2] > 0.37: # table hight + object hight + lift distance
                    # grasping success
                    reward_grasping += 10.0
                    if object_pos[2] > 0.5:
                        reward_grasping += 100.0
                        _is_success = True
                        # if object_pos[2] > 0.5:
                            # reward_grasping += 10.0
        reward = 0.01 * reward_ctrl + reward_dist_object + reward_grasping
        # reward = 0.05 * reward_ctrl + reward_dist_object

        # stage 2: approaching and target
        # if reward_grasping > 0:
            # if np.linalg.norm(obj_target_pos) < 0.05:
                # reward_target = 20
            # reward = 0.05 * reward_ctrl + reward_dist_target + reward_target
            
        # reward = 0.05 * reward_ctrl + reward_dist_object + reward_grasping + 10 * reward_dist_target + reward_target
        print("object_pose: ", object_pos)
        print("reward_dist_object: ", reward_dist_object)
        print("reward_ctrl: ", 0.05 * reward_ctrl)
        print("reward_grasping: ", reward_grasping)
        # print("reward_dist_target: ", reward_dist_target)
        # print("reward_target: ", reward_target)
        print("total reward: ", reward)
        done = False
        if object_pos[2] < 0.1:
            # done = True
            reward -= 10
        info = {
            'is_success': _is_success,
        }
        return reward, done, info

    # RobotEnv methods
    # ----------------------------

    def _step_callback(self):
        if self.block_gripper:
            # self.sim.data.set_joint_qpos('robot0:l_gripper_finger_joint', 0.)
            # self.sim.data.set_joint_qpos('robot0:r_gripper_finger_joint', 0.)
            self.sim.forward()

    def _set_action(self, action):
        assert action.shape == (self.n_actions,) # 6 mobile base
        action = action.copy()  # ensure that we don't change the action outside of this scope
        print("_set_action:", action)
        pos_ctrl, base_ctrl, gripper_ctrl = action[:3], action[3:-1], action[-1]
        # pos_ctrl, gripper_ctrl = action[:3], action[3:]

        pos_ctrl *= 0.03  # limit maximum change in position
        base_ctrl *= 0.01

        rot_ctrl = [0, 0.707, 0.707, 0] # fixed rotation of the end effector, expressed as a quaternion

        if self.gripper_close:
            gripper_ctrl = -1.0
        else:
            gripper_ctrl = 1.0
        gripper_ctrl = self.gripper_format_action(gripper_ctrl)
        # assert gripper_ctrl.shape == (2,)
        assert gripper_ctrl.shape == (self.gripper_actual_dof,)
        if self.block_gripper:
            gripper_ctrl = np.zeros_like(gripper_ctrl)
        action = np.concatenate([pos_ctrl, rot_ctrl, base_ctrl, gripper_ctrl])
        # action = np.concatenate([pos_ctrl, rot_ctrl, gripper_ctrl])

        # Apply action to simulation.
        # utils.ctrl_set_action(self.sim, action) # base control + gripper control
        # utils.mocap_set_action(self.sim, action) # arm control in cartesion (x, y, z)
        self.set_trajectory_ee(action)
        utils_gripper.gripper_control(action)

    def _get_obs1(self):
        # positions
        grip_pos = self.get_ee_pose()
        grip_pos_array = np.array([grip_pos.pose.position.x, grip_pos.pose.position.y, grip_pos.pose.position.z])
        dt = self.sim.nsubsteps * self.sim.model.opt.timestep
        # grip_velp = self.sim.data.get_site_xvelp('r_grip_site') * dt
        grip_rpy = self.get_ee_rpy()
        grip_velp = np.array([grip_rpy.y, grip_rpy.y])
        robot_qpos, robot_qvel = self.robot_get_obs(self.joints)
        # robot_qpos, robot_qvel = utils.robot_get_obs(self.sim)
        if self.has_object:
            object_pos = self.sim.data.get_site_xpos('object0')
            # # rotations
            object_rot = rotations.mat2euler(self.sim.data.get_site_xmat('object0'))
            # # velocities
            object_velp = self.sim.data.get_site_xvelp('object0') * dt
            object_velr = self.sim.data.get_site_xvelr('object0') * dt
            # # gripper state
            # object_rel_pos = object_pos - grip_pos
            object_rel_pos = 0.1 * object_pos
            object_velp -= grip_velp

        else:
            object_pos = object_rot = object_velp = object_velr = object_rel_pos = np.zeros(0)
        # gripper_state = robot_qpos[-2:]
        gripper_state = robot_qpos[-13:-1]
        gripper_vel = robot_qvel[-2:] * dt  # change to a scalar if the gripper is made symmetric

        if not self.has_object:
            achieved_goal = grip_pos.copy()
        else:
            achieved_goal = np.squeeze(object_pos.copy())
        # obs = np.concatenate([
            # grip_pos, 
            # grip_pos_array,
            # grip_rpy,
            # grip_velp,
            # robot_qpos,
            # robot_qvel,
            # object_pos.ravel(), 
            # object_rel_pos.ravel(), 
            # gripper_state, 
            # object_rot.ravel(),
            # object_velp.ravel(), 
            # object_velr.ravel(), 
            # grip_velp, 
            # gripper_vel,
        # ])

        obs = np.concatenate([
            grip_pos, 
            object_pos.ravel(), 
            object_rel_pos.ravel(), 
            gripper_state, 
            object_rot.ravel(),
            object_velp.ravel(), 
            object_velr.ravel(), 
            grip_velp, 
            gripper_vel,
        ])

        return obs

        # return {
        #     'observation': obs.copy(),
        #     'achieved_goal': achieved_goal.copy(),
        #     'desired_goal': self.goal.copy(),
        # }
    def _get_obs(self):
        # positions
        grip_pos = self.sim.data.get_site_xpos('r_grip_site')
        dt = self.sim.nsubsteps * self.sim.model.opt.timestep
        grip_velp = self.sim.data.get_site_xvelp('r_grip_site') * dt
        robot_qpos, robot_qvel = utils.robot_get_obs(self.sim)
        if self.has_object:
            object_pos = self.sim.data.get_site_xpos('object0')
            # rotations
            object_rot = rotations.mat2euler(self.sim.data.get_site_xmat('object0'))
            # velocities
            object_velp = self.sim.data.get_site_xvelp('object0') * dt
            object_velr = self.sim.data.get_site_xvelr('object0') * dt
            # gripper state
            object_rel_pos = object_pos - grip_pos
            object_velp -= grip_velp
        else:
            object_pos = object_rot = object_velp = object_velr = object_rel_pos = np.zeros(0)
        # gripper_state = robot_qpos[-2:]
        gripper_state = robot_qpos[-13:-1]
        gripper_vel = robot_qvel[-2:] * dt  # change to a scalar if the gripper is made symmetric

        if not self.has_object:
            achieved_goal = grip_pos.copy()
        else:
            achieved_goal = np.squeeze(object_pos.copy())
        obs = np.concatenate([
            grip_pos, object_pos.ravel(), object_rel_pos.ravel(), gripper_state, object_rot.ravel(),
            object_velp.ravel(), object_velr.ravel(), grip_velp, gripper_vel,
        ])

        return obs
        # return {
        #     'observation': obs.copy(),
        #     'achieved_goal': achieved_goal.copy(),
        #     'desired_goal': self.goal.copy(),
        # }

    def _viewer_setup(self):
        body_id = self.sim.model.body_name2id('r_gripper_palm_link')
        lookat = self.sim.data.body_xpos[body_id]
        for idx, value in enumerate(lookat):
            self.viewer.cam.lookat[idx] = value
        self.viewer.cam.distance = 2.5
        self.viewer.cam.azimuth = 132.
        self.viewer.cam.elevation = -14.

    def _render_callback(self):
        # Visualize target.
        sites_offset = (self.sim.data.site_xpos - self.sim.model.site_pos).copy()
        site_id = self.sim.model.site_name2id('target0')
        self.sim.model.site_pos[site_id] = self.goal - sites_offset[0]
        self.sim.forward()

    def _reset_sim(self):
        self.sim.set_state(self.initial_state)

        # Randomize start position of object.
        if self.has_object:
            object_xpos = self.initial_gripper_xpos[:2]
            while np.linalg.norm(object_xpos - self.initial_gripper_xpos[:2]) < 0.1:
                object_xpos = self.initial_gripper_xpos[:2] + self.np_random.uniform(-self.obj_range, self.obj_range, size=2)

            object_xpos = np.array([0.7, -0.5]) # + self.np_random.uniform(-0.02, 0.07, size=2)
            object_xpos[0] += self.np_random.uniform(-0.07, 0.4)
            object_xpos[1] += self.np_random.uniform(-0.25, 0.2)
            object_qpos = self.sim.data.get_joint_qpos('object0:joint')
            # object_qpos1 = self.sim.data.get_joint_qpos('object1:joint')
            assert object_qpos.shape == (7,)
            print("object_xpos0: ", object_xpos)
            # print("object1 pos: ", object_qpos1)
            object_qpos[:2] = object_xpos
            object_qpos[2] = 0.5
            # object_qpos[0] += 0.3
            # object_qpos[1] -= 0.1
            print("set_joint_qpos object_qpos: ", object_qpos)
            self.sim.data.set_joint_qpos('object0:joint', object_qpos)
            # print("get_body_xquat: ", self.sim.data.get_body_xquat('r_gripper_palm_link'))

        # set random gripper position
        # for i in range(3):
        #     gripper_target[i] += self.np_random.uniform(-0.2, 0.2)
        # print("gripper target random: ", gripper_target)
 

        # set random husky initial position
        base_ctrl = [0.0, 0.0]
        base_ctrl[0] += self.np_random.uniform(-1.0, -0.6) # position
        base_ctrl[1] += self.np_random.uniform(-0.2, 0.2) # rotation
        gripper_control = self.np_random.uniform(-1.0, 1.0)
        gripper_control = self.gripper_format_action(gripper_control)

        gripper_target = np.array([0.5, -0.3, 0.6])
        gripper_target[0] += base_ctrl[0]
        gripper_rotation = np.array([0, 0.707, 0.707, 0]) #(0, 0, -90)
        # for i in range(3):
        gripper_target[0] += self.np_random.uniform(-0.0, 0.1) # x
        gripper_target[1] += self.np_random.uniform(-0.1, 0.1) # y
        gripper_target[2] += self.np_random.uniform(-0.1, 0.1) # z
        self.sim.data.set_mocap_pos('gripper_r:mocap', gripper_target)
        self.sim.data.set_mocap_quat('gripper_r:mocap', gripper_rotation)

        action = np.concatenate([gripper_target, gripper_rotation, base_ctrl, gripper_control])
        # Apply action to simulation.
        utils.ctrl_set_action(self.sim, action) # base control + gripper control
        # utils.mocap_set_action(self.sim, action) # arm control in cartesion (x, y, z)

        for _ in range(15):
            self.sim.step()

        self.sim.forward()
        return True

    def _sample_goal(self):
        if self.has_object:
            goal = self.initial_gripper_xpos[:3] + self.np_random.uniform(-self.target_range, self.target_range, size=3)
            goal += self.target_offset
            goal[2] = self.height_offset
            if self.target_in_the_air and self.np_random.uniform() < 0.5:
                goal[2] += self.np_random.uniform(0.2, 0.45)
        else:
            goal = self.initial_gripper_xpos[:3] + self.np_random.uniform(-0.15, 0.15, size=3)
        return goal.copy()

    def _is_success(self, achieved_goal, desired_goal):
        pass

    def _env_setup(self, initial_qpos):

        print("Init Pos:")
        print(initial_qpos)
        self.set_trajectory_joints(initial_qpos)

        # Move end effector into position.
        gripper_target = np.array([0.5, -0.3, 0.6])
        gripper_rotation = np.array([0, 0.707, 0.707, 0]) #(0, 0, -90)

        action = np.concatenate([gripper_target, gripper_rotation])
        self.set_trajectory_ee(action)

        # Extract information for sampling goals.
        self.initial_gripper_xpos = self.sim.data.get_site_xpos('r_grip_site').copy()
        if self.has_object:
            self.height_offset = self.sim.data.get_site_xpos('object0')[2]

        gripper_pos = self.get_ee_pose()
        gripper_pose_array = np.array([gripper_pos.pose.position.x, gripper_pos.pose.position.y, gripper_pos.pose.position.z])
        self.initial_gripper_xpos = gripper_pose_array.copy()
        self._get_obs()

    def render(self, mode='human', width=500, height=500):
        return super(DualUR5HuskyEnv, self).render(mode, width, height)

    ### Add function for dual_ur5_husky

    def gripper_format_action(self, action):
        """ Given (-1,1) abstract control as np-array return the (-1,1) control signals
        for underlying actuators as 1-d np array
        Args:
            action: 1 => open, -1 => closed
        """
        movement = np.array([1, 1, 1, 0])
        return -1 * movement * action

        gripper_cmd = action[-1]


    def robot_get_obs(self, data):
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