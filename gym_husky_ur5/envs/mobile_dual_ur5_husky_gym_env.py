import numpy as np

from gym_husky_ur5.envs import robot_gym_env, utils_ros, utils_test
from gym.envs.robotics import rotations, utils

import rospy

from gym_husky_ur5.envs.ros_interface import husky_ur_ros

def goal_distance(goal_a, goal_b):
    assert goal_a.shape == goal_b.shape
    return np.linalg.norm(goal_a - goal_b, axis=-1)


class MobileDualUR5HuskyGymEnv(robot_gym_env.RobotGymEnv):
    """Superclass for all Dual_UR5_Husky environments.
    """

    def __init__(
        self, model_path, n_substeps, gripper_extra_height, block_gripper,
        has_object, target_in_the_air, target_offset, obj_range, target_range,
        distance_threshold, initial_qpos, reward_type, n_actions,
        use_real_robot
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

        self.left_arm_joint_names = [
            'l_ur5_arm_shoulder_pan_joint',
            'l_ur5_arm_shoulder_lift_joint',
            'l_ur5_arm_elbow_joint',
            'l_ur5_arm_wrist_1_joint',
            'l_ur5_arm_wrist_2_joint',
            'l_ur5_arm_wrist_3_joint',
        ]
        self.right_arm_joint_names = [
            'r_ur5_arm_shoulder_pan_joint',
            'r_ur5_arm_shoulder_lift_joint',
            'r_ur5_arm_elbow_joint',
            'r_ur5_arm_wrist_1_joint',
            'r_ur5_arm_wrist_2_joint',
            'r_ur5_arm_wrist_3_joint',
        ]
        self.init_pos = {
            'r_ur5_arm_shoulder_pan_joint': 0.0,
            'r_ur5_arm_shoulder_lift_joint': 0.0,
            'r_ur5_arm_elbow_joint': 0.0,
            'r_ur5_arm_wrist_1_joint': 0.0,
            'r_ur5_arm_wrist_2_joint': 0.0,
            'r_ur5_arm_wrist_3_joint': 0.0,
        }

        self.debug_print = True

        # rospy.init_node("gym")
        use_real_robot = True
        self._use_real_robot = use_real_robot
        if self._use_real_robot:
            self.husky_ur5_robot = husky_ur_ros.HuskyUR5ROS(debug_print=self.debug_print)
            self.use_arm = 'left'

        super(MobileDualUR5HuskyGymEnv, self).__init__(
            model_path=model_path, n_substeps=n_substeps, n_actions=self.n_actions,
            initial_qpos=initial_qpos)

    # GoalEnv methods
    # ----------------------------

    def compute_reward_old(self, achieved_goal, goal, info):
        # Compute distance between goal and the achieved goal.
        d = goal_distance(achieved_goal, goal)
        if self.reward_type == 'sparse':
            return -(d > self.distance_threshold).astype(np.float32)
        else:
            return -d

    def compute_reward1(self, action, goal):
        # control_mult = 0.1
        # r_control = (1 - np.tanh(np.square(action).sum())) * control_mult
        r_control = -0.01 * np.square(action).sum()
        print("r_control: ", r_control)

        staged_reward = self.staged_reward(action, goal)
        # print("staged_reward: ", staged_reward)
        reward = r_control + staged_reward
        done = False
        object_pos = self.sim.data.get_site_xpos('object0')
        # if object_pos[2] < 0.05:
            # done = True
        print("total reward: ", reward)
        return reward, done

    def compute_reward(self, action, goal):
        # return 0, 0
        return self.reward_pick(action, goal)
        # return self.reward_place(action, goal)
        # return self.staged_reward_simple(action, goal)
        # return self.staged_reward_standford(action, goal)

    def staged_reward_simple(self, action, goal):
        """
        Returns staged rewards based on current physical states.
        Stages consist of reaching, grasping, lifting, and hovering.
        """
        object_pos = self.sim.data.get_site_xpos('object0')
        grip_pos = self.sim.data.get_site_xpos('r_grip_site')
        grip_obj_pos = object_pos - grip_pos
        obj_target_pos = goal - object_pos
        
        reach_mult = 0.1
        grasp_mult = 0.35
        lift_mult = 0.5
        hover_mult = 0.7
        target_mult = 0.9
        reward = 0.

        ### reaching reward governed by distance to closest object ###
        r_reach = 0
        # get reaching reward via minimum distance to a target object
        dist = np.linalg.norm(grip_obj_pos)
        # r_reach = (1 - np.tanh(1.0 * dist)) * reach_mult
        r_reach = - dist * reach_mult

        ### grasping reward for touching any objects of interest ###
        r_grasp = 0.
        # touch_left_finger = False
        # touch_right_finger = False
        # touch_object = False
        # if np.linalg.norm(grip_obj_pos) < 0.05:
            # touch_object = True
        
        if object_pos[2] > (0.4 + 0.025 + 0.05):
            r_grasp = grasp_mult

        ### lifting reward for picking up an object ###
        r_lift = 0.
        # if r_grasp > 0.:
            # object_pos[2] > (0.4 + 0.025 + 0.05)
            # r_lift = lift_mult
            # z_dist = 0.5 - object_pos[2]
            # r_lift = grasp_mult + (1 - np.tanh(z_dist)) * (lift_mult - grasp_mult)

        ### hover reward for getting object to target ###
        r_hover = 0.
        # if r_lift > 0.:
        #     dist_hover = np.linalg.norm(obj_target_pos)
        #     r_hover = grasp_mult + (1 - np.tanh(dist_hover)) * (hover_mult - grasp_mult)
        
        if r_grasp > 0.: 
            dist_hover = np.linalg.norm(obj_target_pos)
            r_hover = grasp_mult + (1 - np.tanh(dist_hover)) * (hover_mult - grasp_mult)

        ### target ###
        r_target = 0.
        # if r_hover > 0.:
        dist_target = np.linalg.norm(obj_target_pos)
        if dist_target < 0.05:
            r_target = target_mult

        if r_grasp > 0.:
            reward = r_hover + r_target
        else:
            reward = r_reach + r_grasp
        # staged_reward = r_reach + r_grasp + r_hover + r_target
        if self.debug_print:
            print("reward_reach: ", r_reach)
            print("reward_grasp: ", r_grasp)
            print("reward_hover: ", r_hover)
            print("reward_target: ", r_target)

        done = False
        if object_pos[2] < 0.2:
            done = True
        return reward, done
        # return r_reach, r_grasp, r_lift, r_hover, r_target

    def staged_reward_standford(self, action, goal):
        """
        Returns staged rewards based on current physical states.
        Stages consist of reaching, grasping, lifting, and hovering.
        """
        object_pos = self.sim.data.get_site_xpos('object0')
        grip_pos = self.sim.data.get_site_xpos('r_grip_site')
        grip_obj_pos = object_pos - grip_pos
        obj_target_pos = goal - object_pos
        
        control_mult = 0.1
        reach_mult = 0.1
        grasp_mult = 0.5
        lift_mult = 0.35
        hover_mult = 0.7
        target_mult = 0.9
        reward = 0.

        ### control action ###
        action_sum = np.square(action).sum()
        r_ctrl = (1 - np.tanh(0.5 * action_sum)) * control_mult

        ### reaching reward governed by distance to closest object ###
        r_reach = 0
        # get reaching reward via minimum distance to a target object
        dist = np.linalg.norm(grip_obj_pos)
        r_reach = (1 - np.tanh(1.0 * dist)) * reach_mult
        # r_reach = - dist * reach_mult

        ### grasping reward for touching any objects of interest ###
        # r_grasp = 0.
        # touch_left_finger = False
        # touch_right_finger = False
        # touch_object = False
        # if np.linalg.norm(grip_obj_pos) < 0.05:
            # r_grasp = grasp_mult
            # touch_object = True
        # if object_pos[2] > (0.4 + 0.025 + 0.05):
            # r_grasp = grasp_mult

        ### lifting reward for picking up an object ###
        r_lift = 0.
        # if r_grasp > 0. and np.linalg.norm(grip_obj_pos) < 0.04:
            # object_pos[2] > (0.4 + 0.025 + 0.05)
            # r_lift = lift_mult
        if np.linalg.norm(grip_obj_pos) < 0.05:
            z_dist = 0.5 - object_pos[2]    
            if self.debug_print:
                print("object_pos_z: ", object_pos[2])
                print("z_dist: ", z_dist)
            # r_lift = reach_mult + (1 - np.tanh(1.0 * np.linalg.norm(z_dist))) * (lift_mult - reach_mult)

        # r_lift = 0.
        # if object_pos[2] > (0.4 + 0.025 + 0.05):
            # r_lift = lift_mult

        r_grasp = 0.
        if object_pos[2] > (0.4 + 0.025 + 0.05):
            r_grasp = grasp_mult

        ### hover reward for getting object to target ###
        r_hover = 0.
        if r_grasp > 0.:
            dist_hover = np.linalg.norm(obj_target_pos)
            if self.debug_print:
                print("dist_hover: ", dist_hover)
            r_hover = grasp_mult + (1 - np.tanh(1.0 * dist_hover)) * (hover_mult - grasp_mult)

        ### target ###
        r_target = 0.
        # if r_hover > 0.:
        dist_target = np.linalg.norm(obj_target_pos)
        if dist_target < 0.1:
            r_target = target_mult
            if dist_target < 0.05:
                r_target += target_mult
                if dist_target < 0.01:
                    r_target += target_mult

        staged_reward = [r_reach, r_grasp, r_lift, r_hover, r_target]
        # staged_reward = [r_reach, r_grasp, r_lift]
        
        reward = r_ctrl + max(staged_reward)
        
        if self.debug_print:
            print("reward_control: ", r_ctrl)
            print("reward_reach: ", r_reach)
            print("reward_lift: ", r_lift)
            print("reward_grasp: ", r_grasp)
            print("reward_hover: ", r_hover)
            print("reward_target: ", r_target)
            print("staged_reward: ", staged_reward)
            print("total reward: ", reward)
        done = False
        if object_pos[2] < 0.2:
            done = True
        
        return reward, done
        # return r_reach, r_grasp, r_lift, r_hover, r_target

    def reward_pick(self, action, goal):
        """
        Simple reward function: reach and pick
        """
        # object_pos_1 = self.sim.data.get_site_xpos('object1')
        object_pos = self.sim.data.get_site_xpos('object0')
        if self.debug_print:
            print("self.sim.data.get_site_xpos('object0'): ", object_pos)
        grip_pos = self.sim.data.get_site_xpos('r_grip_site')
        if self.debug_print:
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
        if self.debug_print:
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
        if self.debug_print:
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

    def reward_place(self, action, goal):
        """
        Simple reward function: reach and pick
        """
        object_pos = self.sim.data.get_site_xpos('object0')
        grip_pos = self.sim.data.get_site_xpos('r_grip_site')
        grip_obj_pos = object_pos - grip_pos
        obj_target_pos = goal - object_pos

        reward_ctrl = 0
        reward_dist_object = 0
        reward_grasping = 0
        reward_dist_target = 0
        reward_target = 0
        reward = 0

        reward_ctrl = -np.square(action).sum()

        reward_dist_object = -np.linalg.norm(grip_obj_pos)
        reward_dist_target = -np.linalg.norm(obj_target_pos)

        # stage 1: approaching and grasping
        if object_pos[2] > 0.45: # table hight + object hight + lift distance
            # grasping success
            reward_grasping = 1
            if object_pos[2] > 0.47:
                reward_grasping += 5
                if object_pos[2] > 0.5:
                    reward_dist_target + 10
        reward = 0.05 * reward_ctrl + reward_dist_object + reward_grasping

        # stage 2: approaching and target
        if reward_grasping > 0:
            if np.linalg.norm(obj_target_pos) < 0.1:
                reward_target = 10
                if np.linalg.norm(obj_target_pos) < 0.05:
                    reward_target += 10
                    if np.linalg.norm(obj_target_pos) < 0.01:
                        reward_target += 10

        reward = 0.05 * reward_ctrl + reward_dist_target + reward_target

        # reward = 0.05 * reward_ctrl + reward_dist_object + reward_grasping + 10 * reward_dist_target + reward_target
        if self.debug_print:
            print("object_pose: ", object_pos)
            print("reward_dist_object: ", reward_dist_object)
            print("reward_ctrl: ", 0.05 * reward_ctrl)
            print("reward_grasping: ", reward_grasping)
            print("reward_dist_target: ", reward_dist_target)
            # print("reward_target: ", reward_target)
            print("total reward: ", reward)
        done = False
        if object_pos[2] < 0.2:
            done = True
        return reward, done

    # RobotEnv methods
    # ----------------------------

    def _step_callback(self):
        if self.block_gripper:
            # self.sim.data.set_joint_qpos('robot0:l_gripper_finger_joint', 0.)
            # self.sim.data.set_joint_qpos('robot0:r_gripper_finger_joint', 0.)
            self.sim.forward()

    def _set_action1(self, action):
        assert action.shape == (self.n_actions,) # 6 mobile base
        action = action.copy()  # ensure that we don't change the action outside of this scope
        if self.debug_print:
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
        # gripper_ctrl = self.gripper_format_action(gripper_ctrl)
        # assert gripper_ctrl.shape == (2,)
        # assert gripper_ctrl.shape == (self.gripper_actual_dof,)
        if self.block_gripper:
            gripper_ctrl = np.zeros_like(gripper_ctrl)
        action = np.concatenate([pos_ctrl, rot_ctrl, base_ctrl, gripper_ctrl])
        # action = np.concatenate([pos_ctrl, rot_ctrl, gripper_ctrl])

        # Apply action to simulation.
        # utils.ctrl_set_action(self.sim, action) # base control + gripper control
        # utils.mocap_set_action(self.sim, action) # arm control in cartesion (x, y, z)
        self.set_trajectory_ee(action)
        utils_gripper.gripper_control(action)

    def _set_action(self, action):
        if self._use_real_robot:
  
            assert action.shape == (self.n_actions,) # 6 mobile base
            action = action.copy()  # ensure that we don't change the action outside of this scope
            if self.debug_print:
                print("_set_action:", action)
            pos_ctrl, base_ctrl, gripper_ctrl = action[:3], action[3:-1], action[-1]

            pos_ctrl *= 0.03  # limit maximum change in position
            base_ctrl *= 0.01

            rot_ctrl = [0, 0.707, 0.707, 0] # fixed rotation of the end effector, expressed as a quaternion

            if self.gripper_close:
                gripper_ctrl = -1.0
            else:
                gripper_ctrl = 1.0

            if self.block_gripper:
                gripper_ctrl = np.zeros_like(gripper_ctrl)
            # action = np.concatenate([pos_ctrl, rot_ctrl, base_ctrl, gripper_ctrl])

            ee_pose = self.husky_ur5_robot.arm_get_ee_pose(self.use_arm)
            arm_action = [ee_pose.pose.position.x + pos_ctrl[0], 
                          ee_pose.pose.position.y + pos_ctrl[1], 
                          ee_pose.pose.position.z + pos_ctrl[2],
                          ee_pose.pose.orientation.w,
                          ee_pose.pose.orientation.x,
                          ee_pose.pose.orientation.y,
                          ee_pose.pose.orientation.z]

            # Applay action to real robot
            # self.husky_ur5_robot.arm_set_ee_pose_relative(pos_ctrl)
            self.husky_ur5_robot.arm_set_ee_pose(arm_action)
            self.husky_ur5_robot.base_velocity_cmd(base_ctrl)
            # self.husky_ur5_robot.base_go_to_relative(base_ctrl)
            if self.gripper_close:
                self.husky_ur5_robot.gripper_close(self.use_arm)
            else:
                self.husky_ur5_robot.gripper_open(self.use_arm)

        else:
                
            assert action.shape == (self.n_actions,) # 6 mobile base
            action = action.copy()  # ensure that we don't change the action outside of this scope
            if self.debug_print:
                print("_set_action:", action)
            pos_ctrl, base_ctrl, gripper_ctrl = action[:3], action[3:-1], action[-1]
            # pos_ctrl, gripper_ctrl = action[:3], action[3:]

            pos_ctrl *= 0.03  # limit maximum change in position
            base_ctrl *= 0.01
            # rot_ctrl = [1., 0., 1., 0.]  # fixed rotation of the end effector, expressed as a quaternion

            rot_ctrl = [0, 0.707, 0.707, 0] #(0 0 0)
            # rot_ctrl = [0.707, 0.0, 0.0, -0.707] # (0 0 -90)
            # rot_ctrl = np.array([0.5, -0.5, 0.5, -0.5]) #(-90, 90, 0)
            # rot_ctrl = np.array([0.5, 0.5, 0.5, -0.5]) #(90, 0, 90) gripper down
            # rot_ctrl = np.array([0.707, 0.0, 0.0, -0.707]) #(0, 0, -90)
            # gripper_ctrl = np.array([gripper_ctrl, gripper_ctrl])
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
            utils.ctrl_set_action(self.sim, action) # base control + gripper control
            utils.mocap_set_action(self.sim, action) # arm control in cartesion (x, y, z)

            # Applay action to real robot
            gripper_cmd = 0
            if self.gripper_close:
                gripper_cmd = np.array([-1.0])
            else:
                gripper_cmd = np.array([1.0])
            action_ros = np.concatenate([pos_ctrl, rot_ctrl, base_ctrl, gripper_cmd])
            # utils_ros.set_trajectory_ee(action)
            # utils_ros.gripper_control()

    def _get_obs(self):
        if self._use_real_robot:
            joint_angles = []
            joint_velocity = []
            ee_position = []
            ee_orientation = []

            if self.use_arm == 'left':
                joint_names_dict = self.husky_ur5_robot.arm_get_joint_angles(self.use_arm)
                joint_velocity_dict = self.husky_ur5_robot.arm_get_joint_velocity(self.use_arm)
                for i in self.left_arm_joint_names:
                    joint_angles.append(joint_names_dict[i])
                    joint_velocity.append(joint_velocity_dict[i])
                ee_pose = self.husky_ur5_robot.arm_get_ee_pose(self.use_arm)
                ee_position = [ee_pose.pose.position.x, 
                               ee_pose.pose.position.y,
                               ee_pose.pose.position.z]
                ee_orientation = [ee_pose.pose.orientation.w,                
                                  ee_pose.pose.orientation.x,
                                  ee_pose.pose.orientation.y,
                                  ee_pose.pose.orientation.z,]

            if self.use_arm == 'right':
                joint_names_dict = self.husky_ur5_robot.arm_get_joint_angles(self.use_arm)
                joint_velocity_dict = self.husky_ur5_robot.arm_get_joint_velocity(self.use_arm)
                for i in self.left_arm_joint_names:
                    joint_angles.append(joint_names_dict[i])
                    joint_velocity.append(joint_velocity_dict[i])
                ee_pose = self.husky_ur5_robot.arm_get_ee_pose(self.use_arm)
                ee_position = [ee_pose.pose.position.x, 
                               ee_pose.pose.position.y,
                               ee_pose.pose.position.z]
                ee_orientation = [ee_pose.pose.orientation.w,                
                                  ee_pose.pose.orientation.x,
                                  ee_pose.pose.orientation.y,
                                  ee_pose.pose.orientation.z,]

            grip_pos = ee_position
            object_pos = np.array([0.2, 0.2, 0.2])
            object_rel_pos = np.array([0.1, 0.1, 0.1])
            ur5_qpos = np.array(joint_angles)
            ur5_qvel = np.array(joint_velocity)
            if self.debug_print:
                print("grip_pos: ", grip_pos)
                print("object_pos: ", object_pos)
                print("object_rel_pos: ", object_rel_pos)
                print("ur5_qpos: ", ur5_qpos)
                print("ur5_qvel: ", ur5_qvel)
            obs = np.concatenate([
                grip_pos,
                object_pos,
                object_rel_pos,
                ur5_qpos,
                ur5_qvel
            ])
            if self.debug_print:
                print("observation: ", obs)
            return obs

        else:
            # positions
            grip_pos = self.sim.data.get_site_xpos('r_grip_site')
            dt = self.sim.nsubsteps * self.sim.model.opt.timestep
            grip_velp = self.sim.data.get_site_xvelp('r_grip_site') * dt
            robot_qpos, robot_qvel = utils.robot_get_obs(self.sim)
            ur5_qpos, ur5_qvel = utils_test.robot_get_ur5_joint_state_obs(self.sim)
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
            if self.debug_print:
                print("grip_pos: ", grip_pos)
                print("object_pos: ", object_pos)
                print("object_pos.ravel: ", object_pos.ravel())
                print("object_rel_pos.ravel: ", object_rel_pos.ravel())
                print("object_rel_pos: ", object_rel_pos)
                print("ur5_qpos: ", ur5_qpos)
                print("ur5_qvel: ", ur5_qvel)
            obs = np.concatenate([
                grip_pos, 
                object_pos.ravel(), 
                object_rel_pos.ravel(), 
                ur5_qpos,
                ur5_qvel,
                # gripper_state, 
                # object_rot.ravel(),
                # object_velp.ravel(), 
                # object_velr.ravel(), 
                # grip_velp, 
                # gripper_vel,
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
            if self.debug_print:
                print("object_xpos0: ", object_xpos)
            # print("object1 pos: ", object_qpos1)
            object_qpos[:2] = object_xpos
            object_qpos[2] = 0.5
            # object_qpos[0] += 0.3
            # object_qpos[1] -= 0.1
            if self.debug_print:
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
        d = goal_distance(achieved_goal, desired_goal)
        return (d < self.distance_threshold).astype(np.float32)

    def _env_setup(self, initial_qpos):
        for name, value in initial_qpos.items():
            self.sim.data.set_joint_qpos(name, value)
        utils.reset_mocap_welds(self.sim)
        self.sim.forward()

        # Move end effector into position.
        # gripper_target = np.array([-0.498, 0.005, -0.431 + self.gripper_extra_height]) + self.sim.data.get_site_xpos('r_grip_site')
        gripper_target = np.array([0.498, 0.005, 0.431 + self.gripper_extra_height]) + self.sim.data.get_site_xpos('r_grip_site')
        if self.debug_print:
            print("gripper quat: ", self.sim.data.get_site_xmat('r_grip_site'))
            print("get_mocap_quat: ", self.sim.data.get_mocap_quat('gripper_r:mocap'))
        # gripper_rotation = np.array([1., 0., 1., 0.])
        # gripper_rotation = np.array([-0.82031777, -0.33347336, -0.32553968,  0.33150896])

        # gripper_target = np.array([0.9, -0.3, 0.6])
        gripper_target = np.array([0.5, -0.3, 0.6])
        # gripper_rotation = np.array([0.5, -0.5, -0.5, -0.5]) #(-90, 0, -90)
        # gripper_rotation = np.array([0.5, 0.5, 0.5, -0.5]) #(90, 0, 90) gripper down
        # gripper_rotation = np.array([0.707, 0.0, 0.0, -0.707]) #(0, 0, -90)
        gripper_rotation = np.array([0, 0.707, 0.707, 0]) #(0, 0, -90)
        # gripper_rotation = np.array([1.0, 0, 0, 0])
        # set random gripper position
        # for i in range(3):
        #     gripper_target[i] += self.np_random.uniform(-0.2, 0.2)
        # print("gripper target random: ", gripper_target)
        self.sim.data.set_mocap_pos('gripper_r:mocap', gripper_target)
        self.sim.data.set_mocap_quat('gripper_r:mocap', gripper_rotation)
        for _ in range(10):
            self.sim.step()

        # # set random end-effector position
        # end_effector_pos = self.initial_gripper_xpos
        # rot_ctrl = [0, 0.707, 0.707, 0] #(0 0 0)
        # end_effector_pos = np.concatenate([end_effector_pos, rot_ctrl])
        # print("end_effector_pos: ", end_effector_pos)
        # for i in range(3):
        #     end_effector_pos[i] = self.initial_gripper_xpos[i] + self.np_random.uniform(-0.1, 0.1)
        # utils.mocap_set_action(self.sim, end_effector_pos) # arm control in cartesion (x, y, z)
        # # for _ in range(100):
        #     # self.sim.step()

        # Extract information for sampling goals.
        self.initial_gripper_xpos = self.sim.data.get_site_xpos('r_grip_site').copy()
        if self.has_object:
            self.height_offset = self.sim.data.get_site_xpos('object0')[2]

    def render(self, mode='human', width=500, height=500):
        return super(MobileDualUR5HuskyGymEnv, self).render(mode, width, height)

    ### Add function for dual_ur5_husky

    def gripper_format_action(self, action):
        """ Given (-1,1) abstract control as np-array return the (-1,1) control signals
        for underlying actuators as 1-d np array
        Args:
            action: 1 => open, -1 => closed
        """
        movement = np.array([1, 1, 1, 0])
        return -1 * movement * action

    def gripper_format_action11(self, action):
        """ Given (-1,1) abstract control as np-array return the (-1,1) control signals
        for underlying actuators as 1-d np array
        Args:
            action: 1 => open, -1 => closed
        """
        movement = np.array([0, 1, 1, 1, 0, 1, 1, 1, 1, 1, 1])
        return -1 * movement * action

    