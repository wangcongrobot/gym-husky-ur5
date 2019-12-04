import gym
import numpy as np
import matplotlib.pyplot as plt

# import the new env
import gym_husky_ur5

mode = 'human'
#mode = 'rgb_array'

env = gym.make("MobileDualUR5HuskyPickAndPlace-v1")
# env = gym.make("FetchReach-v1")
print("action space high: ", env.action_space.high)
print("action space low: ", env.action_space.low)
num_actuator = env.sim.model.nu
print('num_actuator: ', num_actuator)
print('joint names:', env.sim.model.joint_names)
print("gripper pose: ", env.sim.data.get_site_xpos("r_grip_site"), env.sim.data.get_site_xmat("r_grip_site"))
print("qpos: ", env.sim.data.qpos)
# env.render('human')
#env = gym.wrappers.Monitor(env, './video', force=True)
# plt.imshow(env.render(mode='rgb_array', camera_id=0))
plt.show()

# plt.show()
# action = np.array([0.1, 0.0, 0.0, -0.1])
for i in range(20):
  env.reset()
#   env.render('human')
  # action[-1] += 0.1
  # if action[-1] > 0.01:
        # action[-1] = -1.0
  # action = np.array([0, 0.0, 0.0, -1.0])
  for i in range(20):
    action = env.action_space.sample()
#     action = np.array([0.0, 0.0, 0.0, -0.5, 0., -1.0]) # 1.0 open -1.0 close
    # action[-1] += 0.1
    # if action[-1] > 1.0:
          # action[-1] = -1.0
    print("action_space:", env.action_space)
    print("action: ", action)
    obs, reward, done, info = env.step(action)
    # print("observation:", obs)
    # print("reward:", reward)
    # print("done:", done)
    # print("info:", info)
#     env.render('human')
    print("gripper mocap pos: ", env.sim.data.get_mocap_pos('gripper_r:mocap'))
    print("gripper mocap quat: ", env.sim.data.get_mocap_quat('gripper_r:mocap'))
    print("gripper site xpos and xmat : ", env.sim.data.get_site_xpos("r_grip_site"), env.sim.data.get_site_xmat("r_grip_site"))
    print("number actuator: ", num_actuator)
#     print("qpos: ", env.sim.data.qpos)
#     print("state: ", env.sim.get_state())
    print("name: ", env.sim.model.name_actuatoradr)
    print("actuator contrl range: ", env.sim.model.actuator_ctrlrange)
    print("body robot0: gripper_link pos and xmat: ", env.sim.data.get_body_xipos("r_gripper_palm_link"), env.sim.data.get_body_xmat("r_gripper_palm_link"))
    # print("site pos: ", env.sim.data.get_site_xpos())
    print("ur5 joint1: ", env.sim.data.get_joint_qpos("r_ur5_arm_shoulder_pan_joint"))
    print("ur5 joint2: ", env.sim.data.get_joint_qpos("r_ur5_arm_shoulder_lift_joint"))
    print("ur5 joint3: ", env.sim.data.get_joint_qpos("r_ur5_arm_elbow_joint"))
    print("ur5 joint4: ", env.sim.data.get_joint_qpos("r_ur5_arm_wrist_1_joint"))
    print("ur5 joint5: ", env.sim.data.get_joint_qpos("r_ur5_arm_wrist_2_joint"))
    print("ur5 joint6: ", env.sim.data.get_joint_qpos("r_ur5_arm_wrist_3_joint"))
    print("env.sim.model.nq: ", env.sim.model.nq)

    # if done:
          # break

# https://github.com/openai/mujoco-py/blob/master/mujoco_py/tests/test_cymj.py