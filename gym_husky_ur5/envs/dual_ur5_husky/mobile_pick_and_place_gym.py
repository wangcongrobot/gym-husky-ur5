import os
from gym import utils
from gym_husky_ur5.envs import mobile_dual_ur5_husky_gym_env

# Ensure we get the path separator correct on windows
MODEL_XML_PATH = os.path.join('dual_ur5_husky', 'mobile_pick_and_place.xml')

print(MODEL_XML_PATH)

class MobileDualUR5HuskyPickAndPlaceGymEnv(mobile_dual_ur5_husky_gym_env.MobileDualUR5HuskyGymEnv, utils.EzPickle):
    def __init__(self, reward_type='sparse'):
        initial_qpos = {
            'robot0:slide0': 0.0,
            'robot0:slide1': 0.,
            'robot0:slide2': 0.3,
            'object0:joint': [1.25, 0.53, 0.4, 1., 0., 0., 0.],
        }
        mobile_dual_ur5_husky_gym_env.MobileDualUR5HuskyGymEnv.__init__(
            self, MODEL_XML_PATH, has_object=True, block_gripper=False, n_substeps=20,
            gripper_extra_height=0.2, target_in_the_air=True, target_offset=0.0,
            obj_range=0.1, target_range=0.1, distance_threshold=0.05,
            initial_qpos=initial_qpos, reward_type=reward_type, n_actions=6,
            use_real_robot=False, debug_print=True)
        utils.EzPickle.__init__(self)
