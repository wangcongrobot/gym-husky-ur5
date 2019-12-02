from gym.envs.registration import register



# Robotics
# ----------------------------------------

def _merge(a, b):
    a.update(b)
    return a

for reward_type in ['sparse', 'dense']:
    suffix = 'Dense' if reward_type == 'dense' else ''
    kwargs = {
        'reward_type': reward_type,
    }

    # Dual_UR5_Husky
    register(
        id='DualUR5HuskyPickAndPlace{}-v1'.format(suffix),
        entry_point='gym_husky_ur5.envs:DualUR5HuskyPickAndPlaceEnv',
        kwargs=kwargs,
        max_episode_steps=100,
    )

    # Mobile Dual_UR5_Husky GymGoalEnv
    register(
        id='MobileDualUR5HuskyPickAndPlace{}-v1'.format(suffix),
        entry_point='gym_husky_ur5.envs:MobileDualUR5HuskyPickAndPlaceEnv',
        kwargs=kwargs,
        max_episode_steps=100,
    )

    # Mobile Dual_UR5_Husky GymEnv
    register(
        id='MobileDualUR5HuskyPickAndPlaceGym{}-v1'.format(suffix),
        entry_point='gym_husky_ur5.envs:MobileDualUR5HuskyPickAndPlaceGymEnv',
        kwargs=kwargs,
        max_episode_steps=100,
    )

    # Mobile Dual_UR5_Husky ROSEnv
    register(
        id='MobileDualUR5HuskyPickAndPlaceROS{}-v1'.format(suffix),
        entry_point='gym_husky_ur5.envs:MobileDualUR5HuskyPickAndPlaceROSEnv',
        kwargs=kwargs,
        max_episode_steps=100,
    )
