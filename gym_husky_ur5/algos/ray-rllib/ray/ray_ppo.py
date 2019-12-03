import ray
import ray.rllib.agents.ppo as ppo
from ray.tune.logger import pretty_print
from ray import tune

from ray.tune.registry import register_env

import gym
from gym import error, spaces, utils, wrappers
from gym.utils import seeding
from gym.envs.registration import register
from gym.spaces import Discrete, Box

import gym_husky_ur5

from gym_husky_ur5.envs import MobileDualUR5HuskyPickAndPlaceGymEnv


def create_husky_env(env_config):
    import gym
    from gym.envs.registration import register
    # This import must happen inside the method so that worker processes import this code
    # Mobile Dual_UR5_Husky GymEnv
    register(
        id='MobileDualUR5HuskyPickAndPlaceGym-v1',
        entry_point='gym_husky_ur5.envs:MobileDualUR5HuskyPickAndPlaceGymEnv',
        # kwargs=kwargs,
        max_episode_steps=100,
    )
    return gym.make("MobileDualUR5HuskyPickAndPlace-v1")

def create_env(env_config):
    # from gym_husky_ur5.envs import MobileDualUR5HuskyPickAndPlaceGymEnv
    return MobileDualUR5HuskyPickAndPlaceGymEnv()

register_env("husky", create_env)
ray.init()
# trainer = ppo.PPOTrainer(env="husky")

tune.run(
    "PPO",
    stop={"episode_reward_mean": 2000000},
    config={
        "env": "husky",
        "num_gpus": 0,
        "num_workers": 15,
        "lr": tune.grid_search([0.01, 0.001, 0.0001]),
        "eager": False,
    },
)

# config = ppo.DEFAULT_CONFIG.copy()
# config["num_workers"] = 10

# trainer = ppo.PPOTrainer(config=config, env="husky")

# # # Can optionally call trainer.restore(path) to load a checkpoint.

# for i in range(1000):
#     # Perform one iteration of training the policy with PPO
#     result = trainer.train()
#     print(pretty_print(result))

#     if i % 100 == 0:
#         checkpoint = trainer.save()
#         print("checkpoint saved at ", checkpoint)

