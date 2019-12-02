import gym
import gym_husky_ur5

#add parent dir to find package. Only needed for source code build, pip install doesn't need it.
import os, inspect
currentdir = os.path.dirname(os.path.abspath(inspect.getfile(inspect.currentframe())))
parentdir = os.path.dirname(os.path.dirname(currentdir))
os.sys.path.insert(0, parentdir)


# import RL agent
from stable_baselines.bench import Monitor
from stable_baselines.results_plotter import load_results, ts2xy

from stable_baselines.sac.policies import MlpPolicy, LnMlpPolicy
from stable_baselines import SAC

from stable_baselines.ppo2.ppo2 import constfn
from robot_agents.utils import linear_schedule
from stable_baselines.ddpg import AdaptiveParamNoiseSpec
#
import numpy as np
import math as m
global output_dir

best_mean_reward, n_steps = -np.inf, 0

def callback(_locals, _globals):
    """
    Callback called at each step (for DQN an others) or after n steps (see ACER or PPO2)
    :param _locals: (dict)
    :param _globals: (dict)
    """
    global n_steps, best_mean_reward
    # Print stats every 1000 calls
    if (n_steps + 1) % 500 == 0:
        # Evaluate policy training performance
        x, y = ts2xy(load_results(os.path.join(output_dir,'log')), 'timesteps')
        if len(x) > 0:
            mean_reward = np.mean(y[-100:])
            print(x[-1], 'timesteps')
            print("Best mean reward: {:.2f} - Last mean reward per episode: {:.2f}".format(best_mean_reward, mean_reward))

            # New best model, you could save the agent here
            if mean_reward > best_mean_reward:
                best_mean_reward = mean_reward
            if (n_steps + 1) % 500000 == 0:
                # Save model
                print("Saving model at iter {}".format(x[-1]))
                _locals['self'].save(os.path.join(output_dir, str(x[-1])+'model.pkl'))
    n_steps += 1
    # Returning False will stop training early
    return True


def train_SAC( env, out_dir, seed=None, **kwargs):

    # Logs will be saved in log_dir/monitor.csv
    global output_dir
    output_dir = out_dir
    log_dir = os.path.join(out_dir,'log')
    os.makedirs(log_dir, exist_ok=True)
    env = gym.make(env)
    env = Monitor(env, log_dir+'/', allow_early_resets=True)

    # Delete keys so the dict can be pass to the model constructor
    # policy = kwargs['policy']
    policy = 'MlpPolicy'
    # n_timesteps = kwargs['n_timesteps']
    n_timesteps = int(1e6)
    noise_type = None
    # Add some param noise for exploration
    param_noise = AdaptiveParamNoiseSpec(initial_stddev=0.1, desired_action_stddev=0.1)

    continue_model = False
    if continue_model is True:
        # Continue training
        print("Loading pretrained agent")
        model = SAC.load(os.path.join(out_dir,'final_model.pkl'), env=env,
                         tensorboard_log=os.path.join(log_dir,'tb'), verbose=1, **kwargs)
    else:
        model = SAC(policy, env,# action_noise=param_noise,
                    verbose=1, tensorboard_log=os.path.join(log_dir,'tb'), full_tensorboard_log=False, **kwargs)

    model.learn(total_timesteps=n_timesteps, seed=seed, callback=callback, log_interval=10)

    return model

if __name__ == '__main__':

    train_SAC('UR5Gripper-v0', '/tmp/gym/sac')