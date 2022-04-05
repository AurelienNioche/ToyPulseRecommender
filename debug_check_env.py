import gym
import numpy as np
from stable_baselines3.common.env_checker import check_env


def f(x):
    return x**2


class CustomEnv(gym.Env):
    """Custom Environment that follows gym interface"""
    metadata = {'render.modes': ['human']}

    def __init__(self):
        super(CustomEnv, self).__init__()
        self.action_space = gym.spaces.Box(low=-1, high=1, shape=(1,))

        self.observation_space = gym.spaces.Box(low=-1, high=1,
                                                shape=(1,))  # spaces.Box(low=obs_min, high=obs_max, shape=(1, ))

        self.obs_ref = self.observation_space.sample()

    def step(self, action):
        x = action[0] * 10.0
        observation = np.zeros_like(self.obs_ref)
        # observation[0] = 0.0
        reward = - f(x)
        done = False
        info = {}
        return observation, reward, done, info

    def reset(self):
        observation = np.zeros_like(self.obs_ref)
        return observation  # reward, done, info can't be included

    def render(self, mode='human'):
        pass

    def close(self):
        pass


env = CustomEnv()

check_env(env)



