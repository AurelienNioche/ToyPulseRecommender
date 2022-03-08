import  numpy as np

from stable_baselines3.common.env_checker import check_env

import gym
from gym import spaces

class CustomEnv(gym.Env):
    """Custom Environment that follows gym interface"""
    metadata = {'render.modes': ['human']}

    def __init__(self):
        super(CustomEnv, self).__init__()
        # Define action and observation space
        # They must be gym.spaces objects
        # Example when using discrete actions:
        self.action_space = spaces.Box(low=-1, high=1, shape=(1, ))
        # Example for using image as input (channel-first; channel-last also works):
        self.observation_space = spaces.Box(low=40, high=140, shape=(1, ))

    def step(self, action):
        observation = np.array([82, ])
        reward = 10
        done = False
        info = {}
        return observation, reward, done, info
    def reset(self):
        observation = np.array([82, ])
        return observation  # reward, done, info can't be included
    def render(self, mode='human'):
        pass
    def close (self):
        pass


def main():
    env = CustomEnv()
    check_env(env=env)


if __name__ == "__main__":
    main()
