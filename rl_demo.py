import numpy as np
import matplotlib.pyplot as plt

from stable_baselines3.common.env_checker import check_env

from stable_baselines3 import PPO
from stable_baselines3.ppo import MlpPolicy

import gym
from gym import spaces

import scipy.stats


def f(x):
    mean = 5
    std = 1
    dist = scipy.stats.norm(mean, std)
    return dist.pdf(x)


class CustomEnv(gym.Env):
    """Custom Environment that follows gym interface"""
    metadata = {'render.modes': ['human']}

    def __init__(self):
        super(CustomEnv, self).__init__()
        # Define action and observation space
        # They must be gym.spaces objects
        # Example when using discrete actions:
        # self.action_space = spaces.Discrete(2)
        self.action_space = spaces.Box(low=-1, high=1, shape=(1, ))
        # Example for using image as input (channel-first; channel-last also works):
        self.observation_space = spaces.Box(low=0, high=10, shape=(1, ))

        self.pos = np.array([10.0])

    def step(self, action):
        try:
            self.pos[:] += action[0]
        except:
            self.pos[:] += action - 0.5
        self.pos[self.pos > 10] = 10.0
        self.pos[self.pos < 0] = 0.0

        observation = self.pos
        reward = f(self.pos[0])
        done = False
        info = {}
        return observation, reward, done, info

    def reset(self):

        self.pos = np.array([10.0])
        observation = self.pos
        return observation  # reward, done, info can't be included

    def render(self, mode='human'):
        pass

    def close(self):
        pass


def main():
    env = CustomEnv()
    check_env(env=env)

    expert = PPO(
        policy=MlpPolicy,
        env=env,
        # seed=0,
        # batch_size=64,
        # ent_coef=0.0,
        # learning_rate=0.0003,
        # n_epochs=10,
        # n_steps=64,
    )

    # If discrete action, needs 50000
    expert.learn(50000)

    x = np.linspace(0, 10, 100)
    y = np.zeros_like(x)
    for i in range(x.shape[0]):
        obs = np.array([x[i], ])
        action, _ = expert.predict(obs)
        try:
            y[i] = action[0]
        except:
            y[i] = action

    fig, ax = plt.subplots()

    ax.plot(x, y)
    ax2 = ax.twinx()
    ax2.plot(x, f(x), color='C1')
    plt.show()

    # action


if __name__ == "__main__":
    main()
