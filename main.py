import numpy as np
import matplotlib.pyplot as plt

from stable_baselines3.common.env_checker import check_env

from stable_baselines3 import PPO
from stable_baselines3.ppo import MlpPolicy

import gym
from gym import spaces

import serial
import json

import os
import time


class CustomEnv(gym.Env):
    """Custom Environment that follows gym interface"""
    metadata = {'render.modes': ['human']}

    def __init__(self, obs_min=40, obs_max=140,
                 stim_duration=2.0,
                 init_pos=90.0,
                 continuous_action_space=True):

        super(CustomEnv, self).__init__()

        if continuous_action_space:
            self.action_space = spaces.Box(low=-1, high=1, shape=(1, ))
        else:
            self.action_space = spaces.Discrete(2)

        self.observation_space = spaces.Box(low=obs_min, high=obs_max, shape=(1, ))

        self.obs_min = obs_min
        self.obs_max = obs_max
        self.init_pos = init_pos

        self.stim_duration = stim_duration

        self.pos = np.array([init_pos])

    def step(self, action):

        if isinstance(self.action_space, spaces.Box):
            self.pos[:] += action[:] * 10
        else:
            self.pos[:] += (action - 0.5) * 10

        self.pos[self.pos > self.obs_max] = self.obs_max
        self.pos[self.pos < self.obs_min] = self.obs_min

        observation = self.pos

        frequency = self.pos[0]  # Hertz

        print("choose frequency", frequency)

        os.system(f'play -v 2.0 -n synth {self.stim_duration} sin {frequency}')

        start_time = time.time()

        with serial.Serial('/dev/cu.usbmodem112201', 115200, timeout=.1) as arduino:
            # need to drop first data
            while True:
                data = arduino.readline().strip()
                if data:
                    break

            # timestamp = []
            bpm = []
            spO2 = []
            while time.time() - start_time < self.stim_duration:
                data = arduino.readline().strip()
                if data:
                    data = data.decode()
                    try:
                        dic_data = json.loads(data)
                        # timestamp.append(dic_data["timestamp"])
                    except (UnicodeDecodeError, json.decoder.JSONDecodeError):
                        print("Not data", data)
                        continue

                    bpm.append(dic_data["bpm"])
                    spO2.append(dic_data["spO2"])
        mean_bpm = np.mean(bpm)
        reward = - mean_bpm / 100



        done = False
        info = {}
        return observation, reward, done, info

    def reset(self):

        self.pos = np.array([self.init_pos, ])
        observation = self.pos
        print(observation)
        return observation  # reward, done, info can't be included

    def render(self, mode='human'):
        pass

    def close(self):
        pass


def main():
    print("Initializing...")
    with serial.Serial('/dev/cu.usbmodem112201', 115200, timeout=.1) as arduino:
        # need to drop first data
        while True:
            data = arduino.readline().strip()

            if data:
                data = data.decode()
                try:
                    dic_data = json.loads(data)
                    # timestamp.append(dic_data["timestamp"])
                    if dic_data["bpm"] > 40:
                        break

                except (UnicodeDecodeError, json.decoder.JSONDecodeError):
                    continue

    print("ready!")
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

    # x = np.linspace(0, 10, 100)
    # y = np.zeros_like(x)
    # for i in range(x.shape[0]):
    #     obs = np.array([x[i], ])
    #     action, _ = expert.predict(obs)
    #     try:
    #         y[i] = action[0]
    #     except:
    #         y[i] = action




if __name__ == "__main__":
    main()
