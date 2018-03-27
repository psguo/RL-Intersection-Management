import envs
import gym
env = gym.make('Traffic-Multicar-gui-v0')

import IPython
IPython.embed()
for _ in range(1000):
    # env.render()
    observation, reward, done, info = env.step(env.action_space.sample()) # take a random action
    if done:
        env.reset()