import gym
import envs
env = gym.make('Traffic-Multicar-gui-v0')
env.reset()
for _ in range(1000):
    # env.render()
    observation, reward, done, info = env.step(env.action_space.sample()) # take a random action
    if done:
        env.reset()