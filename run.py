import envs
import gym
env = gym.make('Traffic-Multicar-gui-v0')

# import IPython
# IPython.embed()
env.reset()
while True:
    # env.render()
    observation, reward, done, info = env.step(env.action_space.sample()) # take a random action
    if done:
        print("reset!!!!!!!!!!!!!")
        env.reset()