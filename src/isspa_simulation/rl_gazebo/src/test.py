import gym
from rl_gazebo_env import GazeboCarEnv

env = GazeboCarEnv()
obs = env.reset()
print(obs)

