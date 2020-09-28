import gym
import time

env = gym.make("CartPole-v0")
obs = env.reset()
env.render()

while True:
    obs, reward, done, info = env.step(1)
    env.render()
    time.sleep(0.1)
    if done:
        break