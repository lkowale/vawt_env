# based on RL/openai_gym/nchain

import gym
import time
import numpy as np
import learn.airfoil_dynamics.ct_plot.vawt_blade as vb
import math
import learn.airfoil_dynamics.ct_plot.base_calculus as bc
import pandas as pd


class VawtEnvironment:

    def __init__(self, blade, steps=1000):
        self.blade = blade
        self.steps = steps
        # current state of blade (theta, pitch)
        self.data = self.tf_data()
        self.theta_num = self.data.shape[0]
        self.pitch_num = self.data.shape[1]
        self.current_step = 0
        # indexes of current state
        self.position = (0, 0)

    def reset(self):
        self.position = (0, round(self.data.shape[1]/2))
        return self.position

    def step(self, action):
        self.current_step += 1
        self.position = (self.get_new_theta(), self.get_new_pitch(action))
        #todo dispatch IndexError: single positional indexer is out-of-bounds
        reward = self.data.iloc[self.position[0], self.position[1]]
        done = False
        if self.current_step > self.steps:
            done = True
        debug = None
        return self.position, reward, done, debug

    def tf_data(self):
        wind_direction = 0
        wind_speed = 3
        rotor_speed = 3
        wind_vector = bc.get_wind_vector(wind_direction, wind_speed)
        theta_range = [x * math.tau / 360 for x in range(-180, 180, 5)]
        pitch_range = [x * math.tau / 360 for x in range(-180, 180, 5)]
        thetas = []
        for theta in theta_range:
            theta_ct_polar = [blade.get_tangential_force(wind_vector, rotor_speed, theta, pitch) for pitch in pitch_range]
            thetas.append(theta_ct_polar)

        df = pd.DataFrame(thetas, index=theta_range, columns=pitch_range)
        return df

    def get_new_theta(self):
        new_theta = self.position[0] + 1
        if new_theta > self.data.shape[0]:
            new_theta = 0
        return new_theta

    def get_new_pitch(self, delta):
        new_pitch = self.position[1] + delta
        if new_pitch > self.data.shape[1]:
            new_pitch = 0
        if new_pitch < 0:
            new_pitch = self.data.shape[1] + new_pitch
        return new_pitch


def naive_sum_reward_agent(env, num_episodes=500):
    # state is rotor theta and blade pitch
    # next state is new theta and pitch
    # there are theta * pitch 360*360= 129600 states
    # assuming theta 2pi range and 5 degrees step = 120 positions
    # and pitch (-1,1) range and 5 degree step = 120 positions
    # we get 120*120 = 14400 states
    # an action is a transition to theta + 1 and pitch + (angle from range of angle changes)
    # a reward is tangential force generated by blade in current position
    # assume maximum change of blade pitch

    max_pitch_change = 15

    # create q table
    r_table = np.zeros((env.data.size, max_pitch_change * 2 + 1))
    indexes = [range(env.data.shape[0]), range(env.data.shape[1])]
    m_index = pd.MultiIndex.from_product(indexes, names=['theta', 'pitch'])
    r_df = pd.DataFrame(r_table, index=m_index, columns=range(-max_pitch_change, max_pitch_change+1))
    for g in range(num_episodes):
        s = env.reset()
        done = False
        while not done:
            # multiindex loc
            s_index = (s[0], s[1])
            if np.sum(r_df.loc[s_index]) == 0:
                # make a random selection of actions
                a = np.random.randint(-max_pitch_change, max_pitch_change)
            else:
                # select the action with highest cummulative reward
                a = np.argmax(r_df.loc[s_index])
            new_s, r, done, _ = env.step(a)
            r_df.loc[s_index, a] += r
            s = new_s
        print("Episode {}".format(g))
    return r_table


airfoil_dir = '/home/aa/vawt_env/learn/AeroDyn polars/naca0018_360'
# vb.VawtBlade(blade chord, airfoil_dir, rotor_radius)
blade = vb.VawtBlade(0.2, airfoil_dir, 1)
env = VawtEnvironment(blade)
table = naive_sum_reward_agent(env)
print(table)