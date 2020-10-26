# based on RL/openai_gym/nchain

import gym
import time
import numpy as np
import learn.airfoil_dynamics.ct_plot.vawt_blade as vb
import math
import learn.airfoil_dynamics.ct_plot.base_calculus as bc
import pandas as pd
import matplotlib.pyplot as plt

# on base of given parameters a VAWT blade is constructed with its ct = f(pitch, theta) function
# having given theta range - usually 360 degrees that is a full revolve
# and pitch range - depends on how far will the blade can move to change its pitch think -60,60 degrees will be sufficent
# use blade ct function to construct a table of rewards ct with pitch in column and theta as row

# the q table is a table with multiindex (theta,pitch) as row and index change of pitch as column
# iow for each row of qtable representing given state (theta,pitch) i have a couple of columns where their index represents change of pitch as action
# column named '3' tells that next pitch index will be current_index + 3

class VawtRLEnvironment:

    def __init__(self, blade, wind_direction, wind_speed, rotor_speed, theta_resolution, pitch_resolution, steps=10000):

        self.pitch_resolution = pitch_resolution
        self.theta_resolution = theta_resolution
        self.blade = blade
        self.steps = steps
        self.wind_direction = wind_direction
        self.wind_speed = wind_speed
        self.rotor_speed = rotor_speed
        # current state of blade (theta, pitch)
        self.data = self.tf_data(self.wind_direction, self.wind_speed, self.rotor_speed, self.theta_resolution, self.pitch_resolution)
        self.theta_num = self.data.shape[0]
        self.pitch_num = self.data.shape[1]
        self.reset()

    def reset(self):
        # position is an index of theta as a row and pitch as a column in self.data
        self.position = (0, round(self.data.shape[1]/2))
        self.current_step = 0
        return self.position

    def step(self, action):
        self.current_step += 1
        try:
            reward = self.data.iloc[self.position[0], self.position[1]]
        except IndexError:
            print("Index Error")

        self.position = (self.get_new_theta(), self.get_new_pitch(action))
        done = False
        if self.current_step > self.steps:
            done = True
        debug = None
        return self.position, reward, done, debug


    def tf_data(self, wind_direction, wind_speed, rotor_speed, theta_resolution, pitch_resolution):

        wind_vector = bc.get_wind_vector(wind_direction, wind_speed)
        theta_range = [x * math.tau / 360 for x in range(-180, 180, theta_resolution)]
        pitch_range = [x * math.tau / 360 for x in range(-70, 70, pitch_resolution)]
        thetas = []
        for theta in theta_range:
            theta_ct_polar = [self.blade.get_tangential_force(wind_vector, rotor_speed, theta, pitch) for pitch in pitch_range]
            thetas.append(theta_ct_polar)

        df = pd.DataFrame(thetas, index=theta_range, columns=pitch_range)
        return df

    def get_new_theta(self):
        new_theta = self.position[0] + 1
        if new_theta >= self.data.shape[0]:
            new_theta = new_theta - self.data.shape[0]
        return new_theta

    def get_new_pitch(self, delta):
        new_pitch = self.position[1] + delta
        # conditionals for circular/revolute pitch displacement
        # if new_pitch >= self.data.shape[1]:
        #     new_pitch = self.data.shape[1] - 1
        # if new_pitch < 0:
        #     new_pitch = 0
        return new_pitch


def save_environment(env, filename, tsr):
    dict = {
    'airfoil_dir':env.blade.airfoil_dir,
    'blade_shaft_dist':env.blade.rotor_radius,
    'blade_chord_length':env.blade.chord_length,
    'wind_direction':env.wind_direction,
    'wind_speed':env.wind_speed,
    'rotor_speed':env.rotor_speed,
    'theta_resolution':env.theta_resolution,
    'pitch_resolution':env.pitch_resolution,
    'steps':env.steps,
    'tsr':tsr,
    }

    pd.DataFrame(dict, index=[0]).to_csv(filename)


def load_environment(filename):
    # load df from csv
    params = pd.read_csv(filename)
    params = params.iloc[0]
    # create blade
    airfoil_dir = params['airfoil_dir']
    blade_shaft_dist = params['blade_shaft_dist']
    blade_chord_length = params['blade_chord_length']
    blade = vb.VawtBlade(blade_chord_length, airfoil_dir, blade_shaft_dist)
    # create environment
    wind_direction = params['wind_direction']
    wind_speed = params['wind_speed']
    rotor_speed = params['rotor_speed']
    theta_resolution = params['theta_resolution']
    pitch_resolution = params['pitch_resolution']
    steps = params['steps']
    tsr = params['tsr']
    env = VawtRLEnvironment(blade, wind_direction, wind_speed, rotor_speed, theta_resolution, pitch_resolution, steps=steps)
    return env


def eps_greedy_q_learning_with_table(env, num_episodes=500, display_plot=False, save_file_name=None):

    y = 0.95
    eps = 0.5
    lr = 0.8
    # with decay 0.9985 eps gets ~0.24 after 500 episodes, with 0.999 0.30
    decay_factor = 0.999

    max_pitch_change = 5

    # create r table of size (num of theta samples*num of pitch samples)xnumber of possible new pitch values
    q_table = np.empty((env.data.size, max_pitch_change * 2 + 1))

    indexes = [range(env.data.shape[0]), range(env.data.shape[1])]
    m_index = pd.MultiIndex.from_product(indexes, names=['theta', 'pitch'])
    q_df = pd.DataFrame(q_table, index=m_index, columns=range(-max_pitch_change, max_pitch_change+1))
    q_df[:] = np.nan
    # identical in size as theta-pitch dataframe
    coverage_df = env.data.copy()
    # zero coverage df
    coverage_df[:] = 0

    for i in range(num_episodes):
        s = env.reset()
        eps *= decay_factor
        done = False
        while not done:
            # current state multiindex
            s_index = (s[0], s[1])
            # update coverage
            coverage_df.iloc[s_index] += 1
            # if there aren't any action with rewards pick randomly form theme
            # or if eps
            if q_df.loc[s_index].isnull().all() or np.random.random() < eps:
                # make a random selection of actions
                a = np.random.randint(max(-max_pitch_change, -s[1]), min(max_pitch_change+1, coverage_df.shape[1] - s[1]))
            else:
                # select the action with highest cummulative reward
                # as an action pick name of column that is has highest value for given state
                a = q_df.columns.values[np.argmax(q_df.loc[s_index])]

            # print('i:{} s:{} a:{} -max_pitch_change:{} -s[1]{} max_pitch_change+1:{} coverage_df.shape[1] - s[1]:{} rand.from:{} rand_to:{}'.format(i,s_index,a,-max_pitch_change,-s[1],max_pitch_change+1,coverage_df.shape[1] - s[1],max(-max_pitch_change, -s[1]),min(max_pitch_change+1, coverage_df.shape[1] - s[1])))
            new_s, r, done, _ = env.step(a)
            new_reward = r + lr*(y*np.max(q_df.loc[(new_s[0], new_s[1]), :]) - q_df.loc[s_index, a])
            if np.isnan(new_reward):
                q_df.loc[s_index, a] = r
            else:
                q_df.loc[s_index, a] += new_reward

            s = new_s
        print("Episode {}".format(i))
    # plot coverage
    xx, yy = np.meshgrid(env.data.index.values, env.data.columns.values)
    fig = plt.figure()
    ax = fig.add_subplot(121, projection='3d')
    ax.title.set_text('coverage')
    # ax.set_xlabel('x')
    # ax.set_ylabel('y')
    ax.set_xlabel('theta')
    ax.set_ylabel('pitch')
    ax.azim = -150
    ax.elev = 88
    ax.plot_surface(xx, yy, np.transpose(coverage_df), rstride=1, cstride=1, cmap='viridis', edgecolor='none')

    ax = fig.add_subplot(122, projection='3d')
    ax.title.set_text('ct')
    # ax.set_xlabel('x')
    # ax.set_ylabel('y')
    ax.set_xlabel('theta')
    ax.set_ylabel('pitch')
    ax.azim = -150
    ax.elev = 88
    ax.plot_surface(xx, yy, np.transpose(env.data), rstride=1, cstride=1, cmap='viridis', edgecolor='none')

    if display_plot:
        plt.show()
    if save_file_name:
        plt.savefig(save_file_name, bbox_inches='tight')

    return q_df, coverage_df


if __name__ == '__main__':
    airfoil_dir = '/home/aa/vawt_env/learn/AeroDyn polars/naca0018_360'
    # vb.VawtBlade(blade chord, airfoil_dir, rotor_radius)
    blade = vb.VawtBlade(0.2, airfoil_dir, 1)
    wind_direction = 0
    wind_speed = 3
    rotor_speed = 3
    theta_resolution = 5
    pitch_resolution = 3
    # env = VawtRLEnvironment(blade, wind_direction, wind_speed, rotor_speed)
    env = VawtRLEnvironment(blade, wind_direction, wind_speed, rotor_speed, theta_resolution, pitch_resolution, steps=10000)
    # table = naive_sum_reward_agent(env)
    # q_table = q_learning_with_table(env)
    # q_df, coverage_df = eps_greedy_q_learning_with_table(env, 1, save_file_name='foo.png')
    q_df, coverage_df = eps_greedy_q_learning_with_table(env, 5, display_plot=True)
    # q_df, coverage_df = eps_greedy_q_learning_with_table(env, 20, display_plot=True)
    # make one go of the game
    # run_game(q_df, env)
    # save coverage table
    # coverage_df.to_csv("eps_greedy_q_learning_old.csv")
    # save qdf dataframe
    file_name = "q_df.csv"
    q_df.to_csv(file_name)
    # print(table)
    # started 9:38 taking 100000 steps stopped about 9:41 - 3 minutes long