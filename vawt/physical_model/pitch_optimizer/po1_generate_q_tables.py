import learn.airfoil_dynamics.ct_plot.vawt_blade as vb
import learn.RL.rl1_qtables_1 as rl1
import learn.RL.rl3_run_game as rl3
import pandas as pd
from symfit import parameters, variables, sin, cos, Fit
import numpy as np
import collections
import matplotlib.pyplot as plt
import os
import time
from vawt.physical_model.vawt_parameters import *


class PitchOptimizer:

    def __init__(self, env_params):
        self.env_params = env_params
        self.pitch_resolution = self.env_params['pitch_resolution']
        self.theta_resolution = self.env_params['theta_resolution']
        self.wind_direction = self.env_params['wind_direction']
        self.blade = vb.VawtBlade(self.env_params['blade_chord_length'], self.env_params['airfoil_dir'],
                                  self.env_params['blade_shaft_dist'])
        # for given blade create a function of (tsr,theta) that gives optimum pitch position
        # for given amount/set of TSR's range(0.1,6.0, step=0.1)
        #       for each TSR create ct=f(pitch,theta) than RL the optimum pitch set in rl1_qtables
        #       save q_table as csv DataFrame
        self.wind_speeds = self.env_params['wind_speeds']
        self.rotor_speeds = self.env_params['rotor_speeds']

    def gensave_q_tables(self):
        for wind_speed in self.wind_speeds:
            for rotor_speed in self.rotor_speeds:
                self.save_q_table_tsr(wind_speed, rotor_speed)

    def save_q_table_tsr(self, wind_speed, rotor_speed):
        # set folder and files base name
        airfoil_name = self.blade.airfoil_dir.split('/')[-1].split('_')[0]
        folder_name = 'exps/' + self.env_params['results_dir_prepend'] + airfoil_name + self.env_params['results_dir_append'] # TODO + data
        file_base_name = "w_r_{}_{}".format(wind_speed, rotor_speed)
        base_file_path = folder_name + file_base_name
        env_params_file = base_file_path + '_env_params.csv'
        if not os.path.exists(folder_name):
            os.makedirs(folder_name)
        # set environment
        rl_environment = rl1.VawtRLEnvironment(self.blade, self.wind_direction, wind_speed, rotor_speed,
                                               self.theta_resolution, self.pitch_resolution)
        # save environment
        rl1.save_environment(rl_environment, env_params_file)
        # get RL occupation/coverage dataframe and save plot of coverage + tangent coeef
        file_name = base_file_path + ".png"
        q_df, coverage_df = rl1.eps_greedy_q_learning_with_table(rl_environment, 20, save_file_name=file_name)
        # save the coverage dataframe
        file_name = base_file_path + "_coverage.csv"
        coverage_df.to_csv(file_name)
        # save q_table
        file_name = base_file_path + "_q_table.csv"
        q_df.to_csv(file_name)
        print("Saved coverage dataframe to " + file_name)


if __name__ == '__main__':

    start_time = time.time()
    # airfoil_dir = '/learn/AeroDyn polars/cp10_360'
    params = {
        'airfoil_dir': '/home/aa/vawt_env/learn/AeroDyn polars/naca0018_360',
        # 'airfoil_dir': '/home/aa/vawt_env/learn/AeroDyn polars/cp10_360',
        # 'results_dir_prepend': 'tsr_test/',
        'results_dir_prepend': '',
        'results_dir_append': '_m_5/',
        'blade_shaft_dist': 1,
        'blade_chord_length': 0.2,
        'pitch_resolution': 4,
        'theta_resolution': 5,
        'wind_speeds': np.arange(3, 13),
        'rotor_speeds': [0.001, 1, 5, 10, 20, 30, 40, 50, 60],
        'wind_direction': 0,
    }
    po = PitchOptimizer(params)
    po.gensave_q_tables()
    exec_time = time.time() - start_time
    print("Execution time {:2.2f} minutes ---".format(exec_time/60))
