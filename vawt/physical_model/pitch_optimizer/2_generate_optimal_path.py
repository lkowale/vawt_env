import learn.airfoil_dynamics.ct_plot.vawt_blade as vb
import learn.RL.rl1_qtables as rl1
import learn.RL.rl3_run_game as rl3
import pandas as pd
from symfit import parameters, variables, sin, cos, Fit
import numpy as np
import collections
import matplotlib.pyplot as plt
import os
import time
import vawt.physical_model.vawt_parameters as params
import glob


class OptimalPath:

    def __init__(self, data_dir):
        # get directory
        self.data_dir = data_dir

    def find_optimum_params(self):
        # get files from directory
        files_list = [f for f in glob.glob(self.data_dir + "*_q_table.csv")]
        blade = vb.VawtBlade(0.2, airfoil_dir, 1)
        wind_direction = 0
        wind_speed = 3
        rotor_speed = 3
        theta_resolution = 5
        pitch_resolution = 3
        # for each file on list
        for file in files_list:
            # load q_table
            q_df = pd.read_csv(file, index_col=[0, 1])
            q_df.columns = q_df.columns.map(int)
            # run a game for it

            # env = VawtRLEnvironment(blade, wind_direction, wind_speed, rotor_speed)
            env = rl1.VawtRLEnvironment(blade, wind_direction, wind_speed, rotor_speed, theta_resolution,
                                        pitch_resolution, steps=10000)

            rg = RunGame(q_df, env)
            total_reward, plot_df = rg.run()
            # regres with fourier
            # add to fourier params + total reward
        # save params for all tsr as a csv


if __name__ == '__main__':

    start_time = time.time()
    # airfoil_dir = '/learn/AeroDyn polars/cp10_360'
    airfoil_dir = '/home/aa/vawt_env/learn/AeroDyn polars/naca0018_360'
    blade = vb.VawtBlade(params.blade_chord_length, airfoil_dir, params.blade_shaft_dist)
    op = OptimalPath('/home/aa/vawt_env/vawt/physical_model/naca0018_RLcover')
    op.find_optimum_params()
    exec_time = time.time() - start_time
    print("Execution time {:2.2f} minutes ---".format(exec_time/60))
