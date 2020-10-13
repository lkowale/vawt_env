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
from vawt.physical_model.vawt_parameters import *


class PitchOptimizer:

    def __init__(self, blade):
        self.blade = blade
        self.coverage_treshold = 300
        # for given blade create a function of (tsr,theta) that gives optimum pitch position
        # for given amount/set of TSR's range(0.1,6.0, step=0.1)
        #       for each TSR create ct=f(pitch,theta) than RL the optimum pitch set in rl1_qtables
        #       than use regression to fourier series 3 grade
        # than on set of functions get from regression
        #     use interpolate
        #     or use another regression
        #       to get a function pitch = g(theta) that is continuous
        #              and sum of ct=f(g(theta),theta) for  theta in [-pi,pi] is the greatest


    def find_optimum_params(self):
        # for given blade get  params for fourier series and sve them as csv
        # get list of optimal pitch fourier params
        fourier_params = []
        for tsr in np.arange(0.1, 5.0, 0.3):
            # fourier_params.append([tsr, *self.get_fourier_params(tsr)])
            self.get_fourier_params(tsr)


    def get_fourier_params(self, tsr):
        # set folder and files base name
        airfoil_name = self.blade.airfoil_dir.split('/')[-1].split('_')[0]
        folder_name = airfoil_name + '_RLcover'
        file_base_name = "/tsr{:1.1f}".format(tsr)
        base_file_path = folder_name + file_base_name

        if not os.path.exists(folder_name):
            os.makedirs(folder_name)
        # set environment
        wind_direction = 0
        wind_speed = 1
        rotor_speed = wind_speed * tsr
        theta_resolution = 5
        pitch_resolution = 3
        rl_environment = rl1.VawtRLEnvironment(blade, wind_direction, wind_speed, rotor_speed, theta_resolution, pitch_resolution)
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
    airfoil_dir = '/home/aa/vawt_env/learn/AeroDyn polars/naca0018_360'
    blade_shaft_dist = 1
    blade_chord_length = 0.2
    blade = vb.VawtBlade(blade_chord_length, airfoil_dir, blade_shaft_dist)
    po = PitchOptimizer(blade)
    po.find_optimum_params()
    exec_time = time.time() - start_time
    print("Execution time {:2.2f} minutes ---".format(exec_time/60))
