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

    def find_optimum_params(self, display_plot=False, save_plot=False):
        # get list of tsr's that has been used
        # file_list = [f for f in glob.glob(self.data_dir + "*_q_table.csv")]
        tsr_list = [f[-15:-12] for f in glob.glob(self.data_dir + "*_q_table.csv")]
        # for each tsr on list
        rewards = []
        for tsr in tsr_list:
            # recreate environment on base of params file
            params_file = self.data_dir + "tsr{}".format(tsr) + '_env_params.csv'
            env = rl1.load_environment(params_file)
            # load q_table
            q_tables_file = self.data_dir + "tsr{}".format(tsr) + "_q_table.csv"
            q_df = pd.read_csv(q_tables_file, index_col=[0, 1])
            q_df.columns = q_df.columns.map(int)
            # run a game for it
            rg = rl3.RunGame(q_df, env)
            total_reward, op_df = rg.run()
            rewards.append(total_reward)
            save_file_name = self.data_dir + "tsr{}".format(tsr) + '_op.csv'
            op_df.to_csv(save_file_name)
            op_df.plot.scatter(x='theta', y='pitch')
            if display_plot:
                plt.show()
            if save_plot:
                save_file_name = self.data_dir + "tsr{}".format(tsr) + '_op.png'
                plt.savefig(save_file_name, bbox_inches='tight')

        tsr_list = list(map(float, tsr_list))
        rewards_df = pd.DataFrame()
        rewards_df['tsr'] = tsr_list
        rewards_df['rewards'] = rewards
        save_file_name = self.data_dir + "rewards.csv"
        rewards_df.to_csv(save_file_name)

        rewards_df.plot.scatter(x='tsr', y='rewards')
        plt.savefig(self.data_dir + '_rewrads.png', bbox_inches='tight')
        # plt.plot(tsr_list, rewards, ls=':')
        # plt.xlabel('x')
        # plt.ylabel('y')
        # plt.show()
        # regres with fourier
        # save params for all tsr as a csv


if __name__ == '__main__':

    start_time = time.time()
    op = OptimalPath('/home/aa/vawt_env/vawt/physical_model/exps/naca0018_RL_1/')
    # op = OptimalPath('/home/aa/vawt_env/vawt/physical_model/exps/cp10_RL_1/')
    op.find_optimum_params(save_plot=True)
    exec_time = time.time() - start_time
    print("Execution time {:2.2f} minutes ---".format(exec_time/60))
