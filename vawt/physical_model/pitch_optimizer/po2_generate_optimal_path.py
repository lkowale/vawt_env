import learn.airfoil_dynamics.ct_plot.vawt_blade as vb
import learn.RL.rl1_qtables_2 as rl1
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
        # for each tsr on list
        # list of experiments available in directory
        # make dictionary to construct datframe wind_speedxtsr
        rewards = []
        for q_tables_file in glob.glob(self.data_dir + "*_q_table.csv"):
            # recreate environment on base of params file
            params_file = q_tables_file[:-12] + '_env_params.csv'
            env = rl1.load_environment(params_file)
            # load q_table
            q_df = pd.read_csv(q_tables_file, index_col=[0, 1])
            q_df.columns = q_df.columns.map(int)
            # run a game for it
            rg = rl3.RunGame(q_df, env)
            total_reward, op_df = rg.run()
            rewards.append([env.wind_speed, env.tsr, total_reward])

            save_file_name = q_tables_file[:-12] + '_op.csv'
            op_df.to_csv(save_file_name)
            op_df.plot.scatter(x='theta', y='pitch')
            if display_plot:
                plt.show()
            if save_plot:
                save_file_name = q_tables_file[:-12] + '_op.png'
                plt.savefig(save_file_name, bbox_inches='tight')

        rewards_df = pd.DataFrame(rewards, columns=['wind_speed', 'tsr', 'reward'])
        # rewards_df = rewards_df.set_index(['wind_speed', 'tsr']).unstack(level=0)
        rewards_df = rewards_df.pivot(index='wind_speed', columns='tsr', values='reward')
        # plot coverage
        xx, yy = np.meshgrid(rewards_df.index.values, rewards_df.columns.values)
        fig = plt.figure()
        ax = fig.add_subplot(111, projection='3d')
        ax.title.set_text('rewards')
        ax.set_ylabel('tsr')
        ax.set_xlabel('wind_speed')
        ax.azim = -150
        ax.elev = 88
        ax.plot_surface(xx, yy, np.transpose(rewards_df.fillna(0)), rstride=1, cstride=1, cmap='viridis', edgecolor='none')

        plt.show()



if __name__ == '__main__':

    start_time = time.time()
    op = OptimalPath('/home/aa/vawt_env/vawt/physical_model/pitch_optimizer/exps/naca0018_10/')
    # op = OptimalPath('/home/aa/vawt_env/vawt/physical_model/pitch_optimizer/exps/ws5tsr1/')

    # op = OptimalPath('/home/aa/vawt_env/vawt/physical_model/pitch_optimizer/exps/cp10_RL_5/')
    op.find_optimum_params(save_plot=True)
    exec_time = time.time() - start_time
    print("Execution time {:2.2f} minutes ---".format(exec_time/60))
