from scipy import interpolate
import numpy as np
import matplotlib.pyplot as plt
from matplotlib import cm
import pandas as pd
from scipy import interpolate
from learn.airfoil_model.data_load import *
import time
import vawt.physical_model.vawt_parameters as params
import glob
from matplotlib.ticker import LinearLocator, FormatStrFormatter
from mpl_toolkits.mplot3d import Axes3D
import learn.RL.rl1_qtables_2 as rl1
from scipy.interpolate import LinearNDInterpolator


class OptimalPathInterpolate:

    def __init__(self, data_dir):
        # get directory
        self.data_dir = data_dir
        self.data = self.get_op_grid()
        self.interpolate = self.get_interpolation(self.data)

    def get_op_grid(self):
        # for every optimal path in folder
        op_df_list = []
        for op_file in glob.glob(self.data_dir + "*_op.csv"):
            # load op form csv
            op_df = pd.read_csv(op_file, index_col=0)
            # recreate environment on base of params file
            params_file = op_file[:-7] + '_env_params.csv'
            # TODO do not load the whole environment - it takes lot of time, load just parameters instead
            params = pd.read_csv(params_file)
            params = params.iloc[0]
            # translate indexes of optimal path to radians using cover_table
            op_df['theta'] = [rl1.get_theta_range(params['theta_resolution'], 180)[i] for i in op_df['theta']]
            op_df['pitch'] = [rl1.get_pitch_range(params['pitch_resolution'], params['pitch_change_width'])[i] for i in op_df['pitch']]
            # add columns for wind_speed and tsr
            op_df['wind_speed'] = params['wind_speed']
            op_df['tsr'] = params['tsr']
            op_df_list.append(op_df)

        pitch_df = pd.concat(op_df_list, ignore_index=True)
        return pitch_df

    def plot_grid(self, wind_speed):

        # interpolation
        # tsr
        x = np.linspace(0.1, 6, 50)
        # theta
        y = np.linspace(-np.pi, np.pi, 72)
        xx, yy = np.meshgrid(x, y)
        op_i = []
        for _x in x:
            op_i.append([float(self.get_optimal_pitch(wind_speed, _x, _y)) for _y in y])
        op_i_df = pd.DataFrame(op_i)
        fig = plt.figure()
        ax = fig.add_subplot(111, projection='3d')
        ax.plot_surface(xx, yy, op_i_df.T, rstride=1, cstride=1, cmap=cm.coolwarm,
                               linewidth=0, antialiased=False)
        ax.title.set_text('maximum power pitch')
        ax.set_xlabel('tsr')
        ax.set_ylabel('theta')
        ax.set_zlabel('optimal pitch interpolation wind_speed: {}'.format(wind_speed))
        plt.show()

    def get_interpolation(self, data):
        points = data[['wind_speed', 'tsr', 'theta']]
        values = data['pitch']
        f = LinearNDInterpolator(points, values)
        return f

    def get_optimal_pitch(self, wind_speed, tsr, theta):
        return float(self.interpolate((wind_speed, tsr, theta)))


if __name__ == '__main__':

    start_time = time.time()
    opi = OptimalPathInterpolate('/home/aa/vawt_env/vawt/physical_model/pitch_optimizer/exps/naca0018_m_7/')
    # opi.plot_grid(3.5)
    exec_time = time.time() - start_time
    print("Execution time {:2.2f} minutes ---".format(exec_time/60))
    start_time = time.time()
    a = opi.get_optimal_pitch(3.1, 3.1, 0.1)
    exec_time = time.time() - start_time
    print("Execution time {} s".format(exec_time))