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


class OptimalPathInterpolate:

    def __init__(self, data_dir):
        # get directory
        self.data_dir = data_dir
        self.data = self.get_op_grid()
        self.interpolate = self.get_interpolation(self.data)

    def get_op_grid(self):
        # get list of tsr's that has been used
        tsr_list = [f[-15:-12] for f in glob.glob(self.data_dir + "*_q_table.csv")]
        # sort it
        tsr_list = sorted(tsr_list, key=float)
        # load coverage table csv
        coverage_file = self.data_dir + "tsr{}".format(tsr_list[0]) + "_coverage.csv"
        coverage_df = pd.read_csv(coverage_file, index_col=0)
        pitch_df = pd.DataFrame()
        for tsr in tsr_list:
            # load optimal path csv
            op_file = self.data_dir + "tsr{}".format(tsr) + "_op.csv"
            op_df = pd.read_csv(op_file, index_col=0)
            # translate indexes of optimal path to radians using cover_table
            # TODO it should be done with recreating environment and with env.data
            op_df['theta'] = coverage_df.index.values[op_df['theta']]
            op_df['pitch'] = coverage_df.columns.values[op_df['pitch']]
            op_df['theta'] = op_df['theta'].astype(float)
            op_df['pitch'] = op_df['pitch'].astype(float)
            pitch_df[float(tsr)] = op_df['pitch']

        pitch_df.index = coverage_df.index
        # done; pitch df now is pitch = f(tsr, theta)
        return pitch_df

    def plot_grid(self, pitch_df):
        # tsr
        x = pitch_df.columns.values
        # theta
        y = pitch_df.index.values
        xx, yy = np.meshgrid(x, y)


        # plot using plot_surface
        z = pitch_df.values
        fig = plt.figure()
        ax = fig.add_subplot(121, projection='3d')
        ax.plot_surface(xx, yy, z, rstride=1, cstride=1, cmap=cm.coolwarm,
                               linewidth=0, antialiased=False)
        ax.title.set_text('maximum power pitch')
        ax.set_xlabel('tsr')
        ax.set_ylabel('theta')
        ax.set_zlabel('optimal pitch - original')

        # interpolation
        # tsr
        x = np.linspace(0.0, 5, 50)
        # theta
        y = np.linspace(-np.pi, np.pi, 150)
        xx, yy = np.meshgrid(x, y)
        z = self.get_optimal_pitch(x, y)
        ax = fig.add_subplot(122, projection='3d')
        ax.plot_surface(xx, yy, z, rstride=1, cstride=1, cmap=cm.coolwarm,
                               linewidth=0, antialiased=False)
        ax.title.set_text('maximum power pitch')
        ax.set_xlabel('tsr')
        ax.set_ylabel('theta')
        ax.set_zlabel('optimal pitch - interpolation')


        # # Plot using `.trisurf()`:
        # size = y.size * len(x)
        # x = xx.reshape(size)
        # y = yy.reshape(size)
        # z = pitch_df.values.reshape(size)
        #
        # fig = plt.figure()
        # ax = fig.add_subplot(111, projection='3d')
        # ax.title.set_text('maximum power pitch')
        # ax.plot_trisurf(x, y, z, cmap=cm.jet, linewidth=0.5)
        # ax.set_xlabel('tsr')
        # ax.set_ylabel('theta')
        # ax.set_zlabel('optimal pitch')


        plt.show()

    def get_interpolation(self, data):
        x = data.columns.values
        y = data.index.values
        z = data.values
        f = interpolate.interp2d(x, y, z, kind='cubic')
        return f

    def get_optimal_pitch(self, tsr, theta):
        return self.interpolate(tsr, theta)


if __name__ == '__main__':

    start_time = time.time()
    opi = OptimalPathInterpolate('/home/aa/vawt_env/vawt/physical_model/pitch_optimizer/exps/naca0018_RL_1/')
    opi.plot_grid(opi.get_op_grid())
    exec_time = time.time() - start_time
    print("Execution time {:2.2f} minutes ---".format(exec_time/60))

