import pandas as pd
from symfit import parameters, variables, sin, cos, Fit
import numpy as np
import collections
import matplotlib.pyplot as plt
import os
import time
import vawt.physical_model.vawt_parameters as params
import glob
from mpl_toolkits.mplot3d import Axes3D

class FsrParams:

    def __init__(self, data_dir, degree):
        # load params df from csv
        self.degree = degree
        self.data_dir = data_dir
        file_name = self.data_dir + "_fsr{}.csv".format(self.degree)
        fsr_df = pd.read_csv(file_name, index_col=0)
        fsr_df = fsr_df.sort_index(axis=0)
        # plot fourier params in function of tsr
        fig = plt.figure()
        ax = Axes3D(fig)
        ax.title.set_text('fsr')
        for i, parameter in enumerate(fsr_df.columns):
            ax.plot(np.full(fsr_df.index.size, i), fsr_df.index.values, fsr_df[parameter])
        plt.xticks(np.arange(fsr_df.columns.size), fsr_df.columns.values, rotation=90)
        plt.show()


if __name__ == '__main__':

    start_time = time.time()
    fsr = FsrParams('/vawt/physical_model/pitch_optimizer/exps/naca0018_RL_1/', 3)
    exec_time = time.time() - start_time
    print("Execution time {:2.2f} minutes ---".format(exec_time/60))