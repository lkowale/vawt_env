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


class FourierSeriesRegression:

    def __init__(self, data_dir, degree):
        # get directory
        self.data_dir = data_dir
        self.degree = degree
        x, y = variables('x, y')
        w, = parameters('w')
        self.model_dict = {y: self.fourier_series(x, f=w, n=self.degree)}

    def regress(self, save_plot=False):
        # for each tsr in given directory
        tsr_list = [f[-15:-12] for f in glob.glob(self.data_dir + "*_q_table.csv")]
        # for each tsr on list
        fsr_params = []
        for tsr in tsr_list:
            # load optimal path csv
            op_file = self.data_dir + "tsr{}".format(tsr) + "_op.csv"
            op_df = pd.read_csv(op_file, index_col=0)
            # load coverage table csv
            coverage_file = self.data_dir + "tsr{}".format(tsr) + "_coverage.csv"
            coverage_df = pd.read_csv(coverage_file, index_col=0)
            # translate indexes of optimal path to radians using cover_table
            # TODO it should be done with recreating environment and with env.data
            op_df['theta'] = coverage_df.index.values[op_df['theta']]
            op_df['pitch'] = coverage_df.columns.values[op_df['pitch']]
            op_df['theta'] = op_df['theta'].astype(float)
            op_df['pitch'] = op_df['pitch'].astype(float)
            # fourier regress
            xdata = op_df['theta']
            ydata = op_df['pitch']
            fit = Fit(self.model_dict, x=xdata, y=ydata)
            fit_result = fit.execute()
            # add fourier params to params list
            fsr_params.append(fit_result.params)
            if save_plot:
                # plot optimal path points
                ax = op_df.plot.scatter(x='theta', y='pitch')
                # plot regression function
                xdata = np.linspace(-np.pi, np.pi)
                f_model = fit.model(x=xdata, **fit_result.params)
                ax.plot(xdata, f_model.y)
                save_file_name = self.data_dir + "tsr{}".format(tsr) + '_fsr{}.png'.format(self.degree)
                plt.savefig(save_file_name, bbox_inches='tight')

        # save fourier params
        fsr_df = pd.DataFrame(fsr_params, index=tsr_list)
        file_name = self.data_dir + "_fsr{}.csv".format(self.degree)
        fsr_df.to_csv(file_name)


    def fourier_series(self, x, f, n=0):
        """
        Returns a symbolic fourier series of order `n`.

        :param n: Order of the fourier series.
        :param x: Independent variable
        :param f: Frequency of the fourier series
        """
        # Make the parameter objects for all the terms
        a0, *cos_a = parameters(','.join(['a{}'.format(i) for i in range(0, n + 1)]))
        sin_b = parameters(','.join(['b{}'.format(i) for i in range(1, n + 1)]))
        # Construct the series
        series = a0 + sum(ai * cos(i * f * x) + bi * sin(i * f * x)
                         for i, (ai, bi) in enumerate(zip(cos_a, sin_b), start=1))
        return series


if __name__ == '__main__':

    start_time = time.time()
    fsr = FourierSeriesRegression('/vawt/physical_model/pitch_optimizer/exps/naca0018_RL_1/', 3)
    fsr.regress(save_plot=True)
    exec_time = time.time() - start_time
    print("Execution time {:2.2f} minutes ---".format(exec_time/60))
