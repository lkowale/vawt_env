import learn.airfoil_dynamics.ct_plot.vawt_blade as vb
import learn.RL.rl1_qtables as rl1
import pandas as pd
from symfit import parameters, variables, sin, cos, Fit
import numpy as np
import collections


def fourier_series(x, f, n=0):
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


class PitchOptimizer:

    def __init__(self, blade):
        self.blade = blade

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
            fourier_params.append(collections.OrderedDict([('tsr', tsr)]).update(self.get_fourier_params(tsr)))
        # get dataframe of optimal pitch fourier params
        # fourier_params_df = pd.DataFrame(fourier_params, index_col=0)
        # https://stackoverflow.com/questions/44365209/generate-a-pandas-dataframe-from-ordereddict
        fourier_params_df = pd.DataFrame(fourier_params)
        # save dataframe to csv
        file_name = blade.airfoil_dir.split('/')[-1] + "_fourier_params.csv"
        fourier_params_df.to_csv(file_name)

    def get_fourier_params(self, tsr, coverage_treshold):
        # get RL occupation/coverage dataframe
        wind_direction = 0
        wind_speed = 1
        rotor_speed = wind_speed * tsr
        _, coverage_df = rl1.eps_greedy_q_learning_with_table(self.rl_environment, wind_direction, wind_speed, rotor_speed, 20)
        # save the coverage dataframe
        file_name = blade.airfoil_dir.split('/')[-1] + "_" + tsr + "_RLcover.csv"
        coverage_df.to_csv(file_name)
        print("Saved coverage dataframe to " + file_name)
        # use treshold on coverage dataframe to pull up only mostly used actions/pitches
        thetas = []
        pitches = []
        for theta in coverage_df.index.values:
            for pitch in coverage_df.columns.values:
                if coverage_df.loc[theta, pitch] > coverage_treshold:
                    thetas.append(theta)
                    pitches.append(pitch)
        # make fourier series regression
        x, y = variables('x, y')
        w, = parameters('w')
        model_dict = {y: fourier_series(x, f=w, n=3)}
        xdata = np.array(thetas)
        ydata = np.array(pitches)
        fit = Fit(model_dict, x=xdata, y=ydata)
        fit_result = fit.execute()
        return fit_result.params


if __name__ == '__main__':
    airfoil_dir = '/home/aa/vawt_env/learn/AeroDyn polars/naca0018_360'
    blade_shaft_dist = 1
    blade_chord_length = 0.2
    blade = vb.VawtBlade(blade_chord_length, airfoil_dir, blade_shaft_dist)
